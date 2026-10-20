#include <math.h>
#include <stdint.h>
#include "Compass_Module.h"

namespace {
  struct Vec3 {
    float x, y, z;
    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  };

  struct AxisMap {
    uint8_t src[3];
    int8_t sign[3];

    AxisMap(uint8_t s0, uint8_t s1, uint8_t s2,
            int8_t sg0, int8_t sg1, int8_t sg2) {
      src[0] = s0; src[1] = s1; src[2] = s2;
      sign[0] = sg0; sign[1] = sg1; sign[2] = sg2;
    }
  };

  static Vec3 vAdd(const Vec3 &a, const Vec3 &b) {
    return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
  }

  static Vec3 vSub(const Vec3 &a, const Vec3 &b) {
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
  }

  static Vec3 vScale(const Vec3 &a, float s) {
    return Vec3(a.x * s, a.y * s, a.z * s);
  }

  static float vDot(const Vec3 &a, const Vec3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  static Vec3 vCross(const Vec3 &a, const Vec3 &b) {
    return Vec3(
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x
    );
  }

  static float vNorm(const Vec3 &a) {
    return sqrtf(vDot(a, a));
  }

  static Vec3 vNormalize(const Vec3 &a) {
    const float n = vNorm(a);
    if (!isfinite(n) || n < 1.0e-9f) return Vec3();
    return vScale(a, 1.0f / n);
  }

  static float clampfLocal(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
  }

  static float wrap360(float degrees) {
    if (!isfinite(degrees)) return NAN;
    degrees = fmodf(degrees, 360.0f);
    if (degrees < 0.0f) degrees += 360.0f;
    return degrees;
  }

  static float wrap180(float degrees) {
    degrees = wrap360(degrees);
    if (!isfinite(degrees)) return NAN;
    if (degrees > 180.0f) degrees -= 360.0f;
    return degrees;
  }

  static Vec3 applyAxisMap(const Vec3 &input, const AxisMap &map) {
    const float values[3] = {input.x, input.y, input.z};
    return Vec3(
      values[map.src[0]] * map.sign[0],
      values[map.src[1]] * map.sign[1],
      values[map.src[2]] * map.sign[2]
    );
  }

  /*
   * Body frame follows the HMC5883L mounting specified for this device:
   *   +X = up, +Y = wearer's left, -Z = forward.
   *
   * These identity maps assume ADXL345, ITG3200 and HMC5883L axes are
   * physically aligned. If the boards/chips are rotated relative to one
   * another, change each map so all three measurements use the same frame.
   */
  static const AxisMap ACCEL_MAP(0, 1, 2, +1, +1, +1);
  static const AxisMap GYRO_MAP (0, 1, 2, +1, +1, +1);
  static const AxisMap MAG_MAP  (0, 1, 2, +1, +1, +1);

  static const Vec3 FORWARD_BODY(0.0f, 0.0f, -1.0f);

  /*
   * HMC5883L calibration. Offsets are in microtesla because the Adafruit
   * Unified Sensor driver reports event.magnetic in uT. Keep defaults neutral
   * until calibration measurements have been collected on the final assembly.
   *
   * Hard-iron: subtract the three offsets.
   * Soft-iron: apply per-axis gains after offset subtraction. This diagonal
   * correction improves scale mismatch; a full ellipsoid fit is better when
   * the enclosure or nearby metal causes cross-axis distortion.
   */
  static const Vec3 MAG_HARD_IRON_OFFSET_UT(0.0f, 0.0f, 0.0f);
  static const Vec3 MAG_SOFT_IRON_SCALE(1.0f, 1.0f, 1.0f);

  /*
   * East-positive magnetic declination, in degrees. Set this for the deployment
   * location if XY is intended to align with true north/east. Zero means the
   * output remains relative to magnetic north.
   */
  static const float MAGNETIC_DECLINATION_DEG = 0.0f;

  static const float DEG_TO_RAD = 0.01745329251994329577f;
  static const float RAD_TO_DEG = 57.29577951308232f;

  static const float MAG_FIELD_MIN_UT = 10.0f;
  static const float MAG_FIELD_MAX_UT = 100.0f;
  static const float MIN_HORIZONTAL_FIELD_RATIO = 0.10f;
  static const float MIN_FORWARD_HORIZONTAL = 0.20f;
  static const uint32_t HEADING_VALID_TIMEOUT_US = 300000UL;

  /*
   * Gyro-propagated gravity direction with accelerometer correction.
   * upBody is the upward direction expressed in the sensor/body frame. The
   * accelerometer correction is down-weighted when |a| is far from 1 g, since
   * walking/impact acceleration must not be mistaken for gravity.
   */
  class GravityEstimator {
  public:
    bool initialized;
    Vec3 upBody;

    GravityEstimator() : initialized(false), upBody(1.0f, 0.0f, 0.0f) {}

    void resetFromAccel(const Vec3 &accelG) {
      const float magnitude = vNorm(accelG);
      if (!isfinite(magnitude) || magnitude < 0.2f) return;
      upBody = vNormalize(accelG);
      initialized = (vNorm(upBody) > 0.5f);
    }

    void update(const Vec3 &accelG, const Vec3 &gyroDps, float dt) {
      if (!initialized) {
        resetFromAccel(accelG);
        if (!initialized) return;
      }

      if (isfinite(gyroDps.x) && isfinite(gyroDps.y) &&
          isfinite(gyroDps.z) && dt > 0.0f) {
        const Vec3 omega = vScale(gyroDps, DEG_TO_RAD);

        // In body coordinates: d(up)/dt = up x angular_velocity.
        const Vec3 derivative = vCross(upBody, omega);
        const Vec3 predicted = vNormalize(
          vAdd(upBody, vScale(derivative, dt))
        );
        if (vNorm(predicted) > 0.5f) upBody = predicted;
      }

      const float accelMagnitude = vNorm(accelG);
      if (!isfinite(accelMagnitude) ||
          accelMagnitude < 0.55f || accelMagnitude > 1.45f) {
        return;
      }

      const Vec3 measuredUp = vNormalize(accelG);
      if (vNorm(measuredUp) < 0.5f) return;

      const float deviation = fabsf(accelMagnitude - 1.0f);
      float confidence = 1.0f - clampfLocal(deviation / 0.30f, 0.0f, 1.0f);
      confidence *= confidence;

      // Complementary correction: slow enough to reject gait acceleration.
      const float alpha = (1.0f - expf(-dt / 0.55f)) * confidence;
      const Vec3 corrected = vNormalize(
        vAdd(vScale(upBody, 1.0f - alpha), vScale(measuredUp, alpha))
      );
      if (vNorm(corrected) > 0.5f) upBody = corrected;
    }
  };

  static GravityEstimator gravityEstimator;

  /*
   * General 3D tilt compensation:
   * 1. Estimate vertical (up) in the sensor frame.
   * 2. Remove the vertical component from the magnetic field.
   * 3. Project the device's -Z forward axis onto the horizontal plane.
   * 4. Compute the signed clockwise angle from magnetic north to forward.
   *
   * This vector form works with the non-standard mounting (+X up, +Y left,
   * -Z forward) without assuming that gravity is always on one sensor axis.
   */
  static bool computeRawHeading(const Vec3 &magRaw, float &outHeadingDeg) {
    if (!gravityEstimator.initialized) return false;

    const Vec3 mag = Vec3(
      (magRaw.x - MAG_HARD_IRON_OFFSET_UT.x) * MAG_SOFT_IRON_SCALE.x,
      (magRaw.y - MAG_HARD_IRON_OFFSET_UT.y) * MAG_SOFT_IRON_SCALE.y,
      (magRaw.z - MAG_HARD_IRON_OFFSET_UT.z) * MAG_SOFT_IRON_SCALE.z
    );

    const float magMagnitude = vNorm(mag);
    if (!isfinite(magMagnitude) ||
        magMagnitude < MAG_FIELD_MIN_UT ||
        magMagnitude > MAG_FIELD_MAX_UT) {
      return false;
    }

    const Vec3 up = vNormalize(gravityEstimator.upBody);
    if (vNorm(up) < 0.5f) return false;

    Vec3 northHorizontal = vSub(mag, vScale(up, vDot(mag, up)));
    const float horizontalMagnitude = vNorm(northHorizontal);
    if (!isfinite(horizontalMagnitude) ||
        horizontalMagnitude < 1.0e-4f ||
        horizontalMagnitude / magMagnitude < MIN_HORIZONTAL_FIELD_RATIO) {
      return false;
    }
    northHorizontal = vScale(northHorizontal, 1.0f / horizontalMagnitude);

    Vec3 forwardHorizontal = vSub(
      FORWARD_BODY,
      vScale(up, vDot(FORWARD_BODY, up))
    );
    const float forwardMagnitude = vNorm(forwardHorizontal);
    if (!isfinite(forwardMagnitude) ||
        forwardMagnitude < MIN_FORWARD_HORIZONTAL) {
      // Forward axis is almost vertical, so a 2D bearing is ill-defined.
      return false;
    }
    forwardHorizontal = vScale(forwardHorizontal, 1.0f / forwardMagnitude);

    const float sinAngle = vDot(up, vCross(forwardHorizontal, northHorizontal));
    const float cosAngle = vDot(northHorizontal, forwardHorizontal);
    const float magneticHeading = atan2f(sinAngle, cosAngle) * RAD_TO_DEG;

    outHeadingDeg = wrap360(magneticHeading + MAGNETIC_DECLINATION_DEG);
    return isfinite(outHeadingDeg);
  }

  static bool headingFilterInitialized = false;
  static float headingFilteredDeg = 0.0f;

  static float updateHeadingFilter(float rawHeadingDeg) {
    if (!headingFilterInitialized) {
      headingFilteredDeg = wrap360(rawHeadingDeg);
      headingFilterInitialized = true;
      return headingFilteredDeg;
    }

    // Circular low-pass filter: take the shortest path across 0/360 degrees.
    const float error = wrap180(rawHeadingDeg - headingFilteredDeg);
    if (!isfinite(error)) return headingFilteredDeg;

    const float dynamic = clampfLocal(fabsf(error) / 45.0f, 0.0f, 1.0f);
    const float alpha = 0.06f + 0.22f * dynamic;
    headingFilteredDeg = wrap360(headingFilteredDeg + alpha * error);
    return headingFilteredDeg;
  }

  static String headingToDirection(float headingDeg) {
    static const char *directions[8] = {
      "N", "NE", "E", "SE", "S", "SW", "W", "NW"
    };
    if (!isfinite(headingDeg)) return "None";
    const int index = ((int)((wrap360(headingDeg) + 22.5f) / 45.0f)) & 7;
    return String(directions[index]);
  }

  static bool haveValidHeading = false;
  static uint32_t lastValidHeadingUs = 0;
  static uint32_t lastUpdateUs = 0;
  static float currentCompassValue = 0.0f;
  static String currentCompassHeading = "N";
} // namespace

void setupCompass() {
  gravityEstimator = GravityEstimator();
  headingFilterInitialized = false;
  headingFilteredDeg = 0.0f;
  haveValidHeading = false;
  lastValidHeadingUs = 0;
  lastUpdateUs = micros();
  currentCompassValue = 0.0f;
  currentCompassHeading = "N";
}

void processCompassData(float accX, float accY, float accZ,
                        float gyrX, float gyrY, float gyrZ,
                        float magX, float magY, float magZ) {
  const Vec3 accelBody = applyAxisMap(Vec3(accX, accY, accZ), ACCEL_MAP);
  const Vec3 gyroBody = applyAxisMap(Vec3(gyrX, gyrY, gyrZ), GYRO_MAP);
  const Vec3 magBody = applyAxisMap(Vec3(magX, magY, magZ), MAG_MAP);

  const uint32_t nowUs = micros();
  float dt = (uint32_t)(nowUs - lastUpdateUs) * 1.0e-6f;
  lastUpdateUs = nowUs;
  if (!isfinite(dt) || dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  gravityEstimator.update(accelBody, gyroBody, dt);

  float rawHeadingDeg = 0.0f;
  if (computeRawHeading(magBody, rawHeadingDeg)) {
    currentCompassValue = updateHeadingFilter(rawHeadingDeg);
    currentCompassHeading = headingToDirection(currentCompassValue);
    haveValidHeading = true;
    lastValidHeadingUs = nowUs;
  }
}

float getCompassValue() {
  return currentCompassValue;
}

String getCompassHeading() {
  return currentCompassHeading;
}

bool isCompassReady() {
  if (!gravityEstimator.initialized || !haveValidHeading) return false;
  return (uint32_t)(micros() - lastValidHeadingUs) <= HEADING_VALID_TIMEOUT_US;
}
