#include <math.h>
#include <string.h>
#include "Compass_Module.h"

namespace {
  // Kiểu dữ liệu và phép toán vector 3D
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
    if (n < 1.0e-9f) return Vec3(0.0f, 0.0f, 0.0f);
    return vScale(a, 1.0f / n);
  }

  static float clampfLocal(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }

  static float wrap360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
  }

  static float wrap180(float deg) {
    deg = wrap360(deg);
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
  }

  static Vec3 applyAxisMap(const Vec3 &in, const AxisMap &m) {
    const float a[3] = {in.x, in.y, in.z};
    return Vec3(
      a[m.src[0]] * m.sign[0],
      a[m.src[1]] * m.sign[1],
      a[m.src[2]] * m.sign[2]
    );
  }

  // Quy ước hệ trục tọa độ
  static const AxisMap ACCEL_MAP(0, 1, 2, +1, +1, +1);
  static const AxisMap GYRO_MAP (0, 1, 2, +1, +1, +1);
  static const AxisMap MAG_MAP  (0, 1, 2, +1, +1, +1);

  // Quy ước hướng đeo trên cơ thể: Hướng tiến là -Z
  static const Vec3 FORWARD_BODY(0.0f, 0.0f, -1.0f);

  // Bộ ước lượng vector trọng lực
  class GravityEstimator {
  public:
    bool initialized;
    Vec3 upBody;

    GravityEstimator()
        : initialized(false),
          upBody(0.0f, 0.0f, 1.0f) {}

    void resetFromAccel(const Vec3 &accelG) {
      if (vNorm(accelG) < 0.2f) return;
      upBody = vNormalize(accelG);
      initialized = true;
    }

    void update(const Vec3 &accelG, const Vec3 &gyroDps, float dt) {
      if (!initialized) {
        resetFromAccel(accelG);
        if (!initialized) return;
      }

      const float DEG2RAD = 0.01745329251994329577f;
      const Vec3 omega = vScale(gyroDps, DEG2RAD);

      // Tích phân cập nhật hướng: du/dt = up x omega
      const Vec3 du = vCross(upBody, omega);
      upBody = vNormalize(vAdd(upBody, vScale(du, dt)));

      // Hiệu chỉnh bằng gia tốc kế khi độ lớn xấp xỉ 1 g
      const float amag = vNorm(accelG);
      if (amag > 0.55f && amag < 1.45f) {
        const Vec3 aHat = vNormalize(accelG);
        const float dev = fabsf(amag - 1.0f);
        float confidence = 1.0f - clampfLocal(dev / 0.25f, 0.0f, 1.0f);
        confidence *= confidence;

        const float alpha = (1.0f - expf(-dt / 0.45f)) * confidence;
        upBody = vNormalize(
          vAdd(
            vScale(upBody, 1.0f - alpha),
            vScale(aHat, alpha)
          )
        );
      }
    }
  };
  static GravityEstimator gravityEstimator;

  // Giải thuật tính toán góc phương vị bù nghiêng
  static const float MIN_FORWARD_HORIZONTAL = 0.17f;

  static bool computeRawHeading(const Vec3 &magRaw, float &outHeadingDeg) {
    if (!gravityEstimator.initialized) return false;

    const Vec3 up = vNormalize(gravityEstimator.upBody);
    const Vec3 mag = magRaw;

    const float magNorm = vNorm(mag);
    if (magNorm < 1.0e-6f) return false;

    // Chiếu vector từ trường lên mặt phẳng ngang: northH = B - up * (B . up)
    Vec3 northH = vSub(mag, vScale(up, vDot(mag, up)));
    const float northNorm = vNorm(northH);
    if (northNorm < 1.0e-6f) return false;
    northH = vScale(northH, 1.0f / northNorm);

    // Chiếu vector hướng tiến lên mặt phẳng ngang
    Vec3 forwardH = vSub(
      FORWARD_BODY,
      vScale(up, vDot(FORWARD_BODY, up))
    );

    const float forwardHorizontal = vNorm(forwardH);
    if (forwardHorizontal < MIN_FORWARD_HORIZONTAL) {
      return false; // Trường hợp hướng tiến gần vuông góc với mặt phẳng ngang
    }

    forwardH = vScale(forwardH, 1.0f / forwardHorizontal);

    // Tính góc quay theo chiều kim đồng hồ từ Bắc đến hướng tiến
    const float sinAngle = vDot(up, vCross(forwardH, northH));
    const float cosAngle = vDot(northH, forwardH);

    float heading = atan2f(sinAngle, cosAngle) * 57.29577951308232f;
    outHeadingDeg = wrap360(heading);
    return true;
  }

  // Bộ lọc làm mượt góc phương vị
  static bool headingFilterInit = false;
  static float headingFilteredDeg = 0.0f;

  static void resetHeadingFilter(float headingDeg) {
    headingFilteredDeg = wrap360(headingDeg);
    headingFilterInit = true;
  }

  static float updateHeadingFilter(float rawDeg) {
    if (!headingFilterInit) {
      resetHeadingFilter(rawDeg);
      return headingFilteredDeg;
    }

    const float err = wrap180(rawDeg - headingFilteredDeg);
    const float dynamic = clampfLocal(fabsf(err) / 45.0f, 0.0f, 1.0f);
    const float alpha = 0.04f + 0.18f * dynamic;

    headingFilteredDeg = wrap360(headingFilteredDeg + alpha * err);
    return headingFilteredDeg;
  }

  // Ánh xạ góc phương vị sang 8 hướng
  static String headingToDirection(float heading) {
    static const char *directions[8] = {
      "N", "NE", "E", "SE", "S", "SW", "W", "NW"
    };

    const int index = ((int)((wrap360(heading) + 22.5f) / 45.0f)) & 7;
    return String(directions[index]);
  }

  // Biến trạng thái thực thi nội bộ
  static Vec3 latestMagRaw(0.0f, 0.0f, 0.0f);
  static bool haveMagData = false;
  static uint32_t lastUpdateUs = 0;

  static float currentCompassValue = 0.0f;
  static String currentCompassHeading = "N";

} // namespace

// Hàm thực thi
void setupCompass() {
  gravityEstimator = GravityEstimator();
  haveMagData = false;
  headingFilterInit = false;
  headingFilteredDeg = 0.0f;
  lastUpdateUs = micros();
  currentCompassValue = 0.0f;
  currentCompassHeading = "N";
}

void processCompassData(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, float magX, float magY, float magZ) {
  const Vec3 accelBody = applyAxisMap(Vec3(accX, accY, accZ), ACCEL_MAP);
  const Vec3 gyroBody  = applyAxisMap(Vec3(gyrX, gyrY, gyrZ), GYRO_MAP);

  const uint32_t nowUs = micros();
  float dt = (nowUs - lastUpdateUs) * 1.0e-6f;
  lastUpdateUs = nowUs;

  if (dt <= 0.0f || dt > 0.1f) { dt = 0.01f; }

  // Cập nhật bộ ước lượng vector trọng lực
  gravityEstimator.update(accelBody, gyroBody, dt);

  // Lưu giữ mẫu từ trường mới nhất
  latestMagRaw = applyAxisMap(Vec3(magX, magY, magZ), MAG_MAP);
  haveMagData = true;

  // Tính toán bù nghiêng và cập nhật bộ lọc góc liên tục ở tần số 100 Hz (mỗi 10 ms)
  if (haveMagData && gravityEstimator.initialized) {
    float rawHeading = 0.0f;
    if (computeRawHeading(latestMagRaw, rawHeading)) {
      currentCompassValue = updateHeadingFilter(rawHeading);
      currentCompassHeading = headingToDirection(currentCompassValue);
      static uint16_t printCounter = 0;
      printCounter++;
      if (printCounter >= 100) {
        printCounter = 0;
        // Serial.printf("[Góc] %5.1f deg %2s\n", currentCompassValue, currentCompassHeading.c_str());
      }
    }
  }
}

float getCompassValue() { return currentCompassValue; }
String getCompassHeading() { return currentCompassHeading; }
bool isCompassReady() { return gravityEstimator.initialized && haveMagData; }
