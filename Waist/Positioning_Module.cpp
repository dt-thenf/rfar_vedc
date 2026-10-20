#include <math.h>
#include <stdint.h>
#include "Positioning_Module.h"

namespace {
  const float MAP_X_MAX_M = 100.0f;
  const float MAP_Y_MAX_M = 50.0f;
  const float INITIAL_X_M = MAP_X_MAX_M * 0.5f;
  const float INITIAL_Y_M = MAP_Y_MAX_M * 0.5f;

  enum PdrState {
    PDR_WAIT_HEADING = 0,
    PDR_TURNING,
    PDR_STRAIGHT_WALK
  };

  PdrState currentPdrState = PDR_WAIT_HEADING;
  const uint32_t STOP_CONFIRM_INTERVAL_MS = 1200UL;
  const float GYRO_TURNING_THRESHOLD_DPS = 20.0f;
  const float GYRO_STABLE_THRESHOLD_DPS = 10.0f;

  // Recent circular heading window is diagnostic/smoothing context only.
  // It no longer blocks step integration while waiting for a stationary heading.
  const uint8_t HEADING_WINDOW_CAPACITY = 24;
  const uint32_t HEADING_SAMPLE_INTERVAL_MS = 25UL;
  float headingBuffer[HEADING_WINDOW_CAPACITY];
  uint8_t headingBufferCount = 0;
  uint8_t headingBufferWriteIdx = 0;
  uint32_t headingLastSampleMs = 0;

  PDRPosition currentPosition = {};
  uint32_t processedStepCount = 0;
  float processedDistanceM = 0.0f;
  bool isInitialStepSyncNeeded = true;

  float lockedHeadingDeg = NAN;
  bool isWalkingSegmentOpen = false;
  bool hasRecentStepTimestamp = false;
  uint32_t recentStepTimestampMs = 0;
  bool isTurnReportedWhileWalking = false;

  // Keep the unbounded estimate for diagnostics; expose bounded map coordinates.
  float rawPositionXM = INITIAL_X_M;
  float rawPositionYM = INITIAL_Y_M;

  float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
  }

  float normalizeAngle360(float headingDeg) {
    if (!isfinite(headingDeg)) return NAN;
    headingDeg = fmodf(headingDeg, 360.0f);
    if (headingDeg < 0.0f) headingDeg += 360.0f;
    return headingDeg;
  }

  float getSmallestAngleDifference(float angleA, float angleB) {
    if (!isfinite(angleA) || !isfinite(angleB)) return NAN;
    float diff = normalizeAngle360(angleA) - normalizeAngle360(angleB);
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
  }

  bool isMovementActivity(const String &activity) {
    return activity == "WA" || activity == "JO" || activity == "WS";
  }

  String angleToDirectionString(float headingDeg) {
    headingDeg = normalizeAngle360(headingDeg);
    if (!isfinite(headingDeg)) return "None";
    static const char *directions[8] = {
      "N", "NE", "E", "SE", "S", "SW", "W", "NW"
    };
    const int index = ((int)((headingDeg + 22.5f) / 45.0f)) & 7;
    return String(directions[index]);
  }

  const char *getPdrStateName(PdrState state) {
    switch (state) {
      case PDR_TURNING: return "TURNING";
      case PDR_STRAIGHT_WALK: return "STRAIGHT_WALK";
      case PDR_WAIT_HEADING:
      default: return "WAIT_HEADING";
    }
  }

  void updatePdrState(PdrState state) {
    currentPdrState = state;
    currentPosition.pdrState = getPdrStateName(state);
  }

  void resetHeadingWindow() {
    headingBufferCount = 0;
    headingBufferWriteIdx = 0;
    headingLastSampleMs = 0;
  }

  void pushHeadingSample(float headingDeg, uint32_t currentMs) {
    headingDeg = normalizeAngle360(headingDeg);
    if (!isfinite(headingDeg)) return;

    if (headingBufferCount > 0 &&
        (uint32_t)(currentMs - headingLastSampleMs) < HEADING_SAMPLE_INTERVAL_MS) {
      return;
    }

    headingLastSampleMs = currentMs;
    headingBuffer[headingBufferWriteIdx] = headingDeg;
    headingBufferWriteIdx = (uint8_t)((headingBufferWriteIdx + 1) % HEADING_WINDOW_CAPACITY);
    if (headingBufferCount < HEADING_WINDOW_CAPACITY) headingBufferCount++;
  }

  bool computeHeadingStats(float &meanDeg, float &spreadDeg) {
    meanDeg = NAN;
    spreadDeg = 180.0f;
    if (headingBufferCount == 0) return false;

    float sumSin = 0.0f;
    float sumCos = 0.0f;
    for (uint8_t i = 0; i < headingBufferCount; ++i) {
      const float radians = headingBuffer[i] * (PI / 180.0f);
      sumSin += sinf(radians);
      sumCos += cosf(radians);
    }

    const float count = (float)headingBufferCount;
    const float resultantLength = sqrtf(sumSin * sumSin + sumCos * sumCos) / count;
    if (resultantLength < 1.0e-4f) return false;

    meanDeg = normalizeAngle360(atan2f(sumSin, sumCos) * (180.0f / PI));
    float maxSpread = 0.0f;
    for (uint8_t i = 0; i < headingBufferCount; ++i) {
      const float difference = fabsf(getSmallestAngleDifference(headingBuffer[i], meanDeg));
      if (isfinite(difference) && difference > maxSpread) maxSpread = difference;
    }
    spreadDeg = maxSpread;
    return true;
  }

  void closeWalkingSegment() {
    isWalkingSegmentOpen = false;
    lockedHeadingDeg = NAN;
    currentPosition.lockedHeading = "None";
    currentPosition.isMoving = false;
    hasRecentStepTimestamp = false;
    isTurnReportedWhileWalking = false;
  }

  void enterWaitHeadingState() {
    closeWalkingSegment();
    resetHeadingWindow();
    updatePdrState(PDR_WAIT_HEADING);
  }

  void enterTurningState() {
    closeWalkingSegment();
    resetHeadingWindow();
    updatePdrState(PDR_TURNING);
  }

  void noteTurnWhileWalking() {
    if (!isWalkingSegmentOpen || isTurnReportedWhileWalking) return;
    currentPosition.turnWhileMovingCount++;
    isTurnReportedWhileWalking = true;
    updatePdrState(PDR_TURNING);
  }

  void updateConfidenceMetric() {
    float confidence = 1.0f;
    confidence -= clampFloat(currentPosition.distanceSinceAnchorM / 220.0f, 0.0f, 0.45f);
    confidence -= clampFloat(currentPosition.headingSpreadDeg / 80.0f, 0.0f, 0.15f);
    if (currentPosition.isBoundaryHit) confidence -= 0.15f;
    confidence -= clampFloat((float)currentPosition.turnWhileMovingCount * 0.02f, 0.0f, 0.15f);
    currentPosition.relativeConfidence = clampFloat(confidence, 0.0f, 1.0f);
  }

  /*
   * PDR bearing convention:
   *   0 degrees = north, 90 degrees = east.
   * Map X grows east and map Y grows north:
   *   dX = distance * sin(heading)
   *   dY = distance * cos(heading)
   *
   * Integrate each newly detected step with the current filtered heading. Do not
   * lock one stationary heading for an entire walking segment: that fails when
   * the wearer turns while walking.
   */
  void integrateSteps(uint32_t newSteps, float deltaDistanceM, float headingDeg) {
    headingDeg = normalizeAngle360(headingDeg);
    if (!isfinite(headingDeg) || !isfinite(deltaDistanceM) ||
        deltaDistanceM <= 0.0001f) {
      return;
    }

    if (newSteps > 0) {
      currentPosition.stepLengthM = deltaDistanceM / (float)newSteps;
    }

    const float radians = headingDeg * (PI / 180.0f);
    const float deltaX = deltaDistanceM * sinf(radians);
    const float deltaY = deltaDistanceM * cosf(radians);

    rawPositionXM += deltaX;
    rawPositionYM += deltaY;
    currentPosition.distanceSinceAnchorM += deltaDistanceM;

    const float boundedX = clampFloat(rawPositionXM, 0.0f, MAP_X_MAX_M);
    const float boundedY = clampFloat(rawPositionYM, 0.0f, MAP_Y_MAX_M);
    currentPosition.isBoundaryHit =
      fabsf(boundedX - rawPositionXM) > 0.0001f ||
      fabsf(boundedY - rawPositionYM) > 0.0001f;
    if (currentPosition.isBoundaryHit) currentPosition.boundaryHitCount++;

    currentPosition.rawXM = rawPositionXM;
    currentPosition.rawYM = rawPositionYM;
    currentPosition.xM = boundedX;
    currentPosition.yM = boundedY;
    currentPosition.headingSpreadDeg =
      isfinite(currentPosition.headingSpreadDeg) ? currentPosition.headingSpreadDeg : 180.0f;
    updateConfidenceMetric();
  }
} // namespace

void setupPositioning() {
  resetPosition(INITIAL_X_M, INITIAL_Y_M);
}

void processPositioning(const String &rawAct,
                        uint32_t totalSteps,
                        float totalDistanceM,
                        float headingDeg,
                        float gyroMagnitudeDps,
                        bool isCompassReady) {
  const uint32_t now = millis();
  currentPosition.isBoundaryHit = false;

  if (isInitialStepSyncNeeded) {
    processedStepCount = totalSteps;
    processedDistanceM = isfinite(totalDistanceM) ? totalDistanceM : 0.0f;
    isInitialStepSyncNeeded = false;
  }

  if (totalSteps < processedStepCount ||
      !isfinite(totalDistanceM) ||
      totalDistanceM + 0.0001f < processedDistanceM) {
    processedStepCount = totalSteps;
    processedDistanceM = isfinite(totalDistanceM) ? totalDistanceM : 0.0f;
    enterWaitHeadingState();
    return;
  }

  const uint32_t newSteps = totalSteps - processedStepCount;
  const float deltaDistanceM = totalDistanceM - processedDistanceM;
  const bool hasPendingSteps = newSteps > 0 || deltaDistanceM > 0.0001f;
  const bool isMovingAct = isMovementActivity(rawAct);
  const bool gyroTurning = isfinite(gyroMagnitudeDps) &&
                           gyroMagnitudeDps >= GYRO_TURNING_THRESHOLD_DPS;
  const bool gyroStable = isfinite(gyroMagnitudeDps) &&
                          gyroMagnitudeDps <= GYRO_STABLE_THRESHOLD_DPS;
  const bool validHeading = isCompassReady && isfinite(headingDeg);

  if (validHeading) {
    pushHeadingSample(headingDeg, now);
    float meanHeading = NAN;
    float headingSpread = 180.0f;
    if (computeHeadingStats(meanHeading, headingSpread)) {
      currentPosition.headingSpreadDeg = headingSpread;
    }
  }

  if (gyroTurning) {
    if (isWalkingSegmentOpen) {
      noteTurnWhileWalking();
    } else if (!hasPendingSteps && !isMovingAct &&
               currentPdrState != PDR_TURNING) {
      enterTurningState();
      return;
    } else if (!isWalkingSegmentOpen) {
      updatePdrState(PDR_TURNING);
    }
  } else if (gyroStable) {
    isTurnReportedWhileWalking = false;
  }

  /*
   * New steps are integrated immediately with the latest valid compass heading.
   * If the heading is temporarily invalid, leave the counters unconsumed so the
   * distance remains pending and can be integrated when a valid bearing returns.
   */
  if (hasPendingSteps) {
    if (validHeading) {
      if (deltaDistanceM > 0.0001f) {
        integrateSteps(newSteps, deltaDistanceM, headingDeg);
        processedStepCount = totalSteps;
        processedDistanceM = totalDistanceM;

        lockedHeadingDeg = normalizeAngle360(headingDeg);
        currentPosition.lockedHeading = angleToDirectionString(lockedHeadingDeg);
        isWalkingSegmentOpen = true;
        currentPosition.isMoving = true;
        hasRecentStepTimestamp = true;
        recentStepTimestampMs = now;

        if (gyroTurning) {
          noteTurnWhileWalking();
          updatePdrState(PDR_TURNING);
        } else {
          updatePdrState(PDR_STRAIGHT_WALK);
        }
        return;
      }

      // A step count can arrive one loop before its distance update. Consume the
      // count only; any later distance increase is still integrated normally.
      processedStepCount = totalSteps;
      processedDistanceM = totalDistanceM;
    } else {
      currentPosition.isMoving = isMovingAct || hasPendingSteps;
      if (gyroTurning) updatePdrState(PDR_TURNING);
      else if (isMovingAct) updatePdrState(PDR_STRAIGHT_WALK);
      return;
    }
  }

  if (isMovingAct) {
    currentPosition.isMoving = true;
    if (gyroTurning) {
      updatePdrState(PDR_TURNING);
    } else if (validHeading) {
      updatePdrState(PDR_STRAIGHT_WALK);
    }
    return;
  }

  currentPosition.isMoving = false;
  if (isWalkingSegmentOpen) {
    if (!hasRecentStepTimestamp ||
        (uint32_t)(now - recentStepTimestampMs) >= STOP_CONFIRM_INTERVAL_MS) {
      closeWalkingSegment();
      updatePdrState(PDR_WAIT_HEADING);
    }
    return;
  }

  if (gyroTurning) {
    if (currentPdrState != PDR_TURNING) enterTurningState();
    return;
  }

  if (currentPdrState == PDR_TURNING && gyroStable) {
    resetHeadingWindow();
    updatePdrState(PDR_WAIT_HEADING);
  } else if (currentPdrState != PDR_WAIT_HEADING) {
    updatePdrState(PDR_WAIT_HEADING);
  }
}

PDRPosition getPDRPosition() {
  return currentPosition;
}

float getPositionX() {
  return currentPosition.xM;
}

float getPositionY() {
  return currentPosition.yM;
}

void resetPosition(float xM, float yM) {
  rawPositionXM = xM;
  rawPositionYM = yM;

  currentPosition.rawXM = rawPositionXM;
  currentPosition.rawYM = rawPositionYM;
  currentPosition.xM = clampFloat(rawPositionXM, 0.0f, MAP_X_MAX_M);
  currentPosition.yM = clampFloat(rawPositionYM, 0.0f, MAP_Y_MAX_M);
  currentPosition.xMaxM = MAP_X_MAX_M;
  currentPosition.yMaxM = MAP_Y_MAX_M;

  currentPosition.stepLengthM = 0.65f;
  currentPosition.lockedHeading = "None";
  currentPosition.isMoving = false;
  currentPosition.isBoundaryHit = false;
  currentPosition.distanceSinceAnchorM = 0.0f;
  currentPosition.headingSpreadDeg = 0.0f;
  currentPosition.relativeConfidence = 1.0f;
  currentPosition.boundaryHitCount = 0;
  currentPosition.turnWhileMovingCount = 0;
  currentPosition.pdrState = "WAIT_HEADING";

  processedStepCount = 0;
  processedDistanceM = 0.0f;
  isInitialStepSyncNeeded = true;
  lockedHeadingDeg = NAN;
  isWalkingSegmentOpen = false;
  hasRecentStepTimestamp = false;
  recentStepTimestampMs = 0;
  isTurnReportedWhileWalking = false;
  resetHeadingWindow();
  updatePdrState(PDR_WAIT_HEADING);
}
