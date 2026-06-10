#pragma once

#include "Drivetrain.h"
#include "RobotConfig.h"
#include "RandomUnstuckOrder.h"

#include <cmath>

struct UnstuckArcConfig {
    float stuck_check_time;
    float encoder_change_threshold;
    float heading_change_threshold;
    float error_progress_threshold;
    float forward_speed;
    float reverse_speed;
    float turn_speed;
    float forward_time;
    float reverse_time;
    float max_accel;
    int max_attempts;
};

enum UnstuckArcStage {
    UNSTUCK_ARC_IDLE,
    UNSTUCK_ARC_FORWARD_RIGHT,
    UNSTUCK_ARC_FORWARD_LEFT,
    UNSTUCK_ARC_BACK_RIGHT,
    UNSTUCK_ARC_BACK_LEFT,
    UNSTUCK_ARC_FAILED
};

class UnstuckArcHelper {
private:
    UnstuckArcConfig config;
    UnstuckArcStage stage;
    RandomUnstuckOrder unstuckOrder;

    int attemptCount;
    int noProgressCheckCount;

    float stageStartTime;
    float lastCheckTime;
    float lastEncoderPosition;
    float lastHeading;
    float lastErrorMagnitude;
    float previousLeftPower;
    float previousRightPower;
    bool runningLateralShift;
    float lateralShiftStageTime;
    bool lateralShiftStageAtSpeed;
    bool lateralShiftFinishing;

    float getHeadingChange(float currentHeading, float previousHeading) {
        float change = currentHeading - previousHeading;

        while (change > 180.0f) {
            change -= 360.0f;
        }

        while (change < -180.0f) {
            change += 360.0f;
        }

        return change;
    }

    void scaleDrivePower(float& leftPower, float& rightPower) {
        float maxMagnitude = std::fabs(leftPower);

        if (std::fabs(rightPower) > maxMagnitude) {
            maxMagnitude = std::fabs(rightPower);
        }

        if (maxMagnitude > 100.0f) {
            leftPower = leftPower * 100.0f / maxMagnitude;
            rightPower = rightPower * 100.0f / maxMagnitude;
        }
    }

    bool applyArc(Drivetrain& drivetrain, float linearSpeed, float turnSpeed) {
        float targetLeftPower = linearSpeed + turnSpeed;
        float targetRightPower = linearSpeed - turnSpeed;

        scaleDrivePower(targetLeftPower, targetRightPower);

        float leftPower = targetLeftPower;
        float rightPower = targetRightPower;

        if (config.max_accel > 0.0f) {
            float maxChange = config.max_accel / 100.0f;

            if (leftPower > previousLeftPower + maxChange) {
                leftPower = previousLeftPower + maxChange;
            } else if (leftPower < previousLeftPower - maxChange) {
                leftPower = previousLeftPower - maxChange;
            }

            if (rightPower > previousRightPower + maxChange) {
                rightPower = previousRightPower + maxChange;
            } else if (rightPower < previousRightPower - maxChange) {
                rightPower = previousRightPower - maxChange;
            }
        }

        drivetrain.set_drive_power(leftPower, rightPower);
        previousLeftPower = leftPower;
        previousRightPower = rightPower;

        return
            std::fabs(leftPower - targetLeftPower) < 0.01f &&
            std::fabs(rightPower - targetRightPower) < 0.01f;
    }

    void resetMonitor(Drivetrain& drivetrain, float errorMagnitude) {
        noProgressCheckCount = 0;
        lastCheckTime = master_timer.time(msec);
        lastEncoderPosition = drivetrain.get_left_front_motor_position();
        lastHeading = drivetrain.get_heading_degrees();
        lastErrorMagnitude = errorMagnitude;
    }

    void advanceMonitorWindow(Drivetrain& drivetrain, float errorMagnitude) {
        lastCheckTime = master_timer.time(msec);
        lastEncoderPosition = drivetrain.get_left_front_motor_position();
        lastHeading = drivetrain.get_heading_degrees();
        lastErrorMagnitude = errorMagnitude;
    }

    bool hasAnyProgressSignal() const {
        return
            config.encoder_change_threshold > 0.0f ||
            config.heading_change_threshold > 0.0f ||
            config.error_progress_threshold > 0.0f;
    }

    UnstuckArcStage stageFromMove(int move) const {
        if (move == RANDOM_UNSTUCK_FORWARD_RIGHT) {
            return UNSTUCK_ARC_FORWARD_RIGHT;
        }

        if (move == RANDOM_UNSTUCK_FORWARD_LEFT) {
            return UNSTUCK_ARC_FORWARD_LEFT;
        }

        if (move == RANDOM_UNSTUCK_BACK_RIGHT) {
            return UNSTUCK_ARC_BACK_RIGHT;
        }

        return UNSTUCK_ARC_BACK_LEFT;
    }

    float timeForStage(UnstuckArcStage currentStage) const {
        if (runningLateralShift) {
            return lateralShiftStageTime;
        }

        if (currentStage == UNSTUCK_ARC_FORWARD_RIGHT ||
            currentStage == UNSTUCK_ARC_FORWARD_LEFT) {
            return config.forward_time;
        }

        return config.reverse_time;
    }

    bool applyStage(Drivetrain& drivetrain, UnstuckArcStage currentStage) {
        if (currentStage == UNSTUCK_ARC_FORWARD_RIGHT) {
            return applyArc(drivetrain, config.forward_speed, config.turn_speed);
        }
        else if (currentStage == UNSTUCK_ARC_FORWARD_LEFT) {
            return applyArc(drivetrain, config.forward_speed, -config.turn_speed);
        }
        else if (currentStage == UNSTUCK_ARC_BACK_RIGHT) {
            return applyArc(drivetrain, -config.reverse_speed, config.turn_speed);
        }
        else if (currentStage == UNSTUCK_ARC_BACK_LEFT) {
            return applyArc(drivetrain, -config.reverse_speed, -config.turn_speed);
        }

        return false;
    }

    unsigned int makeSeed(Drivetrain& drivetrain) {
        return
            static_cast<unsigned int>(master_timer.time(msec)) ^
            static_cast<unsigned int>(drivetrain.get_left_front_motor_position() * 31.0f) ^
            static_cast<unsigned int>(drivetrain.get_heading_degrees() * 17.0f) ^
            static_cast<unsigned int>(attemptCount * 97);
    }

public:
    UnstuckArcHelper()
        : config(),
          stage(UNSTUCK_ARC_IDLE),
          unstuckOrder(),
          attemptCount(0),
          noProgressCheckCount(0),
          stageStartTime(0),
          lastCheckTime(0),
          lastEncoderPosition(0),
          lastHeading(0),
          lastErrorMagnitude(0),
          previousLeftPower(0),
          previousRightPower(0),
          runningLateralShift(false),
          lateralShiftStageTime(0),
          lateralShiftStageAtSpeed(false),
          lateralShiftFinishing(false)
    {
    }

    void configure(const UnstuckArcConfig& newConfig) {
        config = newConfig;
    }

    void initialize(Drivetrain& drivetrain, float errorMagnitude) {
        stage = UNSTUCK_ARC_IDLE;
        attemptCount = 0;
        noProgressCheckCount = 0;
        stageStartTime = 0;
        previousLeftPower = 0;
        previousRightPower = 0;
        runningLateralShift = false;
        lateralShiftStageTime = 0;
        lateralShiftStageAtSpeed = false;
        lateralShiftFinishing = false;
        resetMonitor(drivetrain, errorMagnitude);
    }

    bool isEnabled() const {
        return
            config.max_attempts > 0 &&
            config.stuck_check_time > 0.0f &&
            hasAnyProgressSignal();
    }

    bool isActive() const {
        return
            stage == UNSTUCK_ARC_FORWARD_RIGHT ||
            stage == UNSTUCK_ARC_FORWARD_LEFT ||
            stage == UNSTUCK_ARC_BACK_RIGHT ||
            stage == UNSTUCK_ARC_BACK_LEFT;
    }

    bool hasFailed() const {
        return stage == UNSTUCK_ARC_FAILED;
    }

    bool shouldStart(Drivetrain& drivetrain, float errorMagnitude, bool shouldCheck) {
        if (!isEnabled() || isActive() || hasFailed()) {
            return false;
        }

        if (!shouldCheck) {
            resetMonitor(drivetrain, errorMagnitude);
            return false;
        }

        float now = master_timer.time(msec);

        if (now - lastCheckTime < config.stuck_check_time) {
            return false;
        }

        float currentEncoderPosition = drivetrain.get_left_front_motor_position();
        float currentHeading = drivetrain.get_heading_degrees();

        float encoderChange =
            currentEncoderPosition - lastEncoderPosition;

        float headingChange =
            getHeadingChange(currentHeading, lastHeading);

        float errorProgress =
            lastErrorMagnitude - errorMagnitude;

        bool encoderMoved =
            config.encoder_change_threshold > 0.0f &&
            std::fabs(encoderChange) >= config.encoder_change_threshold;

        bool headingMoved =
            config.heading_change_threshold > 0.0f &&
            std::fabs(headingChange) >= config.heading_change_threshold;

        bool errorImproved =
            config.error_progress_threshold > 0.0f &&
            errorProgress >= config.error_progress_threshold;

        bool onlyEncoderProgressAvailable =
            config.heading_change_threshold <= 0.0f &&
            config.error_progress_threshold <= 0.0f;

        if (headingMoved ||
            errorImproved ||
            (onlyEncoderProgressAvailable && encoderMoved)) {

            // A later obstruction should still be allowed to use unstuck.
            // max_attempts only limits consecutive escapes with no real progress.
            attemptCount = 0;
            resetMonitor(drivetrain, errorMagnitude);
            return false;
        }

        noProgressCheckCount++;

        if (noProgressCheckCount < REQUIRED_CONSECUTIVE_STUCK_CHECKS) {
            advanceMonitorWindow(drivetrain, errorMagnitude);
            return false;
        }

        return true;
    }

    bool start(Drivetrain& drivetrain, float errorMagnitude) {
        if (!isEnabled()) {
            return false;
        }

        if (attemptCount >= config.max_attempts) {
            stage = UNSTUCK_ARC_FAILED;
            drivetrain.set_drive_power(0, 0);
            return false;
        }

        attemptCount++;
        previousLeftPower = 0;
        previousRightPower = 0;
        runningLateralShift = false;
        lateralShiftStageAtSpeed = false;
        lateralShiftFinishing = false;
        unstuckOrder.reset(makeSeed(drivetrain));
        stage = stageFromMove(unstuckOrder.current());
        stageStartTime = master_timer.time(msec);
        resetMonitor(drivetrain, errorMagnitude);
        return true;
    }

    bool startLateralShift(
        Drivetrain& drivetrain,
        float errorMagnitude,
        bool wasMovingForward,
        float initialLeftPower,
        float initialRightPower
    ) {
        if (!isEnabled()) {
            return false;
        }

        if (attemptCount >= config.max_attempts) {
            stage = UNSTUCK_ARC_FAILED;
            drivetrain.set_drive_power(0, 0);
            return false;
        }

        attemptCount++;
        scaleDrivePower(initialLeftPower, initialRightPower);
        previousLeftPower = initialLeftPower;
        previousRightPower = initialRightPower;
        runningLateralShift = true;
        lateralShiftStageAtSpeed = false;
        lateralShiftFinishing = false;
        lateralShiftStageTime =
            (wasMovingForward
                ? config.reverse_time
                : config.forward_time) *
            LATERAL_SHIFT_TIME_MULTIPLIER;
        unstuckOrder.resetLateralShift(makeSeed(drivetrain), wasMovingForward);
        stage = stageFromMove(unstuckOrder.current());
        stageStartTime = master_timer.time(msec);
        resetMonitor(drivetrain, errorMagnitude);
        return true;
    }

    bool run(Drivetrain& drivetrain, float errorMagnitude) {
        if (hasFailed()) {
            drivetrain.set_drive_power(0, 0);
            return false;
        }

        if (!isActive()) {
            return false;
        }

        float now = master_timer.time(msec);
        float elapsed = now - stageStartTime;

        if (isActive()) {
            if (runningLateralShift && lateralShiftFinishing) {
                if (applyArc(drivetrain, 0.0f, 0.0f)) {
                    stage = UNSTUCK_ARC_IDLE;
                    previousLeftPower = 0;
                    previousRightPower = 0;
                    runningLateralShift = false;
                    lateralShiftStageAtSpeed = false;
                    lateralShiftFinishing = false;
                    resetMonitor(drivetrain, errorMagnitude);
                }

                return true;
            }

            bool stageAtSpeed = applyStage(drivetrain, stage);

            // Do not consume a stage's timed movement while acceleration
            // limiting is still ramping toward the requested arc power.
            if (!lateralShiftStageAtSpeed) {
                if (stageAtSpeed) {
                    lateralShiftStageAtSpeed = true;
                    stageStartTime = now;
                }

                return true;
            }

            if (elapsed >= timeForStage(stage)) {
                if (unstuckOrder.advance()) {
                    stage = stageFromMove(unstuckOrder.current());
                    stageStartTime = now;
                    lateralShiftStageAtSpeed = false;
                    resetMonitor(drivetrain, errorMagnitude);
                } else if (runningLateralShift) {
                    lateralShiftFinishing = true;
                } else {
                    stage = UNSTUCK_ARC_IDLE;
                    previousLeftPower = 0;
                    previousRightPower = 0;
                    runningLateralShift = false;
                    resetMonitor(drivetrain, errorMagnitude);
                }
            }

            return true;
        }

        return false;
    }
};
