#pragma once

#include "Drivetrain.h"
#include "RobotConfig.h"

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

    int attemptCount;

    float stageStartTime;
    float lastCheckTime;
    float lastEncoderPosition;
    float lastHeading;
    float lastErrorMagnitude;

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

    void applyArc(Drivetrain& drivetrain, float linearSpeed, float turnSpeed) {
        float leftPower = linearSpeed + turnSpeed;
        float rightPower = linearSpeed - turnSpeed;

        scaleDrivePower(leftPower, rightPower);
        drivetrain.set_drive_power(leftPower, rightPower);
    }

    void resetMonitor(Drivetrain& drivetrain, float errorMagnitude) {
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

public:
    UnstuckArcHelper()
        : config(),
          stage(UNSTUCK_ARC_IDLE),
          attemptCount(0),
          stageStartTime(0),
          lastCheckTime(0),
          lastEncoderPosition(0),
          lastHeading(0),
          lastErrorMagnitude(0)
    {
    }

    void configure(const UnstuckArcConfig& newConfig) {
        config = newConfig;
    }

    void initialize(Drivetrain& drivetrain, float errorMagnitude) {
        stage = UNSTUCK_ARC_IDLE;
        attemptCount = 0;
        stageStartTime = 0;
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

        if (encoderMoved || headingMoved || errorImproved) {
            resetMonitor(drivetrain, errorMagnitude);
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
        stage = UNSTUCK_ARC_FORWARD_RIGHT;
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

        if (stage == UNSTUCK_ARC_FORWARD_RIGHT) {
            applyArc(drivetrain, config.forward_speed, config.turn_speed);

            if (elapsed >= config.forward_time) {
                stage = UNSTUCK_ARC_FORWARD_LEFT;
                stageStartTime = now;
                resetMonitor(drivetrain, errorMagnitude);
            }

            return true;
        }

        if (stage == UNSTUCK_ARC_FORWARD_LEFT) {
            applyArc(drivetrain, config.forward_speed, -config.turn_speed);

            if (elapsed >= config.forward_time) {
                stage = UNSTUCK_ARC_BACK_RIGHT;
                stageStartTime = now;
                resetMonitor(drivetrain, errorMagnitude);
            }

            return true;
        }

        if (stage == UNSTUCK_ARC_BACK_RIGHT) {
            applyArc(drivetrain, -config.reverse_speed, config.turn_speed);

            if (elapsed >= config.reverse_time) {
                stage = UNSTUCK_ARC_BACK_LEFT;
                stageStartTime = now;
                resetMonitor(drivetrain, errorMagnitude);
            }

            return true;
        }

        if (stage == UNSTUCK_ARC_BACK_LEFT) {
            applyArc(drivetrain, -config.reverse_speed, -config.turn_speed);

            if (elapsed >= config.reverse_time) {
                stage = UNSTUCK_ARC_IDLE;
                resetMonitor(drivetrain, errorMagnitude);
            }

            return true;
        }

        return false;
    }
};
