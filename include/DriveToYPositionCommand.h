#pragma once

#include "Command.h"
#include "Drivetrain.h"
#include "PositionTracking.h"
#include "RobotConfig.h"
#include "UnstuckArcHelper.h"
#include "CommandStatus.h"

#include <cmath>

struct DriveToYPositionTarget {
    float target_y;
    float target_heading;
};

struct DriveToYPositionConfig {
    float max_linear_speed;
    float max_angular_speed;
    float acceptable_error;
    float heading_acceptable_error;
    float max_linear_heading_error;
    float min_linear_speed;
    float min_angular_speed;
    float max_time;
    float settle_time;
    UnstuckArcConfig unstuck;
};

class DriveToYPositionCommand : public Command {
private:
    Drivetrain& drivetrain;
    PositionTracking& positionTracking;
    DriveToYPositionTarget target;
    DriveToYPositionConfig config;
    DrivePID PID;

    bool finished;
    bool thresholdTimerSet;
    bool atTarget;
    bool timedOut;

    float endTime;
    float insideThresholdEndTime;
    float linearPreviousError;
    float linearIntegralError;
    float angularPreviousError;
    float angularIntegralError;
    UnstuckArcHelper unstuck;

    float headingError(float targetHeading, float currentHeading) {
        float error = targetHeading - currentHeading;

        while (error > 180.0f) {
            error -= 360.0f;
        }

        while (error < -180.0f) {
            error += 360.0f;
        }

        return error;
    }

    float clamp(float value, float maxMagnitude) {
        if (std::fabs(value) <= maxMagnitude) {
            return value;
        }

        return value > 0 ? maxMagnitude : -maxMagnitude;
    }

    float applyMinimumSpeed(
        float speed,
        float minimumSpeed,
        float error,
        float acceptableError
    ) {
        if (minimumSpeed <= 0.0f ||
            std::fabs(error) <= acceptableError ||
            std::fabs(speed) >= minimumSpeed) {

            return speed;
        }

        return error > 0.0f ? minimumSpeed : -minimumSpeed;
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

    float getForwardAxisSign() {
        const float PI = 3.14159265f;
        float headingRadians = target.target_heading * PI / 180.0f;
        float yComponent = std::cos(headingRadians);

        if (std::fabs(yComponent) < 0.1f) {
            return 1.0f;
        }

        return yComponent > 0 ? 1.0f : -1.0f;
    }

public:
    DriveToYPositionCommand(
        Drivetrain& drivetrain,
        PositionTracking& positionTracking,
        DriveToYPositionTarget target,
        DriveToYPositionConfig config,
        DrivePID PID
    )
        : drivetrain(drivetrain),
          positionTracking(positionTracking),
          target(target),
          config(config),
          PID(PID),
          finished(false),
          thresholdTimerSet(false),
          atTarget(false),
          timedOut(false),
          endTime(0),
          insideThresholdEndTime(0),
          linearPreviousError(0),
          linearIntegralError(0),
          angularPreviousError(0),
          angularIntegralError(0)
    {
    }

    void initialize() override {
        setCommandStatus("Drive To Y Position");
        finished = false;
        thresholdTimerSet = false;
        atTarget = false;
        timedOut = false;

        endTime = master_timer.time(msec) + config.max_time;
        insideThresholdEndTime = 0;

        positionTracking.update_raw_pose();

        float rawYError = target.target_y - positionTracking.get_y();
        linearPreviousError = rawYError * getForwardAxisSign();
        linearIntegralError = 0;
        angularPreviousError =
            headingError(target.target_heading, drivetrain.get_heading_degrees());
        angularIntegralError = 0;

        unstuck.configure(config.unstuck);
        unstuck.initialize(
            drivetrain,
            std::fabs(rawYError)
        );
    }

    void execute() override {
        positionTracking.update_raw_pose();

        float now = master_timer.time(msec);
        float rawYError = target.target_y - positionTracking.get_y();
        float linearError = rawYError * getForwardAxisSign();
        float linearDerivativeError = linearError - linearPreviousError;
        linearPreviousError = linearError;
        float progressError = std::fabs(rawYError);

        float angularError =
            headingError(target.target_heading, drivetrain.get_heading_degrees());
        float angularDerivativeError = angularError - angularPreviousError;
        angularPreviousError = angularError;

        if (now >= endTime) {
            timedOut = true;
            finished = true;
            return;
        }

        if (unstuck.hasFailed()) {
            timedOut = true;
            finished = true;
            return;
        }

        if (unstuck.run(drivetrain, progressError)) {
            return;
        }

        if (std::fabs(linearError) <= PID.linear_integral_windup_threshold) {
            linearIntegralError += linearError;
        } else {
            linearIntegralError = 0;
        }

        if (std::fabs(angularError) <= PID.angular_integral_windup_threshold) {
            angularIntegralError += angularError;
        } else {
            angularIntegralError = 0;
        }

        float linearSpeed =
            PID.linear_kP * linearError +
            PID.linear_kI * linearIntegralError +
            PID.linear_kD * linearDerivativeError;

        float angularSpeed =
            PID.angular_kP * angularError +
            PID.angular_kI * angularIntegralError +
            PID.angular_kD * angularDerivativeError;

        linearSpeed = applyMinimumSpeed(
            linearSpeed,
            config.min_linear_speed,
            linearError,
            config.acceptable_error
        );
        angularSpeed = applyMinimumSpeed(
            angularSpeed,
            config.min_angular_speed,
            angularError,
            config.heading_acceptable_error
        );

        linearSpeed = clamp(linearSpeed, config.max_linear_speed);
        angularSpeed = clamp(angularSpeed, config.max_angular_speed);

        bool headingReadyForTranslation =
            config.max_linear_heading_error <= 0.0f ||
            std::fabs(angularError) <= config.max_linear_heading_error;

        bool headingAtTarget =
            config.heading_acceptable_error <= 0.0f ||
            std::fabs(angularError) <= config.heading_acceptable_error;

        if (!headingReadyForTranslation) {
            linearSpeed = 0.0f;
            linearIntegralError = 0.0f;
        }

        if (std::fabs(rawYError) <= config.acceptable_error) {
            linearSpeed = 0.0f;
            linearIntegralError = 0.0f;
        }

        if (headingAtTarget) {
            angularSpeed = 0.0f;
            angularIntegralError = 0.0f;
        }

        float leftPower = linearSpeed + angularSpeed;
        float rightPower = linearSpeed - angularSpeed;
        scaleDrivePower(leftPower, rightPower);

        bool tryingToTranslate =
            headingReadyForTranslation &&
            std::fabs(rawYError) > config.acceptable_error &&
            std::fabs(linearSpeed) > 5.0f;

        if (unstuck.shouldStart(drivetrain, progressError, tryingToTranslate)) {
            if (!unstuck.startLateralShift(
                    drivetrain,
                    progressError,
                    linearSpeed >= 0.0f,
                    leftPower,
                    rightPower
                )) {
                timedOut = true;
                finished = true;
            }

            return;
        }

        drivetrain.set_drive_power(leftPower, rightPower);

        if (std::fabs(rawYError) <= config.acceptable_error &&
            headingAtTarget) {

            if (!thresholdTimerSet) {
                insideThresholdEndTime = now + config.settle_time;
                thresholdTimerSet = true;
            }

            if (now >= insideThresholdEndTime) {
                atTarget = true;
            }
        } else {
            thresholdTimerSet = false;
            atTarget = false;
        }

        if (now >= endTime) {
            timedOut = true;
        }

        finished = atTarget || timedOut;
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
        drivetrain.stop();
    }
};
