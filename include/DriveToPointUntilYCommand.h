#pragma once

#include "Command.h"
#include "Drivetrain.h"
#include "DriveToPointCommand.h"
#include "PositionTracking.h"
#include "RobotConfig.h"
#include "CommandStatus.h"

#include <cmath>

enum DriveToPointYExitDirection {
    DRIVE_TO_POINT_EXIT_ABOVE_Y,
    DRIVE_TO_POINT_EXIT_BELOW_Y
};

struct DriveToPointUntilYTarget {
    float target_x;
    float target_y;
    float exit_y;
    DriveToPointYExitDirection exit_direction;
};

class DriveToPointUntilYCommand : public Command {
private:
    Drivetrain& drivetrain;
    PositionTracking& positionTracking;
    DriveToPointUntilYTarget target;
    DriveToPointConfig config;
    DrivePID PID;

    DriveToPointState currentState;

    bool finished;
    bool pointingTimerSet;
    bool timedOut;

    float endTime;
    float pointingSettleEndTime;

    float linearPreviousError;
    float linearIntegralError;
    float angularPreviousError;
    float angularIntegralError;

    float currentX;
    float currentY;
    float targetHeading;
    float distanceError;
    float headingErrorValue;
    float lastStuckCheckTime;
    float lastDistanceError;
    float lastHeadingError;
    float lastEncoderPosition;
    float unstuckStartTime;
    int unstuckAttemptCount;

    float normalizeHeading(float heading) {
        while (heading >= 360.0f) {
            heading -= 360.0f;
        }

        while (heading < 0.0f) {
            heading += 360.0f;
        }

        return heading;
    }

    float getHeadingError(float targetHeading, float currentHeading) {
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
        float value,
        float minMagnitude,
        float maxMagnitude,
        float signSource
    ) {
        if (minMagnitude <= 0 || maxMagnitude <= 0) {
            return value;
        }

        float minimumSpeed = minMagnitude;

        if (minimumSpeed > maxMagnitude) {
            minimumSpeed = maxMagnitude;
        }

        if (std::fabs(value) >= minimumSpeed) {
            return value;
        }

        return signSource >= 0 ? minimumSpeed : -minimumSpeed;
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

    void updatePositionAndTarget() {
        positionTracking.update_raw_pose();

        currentX = positionTracking.get_x();
        currentY = positionTracking.get_y();

        float xError = target.target_x - currentX;
        float yError = target.target_y - currentY;

        const float PI = 3.14159265f;
        targetHeading =
            normalizeHeading(std::atan2(xError, yError) * 180.0f / PI);

        if (config.drive_direction == DRIVE_TO_POINT_DRIVE_BACKWARD) {
            targetHeading = normalizeHeading(targetHeading + 180.0f);
        }

        distanceError = std::sqrt(xError * xError + yError * yError);
        headingErrorValue =
            getHeadingError(targetHeading, drivetrain.get_heading_degrees());
    }

    void resetStuckMonitor() {
        lastStuckCheckTime = master_timer.time(msec);
        lastDistanceError = distanceError;
        lastHeadingError = std::fabs(headingErrorValue);
        lastEncoderPosition = drivetrain.get_left_front_motor_position();
    }

    void startUnstuckForwardRight() {
        if (config.max_unstuck_attempts <= 0) {
            return;
        }

        if (unstuckAttemptCount >= config.max_unstuck_attempts) {
            currentState = DRIVE_TO_POINT_DONE;
            finished = true;
            drivetrain.set_drive_power(0, 0);
            return;
        }

        unstuckAttemptCount++;
        currentState = DRIVE_TO_POINT_UNSTUCK_FORWARD_RIGHT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void startUnstuckForwardLeft() {
        currentState = DRIVE_TO_POINT_UNSTUCK_FORWARD_LEFT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void startUnstuckBackRight() {
        currentState = DRIVE_TO_POINT_UNSTUCK_BACK_RIGHT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void startUnstuckBackLeft() {
        currentState = DRIVE_TO_POINT_UNSTUCK_BACK_LEFT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void applyUnstuckArc(float linearSpeed, float turnSpeed) {
        float leftPower = linearSpeed + turnSpeed;
        float rightPower = linearSpeed - turnSpeed;
        scaleDrivePower(leftPower, rightPower);
        drivetrain.set_drive_power(leftPower, rightPower);
    }

    void returnToPointing() {
        currentState = DRIVE_TO_POINT_POINTING;
        pointingTimerSet = false;
        linearPreviousError = distanceError;
        linearIntegralError = 0;
        angularPreviousError = headingErrorValue;
        angularIntegralError = 0;
        resetStuckMonitor();
    }

    void runUnstuckState() {
        float now = master_timer.time(msec);
        float elapsed = now - unstuckStartTime;

        if (currentState == DRIVE_TO_POINT_UNSTUCK_FORWARD_RIGHT) {
            applyUnstuckArc(config.forward_speed, config.turn_speed);

            if (elapsed >= config.max_turn_time) {
                startUnstuckForwardLeft();
            }

            return;
        }

        if (currentState == DRIVE_TO_POINT_UNSTUCK_FORWARD_LEFT) {
            applyUnstuckArc(config.forward_speed, -config.turn_speed);

            if (elapsed >= config.max_turn_time) {
                startUnstuckBackRight();
            }

            return;
        }

        if (currentState == DRIVE_TO_POINT_UNSTUCK_BACK_RIGHT) {
            applyUnstuckArc(-config.reverse_speed, config.turn_speed);

            if (elapsed >= config.max_reverse_time) {
                startUnstuckBackLeft();
            }

            return;
        }

        if (currentState == DRIVE_TO_POINT_UNSTUCK_BACK_LEFT) {
            applyUnstuckArc(-config.reverse_speed, -config.turn_speed);

            if (elapsed >= config.max_reverse_time) {
                returnToPointing();
            }

            return;
        }
    }

    bool robotMadeProgressWhileDriving(float linearSpeed) {
        if (config.max_unstuck_attempts <= 0 ||
            config.stuck_check_time <= 0 ||
            std::fabs(linearSpeed) < config.min_linear_speed) {

            return true;
        }

        float now = master_timer.time(msec);

        if (now - lastStuckCheckTime < config.stuck_check_time) {
            return true;
        }

        float currentEncoderPosition = drivetrain.get_left_front_motor_position();
        float distanceProgress = lastDistanceError - distanceError;
        float encoderChange = currentEncoderPosition - lastEncoderPosition;

        bool distanceImproved =
            distanceProgress >= config.stuck_distance_progress;

        bool encoderMoved =
            std::fabs(encoderChange) >= config.stuck_encoder_change_threshold;

        if (distanceImproved || encoderMoved) {
            lastStuckCheckTime = now;
            lastDistanceError = distanceError;
            lastEncoderPosition = currentEncoderPosition;
            return true;
        }

        return false;
    }

    bool robotMadeProgressWhilePointing(float angularSpeed) {
        if (config.max_unstuck_attempts <= 0 ||
            config.stuck_check_time <= 0 ||
            std::fabs(angularSpeed) < config.min_angular_speed) {

            return true;
        }

        float now = master_timer.time(msec);

        if (now - lastStuckCheckTime < config.stuck_check_time) {
            return true;
        }

        float headingProgress =
            lastHeadingError - std::fabs(headingErrorValue);

        bool headingImproved =
            headingProgress >= config.stuck_heading_progress;

        if (headingImproved) {
            lastStuckCheckTime = now;
            lastHeadingError = std::fabs(headingErrorValue);
            lastEncoderPosition = drivetrain.get_left_front_motor_position();
            return true;
        }

        return false;
    }

    bool updateSettledTimer(
        bool insideThreshold,
        bool& timerSet,
        float& timerEndTime,
        float settleTime,
        float now
    ) {
        if (insideThreshold) {
            if (!timerSet) {
                timerEndTime = now + settleTime;
                timerSet = true;
            }

            return now >= timerEndTime;
        }

        timerSet = false;
        return false;
    }

    float getAngularSpeed() {
        float angularDerivativeError = headingErrorValue - angularPreviousError;
        angularPreviousError = headingErrorValue;

        if (std::fabs(headingErrorValue) <= PID.angular_integral_windup_threshold) {
            angularIntegralError += headingErrorValue;
        } else {
            angularIntegralError = 0;
        }

        float angularSpeed =
            PID.angular_kP * headingErrorValue +
            PID.angular_kI * angularIntegralError +
            PID.angular_kD * angularDerivativeError;

        angularSpeed = clamp(angularSpeed, config.max_angular_speed);

        if (std::fabs(headingErrorValue) > config.heading_acceptable_error) {
            angularSpeed =
                applyMinimumSpeed(
                    angularSpeed,
                    config.min_angular_speed,
                    config.max_angular_speed,
                    headingErrorValue
                );
        }

        return clamp(angularSpeed, config.max_angular_speed);
    }

    float getLinearSpeed() {
        float linearDerivativeError = distanceError - linearPreviousError;
        linearPreviousError = distanceError;

        if (std::fabs(distanceError) <= PID.linear_integral_windup_threshold) {
            linearIntegralError += distanceError;
        } else {
            linearIntegralError = 0;
        }

        float linearSpeed =
            PID.linear_kP * distanceError +
            PID.linear_kI * linearIntegralError +
            PID.linear_kD * linearDerivativeError;

        linearSpeed = clamp(linearSpeed, config.max_linear_speed);

        if (distanceError > config.position_acceptable_error) {
            linearSpeed =
                applyMinimumSpeed(
                    linearSpeed,
                    config.min_linear_speed,
                    config.max_linear_speed,
                    distanceError
                );
        }

        linearSpeed = clamp(linearSpeed, config.max_linear_speed);

        if (config.drive_direction == DRIVE_TO_POINT_DRIVE_BACKWARD) {
            linearSpeed *= -1.0f;
        }

        return linearSpeed;
    }

    bool crossedExitLine() {
        if (target.exit_direction == DRIVE_TO_POINT_EXIT_ABOVE_Y) {
            return currentY >= target.exit_y;
        }

        return currentY <= target.exit_y;
    }

public:
    DriveToPointUntilYCommand(
        Drivetrain& drivetrain,
        PositionTracking& positionTracking,
        DriveToPointUntilYTarget target,
        DriveToPointConfig config,
        DrivePID PID
    )
        : drivetrain(drivetrain),
          positionTracking(positionTracking),
          target(target),
          config(config),
          PID(PID),
          currentState(DRIVE_TO_POINT_POINTING),
          finished(false),
          pointingTimerSet(false),
          timedOut(false),
          endTime(0),
          pointingSettleEndTime(0),
          linearPreviousError(0),
          linearIntegralError(0),
          angularPreviousError(0),
          angularIntegralError(0),
          currentX(0),
          currentY(0),
          targetHeading(0),
          distanceError(0),
          headingErrorValue(0),
          lastStuckCheckTime(0),
          lastDistanceError(0),
          lastHeadingError(0),
          lastEncoderPosition(0),
          unstuckStartTime(0),
          unstuckAttemptCount(0)
    {
    }

    void initialize() override {
        setCommandStatus("Drive To Point Until Y");
        currentState = DRIVE_TO_POINT_POINTING;

        finished = false;
        pointingTimerSet = false;
        timedOut = false;

        endTime = master_timer.time(msec) + config.max_time;
        pointingSettleEndTime = 0;

        linearPreviousError = 0;
        linearIntegralError = 0;
        angularPreviousError = 0;
        angularIntegralError = 0;

        currentX = 0;
        currentY = 0;
        targetHeading = 0;
        distanceError = 0;
        headingErrorValue = 0;
        lastStuckCheckTime = 0;
        lastDistanceError = 0;
        lastHeadingError = 0;
        lastEncoderPosition = 0;
        unstuckStartTime = 0;
        unstuckAttemptCount = 0;
    }

    void execute() override {
        float now = master_timer.time(msec);

        updatePositionAndTarget();

        if (lastStuckCheckTime == 0) {
            resetStuckMonitor();
        }

        if (now >= endTime) {
            timedOut = true;
            finished = true;
            drivetrain.set_drive_power(0, 0);
            return;
        }

        if (crossedExitLine()) {
            currentState = DRIVE_TO_POINT_DONE;
            finished = true;
            drivetrain.set_drive_power(0, 0);
            return;
        }

        if (currentState == DRIVE_TO_POINT_UNSTUCK_FORWARD_RIGHT ||
            currentState == DRIVE_TO_POINT_UNSTUCK_FORWARD_LEFT ||
            currentState == DRIVE_TO_POINT_UNSTUCK_BACK_RIGHT ||
            currentState == DRIVE_TO_POINT_UNSTUCK_BACK_LEFT) {

            runUnstuckState();
            return;
        }

        float angularSpeed = getAngularSpeed();

        if (currentState == DRIVE_TO_POINT_POINTING) {
            bool pointedAtTarget =
                updateSettledTimer(
                    std::fabs(headingErrorValue) <= config.heading_acceptable_error,
                    pointingTimerSet,
                    pointingSettleEndTime,
                    config.pointing_settle_time,
                    now
                );

            if (pointedAtTarget) {
                currentState = DRIVE_TO_POINT_DRIVING;
                pointingTimerSet = false;
                linearPreviousError = distanceError;
                linearIntegralError = 0;
                resetStuckMonitor();
                endTime = now + config.max_time;
            } else {
                if (!robotMadeProgressWhilePointing(angularSpeed)) {
                    startUnstuckForwardRight();
                    return;
                }

                drivetrain.set_drive_power(angularSpeed, -angularSpeed);
                return;
            }
        }

        if (currentState == DRIVE_TO_POINT_DRIVING) {
            float linearSpeed = getLinearSpeed();

            if (!robotMadeProgressWhileDriving(linearSpeed)) {
                startUnstuckForwardRight();
                return;
            }

            float leftPower = linearSpeed + angularSpeed;
            float rightPower = linearSpeed - angularSpeed;
            scaleDrivePower(leftPower, rightPower);

            drivetrain.set_drive_power(leftPower, rightPower);
            return;
        }

        drivetrain.set_drive_power(0, 0);
        finished = true;
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
        drivetrain.stop();
    }

    bool didTimeOut() const {
        return timedOut;
    }
};
