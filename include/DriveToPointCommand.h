#pragma once

#include "Command.h"
#include "Drivetrain.h"
#include "PositionTracking.h"
#include "RobotConfig.h"
#include "CommandStatus.h"
#include "RandomUnstuckOrder.h"

#include <cmath>

struct DriveToPointTarget {
    float target_x;
    float target_y;
};

enum DriveToPointDriveDirection {
    DRIVE_TO_POINT_DRIVE_FORWARD,
    DRIVE_TO_POINT_DRIVE_BACKWARD
};

struct DriveToPointConfig {
    float max_linear_speed;
    float max_angular_speed;
    float position_acceptable_error;
    float heading_acceptable_error;
    float max_time;
    float pointing_settle_time;
    float position_settle_time;
    float min_linear_speed;
    float min_angular_speed;
    float stuck_check_time;
    float stuck_distance_progress;
    float stuck_heading_progress;
    float stuck_encoder_change_threshold;
    float forward_speed;
    float reverse_speed;
    float turn_speed;
    float max_reverse_time;
    float max_turn_time;
    float unstuck_max_accel;
    int max_unstuck_attempts;
    DriveToPointDriveDirection drive_direction;
};

enum DriveToPointState {
    DRIVE_TO_POINT_POINTING,
    DRIVE_TO_POINT_DRIVING,
    DRIVE_TO_POINT_UNSTUCK_FORWARD_RIGHT,
    DRIVE_TO_POINT_UNSTUCK_FORWARD_LEFT,
    DRIVE_TO_POINT_UNSTUCK_BACK_RIGHT,
    DRIVE_TO_POINT_UNSTUCK_BACK_LEFT,
    DRIVE_TO_POINT_DONE
};

class DriveToPointCommand : public Command {
private:
    Drivetrain& drivetrain;
    PositionTracking& positionTracking;
    DriveToPointTarget target;
    DriveToPointConfig config;
    DrivePID PID;

    DriveToPointState currentState;

    bool finished;
    bool pointingTimerSet;
    bool positionTimerSet;
    bool timedOut;

    float endTime;
    float pointingSettleEndTime;
    float positionSettleEndTime;

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
    float lastRightEncoderPosition;
    float unstuckStartTime;
    int unstuckAttemptCount;
    int noProgressCheckCount;
    float unstuckPreviousLeftPower;
    float unstuckPreviousRightPower;
    float unstuckStageTime;
    bool unstuckStageAtSpeed;
    bool unstuckFinishing;
    RandomUnstuckOrder unstuckOrder;

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
        noProgressCheckCount = 0;
        lastStuckCheckTime = master_timer.time(msec);
        lastDistanceError = distanceError;
        lastHeadingError = std::fabs(headingErrorValue);
        lastEncoderPosition = drivetrain.get_left_front_motor_position();
        lastRightEncoderPosition = drivetrain.get_right_front_motor_position();
    }

    DriveToPointState stateFromUnstuckMove(int move) {
        if (move == RANDOM_UNSTUCK_FORWARD_RIGHT) {
            return DRIVE_TO_POINT_UNSTUCK_FORWARD_RIGHT;
        }

        if (move == RANDOM_UNSTUCK_FORWARD_LEFT) {
            return DRIVE_TO_POINT_UNSTUCK_FORWARD_LEFT;
        }

        if (move == RANDOM_UNSTUCK_BACK_RIGHT) {
            return DRIVE_TO_POINT_UNSTUCK_BACK_RIGHT;
        }

        return DRIVE_TO_POINT_UNSTUCK_BACK_LEFT;
    }

    unsigned int makeUnstuckSeed() {
        return
            static_cast<unsigned int>(master_timer.time(msec)) ^
            static_cast<unsigned int>(drivetrain.get_left_front_motor_position() * 31.0f) ^
            static_cast<unsigned int>(drivetrain.get_heading_degrees() * 17.0f) ^
            static_cast<unsigned int>(unstuckAttemptCount * 97);
    }

    void startCurrentUnstuckMovement() {
        setCommandStatus("Drive To Point Unstuck");
        currentState = stateFromUnstuckMove(unstuckOrder.current());
        unstuckStartTime = master_timer.time(msec);
        unstuckStageAtSpeed = false;
        resetStuckMonitor();
    }

    void startUnstuckLateralShift(
        bool wasMovingForward,
        float initialLeftPower,
        float initialRightPower
    ) {
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
        scaleDrivePower(initialLeftPower, initialRightPower);
        unstuckPreviousLeftPower = initialLeftPower;
        unstuckPreviousRightPower = initialRightPower;
        unstuckFinishing = false;
        unstuckStageTime =
            (wasMovingForward
                ? config.max_reverse_time
                : config.max_turn_time) *
            LATERAL_SHIFT_TIME_MULTIPLIER;
        unstuckOrder.resetLateralShift(makeUnstuckSeed(), wasMovingForward);
        startCurrentUnstuckMovement();
    }

    bool applyUnstuckArc(float linearSpeed, float turnSpeed) {
        float targetLeftPower = linearSpeed + turnSpeed;
        float targetRightPower = linearSpeed - turnSpeed;
        scaleDrivePower(targetLeftPower, targetRightPower);

        float leftPower = targetLeftPower;
        float rightPower = targetRightPower;

        if (config.unstuck_max_accel > 0.0f) {
            float maxChange = config.unstuck_max_accel / 100.0f;

            if (leftPower > unstuckPreviousLeftPower + maxChange) {
                leftPower = unstuckPreviousLeftPower + maxChange;
            } else if (leftPower < unstuckPreviousLeftPower - maxChange) {
                leftPower = unstuckPreviousLeftPower - maxChange;
            }

            if (rightPower > unstuckPreviousRightPower + maxChange) {
                rightPower = unstuckPreviousRightPower + maxChange;
            } else if (rightPower < unstuckPreviousRightPower - maxChange) {
                rightPower = unstuckPreviousRightPower - maxChange;
            }
        }

        drivetrain.set_drive_power(leftPower, rightPower);
        unstuckPreviousLeftPower = leftPower;
        unstuckPreviousRightPower = rightPower;

        return
            std::fabs(leftPower - targetLeftPower) < 0.01f &&
            std::fabs(rightPower - targetRightPower) < 0.01f;
    }

    void returnToPointing() {
        setCommandStatus("Drive To Point");
        currentState = DRIVE_TO_POINT_POINTING;
        pointingTimerSet = false;
        positionTimerSet = false;
        linearPreviousError = distanceError;
        linearIntegralError = 0;
        angularPreviousError = headingErrorValue;
        angularIntegralError = 0;
        unstuckPreviousLeftPower = 0;
        unstuckPreviousRightPower = 0;
        unstuckStageAtSpeed = false;
        unstuckFinishing = false;
        resetStuckMonitor();
    }

    void runUnstuckState() {
        float now = master_timer.time(msec);
        float elapsed = now - unstuckStartTime;
        bool stageAtSpeed = false;

        if (unstuckFinishing) {
            if (applyUnstuckArc(0.0f, 0.0f)) {
                returnToPointing();
            }
            return;
        }

        if (currentState == DRIVE_TO_POINT_UNSTUCK_FORWARD_RIGHT) {
            stageAtSpeed = applyUnstuckArc(config.forward_speed, config.turn_speed);
        }
        else if (currentState == DRIVE_TO_POINT_UNSTUCK_FORWARD_LEFT) {
            stageAtSpeed = applyUnstuckArc(config.forward_speed, -config.turn_speed);
        }
        else if (currentState == DRIVE_TO_POINT_UNSTUCK_BACK_RIGHT) {
            stageAtSpeed = applyUnstuckArc(-config.reverse_speed, config.turn_speed);
        }
        else if (currentState == DRIVE_TO_POINT_UNSTUCK_BACK_LEFT) {
            stageAtSpeed = applyUnstuckArc(-config.reverse_speed, -config.turn_speed);
        }

        if (!unstuckStageAtSpeed) {
            if (stageAtSpeed) {
                unstuckStageAtSpeed = true;
                unstuckStartTime = now;
            }
            return;
        }

        if (elapsed >= unstuckStageTime) {
            if (unstuckOrder.advance()) {
                startCurrentUnstuckMovement();
            } else {
                unstuckFinishing = true;
            }
        }
    }

    bool robotMadeProgressWhileDriving(float linearSpeed) {
        if (config.max_unstuck_attempts <= 0 ||
            config.stuck_check_time <= 0) {

            return true;
        }

        if (std::fabs(linearSpeed) < config.min_linear_speed) {
            resetStuckMonitor();
            return true;
        }

        float now = master_timer.time(msec);

        if (now - lastStuckCheckTime < config.stuck_check_time) {
            return true;
        }

        float currentEncoderPosition = drivetrain.get_left_front_motor_position();
        float currentRightEncoderPosition =
            drivetrain.get_right_front_motor_position();
        float distanceProgress = lastDistanceError - distanceError;
        bool distanceImproved =
            distanceProgress >= config.stuck_distance_progress;

        // Wheel motion alone does not prove the robot moved across the field.
        // A blocked drivetrain can spin its wheels and otherwise suppress unstuck forever.
        if (distanceImproved) {
            unstuckAttemptCount = 0;
            resetStuckMonitor();
            return true;
        }

        noProgressCheckCount++;

        if (noProgressCheckCount < REQUIRED_CONSECUTIVE_STUCK_CHECKS) {
            lastStuckCheckTime = now;
            lastDistanceError = distanceError;
            lastEncoderPosition = currentEncoderPosition;
            lastRightEncoderPosition = currentRightEncoderPosition;
            return true;
        }

        return false;
    }

    bool robotMadeProgressWhilePointing(float angularSpeed) {
        if (config.max_unstuck_attempts <= 0 ||
            config.stuck_check_time <= 0) {

            return true;
        }

        if (std::fabs(angularSpeed) < config.min_angular_speed) {
            resetStuckMonitor();
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
            unstuckAttemptCount = 0;
            resetStuckMonitor();
            return true;
        }

        noProgressCheckCount++;

        if (noProgressCheckCount < REQUIRED_CONSECUTIVE_STUCK_CHECKS) {
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

public:
    DriveToPointCommand(
        Drivetrain& drivetrain,
        PositionTracking& positionTracking,
        DriveToPointTarget target,
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
          positionTimerSet(false),
          timedOut(false),
          endTime(0),
          pointingSettleEndTime(0),
          positionSettleEndTime(0),
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
          lastRightEncoderPosition(0),
          unstuckStartTime(0),
          unstuckAttemptCount(0),
          noProgressCheckCount(0),
          unstuckPreviousLeftPower(0),
          unstuckPreviousRightPower(0),
          unstuckStageTime(0),
          unstuckStageAtSpeed(false),
          unstuckFinishing(false),
          unstuckOrder()
    {
    }

    void initialize() override {
        setCommandStatus("Drive To Point");
        currentState = DRIVE_TO_POINT_POINTING;

        finished = false;
        pointingTimerSet = false;
        positionTimerSet = false;
        timedOut = false;

        endTime = master_timer.time(msec) + config.max_time;
        pointingSettleEndTime = 0;
        positionSettleEndTime = 0;

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
        lastRightEncoderPosition = 0;
        unstuckStartTime = 0;
        unstuckAttemptCount = 0;
        noProgressCheckCount = 0;
        unstuckPreviousLeftPower = 0;
        unstuckPreviousRightPower = 0;
        unstuckStageTime = 0;
        unstuckStageAtSpeed = false;
        unstuckFinishing = false;
    }

    void execute() override {
        float now = master_timer.time(msec);

        updatePositionAndTarget();

        if (lastStuckCheckTime == 0) {
            resetStuckMonitor();
        }

        if (currentState == DRIVE_TO_POINT_UNSTUCK_FORWARD_RIGHT ||
            currentState == DRIVE_TO_POINT_UNSTUCK_FORWARD_LEFT ||
            currentState == DRIVE_TO_POINT_UNSTUCK_BACK_RIGHT ||
            currentState == DRIVE_TO_POINT_UNSTUCK_BACK_LEFT) {

            runUnstuckState();
            return;
        }

        if (now >= endTime) {
            timedOut = true;
            finished = true;
            drivetrain.set_drive_power(0, 0);
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
                positionTimerSet = false;
                linearPreviousError = distanceError;
                linearIntegralError = 0;
                resetStuckMonitor();
                endTime = now + config.max_time;
            } else {
                if (!robotMadeProgressWhilePointing(angularSpeed)) {
                    startUnstuckLateralShift(
                        config.drive_direction == DRIVE_TO_POINT_DRIVE_FORWARD,
                        angularSpeed,
                        -angularSpeed
                    );
                    return;
                }

                drivetrain.set_drive_power(angularSpeed, -angularSpeed);
                return;
            }
        }

        if (currentState == DRIVE_TO_POINT_DRIVING) {
            bool atPosition =
                updateSettledTimer(
                    distanceError <= config.position_acceptable_error,
                    positionTimerSet,
                    positionSettleEndTime,
                    config.position_settle_time,
                    now
                );

            if (atPosition) {
                currentState = DRIVE_TO_POINT_DONE;
                finished = true;
                drivetrain.set_drive_power(0, 0);
                return;
            }

            float linearSpeed = getLinearSpeed();
            float leftPower = linearSpeed + angularSpeed;
            float rightPower = linearSpeed - angularSpeed;
            scaleDrivePower(leftPower, rightPower);

            if (!robotMadeProgressWhileDriving(linearSpeed)) {
                startUnstuckLateralShift(
                    linearSpeed >= 0.0f,
                    leftPower,
                    rightPower
                );
                return;
            }

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
