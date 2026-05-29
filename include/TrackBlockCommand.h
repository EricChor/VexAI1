// #pragma once

// #include "Command.h"
// #include "JetsonSerial.h"
// #include "Drivetrain.h"

// struct TrackingBlocksConfig{
//     int acceptableYError;
//     int acceptableXError;

//     float maxLinearSpeed;
//     float maxAngularSpeed;
// };

// class TrackBlockCommand : public Command{
//     private:
//         Drivetrain& drivetrain;
//         JetsonSerial& jetson;

//         TrackBlockRawConfig trackingConfig;
//         TrackingBlocksConfig config;

//         DrivePID PID;

//         bool finished;

//         bool trackingValid;

//         int xError;
//         int xPrevError;
//         int xDerivativeError;

//         int yError;
//         int yPrevError;
//         int yDerivativeError;

//         float xIntegralError;
//         float yIntegralError;

//         float linearSpeed;
//         float angularSpeed;

//         bool xCentered;
//         bool yCloseEnough;

//     public:
//         TrackBlockCommand(Drivetrain& drivetrain, JetsonSerial& jetson, TrackingBlocksConfig config, TrackBlockRawConfig trackingConfig, DrivePID PID)
//         :drivetrain(drivetrain),
//          jetson(jetson),
//          config(config),
//          trackingConfig(trackingConfig),
//          PID(PID)
//         {}

//         void initialize() override{
//             finished = false;
//             trackingValid = false;

//             xError = 0;
//             xPrevError = 0;
//             xDerivativeError = 0;

//             yError = 0;
//             yPrevError = 0;
//             yDerivativeError = 0;

//             xIntegralError = 0;
//             yIntegralError = 0;

//             linearSpeed = 0;
//             angularSpeed = 0;
            
//             xCentered = false;
//             yCloseEnough = false;

//             jetson.track_block_raw_init(trackingConfig);
//         }

//         void execute() override{
//             trackingValid = jetson.track_block_raw_step();

//             if(!trackingValid){
//                 drivetrain.set_drive_power(0,0);
//                 finished = true;
//                 return;
//             }

//             xError = jetson.getXError();
//             xDerivativeError = xError - xPrevError;
//             if(abs(xError) <= PID.angular_integral_windup_threshold){
//                 xIntegralError += xError;
//             } else {
//                 xIntegralError = 0;
//             }


//             angularSpeed = PID.angular_kP * xError + PID.angular_kI * xIntegralError + PID.angular_kD * xDerivativeError;
//             if(fabs(angularSpeed) > config.maxAngularSpeed){
//                 if(angularSpeed > 0){
//                     angularSpeed = config.maxAngularSpeed;
//                 } else {
//                     angularSpeed = -config.maxAngularSpeed;
//                 }
//             }


//             yError = jetson.getYError();
//             yDerivativeError = yError - yPrevError;
//             if(abs(yError) <= PID.linear_integral_windup_threshold){
//                 yIntegralError += yError;
//             } else {
//                 yIntegralError = 0;
//             }

//             linearSpeed = PID.linear_kP * yError + PID.linear_kI * yIntegralError + PID.linear_kD * yDerivativeError;

//             if(fabs(linearSpeed) > config.maxLinearSpeed){
//                 if(linearSpeed > 0){
//                     linearSpeed = config.maxLinearSpeed;
//                 } else {
//                     linearSpeed = -config.maxLinearSpeed;
//                 }
//             }

//             xPrevError = xError;
//             yPrevError = yError;

//             drivetrain.set_drive_power(linearSpeed + angularSpeed, linearSpeed - angularSpeed);

//             xCentered = abs(xError) <= config.acceptableXError;
//             yCloseEnough = abs(yError) <= config.acceptableYError;

//             if(xCentered && yCloseEnough){
//                 drivetrain.set_drive_power(0,0);
//                 finished = true;
//             }

//         }

//         bool isFinished() override{
//             return finished;
//         }

//         void end() override{
//             drivetrain.set_drive_power(0,0);
//         }
// };

#pragma once

#include "Command.h"
#include "JetsonSerial.h"
#include "Drivetrain.h"
#include "RobotConfig.h"
#include "PositionTracking.h"
#include "FieldAvoidZone.h"

#include <cmath>

struct TrackingBlocksConfig {
    int acceptableYError;
    int acceptableXError;

    float maxLinearSpeed;
    float maxAngularSpeed;

    // Anti-stuck settings
    float stuckCheckTime;                  // ms
    float stuckEncoderChangeThreshold;     // motor degrees
    float stuckHeadingChangeThreshold;     // degrees

    float reverseSpeed;
    float forwardSpeed;
    float turnSpeed;

    float maxReverseTime;                  // ms
    float maxForwardTime;                  // ms

    float minLinearSpeedForStuckCheck;
    float minYErrorProgress;

    // How many raw tracking failures to tolerate before giving up.
    int maxTrackingDroppedFrames;

    // How many full unstuck escape cycles to try before giving up.
    int maxUnstuckAttempts;

    // Used to estimate a block's field position before it is perfectly centered.
    float cameraHorizontalFovDegrees;

    // Avoid-zone settings
    FieldAvoidZone avoidZones[3];
};

enum TrackBlockState {
    TRACKING_NORMAL,
    TRACKING_UNSTUCK_FORWARD_RIGHT,
    TRACKING_UNSTUCK_FORWARD_LEFT,
    TRACKING_UNSTUCK_BACK_RIGHT,
    TRACKING_UNSTUCK_BACK_LEFT,
    TRACK_BLOCK_DONE
};

enum TrackBlockResult {
    TRACK_BLOCK_RUNNING,
    TRACK_BLOCK_SUCCESS,
    TRACK_BLOCK_LOST,
    TRACK_BLOCK_AVOIDED,
    TRACK_BLOCK_STUCK_FAILED
};

class TrackBlockCommand : public Command {
private:
    Drivetrain& drivetrain;
    PositionTracking& positionTracking;
    JetsonSerial& jetson;

    TrackBlockRawConfig trackingConfig;
    TrackingBlocksConfig config;

    DrivePID PID;

    bool finished;
    bool trackingValid;

    TrackBlockState currentState;
    TrackBlockResult result;

    int xError;
    int xPrevError;
    int xDerivativeError;

    int yError;
    int yPrevError;
    int yDerivativeError;

    float xIntegralError;
    float yIntegralError;

    float linearSpeed;
    float angularSpeed;

    bool xCentered;
    bool yCloseEnough;
    int droppedTrackingFrameCount;

    // Stuck detection variables
    float lastStuckCheckTime;
    float unstuckStartTime;
    int unstuckAttemptCount;

    float lastHeading;
    float lastEncoderPosition;

    // Avoid-zone variables
    float estimatedBlockDistance;
    float estimatedBlockFieldX;
    float estimatedBlockFieldY;

    int lastProgressYError;

private:
    void failStuck() {
        drivetrain.set_drive_power(0, 0);
        result = TRACK_BLOCK_STUCK_FAILED;
        currentState = TRACK_BLOCK_DONE;
        finished = true;
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

    float estimateBlockDistanceFromPixelY(float pixelY) {
        return
            (1.76136e-9f * pixelY * pixelY * pixelY * pixelY)
            - (0.00000217361f * pixelY * pixelY * pixelY)
            + (0.00106188f * pixelY * pixelY)
            - (0.30024f * pixelY)
            + 53.72545f;
    }

    void estimateBlockFieldPosition(float blockDistance, float pixelX) {
        positionTracking.update_raw_pose();

        float robotX = positionTracking.get_x();
        float robotY = positionTracking.get_y();
        float robotHeadingDeg = positionTracking.get_heading();

        const float PI = 3.14159265f;
        float blockBearingDeg = 0.0f;

        if (config.cameraHorizontalFovDegrees > 0.0f &&
            trackingConfig.cameraCenterX > 0) {

            float horizontalError = pixelX - trackingConfig.cameraCenterX;
            float normalizedError =
                horizontalError / static_cast<float>(trackingConfig.cameraCenterX);

            blockBearingDeg =
                normalizedError * config.cameraHorizontalFovDegrees / 2.0f;
        }

        float blockFieldHeadingRad =
            (robotHeadingDeg + blockBearingDeg) * PI / 180.0f;

        /*
            Assumption:
            heading 0 degrees = field +Y
            heading 90 degrees = field +X

            The block's pixel x-position gives a rough bearing offset from
            the robot's centerline. This keeps avoid checks useful before
            the block is perfectly centered.
        */

        estimatedBlockFieldX =
            robotX + blockDistance * std::sin(blockFieldHeadingRad);

        estimatedBlockFieldY =
            robotY + blockDistance * std::cos(blockFieldHeadingRad);
    }

    bool pointInsideAvoidZone(float pointX, float pointY, const FieldAvoidZone& zone) {
        if (!zone.enabled) {
            return false;
        }

        float lowX = zone.minX < zone.maxX ? zone.minX : zone.maxX;
        float highX = zone.minX < zone.maxX ? zone.maxX : zone.minX;
        float lowY = zone.minY < zone.maxY ? zone.minY : zone.maxY;
        float highY = zone.minY < zone.maxY ? zone.maxY : zone.minY;

        bool insideX =
            pointX >= lowX &&
            pointX <= highX;

        bool insideY =
            pointY >= lowY &&
            pointY <= highY;

        return insideX && insideY;
    }

    bool blockIsInsideAnyAvoidZone() {
        for (int i = 0; i < 3; i++) {
            if (pointInsideAvoidZone(
                    estimatedBlockFieldX,
                    estimatedBlockFieldY,
                    config.avoidZones[i]
                )) {
                return true;
            }
        }

        return false;
    }

    bool shouldAvoidCurrentBlock() {
        if (jetson.block_x_pos < 0 || jetson.block_y_pos < 0) {
            return false;
        }

        estimatedBlockDistance =
            estimateBlockDistanceFromPixelY(jetson.block_y_pos);

        estimateBlockFieldPosition(
            estimatedBlockDistance,
            jetson.block_x_pos
        );

        return blockIsInsideAnyAvoidZone();
    }

    void resetStuckMonitor() {
        float now = master_timer.time(msec);

        lastStuckCheckTime = now;

        lastHeading = drivetrain.get_heading_degrees();
        lastEncoderPosition = drivetrain.get_left_front_motor_position();

        lastProgressYError = yError;
    }

    void startUnstuckForwardRight() {
        if (unstuckAttemptCount >= config.maxUnstuckAttempts) {
            failStuck();
            return;
        }

        unstuckAttemptCount++;
        currentState = TRACKING_UNSTUCK_FORWARD_RIGHT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void startUnstuckForwardLeft() {
        currentState = TRACKING_UNSTUCK_FORWARD_LEFT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void startUnstuckBackRight() {
        currentState = TRACKING_UNSTUCK_BACK_RIGHT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void startUnstuckBackLeft() {
        currentState = TRACKING_UNSTUCK_BACK_LEFT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void applyUnstuckArc(float linearSpeed, float turnSpeed) {
        drivetrain.set_drive_power(
            linearSpeed + turnSpeed,
            linearSpeed - turnSpeed
        );
    }

    void returnToNormalTracking() {
        currentState = TRACKING_NORMAL;
        resetStuckMonitor();
    }

    bool robotMadeProgressWhileTracking() {
        float now = master_timer.time(msec);

        if (now - lastStuckCheckTime < config.stuckCheckTime) {
            return true;
        }

        float currentEncoderPosition =
            drivetrain.get_left_front_motor_position();

        float currentHeading =
            drivetrain.get_heading_degrees();

        float encoderChange =
            currentEncoderPosition - lastEncoderPosition;

        float headingChange =
            getHeadingChange(currentHeading, lastHeading);

        int yErrorChange =
            yError - lastProgressYError;

        bool encoderMoved =
            std::fabs(encoderChange) >= config.stuckEncoderChangeThreshold;

        bool headingMoved =
            std::fabs(headingChange) >= config.stuckHeadingChangeThreshold;

        /*
            For tracking, this is the most important one:
            if the robot is actually moving toward the block,
            yError should change.
        */
        bool visionYChanged =
            std::abs(yErrorChange) >= config.minYErrorProgress;

        if (encoderMoved || headingMoved || visionYChanged) {
            lastStuckCheckTime = now;
            lastEncoderPosition = currentEncoderPosition;
            lastHeading = currentHeading;
            lastProgressYError = yError;

            return true;
        }

        return false;
    }

    void runUnstuckState() {
        float now = master_timer.time(msec);
        float elapsed = now - unstuckStartTime;

        if (currentState == TRACKING_UNSTUCK_FORWARD_RIGHT) {
            applyUnstuckArc(config.forwardSpeed, config.turnSpeed);

            if (elapsed >= config.maxForwardTime) {
                startUnstuckForwardLeft();
            }

            return;
        }

        if (currentState == TRACKING_UNSTUCK_FORWARD_LEFT) {
            applyUnstuckArc(config.forwardSpeed, -config.turnSpeed);

            if (elapsed >= config.maxForwardTime) {
                startUnstuckBackRight();
            }

            return;
        }

        if (currentState == TRACKING_UNSTUCK_BACK_RIGHT) {
            applyUnstuckArc(-config.reverseSpeed, config.turnSpeed);

            if (elapsed >= config.maxReverseTime) {
                startUnstuckBackLeft();
            }

            return;
        }

        if (currentState == TRACKING_UNSTUCK_BACK_LEFT) {
            applyUnstuckArc(-config.reverseSpeed, -config.turnSpeed);

            if (elapsed >= config.maxReverseTime) {
                returnToNormalTracking();
            }

            return;
        }
    }

public:
    TrackBlockCommand(
        Drivetrain& drivetrain,
        PositionTracking& positionTracking,
        JetsonSerial& jetson,
        const TrackingBlocksConfig& config,
        const TrackBlockRawConfig& trackingConfig,
        const DrivePID& PID
    )
        : drivetrain(drivetrain),
          positionTracking(positionTracking),
          jetson(jetson),
          config(config),
          trackingConfig(trackingConfig),
          PID(PID),
          finished(false),
          trackingValid(false),
          currentState(TRACKING_NORMAL),
          result(TRACK_BLOCK_RUNNING),
          xError(0),
          xPrevError(0),
          xDerivativeError(0),
          yError(0),
          yPrevError(0),
          yDerivativeError(0),
          xIntegralError(0),
          yIntegralError(0),
          linearSpeed(0),
          angularSpeed(0),
          xCentered(false),
          yCloseEnough(false),
          droppedTrackingFrameCount(0),
          lastStuckCheckTime(0),
          unstuckStartTime(0),
          unstuckAttemptCount(0),
          lastHeading(0),
          lastEncoderPosition(0),
          estimatedBlockDistance(0),
          estimatedBlockFieldX(0),
          estimatedBlockFieldY(0),
          lastProgressYError(0)
    {
    }

    void initialize() override {
        finished = false;
        trackingValid = false;

        currentState = TRACKING_NORMAL;
        result = TRACK_BLOCK_RUNNING;

        xError = 0;
        xPrevError = 0;
        xDerivativeError = 0;

        yError = 0;
        yPrevError = 0;
        yDerivativeError = 0;

        xIntegralError = 0;
        yIntegralError = 0;

        linearSpeed = 0;
        angularSpeed = 0;

        xCentered = false;
        yCloseEnough = false;
        droppedTrackingFrameCount = 0;
        unstuckAttemptCount = 0;

        estimatedBlockDistance = 0;
        estimatedBlockFieldX = 0;
        estimatedBlockFieldY = 0;

        lastProgressYError = 0;

        jetson.track_block_raw_init(trackingConfig);

        resetStuckMonitor();
    }

    void execute() override {
        if (currentState == TRACK_BLOCK_DONE) {
            drivetrain.set_drive_power(0, 0);
            finished = true;
            return;
        }

        if (currentState != TRACKING_NORMAL) {
            runUnstuckState();
            return;
        }

        trackingValid = jetson.track_block_raw_step();

        if (!trackingValid) {
            droppedTrackingFrameCount++;
            drivetrain.set_drive_power(0, 0);

            if (droppedTrackingFrameCount <= config.maxTrackingDroppedFrames) {
                resetStuckMonitor();
                return;
            }

            result = TRACK_BLOCK_LOST;
            finished = true;
            return;
        }

        droppedTrackingFrameCount = 0;

        xError = jetson.getXError();
        xDerivativeError = xError - xPrevError;

        xCentered =
            std::abs(xError) <= config.acceptableXError;

        if (shouldAvoidCurrentBlock()) {
            drivetrain.set_drive_power(0, 0);
            result = TRACK_BLOCK_AVOIDED;
            finished = true;
            return;
        }

        if (std::abs(xError) <= PID.angular_integral_windup_threshold) {
            xIntegralError += xError;
        } else {
            xIntegralError = 0;
        }

        angularSpeed =
            PID.angular_kP * xError +
            PID.angular_kI * xIntegralError +
            PID.angular_kD * xDerivativeError;

        if (std::fabs(angularSpeed) > config.maxAngularSpeed) {
            if (angularSpeed > 0) {
                angularSpeed = config.maxAngularSpeed;
            } else {
                angularSpeed = -config.maxAngularSpeed;
            }
        }

        yError = jetson.getYError();
        yDerivativeError = yError - yPrevError;

        if (std::abs(yError) <= PID.linear_integral_windup_threshold) {
            yIntegralError += yError;
        } else {
            yIntegralError = 0;
        }

        linearSpeed =
            PID.linear_kP * yError +
            PID.linear_kI * yIntegralError +
            PID.linear_kD * yDerivativeError;

        if (std::fabs(linearSpeed) > config.maxLinearSpeed) {
            if (linearSpeed > 0) {
                linearSpeed = config.maxLinearSpeed;
            } else {
                linearSpeed = -config.maxLinearSpeed;
            }
        }

        xPrevError = xError;
        yPrevError = yError;

        bool tryingToDrive = std::fabs(linearSpeed) >= config.minLinearSpeedForStuckCheck;

        if (tryingToDrive && !robotMadeProgressWhileTracking()) {
            startUnstuckForwardRight();
            return;
        }

        drivetrain.set_drive_power(
            linearSpeed + angularSpeed,
            linearSpeed - angularSpeed
        );

        yCloseEnough =
            std::abs(yError) <= config.acceptableYError;

        if (xCentered && yCloseEnough) {
            drivetrain.set_drive_power(0, 0);
            result = TRACK_BLOCK_SUCCESS;
            currentState = TRACK_BLOCK_DONE;
            finished = true;
        }
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
        drivetrain.set_drive_power(0, 0);
    }

    TrackBlockResult getResult() const {
        return result;
    }

    bool wasSuccessful() const {
        return result == TRACK_BLOCK_SUCCESS;
    }

    bool wasAvoided() const {
        return result == TRACK_BLOCK_AVOIDED;
    }

    bool lostTracking() const {
        return result == TRACK_BLOCK_LOST;
    }
};
