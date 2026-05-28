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
    float maxTurnTime;                     // ms

    float minLinearSpeedForStuckCheck;
    float minYErrorProgress;
    

    // Avoid-zone settings
    FieldAvoidZone avoidZones[3];
};

enum TrackBlockState {
    TRACKING_NORMAL,
    TRACKING_UNSTUCK_REVERSE,
    TRACKING_UNSTUCK_FORWARD,
    TRACKING_UNSTUCK_TURN_LEFT,
    TRACKING_UNSTUCK_TURN_RIGHT,
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

    // Stuck detection variables
    float lastStuckCheckTime;
    float unstuckStartTime;

    float lastHeading;
    float lastEncoderPosition;

    // Avoid-zone variables
    float estimatedBlockDistance;
    float estimatedBlockFieldX;
    float estimatedBlockFieldY;

    int lastProgressYError;

private:
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

    void estimateBlockFieldPosition(float blockDistance) {
        Pose robotPose = drivetrain.get_pose();

        float robotX = robotPose.x;
        float robotY = robotPose.y;

        float robotHeadingDeg = drivetrain.get_heading_degrees();

        const float PI = 3.14159265f;
        float robotHeadingRad = robotHeadingDeg * PI / 180.0f;

        /*
            Assumption:
            heading 0 degrees = field +Y
            heading 90 degrees = field +X

            Since we are assuming the block is centered in front of the robot:
            relative block X = 0
            relative block Y = blockDistance
        */

        estimatedBlockFieldX =
            robotX + blockDistance * std::sin(robotHeadingRad);

        estimatedBlockFieldY =
            robotY + blockDistance * std::cos(robotHeadingRad);
    }

    bool pointInsideAvoidZone(float pointX, float pointY, const FieldAvoidZone& zone) {
        if (!zone.enabled) {
            return false;
        }

        bool insideX =
            pointX >= zone.minX &&
            pointX <= zone.maxX;

        bool insideY =
            pointY >= zone.minY &&
            pointY <= zone.maxY;

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
        /*
            This uses the block's pixel y-position to estimate distance.
            If your JetsonSerial does not have getYPos(), either add it
            or replace jetson.getYPos() with jetson.block_y_pos.
        */

        estimatedBlockDistance =
            estimateBlockDistanceFromPixelY(jetson.block_y_pos);

        estimateBlockFieldPosition(estimatedBlockDistance);

        return blockIsInsideAnyAvoidZone();
    }

    void resetStuckMonitor() {
            float now = master_timer.time(msec);

            lastStuckCheckTime = now;

            lastHeading = drivetrain.get_heading_degrees();
            lastEncoderPosition = drivetrain.get_left_front_motor_position();

            lastProgressYError = yError;
    }

    void startUnstuckReverse() {
        currentState = TRACKING_UNSTUCK_REVERSE;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void startUnstuckForward() {
        currentState = TRACKING_UNSTUCK_FORWARD;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void startUnstuckTurnLeft() {
        currentState = TRACKING_UNSTUCK_TURN_LEFT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
    }

    void startUnstuckTurnRight() {
        currentState = TRACKING_UNSTUCK_TURN_RIGHT;
        unstuckStartTime = master_timer.time(msec);
        resetStuckMonitor();
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

        if (currentState == TRACKING_UNSTUCK_REVERSE) {
            drivetrain.set_drive_power(-config.reverseSpeed, -config.reverseSpeed);

            if (elapsed >= config.maxReverseTime) {
                startUnstuckForward();
            }

            return;
        }

        if (currentState == TRACKING_UNSTUCK_FORWARD) {
            drivetrain.set_drive_power(config.forwardSpeed, config.forwardSpeed);

            if (elapsed >= config.maxForwardTime) {
                startUnstuckTurnLeft();
            }

            return;
        }

        if (currentState == TRACKING_UNSTUCK_TURN_LEFT) {
            drivetrain.set_drive_power(-config.turnSpeed, config.turnSpeed);

            if (elapsed >= config.maxTurnTime) {
                startUnstuckTurnRight();
            }

            return;
        }

        if (currentState == TRACKING_UNSTUCK_TURN_RIGHT) {
            drivetrain.set_drive_power(config.turnSpeed, -config.turnSpeed);

            if (elapsed >= config.maxTurnTime) {
                returnToNormalTracking();
            }

            return;
        }
    }

public:
    TrackBlockCommand(
        Drivetrain& drivetrain,
        JetsonSerial& jetson,
        const TrackingBlocksConfig& config,
        const TrackBlockRawConfig& trackingConfig,
        const DrivePID& PID
    )
        : drivetrain(drivetrain),
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
          lastStuckCheckTime(0),
          unstuckStartTime(0),
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
            drivetrain.set_drive_power(0, 0);
            result = TRACK_BLOCK_LOST;
            finished = true;
            return;
        }

        if (shouldAvoidCurrentBlock()) {
            drivetrain.set_drive_power(0, 0);
            result = TRACK_BLOCK_AVOIDED;
            finished = true;
            return;
        }

      

        xError = jetson.getXError();
        xDerivativeError = xError - xPrevError;

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
            startUnstuckReverse();
            return;
}

        drivetrain.set_drive_power(
            linearSpeed + angularSpeed,
            linearSpeed - angularSpeed
        );

        xCentered =
            std::abs(xError) <= config.acceptableXError;

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