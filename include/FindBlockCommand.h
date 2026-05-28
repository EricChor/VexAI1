// #pragma once

// #include "Command.h"
// #include "JetsonSerial.h"
// #include "RobotConfig.h"
// #include "Drivetrain.h"

// #include <cmath>

// struct findingBlocksConfig {
//     float searchSpeed;
//     float maxSpeed;
//     int centeringAcceptableX;

//     // Anti-stuck movement speeds
//     float reverseSpeed;
//     float forwardSpeed;

//     // How often to check if the robot is stuck, in milliseconds
//     float stuckCheckTime;

//     // Minimum heading change needed while turning
//     float stuckHeadingChangeThreshold;

//     // Minimum encoder change needed while driving forward/reverse
//     float stuckEncoderChangeThreshold;

//     // Maximum time to reverse/drive forward, in milliseconds
//     float maxReverseTime;
//     float maxForwardTime;
// };

// enum findingBlockState {
//     SEARCHING_FOR_BLOCK,
//     CENTERING_BLOCK,
//     FIND_BLOCK_DONE
// };

// enum searchMovementState {
//     SEARCH_TURN_RIGHT,
//     SEARCH_TURN_LEFT,
//     SEARCH_REVERSE,
//     SEARCH_FORWARD
// };

// class FindBlockCommand : public Command {
// private:
//     Drivetrain& drivetrain;
//     JetsonSerial& jetson;

//     FindBlockRawConfig findingConfig;
//     TrackBlockRawConfig trackingConfig;

//     DrivePID PID;

//     findingBlockState currentState;
//     searchMovementState currentSearchMovement;

//     findingBlocksConfig config;

//     bool finished;
//     bool foundBlock;
//     bool trackingValid;

//     int xError;
//     int xPrevError;
//     int xDerivativeError;

//     float angularSpeed;

//     // Anti-stuck variables
//     float lastStuckCheckTime;
//     float movementStartTime;

//     float lastHeading;
//     float lastEncoderPosition;

// private:
//     float getHeadingChange(float currentHeading, float previousHeading) {
//         float change = currentHeading - previousHeading;

//         while (change > 180.0f) {
//             change -= 360.0f;
//         }

//         while (change < -180.0f) {
//             change += 360.0f;
//         }

//         return change;
//     }

//     void resetStuckMonitor() {
//         float now = master_timer.time(msec);

//         lastStuckCheckTime = now;
//         movementStartTime = now;

//         lastHeading = drivetrain.get_heading_degrees();
//         lastEncoderPosition = drivetrain.get_left_front_motor_position();
//     }

//     void goToNextSearchMovement() {
//         if (currentSearchMovement == SEARCH_TURN_RIGHT) {
//             currentSearchMovement = SEARCH_TURN_LEFT;
//         }
//         else if (currentSearchMovement == SEARCH_TURN_LEFT) {
//             currentSearchMovement = SEARCH_REVERSE;
//         }
//         else if (currentSearchMovement == SEARCH_REVERSE) {
//             currentSearchMovement = SEARCH_FORWARD;
//         }
//         else {
//             currentSearchMovement = SEARCH_TURN_RIGHT;
//         }

//         resetStuckMonitor();
//     }

//     void applySearchMovement() {
//         if (currentSearchMovement == SEARCH_TURN_RIGHT) {
//             drivetrain.set_drive_power(config.searchSpeed, -config.searchSpeed);
//         }
//         else if (currentSearchMovement == SEARCH_TURN_LEFT) {
//             drivetrain.set_drive_power(-config.searchSpeed, config.searchSpeed);
//         }
//         else if (currentSearchMovement == SEARCH_REVERSE) {
//             drivetrain.set_drive_power(-config.reverseSpeed, -config.reverseSpeed);
//         }
//         else if (currentSearchMovement == SEARCH_FORWARD) {
//             drivetrain.set_drive_power(config.forwardSpeed, config.forwardSpeed);
//         }
//     }

//     void updateStuckDetection() {
//         float now = master_timer.time(msec);

//         float movementElapsed = now - movementStartTime;

//         // Reverse should only be used as an unsticking move,
//         // not forever.
//         if (currentSearchMovement == SEARCH_REVERSE &&
//             movementElapsed >= config.maxReverseTime) {

//             goToNextSearchMovement();
//             return;
//         }

//         // Forward should also only be used briefly.
//         if (currentSearchMovement == SEARCH_FORWARD &&
//             movementElapsed >= config.maxForwardTime) {

//             goToNextSearchMovement();
//             return;
//         }

//         // Only check stuck status every stuckCheckTime.
//         if (now - lastStuckCheckTime < config.stuckCheckTime) {
//             return;
//         }

//         float currentHeading = drivetrain.get_heading_degrees();
//         float currentEncoderPosition = drivetrain.get_left_front_motor_position();

//         float headingChange = getHeadingChange(currentHeading, lastHeading);
//         float encoderChange = currentEncoderPosition - lastEncoderPosition;

//         bool madeProgress = false;

//         if (currentSearchMovement == SEARCH_TURN_RIGHT ||
//             currentSearchMovement == SEARCH_TURN_LEFT) {

//             madeProgress =
//                 std::fabs(headingChange) >= config.stuckHeadingChangeThreshold;
//         }
//         else if (currentSearchMovement == SEARCH_REVERSE ||
//                  currentSearchMovement == SEARCH_FORWARD) {

//             madeProgress =
//                 std::fabs(encoderChange) >= config.stuckEncoderChangeThreshold;
//         }

//         if (!madeProgress) {
//             goToNextSearchMovement();
//             return;
//         }

//         // Robot moved enough, so keep doing the same movement.
//         lastStuckCheckTime = now;
//         lastHeading = currentHeading;
//         lastEncoderPosition = currentEncoderPosition;
//     }

// public:
//     FindBlockCommand(
//         Drivetrain& drivetrain,
//         JetsonSerial& jetson,
//         const findingBlocksConfig& config,
//         const FindBlockRawConfig& findConfig,
//         const TrackBlockRawConfig& trackConfig,
//         const DrivePID& PID
//     )
//         : drivetrain(drivetrain),
//           jetson(jetson),
//           config(config),
//           findingConfig(findConfig),
//           trackingConfig(trackConfig),
//           PID(PID),
//           currentState(SEARCHING_FOR_BLOCK),
//           currentSearchMovement(SEARCH_TURN_RIGHT),
//           finished(false),
//           foundBlock(false),
//           trackingValid(false),
//           xError(0),
//           xPrevError(0),
//           xDerivativeError(0),
//           angularSpeed(0),
//           lastStuckCheckTime(0),
//           movementStartTime(0),
//           lastHeading(0),
//           lastEncoderPosition(0)
//     {
//     }

//     void initialize() override {
//         finished = false;

//         currentState = SEARCHING_FOR_BLOCK;
//         currentSearchMovement = SEARCH_TURN_RIGHT;

//         jetson.find_block_raw_init(findingConfig);

//         foundBlock = false;
//         trackingValid = false;

//         xError = 0;
//         xPrevError = 0;
//         xDerivativeError = 0;

//         angularSpeed = 0;

//         resetStuckMonitor();
//     }

//     void execute() override {
//         if (currentState == SEARCHING_FOR_BLOCK) {
//             foundBlock = jetson.find_block_raw_step();

//             if (!foundBlock) {
//                 applySearchMovement();
//                 updateStuckDetection();
//                 return;
//             }

//             drivetrain.set_drive_power(0, 0);

//             jetson.track_block_raw_init(trackingConfig);

//             currentState = CENTERING_BLOCK;
//             return;
//         }

//         else if (currentState == CENTERING_BLOCK) {
//             trackingValid = jetson.track_block_raw_step();

//             if (!trackingValid) {
//                 jetson.find_block_raw_init(findingConfig);

//                 currentState = SEARCHING_FOR_BLOCK;
//                 currentSearchMovement = SEARCH_TURN_RIGHT;

//                 drivetrain.set_drive_power(0, 0);

//                 resetStuckMonitor();

//                 return;
//             }

//             xError = jetson.getXError();
//             xDerivativeError = xError - xPrevError;

//             angularSpeed =
//                 xError * PID.angular_kP +
//                 xDerivativeError * PID.angular_kD;

//             if (std::fabs(angularSpeed) > config.maxSpeed) {
//                 if (angularSpeed > 0) {
//                     angularSpeed = config.maxSpeed;
//                 } else {
//                     angularSpeed = -config.maxSpeed;
//                 }
//             }

//             drivetrain.set_drive_power(angularSpeed, -angularSpeed);

//             xPrevError = xError;

//             if (std::abs(xError) <= config.centeringAcceptableX) {
//                 drivetrain.set_drive_power(0, 0);
//                 currentState = FIND_BLOCK_DONE;
//                 finished = true;
//             }

//             return;
//         }

//         else if (currentState == FIND_BLOCK_DONE) {
//             drivetrain.set_drive_power(0, 0);
//             finished = true;
//             return;
//         }
//     }

//     bool isFinished() override {
//         return finished;
//     }

//     void end() override {
//         drivetrain.set_drive_power(0, 0);
//     }
// };
#pragma once

#include "Command.h"
#include "JetsonSerial.h"
#include "RobotConfig.h"
#include "Drivetrain.h"

#include <cmath>

struct FieldAvoidZone {
    float minX;
    float maxX;
    float minY;
    float maxY;
    bool enabled;
};

struct findingBlocksConfig {
    float searchSpeed;
    float maxSpeed;
    int centeringAcceptableX;

    // Anti-stuck movement speeds
    float reverseSpeed;
    float forwardSpeed;

    // How often to check if the robot is stuck, in milliseconds
    float stuckCheckTime;

    // Minimum heading change needed while turning
    float stuckHeadingChangeThreshold;

    // Minimum encoder change needed while driving forward/reverse
    float stuckEncoderChangeThreshold;

    // Maximum time to reverse/drive forward, in milliseconds
    float maxReverseTime;
    float maxForwardTime;

    // Field rectangles to avoid
    FieldAvoidZone avoidZones[3];

    // How long to turn away from an avoided block, in milliseconds
    float avoidTurnTime;

    // How many bad tracking frames to tolerate while centering
    int maxCenteringDroppedFrames;
};

enum findingBlockState {
    SEARCHING_FOR_BLOCK,
    CENTERING_BLOCK,
    FIND_BLOCK_DONE
};

enum searchMovementState {
    SEARCH_TURN_RIGHT,
    SEARCH_TURN_LEFT,
    SEARCH_REVERSE,
    SEARCH_FORWARD
};

class FindBlockCommand : public Command {
private:
    Drivetrain& drivetrain;
    JetsonSerial& jetson;

    FindBlockRawConfig findingConfig;
    TrackBlockRawConfig trackingConfig;

    DrivePID PID;

    findingBlockState currentState;
    searchMovementState currentSearchMovement;

    findingBlocksConfig config;

    bool finished;
    bool foundBlock;
    bool trackingValid;

    int centeringDroppedFrameCount;

    int xError;
    int xPrevError;
    int xDerivativeError;

    float angularSpeed;

    // Anti-stuck variables
    float lastStuckCheckTime;
    float movementStartTime;

    float lastHeading;
    float lastEncoderPosition;

    // Avoid-zone variables
    float estimatedBlockDistance;
    float estimatedBlockFieldX;
    float estimatedBlockFieldY;

    bool avoidingRejectedBlock;
    float avoidEndTime;

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

        // Use live inertial heading, not stale GPS heading
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

    void resetStuckMonitor() {
        float now = master_timer.time(msec);

        lastStuckCheckTime = now;
        movementStartTime = now;

        lastHeading = drivetrain.get_heading_degrees();
        lastEncoderPosition = drivetrain.get_left_front_motor_position();
    }

    void goToNextSearchMovement() {
        if (currentSearchMovement == SEARCH_TURN_RIGHT) {
            currentSearchMovement = SEARCH_TURN_LEFT;
        }
        else if (currentSearchMovement == SEARCH_TURN_LEFT) {
            currentSearchMovement = SEARCH_REVERSE;
        }
        else if (currentSearchMovement == SEARCH_REVERSE) {
            currentSearchMovement = SEARCH_FORWARD;
        }
        else {
            currentSearchMovement = SEARCH_TURN_RIGHT;
        }

        resetStuckMonitor();
    }

    void startAvoidingRejectedBlock() {
        avoidingRejectedBlock = true;
        avoidEndTime = master_timer.time(msec) + config.avoidTurnTime;

        // For now, turn left away from the rejected block.
        // Later you can choose direction based on estimated block side.
        currentSearchMovement = SEARCH_TURN_LEFT;

        resetStuckMonitor();
    }

    void applySearchMovement() {
        if (currentSearchMovement == SEARCH_TURN_RIGHT) {
            drivetrain.set_drive_power(config.searchSpeed, -config.searchSpeed);
        }
        else if (currentSearchMovement == SEARCH_TURN_LEFT) {
            drivetrain.set_drive_power(-config.searchSpeed, config.searchSpeed);
        }
        else if (currentSearchMovement == SEARCH_REVERSE) {
            drivetrain.set_drive_power(-config.reverseSpeed, -config.reverseSpeed);
        }
        else if (currentSearchMovement == SEARCH_FORWARD) {
            drivetrain.set_drive_power(config.forwardSpeed, config.forwardSpeed);
        }
    }

    void updateStuckDetection() {
        float now = master_timer.time(msec);

        float movementElapsed = now - movementStartTime;

        if (currentSearchMovement == SEARCH_REVERSE &&
            movementElapsed >= config.maxReverseTime) {

            goToNextSearchMovement();
            return;
        }

        if (currentSearchMovement == SEARCH_FORWARD &&
            movementElapsed >= config.maxForwardTime) {

            goToNextSearchMovement();
            return;
        }

        if (now - lastStuckCheckTime < config.stuckCheckTime) {
            return;
        }

        float currentHeading = drivetrain.get_heading_degrees();
        float currentEncoderPosition = drivetrain.get_left_front_motor_position();

        float headingChange = getHeadingChange(currentHeading, lastHeading);
        float encoderChange = currentEncoderPosition - lastEncoderPosition;

        bool madeProgress = false;

        if (currentSearchMovement == SEARCH_TURN_RIGHT ||
            currentSearchMovement == SEARCH_TURN_LEFT) {

            madeProgress =
                std::fabs(headingChange) >= config.stuckHeadingChangeThreshold;
        }
        else if (currentSearchMovement == SEARCH_REVERSE ||
                 currentSearchMovement == SEARCH_FORWARD) {

            madeProgress =
                std::fabs(encoderChange) >= config.stuckEncoderChangeThreshold;
        }

        if (!madeProgress) {
            goToNextSearchMovement();
            return;
        }

        lastStuckCheckTime = now;
        lastHeading = currentHeading;
        lastEncoderPosition = currentEncoderPosition;
    }

public:
    FindBlockCommand(
        Drivetrain& drivetrain,
        JetsonSerial& jetson,
        const findingBlocksConfig& config,
        const FindBlockRawConfig& findConfig,
        const TrackBlockRawConfig& trackConfig,
        const DrivePID& PID
    )
        : drivetrain(drivetrain),
          jetson(jetson),
          config(config),
          findingConfig(findConfig),
          trackingConfig(trackConfig),
          PID(PID),
          currentState(SEARCHING_FOR_BLOCK),
          currentSearchMovement(SEARCH_TURN_RIGHT),
          finished(false),
          foundBlock(false),
          trackingValid(false),
          centeringDroppedFrameCount(0),
          xError(0),
          xPrevError(0),
          xDerivativeError(0),
          angularSpeed(0),
          lastStuckCheckTime(0),
          movementStartTime(0),
          lastHeading(0),
          lastEncoderPosition(0),
          estimatedBlockDistance(0),
          estimatedBlockFieldX(0),
          estimatedBlockFieldY(0),
          avoidingRejectedBlock(false),
          avoidEndTime(0)
    {
    }

    void initialize() override {
        finished = false;

        currentState = SEARCHING_FOR_BLOCK;
        currentSearchMovement = SEARCH_TURN_RIGHT;

        jetson.find_block_raw_init(findingConfig);

        foundBlock = false;
        trackingValid = false;

        centeringDroppedFrameCount = 0;

        xError = 0;
        xPrevError = 0;
        xDerivativeError = 0;

        angularSpeed = 0;

        estimatedBlockDistance = 0;
        estimatedBlockFieldX = 0;
        estimatedBlockFieldY = 0;

        avoidingRejectedBlock = false;
        avoidEndTime = 0;

        resetStuckMonitor();
    }

    void execute() override {
        if (currentState == SEARCHING_FOR_BLOCK) {
            float now = master_timer.time(msec);

            if (avoidingRejectedBlock) {
                applySearchMovement();
                updateStuckDetection();

                if (now >= avoidEndTime) {
                    avoidingRejectedBlock = false;
                    jetson.find_block_raw_init(findingConfig);
                    resetStuckMonitor();
                }

                return;
            }

            foundBlock = jetson.find_block_raw_step();

            if (!foundBlock) {
                applySearchMovement();
                updateStuckDetection();
                return;
            }

            // Estimate where the block is on the field.
            estimatedBlockDistance =
                estimateBlockDistanceFromPixelY(jetson.block_y_pos);

            estimateBlockFieldPosition(estimatedBlockDistance);

            // Reject this block if its estimated field position is inside
            // any of the 3 avoid rectangles.
            if (blockIsInsideAnyAvoidZone()) {
                // drivetrain.set_drive_power(0, 0);

                jetson.find_block_raw_init(findingConfig);

                startAvoidingRejectedBlock();

                return;
            }

            // Block is allowed, so center it.
            // drivetrain.set_drive_power(0, 0);

            jetson.track_block_raw_init(trackingConfig);

            centeringDroppedFrameCount = 0;

            currentState = CENTERING_BLOCK;
            return;
        }

        else if (currentState == CENTERING_BLOCK) {
            trackingValid = jetson.track_block_raw_step();

            if (!trackingValid) {
                centeringDroppedFrameCount++;

                if (centeringDroppedFrameCount <= config.maxCenteringDroppedFrames) {
                    /*
                        Temporarily lost tracking.

                        Do not immediately go back to searching.
                        Stop briefly and wait for tracking to come back.
                    */
                    drivetrain.set_drive_power(0, 0);
                    return;
                }

                /*
                    Too many dropped frames.
                    Now restart the search.
                */
                jetson.find_block_raw_init(findingConfig);

                currentState = SEARCHING_FOR_BLOCK;
                currentSearchMovement = SEARCH_TURN_RIGHT;

                drivetrain.set_drive_power(0, 0);

                resetStuckMonitor();

                centeringDroppedFrameCount = 0;

                return;
            }

            // Tracking is valid again, so reset dropped-frame counter.
            centeringDroppedFrameCount = 0;

            xError = jetson.getXError();
            xDerivativeError = xError - xPrevError;

            angularSpeed =
                xError * PID.angular_kP +
                xDerivativeError * PID.angular_kD;

            if (std::fabs(angularSpeed) > config.maxSpeed) {
                if (angularSpeed > 0) {
                    angularSpeed = config.maxSpeed;
                } else {
                    angularSpeed = -config.maxSpeed;
                }
            }

            drivetrain.set_drive_power(angularSpeed, -angularSpeed);

            xPrevError = xError;

            if (std::abs(xError) <= config.centeringAcceptableX) {
                drivetrain.set_drive_power(0, 0);
                currentState = FIND_BLOCK_DONE;
                finished = true;
            }

            return;
        }

        else if (currentState == FIND_BLOCK_DONE) {
            drivetrain.set_drive_power(0, 0);
            finished = true;
            return;
        }
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
        drivetrain.set_drive_power(0, 0);
    }
};