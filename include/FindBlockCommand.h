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
#include "PositionTracking.h"
#include "FieldAvoidZone.h"
#include "CommandStatus.h"
#include "RandomUnstuckOrder.h"

#include <cmath>

struct findingBlocksConfig {
    float searchSpeed;
    float maxSpeed;
    float minCenteringSpeed;
    int centeringAcceptableX;

    // Anti-stuck movement speeds
    float reverseSpeed;
    float forwardSpeed;
    float turnSpeed;
    float unstuckMaxAccel;

    // How often to check if the robot is stuck, in milliseconds
    float stuckCheckTime;

    // Minimum heading change needed while turning
    float stuckHeadingChangeThreshold;

    // Minimum encoder change needed while driving forward/reverse
    float stuckEncoderChangeThreshold;

    // Maximum time to run backward/forward search arcs, in milliseconds
    float maxReverseTime;
    float maxForwardTime;

    // Used to estimate a block's field position before it is perfectly centered.
    float cameraHorizontalFovDegrees;

    // Field rectangles to avoid
    FieldAvoidZone avoidZones[17];

    // How long to turn away from an avoided block, in milliseconds
    float avoidTurnTime;

    // How many bad tracking frames to tolerate while centering
    int maxCenteringDroppedFrames;

    // Minimum x-error improvement needed while centering on a block
    int centeringXProgressThreshold;

    // Maximum time for one find/center attempt before returning to the routine.
    float maxSearchTime;
};

enum findingBlockState {
    SEARCHING_FOR_BLOCK,
    CENTERING_BLOCK,
    FIND_BLOCK_DONE
};

enum searchMovementState {
    SEARCH_FORWARD_RIGHT,
    SEARCH_FORWARD_LEFT,
    SEARCH_BACK_RIGHT,
    SEARCH_BACK_LEFT
};

class FindBlockCommand : public Command {
private:
    Drivetrain& drivetrain;
    PositionTracking& positionTracking;
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
    bool successful;
    bool timedOut;
    float searchEndTime;

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

    float lastCenteringCheckTime;
    float lastCenteringHeading;
    int lastCenteringXError;
    int searchNoProgressCheckCount;
    int centeringNoProgressCheckCount;

    // Avoid-zone variables
    float estimatedBlockDistance;
    float estimatedBlockFieldX;
    float estimatedBlockFieldY;

    bool runningSearchUnstuck;
    bool avoidingRejectedBlock;
    float avoidEndTime;
    float unstuckPreviousLeftPower;
    float unstuckPreviousRightPower;
    bool searchMovementAtSpeed;
    RandomUnstuckOrder searchUnstuckOrder;

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

    void estimateBlockFieldPosition(float blockDistance, float pixelX) {
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
        for (int i = 0; i < 17; i++) {
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

        searchNoProgressCheckCount = 0;
        lastStuckCheckTime = now;
        movementStartTime = now;

        lastHeading = drivetrain.get_heading_degrees();
        lastEncoderPosition = drivetrain.get_left_front_motor_position();
    }

    bool candidateBlockIsInsideAvoidZone(int pixelX, int pixelY) {
        if (pixelX < 0 || pixelY < 0) {
            return false;
        }

        estimatedBlockDistance =
            estimateBlockDistanceFromPixelY(pixelY);

        estimateBlockFieldPosition(
            estimatedBlockDistance,
            pixelX
        );

        return blockIsInsideAnyAvoidZone();
    }

    bool currentBlockIsInsideAvoidZone() {
        return candidateBlockIsInsideAvoidZone(
            jetson.block_x_pos,
            jetson.block_y_pos
        );
    }

    void resetFindBlockRawState() {
        FindBlockRawVar& var = jetson.findBlockRawVar;

        var.lastBlockXPos = -1;
        var.lastBlockYPos = -1;
        var.lastBlockXDistance = 0;
        var.lastBlockYDistance = 0;
        var.sequentialBlocksCount = 0;
        var.x_is_stable = false;
        var.y_is_stable = false;
    }

    bool selectAllowedBlockClosestTo(int targetX, int targetY) {
        int bestIndex = -1;
        long bestDistanceSquared = 0;

        for (int i = 0; i < jetson.block_count; i++) {
            int candidateX = jetson.block_x_positions[i];
            int candidateY = jetson.block_y_positions[i];

            if (candidateX < 0 || candidateY < 0) {
                continue;
            }

            if (candidateBlockIsInsideAvoidZone(candidateX, candidateY)) {
                continue;
            }

            long dx = candidateX - targetX;
            long dy = candidateY - targetY;
            long distanceSquared = dx * dx + dy * dy;

            if (bestIndex < 0 || distanceSquared < bestDistanceSquared) {
                bestIndex = i;
                bestDistanceSquared = distanceSquared;
            }
        }

        if (bestIndex < 0) {
            jetson.block_x_pos = -1;
            jetson.block_y_pos = -1;
            return false;
        }

        jetson.block_x_pos = jetson.block_x_positions[bestIndex];
        jetson.block_y_pos = jetson.block_y_positions[bestIndex];
        return true;
    }

    bool findAllowedBlockRawStep() {
        FindBlockRawVar& var = jetson.findBlockRawVar;
        FindBlockRawConfig& conf = findingConfig;

        jetson.update_block_pose();

        if (jetson.block_count <= 0) {
            resetFindBlockRawState();
            return false;
        }

        // Use one GPS pose for every candidate in this vision frame.
        positionTracking.update_raw_pose();

        int targetX =
            var.lastBlockXPos >= 0 ?
            var.lastBlockXPos :
            trackingConfig.cameraCenterX;

        int targetY =
            var.lastBlockYPos >= 0 ?
            var.lastBlockYPos :
            trackingConfig.cameraCenterY;

        if (!selectAllowedBlockClosestTo(targetX, targetY)) {
            resetFindBlockRawState();
            return false;
        }

        if (var.lastBlockXPos < 0 || var.lastBlockYPos < 0) {
            var.lastBlockXPos = jetson.block_x_pos;
            var.lastBlockYPos = jetson.block_y_pos;

            var.lastBlockXDistance = 0;
            var.lastBlockYDistance = 0;

            var.sequentialBlocksCount = 0;

            var.x_is_stable = false;
            var.y_is_stable = false;

            return false;
        }

        var.lastBlockXDistance =
            jetson.block_x_pos - var.lastBlockXPos;

        var.lastBlockYDistance =
            jetson.block_y_pos - var.lastBlockYPos;

        var.x_is_stable =
            std::abs(var.lastBlockXDistance) <= conf.maxDifferenceDistance;

        var.y_is_stable =
            std::abs(var.lastBlockYDistance) <= conf.maxDifferenceDistance;

        if (var.x_is_stable && var.y_is_stable) {
            var.sequentialBlocksCount++;
        } else {
            var.sequentialBlocksCount = 0;
        }

        var.lastBlockXPos = jetson.block_x_pos;
        var.lastBlockYPos = jetson.block_y_pos;

        return var.sequentialBlocksCount >= conf.numSequentialBlocks;
    }

    bool updateAllowedTrackingLostFrame() {
        TrackBlockRawVar& var = jetson.trackBlockRawVar;
        TrackBlockRawConfig& conf = trackingConfig;

        var.lostFrameCount++;
        var.targetVisible = false;

        if (var.lostFrameCount > conf.maxLostFrames) {
            var.trackingLocked = false;

            var.xError = 0;
            var.yError = 0;

            var.xJump = 0;
            var.yJump = 0;

            var.xJumpValid = false;
            var.yJumpValid = false;

            return false;
        }

        return var.trackingLocked;
    }

    bool trackAllowedBlockRawStep() {
        TrackBlockRawVar& var = jetson.trackBlockRawVar;
        TrackBlockRawConfig& conf = trackingConfig;

        jetson.update_block_pose();

        if (jetson.block_count <= 0) {
            return updateAllowedTrackingLostFrame();
        }

        // Use one GPS pose for every candidate in this vision frame.
        positionTracking.update_raw_pose();

        int targetX =
            var.lastBlockXPos >= 0 ?
            var.lastBlockXPos :
            conf.cameraCenterX;

        int targetY =
            var.lastBlockYPos >= 0 ?
            var.lastBlockYPos :
            conf.cameraCenterY;

        if (!selectAllowedBlockClosestTo(targetX, targetY)) {
            return updateAllowedTrackingLostFrame();
        }

        var.targetVisible = true;

        if (!var.trackingLocked) {
            var.lastBlockXPos = jetson.block_x_pos;
            var.lastBlockYPos = jetson.block_y_pos;

            var.xJump = 0;
            var.yJump = 0;

            var.xJumpValid = true;
            var.yJumpValid = true;

            var.xError = jetson.block_x_pos - conf.cameraCenterX;
            var.yError = conf.cameraCenterY - jetson.block_y_pos;

            var.lostFrameCount = 0;
            var.trackingLocked = true;
            var.targetVisible = true;

            return true;
        }

        var.xJump = jetson.block_x_pos - var.lastBlockXPos;
        var.yJump = jetson.block_y_pos - var.lastBlockYPos;

        var.xJumpValid =
            std::abs(var.xJump) <= conf.maxTrackingXJump;

        var.yJumpValid =
            std::abs(var.yJump) <= conf.maxTrackingYJump;

        if (!var.xJumpValid || !var.yJumpValid) {
            return updateAllowedTrackingLostFrame();
        }

        var.lastBlockXPos = jetson.block_x_pos;
        var.lastBlockYPos = jetson.block_y_pos;

        var.xError = jetson.block_x_pos - conf.cameraCenterX;
        var.yError = conf.cameraCenterY - jetson.block_y_pos;

        var.lostFrameCount = 0;
        var.trackingLocked = true;
        var.targetVisible = true;

        return true;
    }

    void resetCenteringStuckMonitor() {
        centeringNoProgressCheckCount = 0;
        lastCenteringCheckTime = master_timer.time(msec);
        lastCenteringHeading = drivetrain.get_heading_degrees();
        lastCenteringXError = xError;
    }

    searchMovementState searchMovementFromUnstuckMove(int move) {
        if (move == RANDOM_UNSTUCK_FORWARD_RIGHT) {
            return SEARCH_FORWARD_RIGHT;
        }

        if (move == RANDOM_UNSTUCK_FORWARD_LEFT) {
            return SEARCH_FORWARD_LEFT;
        }

        if (move == RANDOM_UNSTUCK_BACK_RIGHT) {
            return SEARCH_BACK_RIGHT;
        }

        return SEARCH_BACK_LEFT;
    }

    unsigned int makeSearchUnstuckSeed() {
        return
            static_cast<unsigned int>(master_timer.time(msec)) ^
            static_cast<unsigned int>(drivetrain.get_left_front_motor_position() * 31.0f) ^
            static_cast<unsigned int>(drivetrain.get_heading_degrees() * 17.0f);
    }

    void startCurrentSearchUnstuckMovement() {
        currentSearchMovement = searchMovementFromUnstuckMove(searchUnstuckOrder.current());
        searchMovementAtSpeed = false;
        resetStuckMonitor();
    }

    void goToNextSearchMovement() {
        if (searchUnstuckOrder.advance()) {
            startCurrentSearchUnstuckMovement();
        } else {
            currentSearchMovement = SEARCH_FORWARD_RIGHT;
            runningSearchUnstuck = false;
            unstuckPreviousLeftPower = 0;
            unstuckPreviousRightPower = 0;
            searchMovementAtSpeed = false;
            setCommandStatus("Find Block");
        }
    }

    void startSearchUnstuck() {
        setCommandStatus("Find Block Unstuck");
        runningSearchUnstuck = true;
        unstuckPreviousLeftPower = config.searchSpeed;
        unstuckPreviousRightPower = -config.searchSpeed;
        searchUnstuckOrder.reset(makeSearchUnstuckSeed());
        startCurrentSearchUnstuckMovement();
    }

    void startAvoidingRejectedBlock() {
        setCommandStatus("Avoid Rejected Block");
        avoidingRejectedBlock = true;
        runningSearchUnstuck = true;
        unstuckPreviousLeftPower = 0;
        unstuckPreviousRightPower = 0;
        searchMovementAtSpeed = false;
        avoidEndTime = master_timer.time(msec) + config.avoidTurnTime;

        // Back away from the rejected block while curving left.
        currentSearchMovement = SEARCH_BACK_LEFT;

        resetStuckMonitor();
    }

    void restartSearchAfterCenteringStuck() {
        jetson.find_block_raw_init(findingConfig);

        currentState = SEARCHING_FOR_BLOCK;
        runningSearchUnstuck = false;
        unstuckPreviousLeftPower = 0;
        unstuckPreviousRightPower = 0;
        searchMovementAtSpeed = false;

        drivetrain.set_drive_power(0, 0);

        centeringDroppedFrameCount = 0;
        lastCenteringCheckTime = 0;

        resetStuckMonitor();
        startSearchUnstuck();
    }

    void applySearchMovement() {
        if (!runningSearchUnstuck) {
            searchMovementAtSpeed = false;
            drivetrain.set_drive_power(config.searchSpeed, -config.searchSpeed);
            return;
        }

        float linearPower =
            currentSearchMovement == SEARCH_FORWARD_RIGHT ||
            currentSearchMovement == SEARCH_FORWARD_LEFT
                ? config.forwardSpeed
                : -config.reverseSpeed;

        float turnPower =
            currentSearchMovement == SEARCH_FORWARD_RIGHT ||
            currentSearchMovement == SEARCH_BACK_RIGHT
                ? config.turnSpeed
                : -config.turnSpeed;

        float leftPower = linearPower + turnPower;
        float rightPower = linearPower - turnPower;

        if (config.unstuckMaxAccel > 0.0f) {
            float maxChange = config.unstuckMaxAccel / 100.0f;

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

        if (!searchMovementAtSpeed &&
            std::fabs(leftPower - (linearPower + turnPower)) < 0.01f &&
            std::fabs(rightPower - (linearPower - turnPower)) < 0.01f) {

            searchMovementAtSpeed = true;
            movementStartTime = master_timer.time(msec);
        }
    }

    void updateStuckDetection() {
        float now = master_timer.time(msec);

        if (!runningSearchUnstuck) {
            if (now - lastStuckCheckTime < config.stuckCheckTime) {
                return;
            }

            float currentHeading = drivetrain.get_heading_degrees();
            float headingChange = getHeadingChange(currentHeading, lastHeading);

            bool headingMoved =
                std::fabs(headingChange) >= config.stuckHeadingChangeThreshold;

            if (!headingMoved) {
                searchNoProgressCheckCount++;

                if (searchNoProgressCheckCount >= REQUIRED_CONSECUTIVE_STUCK_CHECKS) {
                    startSearchUnstuck();
                    return;
                }

                lastStuckCheckTime = now;
                lastHeading = currentHeading;
                lastEncoderPosition = drivetrain.get_left_front_motor_position();
                return;
            }

            resetStuckMonitor();
            return;
        }

        if (!searchMovementAtSpeed) {
            return;
        }

        float movementElapsed = now - movementStartTime;

        bool runningForwardArc =
            currentSearchMovement == SEARCH_FORWARD_RIGHT ||
            currentSearchMovement == SEARCH_FORWARD_LEFT;

        bool runningBackArc =
            currentSearchMovement == SEARCH_BACK_RIGHT ||
            currentSearchMovement == SEARCH_BACK_LEFT;

        float maxMovementTime =
            runningForwardArc ? config.maxForwardTime : config.maxReverseTime;

        if ((runningForwardArc || runningBackArc) &&
            movementElapsed >= maxMovementTime) {

            goToNextSearchMovement();
            return;
        }

        lastStuckCheckTime = now;
        lastHeading = drivetrain.get_heading_degrees();
        lastEncoderPosition = drivetrain.get_left_front_motor_position();
    }

    bool centeringMadeProgress(float angularSpeed) {
        if (config.stuckCheckTime <= 0 ||
            std::fabs(angularSpeed) < 5.0f) {

            resetCenteringStuckMonitor();
            return true;
        }

        float now = master_timer.time(msec);

        if (lastCenteringCheckTime == 0) {
            resetCenteringStuckMonitor();
            return true;
        }

        if (now - lastCenteringCheckTime < config.stuckCheckTime) {
            return true;
        }

        int xErrorProgress =
            std::abs(lastCenteringXError) - std::abs(xError);

        bool xErrorImproved =
            xErrorProgress >= config.centeringXProgressThreshold;

        if (xErrorImproved) {
            resetCenteringStuckMonitor();
            return true;
        }

        centeringNoProgressCheckCount++;

        if (centeringNoProgressCheckCount < REQUIRED_CONSECUTIVE_STUCK_CHECKS) {
            lastCenteringCheckTime = now;
            lastCenteringHeading = drivetrain.get_heading_degrees();
            lastCenteringXError = xError;
            return true;
        }

        return false;
    }

public:
    FindBlockCommand(
        Drivetrain& drivetrain,
        PositionTracking& positionTracking,
        JetsonSerial& jetson,
        const findingBlocksConfig& config,
        const FindBlockRawConfig& findConfig,
        const TrackBlockRawConfig& trackConfig,
        const DrivePID& PID
    )
        : drivetrain(drivetrain),
          positionTracking(positionTracking),
          jetson(jetson),
          config(config),
          findingConfig(findConfig),
          trackingConfig(trackConfig),
          PID(PID),
          currentState(SEARCHING_FOR_BLOCK),
          currentSearchMovement(SEARCH_FORWARD_RIGHT),
          finished(false),
          foundBlock(false),
          trackingValid(false),
          successful(false),
          timedOut(false),
          searchEndTime(0),
          centeringDroppedFrameCount(0),
          xError(0),
          xPrevError(0),
          xDerivativeError(0),
          angularSpeed(0),
          lastStuckCheckTime(0),
          movementStartTime(0),
          lastHeading(0),
          lastEncoderPosition(0),
          lastCenteringCheckTime(0),
          lastCenteringHeading(0),
          lastCenteringXError(0),
          searchNoProgressCheckCount(0),
          centeringNoProgressCheckCount(0),
          estimatedBlockDistance(0),
          estimatedBlockFieldX(0),
          estimatedBlockFieldY(0),
          runningSearchUnstuck(false),
          avoidingRejectedBlock(false),
          avoidEndTime(0),
          unstuckPreviousLeftPower(0),
          unstuckPreviousRightPower(0),
          searchMovementAtSpeed(false),
          searchUnstuckOrder()
    {
    }

    void initialize() override {
        setCommandStatus("Find Block");
        finished = false;
        successful = false;
        timedOut = false;
        searchEndTime =
            config.maxSearchTime > 0.0f
                ? master_timer.time(msec) + config.maxSearchTime
                : 0.0f;

        currentState = SEARCHING_FOR_BLOCK;
        currentSearchMovement = SEARCH_FORWARD_RIGHT;
        runningSearchUnstuck = false;
        unstuckPreviousLeftPower = 0;
        unstuckPreviousRightPower = 0;
        searchMovementAtSpeed = false;

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

        lastCenteringCheckTime = 0;
        lastCenteringHeading = 0;
        lastCenteringXError = 0;
        searchNoProgressCheckCount = 0;
        centeringNoProgressCheckCount = 0;

        avoidingRejectedBlock = false;
        avoidEndTime = 0;

        resetStuckMonitor();
    }

    void execute() override {
        if (searchEndTime > 0.0f &&
            master_timer.time(msec) >= searchEndTime &&
            !runningSearchUnstuck) {

            setCommandStatus("Find Block Timeout");
            drivetrain.set_drive_power(0, 0);
            timedOut = true;
            finished = true;
            return;
        }

        if (currentState == SEARCHING_FOR_BLOCK) {
            float now = master_timer.time(msec);

            if (avoidingRejectedBlock) {
                // Keep consuming live frames so serial data does not become
                // stale while the robot is completing a recovery movement.
                jetson.update_block_pose();
                applySearchMovement();
                updateStuckDetection();

                if (now >= avoidEndTime) {
                    avoidingRejectedBlock = false;
                    runningSearchUnstuck = false;
                    unstuckPreviousLeftPower = 0;
                    unstuckPreviousRightPower = 0;
                    searchMovementAtSpeed = false;
                    setCommandStatus("Find Block");
                    jetson.find_block_raw_init(findingConfig);
                    resetStuckMonitor();
                }

                return;
            }

            // Once unstuck starts, finish all four randomized arc movements.
            // Vision detections during the escape must not cancel the sequence.
            if (runningSearchUnstuck) {
                jetson.update_block_pose();
                applySearchMovement();
                updateStuckDetection();
                return;
            }

            foundBlock = findAllowedBlockRawStep();

            if (!foundBlock) {
                applySearchMovement();
                updateStuckDetection();
                return;
            }

            if (currentBlockIsInsideAvoidZone()) {
                jetson.find_block_raw_init(findingConfig);
                currentState = SEARCHING_FOR_BLOCK;
                runningSearchUnstuck = false;
                resetStuckMonitor();
                return;
            }

            jetson.track_block_raw_init(trackingConfig);

            centeringDroppedFrameCount = 0;
            lastCenteringCheckTime = 0;

            currentState = CENTERING_BLOCK;
            return;
        }

        else if (currentState == CENTERING_BLOCK) {
            trackingValid = trackAllowedBlockRawStep();

            if (!trackingValid || !jetson.trackBlockRawVar.targetVisible) {
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
                currentSearchMovement = SEARCH_FORWARD_RIGHT;

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

            if (std::abs(xError) > config.centeringAcceptableX &&
                config.minCenteringSpeed > 0.0f &&
                std::fabs(angularSpeed) < config.minCenteringSpeed) {

                angularSpeed =
                    xError >= 0
                        ? config.minCenteringSpeed
                        : -config.minCenteringSpeed;
            }

            if (currentBlockIsInsideAvoidZone()) {
                jetson.find_block_raw_init(findingConfig);

                currentState = SEARCHING_FOR_BLOCK;
                runningSearchUnstuck = false;
                resetStuckMonitor();

                centeringDroppedFrameCount = 0;
                lastCenteringCheckTime = 0;

                return;
            }

            if (!centeringMadeProgress(angularSpeed)) {
                restartSearchAfterCenteringStuck();
                return;
            }

            drivetrain.set_drive_power(angularSpeed, -angularSpeed);

            xPrevError = xError;

            if (std::abs(xError) <= config.centeringAcceptableX) {
                drivetrain.set_drive_power(0, 0);

                currentState = FIND_BLOCK_DONE;
                successful = true;
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

    bool wasSuccessful() const {
        return successful;
    }

    bool didTimeOut() const {
        return timedOut;
    }
};
