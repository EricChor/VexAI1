#pragma once

#include "Command.h"
#include "JetsonSerial.h"
#include "Drivetrain.h"
#include "RobotConfig.h"
#include "PositionTracking.h"
#include "FieldAvoidZone.h"
#include "CommandStatus.h"
#include "RandomUnstuckOrder.h"

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
    float maxUnstuckAccel;

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
    FieldAvoidZone avoidZones[17];

    // Maximum time for one tracking attempt before returning to the routine.
    float maxTrackingTime;
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
    TRACK_BLOCK_STUCK_FAILED,
    TRACK_BLOCK_TIMEOUT
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
    float trackingEndTime;

    // Stuck detection variables
    float lastStuckCheckTime;
    float unstuckStartTime;
    int unstuckAttemptCount;
    int noProgressCheckCount;
    float unstuckPreviousLeftPower;
    float unstuckPreviousRightPower;
    float unstuckStageTime;
    bool unstuckStageAtSpeed;
    bool unstuckFinishing;
    RandomUnstuckOrder unstuckOrder;

    float lastHeading;
    float lastEncoderPosition;

    // Avoid-zone variables
    float estimatedBlockDistance;
    float estimatedBlockFieldX;
    float estimatedBlockFieldY;

    int lastProgressXError;
    int lastProgressYError;

private:
    void failStuck() {
        drivetrain.set_drive_power(0, 0);
        unstuckPreviousLeftPower = 0;
        unstuckPreviousRightPower = 0;
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

    void resetStuckMonitor() {
        float now = master_timer.time(msec);

        noProgressCheckCount = 0;
        lastStuckCheckTime = now;

        lastHeading = drivetrain.get_heading_degrees();
        lastEncoderPosition = drivetrain.get_left_front_motor_position();

        lastProgressXError = xError;
        lastProgressYError = yError;
    }

    TrackBlockState stateFromUnstuckMove(int move) {
        if (move == RANDOM_UNSTUCK_FORWARD_RIGHT) {
            return TRACKING_UNSTUCK_FORWARD_RIGHT;
        }

        if (move == RANDOM_UNSTUCK_FORWARD_LEFT) {
            return TRACKING_UNSTUCK_FORWARD_LEFT;
        }

        if (move == RANDOM_UNSTUCK_BACK_RIGHT) {
            return TRACKING_UNSTUCK_BACK_RIGHT;
        }

        return TRACKING_UNSTUCK_BACK_LEFT;
    }

    unsigned int makeUnstuckSeed() {
        return
            static_cast<unsigned int>(master_timer.time(msec)) ^
            static_cast<unsigned int>(drivetrain.get_left_front_motor_position() * 31.0f) ^
            static_cast<unsigned int>(drivetrain.get_heading_degrees() * 17.0f) ^
            static_cast<unsigned int>(unstuckAttemptCount * 97);
    }

    void startCurrentUnstuckMovement() {
        setCommandStatus("Track Block Unstuck");
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
        if (config.maxUnstuckAttempts <= 0) {
            return;
        }

        if (unstuckAttemptCount >= config.maxUnstuckAttempts) {
            failStuck();
            return;
        }

        unstuckAttemptCount++;
        unstuckPreviousLeftPower = initialLeftPower;
        unstuckPreviousRightPower = initialRightPower;
        unstuckFinishing = false;
        unstuckStageTime =
            (wasMovingForward
                ? config.maxReverseTime
                : config.maxForwardTime) *
            LATERAL_SHIFT_TIME_MULTIPLIER;
        unstuckOrder.resetLateralShift(makeUnstuckSeed(), wasMovingForward);
        startCurrentUnstuckMovement();
    }

    bool applyUnstuckArc(float linearSpeed, float turnSpeed) {
        float targetLeftPower = linearSpeed + turnSpeed;
        float targetRightPower = linearSpeed - turnSpeed;
        float leftPower = targetLeftPower;
        float rightPower = targetRightPower;

        if (config.maxUnstuckAccel > 0.0f) {
            float maxChange = config.maxUnstuckAccel / 100.0f;

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

    void returnToNormalTracking() {
        setCommandStatus("Track Block");
        currentState = TRACKING_NORMAL;
        unstuckPreviousLeftPower = 0;
        unstuckPreviousRightPower = 0;
        unstuckStageAtSpeed = false;
        unstuckFinishing = false;
        resetStuckMonitor();
    }

    bool robotMadeProgressWhileTracking() {
        if (config.maxUnstuckAttempts <= 0 ||
            config.stuckCheckTime <= 0.0f) {
            return true;
        }

        float now = master_timer.time(msec);

        if (now - lastStuckCheckTime < config.stuckCheckTime) {
            return true;
        }

        float currentEncoderPosition =
            drivetrain.get_left_front_motor_position();

        float currentHeading =
            drivetrain.get_heading_degrees();

        float headingChange =
            getHeadingChange(currentHeading, lastHeading);

        int yErrorProgress =
            std::abs(lastProgressYError) - std::abs(yError);

        int xErrorProgress =
            std::abs(lastProgressXError) - std::abs(xError);

        bool headingMoved =
            std::fabs(headingChange) >= config.stuckHeadingChangeThreshold;

        /*
            For tracking, this is the most important one:
            if the robot is actually moving toward the block,
            yError should change.
        */
        bool visionYImproved =
            yErrorProgress >= config.minYErrorProgress;

        bool turningTowardBlock =
            !xCentered && headingMoved && xErrorProgress > 0;

        if (turningTowardBlock || visionYImproved) {
            // Limit consecutive failed escape attempts, not every escape used
            // during the full lifetime of this tracking command.
            unstuckAttemptCount = 0;
            lastStuckCheckTime = now;
            lastEncoderPosition = currentEncoderPosition;
            lastHeading = currentHeading;
            lastProgressXError = xError;
            lastProgressYError = yError;

            return true;
        }

        noProgressCheckCount++;

        if (noProgressCheckCount < REQUIRED_CONSECUTIVE_STUCK_CHECKS) {
            lastStuckCheckTime = now;
            lastHeading = currentHeading;
            lastEncoderPosition = currentEncoderPosition;
            lastProgressXError = xError;
            lastProgressYError = yError;
            return true;
        }

        return false;
    }

    void runUnstuckState() {
        float now = master_timer.time(msec);
        float elapsed = now - unstuckStartTime;
        bool stageAtSpeed = false;

        if (unstuckFinishing) {
            if (applyUnstuckArc(0.0f, 0.0f)) {
                returnToNormalTracking();
            }
            return;
        }

        if (currentState == TRACKING_UNSTUCK_FORWARD_RIGHT) {
            stageAtSpeed = applyUnstuckArc(config.forwardSpeed, config.turnSpeed);
        }
        else if (currentState == TRACKING_UNSTUCK_FORWARD_LEFT) {
            stageAtSpeed = applyUnstuckArc(config.forwardSpeed, -config.turnSpeed);
        }
        else if (currentState == TRACKING_UNSTUCK_BACK_RIGHT) {
            stageAtSpeed = applyUnstuckArc(-config.reverseSpeed, config.turnSpeed);
        }
        else if (currentState == TRACKING_UNSTUCK_BACK_LEFT) {
            stageAtSpeed = applyUnstuckArc(-config.reverseSpeed, -config.turnSpeed);
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
          trackingEndTime(0),
          lastStuckCheckTime(0),
          unstuckStartTime(0),
          unstuckAttemptCount(0),
          noProgressCheckCount(0),
          unstuckPreviousLeftPower(0),
          unstuckPreviousRightPower(0),
          unstuckStageTime(0),
          unstuckStageAtSpeed(false),
          unstuckFinishing(false),
          unstuckOrder(),
          lastHeading(0),
          lastEncoderPosition(0),
          estimatedBlockDistance(0),
          estimatedBlockFieldX(0),
          estimatedBlockFieldY(0),
          lastProgressXError(0),
          lastProgressYError(0)
    {
    }

    void initialize() override {
        setCommandStatus("Track Block");
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
        trackingEndTime =
            config.maxTrackingTime > 0.0f
                ? master_timer.time(msec) + config.maxTrackingTime
                : 0.0f;
        unstuckAttemptCount = 0;
        noProgressCheckCount = 0;
        unstuckPreviousLeftPower = 0;
        unstuckPreviousRightPower = 0;
        unstuckStageTime = 0;
        unstuckStageAtSpeed = false;
        unstuckFinishing = false;

        estimatedBlockDistance = 0;
        estimatedBlockFieldX = 0;
        estimatedBlockFieldY = 0;

        lastProgressXError = 0;
        lastProgressYError = 0;

        jetson.track_block_raw_init(trackingConfig);

        resetStuckMonitor();
    }

    void execute() override {
        if (currentState != TRACKING_NORMAL &&
            currentState != TRACK_BLOCK_DONE) {

            runUnstuckState();
            return;
        }

        if (trackingEndTime > 0.0f &&
            master_timer.time(msec) >= trackingEndTime) {

            setCommandStatus("Track Block Timeout");
            drivetrain.set_drive_power(0, 0);
            result = TRACK_BLOCK_TIMEOUT;
            currentState = TRACK_BLOCK_DONE;
            finished = true;
            return;
        }

        if (currentState == TRACK_BLOCK_DONE) {
            drivetrain.set_drive_power(0, 0);
            finished = true;
            return;
        }

        trackingValid = trackAllowedBlockRawStep();

        if (!trackingValid || !jetson.trackBlockRawVar.targetVisible) {
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
            startUnstuckLateralShift(
                linearSpeed >= 0.0f,
                linearSpeed + angularSpeed,
                linearSpeed - angularSpeed
            );
            return;
        }

        if (!tryingToDrive) {
            resetStuckMonitor();
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
