#include "InteractionRoutine.h"

#include "RobotConfig.h"
#include "Drivetrain.h"
#include "Intake.h"
#include "PositionTracking.h"
#include "CommandStatus.h"

#include "SequentialCommandGroup.h"
#include "RepeatForeverCommandGroup.h"
#include "ParallelDeadlineGroup.h"
#include "ConditionalCommandGroup.h"

#include "FindBlockCommand.h"
#include "TrackBlockCommand.h"
#include "IntakeWithSortingCommand.h"
#include "GrabBlockCommand.h"
#include "DriveStraightCommand.h"
#include "DriveToXPositionCommand.h"
#include "DriveToYPositionCommand.h"
#include "DriveToPointCommand.h"
#include "DriveToPointUntilXCommand.h"
#include "DriveToPointUntilYCommand.h"
#include "GetGPSCoordinatesFilteredCommand.h"
#include "WallAlignmentCommand.h"
#include "WaitAndScoreCommand.h"
#include "SetBlocksBeforeScoringCommand.h"
#include "InertialTurnCommand.h"

#include "DoNothingCommand.h"
#include "SetDrivetrainPoseFromGPSCommand.h"

#include <cmath>

// ============================================================
// Interaction routine file map
// ============================================================
//
// 1. Condition helpers
// 2. Exported route object
// 3. PID constants
// 4. Vision and intake configs
// 5. Movement, scoring, and wall-alignment targets/configs
// 6. Command instances
// 7. Command groups and branch selectors
// 8. Routine assembly

// ------------------------------------------------------------
// Condition helpers
// ------------------------------------------------------------

enum SearchCorner {
    SEARCH_CORNER_BOTTOM_LEFT,
    SEARCH_CORNER_BOTTOM_RIGHT,
    SEARCH_CORNER_TOP_LEFT,
    SEARCH_CORNER_TOP_RIGHT
};

static FindBlockCommand* findBlockPtr = nullptr;
static TrackBlockCommand* trackBlockPtr = nullptr;

static bool cornerSearchMonitorInitialized = false;
static SearchCorner monitoredSearchCorner = SEARCH_CORNER_BOTTOM_LEFT;
static SearchCorner recoverySourceCorner = SEARCH_CORNER_BOTTOM_LEFT;
static float cornerSearchStartTime = 0.0f;
static int lastAcceptedBlockCount = 0;
static const float cornerSearchRecoveryTime = 15000.0f;

SearchCorner getCurrentSearchCorner() {
    bool top = positionTracking.get_y() >= 0.0f;
    bool right = positionTracking.get_x() >= 0.0f;

    if (top) {
        return right ? SEARCH_CORNER_TOP_RIGHT : SEARCH_CORNER_TOP_LEFT;
    }

    return right ? SEARCH_CORNER_BOTTOM_RIGHT : SEARCH_CORNER_BOTTOM_LEFT;
}

bool findBlockWasSuccessful() {
    return findBlockPtr != nullptr && findBlockPtr->wasSuccessful();
}

bool trackBlockWasSuccessful() {
    if (trackBlockPtr == nullptr) {
        return false;
    }

    return trackBlockPtr->wasSuccessful();
}

bool hasEnoughBlocksToScore() {
    return intake.has_enough_blocks_to_score();
}

bool shouldRecoverFromEmptyCorner() {
    float now = master_timer.time(msec);
    SearchCorner currentCorner = getCurrentSearchCorner();
    int acceptedBlockCount = intake.get_accepted_block_count();

    if (!cornerSearchMonitorInitialized) {
        cornerSearchMonitorInitialized = true;
        monitoredSearchCorner = currentCorner;
        cornerSearchStartTime = now;
        lastAcceptedBlockCount = acceptedBlockCount;
        return false;
    }

    if (currentCorner != monitoredSearchCorner ||
        acceptedBlockCount != lastAcceptedBlockCount ||
        intake.has_enough_blocks_to_score()) {

        monitoredSearchCorner = currentCorner;
        cornerSearchStartTime = now;
        lastAcceptedBlockCount = acceptedBlockCount;
        return false;
    }

    if (now - cornerSearchStartTime < cornerSearchRecoveryTime) {
        return false;
    }

    recoverySourceCorner = currentCorner;
    cornerSearchStartTime = now;
    return true;
}

bool recoverySourceIsTop() {
    return
        recoverySourceCorner == SEARCH_CORNER_TOP_LEFT ||
        recoverySourceCorner == SEARCH_CORNER_TOP_RIGHT;
}

bool recoverySourceIsRight() {
    return
        recoverySourceCorner == SEARCH_CORNER_BOTTOM_RIGHT ||
        recoverySourceCorner == SEARCH_CORNER_TOP_RIGHT;
}

void reset_interaction_corner_search_monitor() {
    cornerSearchMonitorInitialized = false;
    cornerSearchStartTime = 0.0f;
    lastAcceptedBlockCount = intake.get_accepted_block_count();
}

bool robotIsOnTopSide() {
    return positionTracking.get_y() >= 0.0f;
}

bool robotIsOnRightSide() {
    return positionTracking.get_x() >= 0.0f;
}

bool scoreBottomLeftApproachIsClose();
bool scoreBottomRightApproachIsClose();
bool scoreTopLeftApproachIsClose();
bool scoreTopRightApproachIsClose();

// ------------------------------------------------------------
// Exported route object
// ------------------------------------------------------------

SequentialCommandGroup AI_ROUTE_1;
RepeatForeverCommandGroup AI_INTERACTION_ROUTE(AI_ROUTE_1);

// ------------------------------------------------------------
// PID constants
// ------------------------------------------------------------

DrivePID driveIntoGoalPID{
    .linear_kP = 3,
    .linear_kI = 0.0,
    .linear_kD = 0.25,

    .angular_kP = 0.05,
    .angular_kI = 0.05,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 15,
    .linear_integral_windup_threshold = 15
};

DrivePID grabBlockPID {
    .linear_kP = 3,
    .linear_kI = 0.0,
    .linear_kD = 0.25,

    .angular_kP = 0.05,
    .angular_kI = 0.05,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 15,
    .linear_integral_windup_threshold = 15
};

DrivePID DriveToPointPID{
    .linear_kP = 2,
    .linear_kI = 0.0,
    .linear_kD = 0.1,

    .angular_kP = 0.2,
    .angular_kI = 0.0,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 20,
    .linear_integral_windup_threshold = 20
};


DrivePID defaultPID {
    .linear_kP = 0.25,
    .linear_kI = 0.0,
    .linear_kD = 0.05,

    .angular_kP = 0.05,
    .angular_kI = 0.0,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 20,
    .linear_integral_windup_threshold = 20
};

DrivePID wallAlignmentPID {
    .linear_kP = 0.35,
    .linear_kI = 0.02,
    .linear_kD = 0.05,

    .angular_kP = 0.25,
    .angular_kI = 0.0,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 20,
    .linear_integral_windup_threshold = 20
};

// ------------------------------------------------------------
// Vision configs
// ------------------------------------------------------------

FindBlockRawConfig findBlockRawConfig {
    .numSequentialBlocks = 3,
    .maxDifferenceDistance = 150
};

TrackBlockRawConfig trackBlockRawConfig {
    .cameraCenterX = 320,
    .cameraCenterY = 400,

    .maxTrackingXJump = 150,
    .maxTrackingYJump = 150,

    .maxLostFrames = 8
};

// ------------------------------------------------------------
// Find block command config
// ------------------------------------------------------------

findingBlocksConfig findBlockConfig {
    .searchSpeed = 10,
    .maxSpeed = 25,
    .minCenteringSpeed = 8,
    .centeringAcceptableX = 15,

    .reverseSpeed = 25,
    .forwardSpeed = 20,
    .turnSpeed = 15,
    .unstuckMaxAccel = 100,

    .stuckCheckTime = 750,

    .stuckHeadingChangeThreshold = 3,
    .stuckEncoderChangeThreshold = 10,

    .maxReverseTime = 500,
    .maxForwardTime = 300,

    .cameraHorizontalFovDegrees = 60,

    .avoidZones = {
        {
            .minX = -108.0f,
            .maxX = -72.0f,
            .minY = -108.0f,
            .maxY = 108.0f,
            .enabled = true
        },
        {
            .minX = 72.0f,
            .maxX = 108.0f,
            .minY = -108.0f,
            .maxY = 108.0f,
            .enabled = true
        },
        {
            .minX = -108.0f,
            .maxX = 108.0f,
            .minY = -108.0f,
            .maxY = -72.0f,
            .enabled = true
        },
        {
            .minX = -108.0f,
            .maxX = 108.0f,
            .minY = 72.0f,
            .maxY = 108.0f,
            .enabled = true
        },
        {
            .minX = -72.0f,
            .maxX = -48.0f,
            .minY = -18.0f,
            .maxY = 18.0f,
            .enabled = true
        },
        {
            .minX = 48.0f,
            .maxX = 72.0f,
            .minY = -18.0f,
            .maxY = 18.0f,
            .enabled = true
        },
        {
            .minX = -18.0f,
            .maxX = 18.0f,
            .minY = -18.0f,
            .maxY = 18.0f,
            .enabled = true
        },
        {
            .minX = -30.0f,
            .maxX = 30.0f,
            .minY = 42.0f,
            .maxY = 72.0f,
            .enabled = true
        },
        {
            .minX = -30.0f,
            .maxX = 30.0f,
            .minY = -72.0f,
            .maxY = -42.0f,
            .enabled = true
        },
        {
            .minX = -73.0f,
            .maxX = -67.0f,
            .minY = 45.0f,
            .maxY = 51.0f,
            .enabled = true
        },
        {
            .minX = 67.0f,
            .maxX = 73.0f,
            .minY = 45.0f,
            .maxY = 51.0f,
            .enabled = true
        },
        {
            .minX = 67.0f,
            .maxX = 73.0f,
            .minY = -51.0f,
            .maxY = -45.0f,
            .enabled = true
        },
        {
            .minX = -73.0f,
            .maxX = -67.0f,
            .minY = -51.0f,
            .maxY = -45.0f,
            .enabled = true
        },
        {
            .minX = -72.0f,
            .maxX = -64.0f,
            .minY = 64.0f,
            .maxY = 72.0f,
            .enabled = true
        },
        {
            .minX = 64.0f,
            .maxX = 72.0f,
            .minY = -72.0f,
            .maxY = -64.0f,
            .enabled = true
        },
        {
            .minX = 64.0f,
            .maxX = 72.0f,
            .minY = 64.0f,
            .maxY = 72.0f,
            .enabled = true
        },
        {
            .minX = -72.0f,
            .maxX = -64.0f,
            .minY = -72.0f,
            .maxY = -64.0f,
            .enabled = true
        }
    },

    .avoidTurnTime = 500,

    .maxCenteringDroppedFrames = 5,
    .centeringXProgressThreshold = 5,

    .maxSearchTime = 6000
};

// ------------------------------------------------------------
// Track block command config
// ------------------------------------------------------------

TrackingBlocksConfig trackBlockConfig {
    .acceptableYError = 10,
    .acceptableXError = 12,

    .maxLinearSpeed = 25,
    .maxAngularSpeed = 20,
    .minLinearSpeed = 10,
    .minAngularSpeed = 8,

    .stuckCheckTime = 750,
    .stuckEncoderChangeThreshold = 10,
    .stuckHeadingChangeThreshold = 3,

    .reverseSpeed = 20,
    .forwardSpeed = 20,
    .turnSpeed = 15,
    .maxUnstuckAccel = 100,

    .maxReverseTime = 500,
    .maxForwardTime = 300,
    .minLinearSpeedForStuckCheck = 10,
    .minYErrorProgress = 5,
    .maxTrackingDroppedFrames = 10,
    .maxUnstuckAttempts = 1,

    .cameraHorizontalFovDegrees = 60,

    .avoidZones = {
        {
            .minX = -108.0f,
            .maxX = -72.0f,
            .minY = -108.0f,
            .maxY = 108.0f,
            .enabled = true
        },
        {
            .minX = 72.0f,
            .maxX = 108.0f,
            .minY = -108.0f,
            .maxY = 108.0f,
            .enabled = true
        },
        {
            .minX = -108.0f,
            .maxX = 108.0f,
            .minY = -108.0f,
            .maxY = -72.0f,
            .enabled = true
        },
        {
            .minX = -108.0f,
            .maxX = 108.0f,
            .minY = 72.0f,
            .maxY = 108.0f,
            .enabled = true
        },
        {
            .minX = -72.0f,
            .maxX = -48.0f,
            .minY = -18.0f,
            .maxY = 18.0f,
            .enabled = true
        },
        {
            .minX = 48.0f,
            .maxX = 72.0f,
            .minY = -18.0f,
            .maxY = 18.0f,
            .enabled = true
        },
        {
            .minX = -18.0f,
            .maxX = 18.0f,
            .minY = -18.0f,
            .maxY = 18.0f,
            .enabled = true
        },
        {
            .minX = -30.0f,
            .maxX = 30.0f,
            .minY = 42.0f,
            .maxY = 72.0f,
            .enabled = true
        },
        {
            .minX = -30.0f,
            .maxX = 30.0f,
            .minY = -72.0f,
            .maxY = -42.0f,
            .enabled = true
        },
        {
            .minX = -73.0f,
            .maxX = -67.0f,
            .minY = 45.0f,
            .maxY = 51.0f,
            .enabled = true
        },
        {
            .minX = 67.0f,
            .maxX = 73.0f,
            .minY = 45.0f,
            .maxY = 51.0f,
            .enabled = true
        },
        {
            .minX = 67.0f,
            .maxX = 73.0f,
            .minY = -51.0f,
            .maxY = -45.0f,
            .enabled = true
        },
        {
            .minX = -73.0f,
            .maxX = -67.0f,
            .minY = -51.0f,
            .maxY = -45.0f,
            .enabled = true
        },
        {
            .minX = -72.0f,
            .maxX = -64.0f,
            .minY = 64.0f,
            .maxY = 72.0f,
            .enabled = true
        },
        {
            .minX = 64.0f,
            .maxX = 72.0f,
            .minY = -72.0f,
            .maxY = -64.0f,
            .enabled = true
        },
        {
            .minX = 64.0f,
            .maxX = 72.0f,
            .minY = 64.0f,
            .maxY = 72.0f,
            .enabled = true
        },
        {
            .minX = -72.0f,
            .maxX = -64.0f,
            .minY = -72.0f,
            .maxY = -64.0f,
            .enabled = true
        }

    },

    .maxTrackingTime = 8000
};

// ------------------------------------------------------------
// Intake config
// ------------------------------------------------------------

IntakeWithSortingConfig intakeSortingConfig {
    .initial_intake_velocity = 100,
    .middle_intake_velcoity = 100,
    .final_intake_velocity = 100,

    .red_value = 20,
    .blue_value = 220,

    .red_threshold = 50,
    .blue_threshold = 50,

    .threshold_velocity = 5,

    .sorting_time = 600,
    .accept_time = 800,
    .unjamming_time = 300
};

InertialTurnConfig inertialTurnConfig{
    .max_speed = 25,
    .min_speed = 8,
    .max_time = 5000,
    .settle_error = 4,
    .settle_speed = 120,
    .settle_time = 250,
    .unstuck = {
        .stuck_check_time = 750,
        .heading_change_threshold = 3,
        .error_progress_threshold = 3,
        .forward_speed = 20,
        .reverse_speed = 20,
        .turn_speed = 15,
        .forward_time = 300,
        .reverse_time = 500,
        .max_accel = 125,
        .max_attempts = 1
    }
};

InertialTurnTarget inertialTurnTargetBottomWallAlignment{
    .target_heading = 0
};

InertialTurnTarget inertialTurnTargetTopWallAlignment{
    .target_heading = 180
};

InertialTurnTarget inertialTurnTargetLeftScoring{
    .target_heading = 270
};

InertialTurnTarget inertialTurnTargetRightScoring{
    .target_heading = 90
};

// ------------------------------------------------------------
// Grab-block target/config
// ------------------------------------------------------------

GrabBlockTarget grabBlockTarget {
    .target_distance = 10,
    .target_heading = 0
};

GrabBlockConfig grabBlockConfig {
    .acceptable_error = 0.5,
    .max_accel = 100,
    .max_angular_speed = 25,
    .max_linear_speed = 40,
    .max_time = 5000,
    .settle_time = 250,
    .unstuck = {
        .stuck_check_time = 750,
        .encoder_change_threshold = 15,
        .error_progress_threshold = 1,
        .forward_speed = 20,
        .reverse_speed = 30,
        .turn_speed = 25,
        .forward_time = 300,
        .reverse_time = 600,
        .max_accel = 125,
        .max_attempts = 1
    }
};

// ------------------------------------------------------------
// GPS config
// ------------------------------------------------------------

GetGPSCoordinatesFilteredConfig gpsConfig {
    .maxHeadingJump = 4,
    .maxPositionJump = 3,
    .minAcceptedSamples = 3,
    .sampleCount = 6,
    .sampleIntervals = 50
};

// ============================================================
// Command instances
// ============================================================

// ------------------------------------------------------------
// GPS and block collection commands
// ------------------------------------------------------------

static GetGPSCoordinatesFilteredCommand updateGPS(
    positionTracking,
    gpsConfig
);

static SetDrivetrainPoseFromGPSCommand setPoseFromGPS(
    drivebase,
    positionTracking,
    &updateGPS
);

static GetGPSCoordinatesFilteredCommand updateGPSBeforeScoringTurn(
    positionTracking,
    gpsConfig
);

static SetDrivetrainPoseFromGPSCommand setPoseFromGPSBeforeScoringTurn(
    drivebase,
    positionTracking,
    &updateGPSBeforeScoringTurn
);

static FindBlockCommand find_block(
    drivebase,
    positionTracking,
    jetsonSerial,
    findBlockConfig,
    findBlockRawConfig,
    trackBlockRawConfig,
    defaultPID
);

static TrackBlockCommand track_block(
    drivebase,
    positionTracking,
    jetsonSerial,
    trackBlockConfig,
    trackBlockRawConfig,
    defaultPID
);

static GrabBlockCommand grabBlock(
    drivebase,
    grabBlockConfig,
    grabBlockPID,
    grabBlockTarget
);

static IntakeWithSorting intake_while_tracking(
    intake,
    intakeSortingConfig
);

static IntakeWithSorting intake_while_grabbing(
    intake,
    intakeSortingConfig
);

static DoNothingCommand skip_grab;

// ------------------------------------------------------------
// Scoring approach targets and retry checks
// ------------------------------------------------------------

DriveToPointUntilYTarget scoreBottomLeftTarget {
    .target_x = -48,
    .target_y = -48,
    .exit_y = -48
};

DriveToPointUntilYTarget scoreBottomRightTarget {
    .target_x = 48,
    .target_y = -48,
    .exit_y = -48
};

DriveToPointUntilYTarget scoreTopLeftTarget {
    .target_x = -48,
    .target_y = 48,
    .exit_y = 48
};

DriveToPointUntilYTarget scoreTopRightTarget {
    .target_x = 48,
    .target_y = 48,
    .exit_y = 48
};

static const float scoreApproachRetryDistance = 6.0f;

bool robotIsCloseToPoint(float targetX, float targetY, float acceptableDistance) {
    if (!updateGPS.wasSuccessful()) {
        return false;
    }

    float xError = targetX - positionTracking.get_x();
    float yError = targetY - positionTracking.get_y();
    float distance = std::sqrt(xError * xError + yError * yError);

    return distance <= acceptableDistance;
}

bool scoreBottomLeftApproachIsClose() {
    return robotIsCloseToPoint(
        scoreBottomLeftTarget.target_x,
        scoreBottomLeftTarget.target_y,
        scoreApproachRetryDistance
    );
}

bool scoreBottomRightApproachIsClose() {
    return robotIsCloseToPoint(
        scoreBottomRightTarget.target_x,
        scoreBottomRightTarget.target_y,
        scoreApproachRetryDistance
    );
}

bool scoreTopLeftApproachIsClose() {
    return robotIsCloseToPoint(
        scoreTopLeftTarget.target_x,
        scoreTopLeftTarget.target_y,
        scoreApproachRetryDistance
    );
}

bool scoreTopRightApproachIsClose() {
    return robotIsCloseToPoint(
        scoreTopRightTarget.target_x,
        scoreTopRightTarget.target_y,
        scoreApproachRetryDistance
    );
}

// ------------------------------------------------------------
// Drive-to-point configs
// ------------------------------------------------------------

DriveToPointConfig pointConfig {
    .max_linear_speed = 30,
    .max_angular_speed = 25,
    .position_acceptable_error = 3,
    .heading_acceptable_error = 3,
    .max_time = 10000,
    .pointing_settle_time = 150,
    .position_settle_time = 0,
    .min_linear_speed = 8,
    .min_angular_speed = 8,
    .stuck_check_time = 750,
    .stuck_distance_progress = 2,
    .stuck_heading_progress = 3,
    .stuck_encoder_change_threshold = 20,
    .forward_speed = 20,
    .reverse_speed = 20,
    .turn_speed = 15,
    .max_reverse_time = 500,
    .max_turn_time = 300,
    .unstuck_max_accel = 100,
    .max_unstuck_attempts = 1,
    .drive_direction = DRIVE_TO_POINT_DRIVE_FORWARD
};

// ------------------------------------------------------------
// Empty-corner recovery targets
// ------------------------------------------------------------

DriveToPointTarget recoverBottomLeftResetTarget {
    .target_x = -36,
    .target_y = -36
};

DriveToPointUntilYTarget recoverBottomLeftViaTarget {
    .target_x = -36,
    .target_y = 0,
    .exit_y = 0
};

DriveToPointTarget recoverBottomLeftDestinationTarget {
    .target_x = -36,
    .target_y = 36
};

DriveToPointTarget recoverTopLeftResetTarget {
    .target_x = -36,
    .target_y = 36
};

DriveToPointUntilXTarget recoverTopLeftViaTarget {
    .target_x = 0,
    .target_y = 36,
    .exit_x = 0
};

DriveToPointTarget recoverTopLeftDestinationTarget {
    .target_x = 36,
    .target_y = 36
};

DriveToPointTarget recoverTopRightResetTarget {
    .target_x = 36,
    .target_y = 36
};

DriveToPointUntilYTarget recoverTopRightViaTarget {
    .target_x = 36,
    .target_y = 0,
    .exit_y = 0
};

DriveToPointTarget recoverTopRightDestinationTarget {
    .target_x = 36,
    .target_y = -36
};

DriveToPointTarget recoverBottomRightResetTarget {
    .target_x = 36,
    .target_y = -36
};

DriveToPointUntilXTarget recoverBottomRightViaTarget {
    .target_x = 0,
    .target_y = -36,
    .exit_x = 0
};

DriveToPointTarget recoverBottomRightDestinationTarget {
    .target_x = -36,
    .target_y = -36
};

// ------------------------------------------------------------
// Reverse staging targets before driving into goals
// ------------------------------------------------------------

DriveToXPositionTarget scoreReverseBottomLeftTarget {
    .target_x = -36,
    .target_heading = 270
};

DriveToXPositionTarget scoreReverseBottomRightTarget {
    .target_x = 36,
    .target_heading = 90
};

DriveToXPositionTarget scoreReverseTopLeftTarget {
    .target_x = -36,
    .target_heading = 270
};

DriveToXPositionTarget scoreReverseTopRightTarget {
    .target_x = 36,
    .target_heading = 90
};

DriveToXPositionConfig scoreReverseXConfig {
    .max_linear_speed = 25,
    .max_angular_speed = 20,
    .acceptable_error = 1.5,
    .heading_acceptable_error = 4,
    .max_linear_heading_error = 8,
    .min_linear_speed = 8,
    .min_angular_speed = 6,
    .max_time = 7000,
    .settle_time = 200,
    .unstuck = {
        .stuck_check_time = 750,
        .error_progress_threshold = 1,
        .forward_speed = 20,
        .reverse_speed = 20,
        .turn_speed = 15,
        .forward_time = 300,
        .reverse_time = 500,
        .max_accel = 100,
        .max_attempts = 1
    }
};

// ------------------------------------------------------------
// Wall-alignment targets/config
// ------------------------------------------------------------

WallAlignmentTarget wallBottomTarget {
    .target_heading = 0,
    .target_distance = 16
};

WallAlignmentTarget wallTopTarget {
    .target_heading = 180,
    .target_distance = 16
};

WallAlignmentConfig wallConfig {
    .max_linear_speed = 25,
    .max_angular_speed = 20,
    .acceptable_error = 0.5,
    .heading_acceptable_error = 3,
    .max_linear_heading_error = 15,
    .sensor_difference_kP = 2.0,
    .sensor_difference_acceptable_error = 1.0,
    .max_linear_sensor_difference = 3.0,
    .max_time = 7000,
    .max_accel = 100,
    .settle_time = 200,
    .unstuck = {
        .stuck_check_time = 750,
        .heading_change_threshold = 3,
        .error_progress_threshold = 0.5,
        .forward_speed = 18,
        .reverse_speed = 18,
        .turn_speed = 12,
        .forward_time = 300,
        .reverse_time = 500,
        .max_accel = 100,
        .max_attempts = 2
    }
};

// ------------------------------------------------------------
// Final drive into and out of goal config/targets
// ------------------------------------------------------------

DriveStraightConfig driveIntoGoal{
    .acceptable_error = 1,
    .max_accel = 100,
    .max_angular_speed = 25,
    .max_linear_speed = 25,
    .max_time = 3000,
    .settle_time = 250,
    .unstuck = {
        .max_accel = 100,
        .max_attempts = 0
    }
};

DriveStraightTarget driveIntoGoalLeftTarget{
    .target_distance = -15,
    .target_heading = 270
};

DriveStraightTarget driveIntoGoalRightTarget{
    .target_distance = -15,
    .target_heading = 90
};

DriveStraightTarget driveOutOfGoalLeftTarget{
    .target_distance = 15,
    .target_heading = 270
};

DriveStraightTarget driveOutOfGoalRightTarget{
    .target_distance = 15,
    .target_heading = 90
};

// ------------------------------------------------------------
// Score/intake count configs
// ------------------------------------------------------------

WaitAndScoreConfig scoreConfig {
    .wait_time = 250,
    .score_time = 3000,
    .score_mode = SCORE_HIGH
};

BlocksBeforeScoringConfig blocksBeforeScoringConfig {
    .target_block_count = 3,
    .reset_count = false
};

BlocksBeforeScoringConfig resetBlocksAfterScoringConfig {
    .target_block_count = 0,
    .reset_count = true
};

// ------------------------------------------------------------
// Scoring command instances
// ------------------------------------------------------------

static DriveToPointCommand recover_bottom_left_reset(
    drivebase,
    positionTracking,
    recoverBottomLeftResetTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointUntilYCommand recover_bottom_left_via(
    drivebase,
    positionTracking,
    recoverBottomLeftViaTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointCommand recover_bottom_left_destination(
    drivebase,
    positionTracking,
    recoverBottomLeftDestinationTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointCommand recover_top_left_reset(
    drivebase,
    positionTracking,
    recoverTopLeftResetTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointUntilXCommand recover_top_left_via(
    drivebase,
    positionTracking,
    recoverTopLeftViaTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointCommand recover_top_left_destination(
    drivebase,
    positionTracking,
    recoverTopLeftDestinationTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointCommand recover_top_right_reset(
    drivebase,
    positionTracking,
    recoverTopRightResetTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointUntilYCommand recover_top_right_via(
    drivebase,
    positionTracking,
    recoverTopRightViaTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointCommand recover_top_right_destination(
    drivebase,
    positionTracking,
    recoverTopRightDestinationTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointCommand recover_bottom_right_reset(
    drivebase,
    positionTracking,
    recoverBottomRightResetTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointUntilXCommand recover_bottom_right_via(
    drivebase,
    positionTracking,
    recoverBottomRightViaTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointCommand recover_bottom_right_destination(
    drivebase,
    positionTracking,
    recoverBottomRightDestinationTarget,
    pointConfig,
    DriveToPointPID
);

static WaitAndScoreCommand wait_and_score(
    intake,
    scoreConfig
);

static SetBlocksBeforeScoringCommand set_blocks_before_scoring(
    intake,
    blocksBeforeScoringConfig
);

static SetBlocksBeforeScoringCommand reset_blocks_after_scoring(
    intake,
    resetBlocksAfterScoringConfig
);

static DriveToPointUntilYCommand drive_to_score_bottom_left(
    drivebase,
    positionTracking,
    scoreBottomLeftTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointUntilYCommand drive_to_score_bottom_right(
    drivebase,
    positionTracking,
    scoreBottomRightTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointUntilYCommand drive_to_score_top_left(
    drivebase,
    positionTracking,
    scoreTopLeftTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToPointUntilYCommand drive_to_score_top_right(
    drivebase,
    positionTracking,
    scoreTopRightTarget,
    pointConfig,
    DriveToPointPID
);

static DriveToXPositionCommand reverse_to_score_bottom_left(
    drivebase,
    positionTracking,
    scoreReverseBottomLeftTarget,
    scoreReverseXConfig,
    DriveToPointPID
);

static DriveToXPositionCommand reverse_to_score_bottom_right(
    drivebase,
    positionTracking,
    scoreReverseBottomRightTarget,
    scoreReverseXConfig,
    DriveToPointPID
);

static DriveToXPositionCommand reverse_to_score_top_left(
    drivebase,
    positionTracking,
    scoreReverseTopLeftTarget,
    scoreReverseXConfig,
    DriveToPointPID
);

static DriveToXPositionCommand reverse_to_score_top_right(
    drivebase,
    positionTracking,
    scoreReverseTopRightTarget,
    scoreReverseXConfig,
    DriveToPointPID
);

static WallAlignmentCommand align_to_bottom_left_wall(
    drivebase,
    positionTracking,
    leftDistanceAligner,
    rightDistanceAligner,
    wallBottomTarget,
    wallConfig,
    wallAlignmentPID
);

static WallAlignmentCommand align_to_bottom_right_wall(
    drivebase,
    positionTracking,
    leftDistanceAligner,
    rightDistanceAligner,
    wallBottomTarget,
    wallConfig,
    wallAlignmentPID
);

static WallAlignmentCommand align_to_top_left_wall(
    drivebase,
    positionTracking,
    leftDistanceAligner,
    rightDistanceAligner,
    wallTopTarget,
    wallConfig,
    wallAlignmentPID
);

static WallAlignmentCommand align_to_top_right_wall(
    drivebase,
    positionTracking,
    leftDistanceAligner,
    rightDistanceAligner,
    wallTopTarget,
    wallConfig,
    wallAlignmentPID
);

static InertialTurnCommand turn_for_bottom_left_wall_alignment(
    drivebase,
    inertialTurnTargetBottomWallAlignment,
    inertialTurnConfig,
    wallAlignmentPID
);

static InertialTurnCommand turn_for_bottom_right_wall_alignment(
    drivebase,
    inertialTurnTargetBottomWallAlignment,
    inertialTurnConfig,
    wallAlignmentPID
);

static InertialTurnCommand turn_for_top_left_wall_alignment(
    drivebase,
    inertialTurnTargetTopWallAlignment,
    inertialTurnConfig,
    wallAlignmentPID
);

static InertialTurnCommand turn_for_top_right_wall_alignment(
    drivebase,
    inertialTurnTargetTopWallAlignment,
    inertialTurnConfig,
    wallAlignmentPID
);

static InertialTurnCommand turn_for_left_scoring(
    drivebase,
    inertialTurnTargetLeftScoring,
    inertialTurnConfig,
    wallAlignmentPID
);

static InertialTurnCommand turn_for_right_scoring(
    drivebase,
    inertialTurnTargetRightScoring,
    inertialTurnConfig,
    wallAlignmentPID
);

static DriveStraightCommand drive_into_left_goal(
    drivebase,
    driveIntoGoalLeftTarget,
    driveIntoGoal,
    driveIntoGoalPID
);

static DriveStraightCommand drive_into_right_goal(
    drivebase,
    driveIntoGoalRightTarget,
    driveIntoGoal,
    driveIntoGoalPID
);

static DriveStraightCommand drive_out_of_left_goal(
    drivebase,
    driveOutOfGoalLeftTarget,
    driveIntoGoal,
    driveIntoGoalPID
);

static DriveStraightCommand drive_out_of_right_goal(
    drivebase,
    driveOutOfGoalRightTarget,
    driveIntoGoal,
    driveIntoGoalPID
);

// ------------------------------------------------------------
// Command groups
// ------------------------------------------------------------

static ParallelDeadlineGroup track_and_intake(&track_block);
static ParallelDeadlineGroup grab_and_intake(&grabBlock);
static SequentialCommandGroup track_and_grab_sequence;
static SequentialCommandGroup recover_bottom_left_to_top_left;
static SequentialCommandGroup recover_top_left_to_top_right;
static SequentialCommandGroup recover_top_right_to_bottom_right;
static SequentialCommandGroup recover_bottom_right_to_bottom_left;
static SequentialCommandGroup corner_recovery_sequence;
static SequentialCommandGroup score_sequence;
static SequentialCommandGroup score_bottom_left_sequence;
static SequentialCommandGroup score_bottom_right_sequence;
static SequentialCommandGroup score_top_left_sequence;
static SequentialCommandGroup score_top_right_sequence;

static ConditionalCommandGroup choose_grab_or_skip(
    trackBlockWasSuccessful,
    &grab_and_intake,
    &skip_grab
);

static DoNothingCommand skip_track_and_grab;
static DoNothingCommand skip_corner_recovery;
static DoNothingCommand skip_score;
static DoNothingCommand skip_score_drive_retry;

static ConditionalCommandGroup choose_track_and_grab_or_skip(
    findBlockWasSuccessful,
    &track_and_grab_sequence,
    &skip_track_and_grab
);

static ConditionalCommandGroup choose_top_corner_recovery(
    recoverySourceIsRight,
    &recover_top_right_to_bottom_right,
    &recover_top_left_to_top_right
);

static ConditionalCommandGroup choose_bottom_corner_recovery(
    recoverySourceIsRight,
    &recover_bottom_right_to_bottom_left,
    &recover_bottom_left_to_top_left
);

static ConditionalCommandGroup choose_adjacent_corner_recovery(
    recoverySourceIsTop,
    &choose_top_corner_recovery,
    &choose_bottom_corner_recovery
);

static ConditionalCommandGroup choose_corner_recovery_or_skip(
    shouldRecoverFromEmptyCorner,
    &corner_recovery_sequence,
    &skip_corner_recovery
);

static ConditionalCommandGroup choose_score_or_keep_collecting(
    hasEnoughBlocksToScore,
    &score_sequence,
    &skip_score
);

static ConditionalCommandGroup retry_bottom_left_score_drive_if_needed(
    scoreBottomLeftApproachIsClose,
    &skip_score_drive_retry,
    &drive_to_score_bottom_left
);

static ConditionalCommandGroup retry_bottom_right_score_drive_if_needed(
    scoreBottomRightApproachIsClose,
    &skip_score_drive_retry,
    &drive_to_score_bottom_right
);

static ConditionalCommandGroup retry_top_left_score_drive_if_needed(
    scoreTopLeftApproachIsClose,
    &skip_score_drive_retry,
    &drive_to_score_top_left
);

static ConditionalCommandGroup retry_top_right_score_drive_if_needed(
    scoreTopRightApproachIsClose,
    &skip_score_drive_retry,
    &drive_to_score_top_right
);

static ConditionalCommandGroup choose_top_score_corner(
    robotIsOnRightSide,
    &score_top_right_sequence,
    &score_top_left_sequence
);

static ConditionalCommandGroup choose_bottom_score_corner(
    robotIsOnRightSide,
    &score_bottom_right_sequence,
    &score_bottom_left_sequence
);

static ConditionalCommandGroup choose_score_corner(
    robotIsOnTopSide,
    &choose_top_score_corner,
    &choose_bottom_score_corner
);

// ------------------------------------------------------------
// Routine assembly helpers
// ------------------------------------------------------------

static void add_scoring_sequence(
    SequentialCommandGroup& sequence,
    Command* driveToScore,
    Command* retryDriveIfNeeded,
    Command* turnForWallAlignment,
    Command* alignToWall,
    Command* turnForScoring,
    Command* reverseToScore,
    Command* driveIntoGoal,
    Command* driveOutOfGoal
) {
    sequence.addCommand(driveToScore);
    sequence.addCommand(&updateGPS);
    sequence.addCommand(&setPoseFromGPS);
    sequence.addCommand(retryDriveIfNeeded);
    sequence.addCommand(turnForWallAlignment);
    sequence.addCommand(alignToWall);
    sequence.addCommand(&updateGPSBeforeScoringTurn);
    sequence.addCommand(&setPoseFromGPSBeforeScoringTurn);
    sequence.addCommand(turnForScoring);
    sequence.addCommand(reverseToScore);
    sequence.addCommand(driveIntoGoal);
    sequence.addCommand(&wait_and_score);
    sequence.addCommand(driveOutOfGoal);
    sequence.addCommand(&reset_blocks_after_scoring);
}

static void add_collection_and_scoring_loop() {
    AI_ROUTE_1.addCommand(&set_blocks_before_scoring);

    AI_ROUTE_1.addCommand(&updateGPS);
    AI_ROUTE_1.addCommand(&setPoseFromGPS);

    AI_ROUTE_1.addCommand(&choose_corner_recovery_or_skip);
    AI_ROUTE_1.addCommand(&find_block);

    AI_ROUTE_1.addCommand(&updateGPS);
    AI_ROUTE_1.addCommand(&setPoseFromGPS);

    AI_ROUTE_1.addCommand(&choose_track_and_grab_or_skip);
    AI_ROUTE_1.addCommand(&choose_score_or_keep_collecting);
}

// ------------------------------------------------------------
// Build routine
// ------------------------------------------------------------

void build_interaction_routine() {
    static bool built = false;

    if (built) {
        return;
    }

    built = true;

    findBlockPtr = &find_block;
    trackBlockPtr = &track_block;

    track_and_intake.addCommand(&intake_while_tracking);
    grab_and_intake.addCommand(&intake_while_grabbing);
    track_and_grab_sequence.addCommand(&track_and_intake);
    track_and_grab_sequence.addCommand(&choose_grab_or_skip);

    recover_bottom_left_to_top_left.addCommand(&recover_bottom_left_reset);
    recover_bottom_left_to_top_left.addCommand(&recover_bottom_left_via);
    recover_bottom_left_to_top_left.addCommand(&recover_bottom_left_destination);

    recover_top_left_to_top_right.addCommand(&recover_top_left_reset);
    recover_top_left_to_top_right.addCommand(&recover_top_left_via);
    recover_top_left_to_top_right.addCommand(&recover_top_left_destination);

    recover_top_right_to_bottom_right.addCommand(&recover_top_right_reset);
    recover_top_right_to_bottom_right.addCommand(&recover_top_right_via);
    recover_top_right_to_bottom_right.addCommand(&recover_top_right_destination);

    recover_bottom_right_to_bottom_left.addCommand(&recover_bottom_right_reset);
    recover_bottom_right_to_bottom_left.addCommand(&recover_bottom_right_via);
    recover_bottom_right_to_bottom_left.addCommand(&recover_bottom_right_destination);

    corner_recovery_sequence.addCommand(&updateGPS);
    corner_recovery_sequence.addCommand(&setPoseFromGPS);
    corner_recovery_sequence.addCommand(&choose_adjacent_corner_recovery);

    add_scoring_sequence(
        score_bottom_left_sequence,
        &drive_to_score_bottom_left,
        &retry_bottom_left_score_drive_if_needed,
        &turn_for_bottom_left_wall_alignment,
        &align_to_bottom_left_wall,
        &turn_for_left_scoring,
        &reverse_to_score_bottom_left,
        &drive_into_left_goal,
        &drive_out_of_left_goal
    );

    add_scoring_sequence(
        score_bottom_right_sequence,
        &drive_to_score_bottom_right,
        &retry_bottom_right_score_drive_if_needed,
        &turn_for_bottom_right_wall_alignment,
        &align_to_bottom_right_wall,
        &turn_for_right_scoring,
        &reverse_to_score_bottom_right,
        &drive_into_right_goal,
        &drive_out_of_right_goal
    );

    add_scoring_sequence(
        score_top_left_sequence,
        &drive_to_score_top_left,
        &retry_top_left_score_drive_if_needed,
        &turn_for_top_left_wall_alignment,
        &align_to_top_left_wall,
        &turn_for_left_scoring,
        &reverse_to_score_top_left,
        &drive_into_left_goal,
        &drive_out_of_left_goal
    );

    add_scoring_sequence(
        score_top_right_sequence,
        &drive_to_score_top_right,
        &retry_top_right_score_drive_if_needed,
        &turn_for_top_right_wall_alignment,
        &align_to_top_right_wall,
        &turn_for_right_scoring,
        &reverse_to_score_top_right,
        &drive_into_right_goal,
        &drive_out_of_right_goal
    );

    score_sequence.addCommand(&updateGPS);
    score_sequence.addCommand(&setPoseFromGPS);
    score_sequence.addCommand(&choose_score_corner);

    add_collection_and_scoring_loop();
}
