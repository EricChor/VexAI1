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


static TrackBlockCommand* trackBlockPtr = nullptr;

bool trackBlockWasSuccessful() {
    if (trackBlockPtr == nullptr) {
        return false;
    }

    return trackBlockPtr->wasSuccessful();
}

bool hasEnoughBlocksToScore() {
    return intake.has_enough_blocks_to_score();
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
// Main routine objects
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
    .linear_kP = 0.65,
    .linear_kI = 0.01,
    .linear_kD = 0.05,

    .angular_kP = 0.4,
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
    .linear_kP = 0.275,
    .linear_kI = 0.01,
    .linear_kD = 0.05,

    .angular_kP = 0.4,
    .angular_kI = 0.0,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 20,
    .linear_integral_windup_threshold = 20
};

// ------------------------------------------------------------
// Raw Jetson vision configs
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
    .centeringAcceptableX = 15,

    .reverseSpeed = 20,
    .forwardSpeed = 20,
    .turnSpeed = 15,

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
        }
    },

    .avoidTurnTime = 500,

    .maxCenteringDroppedFrames = 5,
    .centeringXProgressThreshold = 5
};

// ------------------------------------------------------------
// Track block command config
// ------------------------------------------------------------

TrackingBlocksConfig trackBlockConfig {
    .acceptableYError = 10,
    .acceptableXError = 12,

    .maxLinearSpeed = 25,
    .maxAngularSpeed = 20,

    .stuckCheckTime = 750,
    .stuckEncoderChangeThreshold = 10,
    .stuckHeadingChangeThreshold = 3,

    .reverseSpeed = 20,
    .forwardSpeed = 20,
    .turnSpeed = 15,

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
        }

    },

};

// ------------------------------------------------------------
// Intake command config
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
    .max_time = 5000,
    .settle_error = 4,
    .settle_speed = 120,
    .settle_time = 250,
    .unstuck = {
        .stuck_check_time = 750,
        .heading_change_threshold = 3,
        .error_progress_threshold = 3,
        .forward_speed = 15,
        .reverse_speed = 15,
        .turn_speed = 10,
        .forward_time = 300,
        .reverse_time = 400,
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
// Grab block config
// ------------------------------------------------------------

GrabBlockConfig grabBlockConfig {
    .acceptable_error = 0.5,
    .max_accel = 150,
    .max_angular_speed = 25,
    .max_linear_speed = 40,
    .max_time = 5000,
    .settle_time = 250,
    .unstuck = {
        .stuck_check_time = 750,
        .encoder_change_threshold = 15,
        .error_progress_threshold = 1,
        .forward_speed = 20,
        .reverse_speed = 20,
        .turn_speed = 15,
        .forward_time = 300,
        .reverse_time = 500,
        .max_attempts = 1
    }
};

GrabBlockTarget grabBlockTarget {
    .target_distance = 10,
    .target_heading = 0
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

// ------------------------------------------------------------
// Commands
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

DriveToPointUntilYTarget scoreBottomLeftTarget {
    .target_x = -48,
    .target_y = -48,
    .exit_y = -48,
    .exit_direction = DRIVE_TO_POINT_EXIT_BELOW_Y
};

DriveToPointUntilYTarget scoreBottomRightTarget {
    .target_x = 48,
    .target_y = -48,
    .exit_y = -48,
    .exit_direction = DRIVE_TO_POINT_EXIT_BELOW_Y
};

DriveToPointUntilYTarget scoreTopLeftTarget {
    .target_x = -48,
    .target_y = 48,
    .exit_y = 48,
    .exit_direction = DRIVE_TO_POINT_EXIT_ABOVE_Y
};

DriveToPointUntilYTarget scoreTopRightTarget {
    .target_x = 48,
    .target_y = 48,
    .exit_y = 48,
    .exit_direction = DRIVE_TO_POINT_EXIT_ABOVE_Y
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

DriveToPointConfig pointConfig {
    .max_linear_speed = 30,
    .max_angular_speed = 25,
    .position_acceptable_error = 2,
    .heading_acceptable_error = 3,
    .max_time = 10000,
    .pointing_settle_time = 150,
    .position_settle_time = 200,
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
    .max_unstuck_attempts = 1,
    .drive_direction = DRIVE_TO_POINT_DRIVE_FORWARD
};

DriveToPointConfig pointReverseConfig {
    .max_linear_speed = 30,
    .max_angular_speed = 25,
    .position_acceptable_error = 2,
    .heading_acceptable_error = 3,
    .max_time = 10000,
    .pointing_settle_time = 150,
    .position_settle_time = 200,
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
    .max_unstuck_attempts = 1,
    .drive_direction = DRIVE_TO_POINT_DRIVE_BACKWARD
};

DriveToPointUntilXTarget scoreReverseBottomLeftTarget {
    .target_x = -36,
    .target_y = -48,
    .exit_x = -36,
    .exit_direction = DRIVE_TO_POINT_EXIT_LEFT_OF_X
};

DriveToPointUntilXTarget scoreReverseBottomRightTarget {
    .target_x = 36,
    .target_y = -48,
    .exit_x = 36,
    .exit_direction = DRIVE_TO_POINT_EXIT_RIGHT_OF_X
};

DriveToPointUntilXTarget scoreReverseTopLeftTarget {
    .target_x = -36,
    .target_y = 48,
    .exit_x = -36,
    .exit_direction = DRIVE_TO_POINT_EXIT_LEFT_OF_X
};

DriveToPointUntilXTarget scoreReverseTopRightTarget {
    .target_x = 36,
    .target_y = 48,
    .exit_x = 36,
    .exit_direction = DRIVE_TO_POINT_EXIT_RIGHT_OF_X
};

WallAlignmentTarget wallBottomTarget {
    .target_heading = 0,
    .target_distance = 18
};

WallAlignmentTarget wallTopTarget {
    .target_heading = 180,
    .target_distance = 18
};

WallAlignmentConfig wallConfig {
    .max_linear_speed = 25,
    .max_angular_speed = 20,
    .acceptable_error = 0.5,
    .heading_acceptable_error = 3,
    .max_linear_heading_error = 15,
    .max_time = 7000,
    .max_accel = 150,
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
        .max_attempts = 2
    }
};

DriveStraightConfig driveIntoGoal{
    .acceptable_error = 1,
    .max_accel = 0,
    .max_angular_speed = 25,
    .max_linear_speed = 25,
    .max_time = 3000,
    .settle_time = 250,
    .unstuck = {
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

static DriveToPointUntilXCommand reverse_to_score_bottom_left(
    drivebase,
    positionTracking,
    scoreReverseBottomLeftTarget,
    pointReverseConfig,
    DriveToPointPID
);

static DriveToPointUntilXCommand reverse_to_score_bottom_right(
    drivebase,
    positionTracking,
    scoreReverseBottomRightTarget,
    pointReverseConfig,
    DriveToPointPID
);

static DriveToPointUntilXCommand reverse_to_score_top_left(
    drivebase,
    positionTracking,
    scoreReverseTopLeftTarget,
    pointReverseConfig,
    DriveToPointPID
);

static DriveToPointUntilXCommand reverse_to_score_top_right(
    drivebase,
    positionTracking,
    scoreReverseTopRightTarget,
    pointReverseConfig,
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
// ------------------------------------------------------------
// Command groups
// ------------------------------------------------------------

static ParallelDeadlineGroup track_and_intake(&track_block);
static ParallelDeadlineGroup grab_and_intake(&grabBlock);
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

static DoNothingCommand skip_score;
static DoNothingCommand skip_score_drive_retry;

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
// Build routine
// ------------------------------------------------------------

void build_interaction_routine() {
    static bool built = false;

    if (built) {
        return;
    }

    built = true;

    trackBlockPtr = &track_block;

    track_and_intake.addCommand(&intake_while_tracking);
    grab_and_intake.addCommand(&intake_while_grabbing);

    score_bottom_left_sequence.addCommand(&drive_to_score_bottom_left);
    score_bottom_left_sequence.addCommand(&updateGPS);
    score_bottom_left_sequence.addCommand(&setPoseFromGPS);
    score_bottom_left_sequence.addCommand(&retry_bottom_left_score_drive_if_needed);
    score_bottom_left_sequence.addCommand(&turn_for_bottom_left_wall_alignment);
    score_bottom_left_sequence.addCommand(&align_to_bottom_left_wall);
    score_bottom_left_sequence.addCommand(&turn_for_left_scoring);
    score_bottom_left_sequence.addCommand(&reverse_to_score_bottom_left);
    score_bottom_left_sequence.addCommand(&drive_into_left_goal);
    score_bottom_left_sequence.addCommand(&wait_and_score);
    score_bottom_left_sequence.addCommand(&reset_blocks_after_scoring);

    score_bottom_right_sequence.addCommand(&drive_to_score_bottom_right);
    score_bottom_right_sequence.addCommand(&updateGPS);
    score_bottom_right_sequence.addCommand(&setPoseFromGPS);
    score_bottom_right_sequence.addCommand(&retry_bottom_right_score_drive_if_needed);
    score_bottom_right_sequence.addCommand(&turn_for_bottom_right_wall_alignment);
    score_bottom_right_sequence.addCommand(&align_to_bottom_right_wall);
    score_bottom_right_sequence.addCommand(&turn_for_right_scoring);
    score_bottom_right_sequence.addCommand(&reverse_to_score_bottom_right);
    score_bottom_right_sequence.addCommand(&drive_into_right_goal);
    score_bottom_right_sequence.addCommand(&wait_and_score);
    score_bottom_right_sequence.addCommand(&reset_blocks_after_scoring);

    score_top_left_sequence.addCommand(&drive_to_score_top_left);
    score_top_left_sequence.addCommand(&updateGPS);
    score_top_left_sequence.addCommand(&setPoseFromGPS);
    score_top_left_sequence.addCommand(&retry_top_left_score_drive_if_needed);
    score_top_left_sequence.addCommand(&turn_for_top_left_wall_alignment);
    score_top_left_sequence.addCommand(&align_to_top_left_wall);
    score_top_left_sequence.addCommand(&turn_for_left_scoring);
    score_top_left_sequence.addCommand(&reverse_to_score_top_left);
    score_top_left_sequence.addCommand(&drive_into_left_goal);
    score_top_left_sequence.addCommand(&wait_and_score);
    score_top_left_sequence.addCommand(&reset_blocks_after_scoring);

    score_top_right_sequence.addCommand(&drive_to_score_top_right);
    score_top_right_sequence.addCommand(&updateGPS);
    score_top_right_sequence.addCommand(&setPoseFromGPS);
    score_top_right_sequence.addCommand(&retry_top_right_score_drive_if_needed);
    score_top_right_sequence.addCommand(&turn_for_top_right_wall_alignment);
    score_top_right_sequence.addCommand(&align_to_top_right_wall);
    score_top_right_sequence.addCommand(&turn_for_right_scoring);
    score_top_right_sequence.addCommand(&reverse_to_score_top_right);
    score_top_right_sequence.addCommand(&drive_into_right_goal);
    score_top_right_sequence.addCommand(&wait_and_score);
    score_top_right_sequence.addCommand(&reset_blocks_after_scoring);

    score_sequence.addCommand(&updateGPS);
    score_sequence.addCommand(&setPoseFromGPS);
    score_sequence.addCommand(&choose_score_corner);

    AI_ROUTE_1.addCommand(&set_blocks_before_scoring);

    AI_ROUTE_1.addCommand(&updateGPS);
    AI_ROUTE_1.addCommand(&setPoseFromGPS);

    AI_ROUTE_1.addCommand(&find_block);

    AI_ROUTE_1.addCommand(&updateGPS);
    AI_ROUTE_1.addCommand(&setPoseFromGPS);

    AI_ROUTE_1.addCommand(&track_and_intake);

    AI_ROUTE_1.addCommand(&choose_grab_or_skip);

    AI_ROUTE_1.addCommand(&choose_score_or_keep_collecting);
}
