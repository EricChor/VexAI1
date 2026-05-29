#include "Routines.h"

#include "RobotConfig.h"
#include "Drivetrain.h"
#include "Intake.h"
#include "PositionTracking.h"

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


// ------------------------------------------------------------
// Simple skip command
// ------------------------------------------------------------

class DoNothingCommand : public Command {
private:
    bool finished;

public:
    DoNothingCommand()
        : finished(false)
    {
    }

    void initialize() override {
        finished = true;
    }

    void execute() override {
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
    }
};

// ------------------------------------------------------------
// Set drivetrain pose from filtered GPS
// ------------------------------------------------------------

class SetDrivetrainPoseFromGPSCommand : public Command {
private:
    Drivetrain& drivetrain;
    PositionTracking& positionTracking;

    bool finished;

public:
    SetDrivetrainPoseFromGPSCommand(
        Drivetrain& drivetrain,
        PositionTracking& positionTracking
    )
        : drivetrain(drivetrain),
          positionTracking(positionTracking),
          finished(false)
    {
    }

    void initialize() override {
        GPSPose pose = positionTracking.getPose();

        drivetrain.set_odom_pose(
            pose.x,
            pose.y,
            pose.heading
        );

        finished = true;
    }

    void execute() override {
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
    }
};

// ------------------------------------------------------------
// Track-block success wrapper
// ------------------------------------------------------------

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

// ------------------------------------------------------------
// Main routine objects
// ------------------------------------------------------------

SequentialCommandGroup AI_ROUTE_1;
RepeatForeverCommandGroup AI_ROUTE_ONE(AI_ROUTE_1);

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

    .angular_kP = 0.5,
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
    .linear_kP = 0.25,
    .linear_kI = 0.0,
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

    .reverseSpeed = 15,
    .forwardSpeed = 15,

    .stuckCheckTime = 1000,

    .stuckHeadingChangeThreshold = 5,
    .stuckEncoderChangeThreshold = 20,

    .maxReverseTime = 2000,
    .maxForwardTime = 2000,

    .cameraHorizontalFovDegrees = 60,

    .avoidZones = {
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
            .maxY = 90.0f,
            .enabled = true
        },
        {
            .minX = -30.0f,
            .maxX = 30.0f,
            .minY = -90.0f,
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
            .maxY = 90.0f,
            .enabled = true
        },
        {
            .minX = -30.0f,
            .maxX = 30.0f,
            .minY = -90.0f,
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

InertialTurnTarget inertialTurnTargetWallAlignment{
    .target_heading = 0
};

InertialTurnTarget inertialTurnTargeScoring{
    .target_heading = 270
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
    positionTracking
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

DriveToPointUntilYTarget scoreUntilY {
    .target_x = -48,
    .target_y = -48,
    .exit_y = -48,
    .exit_direction = DRIVE_TO_POINT_EXIT_BELOW_Y
};

DriveToPointTarget pointTarget {
    .target_x = -48,
    .target_y = -48
};

DriveToPointConfig pointConfig {
    .max_linear_speed = 30,
    .max_angular_speed = 25,
    .position_acceptable_error = 2,
    .heading_acceptable_error = 3,
    .max_time = 50000,
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
    .max_unstuck_attempts = 1
};

WallAlignmentTarget wallTarget {
    .target_heading = 0,
    .target_distance = 21
};

WallAlignmentConfig wallConfig {
    .max_linear_speed = 25,
    .max_angular_speed = 20,
    .acceptable_error = 0.5,
    .max_time = 30000,
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

DriveStraightTarget driveIntoGoalTarget{
    .target_distance = -12,
    .target_heading = 270
};

WaitAndScoreConfig scoreConfig {
    .wait_time = 250,
    .score_time = 2000,
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

static WallAlignmentCommand align_to_wall(
    drivebase,
    positionTracking,
    leftDistanceAligner,
    rightDistanceAligner,
    wallTarget,
    wallConfig,
    wallAlignmentPID
);



static DriveToPointUntilYCommand drive_to_score(drivebase,positionTracking,scoreUntilY,pointConfig,DriveToPointPID);
// ------------------------------------------------------------
// Command groups
// ------------------------------------------------------------

static ParallelDeadlineGroup track_and_intake(&track_block);
static ParallelDeadlineGroup grab_and_intake(&grabBlock);
static SequentialCommandGroup score_sequence;

static ConditionalCommandGroup choose_grab_or_skip(
    trackBlockWasSuccessful,
    &grab_and_intake,
    &skip_grab
);

static DoNothingCommand skip_score;

static ConditionalCommandGroup choose_score_or_keep_collecting(
    hasEnoughBlocksToScore,
    &score_sequence,
    &skip_score
);

static InertialTurnCommand TurnForWallAlignment(drivebase,inertialTurnTargetWallAlignment,inertialTurnConfig,wallAlignmentPID);
static InertialTurnCommand TurnForScoring(drivebase,inertialTurnTargeScoring,inertialTurnConfig,wallAlignmentPID);

static DriveStraightCommand driveIntoGoalToScore(drivebase,driveIntoGoalTarget,driveIntoGoal,driveIntoGoalPID);

// ------------------------------------------------------------
// Build routine
// ------------------------------------------------------------

void build_AI_routine() {
    static bool built = false;

    if (built) {
        return;
    }

    built = true;

    trackBlockPtr = &track_block;

    track_and_intake.addCommand(&intake_while_tracking);
    grab_and_intake.addCommand(&intake_while_grabbing);

    score_sequence.addCommand(&drive_to_score);
    score_sequence.addCommand(&TurnForWallAlignment);
    score_sequence.addCommand(&align_to_wall);
    score_sequence.addCommand(&TurnForScoring);
    score_sequence.addCommand(&driveIntoGoalToScore);
    score_sequence.addCommand(&wait_and_score);
    score_sequence.addCommand(&reset_blocks_after_scoring);

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
