#include "IsolationRoutine.h"

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

SequentialCommandGroup AI_ISOLATION_ROUTE;

static ParallelDeadlineGroup FirstTowerAndIntake;

DriveToPointTarget FirstTowerAndIntakeDriveTarget{
    .target_x = 24,
    .target_y = 24
};

DriveToPointTarget SecondTowerAndIntakeDriveTarget{
    .target_x = 48,
    .target_y = 48
};

DriveToPointConfig DriveToPointDefaultConfig{
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

DrivePID defaultPID {
    .linear_kP = 0.25,
    .linear_kI = 0.0,
    .linear_kD = 0.05,

    .angular_kP = 0.35,
    .angular_kI = 0.0,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 20,
    .linear_integral_windup_threshold = 20
};

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

InertialTurnTarget turnForWallAlignmentTarget{
    .target_heading = 180
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

/////////////////////////////////////////////////////////////////
static DriveToPointCommand FirstTowerAndIntakeDriveCommand(drivebase,
    positionTracking,
    FirstTowerAndIntakeDriveTarget,
    DriveToPointDefaultConfig, 
    defaultPID);

static DriveToPointCommand SecondTowerAndIntakeDriveCommand(drivebase,
    positionTracking,
    SecondTowerAndIntakeDriveTarget,
    DriveToPointDefaultConfig,
    defaultPID);

static IntakeWithSorting defaultIntake(intake, intakeSortingConfig);

static InertialTurnCommand turnForWallAlignment(drivebase,turnForWallAlignmentTarget, inertialTurnConfig, defaultPID);

static WallAlignmentCommand topWallAlignmentForLoading(drivebase,positionTracking, leftDistanceAligner,rightDistanceAligner,wallTopTarget,wallConfig,defaultPID);
////////////////////////////////


static ParallelDeadlineGroup FirstTowerAndIntakeGroup(&FirstTowerAndIntakeDriveCommand);
static ParallelDeadlineGroup SecondTowerAndIntakeGroup(&SecondTowerAndIntakeDriveCommand);

void build_isolation_routine(){
    FirstTowerAndIntakeGroup.addCommand(&defaultIntake);
    SecondTowerAndIntakeGroup.addCommand(&defaultIntake);

    AI_ISOLATION_ROUTE.addCommand(&FirstTowerAndIntakeGroup);
    AI_ISOLATION_ROUTE.addCommand(&SecondTowerAndIntakeGroup);
    AI_ISOLATION_ROUTE.addCommand(&turnForWallAlignment);
    AI_ISOLATION_ROUTE.addCommand(&topWallAlignmentForLoading);
    
}
