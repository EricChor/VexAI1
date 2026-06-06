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
#include "ChangeLoaderStateCommand.h"

#include "DoNothingCommand.h"
#include "SetDrivetrainPoseFromGPSCommand.h"

#include <cmath>

SequentialCommandGroup AI_ISOLATION_ROUTE;

static DriveToPointTarget FirstTowerAndIntakeDriveTarget{
    .target_x = 72,
    .target_y = 72
};

static DriveToPointUntilYTarget SecondTowerAndIntakeDriveTarget{
    .target_x = 43,
    .target_y = 48,
    .exit_y = 48,
    .exit_direction = DRIVE_TO_POINT_EXIT_ABOVE_Y
};

static DriveToPointConfig DriveToPointDefaultConfig{
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
    .max_unstuck_attempts = 1,
    .drive_direction = DRIVE_TO_POINT_DRIVE_FORWARD
};

static DrivePID isolationDefaultPID {
    .linear_kP = 2,
    .linear_kI = 0.0,
    .linear_kD = 0.1,

    .angular_kP = 0.3,
    .angular_kI = 0.0,
    .angular_kD = 0.004,

    .angular_integral_windup_threshold = 20,
    .linear_integral_windup_threshold = 20
};

static DrivePID isolationDriveToPointPID{
    .linear_kP = 2,
    .linear_kI = 0.0,
    .linear_kD = 0.1,

    .angular_kP = 0.2,
    .angular_kI = 0.0,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 20,
    .linear_integral_windup_threshold = 20
};

static DrivePID isolationDriveIntoGoalPID{
    .linear_kP = 3,
    .linear_kI = 0.0,
    .linear_kD = 0.25,

    .angular_kP = 0.05,
    .angular_kI = 0.05,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 15,
    .linear_integral_windup_threshold = 15
};

static IntakeWithSortingConfig isolationIntakeSortingConfig {
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

static InertialTurnTarget isolationTurnForWallAlignmentTarget{
    .target_heading = 180
};

static InertialTurnTarget isolationTurnForLoaderWallTarget{
    .target_heading = 90
};

static InertialTurnConfig isolationInertialTurnConfig{
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

static WallAlignmentTarget isolationWallTopTarget {
    .target_heading = 180,
    .target_distance = 16
};

static WallAlignmentConfig isolationWallConfig {
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

static DriveStraightConfig isolationDriveIntoLoaderWallConfig{
    .max_linear_speed = 25,
    .max_angular_speed = 20,
    .acceptable_error = 1,
    .max_time = 2000,
    .max_accel = 0,
    .settle_time = 100,
    .unstuck = {
        .max_attempts = 0
    }
};

static DriveStraightTarget isolationDriveIntoLoaderWallTarget{
    .target_distance = 30,
    .target_heading = 90
};

static DriveToPointConfig isolationReverseToHighGoalConfig{
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

static DriveToPointUntilXTarget isolationReverseToHighGoalTarget{
    .target_x = 36,
    .target_y = 48,
    .exit_x = 36,
    .exit_direction = DRIVE_TO_POINT_EXIT_LEFT_OF_X
};

static DriveStraightConfig isolationDriveIntoHighGoalConfig{
    .max_linear_speed = 25,
    .max_angular_speed = 25,
    .acceptable_error = 1,
    .max_time = 3000,
    .max_accel = 0,
    .settle_time = 250,
    .unstuck = {
        .max_attempts = 0
    }
};

static DriveStraightTarget isolationDriveIntoHighGoalTarget{
    .target_distance = -15,
    .target_heading = 90
};

static WaitAndScoreConfig isolationScoreHighConfig{
    .wait_time = 250,
    .score_time = 3000,
    .score_mode = SCORE_HIGH
};

/////////////////////////////////////////////////////////////////
static DriveToPointCommand FirstTowerAndIntakeDriveCommand(drivebase,
    positionTracking,
    FirstTowerAndIntakeDriveTarget,
    DriveToPointDefaultConfig, 
    isolationDefaultPID);

static DriveToPointUntilYCommand SecondTowerAndIntakeDriveCommand(drivebase,
    positionTracking,
    SecondTowerAndIntakeDriveTarget,
    DriveToPointDefaultConfig,
    isolationDefaultPID);

static IntakeWithSorting defaultIntake(intake, isolationIntakeSortingConfig);
static IntakeWithSorting intakeWhileLoading(intake, isolationIntakeSortingConfig);

static InertialTurnCommand turnForWallAlignment(drivebase,isolationTurnForWallAlignmentTarget, isolationInertialTurnConfig, isolationDefaultPID);

static WallAlignmentCommand topWallAlignmentForLoading(drivebase,positionTracking, leftDistanceAligner,rightDistanceAligner,isolationWallTopTarget,isolationWallConfig,isolationDefaultPID);

static ChangeLoaderStateCommand lowerLoader(loaderPiston, false);

static InertialTurnCommand turnForLoaderWall(drivebase,isolationTurnForLoaderWallTarget, isolationInertialTurnConfig, isolationDefaultPID);

static DriveStraightCommand driveIntoLoaderWall(drivebase, isolationDriveIntoLoaderWallTarget, isolationDriveIntoLoaderWallConfig, isolationDriveIntoGoalPID);

static DriveToPointUntilXCommand reverseToHighGoal(drivebase, positionTracking, isolationReverseToHighGoalTarget, isolationReverseToHighGoalConfig, isolationDriveToPointPID);

static DriveStraightCommand driveIntoHighGoal(drivebase, isolationDriveIntoHighGoalTarget, isolationDriveIntoHighGoalConfig, isolationDriveIntoGoalPID);

static WaitAndScoreCommand scoreHighGoal(intake, isolationScoreHighConfig);
////////////////////////////////


static ParallelDeadlineGroup FirstTowerAndIntakeGroup(&FirstTowerAndIntakeDriveCommand);
static ParallelDeadlineGroup SecondTowerAndIntakeGroup(&SecondTowerAndIntakeDriveCommand);
static ParallelDeadlineGroup loadFromRightWallGroup(&driveIntoLoaderWall);

void build_isolation_routine(){
    static bool built = false;
    if (built) {
        return;
    }
    built = true;

    FirstTowerAndIntakeGroup.addCommand(&defaultIntake);
    SecondTowerAndIntakeGroup.addCommand(&defaultIntake);
    loadFromRightWallGroup.addCommand(&intakeWhileLoading);

    AI_ISOLATION_ROUTE.addCommand(&FirstTowerAndIntakeGroup);
    AI_ISOLATION_ROUTE.addCommand(&turnForWallAlignment);
    AI_ISOLATION_ROUTE.addCommand(&topWallAlignmentForLoading);
    AI_ISOLATION_ROUTE.addCommand(&lowerLoader);
    AI_ISOLATION_ROUTE.addCommand(&turnForLoaderWall);
    AI_ISOLATION_ROUTE.addCommand(&loadFromRightWallGroup);
    AI_ISOLATION_ROUTE.addCommand(&reverseToHighGoal);
    AI_ISOLATION_ROUTE.addCommand(&driveIntoHighGoal);
    AI_ISOLATION_ROUTE.addCommand(&scoreHighGoal);
    
}

/////////////////////////////////////////////////////////////////////////////////////////////
SequentialCommandGroup AI_ISO_ROUTE;
DriveStraightTarget DST1{
    .target_distance =32.5, .target_heading = 260

};
DriveStraightConfig DSC1{
    .max_angular_speed = 30, .max_linear_speed = 50, .max_time = 3000, .max_accel= 70, .acceptable_error = 2,
     .settle_time = 50, 
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
InertialTurnConfig ITCg1{
    .max_speed = 60, .max_time = 3000, .settle_error = 2,
    .settle_speed = 30, .settle_time = 100,
    .unstuck ={
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
InertialTurnTarget ITT1{
    .target_heading = 128
};

DriveStraightCommand DS1(drivebase,DST1, DSC1, isolationDefaultPID);
IntakeWithSorting IWS1(intake, isolationIntakeSortingConfig);
ParallelDeadlineGroup ISO_TOWER_ONE(&DS1);
InertialTurnCommand ITC1(drivebase,ITT1,ITCg1,isolationDefaultPID);

void build_iso_route(){
ISO_TOWER_ONE.addCommand(&IWS1);

AI_ISO_ROUTE.addCommand(&ISO_TOWER_ONE);
AI_ISO_ROUTE.addCommand(&ITC1);
}



