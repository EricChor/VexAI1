#include "Routines.h"
#include "Drivetrain.h"
#include "SequentialCommandGroup.h"
#include "ParallelCommandGroup.h"
#include "DriveStraightCommand.h"
#include "InertialTurnCommand.h"
#include "RobotConfig.h"

DrivePID defaultPID{
    .linear_kP = 1.5,
    .linear_kI = 0.0,
    .linear_kD = 0.2,
    .angular_kP = 0.25,
    .angular_kI = 0,
    .angular_kD = 0.2,
    .angular_integral_windup_threshold = 4,
    .linear_integral_windup_threshold = 2
};

DriveStraightConfig driveStraightConfigOne{
    .max_linear_speed = 40,
    .max_angular_speed = 30,
    .acceptable_error = 2,
    .max_time = 30000,
    .max_accel = 100,
    .settle_time = 250
};

InertialTurnConfig inertialTurnConfigOne{
    .max_speed = 30,
    .max_time = 3000,
    .settle_error = 3,
    .settle_speed = 50,
    .settle_time = 250
};

SequentialCommandGroup route_one;

void build_routine_one(){
    static DriveStraightTarget drive_one_target{
        .target_distance = 24,
        .target_heading = 0
    };

    static DriveStraightTarget drive_two_target{
        .target_distance = 24,
        .target_heading = 90
    };

    static InertialTurnTarget turn_one_target{
        .target_heading = 90
    };

    static InertialTurnTarget turn_two_target{
        .target_heading = 180
    };

    static DriveStraightCommand drive_one(drivebase,drive_one_target,driveStraightConfigOne,defaultPID);
    static InertialTurnCommand turn_one(drivebase,turn_one_target,inertialTurnConfigOne,defaultPID);

    static DriveStraightCommand drive_two(drivebase,drive_two_target,driveStraightConfigOne,defaultPID);
    static InertialTurnCommand turn_two(drivebase,turn_two_target, inertialTurnConfigOne,defaultPID);


    route_one.addCommand(&drive_one);
    route_one.addCommand(&turn_one);
    route_one.addCommand(&drive_two);
    route_one.addCommand(&turn_two);
}

