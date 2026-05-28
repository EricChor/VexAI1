#include "Drivetrain.h"
#include "RobotConfig.h"
#include <iostream>

Pose Drivetrain::get_pose() const{
    return pose;
}

Drivetrain::Drivetrain (float driveBaseWheelDiameter, float driveBaseGearRatio)
:
 left_front(::left_front),
 left_back_bottom(::left_back_bottom),
 left_back_top(::left_back_top),

 right_front(::right_front),
 right_back_bottom(::right_back_bottom),
 right_back_top(::right_back_top),

 left_drive(::left_drive),
 right_drive(::right_drive),

 drivetrain_inertial(::drivetrain_inertial),

 driveBaseWheelDiameter(driveBaseWheelDiameter),
 driveBaseGearRatio(driveBaseGearRatio),

 left_drive_power(0),
 right_drive_power(0)
{}

void Drivetrain::set_odom_pose(float x, float y, float theta) {
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
}

void Drivetrain::update_odom_pose(){
    pose.theta = get_heading_degrees();
}

void Drivetrain::set_drive_power(float left_power, float right_power){
    left_drive_power = left_power;
    right_drive_power = right_power;
}
        
void Drivetrain::apply_motor_power(){
    left_drive.spin(fwd,left_drive_power,pct);
    right_drive.spin(fwd,right_drive_power,pct);
}
        
void Drivetrain::stop(){
    left_drive_power = 0;
    right_drive_power = 0;
    left_drive.stop(brake);
    right_drive.stop(brake);
}

float Drivetrain::get_heading_degrees(){
    return drivetrain_inertial.heading(degrees);
}

float Drivetrain::get_drivebase_wheel_diameter(){
    return driveBaseWheelDiameter;
}

float Drivetrain::get_drivebase_gear_ratio(){
    return driveBaseGearRatio;
}

float Drivetrain::get_left_front_motor_position(){
    return left_front.position(degrees);
}