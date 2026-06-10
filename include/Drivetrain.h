#pragma once
#include "vex.h"
using namespace vex;

struct Pose{
    float x;
    float y;
    float theta;
};

struct DrivePID{
    float linear_kP;
    float linear_kI;
    float linear_kD;

    float angular_kP;
    float angular_kI;
    float angular_kD;

    float linear_integral_windup_threshold;
    float angular_integral_windup_threshold;
};

class Drivetrain{
    private:
        DrivePID PID;
        Pose pose;

        motor& left_front;
        motor& left_back_bottom;
        motor& left_back_top;

        motor& right_front;
        motor& right_back_bottom;
        motor& right_back_top;

        motor_group& left_drive;
        motor_group& right_drive;

        inertial& drivetrain_inertial;

        float driveBaseWheelDiameter; //diameter in inches
        float driveBaseGearRatio;

        float left_drive_power;
        float right_drive_power;
    public:
        Pose get_pose() const;
        Drivetrain (float driveBaseWheelDiameter, float driveBaseGearRatio);
        void set_odom_pose(float x, float y, float theta);
        void set_heading_degrees(float theta);
        void update_odom_pose();

        void set_drive_power(float left_power, float right_power);
        void apply_motor_power();
        void stop();


        float get_heading_degrees();
        float get_drivebase_wheel_diameter();
        float get_drivebase_gear_ratio();
        float get_left_front_motor_position();
        float get_right_front_motor_position();
};
