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

struct DriveStraightTarget{
    float target_distance;
    float target_heading;
};

struct DriveStraightConfig {
    float max_linear_speed;
    float max_angular_speed;
    float acceptable_error;
    float max_time;
    float max_accel;
    float settle_time;
};

struct DriveStraightVar{
    float end_time;
    float current_heading;
    float linear_error;
    float linear_derivative_error;
    float linear_previous_error;
    float linear_integral_error;
    float angular_error;
    float angular_derivative_error;
    float angular_previous_error;
    float angular_integral_error;
    float initial_position;
    float current_position;
    float left_drive_angular_velocity;
    float right_drive_angular_velocity;
    float left_drive_linear_velocity;
    float right_drive_linear_velocity;
    float prev_left_drive_linear_velocity;
    float prev_right_drive_linear_velocity;
    float max_accel;
    float inside_threshold_timer;
    float current_time;

    bool threshold_timer_set;
    bool at_target;
    bool timed_out;
};

struct InertialTurnTarget{
    float target_heading;
};

struct InertialTurnConfig{
    float max_speed;
    float max_time;
    float settle_error;
    float settle_time;
    float settle_speed;
};

struct InertialTurnVar{
    float end_time;
    float current_heading;
    float error;
    float derivative_error;
    float previous_error;
    float integral_error;
    float settle_speed;
    float left_drive_velocity;
    float right_drive_velocity;

    float inside_threshold_timer;
    float current_time;

    bool threshold_timer_set;
    bool at_target;
    bool timed_out;
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

        DriveStraightTarget driveStraightTarget;
        DriveStraightConfig driveStraightConfig;

        InertialTurnTarget inertialTurnTarget;
        InertialTurnConfig inertialTurnConfig;

        inertial& drivetrain_inertial;

        float driveBaseWheelDiameter; //diameter in inches
        float driveBaseGearRatio;

        float left_drive_power;
        float right_drive_power;
    public:

        DriveStraightVar driveStraightVar;
        InertialTurnVar inertialTurnVar;

        Pose get_pose() const;
        Drivetrain (float driveBaseWheelDiameter, float driveBaseGearRatio);
        void set_odom_pose(float x, float y, float theta);
        void update_odom_pose();

        void set_drive_power(float left_power, float right_power);
        void apply_motor_power();
        void stop();

        void drive_straight_init(const DriveStraightConfig& config, const DriveStraightTarget& target, const DrivePID& drivePID);
        bool drive_straight_step();

        void inertial_turn_init(const InertialTurnConfig& config, const InertialTurnTarget& target, const DrivePID& drivePID);
        bool inertial_turn_step();

};