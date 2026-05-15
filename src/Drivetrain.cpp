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
 driveBaseGearRatio(driveBaseGearRatio)
{}

void Drivetrain::set_odom_pose(float x, float y, float theta) {
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
}

void Drivetrain::update_odom_pose(){

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

void Drivetrain::drive_straight_init(const DriveStraightConfig& config, const DriveStraightTarget& target, const DrivePID& drivePID){
    driveStraightConfig = config;
    driveStraightTarget = target;
    PID = drivePID;

    DriveStraightVar& var = driveStraightVar;

    var.initial_position = left_front.position(degrees);
    var.current_position = var.initial_position;
    var.end_time = master_timer.time() + driveStraightConfig.max_time;
    var.max_accel = driveStraightConfig.max_accel / 100;

    var.linear_error = 0;
    var.linear_derivative_error = 0;
    var.linear_previous_error = 0;
    var.linear_integral_error = 0;

    var.angular_error = 0;
    var.angular_derivative_error = 0;
    var.angular_previous_error = 0;
    var.angular_integral_error = 0;

    var.left_drive_angular_velocity = 0;
    var.right_drive_angular_velocity = 0;
    var.left_drive_linear_velocity = 0;
    var.right_drive_linear_velocity = 0;

    var.prev_left_drive_linear_velocity = 0;
    var.prev_right_drive_linear_velocity = 0;

    var.inside_threshold_timer = 0;
    var.threshold_timer_set = false;
    var.at_target = false;
    var.timed_out = false;

}

bool Drivetrain::drive_straight_step(){
    DriveStraightVar& var = driveStraightVar;
    DriveStraightConfig& conf = driveStraightConfig;
    DriveStraightTarget& target = driveStraightTarget;
    
    var.current_heading = drivetrain_inertial.heading(degrees);
    var.angular_error = target.target_heading - var.current_heading;
    if(fabs(var.angular_error) > 180){
        if(var.angular_error > 0){
            var.angular_error -= 360;
        } else {
            var.angular_error += 360;
        }
    }

    var.angular_derivative_error = var.angular_error - var.angular_previous_error;
    var.angular_previous_error = var.angular_error;
    var.current_position = ((left_front.position(degrees) - var.initial_position) * driveBaseWheelDiameter * M_PI / 360.0f * (1.0f/driveBaseGearRatio));
    // std::cout << "Distance: " << driveStraightVar.current_position << std::endl;

    var.linear_error = target.target_distance - var.current_position;
    var.linear_derivative_error = var.linear_error - var.linear_previous_error;
    var.linear_previous_error = var.linear_error;

    if(fabs(var.angular_error) <= PID.angular_integral_windup_threshold){
        var.angular_integral_error += var.angular_error;
    } else {
        var.angular_integral_error = 0;
    }

    if(fabs(var.linear_error) <= PID.linear_integral_windup_threshold){
        var.linear_integral_error += var.linear_error;
    } else {
        var.linear_integral_error = 0;
    }

    var.left_drive_angular_velocity = PID.angular_kP * var.angular_error + PID.angular_kI * var.angular_integral_error + PID.angular_kD * var.angular_derivative_error;
    var.right_drive_angular_velocity = -1*(PID.angular_kP * var.angular_error + PID.angular_kI * var.angular_integral_error + PID.angular_kD * var.angular_derivative_error);

    if(fabs(var.left_drive_angular_velocity) > conf.max_angular_speed){
        if(var.left_drive_angular_velocity > 0){
            var.left_drive_angular_velocity = conf.max_angular_speed;
        } else {
            var.left_drive_angular_velocity = -conf.max_angular_speed;
        }
    }

    if(fabs(var.right_drive_angular_velocity) > conf.max_angular_speed){
        if(var.right_drive_angular_velocity > 0){
            var.right_drive_angular_velocity = conf.max_angular_speed;
        } else {
            var.right_drive_angular_velocity = -conf.max_angular_speed;
        }
    }

    var.left_drive_linear_velocity = PID.linear_kP * var.linear_error + PID.linear_kI * var.linear_integral_error + PID.linear_kD * var.linear_derivative_error;
    var.right_drive_linear_velocity = PID.linear_kP * var.linear_error + PID.linear_kI * var.linear_integral_error + PID.linear_kD * var.linear_derivative_error;
    
    if(((var.left_drive_linear_velocity) > (var.prev_left_drive_linear_velocity + var.max_accel)) && (var.max_accel != 0)){
        var.left_drive_linear_velocity = var.prev_left_drive_linear_velocity + var.max_accel;
        var.right_drive_linear_velocity = var.prev_right_drive_linear_velocity + var.max_accel;
    }

    if(fabs(var.left_drive_linear_velocity) > conf.max_linear_speed){
        if(var.left_drive_linear_velocity > 0){
            var.left_drive_linear_velocity = conf.max_linear_speed;
        } else {
            var.left_drive_linear_velocity = -conf.max_linear_speed;
        }
    }

    if(fabs(var.right_drive_linear_velocity) > conf.max_linear_speed){
        if(var.right_drive_linear_velocity > 0){
            var.right_drive_linear_velocity = conf.max_linear_speed;
        } else {
            var.right_drive_linear_velocity = -conf.max_linear_speed;
        }
    }

    set_drive_power(var.left_drive_linear_velocity+var.left_drive_angular_velocity,var.right_drive_linear_velocity+var.right_drive_angular_velocity);

    var.current_time = master_timer.time();

    if(fabs(var.linear_error) < conf.acceptable_error){
        if(!var.threshold_timer_set){
            var.inside_threshold_timer = var.current_time + conf.settle_time;
            var.threshold_timer_set = true;
        }

        if(var.current_time > var.inside_threshold_timer){
            var.at_target = true;
        }
    } else {
        var.threshold_timer_set = false;
        var.at_target = false;
    }

    if(var.current_time >= var.end_time){
        var.timed_out = true;
    }

    var.prev_left_drive_linear_velocity = var.left_drive_linear_velocity;
    var.prev_right_drive_linear_velocity = var.right_drive_linear_velocity;

    return var.at_target || var.timed_out;
}

void Drivetrain::inertial_turn_init(const InertialTurnConfig& config, const InertialTurnTarget& target, const DrivePID& drivePID){
    inertialTurnConfig = config;
    inertialTurnTarget = target;
    PID = drivePID;
    InertialTurnVar& var = inertialTurnVar;

    var.end_time = master_timer.time() + inertialTurnConfig.max_time;
    var.current_heading = 0;
    var.derivative_error = 0;
    var.previous_error = 0;
    var.integral_error = 0;
    var.settle_speed = inertialTurnConfig.settle_speed / 100;
    var.left_drive_velocity = 0;
    var.right_drive_velocity = 0;
    var.inside_threshold_timer = 0;
    var.threshold_timer_set = false;
    var.at_target = false;
    var.timed_out = false;
}

bool Drivetrain::inertial_turn_step(){
    InertialTurnConfig& conf = inertialTurnConfig;
    InertialTurnVar& var = inertialTurnVar;
    InertialTurnTarget& target = inertialTurnTarget;
    
    var.current_heading = drivetrain_inertial.heading();
    var.error = target.target_heading - var.current_heading;
    
    if(fabs(var.error) > 180){
        if(var.error > 0){
            var.error -= 360;
        } else {
            var.error += 360;
        }
    }

    var.derivative_error = var.error - var.previous_error;
    var.previous_error = var.error;

    if(fabs(var.error) <= PID.angular_integral_windup_threshold){
        var.integral_error += var.error;
    } else {
        var.integral_error = 0;
    }

    var.left_drive_velocity = PID.angular_kP * var.error + PID.angular_kI * var.integral_error + PID.angular_kD * var.derivative_error;
    var.right_drive_velocity = PID.angular_kP * var.error + PID.angular_kI * var.integral_error + PID.angular_kD * var.derivative_error;

    if(fabs(var.left_drive_velocity) > conf.max_speed){
        if(var.left_drive_velocity > 0){
            var.left_drive_velocity = conf.max_speed;
        } else {
            var.left_drive_velocity = -conf.max_speed;
        }
    }

    if(fabs(var.right_drive_velocity) > conf.max_speed){
        if(var.right_drive_velocity > 0){
            var.right_drive_velocity = conf.max_speed;
        } else {
            var.right_drive_velocity = -conf.max_speed;
        }
    }
    
    set_drive_power(var.left_drive_velocity,-var.right_drive_velocity);

    var.current_time = master_timer.time();

    if((fabs(var.error) < conf.settle_error) && (fabs(var.derivative_error) < var.settle_speed)){
        if(!var.threshold_timer_set){
            var.inside_threshold_timer = var.current_time + conf.settle_time;
            var.threshold_timer_set = true;
        }

        if(var.current_time > var.inside_threshold_timer){
            var.at_target = true;
        }
    } else {
        var.threshold_timer_set = false;
        var.at_target = false;
    }

    if(var.current_time >= var.end_time){
        var.timed_out = true;
    }

    return var.at_target || var.timed_out;
}