#pragma once
#include <cmath>
#include <iostream>
#include "Command.h"
#include "Drivetrain.h"
#include "UnstuckArcHelper.h"


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
    UnstuckArcConfig unstuck;
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

class DriveStraightCommand : public Command{
    private:
        Drivetrain& drivetrain;
        DriveStraightConfig driveStraightConfig;
        DriveStraightTarget driveStraightTarget;
        DriveStraightVar driveStraightVar;
        DrivePID PID;
        UnstuckArcHelper unstuck;

        bool finished;

    public:
        DriveStraightCommand(Drivetrain& drivetrain, DriveStraightTarget driveStraightTarget, DriveStraightConfig driveStraightConfig, DrivePID PID)
            :drivetrain(drivetrain),
             driveStraightTarget(driveStraightTarget),
             driveStraightConfig(driveStraightConfig),
             PID(PID),
             finished(false) {}
        
        void initialize() override{
            finished = false;
            DriveStraightVar& var = driveStraightVar;

            var.initial_position = drivetrain.get_left_front_motor_position();
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

            unstuck.configure(driveStraightConfig.unstuck);
            unstuck.initialize(
                drivetrain,
                std::fabs(driveStraightTarget.target_distance)
            );
            
        }

        void execute() override{
            DriveStraightVar& var = driveStraightVar;
            DriveStraightConfig& conf = driveStraightConfig;
            DriveStraightTarget& target = driveStraightTarget;
            
            var.current_heading = drivetrain.get_heading_degrees();
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
            var.current_position = ((drivetrain.get_left_front_motor_position() - var.initial_position) * drivetrain.get_drivebase_wheel_diameter() * M_PI / 360.0f * (1.0f/drivetrain.get_drivebase_gear_ratio()));
            // std::cout << "Distance: " << driveStraightVar.current_position << std::endl;

            var.linear_error = target.target_distance - var.current_position;
            var.linear_derivative_error = var.linear_error - var.linear_previous_error;
            var.linear_previous_error = var.linear_error;

            float progressError = std::fabs(var.linear_error);

            var.current_time = master_timer.time();

            if (var.current_time >= var.end_time) {
                var.timed_out = true;
                finished = true;
                return;
            }

            if (unstuck.run(drivetrain, progressError)) {
                return;
            }

            if (unstuck.hasFailed()) {
                var.timed_out = true;
                finished = true;
                return;
            }

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

            bool tryingToMove =
                std::fabs(var.linear_error) > conf.acceptable_error &&
                (
                    std::fabs(var.left_drive_linear_velocity) > 5.0f ||
                    std::fabs(var.right_drive_linear_velocity) > 5.0f ||
                    std::fabs(var.left_drive_angular_velocity) > 5.0f ||
                    std::fabs(var.right_drive_angular_velocity) > 5.0f
                );

            if (unstuck.shouldStart(drivetrain, progressError, tryingToMove)) {
                if (!unstuck.start(drivetrain, progressError)) {
                    var.timed_out = true;
                    finished = true;
                }

                return;
            }

            drivetrain.set_drive_power(var.left_drive_linear_velocity+var.left_drive_angular_velocity,var.right_drive_linear_velocity+var.right_drive_angular_velocity);

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

            finished = var.at_target || var.timed_out;
        }

        bool isFinished() override{
            return finished;
        }

        void end() override{
            // std::cout << "Drive straight distance from target: " << drivetrain.driveStraightVar.linear_error << std::endl;
            drivetrain.stop();
        }
};
