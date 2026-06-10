#pragma once
#include <cmath>
#include <iostream>
#include "Command.h"
#include "Drivetrain.h"
#include "RobotConfig.h"
#include "UnstuckArcHelper.h"
#include "CommandStatus.h"


struct InertialTurnTarget{
    float target_heading;
};

struct InertialTurnConfig{
    float max_speed;
    float min_speed;
    float max_time;
    float settle_error;
    float settle_time;
    float settle_speed;
    UnstuckArcConfig unstuck;
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


class InertialTurnCommand : public Command{
    private:
        Drivetrain& drivetrain;
        InertialTurnConfig inertialTurnConfig;
        InertialTurnTarget inertialTurnTarget;
        InertialTurnVar inertialTurnVar;
        DrivePID PID;
        UnstuckArcHelper unstuck;
        bool finished;

    public:
        InertialTurnCommand(Drivetrain& drivetrain, InertialTurnTarget& inertialTurnTarget, InertialTurnConfig& inertialTurnConfig, DrivePID& PID)
            :drivetrain(drivetrain),
             inertialTurnTarget(inertialTurnTarget),
             inertialTurnConfig(inertialTurnConfig),
             PID(PID),
             finished(false) {}
        
        void initialize() override{
            setCommandStatus("Inertial Turn");
            finished = false;
            InertialTurnVar& var = inertialTurnVar;

            var.end_time = master_timer.time() + inertialTurnConfig.max_time;
            var.current_heading = drivetrain.get_heading_degrees();
            var.error = inertialTurnTarget.target_heading - var.current_heading;

            if(fabs(var.error) > 180){
                if(var.error > 0){
                    var.error -= 360;
                } else {
                    var.error += 360;
                }
            }

            var.derivative_error = 0;
            var.previous_error = var.error;
            var.integral_error = 0;
            var.settle_speed = inertialTurnConfig.settle_speed / 100;
            var.left_drive_velocity = 0;
            var.right_drive_velocity = 0;
            var.inside_threshold_timer = 0;
            var.threshold_timer_set = false;
            var.at_target = false;
            var.timed_out = false;

            unstuck.configure(inertialTurnConfig.unstuck);
            unstuck.initialize(drivetrain, std::fabs(var.error));
        }

        void execute() override{
            InertialTurnConfig& conf = inertialTurnConfig;
            InertialTurnVar& var = inertialTurnVar;
            InertialTurnTarget& target = inertialTurnTarget;
            
            var.current_heading = drivetrain.get_heading_degrees();
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

            float progressError = std::fabs(var.error);

            var.current_time = master_timer.time();

            if (unstuck.run(drivetrain, progressError)) {
                return;
            }

            if (var.current_time >= var.end_time) {
                var.timed_out = true;
                finished = true;
                return;
            }

            if (unstuck.hasFailed()) {
                var.timed_out = true;
                finished = true;
                return;
            }

            if(fabs(var.error) <= PID.angular_integral_windup_threshold){
                var.integral_error += var.error;
            } else {
                var.integral_error = 0;
            }

            var.left_drive_velocity = PID.angular_kP * var.error + PID.angular_kI * var.integral_error + PID.angular_kD * var.derivative_error;
            var.right_drive_velocity = PID.angular_kP * var.error + PID.angular_kI * var.integral_error + PID.angular_kD * var.derivative_error;

            if (conf.min_speed > 0.0f &&
                fabs(var.error) >= conf.settle_error &&
                fabs(var.left_drive_velocity) < conf.min_speed) {

                float minimumTurnSpeed =
                    var.error > 0.0f ? conf.min_speed : -conf.min_speed;

                var.left_drive_velocity = minimumTurnSpeed;
                var.right_drive_velocity = minimumTurnSpeed;
            }

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

            bool tryingToTurn =
                std::fabs(var.error) > conf.settle_error &&
                (
                    std::fabs(var.left_drive_velocity) > 5.0f ||
                    std::fabs(var.right_drive_velocity) > 5.0f
                );

            if (unstuck.shouldStart(drivetrain, progressError, tryingToTurn)) {
                if (!unstuck.start(drivetrain, progressError)) {
                    var.timed_out = true;
                    finished = true;
                }

                return;
            }
            
            drivetrain.set_drive_power(var.left_drive_velocity,-var.right_drive_velocity);

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

            finished = var.at_target || var.timed_out;
               
        }

        bool isFinished() override{
            return finished;
        }

        void end() override{
            drivetrain.stop();
        }
};
