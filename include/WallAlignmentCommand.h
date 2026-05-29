#pragma once

#include "Command.h"
#include "Drivetrain.h"
#include "PositionTracking.h"
#include "RobotConfig.h"
#include "UnstuckArcHelper.h"
#include "CommandStatus.h"

#include <cmath>

struct WallAlignmentTarget {
    float target_heading;
    float target_distance;
};

struct WallAlignmentConfig {
    float max_linear_speed;
    float max_angular_speed;
    float acceptable_error;
    float heading_acceptable_error;
    float max_linear_heading_error;
    float max_time;
    float max_accel;
    float settle_time;
    UnstuckArcConfig unstuck;
};

struct WallAlignmentVar {
    float end_time;
    float current_time;

    float current_heading;
    float angular_error;
    float angular_derivative_error;
    float angular_previous_error;
    float angular_integral_error;

    float left_distance;
    float right_distance;
    float average_distance;
    float corrected_distance;
    float linear_error;
    float linear_derivative_error;
    float linear_previous_error;
    float linear_integral_error;

    float left_drive_angular_velocity;
    float right_drive_angular_velocity;

    float left_drive_linear_velocity;
    float right_drive_linear_velocity;

    float prev_left_drive_linear_velocity;
    float prev_right_drive_linear_velocity;

    float max_accel;
    float inside_threshold_timer;

    bool threshold_timer_set;
    bool at_target;
    bool timed_out;
    bool left_sensor_detected;
    bool right_sensor_detected;
    bool wall_detected;
};

class WallAlignmentCommand : public Command {
private:
    Drivetrain& drivetrain;
    PositionTracking& positionTracking;
    vex::distance& leftDistanceSensor;
    vex::distance& rightDistanceSensor;

    WallAlignmentTarget target;
    WallAlignmentConfig config;
    WallAlignmentVar var;
    DrivePID PID;
    UnstuckArcHelper unstuck;

    bool finished;

    float headingError(float targetHeading, float currentHeading) {
        float error = targetHeading - currentHeading;

        while (error > 180.0f) {
            error -= 360.0f;
        }

        while (error < -180.0f) {
            error += 360.0f;
        }

        return error;
    }

    float clamp(float value, float maxMagnitude) {
        if (std::fabs(value) <= maxMagnitude) {
            return value;
        }

        return value > 0 ? maxMagnitude : -maxMagnitude;
    }

    void applyAccelLimit() {
        if (var.max_accel == 0) {
            return;
        }

        if (var.left_drive_linear_velocity >
            var.prev_left_drive_linear_velocity + var.max_accel) {

            var.left_drive_linear_velocity =
                var.prev_left_drive_linear_velocity + var.max_accel;
        }

        if (var.left_drive_linear_velocity <
            var.prev_left_drive_linear_velocity - var.max_accel) {

            var.left_drive_linear_velocity =
                var.prev_left_drive_linear_velocity - var.max_accel;
        }

        if (var.right_drive_linear_velocity >
            var.prev_right_drive_linear_velocity + var.max_accel) {

            var.right_drive_linear_velocity =
                var.prev_right_drive_linear_velocity + var.max_accel;
        }

        if (var.right_drive_linear_velocity <
            var.prev_right_drive_linear_velocity - var.max_accel) {

            var.right_drive_linear_velocity =
                var.prev_right_drive_linear_velocity - var.max_accel;
        }
    }

    void scaleDrivePower(float& leftPower, float& rightPower) {
        float maxMagnitude = std::fabs(leftPower);

        if (std::fabs(rightPower) > maxMagnitude) {
            maxMagnitude = std::fabs(rightPower);
        }

        if (maxMagnitude > 100.0f) {
            leftPower = leftPower * 100.0f / maxMagnitude;
            rightPower = rightPower * 100.0f / maxMagnitude;
        }
    }

public:
    WallAlignmentCommand(
        Drivetrain& drivetrain,
        PositionTracking& positionTracking,
        vex::distance& leftDistanceSensor,
        vex::distance& rightDistanceSensor,
        WallAlignmentTarget target,
        WallAlignmentConfig config,
        DrivePID PID
    )
        : drivetrain(drivetrain),
          positionTracking(positionTracking),
          leftDistanceSensor(leftDistanceSensor),
          rightDistanceSensor(rightDistanceSensor),
          target(target),
          config(config),
          PID(PID),
          finished(false)
    {
    }

    void initialize() override {
        setCommandStatus("Wall Alignment");
        finished = false;

        var.end_time = master_timer.time(msec) + config.max_time;
        var.current_time = 0;

        positionTracking.update_raw_pose();
        var.current_heading = positionTracking.get_heading();
        var.angular_error = 0;
        var.angular_derivative_error = 0;
        var.angular_previous_error = 0;
        var.angular_integral_error = 0;

        var.left_distance = 0;
        var.right_distance = 0;
        var.average_distance = 0;
        var.corrected_distance = 0;
        var.linear_error = 0;
        var.linear_derivative_error = 0;
        var.linear_previous_error = 0;
        var.linear_integral_error = 0;

        var.left_drive_angular_velocity = 0;
        var.right_drive_angular_velocity = 0;

        var.left_drive_linear_velocity = 0;
        var.right_drive_linear_velocity = 0;

        var.prev_left_drive_linear_velocity = 0;
        var.prev_right_drive_linear_velocity = 0;

        var.max_accel = config.max_accel / 100.0f;
        var.inside_threshold_timer = 0;

        var.threshold_timer_set = false;
        var.at_target = false;
        var.timed_out = false;
        var.left_sensor_detected = false;
        var.right_sensor_detected = false;
        var.wall_detected = false;

        unstuck.configure(config.unstuck);
        unstuck.initialize(drivetrain, std::fabs(target.target_distance));
    }

    void execute() override {
        positionTracking.update_raw_pose();
        var.current_heading = positionTracking.get_heading();
        var.angular_error =
            headingError(target.target_heading, var.current_heading);

        var.angular_derivative_error =
            var.angular_error - var.angular_previous_error;
        var.angular_previous_error = var.angular_error;

        var.left_sensor_detected = leftDistanceSensor.isObjectDetected();
        var.right_sensor_detected = rightDistanceSensor.isObjectDetected();
        var.wall_detected =
            var.left_sensor_detected || var.right_sensor_detected;

        if (var.left_sensor_detected) {
            var.left_distance = leftDistanceSensor.objectDistance(inches);
        }

        if (var.right_sensor_detected) {
            var.right_distance = rightDistanceSensor.objectDistance(inches);
        }

        if (var.left_sensor_detected && var.right_sensor_detected) {
            var.average_distance =
                (var.left_distance + var.right_distance) / 2.0f;
        } else if (var.left_sensor_detected) {
            var.average_distance = var.left_distance;
        } else if (var.right_sensor_detected) {
            var.average_distance = var.right_distance;
        } else {
            var.average_distance = target.target_distance;
        }

        if (var.wall_detected) {
            setLastWallAlignmentDistance(var.average_distance);
        }

        const float PI = 3.14159265f;
        float angularErrorRadians = var.angular_error * PI / 180.0f;

        var.corrected_distance =
            std::cos(angularErrorRadians) * var.average_distance;

        var.linear_error =
            var.corrected_distance - target.target_distance;

        var.linear_derivative_error =
            var.linear_error - var.linear_previous_error;
        var.linear_previous_error = var.linear_error;

        float progressError =
            std::fabs(var.linear_error) + 0.1f * std::fabs(var.angular_error);

        var.current_time = master_timer.time(msec);

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

        if (std::fabs(var.angular_error) <= PID.angular_integral_windup_threshold) {
            var.angular_integral_error += var.angular_error;
        } else {
            var.angular_integral_error = 0;
        }

        if (std::fabs(var.linear_error) <= PID.linear_integral_windup_threshold) {
            var.linear_integral_error += var.linear_error;
        } else {
            var.linear_integral_error = 0;
        }

        var.left_drive_angular_velocity =
            PID.angular_kP * var.angular_error +
            PID.angular_kI * var.angular_integral_error +
            PID.angular_kD * var.angular_derivative_error;

        var.right_drive_angular_velocity =
            -1.0f * var.left_drive_angular_velocity;

        var.left_drive_angular_velocity =
            clamp(var.left_drive_angular_velocity, config.max_angular_speed);
        var.right_drive_angular_velocity =
            clamp(var.right_drive_angular_velocity, config.max_angular_speed);

        var.left_drive_linear_velocity =
            PID.linear_kP * var.linear_error +
            PID.linear_kI * var.linear_integral_error +
            PID.linear_kD * var.linear_derivative_error;

        var.right_drive_linear_velocity = var.left_drive_linear_velocity;

        if (!var.wall_detected) {
            var.left_drive_linear_velocity = config.max_linear_speed;
            var.right_drive_linear_velocity = config.max_linear_speed;
        }

        bool headingTooFarForDistanceCorrection =
            config.max_linear_heading_error > 0.0f &&
            std::fabs(var.angular_error) > config.max_linear_heading_error;

        if (headingTooFarForDistanceCorrection && var.wall_detected) {
            var.left_drive_linear_velocity = 0;
            var.right_drive_linear_velocity = 0;
        }

        // Rear sensors: if the robot is too far from the wall, drive backward.
        var.left_drive_linear_velocity *= -1.0f;
        var.right_drive_linear_velocity *= -1.0f;

        applyAccelLimit();

        var.left_drive_linear_velocity =
            clamp(var.left_drive_linear_velocity, config.max_linear_speed);
        var.right_drive_linear_velocity =
            clamp(var.right_drive_linear_velocity, config.max_linear_speed);

        float leftPower =
            var.left_drive_linear_velocity + var.left_drive_angular_velocity;
        float rightPower =
            var.right_drive_linear_velocity + var.right_drive_angular_velocity;

        scaleDrivePower(leftPower, rightPower);

        bool tryingToAlign =
            (
                std::fabs(var.linear_error) > config.acceptable_error ||
                std::fabs(var.angular_error) > 3.0f
            ) &&
            (
                std::fabs(leftPower) > 5.0f ||
                std::fabs(rightPower) > 5.0f
            );

        if (unstuck.shouldStart(drivetrain, progressError, tryingToAlign)) {
            if (!unstuck.start(drivetrain, progressError)) {
                var.timed_out = true;
                finished = true;
            }

            return;
        }

        drivetrain.set_drive_power(leftPower, rightPower);

        var.current_time = master_timer.time(msec);

        bool alignedToWall =
            var.wall_detected &&
            std::fabs(var.linear_error) < config.acceptable_error &&
            std::fabs(var.angular_error) < config.heading_acceptable_error;

        if (alignedToWall) {
            if (!var.threshold_timer_set) {
                var.inside_threshold_timer =
                    var.current_time + config.settle_time;
                var.threshold_timer_set = true;
            }

            if (var.current_time >= var.inside_threshold_timer) {
                var.at_target = true;
            }
        } else {
            var.threshold_timer_set = false;
            var.at_target = false;
        }

        if (var.current_time >= var.end_time) {
            var.timed_out = true;
        }

        var.prev_left_drive_linear_velocity = var.left_drive_linear_velocity;
        var.prev_right_drive_linear_velocity = var.right_drive_linear_velocity;

        finished = var.at_target || var.timed_out;
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
        drivetrain.stop();
    }
};
