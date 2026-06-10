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
    float sensor_difference_kP;
    float sensor_difference_acceptable_error;
    float max_linear_sensor_difference;
    float max_time;
    float max_accel;
    float settle_time;
    UnstuckArcConfig unstuck;
};

struct WallAlignmentVar {
    float end_time;
    float current_time;

    float current_heading;
    float start_x;
    float start_y;
    float angular_error;
    float angular_derivative_error;
    float angular_previous_error;
    float angular_integral_error;

    float left_distance;
    float right_distance;
    float sensor_distance_difference;
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
    bool both_sensors_detected;
    bool wall_detected;
    bool previous_left_sensor_detected;
    bool previous_right_sensor_detected;
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
        var.current_heading = drivetrain.get_heading_degrees();
        var.start_x = positionTracking.get_x();
        var.start_y = positionTracking.get_y();
        var.angular_error = 0;
        var.angular_derivative_error = 0;
        var.angular_previous_error = 0;
        var.angular_integral_error = 0;

        var.left_distance = 0;
        var.right_distance = 0;
        var.sensor_distance_difference = 0;
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
        var.both_sensors_detected = false;
        var.wall_detected = false;
        var.previous_left_sensor_detected = false;
        var.previous_right_sensor_detected = false;

        unstuck.configure(config.unstuck);
        unstuck.initialize(drivetrain, 0.0f);
    }

    void execute() override {
        positionTracking.update_raw_pose();
        var.current_heading = drivetrain.get_heading_degrees();
        float previousAngularError = var.angular_previous_error;
        var.angular_error =
            headingError(target.target_heading, var.current_heading);

        var.angular_derivative_error =
            var.angular_error - previousAngularError;

        if (var.angular_error * previousAngularError < 0.0f) {
            var.angular_integral_error = 0;
        }

        var.angular_previous_error = var.angular_error;

        var.left_sensor_detected = leftDistanceSensor.isObjectDetected();
        var.right_sensor_detected = rightDistanceSensor.isObjectDetected();

        if (var.left_sensor_detected) {
            var.left_distance = leftDistanceSensor.objectDistance(inches);

            if (!std::isfinite(var.left_distance) || var.left_distance <= 0.0f) {
                var.left_sensor_detected = false;
            }
        }

        if (var.right_sensor_detected) {
            var.right_distance = rightDistanceSensor.objectDistance(inches);

            if (!std::isfinite(var.right_distance) || var.right_distance <= 0.0f) {
                var.right_sensor_detected = false;
            }
        }

        var.both_sensors_detected =
            var.left_sensor_detected && var.right_sensor_detected;

        var.wall_detected =
            var.left_sensor_detected || var.right_sensor_detected;

        if (var.both_sensors_detected) {
            var.average_distance =
                (var.left_distance + var.right_distance) / 2.0f;

            // Positive means the left sensor is closer, so turn clockwise.
            var.sensor_distance_difference =
                var.right_distance - var.left_distance;
        } else if (var.left_sensor_detected) {
            var.average_distance = var.left_distance;
            var.sensor_distance_difference = 0;
        } else if (var.right_sensor_detected) {
            var.average_distance = var.right_distance;
            var.sensor_distance_difference = 0;
        } else {
            var.average_distance = target.target_distance;
            var.sensor_distance_difference = 0;
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

        bool sameSensorsDetected =
            var.left_sensor_detected == var.previous_left_sensor_detected &&
            var.right_sensor_detected == var.previous_right_sensor_detected;

        float previousLinearError = var.linear_previous_error;

        if (var.wall_detected && sameSensorsDetected) {
            var.linear_derivative_error =
                var.linear_error - previousLinearError;
        } else {
            var.linear_derivative_error = 0;
            var.linear_integral_error = 0;
        }

        if (var.linear_error * previousLinearError < 0.0f) {
            var.linear_integral_error = 0;
        }

        var.linear_previous_error = var.linear_error;
        var.previous_left_sensor_detected = var.left_sensor_detected;
        var.previous_right_sensor_detected = var.right_sensor_detected;

        float progressError = 0;

        if (var.wall_detected) {
            progressError =
                std::fabs(var.linear_error) +
                0.1f * std::fabs(var.angular_error) +
                std::fabs(var.sensor_distance_difference);
        } else {
            float xTravel = positionTracking.get_x() - var.start_x;
            float yTravel = positionTracking.get_y() - var.start_y;

            // The helper expects its progress error to decrease. While the wall
            // is unseen, increasing GPS displacement proves the robot is moving.
            progressError = -std::sqrt(xTravel * xTravel + yTravel * yTravel);
        }

        var.current_time = master_timer.time(msec);

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

        float angularVelocity =
            PID.angular_kP * var.angular_error +
            PID.angular_kI * var.angular_integral_error +
            PID.angular_kD * var.angular_derivative_error;

        if (var.both_sensors_detected) {
            angularVelocity +=
                config.sensor_difference_kP * var.sensor_distance_difference;
        }

        angularVelocity = clamp(angularVelocity, config.max_angular_speed);

        var.left_drive_angular_velocity = angularVelocity;
        var.right_drive_angular_velocity = -angularVelocity;

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

        bool sensorDifferenceTooLargeForDistanceCorrection =
            var.both_sensors_detected &&
            config.max_linear_sensor_difference > 0.0f &&
            std::fabs(var.sensor_distance_difference) >
                config.max_linear_sensor_difference;

        bool pauseLinearCorrection =
            headingTooFarForDistanceCorrection ||
            sensorDifferenceTooLargeForDistanceCorrection;

        if (pauseLinearCorrection) {

            var.left_drive_linear_velocity = 0;
            var.right_drive_linear_velocity = 0;
            var.prev_left_drive_linear_velocity = 0;
            var.prev_right_drive_linear_velocity = 0;
            var.linear_integral_error = 0;
        }

        // Rear sensors: if the robot is too far from the wall, drive backward.
        var.left_drive_linear_velocity *= -1.0f;
        var.right_drive_linear_velocity *= -1.0f;

        if (!pauseLinearCorrection) {
            applyAccelLimit();
        }

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
                !var.wall_detected ||
                std::fabs(var.linear_error) > config.acceptable_error ||
                std::fabs(var.angular_error) > config.heading_acceptable_error ||
                (
                    var.both_sensors_detected &&
                    std::fabs(var.sensor_distance_difference) >
                        config.sensor_difference_acceptable_error
                )
            ) &&
            (
                std::fabs(leftPower) > 5.0f ||
                std::fabs(rightPower) > 5.0f
            );

        if (unstuck.shouldStart(
                drivetrain,
                progressError,
                tryingToAlign && (!var.wall_detected || sameSensorsDetected)
            )) {
            if (!unstuck.start(drivetrain, progressError)) {
                var.timed_out = true;
                finished = true;
            }

            return;
        }

        drivetrain.set_drive_power(leftPower, rightPower);

        var.current_time = master_timer.time(msec);

        bool alignedToWall =
            var.both_sensors_detected &&
            std::fabs(var.linear_error) < config.acceptable_error &&
            std::fabs(var.angular_error) < config.heading_acceptable_error &&
            std::fabs(var.sensor_distance_difference) <
                config.sensor_difference_acceptable_error;

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

    bool wasSuccessful() const {
        return var.at_target;
    }

    bool didTimeOut() const {
        return var.timed_out;
    }
};
