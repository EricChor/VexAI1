#pragma once
#include "RobotConfig.h"
#include "GetGPSCoordinatesFilteredCommand.h"
#include <cmath>

class SetDrivetrainPoseFromGPSCommand : public Command {
private:
    Drivetrain& drivetrain;
    PositionTracking& positionTracking;
    GetGPSCoordinatesFilteredCommand* gpsCommand;

    bool finished;

public:
    SetDrivetrainPoseFromGPSCommand(
        Drivetrain& drivetrain,
        PositionTracking& positionTracking,
        GetGPSCoordinatesFilteredCommand* gpsCommand = nullptr
    )
        : drivetrain(drivetrain),
          positionTracking(positionTracking),
          gpsCommand(gpsCommand),
          finished(false)
    {
    }

    void initialize() override {
        setCommandStatus("Set Pose From GPS");
        finished = false;

        if (gpsCommand != nullptr && !gpsCommand->wasSuccessful()) {
            setCommandStatus("GPS Filter Failed");
            finished = true;
            return;
        }

        GPSPose pose = positionTracking.getPose();

        if (!std::isfinite(pose.x) ||
            !std::isfinite(pose.y) ||
            !std::isfinite(pose.heading)) {

            setCommandStatus("GPS Pose Invalid");
            finished = true;
            return;
        }

        pose.heading = positionTracking.normalize_heading(pose.heading);

        drivetrain.set_odom_pose(
            pose.x,
            pose.y,
            pose.heading
        );

        setCommandStatus("Set Pose Done");
        finished = true;
    }

    void execute() override {
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
    }
};
