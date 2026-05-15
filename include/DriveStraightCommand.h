#pragma once
#include <iostream>
#include "Command.h"
#include "Drivetrain.h"

class DriveStraightCommand : public Command{
    private:
        Drivetrain& drivetrain;
        DriveStraightConfig config;
        DriveStraightTarget target;
        DrivePID PID;

        bool finished;

    public:
        DriveStraightCommand(Drivetrain& drivetrain, DriveStraightTarget target, DriveStraightConfig config, DrivePID PID)
            :drivetrain(drivetrain),
             target(target),
             config(config),
             PID(PID),
             finished(false) {}
        
        void initialize() override{
            finished = false;
            drivetrain.drive_straight_init(config,target,PID);
        }

        void execute() override{
            finished = drivetrain.drive_straight_step();
        }

        bool isFinished() override{
            return finished;
        }

        void end() override{
            std::cout << "Drive straight distance from target: " << drivetrain.driveStraightVar.linear_error << std::endl;
            drivetrain.stop();
        }
};