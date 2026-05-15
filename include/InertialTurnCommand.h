#pragma once
#include "Command.h"
#include "Drivetrain.h"

class InertialTurnCommand : public Command{
    private:
        Drivetrain& drivetrain;
        InertialTurnConfig config;
        InertialTurnTarget target;
        DrivePID PID;
        bool finished;

    public:
        InertialTurnCommand(Drivetrain& drivetrain, InertialTurnTarget target, InertialTurnConfig config, DrivePID PID)
            :drivetrain(drivetrain),
             target(target),
             config(config),
             PID(PID),
             finished(false) {}
        
        void initialize() override{
            finished = false;
            drivetrain.inertial_turn_init(config,target,PID);
        }

        void execute() override{
            finished = drivetrain.inertial_turn_step();
        }

        bool isFinished() override{
            return finished;
        }

        void end() override{
            std::cout << "Inertial turn distance from target: " << drivetrain.inertialTurnVar.error << std::endl;
            drivetrain.stop();
        }
};