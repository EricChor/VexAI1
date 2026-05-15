#pragma once
#include <iostream>
#include "Command.h"
#include "Intake.h"

class IntakeWithSorting : public Command{
    private:
        Intake& intake;
        IntakeWithSortingConfig config;

        bool finished;

    public:
        IntakeWithSorting(Intake& intake, IntakeWithSortingConfig config)
            :intake(intake),
             config(config)
             {}
        
        void initialize() override{
            finished = false;
            intake.intake_with_sorting_init(config);
        }

        void execute() override{
            finished = intake.intake_with_sorting_step();
        }

        bool isFinished() override{
            return finished;
        }

        void end() override{
            intake.stop();
        }
};