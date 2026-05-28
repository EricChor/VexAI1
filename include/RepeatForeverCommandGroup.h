#pragma once
#include "Command.h"
#include <vector>

class RepeatForeverCommandGroup : public Command{
    private:
        Command& command;
    public:
        RepeatForeverCommandGroup(Command& command)
        :command(command)
        {}

        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end() override;


};