#pragma once
#include "Command.h"
#include <vector>

class RaceCommandGroup : public Command{
    private:
        std::vector<Command*> command_list;
        bool finished;

    public:
        RaceCommandGroup()
        :finished(false)
        {
            command_list.reserve(5);
        }
        void addCommand(Command* command);
        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end() override;

};
