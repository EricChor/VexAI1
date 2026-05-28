#pragma once
#include "Command.h"
#include <vector>
class ParallelDeadlineGroup : public Command{
    private:
        Command* deadlineCommand;
        std::vector<Command*> command_list;

        bool finished;
    
    public:
        ParallelDeadlineGroup(Command* deadlineCommand)
            :deadlineCommand(deadlineCommand),
             finished(false)
            { 
                command_list.reserve(5);
            }

        void addCommand(Command* command);
        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end() override;
};