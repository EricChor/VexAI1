#pragma once
#include "Command.h"
#include <vector>
using namespace std;

class ParallelCommandGroup : public Command{
    private:
        vector<Command*> command_list;
        vector<bool> commandFinished;

        bool finished;
    public:
        ParallelCommandGroup()
            :finished(false) 
            {
                command_list.reserve(5);
                commandFinished.reserve(5);
            }

        void addCommand(Command* command);

        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end() override;
        ~ParallelCommandGroup() override;
};