#pragma once
#include <vector>
#include "Command.h"
using namespace std;

class SequentialCommandGroup : public Command{
    private:
        vector<Command*> command_list;

        int currentIndex;
        bool finished;
        bool currentCommandInitialized;

    public:
        SequentialCommandGroup()
            :currentIndex(0),
             finished(false),
             currentCommandInitialized(false)
            {
                command_list.reserve(20);
            }

        void addCommand(Command* command);
        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end() override;
        ~SequentialCommandGroup() override;
};