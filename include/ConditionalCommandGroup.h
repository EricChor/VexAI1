#pragma once
#include "Command.h"

class ConditionalCommandGroup : public Command{
    private:
        bool(*condition)();
        Command* trueCommand;
        Command* falseCommand;
        Command* selectedCommand;

        bool finished;
        bool initializeSelected;
    
    public:
        ConditionalCommandGroup(bool (*condition)(), Command* trueCommand, Command* falseCommand)
        :condition(condition),
         trueCommand(trueCommand),
         falseCommand(falseCommand),
         selectedCommand(nullptr),
         finished(false),
         initializeSelected(false) {}
         
        void initialize() override;
        void execute() override;
        bool isFinished() override;
        void end() override;

};