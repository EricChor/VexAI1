#pragma once
#include "Command.h"
#include "CommandStatus.h"

class DoNothingCommand : public Command {
private:
    bool finished;

public:
    DoNothingCommand()
        : finished(false)
    {
    }

    void initialize() override {
        setCommandStatus("Skip");
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