#pragma once

#include "Command.h"
#include "CommandStatus.h"
#include "vex.h"

class ChangeLoaderStateCommand : public Command {
private:
    vex::digital_out& loaderPiston;
    bool raiseLoader;
    bool finished;

public:
    ChangeLoaderStateCommand(vex::digital_out& loaderPiston, bool raiseLoader)
        : loaderPiston(loaderPiston),
          raiseLoader(raiseLoader),
          finished(false)
    {
    }

    void initialize() override {
        setCommandStatus(raiseLoader ? "Raise Loader" : "Lower Loader");
        loaderPiston.set(raiseLoader);
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
