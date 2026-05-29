#pragma once

#include "Command.h"
#include "Intake.h"
#include "CommandStatus.h"

struct BlocksBeforeScoringConfig {
    int target_block_count;
    bool reset_count;
};

class SetBlocksBeforeScoringCommand : public Command {
private:
    Intake& intake;
    BlocksBeforeScoringConfig config;
    bool finished;

public:
    SetBlocksBeforeScoringCommand(
        Intake& intake,
        BlocksBeforeScoringConfig config
    )
        : intake(intake),
          config(config),
          finished(false)
    {
    }

    void initialize() override {
        setCommandStatus("Set Blocks Before Score");
        if (config.target_block_count > 0) {
            intake.set_blocks_before_scoring(config.target_block_count);
        }

        if (config.reset_count) {
            intake.reset_accepted_block_count();
        }

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
