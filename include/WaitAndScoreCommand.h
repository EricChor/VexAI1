#pragma once

#include "Command.h"
#include "Intake.h"
#include "RobotConfig.h"
#include "CommandStatus.h"

enum WaitAndScoreMode {
    SCORE_HIGH,
    SCORE_MID
};

struct WaitAndScoreConfig {
    float wait_time;
    float score_time;
    WaitAndScoreMode score_mode;
};

class WaitAndScoreCommand : public Command {
private:
    Intake& intake;
    WaitAndScoreConfig config;

    bool finished;
    bool scoring;

    float scoreStartTime;
    float scoreEndTime;

    void runScoreMode() {
        if (config.score_mode == SCORE_HIGH) {
            intake.score_high();
        } else {
            intake.score_mid();
        }
    }

public:
    WaitAndScoreCommand(Intake& intake, WaitAndScoreConfig config)
        : intake(intake),
          config(config),
          finished(false),
          scoring(false),
          scoreStartTime(0),
          scoreEndTime(0)
    {
    }

    void initialize() override {
        setCommandStatus("Wait And Score");
        finished = false;
        scoring = false;

        float now = master_timer.time(msec);
        scoreStartTime = now + config.wait_time;
        scoreEndTime = scoreStartTime + config.score_time;

        intake.stop();
    }

    void execute() override {
        float now = master_timer.time(msec);

        if (now < scoreStartTime) {
            intake.stop();
            return;
        }

        if (now < scoreEndTime) {
            scoring = true;
            runScoreMode();
            return;
        }

        scoring = false;
        finished = true;
        intake.stop();
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
        scoring = false;
        intake.stop();
    }

    bool isScoring() const {
        return scoring;
    }
};
