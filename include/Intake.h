#pragma once
#include "vex.h"
#include "allianceColorEnum.h"
using namespace vex;


struct IntakeWithSortingConfig {
    float initial_intake_velocity;
    float middle_intake_velcoity;
    float final_intake_velocity;

    float red_value;
    float blue_value;

    float red_threshold;
    float blue_threshold;

    float sorting_time;
    float accept_time;

    float threshold_velocity;
    float unjamming_time;
};
struct IntakeWithSortingVar {
    float red_hue_difference;
    float blue_hue_difference;
    float optical_value;

    alliance_color block_color;

    bool correct_block_detected;
    bool accepting_correct_block;
    bool accepting_timer_set;
    float accepting_end_time;

    bool sorting_out_block;
    bool sorting_out_block_timer_set;
    float sorting_end_time;
    bool done_with_color_sorting;

    bool unjamming;
    bool threshold_velocity_passed;
    bool unjamming_timer_set;
    float unjamming_end_time;
    bool done_with_jamming;

    bool objectSeen;
    bool seesRed;
    bool seesBlue;
};


class Intake{
    private:
        motor& initial_intake;
        motor& middle_intake;
        motor& final_intake;

        optical& intake_color_sorting_optical;

        float initial_intake_power;
        float middle_intake_power;
        float final_intake_power;

        alliance_color& current_alliance_color;
    public:

        IntakeWithSortingConfig intakeWithSortingConfig;
        IntakeWithSortingVar intakeWithSortingVar;

        Intake();
        void intake_with_sorting_init(const IntakeWithSortingConfig& config);
        bool intake_with_sorting_step();

        void intaking();
        void unjamming();
        void color_sorting();
        void score_high();
        void score_mid();

        void set_intake_power(float initial_intake_power, float middle_intake_power, float final_intake_power);
        void apply_motor_power();
        void stop();
};