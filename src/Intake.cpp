#include "Intake.h"
#include "RobotConfig.h"
#include <iostream>

Intake::Intake()
:initial_intake(::initial_intake),
 middle_intake(::middle_intake),
 final_intake(::final_intake),
 intake_color_sorting_optical(::intake_color_sorting_optical),
 current_alliance_color(::current_alliance_color)
{}

void Intake::set_intake_power(float initial_intake_power, float middle_intake_power, float final_intake_power){
    Intake::initial_intake_power = initial_intake_power;
    Intake::middle_intake_power = middle_intake_power;
    Intake::final_intake_power = final_intake_power;

}

void Intake::intaking(){
    Intake::set_intake_power(100,0,100);
}

void Intake::unjamming(){
    Intake::set_intake_power(-100,100,100);
}

void Intake::color_sorting(){
    Intake::set_intake_power(100,0,-100);
}

void Intake::score_high(){
    Intake::set_intake_power(100,-100,-100);
}

void Intake::score_mid(){
    Intake::set_intake_power(100,-100,100);
}

void Intake::stop(){
    Intake::set_intake_power(0,0,0);
}


void Intake::intake_with_sorting_init(const IntakeWithSortingConfig& config) {
    intakeWithSortingConfig = config;
    IntakeWithSortingVar& var = intakeWithSortingVar;

    initial_intake_power = config.initial_intake_velocity;
    middle_intake_power = config.middle_intake_velcoity;
    final_intake_power = config.final_intake_velocity;

    var.red_hue_difference = 0;
    var.blue_hue_difference = 0;
    var.optical_value = 0;

    var.block_color = current_alliance_color;

    var.correct_block_detected = false;
    var.accepting_correct_block = false;
    var.accepting_timer_set = false;
    var.accepting_end_time = 0;

    var.sorting_out_block = false;
    var.sorting_out_block_timer_set = false;
    var.sorting_end_time = 0;
    var.done_with_color_sorting = true;

    var.unjamming = false;
    var.threshold_velocity_passed = false;
    var.unjamming_timer_set = false;
    var.unjamming_end_time = 0;
    var.done_with_jamming = true;

    var.objectSeen = false;
    var.seesRed = false;
    var.seesBlue = false;
}

bool Intake::intake_with_sorting_step() {
    IntakeWithSortingVar& var = intakeWithSortingVar;
    IntakeWithSortingConfig& conf = intakeWithSortingConfig;

    float now = master_timer.time();

    var.optical_value = intake_color_sorting_optical.hue();

    var.red_hue_difference = conf.red_value - var.optical_value;

    if (fabs(var.red_hue_difference) > 180) {
        if (var.red_hue_difference > 0) {
            var.red_hue_difference -= 360;
        } else {
            var.red_hue_difference += 360;
        }
    }

    var.blue_hue_difference = conf.blue_value - var.optical_value;

    if (fabs(var.blue_hue_difference) > 180) {
        if (var.blue_hue_difference > 0) {
            var.blue_hue_difference -= 360;
        } else {
            var.blue_hue_difference += 360;
        }
    }

    var.objectSeen = intake_color_sorting_optical.isNearObject();

    var.correct_block_detected = false;
    var.sorting_out_block = false;

    if (var.objectSeen) {
        var.seesRed =
            fabs(var.red_hue_difference) < conf.red_threshold &&
            conf.red_threshold != 0;

        var.seesBlue =
            fabs(var.blue_hue_difference) < conf.blue_threshold &&
            conf.blue_threshold != 0;

        if (fabs(var.red_hue_difference) < fabs(var.blue_hue_difference)) {
            var.block_color = RED;
        } else {
            var.block_color = BLUE;
        }

        if (current_alliance_color == RED) {
            var.correct_block_detected = var.seesRed;
            var.sorting_out_block = var.seesBlue;
        } else {
            var.correct_block_detected = var.seesBlue;
            var.sorting_out_block = var.seesRed;
        }
    }

    if (var.correct_block_detected && !var.accepting_timer_set) {
        var.accepting_end_time = now + conf.accept_time;
        var.accepting_timer_set = true;
        var.accepting_correct_block = true;
    }

    if (var.accepting_timer_set) {
        if (now < var.accepting_end_time) {
            var.accepting_correct_block = true;
        } else {
            var.accepting_timer_set = false;
            var.accepting_correct_block = false;
        }
    }

    if (var.sorting_out_block && !var.sorting_out_block_timer_set) {
        var.sorting_end_time = now + conf.sorting_time;
        var.done_with_color_sorting = false;
        var.sorting_out_block_timer_set = true;

        // Wrong color overrides accepting
        var.accepting_timer_set = false;
        var.accepting_correct_block = false;
    }

    if (var.sorting_out_block_timer_set) {
        if (now < var.sorting_end_time) {
            var.done_with_color_sorting = false;
        } else {
            var.sorting_out_block_timer_set = false;
            var.done_with_color_sorting = true;
            var.sorting_out_block = false;

            // After sorting, force unjamming to re-check from scratch
            var.unjamming_timer_set = false;
            var.done_with_jamming = true;
            var.threshold_velocity_passed = false;
        }
    } else {
        var.done_with_color_sorting = true;
    }

    float intakeVelocity = fabs(initial_intake.velocity(pct));

    if (var.accepting_correct_block &&
        var.done_with_color_sorting &&
        intakeVelocity > conf.threshold_velocity) {

        var.threshold_velocity_passed = true;
    }

    if ((intakeVelocity < conf.threshold_velocity) &&
        (!var.unjamming_timer_set) &&
        (var.threshold_velocity_passed) &&
        (var.done_with_color_sorting) &&
        (var.accepting_correct_block)) {

        var.unjamming_end_time = now + conf.unjamming_time;
        var.unjamming_timer_set = true;
        var.done_with_jamming = false;
        var.threshold_velocity_passed = false;
    }

    if (var.unjamming_timer_set) {
        if (now < var.unjamming_end_time) {
            var.done_with_jamming = false;
            var.threshold_velocity_passed = false;
        } else {
            var.unjamming_timer_set = false;
            var.done_with_jamming = true;
            var.threshold_velocity_passed = false;
        }
    } else {
        var.done_with_jamming = true;
    }

    // -------------------------
    // Final behavior priority
    // -------------------------

    if (!var.done_with_color_sorting) {
        // Wrong color was detected, actively reject it
        color_sorting();
    }
    else if (!var.accepting_correct_block) {
        // Default state: reject/sort unless correct color has been detected
        color_sorting();
    }
    else if (!var.done_with_jamming) {
        // Correct block is being accepted, but intake appears jammed
        unjamming();
    }
    else {
        // Correct block was detected and accept timer is active
        intaking();
    }

    return false;
}

void Intake::apply_motor_power(){
    initial_intake.spin(fwd,initial_intake_power,pct);
    middle_intake.spin(fwd,middle_intake_power,pct);
    final_intake.spin(fwd,final_intake_power,pct);
}