#include "Intake.h"
#include "RobotConfig.h"

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
    Intake::set_intake_power(100,100,100);
}

void Intake::unjamming(){
    Intake::set_intake_power(100,100,100);
}

void Intake::color_sorting(){
    Intake::set_intake_power(100,100,100);
}

void Intake::score_high(){
    Intake::set_intake_power(100,100,100);
}

void Intake::score_mid(){
    Intake::set_intake_power(100,100,100);
}

void Intake::stop(){
    Intake::set_intake_power(0,0,0);
}


void Intake::intake_with_sorting_init(const IntakeWithSortingConfig& config){
    intakeWithSortingConfig = config;
    IntakeWithSortingVar& var = intakeWithSortingVar;

    var.red_hue_difference = 0;
    var.blue_hue_difference = 0;
    var.optical_value = 0;
    var.threshold_velocity_passed = false;
    var.block_color = current_alliance_color;

    var.unjamming = false;
    var.threshold_velocity_passed = false;
    var.unjamming_timer_set = false;
    var.done_with_jamming = true;

    var.sorting_out_block =  false;
    var.sorting_out_block_timer_set = false;
    var.sorting_end_time = 0;
    var.done_with_color_sorting = true;
    var.unjamming_end_time = 0;
}

bool Intake::intake_with_sorting_step(){
    IntakeWithSortingVar& var = intakeWithSortingVar;
    IntakeWithSortingConfig& conf = intakeWithSortingConfig;
    var.optical_value = intake_color_sorting_optical.hue();
    var.red_hue_difference = conf.red_value - var.optical_value;

    if(fabs(var.red_hue_difference) > 180){
        if(var.red_hue_difference > 0){
            var.red_hue_difference -= 360;
        } else {
            var.red_hue_difference += 360;
        }
    }
    
    var.blue_hue_difference = conf.blue_value - var.optical_value;
    if(fabs(var.blue_hue_difference) > 180){
        if(var.blue_hue_difference > 0){
            var.blue_hue_difference -= 360;
        } else {
            var.blue_hue_difference += 360;
        }
    }

    if(intake_color_sorting_optical.isNearObject()){
        if(fabs(var.red_hue_difference) < fabs(var.blue_hue_difference)){
            var.block_color = RED;
        } else {
            var.block_color = BLUE;
        }
    }

    if(current_alliance_color != var.block_color && intake_color_sorting_optical.isNearObject() == true) {
        if(current_alliance_color == BLUE){
            if((fabs(var.blue_hue_difference) < conf.blue_threshold) && (conf.blue_threshold != 0)){
                var.sorting_out_block = true;
            } else {
                var.sorting_out_block = false;
            }
        } else {
            if((fabs(var.red_hue_difference) < conf.red_threshold) && (conf.red_threshold != 0)){
                var.sorting_out_block = true;
            } else {
                var.sorting_out_block = false;
            }
        }
    } else {
        var.sorting_out_block = false;
    }


    float now = master_timer.time();
    if(var.sorting_out_block && !var.sorting_out_block_timer_set){
        var.sorting_end_time = now + conf.sorting_time;
        var.done_with_color_sorting = false;
        var.sorting_out_block_timer_set = true;
    }
    
    if(var.sorting_out_block_timer_set){
        if(now < var.sorting_end_time){
            var.done_with_color_sorting = false;
        } else {
            var.sorting_out_block_timer_set = false;
            var.done_with_color_sorting = true;
            var.sorting_out_block = false;

            var.unjamming_timer_set = false;
            var.done_with_jamming = true;
            var.threshold_velocity_passed = false;
        }
    } else {
        var.done_with_color_sorting = true;
    }

    if (fabs(initial_intake.velocity(pct)) > conf.threshold_velocity) {
        var.threshold_velocity_passed = true;
    }

    if((fabs(initial_intake.velocity(pct)) < conf.threshold_velocity) && (!var.unjamming_timer_set) && (var.threshold_velocity_passed) && (var.done_with_color_sorting)){
        var.unjamming_end_time = now + conf.unjamming_time;
        var.unjamming_timer_set = true;
        var.done_with_jamming = false;
        var.threshold_velocity_passed = false;
    }

    if(var.unjamming_timer_set){
        if(now < var.unjamming_end_time){
            var.done_with_jamming = false;
            var.threshold_velocity_passed = false;
        } else {
            var.unjamming_timer_set = false;
            var.done_with_jamming = true;
            var.threshold_velocity_passed = true;
        }
    } else {
        var.done_with_jamming = true;
    }

    //prioritize_color_sorting_over unjamming
    if(!var.done_with_color_sorting){
        color_sorting();
    }
    else if(!var.done_with_jamming){
        unjamming();
    }
    else {
        intaking();
    }

    return false;
}