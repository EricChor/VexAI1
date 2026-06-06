#pragma once
#include "vex.h"
#include "Scheduler.h"
#include "Drivetrain.h"
#include "SequentialCommandGroup.h"
#include "JetsonSerial.h"
#include "Intake.h"
#include "allianceColorEnum.h"
#include "PositionTracking.h"

using namespace vex;

extern motor left_front;
extern motor left_back_top;
extern motor left_back_bottom;

extern motor right_front;
extern motor right_back_top;
extern motor right_back_bottom;

extern motor_group left_drive;
extern motor_group right_drive;

extern motor initial_intake;
extern motor middle_intake;
extern motor final_intake;

extern inertial drivetrain_inertial;

extern timer master_timer;

extern Drivetrain drivebase;

extern Intake intake;

extern Scheduler scheduler;

extern controller Controller;

extern brain Brain;

extern optical intake_color_sorting_optical;

extern alliance_color current_alliance_color;

extern JetsonSerial jetsonSerial;

extern gps GPSSensor;

extern gps GPSBackup;

extern PositionTracking positionTracking;

extern vex::distance leftDistanceAligner;

extern vex::distance rightDistanceAligner;

extern digital_out loaderPiston;

extern vex::competition Competition;
