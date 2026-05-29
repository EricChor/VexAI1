#include "RobotConfig.h"


brain Brain;

motor left_front(PORT15,ratio6_1,true);
motor left_back_bottom(PORT18,ratio6_1,true);
motor left_back_top(PORT11,ratio6_1,false);

motor right_front(PORT1,ratio6_1,false);
motor right_back_bottom(PORT8, ratio6_1,false);
motor right_back_top(PORT9,ratio6_1,true);

motor initial_intake(PORT7,ratio6_1,false);
motor middle_intake(PORT10, ratio18_1,false);
motor final_intake(PORT20,ratio18_1,false);

motor_group left_drive(left_front,left_back_bottom,left_back_top);
motor_group right_drive(right_front,right_back_bottom,right_back_top);

inertial drivetrain_inertial(PORT3);

timer master_timer;

Drivetrain drivebase(3.25,2.0/1);

Intake intake;

Scheduler scheduler{};

controller Controller(primary);

optical intake_color_sorting_optical(PORT19);

alliance_color current_alliance_color = RED;

JetsonSerial jetsonSerial;

PositionTracking positionTracking;

gps GPSSensor (PORT5);

vex::distance leftDistanceAligner (PORT17);

vex::distance rightDistanceAligner (PORT6);