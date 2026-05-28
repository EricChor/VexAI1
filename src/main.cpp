#include "vex.h"
#include "Scheduler.h"
#include "Drivetrain.h"
#include "RobotConfig.h"
#include "JetsonSerial.h"
#include "Routines.h"
using namespace vex;

int main() {
    jetsonSerial.JetsonSerialSetup();
    build_AI_routine();

    scheduler.schedule(&AI_ROUTE_ONE);
    while(drivetrain_inertial.isCalibrating() || GPSSensor.isCalibrating()){
        vex::task::sleep(10);
    }
    drivetrain_inertial.setHeading(positionTracking.get_heading(),degrees);
    while(true){
        drivebase.update_odom_pose();
        scheduler.run();
        drivebase.apply_motor_power();
        intake.apply_motor_power();
        jetsonSerial.print_block_pos_on_screen();
        vex::task::sleep(10);
    }

}



