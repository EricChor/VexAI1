#include "vex.h"
#include "Scheduler.h"
#include "Drivetrain.h"
#include "RobotConfig.h"
#include "JetsonSerial.h"
#include "InteractionRoutine.h"
#include "IsolationRoutine.h"
using namespace vex;


void isolationPeriod(){
    scheduler.cancelAll();
    drivebase.stop();
    intake.stop();

    //scheduler.schedule(&AI_ISOLATION_ROUTE);
    scheduler.schedule(&AI_ISO_ROUTE);
    positionTracking.update_raw_pose();
    drivebase.set_odom_pose(
        positionTracking.get_x(),
        positionTracking.get_y(),
        positionTracking.get_heading());
    while(true){
        drivebase.update_odom_pose();
        scheduler.run();
        drivebase.apply_motor_power();
        intake.apply_motor_power();
        jetsonSerial.print_block_pos_on_screen();
        vex::task::sleep(10);
    }

}

void interactionPeriod(){
    scheduler.cancelAll();
    drivebase.stop();
    intake.stop();

    scheduler.schedule(&AI_INTERACTION_ROUTE);
    positionTracking.update_raw_pose();
    drivebase.set_odom_pose(
        positionTracking.get_x(),
        positionTracking.get_y(),
        positionTracking.get_heading());

    while(true){
        drivebase.update_odom_pose();
        scheduler.run();
        drivebase.apply_motor_power();
        intake.apply_motor_power();
        jetsonSerial.print_block_pos_on_screen();
        vex::task::sleep(10);
    }
}



int main() {
    jetsonSerial.JetsonSerialSetup();
    
    build_interaction_routine();
    build_isolation_routine();
    build_iso_route();
    while(drivetrain_inertial.isCalibrating() || GPSSensor.isCalibrating() || GPSBackup.isCalibrating()){
        vex::task::sleep(10);
    }

    Competition.bStopAllTasksBetweenModes = true;
    Competition.autonomous(isolationPeriod);
    Competition.drivercontrol(interactionPeriod);

}
