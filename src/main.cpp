#include "vex.h"
#include "Scheduler.h"
#include "Routines.h"
#include "Drivetrain.h"
#include "RobotConfig.h"
using namespace vex;

// define your global instances of motors and other devices here



int main() {
    build_routine_one();
    scheduler.schedule(&route_one);
    while(drivetrain_inertial.isCalibrating()){
        vex::task::sleep(10);
    }
    while(true){
        scheduler.run();
        drivebase.apply_motor_power();
        vex::task::sleep(10);
    }

}



