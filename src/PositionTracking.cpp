#include "PositionTracking.h"
#include "RobotConfig.h"

PositionTracking::PositionTracking()
:GPSSensor(::GPSSensor),
 gpsPose{0,0,0} {}

float PositionTracking::normalize_heading(float heading) {
    while (heading >= 360.0f) {
        heading -= 360.0f;
    }

    while (heading < 0.0f) {
        heading += 360.0f;
    }

    return heading;
}

float PositionTracking::get_raw_x(){
    return GPSSensor.xPosition(inches);
}
float PositionTracking::get_raw_y(){
    return GPSSensor.yPosition(inches);
}

float PositionTracking::get_raw_heading(){
    return  normalize_heading(GPSSensor.heading(degrees) + 180);
}

void PositionTracking::update_raw_pose(){
    gpsPose.x = get_raw_x();
    gpsPose.y = get_raw_y();
    gpsPose.heading = normalize_heading(GPSSensor.heading(degrees) + 180);
}

void PositionTracking::set_pose(float x, float y, float heading){
    gpsPose.x = x;
    gpsPose.y = y;
    gpsPose.heading = heading;
}

GPSPose PositionTracking::getPose() const{
    return gpsPose;
}

float PositionTracking::get_x() const{
    return gpsPose.x;
}

float PositionTracking::get_y() const{
    return gpsPose.y;
}

float PositionTracking::get_heading() const{
    return gpsPose.heading;
}