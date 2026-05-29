#include "PositionTracking.h"
#include "RobotConfig.h"

#include <cmath>

namespace {
    const float GPS_OFFSET_RIGHT_INCHES = 4.0f;
    const float GPS_OFFSET_FORWARD_INCHES = -7.0f;
    const float PI = 3.14159265f;
}

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
    float headingRadians = get_raw_heading() * PI / 180.0f;

    float sensorOffsetX =
        GPS_OFFSET_RIGHT_INCHES * std::cos(headingRadians) +
        GPS_OFFSET_FORWARD_INCHES * std::sin(headingRadians);

    return GPSSensor.xPosition(inches) - sensorOffsetX;
}
float PositionTracking::get_raw_y(){
    float headingRadians = get_raw_heading() * PI / 180.0f;

    float sensorOffsetY =
        -GPS_OFFSET_RIGHT_INCHES * std::sin(headingRadians) +
        GPS_OFFSET_FORWARD_INCHES * std::cos(headingRadians);

    return GPSSensor.yPosition(inches) - sensorOffsetY;
}

float PositionTracking::get_raw_heading(){
    return  normalize_heading(GPSSensor.heading(degrees) + 180);
}

void PositionTracking::update_raw_pose(){
    gpsPose.x = get_raw_x();
    gpsPose.y = get_raw_y();
    gpsPose.heading = get_raw_heading();
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
