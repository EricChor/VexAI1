#include "PositionTracking.h"
#include "RobotConfig.h"

#include <cmath>

namespace {
    const float MAIN_GPS_OFFSET_RIGHT_INCHES = 4.0f;
    const float MAIN_GPS_OFFSET_FORWARD_INCHES = -7.0f;

    const float BACKUP_GPS_OFFSET_RIGHT_INCHES = 0.0f;
    const float BACKUP_GPS_OFFSET_FORWARD_INCHES = 7.0f;

    const float MAIN_GPS_HEADING_OFFSET_DEGREES = 180.0f;
    const float BACKUP_GPS_HEADING_OFFSET_DEGREES = 0.0f;

    const float MIN_USABLE_GPS_QUALITY = 75.0f;
    const float GPS_QUALITY_SWITCH_MARGIN = 5.0f;
    const float PI = 3.14159265f;

    GPSPose make_center_pose_from_sensor(
        vex::gps& sensor,
        float heading,
        float rightOffsetInches,
        float forwardOffsetInches
    ) {
        float headingRadians = heading * PI / 180.0f;

        float sensorOffsetX =
            rightOffsetInches * std::cos(headingRadians) +
            forwardOffsetInches * std::sin(headingRadians);

        float sensorOffsetY =
            -rightOffsetInches * std::sin(headingRadians) +
            forwardOffsetInches * std::cos(headingRadians);

        return {
            static_cast<float>(sensor.xPosition(inches) - sensorOffsetX),
            static_cast<float>(sensor.yPosition(inches) - sensorOffsetY),
            heading
        };
    }
}

PositionTracking::PositionTracking()
:GPSSensor(::GPSSensor),
 GPSBackup(::GPSBackup),
 gpsPose{0,0,0},
 backupToMainOffsetX(0),
 backupToMainOffsetY(0),
 backupOffsetInitialized(false) {}

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
    update_raw_pose();
    return gpsPose.x;
}
float PositionTracking::get_raw_y(){
    update_raw_pose();
    return gpsPose.y;
}

float PositionTracking::get_raw_heading(){
    update_raw_pose();
    return gpsPose.heading;
}

float PositionTracking::get_main_gps_quality() {
    return GPSSensor.quality();
}

float PositionTracking::get_backup_gps_quality() {
    return GPSBackup.quality();
}

void PositionTracking::update_raw_pose(){
    float mainHeading =
        normalize_heading(GPSSensor.heading(degrees) + MAIN_GPS_HEADING_OFFSET_DEGREES);

    float backupHeading =
        normalize_heading(GPSBackup.heading(degrees) + BACKUP_GPS_HEADING_OFFSET_DEGREES);

    GPSPose mainPose =
        make_center_pose_from_sensor(
            GPSSensor,
            mainHeading,
            MAIN_GPS_OFFSET_RIGHT_INCHES,
            MAIN_GPS_OFFSET_FORWARD_INCHES
        );

    GPSPose backupPose =
        make_center_pose_from_sensor(
            GPSBackup,
            backupHeading,
            BACKUP_GPS_OFFSET_RIGHT_INCHES,
            BACKUP_GPS_OFFSET_FORWARD_INCHES
        );

    float mainQuality = get_main_gps_quality();
    float backupQuality = get_backup_gps_quality();

    bool mainUsable = mainQuality >= MIN_USABLE_GPS_QUALITY;
    bool backupUsable = backupQuality >= MIN_USABLE_GPS_QUALITY;

    if (mainUsable && backupUsable) {
        backupToMainOffsetX = mainPose.x - backupPose.x;
        backupToMainOffsetY = mainPose.y - backupPose.y;
        backupOffsetInitialized = true;
    }

    if (backupOffsetInitialized) {
        backupPose.x += backupToMainOffsetX;
        backupPose.y += backupToMainOffsetY;
    }

    bool useBackup =
        backupUsable &&
        (
            !mainUsable ||
            backupQuality >= mainQuality + GPS_QUALITY_SWITCH_MARGIN
        );

    if (useBackup) {
        gpsPose = backupPose;
        return;
    }

    gpsPose = mainPose;
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
