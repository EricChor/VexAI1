#pragma once
#include "vex.h"

struct GPSPose{
    float x;
    float y;
    float heading;


};

class PositionTracking {
    private:
        vex::gps& GPSSensor;
        GPSPose gpsPose;
    public:
        PositionTracking();

        float normalize_heading(float heading);

        float get_raw_x();

        float get_raw_y();

        float get_raw_heading();

        void update_raw_pose();

        void set_pose(float x, float y, float heading);

        GPSPose getPose() const;

        float get_x() const;

        float get_y() const;

        float get_heading() const;
};