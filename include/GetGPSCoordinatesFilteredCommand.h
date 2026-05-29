#pragma once
#include "Command.h"
#include "PositionTracking.h"
#include "RobotConfig.h"
#include "CommandStatus.h"

struct GetGPSCoordinatesFilteredConfig{
    int sampleCount;
    int minAcceptedSamples;
    int sampleIntervals;
    int maxPositionJump;
    int maxHeadingJump;
};

struct GetGPSCoordinatesFilteredVar{
    int samplesTaken;
    int acceptedSamples;

    float nextSampleTime;
    
    float referenceX;
    float referenceY;
    float referenceHeading;

    float sumX;
    float sumY;
    float sumHeadingDelta;

    float currentX;
    float currentY;
    float currentHeading;

    float xDifference;
    float yDifference;

    float positionalDifference;
    float headingDifference;

    bool positionValid;
    bool headingValid;

    float filteredX;
    float filteredY;

    float averageHeadingDelta;
    float filteredHeading;
};

class GetGPSCoordinatesFilteredCommand : public Command{
    private:
        PositionTracking& positionTracking;
        GetGPSCoordinatesFilteredConfig getGPSCoordinatesFilteredConfig;
        GetGPSCoordinatesFilteredVar getGPSCoordinatesFilteredVar;

        bool finished;
        bool successful;

        float normalizeHeading(float heading){
            while(heading >= 360){
                heading -= 360;
            }

            while(heading < 0){
                heading += 360;
            }

            return heading;
        }

        float headingError(float target, float current){
            float error = target - current;

            if(abs(error) > 180){
                if(error > 0){
                    error -=360;
                } else {
                    error += 360;
                }
            }
            return error;
        }
    public:
        GetGPSCoordinatesFilteredCommand(PositionTracking& positionTracking, GetGPSCoordinatesFilteredConfig getGPSCoordinatesFilteredConfig)
        :positionTracking(positionTracking),
         getGPSCoordinatesFilteredConfig(getGPSCoordinatesFilteredConfig)
        {}

        void initialize() override{
            setCommandStatus("Read Filtered GPS");
            GetGPSCoordinatesFilteredVar& var = getGPSCoordinatesFilteredVar;
            finished = false;
            successful = false;

            var.samplesTaken = 0;
            var.acceptedSamples = 0;
            
            float now = master_timer.time();
            var.nextSampleTime = now;

            var.referenceX = positionTracking.get_raw_x();
            var.referenceY = positionTracking.get_raw_y();
            var.referenceHeading = positionTracking.get_raw_heading();

            var.sumX = 0;
            var.sumY = 0;
            var.sumHeadingDelta = 0;

            var.currentX = 0;
            var.currentY = 0;
            var.currentHeading = 0;

            var.xDifference = 0;
            var.yDifference = 0;

            var.positionalDifference = 0;
            var.headingDifference = 0;

            var.positionValid = false;
            var.headingValid = false;

            var.filteredX = 0;
            var.filteredY = 0;

            var.averageHeadingDelta = 0;
            var.filteredHeading = 0;
        }

        void execute() override{
            GetGPSCoordinatesFilteredVar& var = getGPSCoordinatesFilteredVar;
            GetGPSCoordinatesFilteredConfig& conf = getGPSCoordinatesFilteredConfig;
            float now = master_timer.time();

            if (now < var.nextSampleTime){
                return;
            }

            var.currentX = positionTracking.get_raw_x();
            var.currentY = positionTracking.get_raw_y();
            var.currentHeading = positionTracking.get_raw_heading();

            var.xDifference = var.currentX - var.referenceX;
            var.yDifference = var.currentY - var.referenceY;

            var.positionalDifference = sqrt(var.xDifference * var.xDifference + var.yDifference * var.yDifference);
            
            var.headingDifference = headingError(var.currentHeading,var.referenceHeading);

            var.positionValid = fabs(var.positionalDifference) <= conf.maxPositionJump;
            var.headingValid = fabs(var.headingDifference) <= conf.maxHeadingJump;

            if(var.positionValid && var.headingValid){
                var.sumX += var.currentX;
                var.sumY += var.currentY;
                var.sumHeadingDelta += var.headingDifference;
                var.acceptedSamples++;
            }

            var.samplesTaken++;
            var.nextSampleTime = now + conf.sampleIntervals;

            if(var.samplesTaken >= conf.sampleCount){
                finished = true;

                if(var.acceptedSamples >= conf.minAcceptedSamples){
                    var.filteredX = var.sumX / var.acceptedSamples;
                    var.filteredY = var.sumY / var.acceptedSamples;
                    var.averageHeadingDelta = var.sumHeadingDelta / var.acceptedSamples;

                    var.filteredHeading = normalizeHeading(var.referenceHeading + var.averageHeadingDelta);

                    positionTracking.set_pose(var.filteredX,var.filteredY,var.filteredHeading);
                    successful = true;
                } else {
                    successful = false;
                }
            }



        }

        bool isFinished() override{
            return finished;
        }

        void end() override{}

        bool wasSuccessful() const{
            return successful;
        }
};
