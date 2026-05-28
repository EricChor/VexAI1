// #include "Routines.h"

// #include "RobotConfig.h"
// #include "Drivetrain.h"
// #include "Intake.h"

// #include "SequentialCommandGroup.h"
// #include "ParallelDeadlineGroup.h"

// #include "FindBlockCommand.h"
// #include "TrackBlockCommand.h"
// #include "IntakeWithSortingCommand.h"

// #include "GrabBlockCommand.h"
// #include "DriveStraightCommand.h"

// #include "GetGPSCoordinatesFilteredCommand.h"

// // ------------------------------------------------------------
// // PID constants
// // ------------------------------------------------------------

// DrivePID grabBlockPID{
//     .linear_kP = 3,
//     .linear_kI = 0.0,
//     .linear_kD = 0.25,

//     .angular_kP = 0.05,
//     .angular_kI = 0.0,
//     .angular_kD = 0.1,

//     .angular_integral_windup_threshold = 20,
//     .linear_integral_windup_threshold = 20
// };

// DrivePID defaultPID{
//     .linear_kP = 0.25,
//     .linear_kI = 0.0,
//     .linear_kD = 0.05,

//     .angular_kP = 0.05,
//     .angular_kI = 0.0,
//     .angular_kD = 0.1,

//     .angular_integral_windup_threshold = 20,
//     .linear_integral_windup_threshold = 20
// };

// // ------------------------------------------------------------
// // Raw Jetson vision configs
// // ------------------------------------------------------------

// FindBlockRawConfig findBlockRawConfig{
//     .numSequentialBlocks = 3,
//     .maxDifferenceDistance = 150
// };

// TrackBlockRawConfig trackBlockRawConfig{
//     .cameraCenterX = 320,
//     .cameraCenterY = 400,

//     .maxTrackingXJump = 150,
//     .maxTrackingYJump = 150,

//     .maxLostFrames = 8 
// };

// // ------------------------------------------------------------
// // Find block command config
// // ------------------------------------------------------------

// // findingBlocksConfig findBlockConfig{
// //     .searchSpeed = 10,
// //     .maxSpeed = 25,
// //     .centeringAcceptableX = 15,
// //     .forwardSpeed = 15,
// //     .reverseSpeed = 15,
// //     .stuckCheckTime = 1000,
// //     .stuckEncoderChangeThreshold = 90,
// //     .stuckHeadingChangeThreshold = 15,
// //     .maxForwardTime = 2000,
// //     .maxReverseTime = 2000
// // };

// findingBlocksConfig findBlockConfig{
//         10.0f,   // searchSpeed
//         20.0f,   // maxSpeed
//         15,      // centeringAcceptableX

//         20.0f,   // reverseSpeed
//         20.0f,   // forwardSpeed

//         500.0f,  // stuckCheckTime, ms
//         3.0f,    // stuckHeadingChangeThreshold, degrees
//         20.0f,   // stuckEncoderChangeThreshold, motor degrees

//         800.0f,  // maxReverseTime, ms
//         500.0f,  // maxForwardTime, ms

//     {
//         {.minX = -12.0, .maxX=12.0f, .minY=-12.0f, .maxY=12.0f, true},
//         {.minX = -24.0, .maxX=24.0f, .minY=48.0f, .maxY=72.0f, true},
//         {.minX = -24.0, .maxX=24.0f, .minY=-48.0f, .maxY=-72.0f, true}
//     },

//     400.0f,   // avoidTurnTime, ms
//     5
// };
// // ------------------------------------------------------------
// // Track block command config
// // ------------------------------------------------------------

// TrackingBlocksConfig trackBlockConfig = {
//     10,      // acceptableYError
//     12,      // acceptableXError

//     25.0f,   // maxLinearSpeed
//     20.0f,   // maxAngularSpeed

//     500.0f,  // stuckCheckTime, ms
//     20.0f,   // stuckEncoderChangeThreshold, motor degrees
//     3.0f,    // stuckHeadingChangeThreshold, degrees, not heavily used here yet

//     20.0f,   // reverseSpeed
//     20.0f,   // forwardSpeed
//     15.0f,   // turnSpeed

//     500.0f,  // maxReverseTime
//     300.0f,  // maxForwardTime
//     300.0f,  // maxTurnTime

//     {
//         {.minX = -12.0, .maxX=12.0f, .minY=-12.0f, .maxY=12.0f, true},
//         {.minX = -24.0, .maxX=24.0f, .minY=48.0f, .maxY=72.0f, true},
//         {.minX = -24.0, .maxX=24.0f, .minY=-72.0f, .maxY=-48.0f, true}
//     }
// };

// // ------------------------------------------------------------
// // Intake command config
// // ------------------------------------------------------------

// IntakeWithSortingConfig intakeSortingConfig{
//     .initial_intake_velocity = 100,
//     .middle_intake_velcoity = 100,
//     .final_intake_velocity = 100,

//     .red_value = 20,
//     .blue_value = 220,

//     .red_threshold = 25,
//     .blue_threshold = 25,

//     .threshold_velocity = 5,

//     .sorting_time = 600,
//     .accept_time = 600,
//     .unjamming_time = 300


// };

// GrabBlockConfig grabBlockConfig{
//     .acceptable_error = 0.5,
//     .max_accel = 0,
//     .max_angular_speed = 25,
//     .max_linear_speed = 40,
//     .max_time = 3000,
//     .settle_time = 250
// };
// GrabBlockTarget grabBlockTarget{
//     .target_distance = 6,
//     .target_heading = 0
// };

// GetGPSCoordinatesFilteredConfig gpsConfig{
//     .maxHeadingJump = 4,
//     .maxPositionJump = 3,
//     .minAcceptedSamples = 5,
//     .sampleCount = 10,
//     .sampleIntervals = 100  
// };


// // ------------------------------------------------------------
// // Main routine objects
// // ------------------------------------------------------------

// SequentialCommandGroup AI_ROUTE_1;
// RepeatForeverCommandGroup AI_ROUTE_ONE(AI_ROUTE_1);

// // ------------------------------------------------------------
// // Build routine
// // ------------------------------------------------------------
// static GrabBlockCommand* grabBlockPtr = nullptr;
// bool grabbedBlockSuccessfully() {
//     if (grabBlockPtr == nullptr) {
//         return false;
//     }
//     return grabBlockPtr->wasSuccessful();
// }



// void build_AI_routine() {
//     static FindBlockCommand find_block(
//         drivebase,
//         jetsonSerial,
//         findBlockConfig,
//         findBlockRawConfig,
//         trackBlockRawConfig,
//         defaultPID
//     );

//     static TrackBlockCommand track_block(
//         drivebase,
//         jetsonSerial,
//         trackBlockConfig,
//         trackBlockRawConfig,
//         defaultPID
//     );

//     static IntakeWithSorting intake_block(
//         intake,
//         intakeSortingConfig
//     );

//     static GrabBlockCommand grabBlock(
//         drivebase,
//         grabBlockConfig,
//         grabBlockPID,
//         grabBlockTarget
//     );

//     static GetGPSCoordinatesFilteredCommand updateGPS(positionTracking,gpsConfig);

//     grabBlockPtr = &grabBlock;

//     static ParallelDeadlineGroup track_and_intake(&track_block);
//     static ParallelDeadlineGroup grab_and_intake(&grabBlock);

//     track_and_intake.addCommand(&intake_block);
//     grab_and_intake.addCommand(&intake_block);

//     AI_ROUTE_1.addCommand(&updateGPS);
//     AI_ROUTE_1.addCommand(&find_block);
//     AI_ROUTE_1.addCommand(&updateGPS);
//     AI_ROUTE_1.addCommand(&track_and_intake);
//     AI_ROUTE_1.addCommand(&grab_and_intake);

// }
#include "Routines.h"

#include "RobotConfig.h"
#include "Drivetrain.h"
#include "Intake.h"
#include "PositionTracking.h"

#include "SequentialCommandGroup.h"
#include "RepeatForeverCommandGroup.h"
#include "ParallelDeadlineGroup.h"
#include "ConditionalCommandGroup.h"

#include "FindBlockCommand.h"
#include "TrackBlockCommand.h"
#include "IntakeWithSortingCommand.h"
#include "GrabBlockCommand.h"
#include "DriveStraightCommand.h"
#include "GetGPSCoordinatesFilteredCommand.h"

// ------------------------------------------------------------
// Simple skip command
// ------------------------------------------------------------

class DoNothingCommand : public Command {
private:
    bool finished;

public:
    DoNothingCommand()
        : finished(false)
    {
    }

    void initialize() override {
        finished = true;
    }

    void execute() override {
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
    }
};

// ------------------------------------------------------------
// Set drivetrain pose from filtered GPS
// ------------------------------------------------------------

class SetDrivetrainPoseFromGPSCommand : public Command {
private:
    Drivetrain& drivetrain;
    PositionTracking& positionTracking;

    bool finished;

public:
    SetDrivetrainPoseFromGPSCommand(
        Drivetrain& drivetrain,
        PositionTracking& positionTracking
    )
        : drivetrain(drivetrain),
          positionTracking(positionTracking),
          finished(false)
    {
    }

    void initialize() override {
        GPSPose pose = positionTracking.getPose();

        drivetrain.set_odom_pose(
            pose.x,
            pose.y,
            pose.heading
        );

        finished = true;
    }

    void execute() override {
    }

    bool isFinished() override {
        return finished;
    }

    void end() override {
    }
};

// ------------------------------------------------------------
// Track-block success wrapper
// ------------------------------------------------------------

static TrackBlockCommand* trackBlockPtr = nullptr;

bool trackBlockWasSuccessful() {
    if (trackBlockPtr == nullptr) {
        return false;
    }

    return trackBlockPtr->wasSuccessful();
}

// ------------------------------------------------------------
// Main routine objects
// ------------------------------------------------------------

SequentialCommandGroup AI_ROUTE_1;
RepeatForeverCommandGroup AI_ROUTE_ONE(AI_ROUTE_1);

// ------------------------------------------------------------
// PID constants
// ------------------------------------------------------------

DrivePID grabBlockPID {
    .linear_kP = 3,
    .linear_kI = 0.0,
    .linear_kD = 0.25,

    .angular_kP = 0.05,
    .angular_kI = 0.0,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 20,
    .linear_integral_windup_threshold = 20
};

DrivePID defaultPID {
    .linear_kP = 0.25,
    .linear_kI = 0.0,
    .linear_kD = 0.05,

    .angular_kP = 0.05,
    .angular_kI = 0.0,
    .angular_kD = 0.1,

    .angular_integral_windup_threshold = 20,
    .linear_integral_windup_threshold = 20
};

// ------------------------------------------------------------
// Raw Jetson vision configs
// ------------------------------------------------------------

FindBlockRawConfig findBlockRawConfig {
    .numSequentialBlocks = 3,
    .maxDifferenceDistance = 150
};

TrackBlockRawConfig trackBlockRawConfig {
    .cameraCenterX = 320,
    .cameraCenterY = 400,

    .maxTrackingXJump = 150,
    .maxTrackingYJump = 150,

    .maxLostFrames = 8
};

// ------------------------------------------------------------
// Find block command config
// ------------------------------------------------------------

findingBlocksConfig findBlockConfig {
    .searchSpeed = 10,
    .maxSpeed = 25,
    .centeringAcceptableX = 15,

    .reverseSpeed = 15,
    .forwardSpeed = 15,

    .stuckCheckTime = 1000,

    .stuckHeadingChangeThreshold = 5,
    .stuckEncoderChangeThreshold = 20,

    .maxReverseTime = 2000,
    .maxForwardTime = 2000,

    .avoidZones = {
        {
            .minX = -12.0f,
            .maxX = 12.0f,
            .minY = -12.0f,
            .maxY = 12.0f,
            .enabled = true
        },
        {
            .minX = -24.0f,
            .maxX = 24.0f,
            .minY = 48.0f,
            .maxY = 72.0f,
            .enabled = true
        },
        {
            .minX = -24.0f,
            .maxX = 24.0f,
            .minY = -72.0f,
            .maxY = -48.0f,
            .enabled = true
        }
    },

    .avoidTurnTime = 500,

    .maxCenteringDroppedFrames = 5
};

// ------------------------------------------------------------
// Track block command config
// ------------------------------------------------------------

TrackingBlocksConfig trackBlockConfig {
    .acceptableYError = 10,
    .acceptableXError = 12,

    .maxLinearSpeed = 25,
    .maxAngularSpeed = 20,

    .stuckCheckTime = 750,
    .stuckEncoderChangeThreshold = 10,
    .stuckHeadingChangeThreshold = 3,

    .reverseSpeed = 20,
    .forwardSpeed = 20,
    .turnSpeed = 15,

    .maxReverseTime = 500,
    .maxForwardTime = 300,
    .maxTurnTime = 300,
    .minLinearSpeedForStuckCheck = 10,
    .minYErrorProgress = 5,
    .avoidZones = {
        {
            .minX = -12.0f,
            .maxX = 12.0f,
            .minY = -12.0f,
            .maxY = 12.0f,
            .enabled = true
        },
        {
            .minX = -24.0f,
            .maxX = 24.0f,
            .minY = 48.0f,
            .maxY = 72.0f,
            .enabled = true
        },
        {
            .minX = -24.0f,
            .maxX = 24.0f,
            .minY = -72.0f,
            .maxY = -48.0f,
            .enabled = true
        }
    }

};

// ------------------------------------------------------------
// Intake command config
// ------------------------------------------------------------

IntakeWithSortingConfig intakeSortingConfig {
    .initial_intake_velocity = 100,
    .middle_intake_velcoity = 100,
    .final_intake_velocity = 100,

    .red_value = 20,
    .blue_value = 220,

    .red_threshold = 50,
    .blue_threshold = 50,

    .threshold_velocity = 5,

    .sorting_time = 600,
    .accept_time = 700,
    .unjamming_time = 300
};

// ------------------------------------------------------------
// Grab block config
// ------------------------------------------------------------

GrabBlockConfig grabBlockConfig {
    .acceptable_error = 0.5,
    .max_accel = 150,
    .max_angular_speed = 25,
    .max_linear_speed = 40,
    .max_time = 5000,
    .settle_time = 250
};

GrabBlockTarget grabBlockTarget {
    .target_distance = 10,
    .target_heading = 0
};

// ------------------------------------------------------------
// GPS config
// ------------------------------------------------------------

GetGPSCoordinatesFilteredConfig gpsConfig {
    .maxHeadingJump = 4,
    .maxPositionJump = 3,
    .minAcceptedSamples = 5,
    .sampleCount = 10,
    .sampleIntervals = 100
};

// ------------------------------------------------------------
// Commands
// ------------------------------------------------------------

static GetGPSCoordinatesFilteredCommand updateGPS(
    positionTracking,
    gpsConfig
);

static SetDrivetrainPoseFromGPSCommand setPoseFromGPS(
    drivebase,
    positionTracking
);

static FindBlockCommand find_block(
    drivebase,
    jetsonSerial,
    findBlockConfig,
    findBlockRawConfig,
    trackBlockRawConfig,
    defaultPID
);

static TrackBlockCommand track_block(
    drivebase,
    jetsonSerial,
    trackBlockConfig,
    trackBlockRawConfig,
    defaultPID
);

static GrabBlockCommand grabBlock(
    drivebase,
    grabBlockConfig,
    grabBlockPID,
    grabBlockTarget
);

static IntakeWithSorting intake_while_tracking(
    intake,
    intakeSortingConfig
);

static IntakeWithSorting intake_while_grabbing(
    intake,
    intakeSortingConfig
);

static DoNothingCommand skip_grab;

// ------------------------------------------------------------
// Command groups
// ------------------------------------------------------------

static ParallelDeadlineGroup track_and_intake(&track_block);
static ParallelDeadlineGroup grab_and_intake(&grabBlock);

static ConditionalCommandGroup choose_grab_or_skip(
    trackBlockWasSuccessful,
    &grab_and_intake,
    &skip_grab
);

// ------------------------------------------------------------
// Build routine
// ------------------------------------------------------------

void build_AI_routine() {
    static bool built = false;

    if (built) {
        return;
    }

    built = true;

    trackBlockPtr = &track_block;

    track_and_intake.addCommand(&intake_while_tracking);
    grab_and_intake.addCommand(&intake_while_grabbing);

    AI_ROUTE_1.addCommand(&updateGPS);
    AI_ROUTE_1.addCommand(&setPoseFromGPS);

    AI_ROUTE_1.addCommand(&find_block);

    AI_ROUTE_1.addCommand(&updateGPS);
    AI_ROUTE_1.addCommand(&setPoseFromGPS);

    AI_ROUTE_1.addCommand(&track_and_intake);

    AI_ROUTE_1.addCommand(&choose_grab_or_skip);
}