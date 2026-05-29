#include "CommandStatus.h"

namespace {
    const char* currentCommandStatus = "Starting";
    float lastWallAlignmentDistance = 0.0f;
}

void setCommandStatus(const char* status) {
    currentCommandStatus = status;
}

const char* getCommandStatus() {
    return currentCommandStatus;
}

void setLastWallAlignmentDistance(float distance) {
    lastWallAlignmentDistance = distance;
}

float getLastWallAlignmentDistance() {
    return lastWallAlignmentDistance;
}
