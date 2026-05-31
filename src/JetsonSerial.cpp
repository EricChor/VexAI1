#include "JetsonSerial.h"
#include <stdio.h>
#include "RobotConfig.h"
#include "CommandStatus.h"
#include <cstring>
#include <cstdlib>
#include <algorithm>

void JetsonSerial::JetsonSerialSetup(){
    serialPTR = fopen("/dev/serial1","r");

    if(!serialPTR){
        Brain.Screen.print("Failed to open dev/serial1");
        return;
    }

    clear_blocks();

    Brain.Screen.print("Ready to receive commands");
    srand(time(NULL));
}

void JetsonSerial::clear_blocks(){
    block_count = 0;
    block_x_pos = -1;
    block_y_pos = -1;

    for(int i = 0; i < MAX_BLOCKS; i++){
        block_x_positions[i] = -1;
        block_y_positions[i] = -1;
    }
}

bool JetsonSerial::parse_xy_pair(const std::string& pair, int& x, int& y){
    size_t commaPos = pair.find(',');

    if(commaPos == std::string::npos){
        return false;
    }

    std::string xString = pair.substr(0, commaPos);
    std::string yString = pair.substr(commaPos + 1);

    if(xString.length() == 0 || yString.length() == 0){
        return false;
    }

    x = std::atoi(xString.c_str());
    y = std::atoi(yString.c_str());

    return true;
}

bool JetsonSerial::select_block_index(int index){
    if(index < 0 || index >= block_count){
        block_x_pos = -1;
        block_y_pos = -1;
        return false;
    }

    block_x_pos = block_x_positions[index];
    block_y_pos = block_y_positions[index];

    return true;
}

bool JetsonSerial::select_block_closest_to(int targetX, int targetY){
    if(block_count <= 0){
        block_x_pos = -1;
        block_y_pos = -1;
        return false;
    }

    int bestIndex = 0;
    long bestDistance = 2147483647;

    for(int i = 0; i < block_count; i++){
        long dx = block_x_positions[i] - targetX;
        long dy = block_y_positions[i] - targetY;

        // Manhattan distance is good enough here and avoids using sqrt.
        long distance = std::abs(dx) + std::abs(dy);

        if(distance < bestDistance){
            bestDistance = distance;
            bestIndex = i;
        }
    }

    return select_block_index(bestIndex);
}

void JetsonSerial::update_block_pose(){
    clear_blocks();

    if(serialPTR == nullptr){
        return;
    }

    if(fgets(buffer, sizeof(buffer), serialPTR) == NULL){
        return;
    }

    len = std::strlen(buffer);

    while(len > 0 && (buffer[len - 1] == '\r' || buffer[len - 1] == '\n')){
        buffer[len - 1] = '\0';
        len--;
    }

    cmd_str = buffer;

    if(cmd_str.length() == 0 || cmd_str == "0"){
        return;
    }

    /*
        New Python message format:
            0
        or:
            count;x1,y1;x2,y2;...

        Example:
            3;120,240;300,210;500,180

        This parser also supports the old format:
            x,y
    */

    size_t firstSemicolon = cmd_str.find(';');

    // Backward compatibility for the old "x,y" format.
    if(firstSemicolon == std::string::npos){
        int x = -1;
        int y = -1;

        if(parse_xy_pair(cmd_str, x, y)){
            block_x_positions[0] = x;
            block_y_positions[0] = y;
            block_count = 1;
            select_block_index(0);
        }

        return;
    }

    int expectedCount = std::atoi(cmd_str.substr(0, firstSemicolon).c_str());

    if(expectedCount <= 0){
        return;
    }

    size_t tokenStart = firstSemicolon + 1;

    while(tokenStart < cmd_str.length() && block_count < MAX_BLOCKS){
        size_t tokenEnd = cmd_str.find(';', tokenStart);

        std::string pair;

        if(tokenEnd == std::string::npos){
            pair = cmd_str.substr(tokenStart);
        } else {
            pair = cmd_str.substr(tokenStart, tokenEnd - tokenStart);
        }

        int x = -1;
        int y = -1;

        if(parse_xy_pair(pair, x, y)){
            block_x_positions[block_count] = x;
            block_y_positions[block_count] = y;
            block_count++;
        }

        if(tokenEnd == std::string::npos){
            break;
        }

        tokenStart = tokenEnd + 1;
    }

    // Default selected block is the first red block.
    // find_block_raw_step() and track_block_raw_step() may change this selection.
    if(block_count > 0){
        select_block_index(0);
    }
}

void JetsonSerial::find_block_raw_init(FindBlockRawConfig config){
    findBlockRawConfig = config;
    FindBlockRawVar& var = findBlockRawVar;

    var.lastBlockXPos = -1;
    var.lastBlockYPos = -1;
    var.lastBlockXDistance = 0;
    var.lastBlockYDistance = 0;
    var.sequentialBlocksCount = 0;
    var.x_is_stable = false;
    var.y_is_stable = false;
}

bool JetsonSerial::find_block_raw_step(){
    FindBlockRawVar& var = findBlockRawVar;
    FindBlockRawConfig& conf = findBlockRawConfig;

    update_block_pose();

    if(block_count <= 0){
        var.sequentialBlocksCount = 0;
        var.lastBlockXPos = -1;
        var.lastBlockYPos = -1;

        var.lastBlockXDistance = 0;
        var.lastBlockYDistance = 0;

        var.x_is_stable = false;
        var.y_is_stable = false;

        return false;
    }

    if(var.lastBlockXPos == -1 || var.lastBlockYPos == -1){
        // For the first valid frame, just pick the first red block.
        // After that, this function picks the current red block closest to the previous frame.
        select_block_index(0);

        var.lastBlockXPos = block_x_pos;
        var.lastBlockYPos = block_y_pos;

        var.lastBlockXDistance = 0;
        var.lastBlockYDistance = 0;

        var.sequentialBlocksCount = 0;

        var.x_is_stable = false;
        var.y_is_stable = false;

        return false;
    }

    // Choose the red block closest to the one from the previous frame.
    // This helps avoid jumping between multiple red blocks.
    select_block_closest_to(var.lastBlockXPos, var.lastBlockYPos);

    var.lastBlockXDistance = block_x_pos - var.lastBlockXPos;
    var.lastBlockYDistance = block_y_pos - var.lastBlockYPos;

    var.x_is_stable = std::abs(var.lastBlockXDistance) <= conf.maxDifferenceDistance;
    var.y_is_stable = std::abs(var.lastBlockYDistance) <= conf.maxDifferenceDistance;

    if(var.x_is_stable && var.y_is_stable){
        var.sequentialBlocksCount++;
    } else {
        var.sequentialBlocksCount = 0;
    }

    var.lastBlockXPos = block_x_pos;
    var.lastBlockYPos = block_y_pos;

    return var.sequentialBlocksCount >= conf.numSequentialBlocks;
}

void JetsonSerial::track_block_raw_init(TrackBlockRawConfig config){
    trackBlockRawConfig = config;
    TrackBlockRawVar& var = trackBlockRawVar;
    TrackBlockRawConfig& conf = trackBlockRawConfig;

    var.xError = 0;
    var.yError = 0;

    var.lastBlockXPos = -1;
    var.lastBlockYPos = -1;

    var.xJump = 0;
    var.yJump = 0;

    var.lostFrameCount = 0;

    var.targetVisible = false;
    var.trackingLocked = false;

    var.xJumpValid = false;
    var.yJumpValid = false;

    // Prefer the block that FindBlockCommand already selected and approved.
    // If tracking is started without a selected block, fall back to the closest
    // block to camera center.
    if(block_x_pos < 0 || block_y_pos < 0){
        select_block_closest_to(conf.cameraCenterX, conf.cameraCenterY);
    }

    if(block_x_pos >= 0 && block_y_pos >= 0){

        var.lastBlockXPos = block_x_pos;
        var.lastBlockYPos = block_y_pos;

        var.xError = block_x_pos - conf.cameraCenterX;
        var.yError = conf.cameraCenterY - block_y_pos;

        var.targetVisible = true;
        var.trackingLocked = true;

        var.xJumpValid = true;
        var.yJumpValid = true;
    }
}

bool JetsonSerial::track_block_raw_step(){
    TrackBlockRawVar& var = trackBlockRawVar;
    TrackBlockRawConfig& conf = trackBlockRawConfig;

    update_block_pose();

    // Sees no red blocks.
    if(block_count <= 0){
        var.lostFrameCount++;
        var.targetVisible = false;

        if(var.lostFrameCount > conf.maxLostFrames){
            var.trackingLocked = false;

            var.xError = 0;
            var.yError = 0;

            var.xJump = 0;
            var.yJump = 0;

            var.xJumpValid = false;
            var.yJumpValid = false;

            return false;
        }

        return var.trackingLocked;
    }

    // Sees at least one red block.
    var.targetVisible = true;

    // Sees a red block but tracking is not locked yet.
    // On a fresh lock, start closest to center. On a reacquire after losing
    // tracking, stay with the block closest to the last tracked position.
    if(!var.trackingLocked){
        if(var.lastBlockXPos >= 0 && var.lastBlockYPos >= 0){
            select_block_closest_to(var.lastBlockXPos, var.lastBlockYPos);
        } else {
            select_block_closest_to(conf.cameraCenterX, conf.cameraCenterY);
        }

        var.lastBlockXPos = block_x_pos;
        var.lastBlockYPos = block_y_pos;

        var.xJump = 0;
        var.yJump = 0;

        var.xJumpValid = true;
        var.yJumpValid = true;

        var.xError = block_x_pos - conf.cameraCenterX;
        var.yError = conf.cameraCenterY - block_y_pos;

        var.lostFrameCount = 0;
        var.trackingLocked = true;
        var.targetVisible = true;

        return true;
    }

    // Tracking is already locked.
    // Pick the current red block closest to the previous tracked position.
    select_block_closest_to(var.lastBlockXPos, var.lastBlockYPos);

    var.xJump = block_x_pos - var.lastBlockXPos;
    var.yJump = block_y_pos - var.lastBlockYPos;

    var.xJumpValid = std::abs(var.xJump) <= conf.maxTrackingXJump;
    var.yJumpValid = std::abs(var.yJump) <= conf.maxTrackingYJump;

    if(!var.xJumpValid || !var.yJumpValid){
        var.lostFrameCount++;
        var.targetVisible = false;

        if(var.lostFrameCount > conf.maxLostFrames){
            var.trackingLocked = false;

            var.xError = 0;
            var.yError = 0;

            var.xJump = 0;
            var.yJump = 0;

            var.xJumpValid = false;
            var.yJumpValid = false;

            return false;
        }

        return var.trackingLocked;
    }

    // The selected red block is accepted as the same tracked block.
    var.lastBlockXPos = block_x_pos;
    var.lastBlockYPos = block_y_pos;

    var.xError = block_x_pos - conf.cameraCenterX;
    var.yError = conf.cameraCenterY - block_y_pos;

    var.lostFrameCount = 0;

    var.trackingLocked = true;
    var.targetVisible = true;

    return true;
}

int JetsonSerial::getXError(){
    return trackBlockRawVar.xError;
}

int JetsonSerial::getYError(){
    return trackBlockRawVar.yError;
}

void JetsonSerial::print_block_pos_on_screen(){
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Selected X: %d    ", block_x_pos);

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Selected Y: %d    ", block_y_pos);

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Red Count: %d    ", block_count);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("X Error: %d    ", trackBlockRawVar.xError);

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Y Error: %d    ", trackBlockRawVar.yError);

    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Visible: %d    ", trackBlockRawVar.targetVisible);

    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print(
        "Wall Dist: %.2f        ",
        getLastWallAlignmentDistance()
    );

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("GPS X: %.2f    ", positionTracking.get_x());

    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("GPS Y: %.2f    ", positionTracking.get_y());

    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print("GPS H: %.2f    ", positionTracking.get_heading());

    Brain.Screen.setCursor(11, 1);
    Brain.Screen.print("IMU H: %.2f    ", drivebase.get_heading_degrees());

    Brain.Screen.setCursor(12, 1);
    Brain.Screen.print("Cmd: %s                    ", getCommandStatus());
}
