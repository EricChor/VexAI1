#include "JetsonSerial.h"
#include <stdio.h>
#include "RobotConfig.h"
#include <cstring>
#include <algorithm>

void JetsonSerial::JetsonSerialSetup(){
    serialPTR = fopen("/dev/serial1","r");
    if(!serialPTR){
        Brain.Screen.print("Failed to open dev/serial1");
        return;
    }
    Brain.Screen.print("Ready to receive commands");
    srand(time(NULL));
}

void JetsonSerial::update_block_pose(){
    if(fgets(buffer, sizeof(buffer), serialPTR) != NULL){
        len = std::strlen(buffer);
        if(len > 0 && (buffer[len-1] == '\r' || buffer[len-1] == '\r')){
            buffer[len-1] = '\0';
        }
        cmd_str = buffer;
    }
    std::strcpy(pos_string,cmd_str.c_str());
    for(int i = 0; i < 10; i++){
        if(pos_string[i] == ','){
            comma_index = i;
            continue;
        } else if (pos_string[i] == '\0'){
            null_index = i;
            break;
        }
    }
    tempX = cmd_str.substr(0,comma_index);
    tempY = cmd_str.substr(comma_index + 1, null_index - comma_index - 1);

    block_x_pos = std::atoi(tempX.c_str());
    block_y_pos = std::atoi(tempY.c_str());
}