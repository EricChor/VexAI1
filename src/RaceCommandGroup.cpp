#include "RaceCommandGroup.h"

void RaceCommandGroup::addCommand(Command* command){
    if (command == nullptr) {
        return;
    }
    command_list.push_back(command);
}

void RaceCommandGroup::initialize(){

    if(command_list.empty()){
        finished = true;
        return;
    }

    for(int i = 0; i < static_cast<int>(command_list.size()); i++){
        command_list[i]->initialize();
    }
    finished = false;
}

void RaceCommandGroup::execute(){
    if(finished){
        return;
    }

    for(int i = 0; i < static_cast<int>(command_list.size());i++){
        if(command_list[i] == nullptr){
            continue;
        }

        command_list[i]->execute();

        if(command_list[i]->isFinished()){
            finished = true;
            return;
        }
    }
}

bool RaceCommandGroup::isFinished(){
    return finished;
}

void RaceCommandGroup::end(){
    for(int i = 0; i < static_cast<int>(command_list.size()) ; i++){
        if(command_list[i] != nullptr){
            command_list[i]->end();
        }
    }
    finished = true;
}