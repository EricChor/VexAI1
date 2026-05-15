#include "ParallelCommandGroup.h"

void ParallelCommandGroup::initialize() {
    finished = false;
    for(int i = 0; i < command_list.size(); i++){
        commandFinished[i] = false;

        if(command_list[i] != nullptr){
            command_list[i]->initialize();
        }

        if(command_list.empty()){
            finished = true;
        }
    }
}

void ParallelCommandGroup::execute(){
    if(finished){
        return;
    }

    bool allFinished = true;

    for(int i = 0; i < command_list.size(); i++){
        Command* current_command = command_list[i];

        if(current_command == nullptr){
            commandFinished[i] = true;
        }

        if(!commandFinished[i]){
            current_command->execute();

            if(current_command->isFinished()){
                current_command->end();
                commandFinished[i] = true;
            }
        }

        if(!commandFinished[i]){
            allFinished = false;
        }
    }
    finished = allFinished;
}

bool ParallelCommandGroup::isFinished(){
    return finished;
}

void ParallelCommandGroup::end(){
    for(int i = 0; i < command_list.size(); i++){
        if(command_list[i] != nullptr && !commandFinished[i]){
            command_list[i]->end();
            commandFinished[i] = true;
        }
    }
    finished = true;
}

ParallelCommandGroup::~ParallelCommandGroup(){
    
}