#include "ParallelDeadlineGroup.h"

void ParallelDeadlineGroup::addCommand(Command* command){
    if(command == nullptr){
        return;
    }
    command_list.push_back(command);
}

void ParallelDeadlineGroup::initialize(){
    finished = false;
    if(deadlineCommand != nullptr){
        deadlineCommand->initialize();
    } else {
        finished = true;
        return;
    }

    for(int i = 0; i < static_cast<int>(command_list.size()); i++){
        if(command_list[i]!= nullptr){
            command_list[i]->initialize();
        }
    }
}

void ParallelDeadlineGroup::execute(){
    if(finished){
        return;
    }

    if(deadlineCommand == nullptr){
        finished = true;
        return;
    }

    deadlineCommand->execute();

    for (int i = 0; i < static_cast<int>(command_list.size()); i++) {
        if (command_list[i] != nullptr) {
            command_list[i]->execute();
        }
    }

    if(deadlineCommand->isFinished()){
        finished = true;
    }
}

bool ParallelDeadlineGroup::isFinished(){
    return finished;
}

void ParallelDeadlineGroup::end(){
    if(deadlineCommand != nullptr){
        deadlineCommand->end();
    }

    for(int i = 0; i < static_cast<int>(command_list.size()); i++){
        if(command_list[i] != nullptr){
            command_list[i]->end();
        }
    }
    finished = true;
}