#include "ParallelDeadlineGroup.h"

void ParallelDeadlineGroup::addCommand(Command* command){
    if(command == nullptr){
        return;
    }
    command_list.push_back(command);
    commandFinished.push_back(false);
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
        commandFinished[i] = false;

        if(command_list[i]!= nullptr){
            command_list[i]->initialize();
        } else {
            commandFinished[i] = true;
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
        if (command_list[i] == nullptr || commandFinished[i]) {
            continue;
        }

        command_list[i]->execute();

        if (command_list[i]->isFinished()) {
            command_list[i]->end();
            commandFinished[i] = true;
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
        if(command_list[i] != nullptr && !commandFinished[i]){
            command_list[i]->end();
            commandFinished[i] = true;
        }
    }
    finished = true;
}
