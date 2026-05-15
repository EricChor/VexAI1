#include "SequentialCommandGroup.h"

void SequentialCommandGroup::addCommand(Command* command){
    if(command == nullptr){
        return;
    }
    command_list.push_back(command);
}

void SequentialCommandGroup::initialize(){
    currentIndex = 0;
    finished = false;
    currentCommandInitialized = false;

    if(command_list.empty()){
        finished = true;
    }
}

void SequentialCommandGroup::execute(){
    if(finished){
        return;
    }

    Command* current_command = command_list[currentIndex];

    if(current_command == nullptr){
        currentIndex++;
        currentCommandInitialized = false;

        if(currentIndex > static_cast<int>(command_list.size())){
            finished = true;
        }
        return;
    
    }

    if(!currentCommandInitialized){
        current_command->initialize();
        currentCommandInitialized = true;
    }

    current_command->execute();

    if(current_command->isFinished()){
        current_command->end();
        currentIndex++;
        currentCommandInitialized = false;

        if (currentIndex >= static_cast<int>(command_list.size())) {
                finished = true;
        }
    }
}

bool SequentialCommandGroup::isFinished(){
    return finished;
}

void SequentialCommandGroup::end(){
    if(!finished && currentIndex < static_cast<int>(command_list.size())){
        Command* current_command = command_list[currentIndex];
        if(current_command != nullptr){
            current_command->end();
        }
    }
    finished = true;
}

SequentialCommandGroup::~SequentialCommandGroup(){
    
}