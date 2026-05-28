#include "ConditionalCommandGroup.h"

void ConditionalCommandGroup::initialize(){
    finished = false;
    initializeSelected = false;

    if(condition()){
        selectedCommand = trueCommand;
    } else {
        selectedCommand = falseCommand;
    }

    if(selectedCommand == nullptr){
        finished = true;
        return;
    }

    selectedCommand->initialize();
    initializeSelected = true;
}

void ConditionalCommandGroup::execute(){
    if(finished || selectedCommand == nullptr){
        return;
    }

    selectedCommand->execute();

    if(selectedCommand->isFinished()){
        selectedCommand->end();
        finished = true;
    }
}

bool ConditionalCommandGroup::isFinished(){
    return finished;
}

void ConditionalCommandGroup::end(){
    if(!isFinished() && selectedCommand != nullptr && initializeSelected){
        selectedCommand->end();
    }

    finished = true;
}