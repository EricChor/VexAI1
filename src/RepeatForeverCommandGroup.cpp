#include "RepeatForeverCommandGroup.h"

void RepeatForeverCommandGroup::initialize(){
    command.initialize();
}

void RepeatForeverCommandGroup::execute(){

    command.execute();

    if(command.isFinished()){
        command.end();
        command.initialize();
    }
}
bool RepeatForeverCommandGroup::isFinished(){
    return false;
}
void RepeatForeverCommandGroup::end(){
    command.end();
}
