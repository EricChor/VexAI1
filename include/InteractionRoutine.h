#pragma once
#include "Scheduler.h"
#include "SequentialCommandGroup.h"
#include "ParallelCommandGroup.h"
#include "ParallelDeadlineGroup.h"
#include "RaceCommandGroup.h"
#include "GrabBlockCommand.h"
#include "RepeatForeverCommandGroup.h"

extern RepeatForeverCommandGroup AI_INTERACTION_ROUTE;

extern void build_interaction_routine();