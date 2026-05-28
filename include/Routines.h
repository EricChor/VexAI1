#pragma once
#include "Scheduler.h"
#include "SequentialCommandGroup.h"
#include "ParallelCommandGroup.h"
#include "ParallelDeadlineGroup.h"
#include "RaceCommandGroup.h"
#include "GrabBlockCommand.h"
#include "RepeatForeverCommandGroup.h"

extern RepeatForeverCommandGroup AI_ROUTE_ONE;

extern void build_AI_routine();