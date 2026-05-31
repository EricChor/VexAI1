#pragma once
#include "Scheduler.h"
#include "SequentialCommandGroup.h"
#include "ParallelCommandGroup.h"
#include "ParallelDeadlineGroup.h"
#include "RaceCommandGroup.h"
#include "GrabBlockCommand.h"
#include "RepeatForeverCommandGroup.h"

extern SequentialCommandGroup AI_ISOLATION_ROUTE;

extern void build_isolation_routine();