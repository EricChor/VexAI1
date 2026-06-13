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

extern void build_iso_route();

extern SequentialCommandGroup AI_ISO_ROUTE;

extern void parking_route();

extern SequentialCommandGroup PARKING_AI_ROUTE;

