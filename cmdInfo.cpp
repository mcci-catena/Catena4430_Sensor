/*

Module:	cmdInterval.cpp

Function:
    Process the "info" command

Copyright and License:
    This file copyright (C) 2022 by

        MCCI Corporation
        3520 Krums Corners Road
        Ithaca, NY  14850

    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation	September 2022

*/

#include "Catena4430_cmd.h"

#include <Catena4430_Sensor.h>

using namespace McciCatena;
using namespace McciCatena4430;

extern cMeasurementLoop *gpMeasurementLoopConcrete;

/*

Name:   ::cmdLog()

Function:
    Command dispatcher for "info" command.

Definition:
    McciCatena::cCommandStream::CommandFn cmdInfo;

    McciCatena::cCommandStream::CommandStatus cmdInfo(
        cCommandStream *pThis,
        void *pContext,
        int argc,
        char **argv
        );

Description:
    The "info" command has the following syntax:

    info
        Displays the config information of SCD30.

Returns:
    cCommandStream::CommandStatus::kSuccess if successful.
    Some other value for failure.

*/

/* process "info" -- args are ignored */
// argv[0] is the matched command name.
cCommandStream::CommandStatus cmdInfo(
        cCommandStream *pThis,
        void *pContext,
        int argc,
        char **argv
        )
        {
        bool fResult;

        fResult = false;
        if (argc == 1)
            {
            gpMeasurementLoopConcrete->printSCDinfo();
            fResult = true;
            }

        return fResult ? cCommandStream::CommandStatus::kSuccess
                       : cCommandStream::CommandStatus::kInvalidParameter
                       ;
        }
