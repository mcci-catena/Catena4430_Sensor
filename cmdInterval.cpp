/*

Module:	cmdInterval.cpp

Function:
    Process the "interval" command

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
    Command dispatcher for "interval" command.

Definition:
    McciCatena::cCommandStream::CommandFn cmdInterval;

    McciCatena::cCommandStream::CommandStatus cmdInterval(
        cCommandStream *pThis,
        void *pContext,
        int argc,
        char **argv
        );

Description:
    The "interval" command has the following syntax:

    interval
        Display the current measurement interval.

    interval {number}
        Set the measurement interval {number}

Returns:
    cCommandStream::CommandStatus::kSuccess if successful.
    Some other value for failure.

*/

/* process "interval" */
// argv[0] is the matched command name
// argv[1] is the value
cCommandStream::CommandStatus cmdInterval(
    cCommandStream *pThis,
    void *pContext,
    int argc,
    char **argv
    )
    {
    cCommandStream::CommandStatus result;

    result = cCommandStream::CommandStatus::kInvalidParameter;
    if (argc == 1)
        {
        auto const info = gpMeasurementLoopConcrete->m_Scd.getInfo();
        gCatena.SafePrintf("%u secs\n", info.MeasurementInterval);
        result = cCommandStream::CommandStatus::kSuccess;
        }
    else if (argc != 2)
        {
        // do nothing
        }
    else
        {
        std::uint32_t interval;

        // set interval.
        result = cCommandStream::getuint32(argc, argv, 1, 10, interval, 0);
        if (result == cCommandStream::CommandStatus::kSuccess && interval <= UINT16_MAX)
            {
            if (! gpMeasurementLoopConcrete->m_Scd.setMeasurementInterval(std::uint16_t(interval)))
                {
                gCatena.SafePrintf("setMeasurementInterval failed: %s\n", gpMeasurementLoopConcrete->m_Scd.getLastErrorName());
                result = cCommandStream::CommandStatus::kError;
                }
            else
                {
                result = cCommandStream::CommandStatus::kSuccess;
                }
            }
        }

    return result;
    }
