/*

Module:	cmdHang.cpp

Function:
        Process the "hang" command

Copyright and License:
        See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   April 2024

*/

#include "Catena4430_cmd.h"

#include <Arduino.h>

using namespace McciCatena;

/*

Name:   ::cmdHang()

Function:
        Command dispatcher for "hang" command.

Definition:
        McciCatena::cCommandStream::CommandFn cmdHang;

        McciCatena::cCommandStream::CommandStatus cmdHang(
            cCommandStream *pThis,
            void *pContext,
            int argc,
            char **argv
            );

Description:
        The "hang" command has the following syntax:

        hang
            enter a tight loop waiting for the watchdog to fire.

Returns:
        cCommandStream::CommandStatus::kSuccess if successful.
        Some other value for failure.

*/

// argv[0] is "hang"
// argv[1] is new log flag mask; if omitted, mask is printed
cCommandStream::CommandStatus cmdHang(
    cCommandStream *pThis,
    void *pContext,
    int argc,
    char **argv
    )
    {
    if (argc > 1)
        return cCommandStream::CommandStatus::kInvalidParameter;

    pThis->printf("looping... watchdog should get us out before we exit\n");

    uint32_t now = millis();

    while (millis() - now < 60 * 1000)
        ;
   
    pThis->printf("watchdog did not fire.\n");
    return cCommandStream::CommandStatus::kError;
    }