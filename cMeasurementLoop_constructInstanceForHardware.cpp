/*

Module:	constructInstanceForHardware.cpp

Function:
    McciCatena4430::cMeasurementLoop::constructInstanceForHardware

Copyright and License:
    This file copyright (C) 2023 by

        MCCI Corporation
        3520 Krums Corners Road
        Ithaca, NY  14850

    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar, MCCI Corporation	February 2023

*/

#include "Catena4430_cMeasurementLoop.h"
#include "Catena4430_cMeasurementLoopV1.h"
#include "Catena4430_cMeasurementLoopV2.h"
#include <Catena_functional.h>
#include <Catena4430.h>
#include <stdint.h>
#include <Wire.h>

using namespace McciCatena;
using namespace McciCatena4430;

/*

Name:	cMeasurementLoop::constructInstanceForHardware()

Function:
    Allocate and initialize a cMeasurementLoop that is suitable for this hardware.

Definition:
    cMeasurementLoop * cMeasurementLoop::constructInstanceForHardware(
        bool fVersion2
        );

Description:
    We need to run this app on any 4430 hardware, but there's a problem: the
    v1 4610 is different than the v2 4610: different light sensor, different
    temperature/rh sensor. This in turns means that we need different C++
    objects to represent the different sensors. We have limited RAM, so we
    do not want to instantiate both objects; and we also do not want to
    ask new() to allocate the RAM (because that might fail with an
    exception, which is not really supported on Arduino -- and that would
    be confusing).

    The solution we choose has three parts.

    1. We define v1 and v2 variants of cMeasurementLoop, and we make
       the old cMeasurementLoop somewhat abstract -- the sensors are
       accessed by virtual methods (where appropriate) to isolate
       the application code from the measurement implementation.

    2. We allocate a static buffer (at compile time) that is big enough
       to hold either of the v1 or v2 variant objects.

    3. At run time, we use a "placement new()" to allocate an construct
       whichever version (v1 or v2) is appropriate for the running
       hardware, using the buffer from step (2) above.

    For legacy reasons, this function calls the ::beginSensors()
    method before returning.

Returns:
    Pointer to the new object.

Notes:
    We really should call beginSensors() from a wrapper, it's not
    related to main function of this routine, which is to do the
    construction of a suitable cMeasurementLoop variant.

*/

cMeasurementLoop *cMeasurementLoop::constructInstanceForHardware(
    bool version2
    )
    {
    auto const alignVone = alignof(cMeasurementLoopV1);
    auto const alignVtwo = alignof(cMeasurementLoopV2);
    constexpr auto alignData = max(alignVone, alignVtwo);
    constexpr size_t bufSize = max(sizeof(cMeasurementLoopV1), sizeof(cMeasurementLoopV2));

    // Allocate the buffer, which is big enough for either v1, v2, and aligned properly
    // for either v1 or v2. Only one buffer, so that don't need a wasted copy of the object
    // for the hardware we're *not* running on. Do this inside
    // this function, so it's private, but as a static, so that the
    // allocation happens at link time.
    static alignas(alignData) uint8_t buf[bufSize];

    if (!version2)
        {
        // initialize a v1 measurement loop object, using the buffer
        // as the underlying memory. This runs the constructors.
        auto p = new(buf) cMeasurementLoopV1();

        // Return a pointer; since we're returning a pointer, the buffer
        // can't be on the stack, but must be a static or malloc'ed.
        // (We use a local static.)
        return p;
        }
    else {
        // initialize a v2 measurement loop object, using the buffer
        // as the underlying memory. This runs the constructors.
        auto p = new(buf) cMeasurementLoopV2();

        // Return a pointer; since we're returning a pointer, the buffer
        // can't be on the stack, but must be a static or malloc'ed.
        // (We use a local static.)
        return p;
        }
    }
