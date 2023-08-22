/*

Module: Catena4430_cMeasurementLoopV1.h

Function:
    cMeasurementLoopV1 definitions.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   October 2022

*/

#ifndef _Catena4430_cMeasurementLoopV1_h_
# define _Catena4430_cMeasurementLoopV1_h_

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Catena_Si1133.h>
#include <Catena.h>
#include "Catena4430_cMeasurementLoop.h"
#include <mcciadk_baselib.h>
#include <stdlib.h>

#include <cstdint>

extern McciCatena::Catena gCatena;

namespace McciCatena4430 {

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

template <unsigned a_kMaxActivityEntries>
class cMeasurementFormatV1 : public cMeasurementBase
    {
public:
    static constexpr uint8_t kMessageFormatV1 = 0x26;

    enum class Flags : uint8_t
            {
            Vbat = 1 << 0,      // vBat
            Version = 1 << 1,   // Sketch version
            CO2ppm = 1 << 2,    // Carbondioxide (SCD30)
            Boot = 1 << 3,      // boot count
            TPH = 1 << 4,       // temperature, pressure2, humidity
            Light = 1 << 5,     // light (IR, white, UV)
            Pellets = 1 << 6,   // Pellet count
            Activity = 1 << 7,  // Activity (min/max/avg)
            };

    static constexpr unsigned kMaxActivityEntries = a_kMaxActivityEntries;
    static constexpr size_t kTxBufferSize = (1 + 4 + 1 + 2 + 4 + 2 + 1 + 6 + 2 + 6 + kMaxActivityEntries * 2);

    // the structure of a measurement
    struct Measurement
        {
        //----------------
        // the subtypes:
        //----------------

        // environmental measurements
        struct Env
            {
            // temperature (in degrees C)
            float                   Temperature;
            // pressure (in millibars/hPa)
            float                   Pressure;
            // humidity (in % RH)
            float                   Humidity;
            };

        // ambient light measurements
        struct Light
            {
            // "white" light, in w/m^2
            float                   White;
            };

        //---------------------------
        // the actual members as POD
        //---------------------------

        // flags of entries that are valid.
        Flags                       flags;

        // environmental data
        Env                         env;
        // ambient light
        Light                       light;
        };
    };

class cMeasurementLoopV1 : public cMeasurementLoop
    {
public:
    // some parameters
    static constexpr unsigned kMaxActivityEntries = 8;
    using MeasurementFormat = cMeasurementFormatV1<kMaxActivityEntries>;
    using Measurement = MeasurementFormat::Measurement;
    using FlagsV1 = MeasurementFormat::Flags;
    static constexpr std::uint8_t kMessageFormat = MeasurementFormat::kMessageFormatV1;

    cMeasurementLoopV1() {};

    // neither copyable nor movable
    cMeasurementLoopV1(const cMeasurementLoopV1&) = delete;
    cMeasurementLoopV1& operator=(const cMeasurementLoopV1&) = delete;
    cMeasurementLoopV1(const cMeasurementLoopV1&&) = delete;
    cMeasurementLoopV1& operator=(const cMeasurementLoopV1&&) = delete;

public:
    // things specific to V1, including the measurement flags
    Adafruit_BME280                 m_BME280;
    McciCatena::Catena_Si1133       m_si1133;

    virtual void beginSensors(void) override;
    virtual bool takeMeasurements(void) override;
    virtual bool formatMeasurements(cMeasurementLoop::TxBuffer_t &b, cMeasurementLoop::Measurement const &mData) override;
    virtual bool clearMeasurements(void) override;
    virtual void writeVersionData(File dataFile) override;

    // concrete type for uplink data buffer
    using TxBuffer_t = McciCatena::AbstractTxBuffer_t<MeasurementFormat::kTxBufferSize>;
    using TxBufferBase_t = McciCatena::AbstractTxBufferBase_t;

private:
    // set true if BME280 is present
    bool                            m_fBme280 : 1;
    // set true if SI1133 is present
    bool                            m_fSi1133: 1;

    // the current measurement
    Measurement                     m_data;

    // the data to write to the file
    Measurement                     m_FileData;
    TxBuffer_t                      m_FileTxBuffer;
    };

//
// operator overloads for ORing structured flags
//
static constexpr cMeasurementLoopV1::FlagsV1 operator| (const cMeasurementLoopV1::FlagsV1 lhs, const cMeasurementLoopV1::FlagsV1 rhs)
        {
        return cMeasurementLoopV1::FlagsV1(uint8_t(lhs) | uint8_t(rhs));
        };

static constexpr cMeasurementLoopV1::FlagsV1 operator& (const cMeasurementLoopV1::FlagsV1 lhs, const cMeasurementLoopV1::FlagsV1 rhs)
        {
        return cMeasurementLoopV1::FlagsV1(uint8_t(lhs) & uint8_t(rhs));
        };

static cMeasurementLoopV1::FlagsV1 operator|= (cMeasurementLoopV1::FlagsV1 &lhs, const cMeasurementLoopV1::FlagsV1 &rhs)
        {
        lhs = lhs | rhs;
        return lhs;
        };

} // namespace McciCatena4430

#endif /* _Catena4430_cMeasurementLoopV1_h_ */
