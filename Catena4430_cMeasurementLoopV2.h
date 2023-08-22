/*

Module: Catena4430_cMeasurementLoopV2.h

Function:
    cMeasurementLoopV2 definitions.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   October 2022

*/

#ifndef _Catena4430_cMeasurementLoopV2_h_
# define _Catena4430_cMeasurementLoopV2_h_

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Catena_Mx25v8035f.h>
#include <Catena.h>
#include "Catena4430_cMeasurementLoop.h"
#include <mcciadk_baselib.h>
#include <stdlib.h>
#include <Catena-SHT3x.h>
#include <mcci_ltr_329als.h>

#include <cstdint>

extern McciCatena::Catena gCatena;

namespace McciCatena4430 {

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

template <unsigned a_kMaxActivityEntries>
class cMeasurementFormatV2 : public cMeasurementBase
    {
public:
    static constexpr uint8_t kMessageFormatV2 = 0x36;

    enum class Flags : uint8_t
            {
            Vbat = 1 << 0,      // vBat
            Version = 1 << 1,   // Sketch version
            CO2ppm = 1 << 2,    // Carbondioxide (SCD30)
            Boot = 1 << 3,      // boot count
            TH = 1 << 4,        // temperature, humidity
            Light = 1 << 5,     // light (lux))
            Pellets = 1 << 6,   // Pellet count
            Activity = 1 << 7,  // Activity (min/max/avg)
            };

    static constexpr unsigned kMaxActivityEntries = a_kMaxActivityEntries;
    static constexpr unsigned kMaxPelletEntries = 2;
    static constexpr size_t kTxBufferSize = (1 + 4 + 1 + 2 + 4 + 2 + 1 + 4 + 3 + 6 + kMaxActivityEntries * 2);

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
            // humidity (in % RH)
            float                   Humidity;
            };

        // ambient light measurements
        struct Light
            {
            // light intensity
            float                   Lux;
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

class cMeasurementLoopV2 : public cMeasurementLoop
    {
public:
    // some parameters
    static constexpr unsigned kMaxActivityEntries = 8;
    using MeasurementFormat = cMeasurementFormatV2<kMaxActivityEntries>;
    using Measurement = MeasurementFormat::Measurement;
    using FlagsV2 = MeasurementFormat::Flags;
    static constexpr std::uint8_t kMessageFormat = MeasurementFormat::kMessageFormatV2;

    // things specific to V2, including the measurement flags

    // constructor
    cMeasurementLoopV2() {};

    // neither copyable nor movable
    cMeasurementLoopV2(const cMeasurementLoopV2&) = delete;
    cMeasurementLoopV2& operator=(const cMeasurementLoopV2&) = delete;
    cMeasurementLoopV2(const cMeasurementLoopV2&&) = delete;
    cMeasurementLoopV2& operator=(const cMeasurementLoopV2&&) = delete;

    virtual void beginSensors(void) override;
    virtual bool takeMeasurements(void) override;
    virtual bool formatMeasurements(cMeasurementLoop::TxBuffer_t& b, cMeasurementLoop::Measurement const &mData) override;
    virtual bool clearMeasurements(void) override;
    virtual void writeVersionData(File dataFile) override;

    // concrete type for uplink data buffer
    using TxBuffer_t = McciCatena::AbstractTxBuffer_t<MeasurementFormat::kTxBufferSize>;
    using TxBufferBase_t = McciCatena::AbstractTxBufferBase_t;

private:
    McciCatenaSht3x::cSHT3x        m_Sht{Wire};
    Mcci_Ltr_329als::Ltr_329als    m_Ltr{Wire};
    Mcci_Ltr_329als_Regs::AlsContr_t    m_AlsCtrl;

    // set true if SHT3x is present
    bool                            m_fSht3x : 1;
    // set true if LTR329 is present
    bool                            m_fLtr329: 1;
    // set true if hardware error in LTR329
    bool                            m_fHardError: 1;

    // the current measurement
    Measurement                     m_data;

    // the data to write to the file
    Measurement                     m_FileData;
    TxBuffer_t                      m_FileTxBuffer;
    };

//
// operator overloads for ORing structured flags
//
static constexpr cMeasurementLoopV2::FlagsV2 operator| (const cMeasurementLoopV2::FlagsV2 lhs, const cMeasurementLoopV2::FlagsV2 rhs)
        {
        return cMeasurementLoopV2::FlagsV2(uint8_t(lhs) | uint8_t(rhs));
        };

static constexpr cMeasurementLoopV2::FlagsV2 operator& (const cMeasurementLoopV2::FlagsV2 lhs, const cMeasurementLoopV2::FlagsV2 rhs)
        {
        return cMeasurementLoopV2::FlagsV2(uint8_t(lhs) & uint8_t(rhs));
        };

static cMeasurementLoopV2::FlagsV2 operator|= (cMeasurementLoopV2::FlagsV2 &lhs, const cMeasurementLoopV2::FlagsV2 &rhs)
        {
        lhs = lhs | rhs;
        return lhs;
        };

} // namespace McciCatena4430

#endif /* _Catena4430_cMeasurementLoopV2_h_ */
