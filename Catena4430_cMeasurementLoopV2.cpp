/*

Module: Catena4430_cMeasurementLoopV2.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   October 2022

*/

#include "Catena4430_cMeasurementLoopV2.h"
#include <Catena4430.h>
#include <arduino_lmic.h>
#include <Catena4430_Sensor.h>

using namespace McciCatena4430;
using namespace McciCatena;

using namespace McciCatenaSht3x;
using namespace Mcci_Ltr_329als;

extern cMeasurementLoop *gpMeasurementLoopConcrete;

void cMeasurementLoopV2::beginSensors(void)
    {
    if (this->m_Sht.begin())
        {
        this->m_fSht3x = true;
        gCatena.SafePrintf("On-board Temperature Sensor detected\n");
        }
    else
        {
        this->m_fSht3x = false;
        gCatena.SafePrintf("No SHT3x found: check wiring\n");
        }

    if (this->m_Ltr.begin())
        {
        this->m_fHardError = false;
        this->m_fLtr329 = true;
        gCatena.SafePrintf("On-board Light Sensor detected\n");
        }
    else
        {
        this->m_fLtr329 = false;
        gCatena.SafePrintf("No LTR329 found: check hardware\n");
        }
    }

bool cMeasurementLoopV2::takeMeasurements(void)
    {
    if (this->m_fSht3x)
        {
        cSHT3x::Measurements m;
        this->m_Sht.getTemperatureHumidity(m);
        this->m_data.env.Temperature = m.Temperature;
        this->m_data.env.Humidity = m.Humidity;
        this->m_data.flags |= FlagsV2::TH;
        }

    gpMeasurementLoopConcrete->setTimer(1000);

    if (this->m_fLtr329)
        {
        if (this->m_Ltr.startSingleMeasurement())
            {
            float currentLux;
            bool fHardError;

            static constexpr float kMax_Gain_96 = 640.0f;
            static constexpr float kMax_Gain_48 = 1280.0f;
            static constexpr float kMax_Gain_8 = 7936.0f;
            static constexpr float kMax_Gain_4 = 16128.0f;
            static constexpr float kMax_Gain_2 = 32512.0f;
            static constexpr float kMax_Gain_1 = 65535.0f;

            while (! this->m_Ltr.queryReady(fHardError))
                {
                if (fHardError)
                    break;
                }

            if (fHardError)
                {
                this->m_fHardError = true;
                if (gLog.isEnabled(gLog.DebugFlags::kError))
                    {
                    gLog.printf(
                        gLog.kAlways,
                        "LTR329 queryReady failed: status %s(%u)\n",
                        this->m_Ltr.getLastErrorName(),
                        unsigned(this->m_Ltr.getLastError())
                        );

                    this->m_Ltr.begin();
                    }
                }
            else
                {
                currentLux = this->m_Ltr.getLux();

                if (currentLux <= kMax_Gain_96)
                    this->m_Ltr.setGain(96);
                else if (currentLux <= kMax_Gain_48)
                    this->m_Ltr.setGain(48);
                else if (currentLux <= kMax_Gain_8)
                    this->m_Ltr.setGain(8);
                else if (currentLux <= kMax_Gain_4)
                    this->m_Ltr.setGain(4);
                else if (currentLux <= kMax_Gain_2)
                    this->m_Ltr.setGain(2);
                else
                    this->m_Ltr.setGain(1);

                currentLux = this->m_Ltr.getLux();
                this->m_data.flags |= FlagsV2::Light;
                this->m_data.light.Lux = currentLux;

                if (currentLux <= 100)
                    gpMeasurementLoopConcrete->m_fLowLight = true;
                else
                    gpMeasurementLoopConcrete->m_fLowLight = false;
                }
            }
        else if (gpMeasurementLoopConcrete->timedOut())
            {
            this->m_fHardError = true;
            if (gpMeasurementLoopConcrete->isTraceEnabled(gpMeasurementLoopConcrete->DebugFlags::kError))
                gCatena.SafePrintf("LTR329 timed out\n");
            
            return false;
            }
        else
            this->m_fHardError = true;
        }
    return true;
    }

bool cMeasurementLoopV2::clearMeasurements(void)
    {
    memset((void *) &this->m_data, 0, sizeof(this->m_data));
    this->m_data.flags = FlagsV2(0);
    }

bool cMeasurementLoopV2::formatMeasurements(
    cMeasurementLoop::TxBuffer_t& b,
    cMeasurementLoop::Measurement const &mData
    )
    {
    auto const savedLed = gLed.Set(McciCatena::LedPattern::Off);
    if (!(this->fDisableLED && this->m_fLowLight))
        {
        gLed.Set(McciCatena::LedPattern::Measuring);
        }

    if (this->fData_Vbat)
        this->m_data.flags |= FlagsV2::Vbat;

    if (this->fData_Version)
        this->m_data.flags |= FlagsV2::Version;

    if (this->fData_CO2)
        this->m_data.flags |= FlagsV2::CO2ppm;

    if (this->fData_BootCount)
        this->m_data.flags |= FlagsV2::Boot;

    if (this->fData_Activity)
        this->m_data.flags |= FlagsV2::Activity;

    if (this->fData_Pellet)
        this->m_data.flags |= FlagsV2::Pellets;

    // initialize the message buffer to an empty state
    b.begin();

    // insert format byte
    b.put(this->kMessageFormat);

    // insert the timestamp from the data
    // stuff zero if time is not valid.
    b.put4u(std::uint32_t(mData.DateTime.getGpsTime()));
    
    // the flags in Measurement correspond to the over-the-air flags.
    b.put(std::uint8_t(this->m_data.flags));

    // send Vbat
    if ((this->m_data.flags & FlagsV2::Vbat) != FlagsV2(0))
        {
        float Vbat = mData.Vbat;
        gCatena.SafePrintf("Vbat:    %d mV\n", (int) (Vbat * 1000.0f));
        b.putV(Vbat);
        }

    // send firmware version
    if ((this->m_data.flags & FlagsV2::Version) != FlagsV2(0))
        {
        b.put(mData.ver.Major);
        b.put(mData.ver.Minor);
        b.put(mData.ver.Patch);
        b.put(mData.ver.Local);
        }

    // Vbus is shown as 5000 * v
    float Vbus = mData.Vbus;
    gCatena.SafePrintf("Vbus:    %d mV\n", (int) (Vbus * 1000.0f));

    // put co2ppm
    if ((this->m_data.flags & FlagsV2::CO2ppm) != FlagsV2(0))
        {
        gCatena.SafePrintf(
            "SCD30:  T(C): %c%d.%02d  RH(%%): %d.%02d  CO2(ppm): %d.%02d\n",
            this->ts, this->tint, this->tfrac,
            this->rhint, this->rhfrac,
            this->co2int, this->co2frac
            );

        b.put2u(TxBufferBase_t::f2uflt16(mData.co2ppm.CO2ppm / 40000.0f));
        }

    // send boot count
    if ((this->m_data.flags & FlagsV2::Boot) != FlagsV2(0))
        {
        b.putBootCountLsb(mData.BootCount);
        }

    if ((this->m_data.flags & FlagsV2::TH) != FlagsV2(0))
        {
        gCatena.SafePrintf(
                "SHT3x:  T: %d RH: %d\n",
                (int) this->m_data.env.Temperature,
                (int) this->m_data.env.Humidity
                );
        b.putT(this->m_data.env.Temperature);
        // no method for 2-byte RH, directly encode it.
        b.put2uf((this->m_data.env.Humidity / 100.0f) * 65535.0f);
        }

    // put light
    if ((this->m_data.flags & FlagsV2::Light) != FlagsV2(0))
        {
        gCatena.SafePrintf(
                "Ltr329:  %d Lux\n",
                (int) this->m_data.light.Lux
                );

        b.put3f(this->m_data.light.Lux);
        }

    // put pellets
    if ((this->m_data.flags & FlagsV2::Pellets) != FlagsV2(0))
        {
        for (unsigned i = 0; i < cMeasurementLoop::kMaxPelletEntries; ++i)
            {
            b.put2(mData.pellets[i].Total & 0xFFFFu);
            b.put(mData.pellets[i].Recent);
            }
        }

    // put activity
    if ((this->m_data.flags & FlagsV2::Activity) != FlagsV2(0))
        {
        for (unsigned i = 0; i < mData.nActivity; ++i)
            {
            // scale to 0..1
            float aAvg = mData.activity[i].Avg;

            gCatena.SafePrintf(
                "Activity[%u] [0..1000):  %d Avg\n",
                i,
                500 + int(500 * aAvg)
                );

            b.put2uf(TxBufferBase_t::f2sflt16(aAvg));
            }
        }

    if (!(this->fDisableLED && this->m_fLowLight))
        gLed.Set(savedLed);
    }

void cMeasurementLoopV2::writeVersionData(File dataFile)
    {
    if ((m_data.flags & FlagsV2::TH) != FlagsV2(0))
        {
        dataFile.print(m_data.env.Temperature);
        dataFile.print(',');

        dataFile.print(m_data.env.Humidity);
        dataFile.print(',');

        dataFile.print(',');
        }
    else
        {
        dataFile.print(",,,");
        }

    if ((m_data.flags & FlagsV2::Light) != FlagsV2(0))
        {
        dataFile.print(m_data.light.Lux);
        }
    else
        {
        dataFile.print(",");
        }
    }
