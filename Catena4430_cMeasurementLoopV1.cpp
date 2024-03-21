/*

Module: Catena4430_cMeasurementLoopV1.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   October 2022

*/

#include "Catena4430_cMeasurementLoopV1.h"
#include <Catena4430.h>
#include <arduino_lmic.h>
#include <Catena4430_Sensor.h>

using namespace McciCatena4430;
using namespace McciCatena;

extern cMeasurementLoop *gpMeasurementLoopConcrete;

void cMeasurementLoopV1::beginSensors(void)
    {
    if (this->m_BME280.begin(BME280_ADDRESS, Adafruit_BME280::OPERATING_MODE::Sleep))
        {
        this->m_fBme280 = true;
        gCatena.SafePrintf("On-board Temperature Sensor found\n");
        }
    else
        {
        this->m_fBme280 = false;
        gCatena.SafePrintf("No BME280 found: check wiring\n");
        }

    if (this->m_si1133.begin())
        {
        this->m_fSi1133 = true;
        gpMeasurementLoopConcrete->m_fLowLight = true;

        auto const measConfig =	Catena_Si1133::ChannelConfiguration_t()
            .setAdcMux(Catena_Si1133::InputLed_t::LargeWhite)
            .setSwGainCode(7)
            .setHwGainCode(4)
            .setPostShift(1)
            .set24bit(true);

        this->m_si1133.configure(0, measConfig, 0);
        gCatena.SafePrintf("On-board Light Sensor found\n");
        }
    else
        {
        this->m_fSi1133 = false;
        gCatena.SafePrintf("No Si1133 found: check hardware\n");
        }
    }

bool cMeasurementLoopV1::takeMeasurements(void)
    {
    // start SI1133 measurement (one-time)
    if (this->m_fSi1133)
        this->m_si1133.start(true);

    if (this->m_fBme280)
        {
        auto m = this->m_BME280.readTemperaturePressureHumidity();
        this->m_data.env.Temperature = m.Temperature;
        this->m_data.env.Pressure = m.Pressure;
        this->m_data.env.Humidity = m.Humidity;
        this->m_data.flags |= FlagsV1::TPH;
        }

    gpMeasurementLoopConcrete->setTimer(1000);
    bool fSi1133Data = true;

    while (fSi1133Data)
        {
        if (this->m_si1133.isOneTimeReady())
            {
            fSi1133Data = false;
            uint32_t data[1];

            this->m_si1133.readMultiChannelData(data, 1);
            this->m_si1133.stop();

            this->m_data.flags |= FlagsV1::Light;
            this->m_data.light.White = (float) data[0];

            if (data[0] <= 500)
                gpMeasurementLoopConcrete->m_fLowLight = true;
            else
                gpMeasurementLoopConcrete->m_fLowLight = false;
            }
        else if (gpMeasurementLoopConcrete->timedOut())
            {
            fSi1133Data = false;
            this->m_si1133.stop();
            if (gpMeasurementLoopConcrete->isTraceEnabled(gpMeasurementLoopConcrete->DebugFlags::kError))
                gCatena.SafePrintf("S1133 timed out\n");
            }
        else
            fSi1133Data = true;
        }

    return true;
    }

bool cMeasurementLoopV1::clearMeasurements(void)
    {
    memset((void *) &this->m_data, 0, sizeof(this->m_data));
    this->m_data.flags = FlagsV1(0);
    }

bool cMeasurementLoopV1::formatMeasurements(
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
        this->m_data.flags |= FlagsV1::Vbat;

    if (this->fData_Version)
        this->m_data.flags |= FlagsV1::Version;

    if (this->fData_CO2)
        this->m_data.flags |= FlagsV1::CO2ppm;

    if (this->fData_BootCount)
        this->m_data.flags |= FlagsV1::Boot;

    if (this->fData_Activity)
        this->m_data.flags |= FlagsV1::Activity;

    if (this->fData_Pellet)
        this->m_data.flags |= FlagsV1::Pellets;
    
    // initialize the message buffer to an empty state
    b.begin();

    // insert format byte
    b.put(this->kMessageFormat);

    // insert the timestamp from the data
    // stuff zero if time is not valid.
    b.put4u(std::uint32_t(mData.DateTime.getGpsTime()));

    // the flags in Measurement correspond to the over-the-air flags.
    b.put(std::uint8_t(m_data.flags));

    // send Vbat
    if ((m_data.flags & FlagsV1::Vbat) != FlagsV1(0))
        {
        float Vbat = mData.Vbat;
        gCatena.SafePrintf("Vbat:    %d mV\n", (int) (Vbat * 1000.0f));
        b.putV(Vbat);
        }

    // send firmware version
    if ((m_data.flags & FlagsV1::Version) != FlagsV1(0))
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
    if ((m_data.flags & FlagsV1::CO2ppm) != FlagsV1(0))
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
    if ((m_data.flags & FlagsV1::Boot) != FlagsV1(0))
        {
        b.putBootCountLsb(mData.BootCount);
        }

    if ((m_data.flags & FlagsV1::TPH) != FlagsV1(0))
        {
        gCatena.SafePrintf(
                "BME280:  T: %d P: %d RH: %d\n",
                (int) m_data.env.Temperature,
                (int) m_data.env.Pressure,
                (int) m_data.env.Humidity
                );
        b.putT(m_data.env.Temperature);
        b.putP(m_data.env.Pressure);
        // no method for 2-byte RH, directly encode it.
        b.put2uf((m_data.env.Humidity / 100.0f) * 65535.0f);
        }

    // put light
    if ((m_data.flags & FlagsV1::Light) != FlagsV1(0))
        {
        gCatena.SafePrintf(
                "Si1133:  %d White\n",
                (int) m_data.light.White
                );

        b.putLux(TxBufferBase_t::f2uflt16(m_data.light.White / pow(2.0, 24)));
        }

    // put pellets
    if ((m_data.flags & FlagsV1::Pellets) != FlagsV1(0))
        {
        for (unsigned i = 0; i < cMeasurementLoop::kMaxPelletEntries; ++i)
            {
            b.put2(mData.pellets[i].Total & 0xFFFFu);
            b.put(mData.pellets[i].Recent);
            }
        }

    // put activity
    if ((m_data.flags & FlagsV1::Activity) != FlagsV1(0))
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

void cMeasurementLoopV1::writeVersionData(File dataFile)
    {
    if ((m_data.flags & FlagsV1::TPH) != FlagsV1(0))
        {
        dataFile.print(m_data.env.Temperature);
        dataFile.print(',');

        dataFile.print(m_data.env.Humidity);
        dataFile.print(',');

        dataFile.print(m_data.env.Pressure);
        dataFile.print(',');
        }
    else
        {
        dataFile.print(",,,");
        }

    if ((m_data.flags & FlagsV1::Light) != FlagsV1(0))
        {
        dataFile.print(m_data.light.White);
        dataFile.print(',');
        }
    else
        {
        dataFile.print(",");
        }
    }
