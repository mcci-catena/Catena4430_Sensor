/*

Module: Catena4430_cMeasurementLoop.h

Function:
    cMeasurementLoop definitions.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Terry Moore, MCCI Corporation   August 2019

*/

#ifndef _Catena4430_cMeasurementLoop_h_
# define _Catena4430_cMeasurementLoop_h_

#include <stdint.h>
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Catena_Download.h>
#include <Catena_FSM.h>
#include <Catena_Led.h>
#include <Catena_Log.h>
#include <Catena_Mx25v8035f.h>
#include <Catena4430_cClockDriver_PCF8523.h>
#include <Catena_PollableInterface.h>
#include <Catena_Si1133.h>
#include <Catena_Timer.h>
#include <Catena_TxBuffer.h>
#include <Catena_FlashParam.h>
#include <Catena.h>
#include <MCCI_Catena_SCD30.h>
#include <SD.h>
#include <mcciadk_baselib.h>
#include <stdlib.h>
#include "Catena4430_cPelletFeeder.h"
#include "Catena4430_cPIRdigital.h"
#include <Catena_Date.h>

#include <cstdint>

extern McciCatena::Catena gCatena;
extern McciCatena::cDate gDate;
extern McciCatena::Catena::LoRaWAN gLoRaWAN;
extern McciCatena::StatusLed gLed;
extern McciCatena4430::cClockDriver_PCF8523 gClock;

namespace McciCatena4430 {

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

class cMeasurementBase
    {

    };

template <unsigned a_kMaxActivityEntries>
class cMeasurementFormat : public cMeasurementBase
    {
public:
    static constexpr unsigned kMaxActivityEntries = a_kMaxActivityEntries;
    static constexpr unsigned kMaxPelletEntries = 2;
    static constexpr size_t kTxBufferSize = (1 + 4 + 1 + 2 + 2 + 2 + 1 + 6 + 2 + 6 + kMaxActivityEntries * 2);

    // the structure of a measurement
    struct Measurement
        {
        //----------------
        // the subtypes:
        //----------------

        // measure co2ppm
        struct CO2ppm
            {
            float                   CO2ppm;
            };

        // count of food pellets
        struct Pellets
            {
            // the running total since boot.
            std::uint32_t           Total;
            // count of pellets since last measurement.
            std::uint8_t            Recent;
            };

        // activity: -1 to 1 (for inactive to active)
        struct Activity
            {
            float                   Avg;
            };

        // version number
        struct Version
            {
            std::uint8_t            Major;
            std::uint8_t            Minor;
            std::uint8_t            Patch;
            std::uint8_t            Local;
            };

        //---------------------------
        // the actual members as POD
        //---------------------------

        // time of most recent activity measurement
        McciCatena::cDate           DateTime;

        // count of valid activity measurements
        std::uint8_t                nActivity;

        // measured battery voltage, in volts
        float                       Vbat;
        // measured USB bus voltage, in volts.
        float                       Vbus;
        // boot count
        uint32_t                    BootCount;
        // measure co2ppm
        CO2ppm                      co2ppm;
        // food pellet tracking.
        Pellets                     pellets[kMaxPelletEntries];
        // array of potential activity measurements.
        Activity                    activity[kMaxActivityEntries];
        // version number
        Version                     ver;
        };
    };

class cMeasurementLoop : public McciCatena::cPollableObject
    {
public:
    // version parameters
    static constexpr std::uint8_t kMajor = 2;
    static constexpr std::uint8_t kMinor = 4;
    static constexpr std::uint8_t kPatch = 0;
    static constexpr std::uint8_t kLocal = 0;

    // some parameters
    static constexpr std::uint8_t kUplinkPortDefault = 2;
    static constexpr std::uint8_t kUplinkPortwithNwTime = 3;
    static constexpr std::uint8_t kUplinkPortDataLimit = 4;
    static constexpr bool kEnableDeepSleep = false;
    static constexpr unsigned kMaxActivityEntries = 8;
    using MeasurementFormat = cMeasurementFormat<kMaxActivityEntries>;
    static constexpr unsigned kMaxPelletEntries = MeasurementFormat::kMaxPelletEntries;
    using Measurement = MeasurementFormat::Measurement;
    std::uint8_t kMessageFormat;
    static constexpr std::uint8_t kSdCardCSpin = D5;
    using Flash_t = McciCatena::FlashParamsStm32L0_t;
    using ParamBoard_t = Flash_t::ParamBoard_t;
    static constexpr std::uint32_t kNumSecondsFitfulSleepMax = 10;
    static constexpr std::uint32_t kWatchdogSeconds = 26;
    static_assert(kNumSecondsFitfulSleepMax < kWatchdogSeconds, "Wake up often enough to refresh the IWDG watchdog");

    void deepSleepPrepare();
    void deepSleepRecovery();

    // requests
    enum DlRequest_t : uint8_t
        {
        dlrqReset = 2,
        dlrqGetVersion,
        dlrqResetAppEUI,
        dlrqResetAppKey,
        dlrqRejoin
        };

    // downlink error codes
    enum Error_t : uint8_t
        {
        kSuccess = 0,
        kInvalidLength,
        kNotSuccess
        };

    enum OPERATING_FLAGS : uint32_t
        {
        fUnattended = 1 << 0,
        fManufacturingTest = 1 << 1,
        fConfirmedUplink = 1 << 16,
        fDisableDeepSleep = 1 << 17,
        fQuickLightSleep = 1 << 18,
        fDeepSleepTest = 1 << 19,
        fDisableLed = 1 << 30,
        };

    enum DebugFlags : std::uint32_t
        {
        kError      = 1 << 0,
        kWarning    = 1 << 1,
        kTrace      = 1 << 2,
        kInfo       = 1 << 3,
        };

    // constructor
    cMeasurementLoop(
            )
        : m_pirSampleSec(2)                 // PIR sample timer
        , m_txCycleSec_Permanent(6 * 60)    // default uplink interval
        , m_txCycleSec(60)                  // initial uplink interval
        , m_txCycleCount(10)                // initial count of fast uplinks
        , m_rtcSetSec(8 * 60 * 60)          // set RTC time every 8 hours
        , m_versionSec(4 * 60 * 60)         // send Version every 6 hours
        , m_DebugFlags(DebugFlags(kError | kTrace))
        , m_ActivityTimerSec(60)            // the activity time sample interval
        , m_ActivityDataLimitTimerSec(9 * 60)    // the time sample interval to limit data
        , m_txCycleSec_Low_Power(60 * 60)   // uplink interval when Vbat < 3.2V
        {};

    // neither copyable nor movable
    cMeasurementLoop(const cMeasurementLoop&) = delete;
    cMeasurementLoop& operator=(const cMeasurementLoop&) = delete;
    cMeasurementLoop(const cMeasurementLoop&&) = delete;
    cMeasurementLoop& operator=(const cMeasurementLoop&&) = delete;

    enum class State : std::uint8_t
        {
        stNoChange = 0, // this name must be present: indicates "no change of state"
        stInitial,      // this name must be present: it's the starting state.
        stInactive,     // parked; not doing anything.
        stSleeping,     // active; sleeping between measurements
        stWarmup,       // transition from inactive to measure, get some data.
        stMeasure,      // take measurents
        stTransmit,     // transmit data
        stWriteFile,    // write file data
        stTryToUpdate,  // try to update firmware
        stTryToMigrate, // try to migrate device to TTN V3
        stTryToRejoin,  // try to rejoin to lorawan network
        stAwaitCard,    // wait for a card to show up.
        stRebootForUpdate, // reboot system to complete firmware update

        stFinal,        // this name must be present, it's the terminal state.
        };

    static constexpr const char *getStateName(State s)
        {
        switch (s)
            {
        case State::stNoChange: return "stNoChange";
        case State::stInitial:  return "stInitial";
        case State::stInactive: return "stInactive";
        case State::stSleeping: return "stSleeping";
        case State::stWarmup:   return "stWarmup";
        case State::stMeasure:  return "stMeasure";
        case State::stTransmit: return "stTransmit";
        case State::stWriteFile: return "stWriteFile";
        case State::stTryToUpdate: return "stTryToUpdate";
        case State::stTryToMigrate: return "stTryToMigrate";
        case State::stTryToRejoin: return "stTryToRejoin";
        case State::stAwaitCard: return "stAwaitCard";
        case State::stRebootForUpdate: return "stRebootForUpdate";
        case State::stFinal:    return "stFinal";
        default:                return "<<unknown>>";
            }
        }

    // concrete type for uplink data buffer
    using TxBuffer_t = McciCatena::AbstractTxBuffer_t<MeasurementFormat::kTxBufferSize>;
    using TxBufferBase_t = McciCatena::AbstractTxBufferBase_t;

    // Uplink port
    std::uint8_t uplinkPort;

    // flag to disable LED
    bool fDisableLED;

    // set true if light sensor detects low light
    bool m_fLowLight: 1;

    // set flag if Network time set to RTC
    bool fNwTimeSet;

    // flag to if data being read and ready to transmit
    bool fData_Vbat;
    bool fData_Version;
    bool fData_CO2;
    bool fData_BootCount;
    bool fData_Activity;
    bool fData_Pellet;

    // set start time when network time is being set
    std::uint32_t startTime;

    // SCD30 - CO2 sensor
    McciCatenaScd30::cSCD30 m_Scd{Wire};

    // variables to store scd30 data
    char ts;
    int32_t t100;
    int32_t tint;
    int32_t tfrac;
    int32_t rh100;
    int32_t rhint;
    int32_t rhfrac;
    int32_t co2_100;
    int32_t co2int;
    int32_t co2frac;

    // initializtion of cMeasurement
    cMeasurementLoop *constructInstanceForHardware(bool ver);

    // things that are in any measurement loop variant, and then...
    virtual void beginSensors(void){
        gCatena.SafePrintf("error: beginSensors(): points to Base Class");
        }
    virtual bool takeMeasurements(void){
        gCatena.SafePrintf("error: takeMeasurements(): points to Base Class");
        }
    virtual bool formatMeasurements(TxBuffer_t &b, Measurement const &mData){
        gCatena.SafePrintf("error: formatMeasurements(): points to Base Class");
        }
    virtual bool clearMeasurements(void){
        gCatena.SafePrintf("error: clearMeasurements() points to Base Class");
        }
    virtual void writeVersionData(File dataFile){
        gCatena.SafePrintf("error: writeVersionData(): points to Base Class");
        }

    void setBoardRev(uint8_t boardRev)
        {
        // set the board revision.
        this->m_boardRev = boardRev;
        }
    uint8_t readBoardRev()
        {
        // return the board revision.
        return this->m_boardRev;
        }

    void setBoard(uint16_t board)
        {
        // set the board model.
        this->m_board = board;
        }
    uint16_t readBoard()
        {
        // return the board model.
        return this->m_board;
        }
    const ParamBoard_t * m_pBoard;

    // initialize measurement FSM.
    void begin();
    void end();
    void setTxCycleTime(
        std::uint32_t txCycleSec,
        std::uint32_t txCycleCount
        )
        {
        this->m_txCycleSec = txCycleSec;
        this->m_txCycleCount = txCycleCount;

        this->m_UplinkTimer.setInterval(txCycleSec * 1000);
        if (this->m_UplinkTimer.peekTicks() != 0)
            this->m_fsm.eval();
        }
    std::uint32_t getTxCycleTime()
        {
        return this->m_txCycleSec;
        }
    virtual void poll() override;
    void setVbus(float Vbus)
        {
        // set threshold value as 4.0V as there is reverse voltage
        // in vbus(~3.5V) while powered from battery in 4610. 
        this->m_fUsbPower = (Vbus > 4.0f) ? true : false;
        }

    // fetch and print SCD30 device related information
    void printSCDinfo()
        {
        auto const info = m_Scd.getInfo();
        gCatena.SafePrintf(
                    "Found sensor: firmware version %u.%u\n",
                    info.FirmwareVersion / 256u,
                    info.FirmwareVersion & 0xFFu
                    );
        gCatena.SafePrintf("  Automatic Sensor Calibration: %u\n", info.fASC_status);
        gCatena.SafePrintf("  Sample interval:      %6u secs\n", info.MeasurementInterval);
        gCatena.SafePrintf("  Forced Recalibration: %6u ppm\n", info.ForcedRecalibrationValue);
        gCatena.SafePrintf("  Temperature Offset:   %6d centi-C\n", info.TemperatureOffset);
        gCatena.SafePrintf("  Altitude:             %6d meters\n", info.AltitudeCompensation);
        }

    // request that the measurement loop be active/inactive
    void requestActive(bool fEnable);

    // return true if a given debug mask is enabled.
    bool isTraceEnabled(DebugFlags mask) const
        {
        return this->m_DebugFlags & mask;
        }

    // register an additional SPI for sleep/resume
    // can be called before begin().
    void registerSecondSpi(SPIClass *pSpi)
        {
        this->m_pSPI2 = pSpi;
        }

    /// bring up the SD card, if possible.
    bool checkSdCard();
    /// tear down the SD card.
    void sdFinish();

    /// @brief setup STM32Lxx IWDG to require 20 second updates.
    void setupWatchdog();
    /// @brief get another 20 seconds before the watchdog triggers
    void refreshWatchdog();

    /// @brief delay for a while, and refresh the watchdog.
    /// @param millis number of milliseconds, as for \c ::delay().
    void safeDelay(uint32_t millis);

    // timeout handling

    // set the timer
    void setTimer(std::uint32_t ms);
    // clear the timer
    void clearTimer();
    // test (and clear) the timed-out flag.
    bool timedOut();

private:
    // sleep handling
    void sleep();
    bool checkDeepSleep();
    void doSleepAlert(bool fDeepSleep);
    void doDeepSleep();
    void fitfulSleep(uint32_t nSeconds);

    // read data
    void updateSynchronousMeasurements();
    void updateScd30Measurements();
    void resetMeasurements();
    void measureActivity();

    // downlink requests
    void doDlrqCalibCO2(const uint8_t *pMessage, size_t nMessage);
    void doDlrqResetAppEUI(const uint8_t *pMessage, size_t nMessage);
    void doDlrqResetAppKey(const uint8_t *pMessage, size_t nMessage);
    void doDlrqResetMode(const uint8_t *pMessage, size_t nMessage);
    void doDlrqRejoin(const uint8_t *pMessage);
    void doDlrqGetVersion(const uint8_t *pMessage);
    void sendDownlinkAck(void);
    void receiveMessageDone(uint8_t port, const uint8_t *pMessage, size_t nMessage);

    // Downlink pointer
    static Arduino_LoRaWAN::ReceivePortBufferCbFn receiveMessage;

    // telemetry handling.
    // void fillTxBuffer(TxBuffer_t &b, Measurement const & mData);
    void startTransmission(TxBuffer_t &b);
    void sendBufferDone(bool fSuccess);
    bool txComplete()
        {
        return this->m_txcomplete;
        }
    static std::uint16_t activity2uf(float v)
        {
        return McciCatena::TxBuffer_t::f2uflt16(v);
        }
    void updateTxCycleTime();

    // SD card handling
    bool initSdCard();

    bool writeSdCard(TxBuffer_t &b, Measurement const &mData);
    bool handleSdFirmwareUpdate();
    bool handleSdFirmwareUpdateCardUp();
    bool updateFromSd(const char *sFile, McciCatena::cDownload::DownloadRq_t rq);
    void handleSdTTNv3Migrate();
    void handleSdNetworkRejoin();
    void rejoinNetwork();
    void sdPowerUp(bool fOn);
    void sdPrep();

    // pir handling
    void resetPirAccumulation(void);
    void accumulatePirData(void);

    // instance data
private:
    McciCatena::cFSM<cMeasurementLoop, State> m_fsm;
    // evaluate the control FSM.
    State fsmDispatch(State currentState, bool fEntry);

    // second SPI class
    SPIClass                        *m_pSPI2;

    // debug flags
    DebugFlags                      m_DebugFlags;

    // true if object is registered for polling.
    bool                            m_registered : 1;
    // true if object is running.
    bool                            m_running : 1;
    // true to request exit
    bool                            m_exit : 1;
    // true if in active uplink mode, false otehrwise.
    bool                            m_active : 1;

    // set true to request transition to active uplink mode; cleared by FSM
    bool                            m_rqActive : 1;
    // set true to request transition to inactive uplink mode; cleared by FSM
    bool                            m_rqInactive : 1;

    // set true if CO2 (SCD) is present
    bool                            m_fScd30 : 1;
    // set true if device enters Sleep state
    bool                            m_fSleepScd30 : 1;
    // set true if measurement is valid
    bool                            m_measurement_valid: 1;

    // set true if event timer times out
    bool                            m_fTimerEvent : 1;
    // set true while evenet timer is active.
    bool                            m_fTimerActive : 1;
    // set true if USB power is present.
    bool                            m_fUsbPower : 1;

    // set true while a transmit is pending.
    bool                            m_txpending : 1;
    // set true when a transmit completes.
    bool                            m_txcomplete : 1;
    // set true when a transmit complete with an error.
    bool                            m_txerr : 1;
    // set true when we've printed how we plan to sleep
    bool                            m_fPrintedSleeping : 1;
    // set true when SPI2 is active
    bool                            m_fSpi2Active: 1;
    // set true when we've BIN file in SD card to update
    bool                            m_fFwUpdate : 1;
    // set true when Vbat is less than the minimum threshold (3.3V)
    bool                            m_fDatalimit: 1;
    // set true when Version is sent Over the Air
    bool                            m_fVersionOta: 1;
    // set true if Downlink Acknowledgement is present
    bool                            m_fRxAck : 1;

    // catena board flash parameters
    std::uint8_t                    m_boardRev;
    std::uint16_t                   m_board;

    // PIR sample control
    cPIRdigital                     m_pir;
    McciCatena::cTimer              m_pirSampleTimer;
    float                           m_pirMin;
    float                           m_pirMax;
    float                           m_pirSum;
    std::uint32_t                   m_pirBaseTimeMs;
    std::uint32_t                   m_pirLastTimeMs;
    std::uint32_t                   m_pirSampleSec;

    // Pellet Feeder
    cPelletFeeder                   m_PelletFeeder;

    // activity time control
    McciCatena::cTimer              m_ActivityTimer;
    std::uint32_t                   m_ActivityTimerSec;
    std::uint32_t                   m_ActivityDataLimitTimerSec;

    // uplink time control
    McciCatena::cTimer              m_UplinkTimer;
    std::uint32_t                   m_txCycleSec;
    std::uint32_t                   m_txCycleCount;
    std::uint32_t                   m_txCycleSec_Permanent;
    std::uint32_t                   m_txCycleSec_Low_Power;

    // RTC set time control
    std::uint32_t                   m_rtcSetSec;

    // Version OTA - time control
    std::uint32_t                   m_versionSec;
    std::uint32_t                   m_startTimeMs;
    std::uint32_t                   m_currentIntervalSec;

    // simple timer for timing-out sensors.
    std::uint32_t                   m_timer_start;
    std::uint32_t                   m_timer_delay;

    // the current measurement
    Measurement                     m_data;

    // the data to write to the file
    Measurement                     m_FileData;
    TxBuffer_t                      m_FileTxBuffer;
    TxBuffer_t                      m_AckTxBuffer;
    };

} // namespace McciCatena4430

#endif /* _Catena4430_cMeasurementLoop_h_ */

