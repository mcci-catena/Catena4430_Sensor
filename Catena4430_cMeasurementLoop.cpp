/*

Module: Catena4430_cMeasurementLoop.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Terry Moore, MCCI Corporation   August 2019

*/

#include "Catena4430_cMeasurementLoop.h"
#include <TimeLib.h>
#include <Catena4430.h>
#include <arduino_lmic.h>
#include <Catena4430_Sensor.h>

using namespace McciCatena;
using namespace McciCatena4430;

extern c4430Gpios gpio;
extern cMeasurementLoop *gpMeasurementLoopConcrete;
extern Catena::LoRaWAN gLoRaWAN;

static constexpr uint8_t kVddPin = D11;

void lptimSleep(uint32_t timeOut);
uint32_t HAL_AddTick(uint32_t delta);

void user_request_network_time_cb(void *pVoidUserUTCTime, int flagSuccess);

uint32_t timeOut = 200;

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

void cMeasurementLoop::begin()
    {
    // register for polling.
    if (! this->m_registered)
        {
        this->m_registered = true;

        gCatena.registerObject(this);

        this->m_UplinkTimer.begin(this->m_txCycleSec * 1000);
        this->m_pirSampleTimer.begin(this->m_pirSampleSec * 1000);
        this->m_ActivityTimer.begin(this->m_ActivityTimerSec * 1000);
        gLoRaWAN.SetReceiveBufferBufferCb(receiveMessage);
        }

    // start and initialize the PIR sensor
    this->m_pir.begin(gCatena);

    // start and initialize pellet feeder monitoring.
    this->m_PelletFeeder.begin(gCatena);

    Wire.begin();
    gpMeasurementLoopConcrete->beginSensors();

    if (! m_Scd.begin())
        {
        this->m_fScd30 = false;

        gCatena.SafePrintf("No SCD30 found! Begin failed: %s(%u)\n",
                    m_Scd.getLastErrorName(),
                    unsigned(m_Scd.getLastError())
                    );
        }
    else
        {
        this->m_fScd30 = true;
        this->m_fSleepScd30 = false;
        this->printSCDinfo();
        }

    // read network time and set correct UTC time in RTC
    uint32_t userUTCTime; // Seconds since the UTC epoch
    this->fNwTimeSet = false;

    // Schedule a network time request at the next possible time
    LMIC_requestNetworkTime(user_request_network_time_cb, &userUTCTime);

    // timer and flag for version over air
    this->m_startTimeMs = millis();
    this->m_fVersionOta = true;

    // clear flag for Data limit
    this->m_fDatalimit = false;

    // start (or restart) the FSM.
    if (! this->m_running)
        {
        this->m_fFwUpdate = false;
        this->fData_Vbat = false;
        this->fData_Version = false;
        this->fData_CO2 = false;
        this->fData_BootCount = false;
        this->fData_Activity = false;
        this->fData_Pellet = false;
        this->startTime = millis();
        this->m_exit = false;
        this->m_fsm.init(*this, &cMeasurementLoop::fsmDispatch);
        }
    }

void cMeasurementLoop::end()
    {
    if (this->m_running)
        {
        this->m_exit = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::requestActive(bool fEnable)
    {
    if (fEnable)
        this->m_rqActive = true;
    else
        this->m_rqInactive = true;

    this->m_fsm.eval();
    }

cMeasurementLoop::State
cMeasurementLoop::fsmDispatch(
    cMeasurementLoop::State currentState,
    bool fEntry
    )
    {
    this->refreshWatchdog();
    State newState = State::stNoChange;

    if (fEntry && this->isTraceEnabled(this->DebugFlags::kTrace))
        {
        gCatena.SafePrintf("cMeasurementLoop::fsmDispatch: enter %s\n",
                this->getStateName(currentState)
                );
        }

    switch (currentState)
        {
    case State::stInitial:
        newState = State::stInactive;
        this->resetMeasurements();
        break;

    case State::stInactive:
        if (fEntry)
            {
            // turn off anything that should be off while idling.
            }
        if (this->m_rqActive)
            {
            // when going active manually, start the measurement
            // cycle immediately.
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = true;
            this->m_UplinkTimer.retrigger();
            newState = State::stWarmup;
            }
        break;

    case State::stSleeping:
        if (fEntry)
            {
            // reset the counters.
            this->resetPirAccumulation();
            this->fData_Vbat = false;
            this->fData_Version = false;
            this->fData_CO2 = false;
            this->fData_BootCount = false;
            this->fData_Activity = false;
            this->fData_Pellet = false;

            if (!(this->fDisableLED && this->m_fLowLight))
                {
                // set the LEDs to flash accordingly.
                gLed.Set(McciCatena::LedPattern::Sleeping);
                }
            }

        if (this->m_rqInactive)
            {
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = false;
            newState = State::stInactive;
            }
        else if (this->m_UplinkTimer.isready())
            newState = State::stMeasure;
        else if (this->m_UplinkTimer.getRemaining() > 1500)
            {
            this->m_fSleepScd30 = true;
            this->sleep();
            }
        break;

    // get some data. This is only called while booting up.
    case State::stWarmup:
        if (fEntry)
            {
            // reset the counters.
            this->resetPirAccumulation();
            //start the timer
            this->setTimer(5 * 1000);
            }
        if (this->timedOut())
            newState = State::stMeasure;
        break;

    // fill in the measurement
    case State::stMeasure:
        if (fEntry)
            {
            }

        this->updateSynchronousMeasurements();
        newState = State::stTransmit;

        break;

    case State::stTransmit:
        if (fEntry)
            {
            TxBuffer_t b;

            gpMeasurementLoopConcrete->formatMeasurements(b, this->m_data);
            this->m_FileData = this->m_data;

            this->m_FileTxBuffer.begin();
            for (auto i = 0; i < b.getn(); ++i)
                this->m_FileTxBuffer.put(b.getbase()[i]);

            if (gLoRaWAN.IsProvisioned())
                this->startTransmission(b);

            while (true)
                {
                std::uint32_t lmicCheckTime;
                this->refreshWatchdog();
                os_runloop_once();
                lmicCheckTime = this->m_UplinkTimer.getRemaining();

                // if we can sleep, break out of this loop
                // NOTE: if that the TX is not ready, LMIC is still waiting for interrupt
                if (! os_queryTimeCriticalJobs(ms2osticks(lmicCheckTime)) && LMIC_queryTxReady())
                    {
                    break;
                    }

                gCatena.poll();
                yield();
                }
            if (this->m_fRxAck)
                {
                this->sendDownlinkAck();
                this->m_fRxAck = false;
                }
            }
        if (! gLoRaWAN.IsProvisioned())
            {
            newState = State::stWriteFile;
            }
        if (this->txComplete())
            {
            newState = State::stWriteFile;

            // calculate the new sleep interval.
            this->updateTxCycleTime();

            uint32_t currentTimeSec;
            currentTimeSec = uint32_t(millis() - this->startTime) / 1000;
            if (currentTimeSec > m_rtcSetSec)
                {
                uint32_t userUTCTime; // Seconds since the UTC epoch
                // Schedule a network time request at the next possible time
                LMIC_requestNetworkTime(user_request_network_time_cb, &userUTCTime);
                }
            }
        break;

    // if there's an SD card, append to file
    case State::stWriteFile:
        if (fEntry)
            {
            }

        if (this->writeSdCard(this->m_FileTxBuffer, this->m_FileData))
            newState = State::stTryToUpdate;
        else if (gLoRaWAN.IsProvisioned())
            newState = State::stTryToUpdate;
        else
            newState = State::stAwaitCard;

        this->resetMeasurements();
        break;

    // try to update firmware
    case State::stTryToUpdate:
        if (this->handleSdFirmwareUpdate())
            newState = State::stRebootForUpdate;
        else
            newState = State::stTryToMigrate;
        this->m_fFwUpdate = false;
        gpio.setRed(false);
        gpio.setGreen(false);
        gpio.setBlue(false);
        break;

    // try to migrate to TTN V3
    case State::stTryToMigrate:
        if (fEntry)
            {
            this->handleSdTTNv3Migrate();
            }
        newState = State::stTryToRejoin;
        break;

    // try to rejoin to network
    case State::stTryToRejoin:
        if (fEntry)
            {
            this->handleSdNetworkRejoin();
            }
        newState = State::stSleeping;
        break;

    // no SD card....
    case State::stAwaitCard:
        if (fEntry)
            {
            if (! this->fDisableLED)
                {
                uint8_t nBlink = 0;
                while (nBlink < 5)
                    {
                    this->refreshWatchdog();
                    gpio.setBlue(true);
                    safeDelay(100);
                    gpio.setBlue(false);
                    safeDelay(100);
                    gpio.setBlue(true);
                    safeDelay(100);
                    gpio.setBlue(false);
                    safeDelay(500);
                    nBlink += 1;
                    }
                }
            gCatena.SafePrintf("** lorawan not provisioned!\n");
            }

        newState = State::stSleeping;
        break;

    // reboot for update
    case State::stRebootForUpdate:
        if (fEntry)
            {
            gLog.printf(gLog.kInfo, "Rebooting to apply firmware\n");
            this->setTimer(1 * 1000);
            }
        if (this->timedOut())
            {
            NVIC_SystemReset();
            }
        break;

    case State::stFinal:
        break;

    default:
        break;
        }

    return newState;
    }

/****************************************************************************\
|
|   Take a measurement
|
\****************************************************************************/

void cMeasurementLoop::resetMeasurements()
    {
    memset((void *) &this->m_data, 0, sizeof(this->m_data));
    gpMeasurementLoopConcrete->clearMeasurements();
    }

void cMeasurementLoop::safeDelay(uint32_t millis)
    {
    this->refreshWatchdog();
    delay(millis);
    this->refreshWatchdog();
    }

void cMeasurementLoop::updateScd30Measurements()
    {
    if (this->m_fScd30)
        {
        bool fError;
        if (this->m_Scd.queryReady(fError))
            {
            this->m_measurement_valid = this->m_Scd.readMeasurement();
            if ((! this->m_measurement_valid) && gLog.isEnabled(gLog.kError))
                {
                gLog.printf(gLog.kError, "SCD30 measurement failed: error %s(%u)\n",
                        this->m_Scd.getLastErrorName(),
                        unsigned(this->m_Scd.getLastError())
                        );
                }
            }
        else if (fError)
            {
            if (gLog.isEnabled(gLog.DebugFlags::kError))
                gLog.printf(
                    gLog.kAlways,
                    "SCD30 queryReady failed: status %s(%u)\n",
                    this->m_Scd.getLastErrorName(),
                    unsigned(this->m_Scd.getLastError())
                    );
            }
        }

    if (this->m_fScd30 && this->m_measurement_valid)
        {
        auto const m = this->m_Scd.getMeasurement();
        // temperature is 2 bytes from -163.840 to +163.835 degrees C
        // pressure is 4 bytes, first signed units, then scale.
        if (gLog.isEnabled(gLog.kInfo))
            {
            this->ts = ' ';
            this->t100 = std::int32_t(m.Temperature * 100.0f + 0.5f);
            if (m.Temperature < 0) {
                this->ts = '-';
                this->t100 = -this->t100;
                }
            this->tint = this->t100 / 100;
            this->tfrac = this->t100 - (tint * 100);

            this->rh100 = std::int32_t(m.RelativeHumidity * 100.0f + 0.5f);
            this->rhint = this->rh100 / 100;
            this->rhfrac = this->rh100 - (this->rhint * 100);

            this->co2_100 = std::int32_t(m.CO2ppm * 100.0f + 0.5f);
            this->co2int = this->co2_100 / 100;
            this->co2frac = this->co2_100 - (this->co2int * 100);
            }

        this->m_data.co2ppm.CO2ppm = m.CO2ppm;
        }
    }

void cMeasurementLoop::updateSynchronousMeasurements()
    {
    this->m_currentIntervalSec = uint32_t(millis() - this->m_startTimeMs) / 1000;
    if (this->m_currentIntervalSec > m_versionSec)
        {
        this->m_startTimeMs = millis();
        this->m_fVersionOta = true;
        }

    this->m_data.Vbat = gCatena.ReadVbat();
    if (! this->m_fVersionOta)
        this->fData_Vbat = true;

    if (this->m_fVersionOta)
        {
        this->m_data.ver.Major = kMajor;
        this->m_data.ver.Minor = kMinor;
        this->m_data.ver.Patch = kPatch;
        this->m_data.ver.Local = kLocal;
        this->fData_Version = true;
        }

    if (this->m_data.Vbat < 3.2f)
        this->m_fDatalimit = true;
    else
        this->m_fDatalimit = false;

    // modify Activity Timer if uplink interval is one hour
    if (this->m_fDatalimit)
        {
        // timer has fired. grab data
        this->m_ActivityTimer.begin(this->m_ActivityDataLimitTimerSec * 1000);
        }
    else    /* modify Activity Timer if uplink interval 6 minutes */
        {
        // timer has fired. grab data
        this->m_ActivityTimer.begin(this->m_ActivityTimerSec * 1000);
        }

    this->m_data.Vbus = gCatena.ReadVbus();

    if (gCatena.getBootCount(this->m_data.BootCount))
        {
        if (! this->m_fVersionOta)
            this->fData_BootCount = true;
        }

    gpMeasurementLoopConcrete->takeMeasurements();

    // disable flag for version number over air
    this->m_fVersionOta = false;

    // update activity -- this is is already handled elsewhere

    // grab data on pellets.
    cPelletFeeder::PelletFeederData data;
    this->m_PelletFeeder.readAndReset(data);
    this->fData_Pellet = true;

    // fill in the measurement.
    for (unsigned i = 0; i < kMaxPelletEntries; ++i)
        {
        this->m_data.pellets[i].Total = data.feeder[i].total;
        this->m_data.pellets[i].Recent = data.feeder[i].current;
        }

    // grab time of last activity update.
    gClock.get(this->m_data.DateTime);

    if (this->m_data.co2ppm.CO2ppm != 0.0f)
        {
        this->fData_CO2 = true;
        }
    }

void cMeasurementLoop::measureActivity()
    {
    if (this->m_data.nActivity == this->kMaxActivityEntries)
        {
        // make room by deleting first entry
        for (unsigned i = 0; i < this->kMaxActivityEntries - 1; ++i)
            this->m_data.activity[i] = this->m_data.activity[i+1];

        this->m_data.nActivity = this->kMaxActivityEntries - 1;
        }

    // get another measurement.
    uint32_t const tDelta = this->m_pirLastTimeMs - this->m_pirBaseTimeMs;
    this->m_data.activity[this->m_data.nActivity++].Avg = this->m_pirSum / tDelta;
    this->fData_Activity = true;

    // record time. Since a zero timevalue is always invalid, we don't
    // need to check validity.
    (void) gClock.get(this->m_data.DateTime);

    // start new measurement.
    this->m_pirBaseTimeMs = this->m_pirLastTimeMs;
    this->m_pirMax = -1.0f;
    this->m_pirMin = 1.0f;
    this->m_pirSum = 0.0f;
    }

void cMeasurementLoop::resetPirAccumulation()
    {
    this->m_pirMax = -1.0f;
    this->m_pirMin = 1.0f;
    this->m_pirSum = 0.0f;
    this->m_pirBaseTimeMs = millis();
    this->m_pirLastTimeMs = this->m_pirBaseTimeMs;
    }

void cMeasurementLoop::accumulatePirData()
    {
    std::uint32_t thisTimeMs;
    std::uint32_t deltaT;
    float v = this->m_pir.readWithTime(thisTimeMs);

    if (v > this->m_pirMax)
        this->m_pirMax = v;
    if (v < this->m_pirMin)
        this->m_pirMin = v;

    deltaT = thisTimeMs - this->m_pirLastTimeMs;
    this->m_pirSum += v * deltaT;
    this->m_pirLastTimeMs = thisTimeMs;
    }

/****************************************************************************\
|
|   Start uplink of data
|
\****************************************************************************/

void cMeasurementLoop::startTransmission(
    cMeasurementLoop::TxBuffer_t &b
    )
    {
    auto const savedLed = gLed.Set(McciCatena::LedPattern::Off);
    if (!(this->fDisableLED && this->m_fLowLight))
        {
        gLed.Set(McciCatena::LedPattern::Sending);
        }

    // by using a lambda, we can access the private contents
    auto sendBufferDoneCb =
        [](void *pClientData, bool fSuccess)
            {
            auto const pThis = (cMeasurementLoop *)pClientData;
            pThis->m_txpending = false;
            pThis->m_txcomplete = true;
            pThis->m_txerr = ! fSuccess;
            pThis->m_fsm.eval();
            };

    bool fConfirmed = false;
    if (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fConfirmedUplink))
        {
        gCatena.SafePrintf("requesting confirmed tx\n");
        fConfirmed = true;
        }

    this->m_txpending = true;
    this->m_txcomplete = this->m_txerr = false;

    if (this->uplinkPort != kUplinkPortDataLimit)
        {
        if (this->fNwTimeSet)
            {
            this->uplinkPort = kUplinkPortwithNwTime;
            this->fNwTimeSet = false;
            }
        else
            {
            this->uplinkPort = kUplinkPortDefault;
            }
        }

    if (! gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, (void *)this, fConfirmed, this->uplinkPort))
        {
        // uplink wasn't launched.
        this->m_txcomplete = true;
        this->m_txerr = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::sendBufferDone(bool fSuccess)
    {
    this->m_txpending = false;
    this->m_txcomplete = true;
    this->m_txerr = ! fSuccess;
    this->m_fsm.eval();
    }

/****************************************************************************\
|
|   The Polling function --
|
\****************************************************************************/

void cMeasurementLoop::poll()
    {
    bool fEvent;

    // no need to evaluate unless something happens.
    fEvent = false;

    // if we're not active, and no request, nothing to do.
    if (! this->m_active)
        {
        if (! this->m_rqActive)
            return;

        // we're asked to go active. We'll want to eval.
        fEvent = true;
        }

    // accumulate PIR data
    if (this->m_pirSampleTimer.isready())
        {
        // timer has fired. grab data
        this->accumulatePirData();
        }

    // record PIR m_pirSampleSec
    if (this->m_ActivityTimer.isready())
        {
        // time to record another minute of data.
        this->measureActivity();
        if (this->m_data.nActivity == this->kMaxActivityEntries)
            fEvent = true;
        }

    auto const msToNext = this->m_Scd.getMsToNextMeasurement();
    if (msToNext < 20)
        updateScd30Measurements();

    if (this->m_fTimerActive)
        {
        if ((millis() - this->m_timer_start) >= this->m_timer_delay)
            {
            this->m_fTimerActive = false;
            this->m_fTimerEvent = true;
            fEvent = true;
            }
        }

    // check the transmit time.
    if (this->m_UplinkTimer.peekTicks() != 0)
        {
        fEvent = true;
        }

    if (fEvent)
        this->m_fsm.eval();

    this->m_data.Vbus = gCatena.ReadVbus();
    setVbus(this->m_data.Vbus);

    if (!(this->m_fUsbPower) && !(this->m_fFwUpdate) && !(os_queryTimeCriticalJobs(ms2osticks(timeOut))))
        lptimSleep(timeOut);
    }

void user_request_network_time_cb(void *pVoidUserUTCTime, int flagSuccess) {
    // Explicit conversion from void* to uint32_t* to avoid compiler errors
    uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;

    // A struct that will be populated by LMIC_getNetworkTimeReference.
    // It contains the following fields:
    //  - tLocal: the value returned by os_GetTime() when the time
    //            request was sent to the gateway, and
    //  - tNetwork: the seconds between the GPS epoch and the time
    //              the gateway received the time request
    lmic_time_reference_t lmicTimeReference;

    if (flagSuccess != 1) {
        gCatena.SafePrintf("USER CALLBACK: Not a success\n");
        return;
        }

    // Populate "lmic_time_reference"
    flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
    if (flagSuccess != 1) {
        gCatena.SafePrintf("USER CALLBACK: LMIC_getNetworkTimeReference didn't succeed\n");
        return;
        }

    // Update userUTCTime, considering the difference between the GPS and UTC
    // epoch, and the leap seconds
    *pUserUTCTime = lmicTimeReference.tNetwork + 315964800;

    // Add the delay between the instant the time was transmitted and
    // the current time

    // Current time, in ticks
    ostime_t ticksNow = os_getTime();
    // Time when the request was sent, in ticks
    ostime_t ticksRequestSent = lmicTimeReference.tLocal;
    uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
    *pUserUTCTime += requestDelaySec;

    // gDate.setGpsTime((int64_t)*pUserUTCTime);
    gDate.setCommonTime((int64_t)*pUserUTCTime);

    gCatena.SafePrintf(
                "The current GPS time is: %04d-%02d-%02d %02d:%02d:%02d\n",
                gDate.year(), gDate.month(), gDate.day(),
                gDate.hour(), gDate.minute(), gDate.second()
                );

    unsigned errCode;
    if (! gClock.set(gDate, &errCode))
        gCatena.SafePrintf("couldn't set clock: %u\n", errCode);
    else
        gpMeasurementLoopConcrete->fNwTimeSet = true;

    gpMeasurementLoopConcrete->startTime = millis();
    }

static void setup_lptim(uint32_t msec)
    {
    // enable clock to LPTIM1
    __HAL_RCC_LPTIM1_CLK_ENABLE();
    __HAL_RCC_LPTIM1_CLK_SLEEP_ENABLE();

    auto const pLptim = LPTIM1;

    // set LPTIM1 clock to LSE clock.
    __HAL_RCC_LPTIM1_CONFIG(RCC_LPTIM1CLKSOURCE_LSE);

    // disable everything so we can tweak the CFGR
    pLptim->CR = 0;

    // upcount from selected internal clock (which is LSE)
    auto rCfg = pLptim->CFGR & ~0x01FEEEDF;
    rCfg |=  0;
    pLptim->CFGR = rCfg;

    // enable the counter but don't start it
    pLptim->CR = LPTIM_CR_ENABLE;
    delayMicroseconds(100);

    // Clear ICR and ISR registers
    pLptim->ICR |= 0x3F;
    pLptim->ISR &= 0x00;

    // Auto-Reload Register is a 16-bit register
    // set ARR to value between 0 to 0xFFFF ( < 1999 ms )
    // must be done after enabling.
    uint32_t timeoutCount;
    timeoutCount = ((32768 * msec) / 1000);
    pLptim->ARR = timeoutCount;

    // Autoreload match interrupt
    pLptim->IER |= LPTIM_IER_ARRMIE;

    NVIC_SetPriority(LPTIM1_IRQn, 1);
    NVIC_DisableIRQ(LPTIM1_IRQn);

    // start in continuous mode.
    pLptim->CR = LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT;

    // enable LPTIM interrupt routine
    NVIC_EnableIRQ(LPTIM1_IRQn);
    }

void lptimSleep(uint32_t timeOut)
    {
    uint32_t sleepTimeMS;
    sleepTimeMS = timeOut;

    setup_lptim(sleepTimeMS);

    gpMeasurementLoopConcrete->deepSleepPrepare();

    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(
          PWR_LOWPOWERREGULATOR_ON,
          PWR_STOPENTRY_WFI
          );

    HAL_IncTick();
    HAL_ResumeTick();
    HAL_AddTick(sleepTimeMS);

    gpMeasurementLoopConcrete->deepSleepRecovery();
    }

uint32_t HAL_AddTick(
   uint32_t delta
    )
    {
    extern __IO uint32_t uwTick;
    // copy old interrupt-enable state to flags.
    uint32_t const flags = __get_PRIMASK();

    // disable interrupts
    __set_PRIMASK(1);

    // observe uwTick, and advance it.
    uint32_t const tickCount = uwTick + delta;

    // save uwTick
    uwTick = tickCount;

    // restore interrupts (does nothing if ints were disabled on entry)
    __set_PRIMASK(flags);

    // return the new value of uwTick.
    return tickCount;
    }

extern "C" {
void LPTIM1_IRQHandler(void)
    {
    NVIC_ClearPendingIRQ(LPTIM1_IRQn);
    if(LPTIM1->ISR & LPTIM_ISR_ARRM) //If there was a compare match
        {
        /* If the interrupt was enabled */
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF;
        LPTIM1->ICR |= LPTIM_ICR_CMPOKCF;
        LPTIM1->CR = 0;
        }
    }
}

/****************************************************************************\
|
|   Update the TxCycle count.
|
\****************************************************************************/

void cMeasurementLoop::updateTxCycleTime()
    {
    auto txCycleCount = this->m_txCycleCount;

    // update the sleep parameters
    if (txCycleCount > 1)
            {
            // values greater than one are decremented and ultimately reset to default.
            this->m_txCycleCount = txCycleCount - 1;
            }
    else if (txCycleCount == 1)
            {
            // it's now one (otherwise we couldn't be here.)
            gCatena.SafePrintf("resetting tx cycle to default: %u\n", this->m_txCycleSec_Permanent);

            this->uplinkPort = kUplinkPortDefault;
            this->setTxCycleTime(this->m_txCycleSec_Permanent, 0);
            }
    else if (this->m_fDatalimit && this->uplinkPort != kUplinkPortDataLimit)
            {
            // transmit to network once in an hour, if Vbat < 3.3V.
            gCatena.SafePrintf("resetting tx cycle to data limit mode: %u\n", this->m_txCycleSec_Low_Power);

            this->uplinkPort = kUplinkPortDataLimit;
            this->setTxCycleTime(this->m_txCycleSec_Low_Power, 0);
            }
    else if (!this->m_fDatalimit && this->uplinkPort == kUplinkPortDataLimit)
            {
            // it's back to default
            gCatena.SafePrintf("resetting tx cycle back to default: %u\n", this->m_txCycleSec_Permanent);

            this->uplinkPort = kUplinkPortDefault;
            this->setTxCycleTime(this->m_txCycleSec_Permanent, 0);
            }
    else
            {
            // it's zero. Leave it alone.
            }
    }

/****************************************************************************\
|
|   Handle sleep between measurements
|
\****************************************************************************/

void cMeasurementLoop::sleep()
    {
    const bool fDeepSleep = checkDeepSleep();

    if (! this->m_fPrintedSleeping)
            this->doSleepAlert(fDeepSleep);

    if (fDeepSleep)
            this->doDeepSleep();
    }

// for now, we simply don't allow deep sleep. In the future might want to
// use interrupts on activity to wake us up; then go back to sleep when we've
// seen nothing for a while.
bool cMeasurementLoop::checkDeepSleep()
    {
    bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
                    static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    bool fDeepSleep;
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (! this->kEnableDeepSleep)
        {
        return false;
        }

    if (sleepInterval < 2)
        fDeepSleep = false;
    else if (fDeepSleepTest)
        {
        fDeepSleep = true;
        }
#ifdef USBCON
    else if (Serial.dtr())
        {
        fDeepSleep = false;
        }
#endif
    else if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDisableDeepSleep))
        {
        fDeepSleep = false;
        }
    else if ((gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)) != 0)
        {
        fDeepSleep = true;
        }
    else
        {
        fDeepSleep = false;
        }

    return fDeepSleep;
    }

void cMeasurementLoop::doSleepAlert(bool fDeepSleep)
    {
    this->m_fPrintedSleeping = true;

    if (fDeepSleep)
        {
        bool const fDeepSleepTest =
                gCatena.GetOperatingFlags() &
                    static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
        const uint32_t deepSleepDelay = fDeepSleepTest ? 10 : 30;

        gCatena.SafePrintf("using deep sleep in %u secs"
#ifdef USBCON
                            " (USB will disconnect while asleep)"
#endif
                            ": ",
                            deepSleepDelay
                            );

        if (!(this->fDisableLED && this->m_fLowLight))
            {
            // sleep and print
            gLed.Set(McciCatena::LedPattern::TwoShort);
            }

        for (auto n = deepSleepDelay; n > 0; --n)
            {
            uint32_t tNow = millis();

            while (uint32_t(millis() - tNow) < 1000)
                {
                this->refreshWatchdog();
                gCatena.poll();
                yield();
                }
            gCatena.SafePrintf(".");
            }
        gCatena.SafePrintf("\nStarting deep sleep.\n");
        uint32_t tNow = millis();
        while (uint32_t(millis() - tNow) < 100)
            {
            this->refreshWatchdog();
            gCatena.poll();
            yield();
            }
        }
    else
        gCatena.SafePrintf("using light sleep\n");
    }

void cMeasurementLoop::doDeepSleep()
    {
    // bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
    //                         static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval == 0)
        return;

    /* ok... now it's time for a deep sleep */
    gLed.Set(McciCatena::LedPattern::Off);
    this->deepSleepPrepare();

    /* sleep */
    // gCatena.Sleep(sleepInterval);
    this->fitfulSleep(sleepInterval);

    /* recover from sleep */
    this->deepSleepRecovery();

    /* and now... we're awake again. trigger another measurement */
    this->m_fsm.eval();
    }

// sleep, broken up into intervals of 10 seconds or so.
void cMeasurementLoop::fitfulSleep(uint32_t seconds)
    {
    /* we need to sleep for not too long each time, so we can have the watchdog enabled */
    while (seconds > 0)
        {
        uint32_t nSecondsThisTime = seconds;
        if (nSecondsThisTime > this->kNumSecondsFitfulSleepMax)
            nSecondsThisTime = kNumSecondsFitfulSleepMax;

        gCatena.Sleep(nSecondsThisTime);
        seconds -= nSecondsThisTime;

        this->refreshWatchdog();
        }
    }

#define FUNCTION "cMeasurementLoop::setupWatchdog"

void cMeasurementLoop::setupWatchdog()
    {
    constexpr uint32_t IWDG_KEY_ENABLE = 0xCCCC;
    constexpr uint32_t IWDG_KEY_WRITE_ACCESS_ENABLE = 0x5555;
    constexpr uint32_t IWDG_KEY_WRITE_ACCESS_DISABLE = 0;

    // compute number of ticks to set in the reload register.
    // 40000 is the rough RC frequency of the watchdog, 256 is the pre-scaler.
    constexpr uint32_t knTicks = (this->kWatchdogSeconds * 40000) / 256;
    static_assert(knTicks <= 0xFFF, "knTicks must fit in a 12-bit field");

    // enable the IWDG
    IWDG->KR = IWDG_KEY_ENABLE;
    // allow write access.
    IWDG->KR = IWDG_KEY_WRITE_ACCESS_ENABLE;
    // set prescaler to 7, i.e., divide by 256. So one tick == 40000 Hz/256 == 156.25 Hz == 6.4ms
    IWDG->PR = 7;
    // set reload register.
    IWDG->RLR = knTicks;
    // wait for the register to update. Since we're initializing, we don't
    // really care very much.
    for (uint32_t tNow = millis(); millis() - tNow < 48;)
        {
        if (IWDG->SR == 0)
            break;
        }
    if (IWDG->SR != 0)
        {
        gCatena.SafePrintf("?" FUNCTION ": watchdog setup failed!: %x\n", IWDG->SR);
        }

    // refresh the watchdog
    this->refreshWatchdog();
    }

#undef FUNCTION

void cMeasurementLoop::refreshWatchdog()
    {
    constexpr uint32_t IWDG_KEY_REFRESH = 0xAAAA;

    IWDG->KR = IWDG_KEY_REFRESH;
    }

//
// call this after waking up from a long (> 15 minute) sleep to correct for LMIC sleep defect
// This should be done after updating micros() and updating LMIC's idea of time based on
// the sleep time.
//
void fixLmicTimeCalculationAfterWakeup(void) {
    ostime_t const now = os_getTime();
    // just tell the LMIC that we're available *now*.
    LMIC.globalDutyAvail = now;
    // no need to randomize
    // for EU-like, we need to reset all the channel avail times to "now"
#if CFG_LMIC_EU_like
    for (unsigned i = 0; i < MAX_BANDS; ++i) {
        LMIC.bands[i].avail = now;
    }
#endif
}

void cMeasurementLoop::deepSleepPrepare(void)
    {
    pinMode(kVddPin, INPUT);

    if (this->m_fSleepScd30)
        {
        // stop the SCD30; we leave it running.
        this->m_Scd.end();
        }

    Serial.end();
    Wire.end();
    SPI.end();
    if (this->m_pSPI2 && this->m_fSpi2Active)
        {
        this->m_pSPI2->end();
        this->m_fSpi2Active = false;
        }
    }

void cMeasurementLoop::deepSleepRecovery(void)
    {
    pinMode(kVddPin, OUTPUT);
    digitalWrite(kVddPin, HIGH);
    
    Serial.begin();
    Wire.begin();
    SPI.begin();
    fixLmicTimeCalculationAfterWakeup();

    if (this->m_fSleepScd30)
        {
        // start the SCD30, and make sure it passes the bring-up.
        // record success in m_fDiffPressure, which is used later
        // when collecting results to transmit.
        this->m_fScd30 = this->m_Scd.begin();
        this->m_fSleepScd30 = false;

        // if it didn't start, log a message.
        if (! this->m_fScd30)
            {
            if (gLog.isEnabled(gLog.DebugFlags::kError))
                gLog.printf(
                        gLog.kAlways,
                        "SCD30 begin() failed after sleep: status %s(%u)\n",
                        this->m_Scd.getLastErrorName(),
                        unsigned(this->m_Scd.getLastError())
                        );
            }
        }
    }

/****************************************************************************\
|
|  Time-out asynchronous measurements.
|
\****************************************************************************/

// set the timer
void cMeasurementLoop::setTimer(std::uint32_t ms)
    {
    this->m_timer_start = millis();
    this->m_timer_delay = ms;
    this->m_fTimerActive = true;
    this->m_fTimerEvent = false;
    }

void cMeasurementLoop::clearTimer()
    {
    this->m_fTimerActive = false;
    this->m_fTimerEvent = false;
    }

bool cMeasurementLoop::timedOut()
    {
    bool result = this->m_fTimerEvent;
    this->m_fTimerEvent = false;
    return result;
    }

