/*

Module: Catena4430_cMeasurementLoop_fillBuffer.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#include <Catena_TxBuffer.h>
#include <Catena_FlashParam.h>
#include <Catena_Log.h>
#include <Catena.h>
#include <Catena_Fram.h>

#include "Catena4430_cMeasurementLoop.h"

#include "Catena4430_Sensor.h"

#include <arduino_lmic.h>

using namespace McciCatena;
using namespace McciCatena4430;

extern cMeasurementLoop *gpMeasurementLoopConcrete;
extern McciCatena::Catena gCatena;
extern Catena::LoRaWAN gLoRaWAN;

Arduino_LoRaWAN::ReceivePortBufferCbFn receiveMessage;

auto const pFram = gCatena.getFram();

static void delayPollingJobs(uint32_t tDelayMillis)
    {
    uint32_t tNow = millis();

    while (os_queryTimeCriticalJobs(ms2osticks(tDelayMillis)))
        {
        gCatena.poll();
        }

    while (millis() - tNow < tDelayMillis)
        {
        gCatena.poll();
        }
    }

void cMeasurementLoop::receiveMessage(
    void *pContext,
    uint8_t port,
    const uint8_t *pMessage,
    size_t nMessage
    )
    {
    gpMeasurementLoopConcrete->receiveMessageDone( port, pMessage, nMessage);
    }

void cMeasurementLoop::receiveMessageDone(
    uint8_t port,
    const uint8_t *pMessage,
    size_t nMessage)
    {
    if (port == 0)
        {
        gCatena.SafePrintf("MAC message:");
        for (unsigned i = 0; i < LMIC.dataBeg; ++i)
            {
            gCatena.SafePrintf(" %02x", LMIC.frame[i]);
            }
        gCatena.SafePrintf("\n");
        return;
        }

    else if (! (port == 2 || port == 3))
        {
        gCatena.SafePrintf("invalid message port(%02x)\n",
            port, nMessage
            );
        return;
        }

    if (port == 2)
        {
        this->doDlrqCalibCO2(pMessage, nMessage);
        return;
        }

    this->m_AckTxBuffer.begin();

    switch (pMessage[0])
        {
    case dlrqReset:
        NVIC_SystemReset();
        break;
    case dlrqGetVersion:
        this->doDlrqGetVersion(pMessage);
        break;
    case dlrqResetAppEUI:
        this->doDlrqResetAppEUI(pMessage, nMessage);
        break;
    case dlrqResetAppKey:
        this->doDlrqResetAppKey(pMessage, nMessage);
        break;
    case dlrqRejoin:
        this->doDlrqRejoin(pMessage);
        break;
    default:
        gLog.printf(gLog.kError, "doCommand: unknown request %u, %u bytes\n",
                pMessage[0],
                nMessage
                );
        }
    }

void cMeasurementLoop::doDlrqCalibCO2(
    const uint8_t *pMessage,
    size_t nMessage)
    {
    uint16_t co2Calib;

    if (! (nMessage == 2))
        {
        gCatena.SafePrintf("invalid length(%x)\n",
            nMessage
            );
        return;
        }

    co2Calib = (pMessage[0] << 8) | pMessage[1];

    if (this->m_fScd30)
        if (this->m_Scd.setForcedRecalibrationValue(co2Calib))
            gCatena.SafePrintf("SCD30 is being calibrated to %u ppm successfully\n", co2Calib);
        else
            gCatena.SafePrintf("SCD30 calibration is failed\n");
    else
        gCatena.SafePrintf("SCD30 is not connected\n");
    }

void cMeasurementLoop::doDlrqResetAppEUI(
    const uint8_t *pMessage,
    size_t nMessage)
    {
    this->m_AckTxBuffer.put(pMessage[0]);

    if (nMessage == 1)
        {
        this->m_AckTxBuffer.put(Error_t::kInvalidLength);

        this->m_fRxAck = true;
        return;
        }
    else if (nMessage > 1 && nMessage <= 9)
        {
        uint8_t i = 0;
        uint8_t userAppEUI[8];
        uint8_t nIndex = nMessage - 1;

        // update the AppEUI to user inputs
        while (i < nIndex)
            {
            userAppEUI[i] = pMessage[nMessage - 1];
            nMessage = nMessage - 1;
            i = i + 1;
            }

        while(i < 8)
            {
            userAppEUI[i] = 0;
            i = i + 1;
            }
        this->m_AckTxBuffer.put(Error_t::kSuccess);

        this->m_fRxAck = true;

        pFram->saveField(cFramStorage::kAppEUI, userAppEUI);
        }
    gCatena.SafePrintf("AppEUI Reset have been processed\n");
    }

void cMeasurementLoop::doDlrqResetAppKey(
    const uint8_t *pMessage,
    size_t nMessage)
    {
    this->m_AckTxBuffer.put(pMessage[0]);

    if (nMessage == 1)
        {
        this->m_AckTxBuffer.put(Error_t::kInvalidLength);

        this->m_fRxAck = true;
        return;
        }
    else if (nMessage > 1 && nMessage <= 17)
        {
        uint8_t i = 16;
        uint8_t userAppKey[16];

        // update the AppKey to user inputs
        while (nMessage > 1 && i > 0)
            {
            userAppKey[i - 1] = pMessage[nMessage - 1];
            nMessage = nMessage - 1;
            i = i - 1;
            }

        while(i > 0)
            {
            userAppKey[i - 1] = 0;
            i = i - 1;
            }

        pFram->saveField(cFramStorage::kAppKey, userAppKey);
        this->m_AckTxBuffer.put(Error_t::kSuccess);

        this->m_fRxAck = true;
        }
    gCatena.SafePrintf("AppKey have been set\n");
    }

void cMeasurementLoop::doDlrqRejoin(
    const uint8_t *pMessage
    )
    {
    this->m_AckTxBuffer.put(pMessage[0]);
    this->m_AckTxBuffer.put(Error_t::kSuccess);

    const uint32_t tDelay = (pMessage[1] << 8) | pMessage[2];
    const uint32_t tDelayMs = tDelay * 1000;

    this->m_fRxAck = true;

    delayPollingJobs(tDelayMs);

    LMIC_unjoinAndRejoin();
    gCatena.SafePrintf("Rejoin have been processed\n");
    }

void cMeasurementLoop::doDlrqGetVersion(
    const uint8_t *pMessage
    )
    {
    this->m_AckTxBuffer.put(pMessage[0]);
    this->m_AckTxBuffer.put(Error_t::kSuccess);

    this->m_AckTxBuffer.put(this->kMajor);
    this->m_AckTxBuffer.put(this->kMinor);
    this->m_AckTxBuffer.put(this->kPatch);
    this->m_AckTxBuffer.put(this->kLocal);
    this->m_AckTxBuffer.put2u(this->readBoard());
    this->m_AckTxBuffer.put(this->readBoardRev());

    this->m_fRxAck = true;
    gCatena.SafePrintf("SW Version: v%u.%u.%u.%u\n", this->kMajor, this->kMinor, this->kPatch, this->kLocal);
    gCatena.SafePrintf("HW Details:\n\tBoard: %u\n\tRev: %u\n",
            this->readBoard(),
            this->readBoardRev()
            );
    }

void cMeasurementLoop::sendDownlinkAck(void)
    {
    // by using a lambda, we can access the private contents
    auto ackSentDoneCb =
        [](void *pClientData, bool fSuccess)
            {
            auto const pThis = (cMeasurementLoop *)pClientData;
            pThis->m_txpending = false;
            pThis->m_txcomplete = true;
            pThis->m_txerr = ! fSuccess;
            pThis->m_fsm.eval();
            };

    bool fConfirmed = false;
    auto ackMessage = this->m_AckTxBuffer.getbase();
    auto nAckMessage = this->m_AckTxBuffer.getn();
    static constexpr uint8_t kDownlinkPort = 3;

    if (! gLoRaWAN.SendBuffer(ackMessage, nAckMessage, ackSentDoneCb, (void *)this, fConfirmed, kDownlinkPort))
        {
        // downlink acknowledgment wasn't launched.
        gCatena.SafePrintf("Downlink Response failed\n");
        this->m_txcomplete = true;
        this->m_txerr = true;
        this->m_fsm.eval();
        }
    }
