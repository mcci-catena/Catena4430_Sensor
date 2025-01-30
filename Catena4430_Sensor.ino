/*

Module: Catena4430_Sensor.ino

Function:
    Remote sensor example for the Catena 4430.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Terry Moore, MCCI Corporation   July 2019

*/

#include <Arduino.h>
#include <Wire.h>
#include <Catena.h>
#include "Catena4430_Sensor.h"
#include <arduino_lmic.h>
#include <Catena_Timer.h>
#include <Catena4430.h>
#include <Catena_FlashParam.h>
#include <Catena_Date.h>
#include <Catena4430_cPCA9570.h>
#include <Catena4430_c4430Gpios.h>
#include <Catena4430_cPIRdigital.h>
#include "Catena4430_cMeasurementLoop.h"
#include "Catena4430_cmd.h"
#include <Catena4430_cClockDriver_PCF8523.h>
#include <MCCI_Catena_SCD30.h>
#include <Catena_WatchdogTimer.h>

extern McciCatena::Catena gCatena;
using namespace McciCatena4430;
using namespace McciCatena;
using namespace McciCatenaScd30;

static_assert(
    CATENA_ARDUINO_PLATFORM_VERSION_COMPARE_GE(
        CATENA_ARDUINO_PLATFORM_VERSION, 
        CATENA_ARDUINO_PLATFORM_VERSION_CALC(0, 21, 0, 5)
        ),
    "This sketch requires Catena-Arduino-Platform v0.21.0-5 or later"
    );

constexpr std::uint32_t kAppVersion = McciCatena4430::makeVersion(2,5,1,0);
constexpr std::uint32_t kDoubleResetWaitMs = 3000;
constexpr std::uint32_t kTripleResetWaitMs = 4000;
constexpr std::uint32_t kSetDoubleResetMagic = 0xCA44301;
constexpr std::uint32_t kSetTripleResetMagic = 0xCA44302;
constexpr std::uint32_t kClearDoubleResetMagic = 0xCA44300;

/****************************************************************************\
|
|   Variables.
|
\****************************************************************************/


// the global I2C GPIO object
cPCA9570                i2cgpio    { &Wire };

// the global clock object
cClockDriver_PCF8523    gClock      { &Wire };

c4430Gpios gpio     { &i2cgpio };
Catena gCatena;
cDate gDate;
cTimer ledTimer;
Catena::LoRaWAN gLoRaWAN;
StatusLed gLed (Catena::PIN_STATUS_LED);

// allocate the buffer for cMeasurementLoop.
constexpr auto alignData = alignof(cMeasurementLoop);
constexpr size_t bufSize = sizeof(cMeasurementLoop);
static alignas(alignData) uint8_t buf[bufSize];

//
// The concrete instance for class `cMeasurementLoop` varies at runtime
// according to the hardware in use.
// This pointer is to the concrete
// instance viewed as an abstract object;
// virtual methods are provided by
// the concrete type to allow portable
// code to access the concrete hardware
// without knowing what type is in use.
//
cMeasurementLoop *gpMeasurementLoopConcrete = new(buf) cMeasurementLoop();

// concrete type for flash parameters
using Flash_t = McciCatena::FlashParamsStm32L0_t;
using ParamBoard_t = Flash_t::ParamBoard_t;
using PageEndSignature1_t = Flash_t::PageEndSignature1_t;
using ParamDescId = Flash_t::ParamDescId;

/* instantiate the bootloader API */
cBootloaderApi gBootloaderApi;

/* instantiate SPI */
SPIClass gSPI2(
		Catena::PIN_SPI2_MOSI,
		Catena::PIN_SPI2_MISO,
		Catena::PIN_SPI2_SCK
		);

/* instantiate the flash */
Catena_Mx25v8035f gFlash;

/* instantiate the downloader */
cDownload gDownload;

unsigned ledCount;
bool fAnalogPin1;
bool fAnalogPin2;
bool fCheckPinA1;
bool fCheckPinA2;
bool fToggle;
bool gfRejoin;

/****************************************************************************\
|
|   User commands
|
\****************************************************************************/

// the individual commmands are put in this table
static const cCommandStream::cEntry sMyExtraCommmands[] =
        {
        { "date", cmdDate },
        { "dir", cmdDir },
        { "log", cmdLog },
        { "tree", cmdDir },
        { "info", cmdInfo },
        { "interval", cmdInterval },
        { "hang", cmdHang },
        // other commands go here....
        };

/* a top-level structure wraps the above and connects to the system table */
/* it optionally includes a "first word" so you can for sure avoid name clashes */
static cCommandStream::cDispatch
sMyExtraCommands_top(
        sMyExtraCommmands,          /* this is the pointer to the table */
        sizeof(sMyExtraCommmands),  /* this is the size of the table */
        nullptr                     /* this is no "first word" for all the commands in this table */
        );


/****************************************************************************\
|
|   Setup
|
\****************************************************************************/

void setup()
    {
    setup_double_reset();

    setup_version();

    setup_platform();
    setup_printSignOn();

    setup_radio();
    setup_flash();
    setup_download();
    setup_measurement();
    setup_gpio();
    setup_rtc();
    setup_commands();
    setup_watchdog();
    setup_start();
    }

void setup_double_reset()
    {
    const uint32_t resetReason = READ_REG(RCC->CSR);
    if (resetReason & RCC_CSR_PINRSTF)
        {
        if (RTC->BKP0R == kSetTripleResetMagic)
            {
            gfRejoin = true;
            RTC->BKP0R = kClearDoubleResetMagic;
            }
        else if (RTC->BKP0R == kSetDoubleResetMagic)
            {
            RTC->BKP0R = kSetTripleResetMagic;
            pinMode(D13, OUTPUT);
            digitalWrite(D13, HIGH);
            delay(kTripleResetWaitMs);
            digitalWrite(D13, LOW);
            fToggle = true;
            RTC->BKP0R = kClearDoubleResetMagic;
            }
        else
            {
            RTC->BKP0R = kSetDoubleResetMagic;
            pinMode(D13, OUTPUT);
            digitalWrite(D13, HIGH);
            delay(kDoubleResetWaitMs);
            digitalWrite(D13, LOW);
            RTC->BKP0R = kClearDoubleResetMagic;
            }
        }
    }

void setup_platform()
    {
    const uint32_t setDisableLedFlag = 0x40000000;
    const uint32_t clearDisableLedFlag = 0x0FFFFFFF;
    uint32_t savedFlag;

    gCatena.begin();
    savedFlag = gCatena.GetOperatingFlags();

    // if running unattended, don't wait for USB connect.
    if (! (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)))
        {
        while (!Serial)
            /* wait for USB attach */
            yield();
        }

    // check for pin reset and second reset
    if (fToggle)
        {
        // check if LEDs are disabled
        if (savedFlag & static_cast<uint32_t>(gpMeasurementLoopConcrete->OPERATING_FLAGS::fDisableLed))
            {
            // clear flag to disable LEDs
            savedFlag &= clearDisableLedFlag;
            gCatena.getFram()->saveField(
                cFramStorage::StandardKeys::kOperatingFlags,
                savedFlag
                );
            }

        // if LEDs are not disabled
        else
            {
            // set flag to disable LEDs
            savedFlag |= setDisableLedFlag;
            gCatena.getFram()->saveField(
                cFramStorage::StandardKeys::kOperatingFlags,
                savedFlag
                );
            }

        // update operatingFlag in the library
        gCatena.SetOperatingFlags(savedFlag);
        }
    }

static constexpr const char *filebasename(const char *s)
    {
    const char *pName = s;

    for (auto p = s; *p != '\0'; ++p)
        {
        if (*p == '/' || *p == '\\')
            pName = p + 1;
        }
    return pName;
    }

void setup_printSignOn()
    {
    static const char dashes[] = "------------------------------------";

    gCatena.SafePrintf("\n%s%s\n", dashes, dashes);

    // need username as other libraries has similar get version APIs
    gCatena.SafePrintf("This is %s v%d.%d.%d-%d.\n",
        filebasename(__FILE__),
        McciCatena4430::getMajor(kAppVersion),
        McciCatena4430::getMinor(kAppVersion),
        McciCatena4430::getPatch(kAppVersion),
        McciCatena4430::getLocal(kAppVersion)
        );

    do
        {
        char sRegion[16];
        gCatena.SafePrintf("Target network: %s / %s\n",
                        gLoRaWAN.GetNetworkName(),
                        gLoRaWAN.GetRegionString(sRegion, sizeof(sRegion))
                        );
        } while (0);

    gCatena.SafePrintf("System clock rate is %u.%03u MHz\n",
        ((unsigned)gCatena.GetSystemClockRate() / (1000*1000)),
        ((unsigned)gCatena.GetSystemClockRate() / 1000 % 1000)
        );
    gCatena.SafePrintf("Enter 'help' for a list of commands.\n");

    gCatena.SafePrintf("%s%s\n" "\n", dashes, dashes);
    }

bool flashParam()
    {
    // fetch the signature
    const PageEndSignature1_t * const pRomSig =
            reinterpret_cast<const PageEndSignature1_t *>(Flash_t::kPageEndSignature1Address);

    // get pointer to memory block */
    uint32_t const descAddr = pRomSig->getParamPointer();

    // find the serial number (must be first)
    gpMeasurementLoopConcrete->m_pBoard = reinterpret_cast<const ParamBoard_t *>(descAddr);

    const auto guid { Flash_t::kPageEndSignature1_Guid };

    if (std::memcmp((const void *) &pRomSig->Guid, (const void *) &guid, sizeof(pRomSig->Guid)) != 0)
        {
        if (gpMeasurementLoopConcrete->isTraceEnabled(cMeasurementLoop::DebugFlags::kError))
            gLog.printf(gLog.kError, "Guid value wrong\n");
        return false;
        }

    if (! pRomSig->isValidParamPointer(descAddr))
        {
        if (gpMeasurementLoopConcrete->isTraceEnabled(cMeasurementLoop::DebugFlags::kError))
            gLog.printf(gLog.kError, "invalid paramter pointer: %#08x\n", descAddr);
        return false;
        }

    // check the ID and length
    if (! (
        gpMeasurementLoopConcrete->m_pBoard->uLen == sizeof(*gpMeasurementLoopConcrete->m_pBoard) &&
        gpMeasurementLoopConcrete->m_pBoard->uType == unsigned(ParamDescId::Board)
        ))
        {
        if (gpMeasurementLoopConcrete->isTraceEnabled(cMeasurementLoop::DebugFlags::kError))
            gLog.printf(
                    gLog.kError,
                    "invalid board length=%02x or type=%02x\n",
                    gpMeasurementLoopConcrete->m_pBoard->uLen,
                    gpMeasurementLoopConcrete->m_pBoard->uType
                    );
        return false;
        }

    gpMeasurementLoopConcrete->setBoardRev(gpMeasurementLoopConcrete->m_pBoard->getRev());
    gpMeasurementLoopConcrete->setBoard(gpMeasurementLoopConcrete->m_pBoard->getModel());
    return true;
    }

void printBoardInfo()
    {
    // print the s/n, model, rev
    gLog.printf(gLog.kInfo, "serial-number:");
    uint8_t serial[gpMeasurementLoopConcrete->m_pBoard->nSerial];
    gpMeasurementLoopConcrete->m_pBoard->getSerialNumber(serial);

    for (unsigned i = 0; i < sizeof(serial); ++i)
        gCatena.SafePrintf("%c%02x", i == 0 ? ' ' : '-', serial[i]);

    gLog.printf(
            gLog.kInfo,
            "\nAssembly-number: %u\nModel: %u\n",
            gpMeasurementLoopConcrete->m_pBoard->getAssembly(),
            gpMeasurementLoopConcrete->m_pBoard->getModel()
            );
    delay(1);
    gLog.printf(
            gLog.kInfo,
            "ModNumber: %u\n",
            gpMeasurementLoopConcrete->m_pBoard->getModNumber()
            );
    gLog.printf(
            gLog.kInfo,
            "RevNumber: %u\n",
            gpMeasurementLoopConcrete->readBoardRev()
            );
    gLog.printf(
            gLog.kInfo,
            "Rev: %c\n",
            gpMeasurementLoopConcrete->m_pBoard->getRevChar()
            );
    gLog.printf(
            gLog.kInfo,
            "Dash: %u\n",
            gpMeasurementLoopConcrete->m_pBoard->getDash()
            );
    }

bool isVersion2()
    {
    if (gpMeasurementLoopConcrete->readBoardRev() < 3)
        return false;
    else
        return true;
    }

void setup_gpio()
    {
    if (! gpio.begin())
        gCatena.SafePrintf("GPIO failed to initialize\n");

    ledTimer.begin(400);

    // set up the LED
    gLed.begin();
    gCatena.registerObject(&gLed);
    gLed.Set(LedPattern::FastFlash);

    if ((gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gpMeasurementLoopConcrete->OPERATING_FLAGS::fDisableLed)))
        {
        gpMeasurementLoopConcrete->fDisableLED = true;
        gLed.Set(McciCatena::LedPattern::Off);
        }
    }

void setup_rtc()
    {
    if (! gClock.begin())
        gCatena.SafePrintf("RTC failed to intialize\n");

    cDate d;
    if (! gClock.get(d))
        {
        if (!gpMeasurementLoopConcrete->fDisableLED)
            {
            uint8_t nBlink = 0;
            while (nBlink < 5)
                {
                gIwdgTimer.refreshWatchdog();
                gpio.setRed(true);
                delay(100);
                gpio.setRed(false);
                delay(100);
                gpio.setRed(true);
                delay(100);
                gpio.setRed(false);
                delay(500);
                nBlink += 1;
                }
            }
        gCatena.SafePrintf("RTC is not running\n");
        }
    else
        {
        gCatena.SafePrintf("RTC is running. Date: %d-%02d-%02d %02d:%02d:%02dZ\n",
            d.year(), d.month(), d.day(),
            d.hour(), d.minute(), d.second()
            );
        }
    }

void setup_flash(void)
    {
    gSPI2.begin();
    if (gFlash.begin(&gSPI2, Catena::PIN_SPI2_FLASH_SS))
        {
        gpMeasurementLoopConcrete->registerSecondSpi(&gSPI2);
        gFlash.powerDown();
        gCatena.SafePrintf("FLASH found, put power down\n");
        }
    else
        {
        gFlash.end();
        gSPI2.end();
        gCatena.SafePrintf("No FLASH found: check hardware\n");
        }
    }

void setup_version(void)
    {
    if (!flashParam())
        {
        gCatena.SafePrintf(
                "**Unable to fetch flash parameters, assuming 4610 version 1 (rev C or earlier)!\n"
                );
        gpMeasurementLoopConcrete->setBoardRev(0);
        gpMeasurementLoopConcrete->setBoard(0);
        }
    else {
        printBoardInfo();
        }

    auto board = gpMeasurementLoopConcrete->readBoard();
    auto boardRev = gpMeasurementLoopConcrete->readBoardRev();
    gpMeasurementLoopConcrete = gpMeasurementLoopConcrete->constructInstanceForHardware(isVersion2());
    gpMeasurementLoopConcrete->setBoardRev(boardRev);
    gpMeasurementLoopConcrete->setBoard(board);
    }

void setup_download()
    {
    gDownload.begin(gFlash, gBootloaderApi);
    }

void setup_radio()
    {
    gLoRaWAN.begin(&gCatena);
    gCatena.registerObject(&gLoRaWAN);
    LMIC_setClockError(10 * MAX_CLOCK_ERROR / 100);
    if (gfRejoin)
        LMIC_unjoinAndRejoin();
    }

void setup_measurement()
    {
    gpMeasurementLoopConcrete->begin();
    }

void setup_commands()
    {
    /* add our application-specific commands */
    gCatena.addCommands(
        /* name of app dispatch table, passed by reference */
        sMyExtraCommands_top,
        /*
        || optionally a context pointer using static_cast<void *>().
        || normally only libraries (needing to be reentrant) need
        || to use the context pointer.
        */
        nullptr
        );
    }

void setup_watchdog()
    {
    gIwdgTimer.setupWatchdog();
    }

void setup_start()
    {
    gpMeasurementLoopConcrete->requestActive(true);
    }

/****************************************************************************\
|
|   Loop
|
\****************************************************************************/

void loop()
    {
    gIwdgTimer.refreshWatchdog();
    gCatena.poll();

    if (gpMeasurementLoopConcrete->fDisableLED)
        {
        gpio.setGreen(false);
        gpio.setBlue(false);

        // set flags of Pin A1 and A2 to false.
        // this used to check A1/A2 when disabling the flag fDisableLed
        fAnalogPin1 = false;
        fAnalogPin2 = false;
        fCheckPinA1 = false;
        fCheckPinA2 = false;
        }
    else
        {
        // copy current PIR state to the blue LED.
        gpio.setRed(digitalRead(A0));

        if (!fCheckPinA1)
            {
            // check the connection pin A1
            if (digitalRead(A1) == 0)
                {
                fAnalogPin1 = true;
                fCheckPinA1 = true;
                }
            }

        if (fAnalogPin1)
            {
            // copy current state of Pin A1 to the Green LED.
            gpio.setGreen(digitalRead(A1));
            }
        else
            gpio.setGreen(false);

        if (!fCheckPinA2)
            {
            // check the connection pin A2
            if (digitalRead(A2) == 0)
                {
                fAnalogPin2 = true;
                fCheckPinA2 = true;
                }
            }

        if (fAnalogPin2)
            {
            // copy current state of Pin A2 to the Green LED.
            gpio.setBlue(digitalRead(A2));
            }
        else
            gpio.setBlue(false);
        }
    }
