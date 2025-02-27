# Downlink Request/commands

This documnent describes the data format for various commands/requests being sent as Downlink.

The Catena4430 Sensor application supports following features with Downlink from version 2.2.0.

1. [Set Uplink Interval](#set-uplink-interval)
2. [Device Reset](#reset-device)
3. [Get Version](#get-version)
4. [Reset AppEUI](#resetset-appeui)
5. [Reset AppKey](#resetset-appkey)
6. [Get Hardware and Software version](#get-version)
7. [Rejoin to a Network](#rejoin)
8. [CO2 Calibration](#co2-calibration)

## Downlink Commands

All downlink commands received on LoRaWAN port 1, port 2 and port 3.

### Set Uplink Interval

The command used to set the uplink interval `doDlrqResetInterval` in the FRAM for this application.
This is the only command received on LoRaWAN port 1.

|Bytes | Value | Comment
|:----:|-------|--------
|  0-1   |  `xx` `xx`   | Seconds of uplink interval
|   2   |  `xx`  | Number of cycle the newly set interval works (optional). If not present, the new application interval set indefinitely.

### Reset Device

This command causes the Catena4430 Sensor controller to `RESET`. This command needs to be send on LoRaWAN port 3.

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `02`   | `dlrqReset`

The system immediately calls `NVIC_Reset()` to perform a hard reset. There is no reply.

### Get version

The command used to get the Software version and Hardware board details. This command needs to be send on LoRaWAN port 3.

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `03`   | `dlrqGetVersion`

The reply:

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `03`   | `dlrqGetVersion`
|  1   |  `??`   | error code; 0 for success
| 2-5  | `??` `??` `??` `??` | Application version number, _big-endian_.
|  6-7   |  `??` `??`  | Board Model used, here `4430`
|  8   |  `??`   | Board Revision, for example, if `A`, value is `0`.

### Reset/Set AppEUI

This command causes the Catena4430 Sensor controller to reset the AppEUI used for LoRaWAN registration to user-defined AppEUI. It does not cause the device to attempt to rejoin, and so has no immediate effect on communications. This command needs to be send on LoRaWAN port 3.

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `04`   | `dlrqResetAppEUI`
|  1 - n   |  `?? ?? .... ??`   | user AppEUI from network console, n cannot be more than 9, as AppEUI is maximum of 8 bytes

The reply:

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `04`   | `dlrqResetAppEUI`
|  1   |  `??`   | error code; 0 for success

### Reset/Set AppKey

This command causes the Catena4430 Sensor controller to reset the AppKey used for LoRaWAN registration to user-defined AppKey. It does not cause the device to attempt to rejoin, and so has no immediate effect on communications. This command needs to be send on LoRaWAN port 3.

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `05`   | `dlrqResetAppKey`
|  1 - n   |  `?? ?? .... ??`   | user AppKey from network console, n cannot be more than 17, as AppKey is maximum of 16 bytes

The reply:

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `05`   | `dlrqResetAppKey`
|  1   |  `??`   | error code; 0 for success

### Rejoin

This command causes the Catena4430 Sensor controller to reply, and then try to rejoin the network. This command needs to be send on LoRaWAN port 3.

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `06`   | `dlrqRejoin`
|  1,2   | `xx` `xx` | Seconds to delay before re-joining

The reply:

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `06`   | `dlrqRejoin`
|  1   |  `??`   | error code; 0 for success

If the command is successful, the Catena4430 Sensor delays the specified period of time, then triggers a rejoin. During that delay, no uplinks will be sent other than MAC messages.

`Note`: 15 seconds of delay is recommended (00 0F in HEX)

### Get Uplink Interval

The command used to get the uplink interval stored in FRAM for this application. This command needs to be send on LoRaWAN port 3.

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `07`   | `dlrqGetUplinkInterval`

The reply:

|Bytes | Value | Comment
|:----:|-------|--------
|  0   |  `07`   | `dlrqGetUplinkInterval`
|  1   |  `??`   | error code; 0 for success
| 2-5  | `??` `??` `??` `??` | Uplink Interval (seconds) from FRAM, _big-endian_.

### CO2 Calibration

The calibration of SCD30 CO2 sensor can be carried out via downlink commands. This is the only command that needs to be send on LoRaWAN port 2.

|Bytes | Value | Comment
|:----:|-------|--------
|  0,1   | `xx` `xx` | Calibration value in HEX

For example, when the calibration value is `800` in decimal, then `03 20` in HEX command needs to be send on LoRaWAN port 2.