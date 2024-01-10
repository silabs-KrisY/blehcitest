# BLE HCI Test

A command line utility for BLE RF testing using HCI on Silicon Labs EFR32 devices running RCP firmware

# Getting Started

## Prerequisites
1. An EFR32 with an RCP firmware image (with BLE) and bootloader flashed. Refer to [AN1328: Enabling a Radio Co-Processor using the Bluetooth® LE HCI Function](https://www.silabs.com/documents/public/application-notes/an1328-enabling-rcp-using-bt-hci.pdf) for details on building and using the RCP firmware and [UG489: Silicon Labs Gecko Bootloader User's Guide for GSDK 4.0 and Higher][https://www.silabs.com/documents/public/user-guides/ug489-gecko-bootloader-user-guide-gsdk-4.pdf] for details and instructions on building the bootloader.

2. Linux/Posix build environment (OSX, Raspberry Pi, etc.) or Windows (Cygwin/MinGW).

3. Linux needs the libbluetooth-dev installed:

    `sudo apt-get install libbluetooth-dev`

## Building

Simply clone the repo and then compile the program using the included makefile:
    `git clone https://github.com/silabs-KrisY/blehcitest.git`
    `cd blehcitest`
    `make`

## To Run
1. First make sure your HCI device is up and running using the 'hciconfig' command. If it's not showing up, you can attach using:
`sudo hciattach <serial port> any`
where `<serial port>` is the COM port connected to the DUT (e.g. /dev/ttyACM0).

2. Run the host application, pointing it to the correct hci device.

3. Usage information is found below and also by running the application with "-h" or "--help":
```
OPTIONS
  -h                      		Print help message
  --help
  --version               	   Print version number defined in application.
  --time   <duration ms>      Set time for test in milliseconds, 0 for infinite mode (exit with control-c)
  --packet_type <payload/modulation type, 0:PBRS9 packet payload, 1:11110000 packet payload, 2:10101010 packet payload, 3: PRBS15
								4:11111111 packet payload, 5:00000000 packet payload, 6:00001111 packet payload, 7:01010101 packet payload>
  --power  <power level>      Set power level for test in 1 dBm steps
  --channel <channel index>   Set channel index for test, frequency=2402 MHz + 2*channel>
  --len <test packet length>  Set test packet length>
  --rx                        DTM receive test. Prints number of received DTM packets.
  --phy  <PHY selection for test packets/waveforms/RX mode, 1:1Mbps, 2:2Mbps, 3:125k LR coded (S=8), 4:500k LR coded (S=2).>
  --hci_port <hci port num>    Number of the DUT's HCI port (0=hci0, 1=hci1, 2=hci2, etc.)
```
## Examples

1. Transmit DTM (direct test mode) packets containing a pseudorandom PRBS9 payload of length=25 for 10 seconds on 2404 MHz at 8dBm output power level on hci1 using the 1Mbps PHY.
```
$ sudo ./exe/blehcitest --time 10000 --packet_type 0 --power 8 -u /dev/ttyACM0 --channel 1 --len 25 --hci_port 1 --phy 1
Opening hci port 1
Outputting modulation type 0x00 for 10000 ms at 2404 MHz at 8 dBm, phy=0x01
Test completed successfully. Number of packets transmitted = 16005
```

2. Receive DTM (direct test mode) packets for 10 seconds on 2404 MHz on hci1 using the 1Mbps PHY. Note that the printout below shows an example of 100% packet reception rate when running example 1 (TX with PRBS9 DTM length=25) on a separate device.
```
$ sudo ./exe/blehcitest --time 10000 --rx --phy 1 --hci_port 1
Opening hci port 1
DTM receive enabled, freq=2402 MHz, phy=0x01
Test completed successfully. Number of packets received = 16005
```