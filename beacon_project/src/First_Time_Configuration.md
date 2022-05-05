## DWM 1000 Notes
1. Basic Setup  (this is reserved for updating firmware)
    - battery can be placed in the back 
    - DWM1000 has nRF52832 ARM MCU, and the sample code is built on Segger Embedded Studio. [Download Segger](https://www.segger.com/downloads/jlink/)
        1. Install J-Link Software and Documentation pack, which includes drivers and J-Flash Lite tool needed for reprogramming new FW binary into tag. (not sure if we can do it without J-Link)
        2. Unpack the source code to the dwm1001_tdoa_tag folder. In the Segger IDE, choose File->Open Solution and select the project tdoa_tag.emProject, located in the examples\tdoa_tag\ folder. 
        3. Connect the board to the PC. You may require to install J-Link drivers
        4. To check if the target board is working select Target->Connect J-Link from the menu. To start the TDOA Tag application select Build->Build and Run from menu.
        5. If it is the first time to start your app you will be asked for a licence. License is free for Nordic MCUS


3. We do need a gateway 
    <p align="center">
    <img src="https://user-images.githubusercontent.com/77752418/161642191-b0708c9d-fa80-4126-92ef-0e51c989cb22.png" height="200" width="width"/>
    </p>

    - Positions are calculated on tags. One listener is just reporting the position of the tag. No range info is available there. PANS2 software
    - one gateway is good for 25m

## Commands (See Section 6 Shell Commands, https://www.decawave.com/wp-content/uploads/2019/01/DWM1001-API-Guide-2.2.pdf)
1. set up the network 
    ```bash 
    dmesg | grep tty    #check usb port
    tio -b 115200 /dev/ttyACM0
    tio -b 115200 /dev/ttyACM1

    # set one device as "anchor initiator", https://decaforum.decawave.com/t/anchor-initiator/6629/2
    nmi

    #set three other devices as "anchors"
    nma

    # set the other device as tag: 
    nmt
    # set each tag's update rate to 10hz (15 tags max)     https://www.decawave.com/wp-content/uploads/2019/01/DWM1001-API-Guide-2.2.pdf
    # the tag has accelerometer in place so it knows when stationary
    # aurs update_rate update_rate_when_stationary (multiple of 0.1s)
    aurs 1 2
    ```

2. Commands
    ```bash
    nmg: Get node mode
    nma: Set mode to AN
    nmp: Set UWB mode to passive
    nmi: Set mode to ANI
    nmt: Set mode to TN
    nmtl: Set mode to TN-LP
    nmb: Set mode to BN
    la: Show AN list
    lb: Show BN list
    stg: Get stats
    apg: Get pos

    les: Show meas. and pos.
    lec: Show meas. and pos. in CSV
    lep: Show pos. in CSV
    ```

3. [power consumption for tag](https://decaforum.decawave.com/uploads/default/original/1X/2e9f701294239686ad22cf6e53219614483ad924.pdf)

## Test Plan 
1. 1 listener 
2. distance measurement
2. 4 anchors at the store,  tag 
    - how many tags?
