# simpleFOC-tests-SSL

Tests on FOC library for BLDC motors to be used on the robots of the SSL category at the UnBall team

## Usage

### Calibration

Test codes are inside [lib folder](/lib), where [utils](/lib/utils) should help calibrating the motor, sensors and drivers. If you are using new motors and sensors you must do this routine to calibrate them.

### Testing codes

In order to test the codes, you should copy the code.cpp inside the lib folder and paste it on [main.cpp file](src/main.cpp) 

#### DRV8313 (mini FOC)

Into [DRV8313](lib/DRV8313) folder you should be able to test each component seperately when using this chip.  

**Please check the pins you are using and put the correct GPIO_NUM at the object constructor for the sensor and the motor.**

Following the steps inside the lib you should be able to fully control velocity or angle on a closed loop control.

#### EG2133

Into [EG2133](lib/EG2133) folder you should be able to test each component seperately when using this chip.  

**Note 1:  
Please note that this board is able to control two motors at the same time. So if anything goes wrong, you should test it using only one motor before continuing.**

**Note 2:  
If you are using the MKS ESP32 FOC the pins for the motors are fixed, so you should not change them, only do it for the sensors. When using PWM for the sensors, you must use the I_0(GPIO_NUM_15) and I_1(GPIO_NUM_13) external ports to connect**

**Note 3:
When using the MKS DUAL FOC PLUS, the pins are also fixed with the esp32 shilded onto the board, but you can also use jumpers to connect an external not-shielded version, just make sure you read the documentation on which pins are connected on the board.**

Following the steps inside the lib you should be able to fully control velocity or angle on a closed loop control for both methods.

#### Collecting data

[Colect serial](colect_serial.py) (UNTESTED) should read data from the serial and save them into a .json file for analisis purpose 

## References

### Main library:
- [Documentation](https://docs.simplefoc.com/)
    - [Monitoring](https://docs.simplefoc.com/monitoring)
    - [Commander interface](https://docs.simplefoc.com/commander_interface)
    - [Studio for testing](https://docs.simplefoc.com/studio)

-   [Torque control](https://docs.simplefoc.com/torque_control)

### Motor:
- [iPower Motor GM4108H (iFlight)](https://shop.iflight.com/gimbal-motors-cat44/ipower-motor-gm4108h-120t-gimbal-motor-with-as5048a-encoder-pro243)
    - [Aliexpress](https://www.aliexpress.us/item/2255801110080005.html?pdp_npi=4%40dis%21USD%21US%20%2464.99%21US%20%2450.29%21%21%2164.99%2150.29%21%402101effb17139667294248360e8ae2%2110000015655310335%21sh%21US%210%21&spm=a2g0o.store_pc_allItems_or_groupList.new_all_items_2007550626242.4001296394757&gatewayAdapt=glo2usa)
- [AS5048A Encoder Datasheet](https://br.mouser.com/datasheet/2/588/AS5048_DS000298_4_00-2324531.pdf)

### Driver:
- [EG2133 Chip](docs/EG2133%20(Translated).pdf)
- [MKS ESP32 FOC](https://www.aliexpress.us/item/3256805324882671.html?spm=a2g0o.store_pc_allItems_or_groupList.0.0.5ca92d15b7Iz9a&pdp_npi=4%40dis%21BRL%21R%24%20159%2C68%21R%24%20107%2C56%21%21%2130.74%2120.71%21%40216661c117096429890006834ebee1%2112000033361166879%21sh%21BR%210%21&gatewayAdapt=glo2usa4itemAdapt)
    - [lib ref](https://github.com/makerbase-motor/MKS-ESP32FOC/tree/master)

- [MKS DUAL FOC PLUS (shield version)](https://www.aliexpress.us/item/3256804875947998.html?pdp_npi=4%40dis%21BRL%21R%24%20133%2C03%21R%24%2096%2C08%21%21%2125.61%2118.50%21%40216661c117096429890006834ebee1%2112000033145077507%21sh%21BR%210%21&spm=a2g0o.store_pc_allItems_or_groupList.new_all_items_2007473458239.1005005062262750&gatewayAdapt=glo2usa4itemAdapt)
    - [lib ref](https://github.com/makerbase-motor/MKS-DUALFOC/tree/main)

### Unused

- [Optical incremental encoder (CUI AMT103)](https://br.mouser.com/ProductDetail/CUI-Devices/AMT103-V?qs=%2Fha2pyFaduiAsBlScvLoAWHUnKz39jAIpNPVt58AQ0PVb84dpbt53g%3D%3D)

- [Triple 1/2-H Bridge Driver (DRV8313)](https://www.ti.com/lit/ds/symlink/drv8313.pdf?ts=1704921368092)

- [Three-phase motor driver (L6234)](docs/l6234-3.pdf)


- Other option for drivers

    - [Texas Instruments overview](https://www.ti.com/motor-drivers/brushless-dc-bldc-drivers/overview.html)

    - [C2000](https://www.ti.com/video/series/c2000-mcus-motor-control.html)

### Tutorials:

ESP:

- [ESP-IDF with PlatformIO](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/third-party-tools/platformio.html)
- [Motor Control PWM](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html#application-examples)
- [Interruptions](https://www.makerhero.com/blog/uso-de-interrupcoes-externas-com-esp32/)
-[Interruptions alloc](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/intr_alloc.html)

Motor:
[BLDC Motor Commutation](https://www.youtube.com/watch?v=InzXA7mWBWE)

[Paper "Robô Holonômico Didático de Baixo Custo"](https://www.sba.org.br/cba2022/wp-content/uploads/artigos_cba2022/paper_776.pdf)