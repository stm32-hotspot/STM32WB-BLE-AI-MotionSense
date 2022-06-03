# STM32-Hotspot/STM32WB-BLE-AI-MotionSense MCU Firmware example

![latest tag](https://img.shields.io/github/v/tag/STMicroelectronics/STM32CubeWB.svg?color=brightgreen)

## Example
This Hotspot FW package includes:
* Application example called STM32WB-BLE-AI-MotionSense.     
   * This example aims to demonstrates how to create a motion sensing application to recognize human activities using machine learning on an STM32WB microcontroller
   * The model used classifies 3 activities: stationary, walking, or running using the motion sensor.
   * You can create your own model using your own data captures and test your model using this project.
   * Additional ST middlewares: [X-CUBE-AI](https://www.st.com/en/embedded-software/x-cube-ai.html) v7.1.0 and [X-CUBE-MEMS1](https://www.st.com/en/embedded-software/x-cube-mems1.html) v1.9.0

![AI0](Utilities/Media/Images/Users_Guide/AI0.png)
![AI1](Utilities/Media/Images/Users_Guide/AI1.gif)

* PC tools requirement
   * [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) v6.5.0
   * [STM32CubeProg](https://www.st.com/en/development-tools/stm32cubeprog.html) v2.10.0
   * Supported IDE toolchains/compilers: IAR EWARM V9.20.x, Keil µVision v5.36.0, STM32CubeIDE v1.9.0

## Library license agreement from STM32CubeMX
This example uses X-CUBE-AI's Network runtime library and X-CUEB-MEMS1 library.
To use these libraries, use the STM32CubeMX to generate the libraries and agree to the libraries' license agreement shown below.

**You must go through this step to build this firmware example.**
![Agree X-CUBE-AI](Utilities/Media/Images/Users_Guide/AI-Lic.gif)
![Agree X-CUEB-MEMS1](Utilities/Media/Images/Users_Guide/MEMS1-Lic.gif)

The next step is to generate these library files via STM32CubeMX. This is required to build this example project.

First, re-select the Keras model file (model.h5) in the Pinout & Configuration tab / X-CUBE-AI's network menu.
![Model](Utilities/Media/Images/Users_Guide/re-select.png)

Next, re-generate a project after selecting your preferred IDE toolchain. As shown in this screenshot, select the IDE and generate the project.
![RE-generate](Utilities/Media/Images/Users_Guide/re-gen.png)


## Hardware Needed
* One [STM32WB5MM-DK](https://www.st.com/en/evaluation-tools/stm32wb5mm-dk.html)

* iOS or Android Smartphone supporting Bluetooth LE 4.x
  
## Software Needed
* Prebuilt firmware image, STM32WB-BLE-AI-MotionSense.hex, provided under "Binary/STM32WB-BLE-AI-MotionSense.hex"

* Smartphone app - ST BLE Sensor
   * [ST BLE Sensor iOS](https://apps.apple.com/us/app/st-ble-sensor/id993670214)
   * [ST BLE Sensor Android](https://play.google.com/store/apps/details?id=com.st.bluems&hl=en_US&gl=US)

## User's Guide
* Install the ST BLE Sensor app on a smarphone
* [Install ST's Bluetooth stack](https://youtu.be/wheGvdXsi4o), found under "STM32WB_Copro_Wireless_Binaries\STM32WB5x"
* To quickly test the application, unstall the prebuilt hex firmware image, "Binary/BLE_AI_MotionSense.hex"
* Or run and debug the firmware on the STM32WB5MM-DK board by building it using the IDE toolchain
* (OPTIONAL) Connect a serial terminal (Tera Term) to view the log messages.

![UG0](Utilities/Media/Images/Users_Guide/UG0.jpg)

* [0] Power the STM32WB5MM-DK board by connecting a micro USB cable to CN11 (USB STLK)
* [1] Launch the ST BLE Sensor app. Click the "Connect to a device" icon to start scanning.
* [2] You should see a device appear named AI-Motion. Tap the discovered device.
* [3] You will see the "Activity Recognition" screen. The "Activity detection started" message should show up so that the STM32WB starts sending the motion data to this app.

![UG1](Utilities/Media/Images/Users_Guide/UG1.jpg)

## AI model

This project used a pre-trained Keras model (model.h5 file), trained using a small dataset created specifically for this example.
The STM32WB5MM-DK board has a 3D accelerometer & 3D gyroscope sensor ISM330DHCX. This accelerometer will be used for the model's input data.
![AI2](Utilities/Media/Images/Users_Guide/AI2.jpg)

* 78 accelerometer data are used as the input for this model.
* 3 output data are generated from this model where one of the highest scored output data will be the predicted motion.

You can create your own model using your own data captures. If using a different model, make sure to adjust the  sensor settings (data rate, scale, etc) accordingly to match the training dataset.


**Caution** : Issues and the pull-requests are **not supported** to submit problems or suggestions related to the software delivered in this repository. This example is being delivered as-is, and not necessarily supported by ST. No major development activities are planned for this example. Check the RELEASE NOTE to see the known limitations.

**For any other question** related to the product, the hardware performance or characteristics, the tools, the environment, you can submit it to the **ST Community** on the STM32 MCUs related [page](https://community.st.com/s/topic/0TO0X000000BSqSWAW/stm32-mcus).