## SafeStep
# Project Details
SafeStep is currently a developing low-cost, portable, IoT-based device that operates on Wi-Fi. This device is meant to assist caregivers of older individuals with caregiving. Featuring fall detection with an email alert system, step tracking, and environemental monitoring, caregivers, especially those whom may have several duties, will have some peace of mind when needed to step away.

*Hardware Components:*
  + x1 Lilygo T-Display
  + x1 MPU6050 Accelerometer & Gyroscope
  + x1 BME280 Temperature & Humidity
  + x1 MAX30102 SPO2 & Oxygen
  + x1 button

# Setup
To get started, have a Microsoft [Azure](portal.azure.com) account, create a resource group, iot hub, event hub namespace, and event hub instance. You will also need to create a cluster and database on Microsoft's [Azure Data Explorer](dataexplorer.azure.com). On Azure, open a terminal and run 'openssl s_client -showcerts -connect [your hub name].azure-devices.net:443 in azure terminal'. Scroll up to the most recent root certificate value and copy its value.

Follow this [link](https://learn.microsoft.com/en-us/azure/iot/howto-use-iot-explorer) and start up the explorer. After adding your device, create a SAS token and replace the SAS_TOKEN value in main.cpp with this point to the end: 'Signature sr=...'. There will be more TODO items in this file, search through each instance and complete it.

When you clone this repository, you will need to create a secrets.h file, and fill out this template:

(`
#ifndef SECRETS_H
#define SECRETS_H

#define WIFI_SSID       ""
#define WIFI_PASSWORD   ""

#define AUTHOR_EMAIL    ""
#define AUTHOR_PASSWORD ""

#define RECIPIENT_EMAIL ""

const char* root_ca =
#endif
`)

If using Gmail, an app password will need to be created. Use this app password in place of the author_password.

