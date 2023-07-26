# MQTT Communication Library for A7672SA (SIMCOM) and ESP32 (Espressif)

![Logo](https://img.shields.io/badge/Library-MQTT_A7672SA-<)
![GitHub repo size](https://img.shields.io/github/repo-size/giovannirosso/MQTT_A7672SA)
[![License](https://img.shields.io/badge/license-LGPL3-blue.svg)](https://github.com/giovannirosso/MQTT_A7672SA/blob/main/LICENSE)
![GitHub stars](https://img.shields.io/github/stars/giovannirosso/MQTT_A7672SA)
![GitHub forks](https://img.shields.io/github/forks/giovannirosso/MQTT_A7672SA)
![GitHub issues](https://img.shields.io/github/issues/giovannirosso/MQTT_A7672SA)
![GitHub pull requests](https://img.shields.io/github/issues-pr/giovannirosso/MQTT_A7672SA)
![GitHub contributors](https://img.shields.io/github/contributors/giovannirosso/MQTT_A7672SA)
![GitHub last commit](https://img.shields.io/github/last-commit/giovannirosso/MQTT_A7672SA)
![GitHub commit activity](https://img.shields.io/github/commit-activity/m/giovannirosso/MQTT_A7672SA)
![GitHub top language](https://img.shields.io/github/languages/top/giovannirosso/MQTT_A7672SA)

![A7672SA](extras/A7672SA.png)

## Description

This C++ library provides an easy-to-use interface for establishing MQTT communication between the A7672SA module from SIMCOM and the ESP32 board from Espressif. The library simplifies the process of setting up an MQTT client on both devices, enabling seamless data exchange over the MQTT protocol.

## Features

- Simple and lightweight MQTT communication between A7672SA and ESP32.
- Easy integration with existing projects.
- Supports both subscribe and publish functionalities.
- Adjustable QoS (Quality of Service) levels for message delivery.
- Customizable MQTT broker configurations.

## Requirements

- A7672SA module from SIMCOM.
- ESP32 board from Espressif.
- PlatformIO IDE for building and uploading the library.
- Internet connectivity for MQTT communication with a broker.

## Installation

1. Clone the repository: `git clone https://github.com/giovannirosso/MQTT_A7672SA.git`
2. In your PlatformIO project, include the library:
   - Add `#include <MQTT_A7672SA.h>` to your main sketch.
