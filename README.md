Matter UDP Bridge for Non-Matter Devices
Experimental project — lightweight bridge for integrating non-Matter microcontrollers (ESP-based) into the Matter ecosystem using pure UDP communication.

Developed by G. Reyna
License: MIT

🧩 Overview

This project implements a Matter bridge designed for non-Matter devices (e.g. ESP01, ESP8266, ESP32 running legacy firmware).
The bridge runs on an ESP32 device using Espressif’s ESP-Matter SDK, acting as an aggregator node that dynamically discovers remote devices over UDP broadcast and exposes them as Matter endpoints.

Unlike the official Espressif examples, this implementation supports:

Dynamic endpoint creation and destruction

Automatic discovery via UDP broadcast

Reachability tracking

Persistent endpoint restoration using NVS

Real hardware integration (PWM-controlled MOSFET + AHT10 sensor)

Pure Matter stack, no MQTT or cloud dependencies

⚙️ Architecture

The project consists of three main modules:

File	Description
bridge.cpp/h	UDP discovery, command dispatching, dynamic endpoint creation, persistence, reachability handling.
wired.cpp/h	Local hardware control (PWM MOSFET, fade, I²C AHT10 temperature & humidity sensor).
app_main.cpp	Matter node initialization, aggregator setup, endpoint definitions, and event dispatching between modules.
Communication Flow
┌──────────────┐      UDP broadcast      ┌──────────────────┐
│   ESP01      │  ───────────────────▶  │  ESP32 Bridge     │
│  (non-Matter)│                        │  (Matter node)    │
│  ID="lamp1"  │  ◀───────────────────  │  Sends commands   │
└──────────────┘      UDP command        └──────────────────┘
                               │
                               ▼
                       Matter Controller
                     (Home Assistant / App)

UDP Protocol
Direction	Port	Payload	Purpose
ESP01 → Bridge	12345	<device_uid>	Discovery / heartbeat
Bridge → ESP01	12346	"ON" / "OFF" / other string payload	Remote control

Each ESP01 device periodically broadcasts its UID (e.g., "lamp01") to the bridge.
When a new UID is detected, the bridge creates a new OnOffPluginUnit endpoint dynamically, labeled with the same UID.

🏗️ Endpoint Structure

The bridge automatically creates and manages Matter endpoints:

Endpoint Type	Description	Creation
Dimmable Light	Local PWM output controlled via MOSFET	Static
Temperature Sensor	AHT10 I²C sensor (°C)	Static
Humidity Sensor	AHT10 I²C sensor (%)	Static
OnOffPluginUnit	Each discovered ESP01 device	Dynamic

Each dynamic endpoint includes:

OnOff cluster (server)

BridgedDeviceBasicInformation cluster

NodeLabel → device UID

Reachable → true/false based on UDP activity

Offline devices are marked unreachable after 60 seconds of inactivity and removed after 24 hours.

💾 Persistent Endpoint Mapping

The bridge saves the endpoint map in NVS under namespace "esp01" key "device_map".

Example saved string:

lamp1:3;fan2:4;


On reboot, restore_endpoints() rebuilds endpoints from this mapping, marking all as initially unreachable.

🧠 Attribute Update Flow

The Matter attribute update callback (app_attribute_update_cb) routes updates as follows:

Endpoint	Handler
Local wired device (PWM dimmer, sensors)	wired::handle_attribute_update()
Remote non-Matter device (UDP bridge)	bridge::handle_attribute_update()

When a Matter controller toggles a bridged device:

The bridge sends a UDP command (e.g. "ON", "OFF") to the corresponding ESP01 IP.

The ESP01 acts accordingly (e.g., toggles GPIO).

🌡️ Local Sensors
AHT10 Temperature & Humidity

Connected via I²C (GPIO1 = SDA, GPIO2 = SCL)

Measurement interval: 30 seconds (ATH10_READ_INTERVAL_MS)

Reports values to respective Matter clusters:

TemperatureMeasurement::MeasuredValue

RelativeHumidityMeasurement::MeasuredValue

MOSFET PWM Output

Controlled via LEDC (GPIO7)

Hardware fade enabled (2 seconds)

Last non-zero level remembered across updates

🔧 Building & Flashing
Requirements

ESP-IDF 5.1+

ESP-Matter SDK

CMake build system

Steps
idf.py set-target esp32
idf.py build
idf.py flash monitor


After flashing:

Commission the bridge into a Matter network using chip-tool or Home Assistant.

Power on your ESP01 devices broadcasting their UID on port 12345.

The bridge will auto-create endpoints visible in your Matter controller.

⚠️ Limitations

UDP protocol is plaintext / no authentication (experimental only).

Not intended for production or secure environments.

Each ESP01 must have a unique UID string.

Bridge currently supports OnOff only; LevelControl and other clusters are planned.

🧭 Roadmap

 Add LevelControl cluster for remote dimmers

 Implement heartbeat encryption (optional key exchange)

 Add MQTT optional backend for cross-protocol bridging

 Web UI for status visualization

 OTA support for bridged devices

🧱 Repository Structure
/main
 ├── app_main.cpp
 ├── bridge.cpp
 ├── bridge.h
 ├── wired.cpp
 ├── wired.h
 └── CMakeLists.txt

📜 License
MIT License

Copyright (c) 2025 G. Reyna

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the “Software”), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

[...standard MIT text continues...]

💬 Notes

This is an independent, community experiment, not affiliated with Espressif or the Matter Working Group.
The goal is to explore lightweight bridges that make legacy devices interoperable with the Matter ecosystem —
without relying on heavy frameworks or cloud infrastructure.