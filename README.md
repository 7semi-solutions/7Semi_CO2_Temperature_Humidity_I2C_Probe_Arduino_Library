# 7Semi_CO2_Temperature_Humidity_I2C_Probe_Arduino_Library
This Arduino library provides support for the **7Semi CO₂, Temperature &amp; Humidity I2C Sensor Probe** — a reliable, compact, and low-power environmental sensor for embedded and IoT systems. It communicates over I²C and outputs real-time measurements of carbon dioxide (CO₂ in ppm), temperature (°C), and relative humidity (%RH).


![Arduino](https://img.shields.io/badge/platform-arduino-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-active-brightgreen.svg)

---

## 🌐 Features

- Supports I²C communication at standard 100kHz
- CO₂ measurement in parts per million (ppm)
- Temperature measurement in degrees Celsius (°C)
- Humidity measurement in %RH
- Works with 3.3V.
- Lightweight and optimized for AVR, STM32, ESP32/ESP8266, and RP2040

---

## 🔧 Hardware Required

- 7Semi CO₂ Temperature & Humidity I2C Probe  
- Arduino-compatible development board  
- I²C connections: SDA, SCL  
- Power supply: 3.3V

---

## 🚀 Getting Started

### 1. Installation via Arduino Library Manager

1. Open the **Arduino IDE**
2. Navigate to:
   - `Sketch > Include Library > Manage Libraries…` (IDE 1.x), or  
   - Use the **Library Manager** sidebar in IDE 2.x
3. Search for: `7Semi_CO2_Temperature_Humidity_I2C_Probe_Arduino_Library`
4. Click **Install**

Alternatively, clone or download this repository and place it in your `libraries/` folder.

---

### 2. I²C Wiring

| CO2TH Sensor | Arduino Board |
|--------------|---------------|
| SDA          | A4 (UNO) / D21 (ESP32) |
| SCL          | A5 (UNO) / D22 (ESP32) |
| VCC          | 3.3V      |
| GND          | GND           |



---

## 📟 Example Output

```plaintext
CO2: 415 ppm    Temperature: 25.37 °C    Humidity: 45.63 %RH
CO2: 417 ppm    Temperature: 25.42 °C    Humidity: 45.59 %RH
