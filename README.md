# STM32F446RE - MPU9250 & BME280 Sensor Interface

This project demonstrates how to interface an **MPU9250 9-DOF IMU sensor** and a **BME280 environmental sensor** with the **STM32F446RE (NUCLEO-F446RE board)** using the **STM32CubeIDE** and **HAL drivers**.

---

## 📌 Features
- STM32F446RE microcontroller (ARM Cortex-M4, 180 MHz, FPU)
- **MPU9250**:  
  - 3-axis accelerometer  
  - 3-axis gyroscope  
  - 3-axis magnetometer (AK8963 inside)  
- **BME280**:  
  - Temperature  
  - Humidity  
  - Barometric pressure  
- Communication via **I2C1**
- CubeMX-generated project structure with HAL-based drivers
- Data output via **UART (printf redirection)**

---

## 📂 Project Structure
```
005_MPU9250_BME/
│── Core/
│   ├── Inc/                # Header files (main.h, ISR headers)
│   ├── Src/                # Application sources (main.c, startup code)
│   └── Startup/            # Assembly startup file
│
│── Drivers/
│   ├── CMSIS/              # ARM Cortex-M core headers
│   └── STM32F4xx_HAL_Driver/ # STM32 HAL libraries
│
│── 005_MPU9250_NUCLEO.ioc  # STM32CubeMX configuration
│── STM32F446RETX_FLASH.ld  # Linker script for Flash
│── STM32F446RETX_RAM.ld    # Linker script for RAM
```

---

## ⚙️ Configuration
- **MCU**: STM32F446RE (180 MHz system clock)
- **IDE**: STM32CubeIDE 1.13+  
- **Communication**:  
  - MPU9250 via **I2C1 @ 400 kHz**  
  - BME280 via **I2C1 @ 400 kHz** (shared bus)  
- **UART2**: Debug output (115200 baud)

---

## 🔬 Sensor Technical Details

### 📌 MPU9250 (9-DOF IMU)
- **Accelerometer ranges**: ±2g, ±4g, ±8g, ±16g  
  - Resolution: 16-bit  
  - Register: `ACCEL_CONFIG` (0x1C)  
- **Gyroscope ranges**: ±250, ±500, ±1000, ±2000 °/s  
  - Resolution: 16-bit  
  - Register: `GYRO_CONFIG` (0x1B)  
- **Magnetometer (AK8963)**: ±4800 µT  
  - Resolution: 16-bit  
  - Register: `CNTL1` (0x0A)  
- **Sample Rate Divider**: `SMPLRT_DIV` (0x19)  
  - `Fs = Gyro_Output_Rate / (1 + SMPLRT_DIV)`  
- **Calibration**:  
  - Accelerometer/Gyroscope bias can be estimated by averaging data at rest.  
  - Magnetometer requires **hard-iron/soft-iron calibration**.  

### 📌 BME280 (Environmental Sensor)
- **Temperature**:  
  - Range: -40 °C to +85 °C  
  - Resolution: 0.01 °C  
  - Typical accuracy: ±1.0 °C  
- **Pressure**:  
  - Range: 300 hPa to 1100 hPa  
  - Resolution: 0.18 Pa  
  - Accuracy: ±1 hPa  
- **Humidity**:  
  - Range: 0% to 100% RH  
  - Resolution: 0.008% RH  
  - Accuracy: ±3% RH  
- **Registers**:  
  - `CTRL_HUM` (0xF2) → Humidity oversampling  
  - `CTRL_MEAS` (0xF4) → Temperature & pressure oversampling + mode  
  - `CONFIG` (0xF5) → Standby time, filter settings  
- **Calibration**:  
  - Factory calibration coefficients stored in non-volatile memory (`0x88–0xA1`, `0xE1–0xE7`)  
  - Must be applied to raw ADC values using compensation formulas from datasheet.  

---

## 🚀 Getting Started
1. Clone the repository:
   ```bash
   git clone https://github.com/bymrt/STM32_BME_Projects.git
   ```
2. Open **STM32CubeIDE** → *Import Existing Project*.
3. Connect STM32 Nucleo-F446RE via **ST-LINK**.
4. Build & Flash the firmware.
5. Open **UART terminal @115200 baud** to view sensor data.

---

## 📊 Output Example
```text
MPU9250:
Accel: X=0.02g, Y=-0.01g, Z=0.99g
Gyro : X=0.2°/s, Y=0.1°/s, Z=-0.3°/s
Mag  : X=120µT, Y=-80µT, Z=45µT

BME280:
Temp : 24.56 °C
Press: 1013.25 hPa
Hum  : 41.2 %RH
```

---

## 🔧 Future Improvements
- Implement **sensor fusion (Madgwick/Mahony filter)** for orientation.
- Integrate **FreeRTOS** for sensor task management.
- Add **SPI interface** option for higher speed.
- Store sensor data on **SD card** or transmit via **BLE/Wi-Fi**.

---

## 📝 License
This project is released under the **MIT License**.  
You are free to use, modify, and distribute.
