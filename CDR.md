
# TITAN Flight Computer Software CDR
This file serves to provide links for code files for those attending the CDR to help them follow along. The files are broken down into 4 categories: [System Design](#system-design), [Core Drivers](#core-drivers), [Device Drivers](#device-drivers), and [Testing Code](#testing-code). Enjoy.
  
### System Design
- Startup  
  - [startup.c](https://github.com/SARP-UW/titan/blob/main/src/internal/startup.c)  
- Allocator  
  - [alloc.h](https://github.com/SARP-UW/titan/blob/main/src/internal/alloc.h)  
  - [alloc.c](https://github.com/SARP-UW/titan/blob/main/src/internal/alloc.c)
  - [allocator unit test](https://github.com/SARP-UW/titan/blob/main/test/test_alloc.c)  
- State Machine  
  - [state machine folder](https://github.com/SARP-UW/titan/tree/main/src/app)  
- Main Loop  
  - [main.c](https://github.com/SARP-UW/titan/blob/main/src/main.c)  
  
### Core Drivers
- GPIO  
  - [gpio.h](https://github.com/SARP-UW/titan/blob/main/src/peripheral/gpio.h)  
  - [gpio.c](https://github.com/SARP-UW/titan/blob/main/src/peripheral/gpio.c)  
- MMIO  
  - [mmio.h](https://github.com/SARP-UW/titan/blob/main/src/internal/mmio.h)  
  - [mmio.c](https://github.com/SARP-UW/titan/blob/main/src/internal/mmio.c)  
- UART/USART  
  - [uart.h](https://github.com/SARP-UW/titan/blob/main/src/peripheral/uart.h)  
  - [uart.c](https://github.com/SARP-UW/titan/blob/main/src/peripheral/uart.c)  
- SPI  
  - [spi.h](https://github.com/SARP-UW/titan/blob/main/src/peripheral/spi.h)  
  - [spi.c](https://github.com/SARP-UW/titan/blob/main/src/peripheral/spi.c)  
- QSPI  
  - [qspi.h](https://github.com/SARP-UW/titan/blob/main/src/peripheral/qspi.h)  
  - [qspi.c](https://github.com/SARP-UW/titan/blob/main/src/peripheral/qspi.c)  
- PWM  
  - [pwm.h](https://github.com/SARP-UW/titan/blob/main/src/peripheral/pwm.h)  
  - [pwm.c](https://github.com/SARP-UW/titan/blob/main/src/peripheral/pwm.c)  
- Watchdog  
  - [watchdog.h](https://github.com/SARP-UW/titan/blob/main/src/peripheral/watchdog.h)  
  - [watchdog.c](https://github.com/SARP-UW/titan/blob/main/src/peripheral/watchdog.c)  
  
### Device Drivers
- Barometer  
  - [barometer.h](https://github.com/SARP-UW/titan/blob/main/src/devices/barometer.h)  
  - [barometer.c](https://github.com/SARP-UW/titan/blob/main/src/devices/barometer.c)  
- Magnetometer  
  - Todo  
- IMU  
  - Todo  
- Actuator  
  - [actuator.h](https://github.com/SARP-UW/titan/blob/main/src/peripheral/actuator.h)  
  - [actuator.c](https://github.com/SARP-UW/titan/blob/main/src/peripheral/actuator.c)  
- Radio  
  - Todo  
- Temperature Sensor  
  - Todo
- GNSS  
  - Todo

 ### Testing Code
  - [test_alloc.c](https://github.com/SARP-UW/titan/blob/main/test/test_alloc.c) - Unit tests
  - [test_oscilloscope.c](https://github.com/SARP-UW/titan/blob/main/test/test_oscilloscope.c)
  - [test_pwm.c](https://github.com/SARP-UW/titan/blob/main/test/test_pwm.c)
  - [test_spi.c](https://github.com/SARP-UW/titan/blob/main/test/test_spi.c)
  - [test_usart.c](https://github.com/SARP-UW/titan/blob/main/test/test_usart.c)

