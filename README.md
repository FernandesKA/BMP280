# BMP280 Linux Kernel Driver

A character device driver for the Bosch BMP280 barometric pressure and temperature sensor with I2C interface support. This project demonstrates kernel module development, character device implementation, and ioctl-based userspace communication.

## Hardware Requirements

- BMP280 sensor connected via I2C
- Linux system with I2C bus support

Available ioctl commands:
- `BMP280_IOC_READ_DATA` - Read temperature and pressure
- `BMP280_IOC_GET_CONFIG` - Get current sensor configuration
- `BMP280_IOC_SET_CONFIG` - Configure oversampling and filter settings
- `BMP280_IOC_GET_CHIP_INFO` - Read chip ID and calibration data
- `BMP280_IOC_RESET` - Software reset

## Project Structure


module:
 - bmp280.c # Main driver implementation
 - bmp280.h # Register definitions and ioctl structures
 - Makefile # Kernel module build
userspace:
 - bmp280_test.c # Test application
 - bmp280.h # Shared header
 - Makefile # Userspace build
dts: # Device tree examples

## Author

**FernandesKA** ([i@kfernandes.ru](mailto:i@kfernandes.ru))

## License

GPL - See module source for details
