/**
 * @file bmp280_test.c
 * @brief BMP280 userspace test application for ioctl interface
 * @author FernandesKA
 * @date 2025-12-05
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include "bmp280.h"

#define DEVICE_PATH "/dev/bmp280"

static void print_usage(const char *prog)
{
	printf("Usage: %s [command]\n", prog);
	printf("Commands:\n");
	printf("  read        - Read temperature and pressure via read()\n");
	printf("  ioctl_read  - Read temperature and pressure via ioctl\n");
	printf("  get_config  - Get current device configuration\n");
	printf("  set_config  - Set device configuration (interactive)\n");
	printf("  chip_info   - Get chip information\n");
	printf("  reset       - Reset the device\n");
	printf("  monitor     - Continuous monitoring (Ctrl+C to stop)\n");
}

static int read_sensor_data(int fd)
{
	char buf[128];
	ssize_t ret;

	lseek(fd, 0, SEEK_SET);
	ret = read(fd, buf, sizeof(buf) - 1);
	if (ret < 0) {
		perror("read");
		return -1;
	}

	buf[ret] = '\0';
	printf("=== Sensor Data (via read()) ===\n");
	printf("%s", buf);
	return 0;
}

static int ioctl_read_data(int fd)
{
	struct bmp280_reading reading;
	int ret;

	ret = ioctl(fd, BMP280_IOC_READ_DATA, &reading);
	if (ret < 0) {
		perror("ioctl BMP280_IOC_READ_DATA");
		return -1;
	}

	printf("=== Sensor Data (via ioctl) ===\n");
	printf("Temperature: %d.%02d °C\n", reading.temperature / 100,
	       abs(reading.temperature % 100));
	printf("Pressure: %u Pa (%.2f hPa)\n", reading.pressure,
	       reading.pressure / 100.0);
	return 0;
}

static int get_config(int fd)
{
	struct bmp280_config config;
	int ret;

	ret = ioctl(fd, BMP280_IOC_GET_CONFIG, &config);
	if (ret < 0) {
		perror("ioctl BMP280_IOC_GET_CONFIG");
		return -1;
	}

	printf("=== Current Configuration ===\n");
	printf("Temperature oversampling: %u\n", config.temp_oversampling);
	printf("Pressure oversampling: %u\n", config.press_oversampling);
	printf("Filter coefficient: %u\n", config.filter_coeff);
	printf("Mode: %u ", config.mode);
	switch (config.mode) {
	case BMP280_MODE_SLEEP:
		printf("(SLEEP)\n");
		break;
	case BMP280_MODE_FORCED:
		printf("(FORCED)\n");
		break;
	case BMP280_MODE_NORMAL:
		printf("(NORMAL)\n");
		break;
	default:
		printf("(UNKNOWN)\n");
	}
	return 0;
}

static int set_config(int fd)
{
	struct bmp280_config config;
	int ret;

	printf("=== Set Configuration ===\n");
	printf("Temperature oversampling (0-5): ");
	ret = scanf("%hhu", &config.temp_oversampling);
    if (ret)
        return -EINVAL;
	printf("Pressure oversampling (0-5): ");
	ret = scanf("%hhu", &config.press_oversampling);
    if (ret)
        return -EINVAL;
	printf("Filter coefficient (0-4): ");
	ret = scanf("%hhu", &config.filter_coeff);
    if (ret)
        return -EINVAL;
	printf("Mode (0=SLEEP, 1=FORCED, 3=NORMAL): ");
	ret = scanf("%hhu", &config.mode);
    if (ret)
        return -EINVAL;

	ret = ioctl(fd, BMP280_IOC_SET_CONFIG, &config);
	if (ret < 0) {
		perror("ioctl BMP280_IOC_SET_CONFIG");
		return -1;
	}

	printf("Configuration updated successfully!\n");
	return 0;
}

static int get_chip_info(int fd)
{
	struct bmp280_chip_info info;
	int ret;

	ret = ioctl(fd, BMP280_IOC_GET_CHIP_INFO, &info);
	if (ret < 0) {
		perror("ioctl BMP280_IOC_GET_CHIP_INFO");
		return -1;
	}

	printf("=== Chip Information ===\n");
	printf("Chip ID: 0x%02X\n", info.chip_id);
	printf("Calibration T1: %u\n", info.calib_t1);
	printf("Calibration T2: %d\n", info.calib_t2);
	printf("Calibration T3: %d\n", info.calib_t3);
	return 0;
}

static int reset_device(int fd)
{
	int ret;

	printf("Resetting device...\n");
	ret = ioctl(fd, BMP280_IOC_RESET);
	if (ret < 0) {
		perror("ioctl BMP280_IOC_RESET");
		return -1;
	}

	printf("Device reset successfully!\n");
	return 0;
}

static int monitor_continuous(int fd)
{
	struct bmp280_reading reading;
	int ret;

	printf("=== Continuous Monitoring (Ctrl+C to stop) ===\n\n");

	while (1) {
		ret = ioctl(fd, BMP280_IOC_READ_DATA, &reading);
		if (ret < 0) {
			perror("ioctl BMP280_IOC_READ_DATA");
			return -1;
		}

		printf("\rTemp: %d.%02d °C | Press: %u Pa (%.2f hPa)   ",
		       reading.temperature / 100,
		       abs(reading.temperature % 100),
		       reading.pressure,
		       reading.pressure / 100.0);
		fflush(stdout);

		sleep(1);
	}

	return 0;
}

int main(int argc, char *argv[])
{
	int fd;
	int ret = 0;

	if (argc < 2) {
		print_usage(argv[0]);
		return 1;
	}

	fd = open(DEVICE_PATH, O_RDWR);
	if (fd < 0) {
		perror("open " DEVICE_PATH);
		return 1;
	}

	if (strcmp(argv[1], "read") == 0) {
		ret = read_sensor_data(fd);
	} else if (strcmp(argv[1], "ioctl_read") == 0) {
		ret = ioctl_read_data(fd);
	} else if (strcmp(argv[1], "get_config") == 0) {
		ret = get_config(fd);
	} else if (strcmp(argv[1], "set_config") == 0) {
		ret = set_config(fd);
	} else if (strcmp(argv[1], "chip_info") == 0) {
		ret = get_chip_info(fd);
	} else if (strcmp(argv[1], "reset") == 0) {
		ret = reset_device(fd);
	} else if (strcmp(argv[1], "monitor") == 0) {
		ret = monitor_continuous(fd);
	} else {
		printf("Unknown command: %s\n", argv[1]);
		print_usage(argv[0]);
		ret = 1;
	}

	close(fd);
	return ret;
}
