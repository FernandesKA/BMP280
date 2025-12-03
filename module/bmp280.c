/**
 * @file bmp280.c
 * @author FernandesKA (i@kfernandes.ru)
 * @brief  simplest bmp280 driver for study i2c client driver writing
 * @version 0.1
 * @date 2025-12-02
 *
 *
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/math64.h> 

#include "bmp280.h"

struct bmp280_calib_data
{
    u16 dig_t1;
    s16 dig_t2;
    s16 dig_t3;
    u16 dig_p1;
    s16 dig_p2;
    s16 dig_p3;
    s16 dig_p4;
    s16 dig_p5;
    s16 dig_p6;
    s16 dig_p7;
    s16 dig_p8;
    s16 dig_p9;
    s32 t_fine;
};

struct bmp280_data
{
    struct i2c_client *client;
    struct bmp280_calib_data calib;
};

static int bmp280_read_calibration_data(struct bmp280_data *data)
{
    struct i2c_client *client = data->client;
    u8 calib_data[BMP280_CONTIGUOUS_CALIB_REGS];
    int ret;
    
    ret = i2c_smbus_read_i2c_block_data(client, BMP280_REG_COMP_TEMP_START,
                                        BMP280_CONTIGUOUS_CALIB_REGS, 
                                        calib_data);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read calibration data\n");
        return ret;
    }
    
    data->calib.dig_t1 = (calib_data[1] << 8) | calib_data[0];
    data->calib.dig_t2 = (s16)((calib_data[3] << 8) | calib_data[2]);
    data->calib.dig_t3 = (s16)((calib_data[5] << 8) | calib_data[4]);
    
    data->calib.dig_p1 = (calib_data[7] << 8) | calib_data[6];
    data->calib.dig_p2 = (s16)((calib_data[9] << 8) | calib_data[8]);
    data->calib.dig_p3 = (s16)((calib_data[11] << 8) | calib_data[10]);
    data->calib.dig_p4 = (s16)((calib_data[13] << 8) | calib_data[12]);
    data->calib.dig_p5 = (s16)((calib_data[15] << 8) | calib_data[14]);
    data->calib.dig_p6 = (s16)((calib_data[17] << 8) | calib_data[16]);
    data->calib.dig_p7 = (s16)((calib_data[19] << 8) | calib_data[18]);
    data->calib.dig_p8 = (s16)((calib_data[21] << 8) | calib_data[20]);
    data->calib.dig_p9 = (s16)((calib_data[23] << 8) | calib_data[22]);
    
    return 0;
}

static int bmp280_chip_config(struct bmp280_data *data)
{
    struct i2c_client *client = data->client;
    u8 ctrl_meas, config;
    int ret;
    
    ctrl_meas = (BMP280_OSRS_TEMP_16X << 5) | 
                (BMP280_OSRS_PRESS_16X << 2) | 
                BMP280_MODE_NORMAL;
    
    ret = i2c_smbus_write_byte_data(client, BMP280_REG_CTRL_MEAS, ctrl_meas);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to write ctrl_meas\n");
        return ret;
    }
    
    config = (BMP280_FILTER_16X << 2) | (0 << 5);
    
    ret = i2c_smbus_write_byte_data(client, BMP280_REG_CONFIG, config);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to write config\n");
        return ret;
    }
    
    return 0;
}

static int bmp280_read_raw_temp(struct bmp280_data *data, s32 *temp)
{
    struct i2c_client *client = data->client;
    u8 buf[3];
    int ret;
    
    ret = i2c_smbus_read_i2c_block_data(client, BMP280_REG_TEMP_MSB, 3, buf);
    if (ret < 0)
        return ret;
    
    *temp = ((u32)buf[0] << 12) | ((u32)buf[1] << 4) | ((u32)buf[2] >> 4);
    return 0;
}

static int bmp280_read_raw_press(struct bmp280_data *data, s32 *press)
{
    struct i2c_client *client = data->client;
    u8 buf[3];
    int ret;
    
    ret = i2c_smbus_read_i2c_block_data(client, BMP280_REG_PRESS_MSB, 3, buf);
    if (ret < 0)
        return ret;
    
    *press = ((u32)buf[0] << 12) | ((u32)buf[1] << 4) | ((u32)buf[2] >> 4);
    return 0;
}

static s32 bmp280_compensate_temp(struct bmp280_data *data, s32 adc_temp)
{
    struct bmp280_calib_data *calib = &data->calib;
    s32 var1, var2, T;
    
    var1 = ((((adc_temp >> 3) - ((s32)calib->dig_t1 << 1))) * 
            ((s32)calib->dig_t2)) >> 11;
    var2 = (((((adc_temp >> 4) - ((s32)calib->dig_t1)) * 
              ((adc_temp >> 4) - ((s32)calib->dig_t1))) >> 12) * 
            ((s32)calib->dig_t3)) >> 14;
    
    calib->t_fine = var1 + var2;
    T = (calib->t_fine * 5 + 128) >> 8;
    
    return T;
}

static u32 bmp280_compensate_press(struct bmp280_data *data, s32 adc_press)
{
    struct bmp280_calib_data *calib = &data->calib;
    s64 var1, var2, p;
    
    var1 = ((s64)calib->t_fine) - 128000;
    var2 = var1 * var1 * (s64)calib->dig_p6;
    var2 = var2 + ((var1 * (s64)calib->dig_p5) << 17);
    var2 = var2 + (((s64)calib->dig_p4) << 35);
    var1 = ((var1 * var1 * (s64)calib->dig_p3) >> 8) +
           ((var1 * (s64)calib->dig_p2) << 12);
    var1 = (((((s64)1) << 47) + var1)) * ((s64)calib->dig_p1) >> 33;
    
    if (var1 == 0)
        return 0;
    
    p = 1048576 - adc_press;
    
    p = div_s64((((p << 31) - var2) * 3125), var1);
    
    var1 = (((s64)calib->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((s64)calib->dig_p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((s64)calib->dig_p7) << 4);
    
    return (u32)p;
}

static ssize_t temperature_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bmp280_data *data = i2c_get_clientdata(client);
    s32 adc_temp, temp;
    int ret;
    
    ret = bmp280_read_raw_temp(data, &adc_temp);
    if (ret < 0)
        return ret;
    
    temp = bmp280_compensate_temp(data, adc_temp);
    
    return sprintf(buf, "%d.%02d\n", temp / 100, temp % 100);
}

static ssize_t pressure_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bmp280_data *data = i2c_get_clientdata(client);
    s32 adc_temp, adc_press;
    u32 press;
    int ret;
    
    ret = bmp280_read_raw_temp(data, &adc_temp);
    if (ret < 0)
        return ret;
    bmp280_compensate_temp(data, adc_temp);
    
    ret = bmp280_read_raw_press(data, &adc_press);
    if (ret < 0)
        return ret;
    
    press = bmp280_compensate_press(data, adc_press);
    
    return sprintf(buf, "%u\n", press >> 8); /* Pa */
}

static DEVICE_ATTR_RO(temperature);
static DEVICE_ATTR_RO(pressure);

static struct attribute *bmp280_attrs[] = {
    &dev_attr_temperature.attr,
    &dev_attr_pressure.attr,
    NULL,
};

static const struct attribute_group bmp280_attr_group = {
    .attrs = bmp280_attrs,
};


static int bmp280_probe(struct i2c_client *client)
{
    struct bmp280_data *data;
    int ret, chip_id;

    if (!client)
        return -ENODEV;

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    i2c_set_clientdata(client, data);

    chip_id = i2c_smbus_read_byte_data(client, BMP280_REG_CHIP_ID);
    if (chip_id < 0)
    {
        dev_err(&client->dev, "Failed to read chip ID\n");
        return chip_id;
    }

    if (chip_id != 0x58)
    {
        dev_err(&client->dev, "Unexpected chip ID: 0x%02x\n", chip_id);
        return -ENODEV;
    }

    ret = bmp280_read_calibration_data(data);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to read calibration data\n");
        return ret;
    }

    ret = bmp280_chip_config(data);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to configure BMP280\n");
        return ret;
    }

    ret = sysfs_create_group(&client->dev.kobj, &bmp280_attr_group);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to create sysfs group\n");
        return ret;
    }

    dev_info(&client->dev, "BMP280 probed ok\n");
    return 0;
}

static void bmp280_remove(struct i2c_client *client)
{
    sysfs_remove_group(&client->dev.kobj, &bmp280_attr_group);
    dev_info(&client->dev, "BMP280 sensor has been removed\n");
}

static struct i2c_device_id bmp280_idtable[] = {
    {"bmp280", 0},
    {}};
MODULE_DEVICE_TABLE(i2c, bmp280_idtable);

static const struct of_device_id bmp280_of_match[] = {
    {.compatible = "my,bmp280"},
    {}};
MODULE_DEVICE_TABLE(of, bmp280_of_match);

static struct i2c_driver bmp280_driver = {
    .driver = {
        .name = "bmp280",
        .of_match_table = bmp280_of_match,
    },
    .id_table = bmp280_idtable,
    .probe = bmp280_probe,
    .remove = bmp280_remove,
};

static int __init bmp280_init(void)
{
    return i2c_add_driver(&bmp280_driver);
}
module_init(bmp280_init);

static void __exit bmp280_cleanup(void)
{
    i2c_del_driver(&bmp280_driver);
}
module_exit(bmp280_cleanup);

MODULE_AUTHOR("FernandesKA fernandes.kir@yandex.ru");
MODULE_DESCRIPTION("bmp280 driver");
MODULE_LICENSE("GPL");
