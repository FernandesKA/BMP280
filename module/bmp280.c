/**
 * @file bmp280.c
 * @author FernandesKA (i@kfernandes.ru)
 * @brief  simplest bmp280 driver for study i2c client driver writing
 * @version 0.1
 * @date 2025-12-02
 *
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include "bmp280.h"

struct bmp280_data
{
    struct i2c_client *client;
};

static int bmp280_probe(struct i2c_client *client)
{
    struct bmp280_data *data;

    if (!client)
        return -ENODEV;

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    i2c_set_clientdata(client, (void *)data);

    dev_info(&client->dev, "BMP280 probed ok\n");
    return 0;
}

static void bmp280_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "BMP280 sensor has been removed\n");
}

static int bmp280_read_register(struct i2c_client *client, u8 reg)
{
    return i2c_smbus_read_byte_data(client, reg);
}

static int bmp280_write_register(struct i2c_client *client, u8 reg, u16 value)
{
    return i2c_smbus_write_byte_data(client, reg, value);
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
