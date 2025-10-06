/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * CH45x I2C Multi-Function Device Driver (LED + Keypad)
 *
 * Author:      Sergey Kharenko <skharenko@hust.edu.cn>
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define CH45X_CELL_LED     0
#define CH45X_CELL_KEYPAD  1

static const struct mfd_cell ch45x_cells[] = {
    {
        .name = "ch45x-led",
        .id   = CH45X_CELL_LED,
        .of_compatible = "wch,ch45x-led",
    },
    {
        .name = "ch45x-keypad",
        .id   = CH45X_CELL_KEYPAD,
        .of_compatible = "wch,ch45x-keypad",
    },
};

struct ch45x_dev {
    struct device *dev;
    struct i2c_client *client;
    // 可以加锁、寄存器缓存、IRQ等
};

static int ch45x_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
    struct ch45x_dev *chip;
    int ret;

    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    chip->dev = &client->dev;
    chip->client = client;
    i2c_set_clientdata(client, chip);

    // TODO: 初始化芯片寄存器，例如复位、模式配置
    // ch45x_init_chip(chip);

    ret = mfd_add_devices(chip->dev, -1, ch45x_cells,
                          ARRAY_SIZE(ch45x_cells), NULL, 0, NULL);
    if (ret) {
        dev_err(&client->dev, "failed to add mfd sub-devices: %d\n", ret);
        return ret;
    }

    dev_info(&client->dev, "CH45x I2C MFD driver probed\n");
    return 0;
}

static int ch45x_remove(struct i2c_client *client)
{
    struct ch45x_dev *chip = i2c_get_clientdata(client);

    mfd_remove_devices(chip->dev);
    dev_info(&client->dev, "CH45x I2C MFD driver removed\n");
    return 0;
}

static const struct of_device_id ch45x_of_match[] = {
    { .compatible = "wch,ch450"},
    { }
};
MODULE_DEVICE_TABLE(of, ch45x_of_match);

static const struct i2c_device_id ch45x_id[] = {
    { "ch450", 0 },
    { "ch452", 1 },
    { "ch453", 2 },
    { "ch454", 3 },
    { "ch455", 4 },
    { "ch456", 5 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ch45x_id);

static struct i2c_driver ch45x_driver = {
    .driver = {
        .name = "ch45x",
        .of_match_table = ch45x_of_match,
    },
    .probe = ch45x_probe,
    .remove = ch45x_remove,
    .id_table = ch45x_id,
};

module_i2c_driver(ch45x_driver);

MODULE_AUTHOR("Sergey Kharenko <skharenko@hust.edu.cn>");
MODULE_DESCRIPTION("CH45x Multi-Function Device Driver (LED + Keypad)");
MODULE_VERSION("0.1.0");
MODULE_LICENSE("GPL");
