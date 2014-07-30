/* drivers/input/misc/cm3218.c - cm3218 Ambient Light Sensor driver
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include "../iio.h"
#include "../sysfs.h"


#define CM3218_REG_CONFIGURE		0x00
#define CM3218_REG_ALS_DATA		0x04
#define CM3218_REG_MAX			0x04


#define CONFIGURE_ALS_MASK		0x01
#define CONFIGURE_ALS_EN		0x00

#define CONFIGURE_SHDN_MASK		CONFIGURE_ALS_MASK
#define CONFIGURE_SHDN_EN		0x01

#define LS_POLLING_DELAY	1000 /* mSec */
#define I2C_MAX_TIMEOUT		msecs_to_jiffies(20) /* 20 mSec */

enum {
	CHIP_POWER_OFF,
	CHIP_POWER_ON_ALS_OFF,
	CHIP_POWER_ON_ALS_ON,
};

struct cm3218_chip {
	struct i2c_client		*client;
	struct i2c_device_id		*id;
	struct mutex			lock;
	struct regulator_bulk_data	*consumers;
	struct notifier_block		regulator_nb;
	wait_queue_head_t		i2c_wait_queue;
	int				i2c_xfer_ready;
	struct regmap			*regmap;

	u8				als_state;
	bool is_als_on_before_suspend;
	int shutdown_complete;
};

/* regulators used by the device */
static struct regulator_bulk_data cm3218_consumers[] = {
	{
		.supply = "vdd",
	},
};

/* device's regmap configuration for i2c communication */
/* non cacheable registers*/
bool cm3218_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg == CM3218_REG_ALS_DATA;
}

static const struct reg_default cm3218_reg_defaults = {
	.reg = 0x00,
	.def = 0x0400,
};

/* TODO * in linux-next we have to add
 * val_format_endian to take care of endianness
 * use regmap_access_table instead of volatile_reg call backs
 */
static const struct regmap_config cm3218_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.volatile_reg = &cm3218_volatile_reg,
	.max_register = CM3218_REG_MAX,
	.reg_defaults = &cm3218_reg_defaults,
	.num_reg_defaults = 1,
	.cache_type = REGCACHE_RBTREE,
};

/* device's read/write functionality and a helper */
static void change_endianness_16(int *val)
{
	u8 *buf = (u8 *)val;
	u8 temp = buf[0];
	buf[0] = buf[1];
	buf[1] = temp;
}

static int _cm3218_register_read(struct cm3218_chip *chip, int reg, int *val)
{
	int ret;

	if (!chip->regmap)
		return -ENODEV;

	ret = wait_event_timeout(chip->i2c_wait_queue,
					chip->i2c_xfer_ready, I2C_MAX_TIMEOUT);
	if (!ret) {
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg: device not ready for i2c xfer\n",
				chip->id->name, __func__, __LINE__);
		return -ETIMEDOUT;
	}

	mutex_lock(&chip->lock);
	ret = regmap_read(chip->regmap, reg, val);
	if (ret)
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg:regmap_read fails\n",
				chip->id->name, __func__, __LINE__);

	change_endianness_16(val);
/*
	temp = i2c_smbus_read_word_data(chip->client, CM3218_REG_ALS_DATA);
	dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"als_i2c_data = %d\n",
				chip->id->name, __func__, __LINE__, temp);
*/	mutex_unlock(&chip->lock);
	return ret;
}

static int _cm3218_register_write(struct cm3218_chip *chip, int reg, int mask,
				int val)
{
	int ret;

	if (!chip->regmap)
		return -ENODEV;

	ret = wait_event_timeout(chip->i2c_wait_queue,
					chip->i2c_xfer_ready, I2C_MAX_TIMEOUT);
	if (!ret) {
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg: device not ready for i2c xfer\n",
				chip->id->name, __func__, __LINE__);
		return -ETIMEDOUT;
	}

	mutex_lock(&chip->lock);
	change_endianness_16(&val);
	ret = regmap_update_bits(chip->regmap, reg, mask, val);
	if (ret)
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg:regmap_write fails\n",
				chip->id->name, __func__, __LINE__);
	mutex_unlock(&chip->lock);

	return ret;
}

/* device's registration with iio to facilitate user operations */
static ssize_t cm3218_chan_regulator_enable(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, const char *buf, size_t len)
{
	u8 enable;
	int ret = 0;
	struct cm3218_chip *chip = iio_priv(indio_dev);

	if (kstrtou8(buf, 10, &enable))
		return -EINVAL;

	if ((enable != 0) && (enable != 1))
		return -EINVAL;

	if (chan->type != IIO_LIGHT)
		return -EINVAL;

	if (enable == (chip->als_state != CHIP_POWER_OFF))
		return 1;

	if (!chip->consumers)
		goto success;

	if (enable)
		ret = regulator_bulk_enable(1, chip->consumers);
	else
		ret = regulator_bulk_disable(1, chip->consumers);

	if (ret) {
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg:_cm3218_register_read fails\n",
				chip->id->name, __func__, __LINE__);
		goto fail;
	}

success:
	chip->als_state = enable;
fail:
	return ret ? ret : 1;
}

static ssize_t cm3218_chan_enable(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, const char *buf, size_t len)
{
	u8 enable;
	int ret = 0;
	struct cm3218_chip *chip = iio_priv(indio_dev);

	if (kstrtou8(buf, 10, &enable))
		return -EINVAL;

	if ((enable != 0) && (enable != 1))
		return -EINVAL;

	if (chip->als_state == CHIP_POWER_OFF) {
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg:please enable regulator first\n",
				chip->id->name, __func__, __LINE__);
		return -EINVAL;
	}

	if (!((enable && (chip->als_state == CHIP_POWER_ON_ALS_OFF)) ||
		(!enable && (chip->als_state == CHIP_POWER_ON_ALS_ON))))
		return -EINVAL;

	if (chan->type != IIO_LIGHT)
		return -EINVAL;

	/* a small optimization*/
	if (enable == (chip->als_state - 1))
		goto success;

	if (enable) {
		ret = _cm3218_register_write(chip, CM3218_REG_CONFIGURE,
						CONFIGURE_ALS_MASK,
						CONFIGURE_ALS_EN);
		if (ret)
			return ret;
	} else {
		ret = _cm3218_register_write(chip, CM3218_REG_CONFIGURE,
						CONFIGURE_ALS_MASK,
						!CONFIGURE_ALS_EN);
		if (ret)
			return ret;
	}

success:
	/* a small optimization*/
	chip->als_state = enable + 1;
	return ret ? ret : 1;
}


/* chan_regulator_enable is used to enable regulators used by
 * particular channel.
 * chan_enable actually configures various registers to activate
 * a particular channel.
 */
static const struct iio_chan_spec_ext_info cm3218_ext_info[] = {
	{
		.name = "regulator_enable",
		.write = cm3218_chan_regulator_enable,
	},
	{
		.name = "enable",
		.write = cm3218_chan_enable,
	},
	{
	},
};

static const struct iio_chan_spec cm3218_channels[] = {
	{
		.type = IIO_LIGHT,
		.ext_info = cm3218_ext_info,
	},
};

static int cm3218_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct cm3218_chip *chip = iio_priv(indio_dev);
	int ret;
	int value;

	if (chip->als_state != CHIP_POWER_ON_ALS_ON)
		return -EINVAL;

	if (chan->type != IIO_LIGHT)
		return -EINVAL;

	ret = _cm3218_register_read(chip, CM3218_REG_ALS_DATA, &value);
	if (ret)
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
			"error_msg:_cm3218_register_read fails\n",
			chip->id->name, __func__, __LINE__);

	if (!ret) {
		*val = value;
		ret = IIO_VAL_INT;
	}
	return ret;
}

/* read_raw is used to report a channel's data to user
 * in non SI units
 */
static const struct iio_info cm3218_iio_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &cm3218_read_raw,
};

/* chip's power management helpers */
static int cm3218_activate_standby_mode(struct cm3218_chip *chip)
{
	int ret;
	ret = _cm3218_register_write(chip, CM3218_REG_CONFIGURE,
					CONFIGURE_SHDN_MASK,
					CONFIGURE_SHDN_EN);
	return 0;
}

/* this detects the regulator enable/disable event and puts
 * the device to low power state if this device does not use the regulator */
static int cm3218_power_manager(struct notifier_block *regulator_nb,
				unsigned long event, void *v)
{
	struct cm3218_chip *chip;

	chip = container_of(regulator_nb, struct cm3218_chip, regulator_nb);

	if (event & (REGULATOR_EVENT_POST_ENABLE |
			REGULATOR_EVENT_OUT_POSTCHANGE)) {
		chip->i2c_xfer_ready = 1;
		cm3218_activate_standby_mode(chip);
	} else if (event & (REGULATOR_EVENT_DISABLE |
			REGULATOR_EVENT_FORCE_DISABLE)) {
		chip->i2c_xfer_ready = 0;
	}
	return NOTIFY_OK;
}

#ifdef CONFIG_PM_SLEEP
static int cm3218_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm3218_chip *chip = iio_priv(indio_dev);
	int ret = 0;

	if (!chip->consumers)
		return 0;


	/* assumes all other devices stop using this regulator */
	if (chip->als_state != CHIP_POWER_OFF)
		ret = regulator_bulk_disable(1, chip->consumers);

	if (ret) {
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg:regulator_bulk_disable fails" \
				"\ncm3218_suspend fails\n",
				chip->id->name, __func__, __LINE__);
		return ret;
	}

	if (!ret && regulator_is_enabled(chip->consumers[0].consumer))
		ret = cm3218_activate_standby_mode(chip);
	if (ret)
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg:cm3218_activate_standby_mode fails" \
				"\ncm3218_suspend fails\n",
				chip->id->name, __func__, __LINE__);
	return ret;
}

static int cm3218_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm3218_chip *chip = iio_priv(indio_dev);
	int ret = 0;
	int temp;

	if (chip->als_state == CHIP_POWER_OFF)
		return 0;

	ret = regulator_bulk_enable(1, cm3218_consumers);
	if (ret) {
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg:regulator_bulk_enable fails" \
				"\ncm3218_suspend fails\n",
				chip->id->name, __func__, __LINE__);
		return ret;
	}

	mutex_lock(&chip->lock);
	if (chip->als_state == CHIP_POWER_ON_ALS_ON)
		ret = _cm3218_register_write(chip, CM3218_REG_CONFIGURE,
						CONFIGURE_ALS_MASK,
						CONFIGURE_ALS_EN);
	if (ret) {
		dev_err(&chip->client->dev, "idname:%s func:%s line:%d " \
				"error_msg:_cm3218_register_write fails" \
				"\ncm3218_suspend fails\n",
				chip->id->name, __func__, __LINE__);
		/* fall back to CHIP_POWER_OFF, we don't need partial resume */
		do {
			temp = regulator_bulk_disable(1, cm3218_consumers);
		} while (temp);
	}
	mutex_unlock(&chip->lock);
	return ret;
}

static SIMPLE_DEV_PM_OPS(cm3218_pm_ops, cm3218_suspend, cm3218_resume);
#define CM3218_PM_OPS (&cm3218_pm_ops)
#else
#define CM3218_PM_OPS NULL
#endif


/* device's i2c registration */
static int cm3218_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm3218_chip *chip;
	struct iio_dev *indio_dev;
	struct regmap *regmap;

	indio_dev = iio_allocate_device(sizeof(*chip));
	if (indio_dev == NULL) {
		dev_err(&client->dev, "idname:%s func:%s line:%d " \
			"error_msg:iio_allocate_device fails\n",
			id->name, __func__, __LINE__);
		return -ENOMEM;
	}
	chip = iio_priv(indio_dev);

	i2c_set_clientdata(client, indio_dev);
	chip->client = client;
	mutex_init(&chip->lock);

	regmap = devm_regmap_init_i2c(client, &cm3218_regmap_config);
	if (IS_ERR_OR_NULL(regmap)) {
		dev_err(&client->dev, "idname:%s func:%s line:%d " \
		"error_msg:devm_regmap_init_i2c fails\n",
		id->name, __func__, __LINE__);
		return -ENOMEM;
	}
	chip->regmap = regmap;

	ret = devm_regulator_bulk_get(&client->dev,
					ARRAY_SIZE(cm3218_consumers),
					cm3218_consumers);
	if (ret)
		dev_err(&client->dev, "idname:%s func:%s line:%d " \
			"error_msg:regulator_get fails\n",
			id->name, __func__, __LINE__);
	else
		chip->consumers = cm3218_consumers;

	if (chip->consumers) {
		chip->regulator_nb.notifier_call = cm3218_power_manager;
		ret = regulator_register_notifier(chip->consumers[0].consumer,
						&chip->regulator_nb);
		if (ret)
			dev_err(&client->dev, "idname:%s func:%s line:%d " \
				"error_msg:regulator_register_notifier fails\n",
				id->name, __func__, __LINE__);
	}

	indio_dev->info = &cm3218_iio_info;
	indio_dev->channels = cm3218_channels;
	indio_dev->num_channels = 1;
	indio_dev->name = id->name;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&client->dev, "idname:%s func:%s line:%d " \
			"error_msg:iio_device_register fails\n",
			id->name, __func__, __LINE__);
		goto err_iio_free;
	}

	init_waitqueue_head(&chip->i2c_wait_queue);
	chip->als_state = 0;
	if (regulator_is_enabled(chip->consumers[0].consumer)) {
		chip->i2c_xfer_ready = 1;
		cm3218_activate_standby_mode(chip);
	}

	dev_info(&client->dev, "idname:%s func:%s line:%d " \
			"probe success\n",
			id->name, __func__, __LINE__);

	return 0;

err_iio_free:
	mutex_destroy(&chip->lock);
	iio_free_device(indio_dev);
	return ret;
}

static void cm3218_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm3218_chip *chip = iio_priv(indio_dev);
	mutex_lock(&chip->lock);
	chip->shutdown_complete = 1;
	mutex_unlock(&chip->lock);
}

static int __devexit cm3218_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm3218_chip *chip = iio_priv(indio_dev);

	mutex_destroy(&chip->lock);
	iio_device_unregister(indio_dev);
	iio_free_device(indio_dev);
	return 0;
}

static const struct i2c_device_id cm3218_id[] = {
	{"cm3218", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cm3218_id);

static const struct of_device_id cm3218_of_match[] = {
	{ .compatible = "capella,cm3218", },
	{ },
};
MODULE_DEVICE_TABLE(of, cm3218_of_match);

static struct i2c_driver cm3218_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "cm3218",
		.owner = THIS_MODULE,
		.of_match_table = cm3218_of_match,
		.pm = CM3218_PM_OPS,
	},
	.id_table = cm3218_id,
	.probe = cm3218_probe,
	.remove = __devexit_p(cm3218_remove),
	.shutdown = cm3218_shutdown,
};

static int __init cm3218_init(void)
{
	return i2c_add_driver(&cm3218_driver);
}

static void __exit cm3218_exit(void)
{
	i2c_del_driver(&cm3218_driver);
}

module_init(cm3218_init);
module_exit(cm3218_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM3218 Driver");
MODULE_AUTHOR("Sri Krishna chowdary <schowdary@nvidia.com>");
