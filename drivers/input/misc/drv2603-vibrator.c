/*
 * DRV2603 haptic driver for LRA and ERM vibrator motor
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/pwm.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/input/drv2603-vibrator.h>
#include <linux/edp.h>

#define DEFAULT_DUTY_CYCLE 80

enum drv2603_state {
	VIBRATOR_OFF,
	VIBRATOR_ON,
};

struct drv2603_chip {
	int pwm_id;
	struct pwm_device *pwm_device;
	struct device *dev;
	struct input_dev *input_dev;
	struct regulator *regulator;
	struct work_struct work;
	int gpio;
	int duty_cycle;
	enum drv2603_mode vibrator_mode;
	enum drv2603_state state;
	struct edp_client *haptic_edp_client;
};

static int drv2603_set_duty_cycle(struct drv2603_chip *chip, int val)
{
	return pwm_config(chip->pwm_device, val * 100, 10000);
}

static int drv2603_vibrator_initialize(struct drv2603_chip *chip)
{
	int ret;

	chip->duty_cycle = DEFAULT_DUTY_CYCLE;
	ret = drv2603_set_duty_cycle(chip, chip->duty_cycle);
	if (ret) {
		dev_err(chip->dev, "Failed to initialize pwm config\n");
		return ret;
	}

	ret = gpio_request_one(chip->gpio, GPIOF_OUT_INIT_LOW, NULL);
	if (ret) {
		dev_err(chip->dev, "Failed to request gpio\n");
		return ret;
	}

	return 0;
}

static int drv2603_vibrate(struct drv2603_chip *chip)
{
	int ret;
	unsigned int approved;

	switch (chip->state) {
	case VIBRATOR_OFF:
		if (chip->haptic_edp_client) {
			ret = edp_update_client_request(chip->haptic_edp_client,
				DRV2603_HAPTIC_EDP_LOW, &approved);
			if (ret) {
				dev_err(chip->dev,
					"E state transition failed\n");
				return ret;
			}
		}
		gpio_set_value_cansleep(chip->gpio, 0);
		drv2603_set_duty_cycle(chip, 0);
		pwm_disable(chip->pwm_device);
		regulator_disable(chip->regulator);
	break;
	case VIBRATOR_ON:
		if (chip->haptic_edp_client) {
			ret = edp_update_client_request(chip->haptic_edp_client,
				DRV2603_HAPTIC_EDP_HIGH, &approved);
			if (ret || approved != DRV2603_HAPTIC_EDP_HIGH) {
				dev_err(chip->dev,
					"E state transition failed\n");
				return ret;
			}
		}
		regulator_enable(chip->regulator);
		drv2603_set_duty_cycle(chip, chip->duty_cycle);
		gpio_set_value_cansleep(chip->gpio, 1);
		pwm_enable(chip->pwm_device);
	break;
	}
	return 0;
}

static void drv2603_haptic_play_effect_work(struct work_struct *work)
{
	struct drv2603_chip *chip =
		container_of(work, struct drv2603_chip, work);

	drv2603_vibrate(chip);
}

static int drv2603_haptic_play_effect(struct input_dev *dev, void *data,
					struct ff_effect *effect)
{
	struct drv2603_chip *chip = input_get_drvdata(dev);
	int magnitude = effect->u.rumble.strong_magnitude;
	if (!magnitude)
		magnitude = effect->u.rumble.weak_magnitude;

	if (magnitude && chip->state == VIBRATOR_OFF) {
		chip->state = VIBRATOR_ON;
		schedule_work(&chip->work);
	} else if (!magnitude && chip->state == VIBRATOR_ON) {
		chip->state = VIBRATOR_OFF;
		schedule_work(&chip->work);
	}
	return 0;
}

static ssize_t drv2603_vibrator_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drv2603_chip *chip = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", chip->state);
}

static ssize_t drv2603_vibrator_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct drv2603_chip *chip = dev_get_drvdata(dev);
	int var;

	sscanf(buf, "%d", &var);
	if (var == 0 && chip->state == VIBRATOR_ON) {
		cancel_work_sync(&chip->work);
		chip->state = VIBRATOR_OFF;
		schedule_work(&chip->work);
	} else if (var == 1 && chip->state == VIBRATOR_OFF) {
		cancel_work_sync(&chip->work);
		chip->state = VIBRATOR_ON;
		schedule_work(&chip->work);
	}

	return count;
}

static ssize_t drv2603_vibrator_duty_cycle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct drv2603_chip *chip = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", chip->duty_cycle);
}

static ssize_t drv2603_vibrator_duty_cycle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct drv2603_chip *chip = dev_get_drvdata(dev);
	int duty_cycle;

	sscanf(buf, "%d", &duty_cycle);

	if (duty_cycle >= 0 && duty_cycle <= 100)
		chip->duty_cycle = duty_cycle;
	else
		dev_err(dev, "Duty cycle should be in range 0-100\n");

	return count;
}

static void drv2603_vibrator_throttle(unsigned int new_state, void *priv_data)
{
	struct drv2603_chip *chip = priv_data;

	if (!chip)
		return;

	if (chip->state == VIBRATOR_ON) {
		chip->state = VIBRATOR_OFF;
		drv2603_vibrate(chip);
	}
}

static DEVICE_ATTR(vibrator_enable, 0640, drv2603_vibrator_enable_show,
					drv2603_vibrator_enable_store);
static DEVICE_ATTR(duty_cycle, 0640, drv2603_vibrator_duty_cycle_show,
					drv2603_vibrator_duty_cycle_store);

static struct attribute *drv2603_vibrator_attr[] = {
	&dev_attr_vibrator_enable.attr,
	&dev_attr_duty_cycle.attr,
	NULL,
};

static const struct attribute_group drv2603_vibrator_attr_group = {
	.attrs = drv2603_vibrator_attr,
};

static int __devinit drv2603_probe(struct platform_device *pdev)
{
	int ret;

	struct drv2603_chip *chip;
	struct input_dev *input_dev;
	struct edp_manager *battery_manager = NULL;
	struct drv2603_platform_data *drv2603_pdata =
				dev_get_platdata(&pdev->dev);

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip),
					GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Memory allocation failed for chip\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev,
			"unable to allocate memory for input dev\n");
		return PTR_ERR(input_dev);
	}

	chip->pwm_id = drv2603_pdata->pwm_id;
	chip->gpio = drv2603_pdata->gpio;
	chip->duty_cycle = drv2603_pdata->duty_cycle;
	chip->vibrator_mode = drv2603_pdata->vibrator_mode;
	chip->dev = &pdev->dev;
	chip->input_dev = input_dev;
	dev_set_drvdata(&pdev->dev, chip);

	chip->regulator = regulator_get(&pdev->dev, "vdd_vbrtr");
	if (IS_ERR_OR_NULL(chip->regulator)) {
		dev_err(&pdev->dev, "Failed to get regulator\n");
		ret = PTR_ERR(chip->regulator);
		goto err_regulator;
	}

	chip->pwm_device = pwm_request(chip->pwm_id, "drv2603-vibrator");

	if (IS_ERR_OR_NULL(chip->pwm_device)) {
		dev_err(&pdev->dev, "Failed to get pwm device\n");
		ret = PTR_ERR(chip->pwm_device);
		goto err_pwm;
	}

	if (drv2603_pdata->edp_states[0] == 0) {
		dev_err(&pdev->dev, "EDP states invalid or not present");
		goto register_input;
	}

	chip->haptic_edp_client = devm_kzalloc(&pdev->dev,
				sizeof(struct edp_client), GFP_KERNEL);

	if (IS_ERR_OR_NULL(chip->haptic_edp_client)) {
		dev_err(&pdev->dev, "Could not allocate edp client\n");
		goto register_input;
	}

	strncpy(chip->haptic_edp_client->name, "drv2603-haptic",
						EDP_NAME_LEN - 1);
	chip->haptic_edp_client->name[EDP_NAME_LEN - 1] = '\0';
	chip->haptic_edp_client->states = drv2603_pdata->edp_states;
	chip->haptic_edp_client->num_states = DRV2603_HAPTIC_EDP_NUM_STATES;
	chip->haptic_edp_client->e0_index = DRV2603_HAPTIC_EDP_LOW;
	chip->haptic_edp_client->priority = EDP_MAX_PRIO + 2;
	chip->haptic_edp_client->throttle = drv2603_vibrator_throttle;
	chip->haptic_edp_client->private_data = chip;

	battery_manager = edp_get_manager("battery");
	if (IS_ERR_OR_NULL(battery_manager)) {
		dev_err(&pdev->dev, "Unable to get edp manager\n");
	} else {
		ret = edp_register_client(battery_manager,
				chip->haptic_edp_client);
		if (ret) {
			dev_err(&pdev->dev, "Unable to register edp client\n");
		} else {
			ret = edp_update_client_request(chip->haptic_edp_client,
					DRV2603_HAPTIC_EDP_LOW, NULL);
			if (ret) {
				dev_err(&pdev->dev,
					"Unable to set E0 EDP state\n");
				edp_unregister_client(chip->haptic_edp_client);
			} else {
				goto register_input;
			}
		}
	}

	devm_kfree(&pdev->dev, chip->haptic_edp_client);
	chip->haptic_edp_client = NULL;

register_input:
	input_dev->name = "drv2603-haptic";
	input_dev->id.version = 1;
	input_dev->dev.parent = &pdev->dev;
	input_set_drvdata(input_dev, chip);
	input_set_capability(input_dev, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(input_dev, NULL,
				drv2603_haptic_play_effect);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to create FF device(ret : %d)\n", ret);
		goto err_input_create;
	}
	INIT_WORK(&chip->work, drv2603_haptic_play_effect_work);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register input device(ret : %d)\n", ret);
		goto err_input_register;
	}

	ret = drv2603_vibrator_initialize(chip);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize resources\n");
		goto err_vibrator_init;
	}

	ret = sysfs_create_group(&pdev->dev.kobj,
				&drv2603_vibrator_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs group\n");
		goto err_sysfs_create;
	}

	chip->state = VIBRATOR_OFF;
	return 0;

err_sysfs_create:
	gpio_free(chip->gpio);
err_vibrator_init:
	input_unregister_device(input_dev);
err_input_register:
	destroy_work_on_stack(&chip->work);
err_input_create:
	edp_unregister_client(chip->haptic_edp_client);
	pwm_free(chip->pwm_device);
err_pwm:
	regulator_put(chip->regulator);
err_regulator:
	input_free_device(input_dev);

	return ret;
}

static int __devexit drv2603_remove(struct platform_device *pdev)
{
	struct drv2603_chip *chip = dev_get_drvdata(&pdev->dev);

	pwm_free(chip->pwm_device);
	gpio_free(chip->gpio);
	destroy_work_on_stack(&chip->work);
	edp_unregister_client(chip->haptic_edp_client);
	input_unregister_device(chip->input_dev);
	input_free_device(chip->input_dev);
	regulator_put(chip->regulator);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int drv2603_suspend(struct device *dev)
{
	struct drv2603_chip *chip = dev_get_drvdata(dev);

	if (chip->state == VIBRATOR_ON) {
		chip->state = VIBRATOR_OFF;
		drv2603_vibrate(chip);
	}

	return 0;
}

static int drv2603_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(drv2603_pm_ops, drv2603_suspend, drv2603_resume);

static struct platform_driver drv2603_driver = {
	.probe = drv2603_probe,
	.remove = __devexit_p(drv2603_remove),
	.driver = {
		.name = "drv2603-vibrator",
		.owner = THIS_MODULE,
		.pm = &drv2603_pm_ops,
	},
};

static int __init drv2603_vibrator_init(void)
{
	return platform_driver_register(&drv2603_driver);
}
module_init(drv2603_vibrator_init);

static void __exit drv2603_vibrator_exit(void)
{
	platform_driver_unregister(&drv2603_driver);
}
module_exit(drv2603_vibrator_exit);

MODULE_DESCRIPTION("Drv2603 vibrator driver");
MODULE_AUTHOR("Sumit Sharma <sumsharma@nvidia.com>");
MODULE_LICENSE("GPL v2");
