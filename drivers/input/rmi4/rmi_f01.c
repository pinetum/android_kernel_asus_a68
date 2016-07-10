/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include "rmi_driver.h"
//ASUS_BSP simpson: add for On/Off touch in P03 +++
static struct rmi_function_container *g_fc;
#include <linux/workqueue.h>
static struct workqueue_struct *g_rmi_wq_resume_check;
static struct delayed_work g_mp_resume_chk_work;
static void rmi_resume_check_work(struct work_struct *work);
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
//#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+
static struct workqueue_struct *g_rmi_wq_attach_detach;
static struct work_struct g_mp_attach_work;
//static struct work_struct g_mp_detach_work;
static struct delayed_work g_mp_detach_work;
static void attach_padstation_work(struct work_struct *work);
static void detach_padstation_work(struct work_struct *work);
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP simpson: add for On/Off touch in P03 ---
//ASUS_BSP simpson: add for suspend/resume +++
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
bool g_bFoneInPad = false;
int g_resumeLog;
unsigned int g_XY;
extern bool g_bIsSuspended;
//ASUS_BSP simpson: add for suspend/resume ---
//ASUS_BSP simpson: add for keypad_bl +++
#include <linux/leds.h>
extern struct delayed_work keypad_off_work;
extern struct led_trigger *keypad_led_trigger;
//extern void turnoff_keypad_bl(struct work_struct *work);
extern void rmi_led_trigger_event(struct led_trigger *trigger, enum led_brightness event);
//ASUS_BSP simpson: add for keypad_bl ---
//ASUS_BSP simpson: add to reset device +++
#include <linux/delay.h>
//ASUS_BSP simpson: add to reset device ---
//ASUS_BSP simpson: add for fast relax +++
//#define FFR 1
#ifdef FFR
#include <linux/workqueue.h>
static struct workqueue_struct *fast_relax_workqueue;
static struct delayed_work fast_relax_work;
#endif
//ASUS_BSP simpson: add for fast relax ---
//ASUS_BSP simpson: add for ASUSEvtlog +++
//#define CONFIG_ASUS_DEBUG y
#ifndef CONFIG_ASUS_DEBUG
#define ASUSEvtlog(...) do{}while(0);
#else
#include <linux/asusdebug.h>
#endif
//ASUS_BSP simpson: add for ASUSEvtlog ---

/* control register bits */
#define RMI_SLEEP_MODE_NORMAL (0x00)
#define RMI_SLEEP_MODE_SENSOR_SLEEP (0x01)
#define RMI_SLEEP_MODE_RESERVED0 (0x02)
#define RMI_SLEEP_MODE_RESERVED1 (0x03)

#define RMI_IS_VALID_SLEEPMODE(mode) \
	(mode >= RMI_SLEEP_MODE_NORMAL && mode <= RMI_SLEEP_MODE_RESERVED1)

union f01_device_commands {
	struct {
		u8 reset:1;
		u8 reserved:1;
	};
	u8 reg;
};

struct f01_device_control_0 {
	union {
		struct {
			u8 sleep_mode:2;
			u8 nosleep:1;
			u8 reserved:2;
			u8 charger_input:1;
			u8 report_rate:1;
			u8 configured:1;
		};
		u8 reg;
	};
};

struct f01_device_control {
	struct f01_device_control_0 ctrl0;
	u8 *interrupt_enable;
	u8 doze_interval;
	u8 wakeup_threshold;
	u8 doze_holdoff;
};

union f01_basic_queries {
	struct {
		u8 manufacturer_id:8;

		u8 custom_map:1;
		u8 non_compliant:1;
		u8 has_lts:1;
		u8 has_sensor_id:1;
		u8 has_charger_input:1;
		u8 has_adjustable_doze:1;
		u8 has_adjustable_doze_holdoff:1;
		u8 has_product_properties_2:1;

		u8 productinfo_1:7;
		u8 q2_bit_7:1;
		u8 productinfo_2:7;
		u8 q3_bit_7:1;

		u8 year:5;
		u8 month:4;
		u8 day:5;
		u8 cp1:1;
		u8 cp2:1;
		u8 wafer_id1_lsb:8;
		u8 wafer_id1_msb:8;
		u8 wafer_id2_lsb:8;
		u8 wafer_id2_msb:8;
		u8 wafer_id3_lsb:8;
	};
	u8 regs[11];
};

union f01_query_42 {
	struct {
		u8 has_ds4_queries:1;
		u8 has_multi_phy:1;
		u8 has_guest:1;
		u8 reserved:5;
	};
	u8 regs[1];
};

union f01_ds4_queries {
	struct {
		u8 length:4;
		u8 reserved_1:4;

		u8 has_package_id_query:1;
		u8 has_packrat_query:1;
		u8 has_reset_query:1;
		u8 has_maskrev_query:1;
		u8 reserved_2:4;

		u8 has_i2c_control:1;
		u8 has_spi_control:1;
		u8 has_attn_control:1;
		u8 reserved_3:5;

		u8 reset_enabled:1;
		u8 reset_polarity:1;
		u8 pullup_enabled:1;
		u8 reserved_4:1;
		u8 reset_pin_number:4;
	};
	u8 regs[4];
};

struct f01_data {
	struct f01_device_control device_control;
	union f01_basic_queries basic_queries;
	union f01_device_status device_status;
	u8 product_id[RMI_PRODUCT_ID_LENGTH+1];

	u8 interrupt_enable_addr;
	u8 doze_interval_addr;
	u8 wakeup_threshold_addr;
	u8 doze_holdoff_addr;

	int irq_count;
	int num_of_irq_regs;

#ifdef	CONFIG_PM
	bool suspended;
	bool old_nosleep;
#endif
};


static ssize_t rmi_fn_01_productinfo_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);

static ssize_t rmi_fn_01_productid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

//ASUS_BSP simpson: add for firmware id +++
static ssize_t rmi_fn_01_firmwareid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);
//ASUS_BSP simpson: add for firmware id ---

//ASUS_BSP simpson: add for ATD check +++
static ssize_t rmi_fn_01_status_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);
//ASUS_BSP simpson: add for ATD check ---

static ssize_t rmi_fn_01_manufacturer_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);

static ssize_t rmi_fn_01_datecode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

static ssize_t rmi_fn_01_reportrate_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);

static ssize_t rmi_fn_01_reportrate_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count);

static ssize_t rmi_fn_01_interrupt_enable_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);

static ssize_t rmi_fn_01_interrupt_enable_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count);

static ssize_t rmi_fn_01_doze_interval_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);

static ssize_t rmi_fn_01_doze_interval_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count);

static ssize_t rmi_fn_01_wakeup_threshold_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);

static ssize_t rmi_fn_01_wakeup_threshold_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count);

static ssize_t rmi_fn_01_doze_holdoff_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);

static ssize_t rmi_fn_01_doze_holdoff_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count);

static ssize_t rmi_fn_01_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count);

static ssize_t rmi_fn_01_sleepmode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_01_sleepmode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

static ssize_t rmi_fn_01_nosleep_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);

static ssize_t rmi_fn_01_nosleep_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count);

static ssize_t rmi_fn_01_chargerinput_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);

static ssize_t rmi_fn_01_chargerinput_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count);

static ssize_t rmi_fn_01_configured_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);

static ssize_t rmi_fn_01_unconfigured_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);

static ssize_t rmi_fn_01_flashprog_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);

static ssize_t rmi_fn_01_statuscode_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);

static int rmi_f01_alloc_memory(struct rmi_function_container *fc,
					int num_of_irq_regs);

static void rmi_f01_free_memory(struct rmi_function_container *fc);

static int rmi_f01_initialize(struct rmi_function_container *fc);

static int rmi_f01_create_sysfs(struct rmi_function_container *fc);

static int rmi_f01_config(struct rmi_function_container *fc);

static int rmi_f01_reset(struct rmi_function_container *fc);


static struct device_attribute fn_01_attrs[] = {
	__ATTR(productinfo, RMI_RO_ATTR,
	       rmi_fn_01_productinfo_show, rmi_store_error),
	__ATTR(productid, RMI_RO_ATTR,
	       rmi_fn_01_productid_show, rmi_store_error),
//ASUS_BSP simpson: add for firmware id +++
	__ATTR(fw_id, RMI_RO_ATTR,
	       rmi_fn_01_firmwareid_show, rmi_store_error),
//ASUS_BSP simpson: add for firmware id ---
//ASUS_BSP simpson: add for ATD check +++
	__ATTR(touch_status, RMI_RO_ATTR,
	       rmi_fn_01_status_show, rmi_store_error),
//ASUS_BSP simpson: add for ATD check ---
	__ATTR(manufacturer, RMI_RO_ATTR,
	       rmi_fn_01_manufacturer_show, rmi_store_error),
	__ATTR(datecode, RMI_RO_ATTR,
	       rmi_fn_01_datecode_show, rmi_store_error),

	/* control register access */
	__ATTR(sleepmode, RMI_RW_ATTR,
	       rmi_fn_01_sleepmode_show, rmi_fn_01_sleepmode_store),
	__ATTR(nosleep, RMI_RW_ATTR,
	       rmi_fn_01_nosleep_show, rmi_fn_01_nosleep_store),
	__ATTR(chargerinput, RMI_RW_ATTR,
	       rmi_fn_01_chargerinput_show, rmi_fn_01_chargerinput_store),
	__ATTR(reportrate, RMI_RW_ATTR,
	       rmi_fn_01_reportrate_show, rmi_fn_01_reportrate_store),
	__ATTR(interrupt_enable, RMI_RW_ATTR,
	       rmi_fn_01_interrupt_enable_show,
		rmi_fn_01_interrupt_enable_store),
	__ATTR(doze_interval, RMI_RW_ATTR,
	       rmi_fn_01_doze_interval_show,
		rmi_fn_01_doze_interval_store),
	__ATTR(wakeup_threshold, RMI_RW_ATTR,
	       rmi_fn_01_wakeup_threshold_show,
		rmi_fn_01_wakeup_threshold_store),
	__ATTR(doze_holdoff, RMI_RW_ATTR,
	       rmi_fn_01_doze_holdoff_show,
		rmi_fn_01_doze_holdoff_store),

	/* We make report rate RO, since the driver uses that to look for
	 * resets.  We don't want someone faking us out by changing that
	 * bit.
	 */
	__ATTR(configured, RMI_RO_ATTR,
	       rmi_fn_01_configured_show, rmi_store_error),

	/* Command register access. */
	__ATTR(reset, RMI_WO_ATTR,
	       rmi_show_error, rmi_fn_01_reset_store),

	/* STatus register access. */
	__ATTR(unconfigured, RMI_RO_ATTR,
	       rmi_fn_01_unconfigured_show, rmi_store_error),
	__ATTR(flashprog, RMI_RO_ATTR,
	       rmi_fn_01_flashprog_show, rmi_store_error),
	__ATTR(statuscode, RMI_RO_ATTR,
	       rmi_fn_01_statuscode_show, rmi_store_error),
};

/* Utility routine to set the value of a bit field in a register. */
int rmi_set_bit_field(struct rmi_device *rmi_dev,
		      unsigned short address,
		      unsigned char field_mask,
		      unsigned char bits)
{
	unsigned char reg_contents;
	int retval;

	retval = rmi_read(rmi_dev, address, &reg_contents);
	if (retval)
		return retval;
	reg_contents = (reg_contents & ~field_mask) | bits;
	retval = rmi_write(rmi_dev, address, reg_contents);
	if (retval == 1)
		return 0;
	else if (retval == 0)
		return -EIO;
	return retval;
}

static ssize_t rmi_fn_01_productinfo_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
			data->basic_queries.productinfo_1,
			data->basic_queries.productinfo_2);
}

static ssize_t rmi_fn_01_productid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%s\n", data->product_id);
}

//ASUS_BSP simpson: add for firmware id +++
static ssize_t rmi_fn_01_firmwareid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	u16 query_base_addr;
	unsigned char fwid[3];
	unsigned int firmwareid;
	int retval = 0;

	fc = to_rmi_function_container(dev);
	query_base_addr = fc->fd.query_base_addr;

	retval = rmi_read_block(fc->rmi_dev, query_base_addr + 18, fwid,
			ARRAY_SIZE(fwid));

	if (retval < 0) {
		dev_err(&fc->dev, "Could not read firmware_id from 0x%04x.\n",
			query_base_addr);
		return retval;
	}

	firmwareid = fwid[0] | (fwid[1] << 8) | (fwid[2] << 16);

	return sprintf(buf, "%u\n", firmwareid);
}
//ASUS_BSP simpson: add for firmware id ---

//ASUS_BSP simpson: add for ATD check +++
static ssize_t rmi_fn_01_status_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;
	if (data) {
		return sprintf(buf, "1\n");
	} else {
		return sprintf(buf, "0\n");
	}
}
//ASUS_BSP simpson: add for ATD check ---

static ssize_t rmi_fn_01_manufacturer_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n",
			data->basic_queries.manufacturer_id);
}

static ssize_t rmi_fn_01_datecode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "20%02u-%02u-%02u\n",
			data->basic_queries.year,
			data->basic_queries.month,
			data->basic_queries.day);
}

static ssize_t rmi_fn_01_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc = NULL;
	unsigned int reset;
	int retval = 0;
	/* Command register always reads as 0, so we can just use a local. */
	union f01_device_commands commands = {};

	fc = to_rmi_function_container(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;
	if (reset < 0 || reset > 1)
		return -EINVAL;

	/* Per spec, 0 has no effect, so we skip it entirely. */
	if (reset) {
		commands.reset = 1;
		retval = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,
				&commands.reg, sizeof(commands.reg));
		if (retval < 0) {
			dev_err(dev, "%s: failed to issue reset command, "
				"error = %d.", __func__, retval);
			return retval;
		}
	}

	return count;
}

static ssize_t rmi_fn_01_sleepmode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE,
			"%d\n", data->device_control.ctrl0.sleep_mode);
}

static ssize_t rmi_fn_01_sleepmode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct f01_data *data = NULL;
	unsigned long new_value;
	int retval;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || !RMI_IS_VALID_SLEEPMODE(new_value)) {
		dev_err(dev, "%s: Invalid sleep mode %s.", __func__, buf);
		return -EINVAL;
	}

	dev_dbg(dev, "Setting sleep mode to %ld.", new_value);
	data->device_control.ctrl0.sleep_mode = new_value;
	retval = rmi_write_block(fc->rmi_dev, fc->fd.control_base_addr,
			&data->device_control.ctrl0.reg,
			sizeof(data->device_control.ctrl0.reg));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write sleep mode, code %d.\n", retval);
	return retval;
}

static ssize_t rmi_fn_01_nosleep_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
		data->device_control.ctrl0.nosleep);
}

static ssize_t rmi_fn_01_nosleep_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct f01_data *data = NULL;
	unsigned long new_value;
	int retval;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid nosleep bit %s.", __func__, buf);
		return -EINVAL;
	}

	data->device_control.ctrl0.nosleep = new_value;
	retval = rmi_write_block(fc->rmi_dev, fc->fd.control_base_addr,
			&data->device_control.ctrl0.reg,
			sizeof(data->device_control.ctrl0.reg));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write nosleep bit.\n");
	return retval;
}

static ssize_t rmi_fn_01_chargerinput_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.ctrl0.charger_input);
}

static ssize_t rmi_fn_01_chargerinput_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct f01_data *data = NULL;
	unsigned long new_value;
	int retval;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid chargerinput bit %s.", __func__, buf);
		return -EINVAL;
	}

	data->device_control.ctrl0.charger_input = new_value;
	retval = rmi_write_block(fc->rmi_dev, fc->fd.control_base_addr,
			&data->device_control.ctrl0.reg,
			sizeof(data->device_control.ctrl0.reg));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write chargerinput bit.\n");
	return retval;
}

static ssize_t rmi_fn_01_reportrate_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.ctrl0.report_rate);
}

static ssize_t rmi_fn_01_reportrate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct f01_data *data = NULL;
	unsigned long new_value;
	int retval;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid reportrate bit %s.", __func__, buf);
		return -EINVAL;
	}

	data->device_control.ctrl0.report_rate = new_value;
	retval = rmi_write_block(fc->rmi_dev, fc->fd.control_base_addr,
			&data->device_control.ctrl0.reg,
			sizeof(data->device_control.ctrl0.reg));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write reportrate bit.\n");
	return retval;
}

static ssize_t rmi_fn_01_interrupt_enable_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function_container *fc;
	struct f01_data *data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	/* loop through each irq value and copy its
	 * string representation into buf */
	for (i = 0; i < data->irq_count; i++) {
		int irq_reg;
		int irq_shift;
		int interrupt_enable;

		irq_reg = i / 8;
		irq_shift = i % 8;
		interrupt_enable =
		    ((data->device_control.interrupt_enable[irq_reg]
			>> irq_shift) & 0x01);

		/* get next irq value and write it to buf */
		len = snprintf(current_buf, PAGE_SIZE - total_len,
			"%u ", interrupt_enable);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "%s: Failed to build interrupt_enable"
				" buffer, code = %d.\n", __func__, len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);
	return total_len;

}

static ssize_t rmi_fn_01_interrupt_enable_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f01_data *data;
	int i;
	int irq_count = 0;
	int retval = count;
	int irq_reg = 0;

	fc = to_rmi_function_container(dev);
	data = fc->data;
	for (i = 0; i < data->irq_count && *buf != 0;
	     i++, buf += 2) {
		int irq_shift;
		int interrupt_enable;
		int result;

		irq_reg = i / 8;
		irq_shift = i % 8;

		/* get next interrupt mapping value and store and bump up to
		 * point to next item in buf */
		result = sscanf(buf, "%u", &interrupt_enable);
		if ((result != 1) ||
			(interrupt_enable != 0 && interrupt_enable != 1)) {
			dev_err(dev,
				"%s: Error - interrupt enable[%d]"
				" is not a valid value 0x%x.\n",
				__func__, i, interrupt_enable);
			return -EINVAL;
		}
		if (interrupt_enable == 0) {
			data->device_control.interrupt_enable[irq_reg] &=
				(1 << irq_shift) ^ 0xFF;
		} else
			data->device_control.interrupt_enable[irq_reg] |=
				(1 << irq_shift);
		irq_count++;
	}

	/* Make sure the irq count matches */
	if (irq_count != data->irq_count) {
		dev_err(dev,
			"%s: Error - interrupt enable count of %d"
			" doesn't match device count of %d.\n",
			 __func__, irq_count, data->irq_count);
		return -EINVAL;
	}

	/* write back to the control register */
	retval = rmi_write_block(fc->rmi_dev, data->interrupt_enable_addr,
			data->device_control.interrupt_enable,
			sizeof(u8)*(data->num_of_irq_regs));
	if (retval < 0) {
		dev_err(dev, "%s : Could not write interrupt_enable_store"
			" to 0x%x\n", __func__, data->interrupt_enable_addr);
		return retval;
	}

	return count;

}

static ssize_t rmi_fn_01_doze_interval_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.doze_interval);

}

static ssize_t rmi_fn_01_doze_interval_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct f01_data *data = NULL;
	unsigned long new_value;
	int retval;
	int ctrl_base_addr;

	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value < 0 || new_value > 255) {
		dev_err(dev, "%s: Invalid doze interval %s.", __func__, buf);
		return -EINVAL;
	}

	data->device_control.doze_interval = new_value;
	ctrl_base_addr = fc->fd.control_base_addr + sizeof(u8) +
			(sizeof(u8)*(data->num_of_irq_regs));
	dev_info(dev, "doze_interval store address %x, value %d",
		ctrl_base_addr, data->device_control.doze_interval);

	retval = rmi_write_block(fc->rmi_dev, data->doze_interval_addr,
			&data->device_control.doze_interval,
			sizeof(u8));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write doze interval.\n");
	return retval;

}

static ssize_t rmi_fn_01_wakeup_threshold_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.wakeup_threshold);

}


static ssize_t rmi_fn_01_wakeup_threshold_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct f01_data *data = NULL;
	unsigned long new_value;
	int retval;

	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value < 0 || new_value > 255) {
		dev_err(dev, "%s: Invalid wakeup threshold %s.", __func__, buf);
		return -EINVAL;
	}

	data->device_control.doze_interval = new_value;
	retval = rmi_write_block(fc->rmi_dev, data->wakeup_threshold_addr,
			&data->device_control.wakeup_threshold,
			sizeof(u8));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write wakeup threshold.\n");
	return retval;

}

static ssize_t rmi_fn_01_doze_holdoff_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.doze_holdoff);

}


static ssize_t rmi_fn_01_doze_holdoff_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct f01_data *data = NULL;
	unsigned long new_value;
	int retval;

	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value < 0 || new_value > 255) {
		dev_err(dev, "%s: Invalid doze holdoff %s.", __func__, buf);
		return -EINVAL;
	}

	data->device_control.doze_interval = new_value;
	retval = rmi_write_block(fc->rmi_dev, data->doze_holdoff_addr,
			&data->device_control.doze_holdoff,
			sizeof(u8));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write doze holdoff.\n");
	return retval;

}

static ssize_t rmi_fn_01_configured_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.ctrl0.configured);
}

static ssize_t rmi_fn_01_unconfigured_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_status.unconfigured);
}

static ssize_t rmi_fn_01_flashprog_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_status.flash_prog);
}

static ssize_t rmi_fn_01_statuscode_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct f01_data *data = NULL;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n",
			data->device_status.status_code);
}

//ASUS_BSP simpson: add to do init_config for FnXX +++
#define F01_RMI_QUERY18			0x00BC	/*  Defined firmware Packrat ID  */
#define F34_FLASH_CTRL00		0x0059	/*  Customer Defined Config ID  */
#define F54_ANALOG_CTRL00		0x010D	/*  General Control  */
#define F54_ANALOG_CTRL03		0x0111	/*  Pixel Touch Threshold  */
#define F54_ANALOG_CTRL08		0x0116	/*  Integration Duration  */
#define F54_AD_CMD				0x0172

void rmi_driver_init_cfg(struct rmi_function_container *fc, u8 rw)
{
	int retval = 0;
	unsigned char F01_fw_id[3];
	unsigned int PR_ID;
	unsigned char F34_cfg_id[4];
	unsigned char F54_Touch_Thre = 0x40;
	unsigned short F54_Duration = 137;
	unsigned char Duration_buf[2];
	unsigned char F54_FroceUpdate = 0x04;

	retval = rmi_read_block(fc->rmi_dev, F01_RMI_QUERY18, F01_fw_id, ARRAY_SIZE(F01_fw_id));
	PR_ID = F01_fw_id[0] | (F01_fw_id[1] << 8) | (F01_fw_id[2] << 16);
	rmi_debug(DEBUG_INFO, "[touch_synaptics] RMI Packrat ID = %d\n", PR_ID);
	retval = rmi_read_block(fc->rmi_dev, F34_FLASH_CTRL00, F34_cfg_id, ARRAY_SIZE(F34_cfg_id));
	rmi_debug(DEBUG_INFO, "[touch_synaptics] RMI Config ID = %02x%02x%02x%02x\n", F34_cfg_id[0], F34_cfg_id[1], F34_cfg_id[2], F34_cfg_id[3]);
	if (rw)
		retval = rmi_write_block(fc->rmi_dev, F54_ANALOG_CTRL03, (u8 *)&F54_Touch_Thre, sizeof(F54_Touch_Thre));
	retval = rmi_read_block(fc->rmi_dev, F54_ANALOG_CTRL03, (u8 *)&F54_Touch_Thre, sizeof(F54_Touch_Thre));
	rmi_debug(DEBUG_INFO, "[touch_synaptics] F54_Touch_Thre = %d\n", F54_Touch_Thre);
	Duration_buf[0] = F54_Duration & 0xFF;
	Duration_buf[1] = (F54_Duration & 0xFF00) >> 8;
	if (rw)
		retval = rmi_write_block(fc->rmi_dev, F54_ANALOG_CTRL08, Duration_buf, ARRAY_SIZE(Duration_buf));
	retval = rmi_read_block(fc->rmi_dev, F54_ANALOG_CTRL08, Duration_buf, ARRAY_SIZE(Duration_buf));
	F54_Duration = Duration_buf[0] | (Duration_buf[1] << 8);
	rmi_debug(DEBUG_INFO, "[touch_synaptics] F54_Duration = %d\n", F54_Duration);

	if (rw){
		retval = rmi_write_block(fc->rmi_dev, F54_AD_CMD, (u8 *)&F54_FroceUpdate, sizeof(F54_FroceUpdate));
		rmi_debug(DEBUG_INFO, "[touch_synaptics] F54_Force_Update!!\n");
	}

}
EXPORT_SYMBOL(rmi_driver_init_cfg);
//ASUS_BSP simpson: add to do init_config for FnXX ---
//ASUS_BSP simpson: add to reset device +++
static int rmi_f01_sw_reset(struct rmi_function_container *fc)
{
	int retval = 0;
	/* Command register always reads as 0, so we can just use a local. */
	union f01_device_commands commands = {};

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_reset ++\n");
	/* Per spec, 0 has no effect, so we skip it entirely. */
	commands.reset = 1;
	retval = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,
			&commands.reg, sizeof(commands.reg));
	if (retval < 0) {
		dev_err(&fc->dev, "%s: failed to issue reset command, "
			"error = %d.", __func__, retval);
		return retval;
	}
	//msleep(300);
	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_reset --\n");

	return retval;
}
EXPORT_SYMBOL(rmi_f01_sw_reset);
//ASUS_BSP simpson: add to reset device ---
//ASUS_BSP simpson: add for fast relax +++
#ifdef FFR
static int rmi_f01_fast_relax(struct rmi_function_container *fc)
{
	int retval = 0;
	unsigned char F54_ForceFastRelax_old;
	unsigned char F54_ForceFastRelax = 0x24;
	unsigned char F54_FroceUpdate = 0x04;

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_fast_relax-on ++\n");

	retval = rmi_read_block(fc->rmi_dev, F54_ANALOG_CTRL00, (u8 *)&F54_ForceFastRelax_old, sizeof(F54_ForceFastRelax_old));
	rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] F54_ForceFastRelax_old = %d\n", F54_ForceFastRelax_old);
	retval = rmi_write_block(fc->rmi_dev, F54_ANALOG_CTRL00, (u8 *)&F54_ForceFastRelax, sizeof(F54_ForceFastRelax));
	retval = rmi_read_block(fc->rmi_dev, F54_ANALOG_CTRL00, (u8 *)&F54_ForceFastRelax, sizeof(F54_ForceFastRelax));
	rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] F54_ForceFastRelax_new = %d\n", F54_ForceFastRelax);
	retval = rmi_write_block(fc->rmi_dev, F54_AD_CMD, (u8 *)&F54_FroceUpdate, sizeof(F54_FroceUpdate));
	rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] F54_Force_Update!!\n");

	if(delayed_work_pending(&fast_relax_work)) {
		cancel_delayed_work_sync(&fast_relax_work);
	}
	queue_delayed_work(fast_relax_workqueue, &fast_relax_work, HZ*2.5);

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_fast_relax-on --\n");

	return retval;
}
static void fast_relax_off(struct work_struct *work){
	int retval = 0;
	unsigned char F54_ForceFastRelax_old;
	unsigned char F54_ForceFastRelax = 0x20;
	unsigned char F54_FroceUpdate = 0x04;

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_fast_relax-off ++\n");
	retval = rmi_read_block(g_fc->rmi_dev, F54_ANALOG_CTRL00, (u8 *)&F54_ForceFastRelax_old, sizeof(F54_ForceFastRelax_old));
	rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] F54_ForceFastRelax_old = %d\n", F54_ForceFastRelax_old);
	retval = rmi_write_block(g_fc->rmi_dev, F54_ANALOG_CTRL00, (u8 *)&F54_ForceFastRelax, sizeof(F54_ForceFastRelax));
	retval = rmi_read_block(g_fc->rmi_dev, F54_ANALOG_CTRL00, (u8 *)&F54_ForceFastRelax, sizeof(F54_ForceFastRelax));
	rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] F54_ForceFastRelax_new = %d\n", F54_ForceFastRelax);
	retval = rmi_write_block(g_fc->rmi_dev, F54_AD_CMD, (u8 *)&F54_FroceUpdate, sizeof(F54_FroceUpdate));
	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_fast_relax-off --\n");

}
#endif
//ASUS_BSP simpson: add for fast relax ---
/* why is this not done in init? */
int rmi_driver_f01_init(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *driver_data = rmi_get_driverdata(rmi_dev);
	struct rmi_function_container *fc = driver_data->f01_container;
	int error;

	error = rmi_f01_alloc_memory(fc, driver_data->num_of_irq_regs);
	if (error < 0)
		goto error_exit;

	error = rmi_f01_initialize(fc);
	if (error < 0)
		goto error_exit;

	error = rmi_f01_create_sysfs(fc);
	if (error < 0)
		goto error_exit;

	rmi_driver_init_cfg(fc,0);	//ASUS_BSP simpson: add to do init_config for FnXX ++
//	rmi_driver_init_cfg(fc,1);	//ASUS_BSP simpson: add to do init_config for FnXX ++

	return 0;

 error_exit:
	rmi_f01_free_memory(fc);

	return error;
}

static int rmi_f01_alloc_memory(struct rmi_function_container *fc,
	int num_of_irq_regs)
{
	struct f01_data *f01;

	f01 = kzalloc(sizeof(struct f01_data), GFP_KERNEL);
	if (!f01) {
		dev_err(&fc->dev, "Failed to allocate fn_01_data.\n");
		return -ENOMEM;
	}

	f01->device_control.interrupt_enable =
		kzalloc(sizeof(u8)*(num_of_irq_regs), GFP_KERNEL);
	if (!f01->device_control.interrupt_enable) {
		kfree(f01);
		return -ENOMEM;
	}
	fc->data = f01;

	return 0;
}

static void rmi_f01_free_memory(struct rmi_function_container *fc)
{
	struct f01_data *f01 = fc->data;
	kfree(f01->device_control.interrupt_enable);
	kfree(fc->data);
	fc->data = NULL;
}

//ASUS_BSP simpson: add for On/Off touch in P03 +++
#ifdef CONFIG_EEPROM_NUVOTON
static int touch_mp_event(struct notifier_block *this, unsigned long event, void *ptr);

static struct notifier_block touch_mp_notifier = {
        .notifier_call = touch_mp_event,
        .priority = TOUCH_MP_NOTIFY,
};
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP simpson: add for On/Off touch in P03 ---

static int rmi_f01_initialize(struct rmi_function_container *fc)
{
	u8 temp;
	int retval;
	int ctrl_base_addr;
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_driver_data *driver_data = rmi_get_driverdata(rmi_dev);
	struct f01_data *data = fc->data;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

//ASUS_BSP simpson: add for On/Off touch in P03 & suspend/resume +++
	g_fc = fc;
//ASUS_BSP simpson: add for On/Off touch in P03 & suspend/resume ---

	/* Set the configured bit and (optionally) other important stuff
	 * in the device control register. */
	ctrl_base_addr = fc->fd.control_base_addr;
	retval = rmi_read_block(rmi_dev, fc->fd.control_base_addr,
			&data->device_control.ctrl0.reg,
			sizeof(data->device_control.ctrl0.reg));
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to read F01 control.\n");
		return retval;
	}
	switch (pdata->power_management.nosleep) {
	case RMI_F01_NOSLEEP_DEFAULT:
		break;
	case RMI_F01_NOSLEEP_OFF:
		data->device_control.ctrl0.nosleep = 0;
		break;
	case RMI_F01_NOSLEEP_ON:
		data->device_control.ctrl0.nosleep = 1;
		break;
	}
	/* Sleep mode might be set as a hangover from a system crash or
	 * reboot without power cycle.  If so, clear it so the sensor
	 * is certain to function.
	 */
	if (data->device_control.ctrl0.sleep_mode != RMI_SLEEP_MODE_NORMAL) {
		dev_warn(&fc->dev,
			 "WARNING: Non-zero sleep mode found. Clearing...\n");
		data->device_control.ctrl0.sleep_mode = RMI_SLEEP_MODE_NORMAL;
	}

	data->device_control.ctrl0.configured = 1;
	retval = rmi_write_block(rmi_dev, fc->fd.control_base_addr,
			&data->device_control.ctrl0.reg,
			sizeof(data->device_control.ctrl0.reg));
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to write F01 control.\n");
		return retval;
	}

	data->irq_count = driver_data->irq_count;
	data->num_of_irq_regs = driver_data->num_of_irq_regs;
	ctrl_base_addr += sizeof(struct f01_device_control_0);

	data->interrupt_enable_addr = ctrl_base_addr;
	retval = rmi_read_block(rmi_dev, ctrl_base_addr,
			data->device_control.interrupt_enable,
			sizeof(u8) * (driver_data->num_of_irq_regs));
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to read F01 control interrupt enable register.\n");
		goto error_exit;
	}
	ctrl_base_addr += (sizeof(u8) * (driver_data->num_of_irq_regs));

	/* dummy read in order to clear irqs */
	retval = rmi_read(rmi_dev, fc->fd.data_base_addr + 1, &temp);
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to read Interrupt Status.\n");
		return retval;
	}

	retval = rmi_read_block(rmi_dev, fc->fd.query_base_addr,
				data->basic_queries.regs,
				sizeof(data->basic_queries.regs));
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to read device query registers.\n");
		return retval;
	}

	retval = rmi_read_block(rmi_dev,
		fc->fd.query_base_addr + sizeof(data->basic_queries.regs),
		data->product_id, RMI_PRODUCT_ID_LENGTH);
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to read product ID.\n");
		return retval;
	}
	data->product_id[RMI_PRODUCT_ID_LENGTH] = '\0';
	dev_info(&fc->dev, "found RMI device, manufacturer: %s, product: %s\n",
		 data->basic_queries.manufacturer_id == 1 ?
							"synaptics" : "unknown",
		 data->product_id);

	/* read control register */
	if (data->basic_queries.has_adjustable_doze) {
		data->doze_interval_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.doze_interval) {
			data->device_control.doze_interval =
				pdata->power_management.doze_interval;
			retval = rmi_write(rmi_dev, data->doze_interval_addr,
					data->device_control.doze_interval);
			if (retval < 0) {
				dev_err(&fc->dev, "Failed to configure F01 doze interval register.\n");
				goto error_exit;
			}
		} else {
			retval = rmi_read(rmi_dev, data->doze_interval_addr,
					&data->device_control.doze_interval);
			if (retval < 0) {
				dev_err(&fc->dev, "Failed to read F01 doze interval register.\n");
				goto error_exit;
			}
		}

		data->wakeup_threshold_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.wakeup_threshold) {
			data->device_control.wakeup_threshold =
				pdata->power_management.wakeup_threshold;
			retval = rmi_write(rmi_dev, data->wakeup_threshold_addr,
					data->device_control.wakeup_threshold);
			if (retval < 0) {
				dev_err(&fc->dev, "Failed to configure F01 wakeup threshold register.\n");
				goto error_exit;
			}
		} else {
			retval = rmi_read(rmi_dev, data->wakeup_threshold_addr,
					&data->device_control.wakeup_threshold);
			if (retval < 0) {
				dev_err(&fc->dev, "Failed to read F01 wakeup threshold register.\n");
				goto error_exit;
			}
		}
	}

	if (data->basic_queries.has_adjustable_doze_holdoff) {
		data->doze_holdoff_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.doze_holdoff) {
			data->device_control.doze_holdoff =
				pdata->power_management.doze_holdoff;
			retval = rmi_write(rmi_dev, data->doze_holdoff_addr,
					data->device_control.doze_holdoff);
			if (retval < 0) {
				dev_err(&fc->dev, "Failed to configure F01 "
					"doze holdoff register.\n");
				goto error_exit;
			}
		} else {
			retval = rmi_read(rmi_dev, data->doze_holdoff_addr,
					&data->device_control.doze_holdoff);
			if (retval < 0) {
				dev_err(&fc->dev, "Failed to read F01 doze"
					" holdoff register.\n");
				goto error_exit;
			}
		}
	}

	retval = rmi_read_block(rmi_dev, fc->fd.data_base_addr,
			&data->device_status.reg,
			sizeof(data->device_status.reg));
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to read device status.\n");
		goto error_exit;
	}

	if (data->device_status.unconfigured) {
		dev_err(&fc->dev,
			"Device reset during configuration process, status: "
			"%#02x!\n", data->device_status.status_code);
		retval = -EINVAL;
		goto error_exit;
	}
//ASUS_BSP simpson: add for On/Off touch in P03 +++
	g_rmi_wq_resume_check = create_singlethread_workqueue("g_rmi_wq_resume_check");
	if (!g_rmi_wq_resume_check) {
		rmi_debug(DEBUG_INFO, "[touch_synaptics] %s: create workqueue failed: g_rmi_wq_resume_check\n", __func__);
	}
	INIT_DELAYED_WORK(&g_mp_resume_chk_work, rmi_resume_check_work);
#ifdef CONFIG_EEPROM_NUVOTON
	g_rmi_wq_attach_detach = create_singlethread_workqueue("g_rmi_wq_attach_detach");
	if (!g_rmi_wq_attach_detach) {
		rmi_debug(DEBUG_INFO, "[touch_synaptics] %s: create workqueue failed: g_rmi_wq_attach_detach\n", __func__);
	}
	INIT_WORK(&g_mp_attach_work, attach_padstation_work);
	//INIT_WORK(&g_mp_detach_work, detach_padstation_work);
	INIT_DELAYED_WORK(&g_mp_detach_work, detach_padstation_work);

	register_microp_notifier(&touch_mp_notifier);
//	notify_register_microp_notifier(&touch_mp_notifier, "rmi_f01"); //ASUS_BSP Lenter+
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP simpson: add for On/Off touch in P03 ---

	return retval;

 error_exit:
	kfree(data);
	return retval;
}

static int rmi_f01_create_sysfs(struct rmi_function_container *fc)
{
	int attr_count = 0;
	int retval = 0;
	struct f01_data *data = fc->data;

	dev_dbg(&fc->dev, "Creating sysfs files.");
	for (attr_count = 0; attr_count < ARRAY_SIZE(fn_01_attrs);
			attr_count++) {
		if (!strcmp("doze_interval", fn_01_attrs[attr_count].attr.name)
			&& !data->basic_queries.has_lts) {
			continue;
		}
		if (!strcmp("wakeup_threshold",
			fn_01_attrs[attr_count].attr.name)
			&& !data->basic_queries.has_adjustable_doze) {
			continue;
		}
		if (!strcmp("doze_holdoff", fn_01_attrs[attr_count].attr.name)
			&& !data->basic_queries.has_adjustable_doze_holdoff) {
			continue;
		}
		retval = sysfs_create_file(&fc->dev.kobj,
				      &fn_01_attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&fc->dev, "Failed to create sysfs file for %s.",
			       fn_01_attrs[attr_count].attr.name);
			goto err_remove_sysfs;
		}
	}

	return 0;

err_remove_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&fc->dev.kobj,
				  &fn_01_attrs[attr_count].attr);

	return retval;
}

static int rmi_f01_config(struct rmi_function_container *fc)
{
	struct f01_data *data = fc->data;
	int retval;

	retval = rmi_write_block(fc->rmi_dev, fc->fd.control_base_addr,
			(u8 *)&data->device_control.ctrl0,
			sizeof(struct f01_device_control_0));
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to write device_control.reg.\n");
		return retval;
	}

	retval = rmi_write_block(fc->rmi_dev, data->interrupt_enable_addr,
			data->device_control.interrupt_enable,
			sizeof(u8)*(data->num_of_irq_regs));

	if (retval < 0) {
		dev_err(&fc->dev, "Failed to write interrupt enable.\n");
		return retval;
	}
	if (data->basic_queries.has_lts) {
		retval = rmi_write_block(fc->rmi_dev, data->doze_interval_addr,
				&data->device_control.doze_interval,
				sizeof(u8));
		if (retval < 0) {
			dev_err(&fc->dev, "Failed to write doze interval.\n");
			return retval;
		}
	}

	if (data->basic_queries.has_adjustable_doze) {
		retval = rmi_write_block(
				fc->rmi_dev, data->wakeup_threshold_addr,
				&data->device_control.wakeup_threshold,
				sizeof(u8));
		if (retval < 0) {
			dev_err(&fc->dev, "Failed to write wakeup threshold.\n");
			return retval;
		}
	}

	if (data->basic_queries.has_adjustable_doze_holdoff) {
		retval = rmi_write_block(fc->rmi_dev, data->doze_holdoff_addr,
				&data->device_control.doze_holdoff,
				sizeof(u8));
		if (retval < 0) {
			dev_err(&fc->dev, "Failed to write doze holdoff.\n");
			return retval;
		}
	}
	return 0;
}

static int rmi_f01_reset(struct rmi_function_container *fc)
{
	/*do nothing here */
	return 0;
}


#ifdef CONFIG_PM
#define RETRY_MAX 4
extern int do_initial_reset(struct rmi_device *rmi_dev);
static int rmi_f01_clear_irqs(struct rmi_function_container *fc);
static void rmi_resume_check_work(struct work_struct *work)
{
	struct rmi_driver_data *driver_data = rmi_get_driverdata(g_fc->rmi_dev);
	struct f01_data *data = driver_data->f01_container->data;
	int retry;
	static int blocked=0;

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_resume_check_work()++\n");

	for(retry=0; retry<RETRY_MAX && g_XY == 0xFFFFFFFF && !data->suspended; retry++) {
		rmi_debug(DEBUG_INFO, "[touch_synaptics] resume_check(%d).\n",retry);
		if(retry<3){
			rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] resume_check(clear-irqs)\n");
			rmi_f01_clear_irqs(g_fc);
		}else if(retry<4){
			rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] resume_check(sw_reset)\n");
			rmi_f01_sw_reset(g_fc);
		}else{
			rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] resume_check(init_reset)\n");
			do_initial_reset(g_fc->rmi_dev);
		}
		ASUSEvtlog("[touch_synaptics] rmi_resume_check_work-reset(%d).\n",retry);
		msleep((retry+1)*1000);
	}
	if((g_XY == 0xFFFFFFFF)&&(retry==RETRY_MAX)){
		if(blocked>=2){
			rmi_debug(DEBUG_INFO, "[touch_synaptics] still no activity on screen!(init_reset)\n");
			do_initial_reset(g_fc->rmi_dev);
			blocked=0;
		} else {
			blocked++;
		}
	} else {
		blocked=0;
	}
	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_resume_check_work(%d)--\n", blocked);
}

static int rmi_f01_suspend(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_driver_data *driver_data = rmi_get_driverdata(rmi_dev);
	struct f01_data *data = driver_data->f01_container->data;
	int retval = 0;

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_suspend(%d) ++\n", data->suspended);
	//force turn-off keypad_backlight ++
	if(delayed_work_pending(&keypad_off_work)) {
		cancel_delayed_work_sync(&keypad_off_work);
	}
	rmi_led_trigger_event(keypad_led_trigger, 0);
	rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] F01 force turn-off keypad_backlight\n");
	//force turn-off keypad_backlight --
	dev_dbg(&fc->dev, "Suspending...\n");
	if (data->suspended)
		return 0;

	data->old_nosleep = data->device_control.ctrl0.nosleep;
	data->device_control.ctrl0.nosleep = 0;
	data->device_control.ctrl0.sleep_mode = RMI_SLEEP_MODE_SENSOR_SLEEP;

	retval = rmi_write_block(rmi_dev,
			driver_data->f01_container->fd.control_base_addr,
			(u8 *)&data->device_control.ctrl0,
			sizeof(struct f01_device_control_0));
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to write sleep mode. Code: %d.\n",
			retval);
		data->device_control.ctrl0.nosleep = data->old_nosleep;
		data->device_control.ctrl0.sleep_mode = RMI_SLEEP_MODE_NORMAL;
	} else {
		data->suspended = true;
		retval = 0;
	}
	if(delayed_work_pending(&g_mp_resume_chk_work)) {
		cancel_delayed_work_sync(&g_mp_resume_chk_work);
	}
	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_suspend(%d) --\n", data->suspended);

	return retval;
}

static int rmi_f01_resume(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct rmi_driver_data *driver_data = rmi_get_driverdata(rmi_dev);
	struct f01_data *data = driver_data->f01_container->data;
	int retval = 0;

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_resume(%d) ++\n", data->suspended);
	dev_dbg(&fc->dev, "Resuming...\n");
	if (!data->suspended)
		return 0;

	data->device_control.ctrl0.nosleep = data->old_nosleep;
	data->device_control.ctrl0.sleep_mode = RMI_SLEEP_MODE_NORMAL;

	retval = rmi_write_block(rmi_dev,
			driver_data->f01_container->fd.control_base_addr,
			(u8 *)&data->device_control.ctrl0,
			sizeof(struct f01_device_control_0));
	if (retval < 0)
		dev_err(&fc->dev,
			"Failed to restore normal operation. Code: %d.\n",
			retval);
	else {
		data->suspended = false;
		retval = 0;
	}

	rmi_driver_init_cfg(fc,0);	//ASUS_BSP simpson: add to do init_config for FnXX ++
//	rmi_driver_init_cfg(fc,1);	//ASUS_BSP simpson: add to do init_config for FnXX ++
	g_XY = 0xFFFFFFFF ;
	g_resumeLog = 30 ;
	//ASUS_BSP simpson: add for ASUSEvtlog +++
	ASUSEvtlog("[touch_synaptics] rmi_f01_resume(%d) --\n", data->suspended);
	//ASUS_BSP simpson: add for ASUSEvtlog ---
	//rmi_f01_sw_reset(g_fc);
	if(delayed_work_pending(&g_mp_resume_chk_work)) {
		cancel_delayed_work_sync(&g_mp_resume_chk_work);
	}
	queue_delayed_work(g_rmi_wq_resume_check, &g_mp_resume_chk_work, msecs_to_jiffies(500));
	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_resume(%d) --\n", data->suspended);

	return retval;
}
#endif /* CONFIG_PM */

static int rmi_f01_init(struct rmi_function_container *fc)
{
	return 0;
}

static void rmi_f01_remove(struct rmi_function_container *fc)
{
	int attr_count;

	for (attr_count = 0; attr_count < ARRAY_SIZE(fn_01_attrs);
			attr_count++) {
		sysfs_remove_file(&fc->dev.kobj, &fn_01_attrs[attr_count].attr);
	}

	rmi_f01_free_memory(fc);
}

static int rmi_f01_attention(struct rmi_function_container *fc, u8 *irq_bits)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f01_data *data = fc->data;
	int retval;

	retval = rmi_read_block(rmi_dev, fc->fd.data_base_addr,
		&data->device_status.reg,
	sizeof(data->device_status.reg));
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to read device status, code: %d.\n",
			retval);
		return retval;
	}
	if (data->device_status.unconfigured) {
		dev_warn(&fc->dev, "Device reset detected.\n");
		retval = rmi_dev->driver->reset_handler(rmi_dev);
		if (retval < 0)
			return retval;
	}
	return 0;
}

static struct rmi_function_handler function_handler = {
	.func = 0x01,
	.init = rmi_f01_init,
	.config = rmi_f01_config,
	.reset = rmi_f01_reset,
	.attention = rmi_f01_attention,
#ifdef	CONFIG_PM

#ifdef CONFIG_HAS_EARLYSUSPEND
//ASUS_BSP simpson: add for suspend/resume +++
/*--register_early_suspend by other way--*/
//	.early_suspend = rmi_f01_suspend,
//	.late_resume = rmi_f01_resume,
//ASUS_BSP simpson: add for suspend/resume ---
#else
	.suspend = rmi_f01_suspend,
	.resume = rmi_f01_resume,
#endif  /* CONFIG_HAS_EARLYSUSPEND */
#endif  /* CONFIG_PM */
	.remove = rmi_f01_remove,
};
//ASUS_BSP simpson: add for suspend/resume +++
#ifdef CONFIG_HAS_EARLYSUSPEND
static void rmi_f01_early_suspend(struct early_suspend *h)
{
	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_early_suspend ++\n");

	if ( g_bFoneInPad == false ){
		rmi_f01_suspend(g_fc);
	} else {
		rmi_debug(DEBUG_INFO, "[touch_synaptics] PadAttached!\n");
	//force turn-off keypad_backlight ++
	if(delayed_work_pending(&keypad_off_work)) {
		cancel_delayed_work_sync(&keypad_off_work);
	}
	rmi_led_trigger_event(keypad_led_trigger, 0);
	rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] F01 force turn-off keypad_backlight\n");
	//force turn-off keypad_backlight --
	}

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_early_suspend --\n");
}

static void rmi_f01_late_resume(struct early_suspend *h)
{
	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_late_resume ++\n");

	if ( g_bFoneInPad == false ){
		rmi_f01_resume(g_fc);
	} else {
		rmi_debug(DEBUG_INFO, "[touch_synaptics] PadAttached!\n");
	}

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_late_resume --\n");
}

static struct early_suspend rmi_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = rmi_f01_early_suspend,
    .resume = rmi_f01_late_resume,
};
#endif
//ASUS_BSP simpson: add for suspend/resume ---

static int __init rmi_f01_module_init(void)
{
	int error;

	error = rmi_register_function_driver(&function_handler);
	
// ASUS_BSP : add for A80 miniporting ++
	if (g_A68_hwID >= A80_EVB)
	{
		rmi_debug(DEBUG_INFO, "[touch_synaptics] Disable rmi_f01_module_init in A80\n");
		error = -EINVAL ;
	}
// ASUS_BSP : add for A80 miniporting --

	if (error < 0) {
		pr_err("%s: register failed!\n", __func__);
		return error;
	}
//ASUS_BSP simpson: add for suspend/resume +++
#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend( &rmi_early_suspend_desc );
#endif
//ASUS_BSP simpson: add for suspend/resume ---

//ASUS_BSP simpson: add for fast relax +++
#ifdef FFR
	fast_relax_workqueue = create_singlethread_workqueue("FFR_wq");
	INIT_DELAYED_WORK(&fast_relax_work, fast_relax_off);
#endif
//ASUS_BSP simpson: add for fast relax ---

	return 0;
}

static void __exit rmi_f01_module_exit(void)
{
	destroy_workqueue(g_rmi_wq_resume_check);
#ifdef CONFIG_EEPROM_NUVOTON
	destroy_workqueue(g_rmi_wq_attach_detach);
	unregister_microp_notifier(&touch_mp_notifier);
//	notify_unregister_microp_notifier(&touch_mp_notifier, "rmi_f01"); //ASUS_BSP Lenter+
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP simpson: add for suspend/resume +++
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend( &rmi_early_suspend_desc );
#endif
//ASUS_BSP simpson: add for suspend/resume ---
	rmi_unregister_function_driver(&function_handler);
}

module_init(rmi_f01_module_init);
module_exit(rmi_f01_module_exit);

//ASUS_BSP simpson: add for On/Off touch in P03 +++
#ifdef CONFIG_EEPROM_NUVOTON
static int rmi_f01_clear_irqs(struct rmi_function_container *fc)
{
	u8 temp;
	int retval;
	struct rmi_device *rmi_dev = fc->rmi_dev;

	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_clear_irqs++\n");
	/* dummy read in order to clear irqs */
	retval = rmi_read(rmi_dev, fc->fd.data_base_addr + 1, &temp);
	if (retval < 0) {
		dev_err(&fc->dev, "Failed to read Interrupt Status.\n");
		return retval;
	}
	rmi_debug(DEBUG_INFO, "[touch_synaptics] rmi_f01_clear_irqs--\n");

	return retval;
}

static void attach_padstation_work(struct work_struct *work)
{
	rmi_debug(DEBUG_INFO, "[touch_synaptics] attach_padstation_work()++\n");
	g_bFoneInPad = true;

	rmi_f01_suspend(g_fc);

	g_bIsSuspended = true;
	//ASUS_BSP simpson: add for ASUSEvtlog +++
	ASUSEvtlog("[touch_synaptics] attach_padstation_work finished.");
	//ASUS_BSP simpson: add for ASUSEvtlog ---
	rmi_debug(DEBUG_INFO, "[touch_synaptics] attach_padstation_work()--\n");
}

static void detach_padstation_work(struct work_struct *work)
{
	rmi_debug(DEBUG_INFO, "[touch_synaptics] detach_padstation_work()++\n");

	//msleep(10);
	rmi_f01_resume(g_fc);
	//rmi_f01_sw_reset(g_fc);
#ifdef FFR
	rmi_f01_fast_relax(g_fc);
#endif
	rmi_f01_clear_irqs(g_fc);

	g_bFoneInPad = false;
	g_bIsSuspended = false;
	//ASUS_BSP simpson: add for ASUSEvtlog +++
	ASUSEvtlog("[touch_synaptics] detach_padstation_work finished.");
	//ASUS_BSP simpson: add for ASUSEvtlog ---
	rmi_debug(DEBUG_INFO, "[touch_synaptics] detach_padstation_work()--\n");
}

int touch_attach_padstation(void)
{
	rmi_debug(DEBUG_INFO, "[touch_synaptics] touch_attach_padstation()++\n");

	if(delayed_work_pending(&g_mp_detach_work)) {
		cancel_delayed_work_sync(&g_mp_detach_work);
	rmi_debug(DEBUG_VERBOSE, "[touch_synaptics] cancel last detach_work\n");
	}
	queue_work(g_rmi_wq_attach_detach, &g_mp_attach_work);

	rmi_debug(DEBUG_INFO, "[touch_synaptics] touch_attach_padstation()--\n");

	return 0;
}
EXPORT_SYMBOL(touch_attach_padstation);

int touch_detach_padstation(void)
{
	rmi_debug(DEBUG_INFO, "[touch_synaptics] touch_detach_padstation()++\n");

	//queue_work(g_rmi_wq_attach_detach, &g_mp_detach_work);
	if(delayed_work_pending(&g_mp_detach_work)) {
		cancel_delayed_work_sync(&g_mp_detach_work);
	}
	queue_delayed_work(g_rmi_wq_attach_detach, &g_mp_detach_work, msecs_to_jiffies(2000));

	rmi_debug(DEBUG_INFO, "[touch_synaptics] touch_detach_padstation()--\n");

	return 0;
}
EXPORT_SYMBOL(touch_detach_padstation);

static int touch_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	printk("%s ++, event=%d\r\n", __FUNCTION__, (int)event);
        switch (event) {

        case P01_ADD:
                rmi_debug(DEBUG_INFO, "[touch_synaptics][MicroP] P01_ADD++\n");
		//ASUS_BSP simpson: add for ASUSEvtlog +++
		ASUSEvtlog("[touch_synaptics][MicroP] P03_ADDED");
		//ASUS_BSP simpson: add for ASUSEvtlog ---

                touch_attach_padstation();

                rmi_debug(DEBUG_INFO, "[touch_synaptics][MicroP] P01_ADD--\n");

                break;
        case P01_REMOVE:
                rmi_debug(DEBUG_INFO, "[touch_synaptics][MicroP] P01_REMOVE++\n");
		//ASUS_BSP simpson: add for ASUSEvtlog +++
		ASUSEvtlog("[touch_synaptics][MicroP] P03_REMOVED");
		//ASUS_BSP simpson: add for ASUSEvtlog ---

                touch_detach_padstation();

                rmi_debug(DEBUG_INFO, "[touch_synaptics][MicroP] P01_REMOVE--\n");

               break; 
        default:

                
			break;
        }
	printk("%s --, event=%d\r\n", __FUNCTION__, (int)event);
	return NOTIFY_DONE;
}
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP simpson: add for On/Off touch in P03 ---

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI F01 module");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
