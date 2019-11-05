/* 
 * drivers/input/touchscreen/wdt8913.c
 *
 * Weida Hi-Tech wdt8913 TouchScreen driver. 
 *
 * Copyright (c) 2015 Weida Hi-Tech
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

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/ioc4.h>
#include <linux/io.h>
#include <linux/module.h> 
#include <linux/proc_fs.h>
#include <asm/unaligned.h>
#include <linux/input/mt.h>
#include <linux/version.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#define		EXYNOS5			0 	// exynos5410
#define		ROCKCHIP		1	// RK3188/3288
#define		ALLWINNER		0	// AW A83

#if ROCKCHIP
#define		ROCKCHIP_3288_OF	1	// RK3288 device tree	
#endif

#if ROCKCHIP	
#define		I2C_MASTER_CLK		400 * 1000
#endif

#define		I2C_RP_ST		0x01
#define 	WDT_COMMAND_DELAY_MS	2

#if 	ALLWINNER
#include <linux/init-input.h>
#endif


/* define for samsung 5260 platform */
#if	EXYNOS5
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>
#include <mach/gpio.h>
#endif

#define WDT8913_NAME			"wdt8913"
#define WDT8913_DRV_VER			"0.9.3"

#define WDT8913_MAX_POINTS		10
#define WDT8913_MAX_X			32768
#define WDT8913_MAX_Y			32768

#define	C_FW_CMD_SET_DEVICE_MODE	0xBB

/* for read and write memory, auto-increment */
#define C_ISP_CMD_SET_MEM_ADDRESS	0xC0
#define C_ISP_CMD_READ_WORDS		0xC1
#define C_ISP_CMD_READ_HALFWORDS	0xC2
#define C_ISP_CMD_READ_BYTES		0xC3
#define C_ISP_CMD_WRITE_WORDS		0xC4
#define C_ISP_CMD_WRITE_HALFWORDS	0xC5
#define C_ISP_CMD_WRITE_BYTES		0xC6

/* Support in vA IC only. Use this commmand to identify the protocol version */
#define C_ISP_CMD_READ_GP_REGISTER	0xC7
#define C_ISP_CMD_READ_DEVICE_INFO	0xC8

/* for read and write, auto-increment */
#define C_ISP_CMD_SET_FLASH_ADDRESS	0xD0
#define C_ISP_CMD_READ_FLASH		0xD1
#define C_ISP_CMD_WRITE_FLASH		0xD2
#define C_ISP_CMD_ERASE_FLASH		0xD3

/* write protection on/off */
#define C_ISP_CMD_LOCK_FLASH		0xD4

/* enable flash interface */
#define C_ISP_CMD_ENABLE_FLASH		0xD5
#define C_ISP_CMD_CHECKSUM_FLASH	0xD6

#define C_ISP_CMD_RESET_CONNECTION	0xDA
#define C_ISP_CMD_CALL_FUNCTION		0xDB
#define C_ISP_CMD_RUN_PROGRAM		0xDC
#define C_ISP_CMD_PING			0xDD

/* success */
#define	C_ISP_RSP_OK			0x80	

/* Invalid address */
#define	C_ISP_RSP_INVALID_ADRESS	0xF0	

/* Invalid count */
#define	C_ISP_RSP_INVALID_COUNT		0xF1

/* Timeout (not enough bytes received)	*/
#define	C_ISP_RSP_TIMEOUT		0xF2

/* Invalid command */
#define	C_ISP_RSP_INVALID_COMMAND	0xF3

/* Invalid flash region */
#define	C_ISP_RSP_INVALID_REGION	0xF4

/* Invalid key */
#define	C_ISP_RSP_INVALID_KEY		0xF5

/* Invalid function  call address */
#define	C_ISP_RSP_INVALID_FUNC_ADDRESS	0xF6

/* Flash busy (callback function) */
#define	C_ISP_RSP_BUSY			0xFE

/* mp test pass */
#define C_MP_RSP_PASS			0x81

/* mp test fail */
#define C_MP_RSP_FAIL			0x8F

#define	FLASH_BATCH_READ_SIZE		30
#define	FLASH_BATCH_WRITE_SIZE		31
#define	MEMORY_BYTE_BATCH_READ_SIZE	30
#define	MEMORY_BYTE_BATCH_WRITE_SIZE 	30

#define VND_REQ_FW_VER			0xF2
#define PARAM_RX_LEN			36

#define	ROMSIGNATURE0			0x81CD3D87
#define	ROMSIGNATURE1			0xC3DD6F81

#if	ALLWINNER
static struct ctp_config_info config_info = {
	.input_type = CTP_TYPE,
	.name = NULL,
	.int_number = 0,
};

static const unsigned short weida_i2c[2] = {0x2C, I2C_CLIENT_END};
#endif


/*
 * BOOTLOADER only exists during ROM and fastboot. It cannot be set as the
 * target mode. SENSING is Normal sensing. DOZE is that device sleeps most time
 * and does sensing in a period. SLEEP is that both device and sensor are in
 * sleep. FACTORY is for MP testing. The device cannot switch to other modes
 * unless reboot it. When entering COMMAND mode, device is waiting for commands.
 * Do flash/memory access commands mostly.
 */
typedef enum {
	BOOTLOADER = 0,
	SENSING = 1,
	DOZE = 2,
	SLEEP = 3,
	FACTORY = 0x80, 
	COMMAND = 0x90,
} device_mode_t;

struct devinfo {
	u8 rpt_id_dev_info;
	u32 fw_id;
	u32 hw_id;
	u32 serial_no;
	u8 n_touches;
	u8 n_bytes;
	u32 reserved0;
	u32 reserved1;
	u8 reserved2;
	u32 rom_sig0;
	u32 rom_sig1;
};

struct wdt8913 {
	struct i2c_client *client;
	struct input_dev *input;
	bool isp_mode;
	bool tip_flags[10];
	bool exiting;
	struct devinfo	dev_info;
};

static int wdt8913_i2c_rx(struct i2c_client *client,
			    void *rxdata, size_t rxlen)
{
	struct i2c_msg msg = {
		.addr		= client->addr,
		.flags		= I2C_M_RD,
		.len		= rxlen,
		.buf		= rxdata,
#if ROCKCHIP
		//.scl_rate	= I2C_MASTER_CLK,
#if !ROCKCHIP_3288_OF
		.udelay 	= client->udelay,
#endif
#endif
	};
	int ret;
	
	ret = i2c_transfer(client->adapter, &msg, 1);
	
	return (ret == 1) ? rxlen : ret;
}

static int wdt8913_i2c_tx(struct i2c_client *client,
			    void *txdata, size_t txlen)
{
	struct i2c_msg msg = {
		.addr		= client->addr,
		.flags		= 0,
		.len		= txlen,
		.buf		= txdata,
#if ROCKCHIP
		//.scl_rate	= I2C_MASTER_CLK,
#if !ROCKCHIP_3288_OF
		.udelay 	= client->udelay,
#endif
#endif
	};
	int ret;
	
	ret = i2c_transfer(client->adapter, &msg, 1);
	
	return (ret == 1) ? txlen : ret;
}

static int wdt8913_i2c_xfer(struct i2c_client *client,
			    void *txdata, size_t txlen,
			    void *rxdata, size_t rxlen)
{
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= txlen,
			.buf	= txdata,
#if ROCKCHIP
			//.scl_rate	= I2C_MASTER_CLK,
#if !ROCKCHIP_3288_OF
			.udelay 	= client->udelay,
#endif
#endif
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= rxlen,
			.buf	= rxdata,
#if ROCKCHIP
			//.scl_rate	= I2C_MASTER_CLK,
#if !ROCKCHIP_3288_OF
			.udelay 	= client->udelay,
#endif
#endif
		},
	};
	int error;
	int ret;
	int array_sz = 1;
#if ROCKCHIP
	/* Repeat start isn't supported by RK */
	int flag = 0;
#else
	int flag = I2C_RP_ST;
#endif
	/* A special command for retrieve i2c cfg */
	if (msgs[0].buf[4] == 0xf4)
		flag &= ~I2C_RP_ST;	

	if (flag & I2C_RP_ST) {
		array_sz = 2;
		ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	} else {
		ret = i2c_transfer(client->adapter, &msgs[0], 1);
		if (ret == array_sz) {
			mdelay(WDT_COMMAND_DELAY_MS);
			ret = i2c_transfer(client->adapter, &msgs[1], 1);
		}
	} 

	if (ret != array_sz) {
		error = ret < 0 ? ret : -EIO;
		dev_err(&client->dev, "%s: i2c transfer failed: %d\n",
			__func__, error);
		return error;
	}

	mdelay(WDT_COMMAND_DELAY_MS);

	return 0;
}

static int wdt8913_wait_complete_and_return_data(struct i2c_client * client,
						 void * result, int result_size,
						 int polling_interval_us)
{
	u8 	read_buf[32];
	u32	retries = 10;

	read_buf[0] = C_ISP_RSP_BUSY;
	while (retries) {
		int rc;
		
		if (polling_interval_us > 0)
			udelay(polling_interval_us);
		
		if ((rc = wdt8913_i2c_rx(client, read_buf, result_size+1)) > 0) {
			if (read_buf[0] == C_ISP_RSP_OK) {
				if (result_size > 0)
					memcpy(result, read_buf+1, result_size);
				return rc;
			}
			else if (read_buf[0] == C_ISP_RSP_BUSY)
				retries --;
			else if (read_buf[0] == C_MP_RSP_PASS)
				return C_MP_RSP_PASS;
			else if (read_buf[0] == C_MP_RSP_FAIL)
				return C_MP_RSP_FAIL;
			else
				return -EINVAL;	
		}
	}	

	dev_err(&client->dev, "access device failed.\n");
	return -EIO;
}

static int wdt8913_write_command(struct i2c_client * client, u8 cmd[],
				 int size, int polling_interval_us)
{
	if (wdt8913_i2c_tx(client, cmd, size) >= 0)
		return wdt8913_wait_complete_and_return_data(client, NULL, 0,
							     polling_interval_us);
	
	return -EIO;	
}

static int wdt8913_set_device_mode(struct i2c_client * client, device_mode_t mode)
{
	if (mode == COMMAND) {
		u8 cmd[] = { C_FW_CMD_SET_DEVICE_MODE, COMMAND, 0x73, 0x50 };

		wdt8913_write_command(client, cmd, sizeof(cmd), 0);
	} else if (mode == FACTORY) {
		u8 cmd[] = { C_FW_CMD_SET_DEVICE_MODE, FACTORY, 0xad, 0xb1 };

		wdt8913_write_command(client, cmd, sizeof(cmd), 0);
	} else	{
		u8 cmd[] = { C_FW_CMD_SET_DEVICE_MODE, mode };

		wdt8913_write_command(client, cmd, sizeof(cmd), 0);
	}

	return 0;
}

static int wdt8913_get_param(struct i2c_client * client)
{
	/* 0xF2 feature command to get fw parameters */
	u8 tx_buf[] = { 0x22, 0x00, 0x3F, 0x02, VND_REQ_FW_VER, 0x23, 0x00 };
	u8 rx_buf[PARAM_RX_LEN]; 
	int error;
	int rx_len;
	struct wdt8913 *wdt = i2c_get_clientdata(client);
	struct devinfo *info = &wdt->dev_info;

	error = wdt8913_i2c_xfer(client, tx_buf, sizeof(tx_buf), rx_buf, PARAM_RX_LEN);
	if (error) {
		dev_err(&client->dev, "get feature failed: (%d)\n", error);
		return error;
	}

	rx_len = get_unaligned_le16(rx_buf);
	if (rx_len > PARAM_RX_LEN) {
		dev_err(&client->dev, "feature length failed: (%d)\n", rx_len);
		return -EINVAL;
	}
	
	if (rx_buf[2] != VND_REQ_FW_VER) {
		dev_err(&client->dev, "wrong id of fw response: 0x%x\n", rx_buf[2]);
		return -EINVAL;
	}

	info->fw_id = get_unaligned_le32(&rx_buf[3]);
	info->hw_id = get_unaligned_le32(&rx_buf[7]);
	info->serial_no = get_unaligned_le32(&rx_buf[11]);
	info->n_touches = rx_buf[15];
	info->n_bytes = rx_buf[16];
	info->rom_sig0 = get_unaligned_le32(&rx_buf[26]);
	info->rom_sig1 = get_unaligned_le32(&rx_buf[30]);
	
	dev_info(&client->dev, "fw_id: 0x%x, hw_id: 0x%x, ser_no: 0x%x\n",
		 info->fw_id, info->hw_id, info->serial_no);
	dev_info(&client->dev, "n_touches: %d, n_bytes: %d\n",
		 info->n_touches, info->n_bytes);
	dev_info(&client->dev, "rom_sig0: 0x%x, rom_sig1: 0x%x\n",
		 info->rom_sig0, info->rom_sig1);

	if ( ((info->fw_id & 0xF0000000) != 0x00000000) ||
	     (info->rom_sig0 != ROMSIGNATURE0 || 
	      info->rom_sig1 != ROMSIGNATURE1) ) {
		dev_err(&client->dev, "not wdt8913 device !\n");
		return -ENODEV;
	}

	return 0;
}

static int wdt8913_report_touch_info(struct wdt8913 * tsdata)
{
	struct i2c_client * client = tsdata->client;
	u8 cmd[] = { 0x90 };
	u8 data[3];
	int n_touches=0;
	int retry;

	if (wdt8913_i2c_tx(client, cmd, sizeof(cmd)) == sizeof(cmd)) {
		retry = 5;
		while (retry-- > 0) {
			udelay(100);
			if (wdt8913_i2c_rx(client, data, 3) == 3)
				break;
		}

		if (retry == 0)
			return -1;	

		/*
		 * data[0] is the finger nuber 
		 */
		n_touches = data[0]; 
	}

	return n_touches;
}
static int wdt8913_report_touch(struct wdt8913 * tsdata, int n_touches)
{
	struct i2c_client * client = tsdata->client;
	u8 cmd[] = {0x91};
	u8 data[50];
	int i;
	int retry;

	if (n_touches > WDT8913_MAX_POINTS) {
		printk("%s: max fingers exceed 10: (%d)\n", __func__, n_touches);
		return -EINVAL;
	}

	if (wdt8913_i2c_tx(client, cmd, sizeof(cmd)) == sizeof(cmd) ) {
		retry = 5;
		while (retry-- > 0) {
			udelay(100);
			if (wdt8913_i2c_rx(client, data, n_touches*5) ==
			    (n_touches*5))
				break;
		}

		if (retry == 0)
			return -1;	

		for (i=0; i<n_touches; i++) {
			int x, y, id, flag;
			input_mt_slot(tsdata->input, i);
			flag = data[5*i];

			id = (flag >> 3) - 1;	

			flag = flag & 1;
			if (id < 0 || id >= WDT8913_MAX_POINTS)
				continue;

			x = WDT8913_MAX_X-get_unaligned_le16(&(data[ 5*i + 1]));
			y = WDT8913_MAX_Y-get_unaligned_le16(&(data[ 5*i + 3]));

			if (flag == 0 && tsdata->tip_flags[id] != false) {
				input_mt_slot(tsdata->input, id);
				input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, false);
				tsdata->tip_flags[id] = false;
			} else {
				input_mt_slot(tsdata->input, id);
				input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, true);
				input_report_abs(tsdata->input, ABS_MT_POSITION_X, x);
				input_report_abs(tsdata->input, ABS_MT_POSITION_Y, y);	
				tsdata->tip_flags[id] = true;
			}
		}
		
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0)
		input_mt_report_pointer_emulation(tsdata->input, 0);
#else
		input_mt_sync_frame(tsdata->input);
#endif
		/* to mark the end of this transfer */
		input_sync(tsdata->input);
		return n_touches;
	}
	return -1;
	
}

static irqreturn_t wdt8913_isr(int irq, void *dev_id)
{
	struct wdt8913 *tsdata = dev_id;
	int n_touches = 0;

	n_touches = wdt8913_report_touch_info(tsdata);
	wdt8913_report_touch(tsdata,n_touches);
	return IRQ_HANDLED;
}

#if	ROCKCHIP
#if	ROCKCHIP_3288_OF
static int wdt8913_handle_of(struct i2c_client *client)
{
	int error;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	enum of_gpio_flags rst_flags;
	unsigned long irq_flags;
	int value, reset_pin;
	
	if (of_property_read_u32(np, "screen_max_x", &value))
		dev_info(&client->dev, "no screen_max_x defined\n");

	if (of_property_read_u32(np, "screen_max_y", &value))
		dev_info(&client->dev, "no screen_max_y defined\n");

	client->irq = of_get_named_gpio_flags(np, "touch_irq_gpio", 0,
					      (enum of_gpio_flags *)&irq_flags);
	reset_pin = of_get_named_gpio_flags(np, "touch_reset_gpio", 0,
					    &rst_flags);
	printk("-------wdt8913_handle_of------\n");
	if(reset_pin)
	{
     	printk("-------wdt8913_handle_of------\n");	
	    gpio_direction_output(reset_pin, 0);
        msleep(300);
	    gpio_direction_output(reset_pin, 1);
        msleep(300);					
	}
	error = gpio_request(client->irq, "GPIO INT");
	if (error < 0) {
		dev_err(&client->dev, "%s: request gpio fail (%d)\n",
			__func__, error);
		return -EINVAL;
	}

	dev_info(&client->dev, "irq gpio num: (%d)\n", client->irq);

	client->irq = gpio_to_irq(client->irq);
	if (client->irq < 0) {
		dev_err(&client->dev, "%s: request irq fail (%d)\n",
			__func__, client->irq);
		return -EINVAL;
	}

	dev_info(&client->dev, "client irq num: (%d)\n", client->irq);
    printk("------wdt8913_handle_of---:%d\n",error);
	return error;
}
#endif
#endif


static int wdt8913_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	dev_info(dev, "suspend\n");

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);
	
	wdt8913_set_device_mode(client, SLEEP);

	return 0;
}

static int wdt8913_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	dev_info(dev, "resume\n");

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	wdt8913_set_device_mode(client, SENSING);

	return 0;
}

static int wdt8913_setup_irq(struct i2c_client *client)
{
	int error = 0;

#if EXYNOS5
	client->irq = EINT_NUM;
	
	error = gpio_request(FT_INT_PORT, "EINT");
	if (error < 0) {		
		dev_err(&client->dev, "Request gpio fail (%d)\n", error);
		return error;
	}

	/* pull high to wait the irq notification */
	s3c_gpio_setpull(FT_INT_PORT, S3C_GPIO_PULL_UP);	
#endif

#if ROCKCHIP
#if	ROCKCHIP_3288_OF
    return 0;
	return wdt8913_handle_of(client);
#else
	client->irq = gpio_to_irq(client->irq);
	if (client->irq < 0) {
		dev_err(&client->dev, "%s: request irq fail (%d)\n",
			__func__, client->irq);
		return -EINVAL;
	}	
	dev_info(&client->dev, "client irq mapped: %d\n", client->irq);
#endif
#endif

#if ALLWINNER
	client->irq = gpio_to_irq(config_info.int_number);
	if (IS_ERR_VALUE(client->irq)) {
		dev_err(&client->dev, "%s: request irq fail (%d)\n",
			__func__, client->irq);
		return -EINVAL;
	}
	dev_info(&client->dev, "client irq mapped: %d\n", client->irq);
#endif
	return error;
}

static int wdt8913_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct wdt8913 *tsdata;
	struct input_dev *input;
	int error;

	dev_info(dev, "probe\n");
    printk("------wdt8913_probe-----\n");
	wdt8913_handle_of(client);	
    printk("------wdt8913_probe1-----\n");
		
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "No I2C!\n");
		return -ENODEV;
	}

	tsdata = devm_kzalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);
	memset(&(tsdata->tip_flags), 0, sizeof(tsdata->tip_flags));

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	input = input_allocate_device();
#else
	input = devm_input_allocate_device(dev);
#endif
	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	
	if (!tsdata || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	tsdata->client = client;
	tsdata->input = input;
	input_set_drvdata(input, tsdata);
	i2c_set_clientdata(client, tsdata);

	error = wdt8913_get_param(client);
	if (error < 0) {
		dev_err(dev, "get param failed!\n");
		goto err_free_mem;
	}	

	dev_info(dev, "input dev init\n");

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_mt_init_slots(input, WDT8913_MAX_POINTS);
#else
	input_mt_init_slots(input, WDT8913_MAX_POINTS,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
#endif

	set_bit(ABS_MT_POSITION_X, input->absbit);
	set_bit(ABS_MT_POSITION_Y, input->absbit);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, WDT8913_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, WDT8913_MAX_Y, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, WDT8913_MAX_POINTS, 0, 0);

	set_bit(EV_ABS, input->evbit);
	set_bit(EV_SYN,	input->evbit);
	
	error = wdt8913_setup_irq(client);
	if (error)
		return error;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
	set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING);	
#else
	irq_set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING);	
#endif

	if (client->irq <= 0) {
		dev_err(dev, "No IRQ!\n");
		return -EINVAL;
	}

	error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
//					  wdt8913_isr, IRQF_TRIGGER_FALLING,
					  wdt8913_isr, IRQF_ONESHOT,
					  client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err_free_mem;
	}

	dev_info(dev, "register input dev\n");
	error = input_register_device(input);
	if (error) 
		goto err_free_irq;

	dev_info(dev, "probe done\n");
	return 0;
err_free_irq:
	//free_irq(client->irq, tsdata);
err_free_mem:
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	input_free_device(input);
#endif	
	return error;
}

static int wdt8913_remove(struct i2c_client *client)
{
	struct wdt8913 *tsdata = i2c_get_clientdata(client);

	dev_info(&client->dev, "remove\n");

	device_init_wakeup(&client->dev, 0);

	tsdata->exiting = true;
	mb();

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	input_unregister_device(tsdata->input);
#endif
	return 0;
}

static const struct i2c_device_id wdt8913_id[] = {
	{ WDT8913_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, wdt8913_id);

static SIMPLE_DEV_PM_OPS(wdt8913_pm_ops, wdt8913_suspend, wdt8913_resume);

static struct i2c_driver wdt8913_driver = {
	.probe		= wdt8913_probe,
	.remove		= wdt8913_remove,
	.id_table	= wdt8913_id,
	.driver	= {
		.name	= WDT8913_NAME,
		.owner	= THIS_MODULE,
		.pm	= &wdt8913_pm_ops,
	},
#if	ALLWINNER
	.address_list	= weida_i2c,
#endif	
};

#if	ALLWINNER
static void ctp_print_info(struct ctp_config_info info,int debug_level)
{
	printk("info.ctp_used: %d\n", info.ctp_used);
	printk("info.twi_id: %d\n", info.twi_id);
	printk("info.screen_max_x: %d\n", info.screen_max_x);
	printk("info.screen_max_y: %d\n", info.screen_max_y);
	printk("info.revert_x_flag: %d\n", info.revert_x_flag);
	printk("info.revert_y_flag: %d\n", info.revert_y_flag);
	printk("info.exchange_x_y_flag: %d\n", info.exchange_x_y_flag);
	printk("info.irq_gpio_number: %d\n", info.irq_gpio.gpio);
	printk("info.wakeup_gpio_number: %d\n", info.wakeup_gpio.gpio);
	printk("info.int_number: %d\n", info.int_number);
}

static int ctp_get_system_config(void)
{   
        ctp_print_info(config_info, 0);

        if((config_info.screen_max_x == 0) || (config_info.screen_max_y == 0)) {
                printk("%s:read config error!\n",__func__);
                return -1;
        }
        return 0;
}

static struct i2c_board_info 	i2c_weida_info	__initdata = 
{
	I2C_BOARD_INFO(WDT8913_NAME, 0x2C),
};
#endif


static int __init wdt8913_init(void)
{
	int ret = 0;
	
#if	ALLWINNER
	struct i2c_adapter	*padapter;
	struct i2c_client	*pclient;

	printk("enter %s\n", __func__);

	if (input_fetch_sysconfig_para(&(config_info.input_type))) {
		printk("%s: fetch sysconfig para err.\n", __func__);
		return 0;
	} else {
		ret = input_init_platform_resource(&(config_info.input_type));
		if (ret) 
			printk("%s:init platform_resource err.\n", __func__);    
	}

	if (!config_info.ctp_used) {
	        printk("ctp_used set to 0 !\n");
	        printk("please set ctp_used to 1 in sys_config.fex.\n");
	        return 0;
	}

        if(ctp_get_system_config()) {
                printk("%s: read config fail!\n", __func__);
                return -1;
        }

	input_set_power_enable(&(config_info.input_type), 1);

	mdelay(10);

	ret = i2c_add_driver(&wdt8913_driver);
	if (ret) {
		printk("failed to add i2c driver\n");
		return ret;
	}

	padapter = i2c_get_adapter(config_info.twi_id);
	if (padapter == NULL) {
		printk("failed to get adapter (%d)\n", config_info.twi_id);
		return -ENODEV;
	}

	pclient = i2c_new_device(padapter, &i2c_weida_info);

	if (pclient == NULL) {
		printk("failed to get client");
		return -ENODEV;
	}

	return 0;
#else	
	printk("%s init\n", WDT8913_NAME);
	ret = i2c_add_driver(&wdt8913_driver);
	printk("ret = %d\n",ret);

	return ret;
#endif	
}

static void __exit wdt8913_exit(void)
{
	printk(KERN_INFO"%s exit\n", WDT8913_NAME);
	i2c_del_driver(&wdt8913_driver);
	
#if	ALLWINNER
	input_free_platform_resource(&(config_info.input_type));
#endif	
}


//late_initcall(wdt8913_init);
module_init(wdt8913_init);
module_exit(wdt8913_exit);

MODULE_AUTHOR("<victor.chang@weidahitech.com>");
MODULE_DESCRIPTION("Weida Hi-Tech WDT8913 touch driver");
MODULE_VERSION(WDT8913_DRV_VER);
MODULE_LICENSE("GPL");

