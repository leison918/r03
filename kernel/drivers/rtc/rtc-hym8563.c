/* drivers/rtc/rtc-HYM8563.c - driver for HYM8563
 *
 * Copyright (C) 2010 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

//#define DEBUG
#define pr_fmt(fmt) "rtc: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include "rtc-HYM8563.h"
#include <linux/of_gpio.h>
#include <linux/irqdomain.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_ARCH_ROCKCHIP
#include <asm/system_misc.h>
#endif
//extern atomic_t sys_poweroff_flag;

#define RTC_SPEED 	200 * 1000
#define open_8563_mic 1

//#define GPIO7_A4 228
//#define GPIO7_A2 226

/**********R03************/
#define GPIO8_A0 248 //FP42
#define GPIO8_A1 249 //FP53
#define MCU_IO_INIT GPIO8_A0
#define MCU_IO_DET  GPIO8_A1


/**********R08************/
//#define GPIO4_D3    1155
//#define GPIO4_C6    1150
//#define MCU_IO_INIT GPIO4_D3
//#define MCU_IO_DET  GPIO4_C6


/**********R35************/
/*#define GPIO7_A4 220
#define GPIO7_A2 218
#define MCU_IO_INIT GPIO7_A4
#define MCU_IO_DET  GPIO7_A2*/

static struct delayed_work r_delayed_work;
static struct delayed_work i_delayed_work;

struct timer_list mtimer; 
static int powerflag;

struct hym8563 {
	int irq;
	struct i2c_client *client;
	struct mutex mutex;
	struct rtc_device *rtc;
	struct rtc_wkalrm alarm;
	struct wake_lock wake_lock;     
	/* add time poweroff */ 
	int status;
	int voltage_low;
	struct rtc_time  onalarm;	
};
static struct i2c_client *gClient = NULL;
struct hym8563 *pcf8563 = NULL;
struct hym8563 *mt8563 = NULL;
struct i2c_client *mclient = NULL;
static int ontimeflag=0;
struct rtc_time ontime;
u8 mval;
static int poweroff=0;

static int i2c_master_reg8_send(const struct i2c_client *client, const char reg, const char *buf, int count, int scl_rate)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int ret;
	char *tx_buf = (char *)kzalloc(count + 1, GFP_KERNEL);
	if(!tx_buf)
		return -ENOMEM;
	tx_buf[0] = reg;
	memcpy(tx_buf+1, buf, count); 

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count + 1;
	msg.buf = (char *)tx_buf;
	//msg.scl_rate = scl_rate;

	ret = i2c_transfer(adap, &msg, 1);
	kfree(tx_buf);
	return (ret == 1) ? count : ret;

}

static int i2c_master_reg8_recv(const struct i2c_client *client, const char reg, char *buf, int count, int scl_rate)
{
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msgs[2];
	int ret;
	char reg_buf = reg;
	
	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len = 1;
	msgs[0].buf = &reg_buf;
	//msgs[0].scl_rate = scl_rate;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = count;
	msgs[1].buf = (char *)buf;
	//msgs[1].scl_rate = scl_rate;

	ret = i2c_transfer(adap, msgs, 2);

	return (ret == 2)? count : ret;
}



static int hym8563_i2c_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret; 
	ret = i2c_master_reg8_recv(client, reg, buf, len, RTC_SPEED);
	return ret; 
}

static int hym8563_i2c_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], __u16 len)
{
	int ret; 
	ret = i2c_master_reg8_send(client, reg, buf, (int)len, RTC_SPEED);
	return ret;
}


int hym8563_enable_count(struct i2c_client *client, int en)
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);	
	u8 regs[2];

	if (!hym8563)
		return -1;

	if (en) {
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
		regs[0] |= TIE;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		regs[0] = 0;
		regs[0] |= (TE | TD1);
		hym8563_i2c_set_regs(client, RTC_T_CTL, regs, 1);
	}
	else {
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
		regs[0] &= ~TIE;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		regs[0] = 0;
		regs[0] |= (TD0 | TD1);
		hym8563_i2c_set_regs(client, RTC_T_CTL, regs, 1);
	}
	return 0;
}

//0 < sec <=255
int hym8563_set_count(struct i2c_client *client, int sec)
{	
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[2];

	if (!hym8563)
		return -1;
		
	if (sec >= 255)
		regs[0] = 255;
	else if (sec <= 1)
		regs[0] = 1;
	else
		regs[0] = sec;
	
	hym8563_i2c_set_regs(client, RTC_T_COUNT, regs, 1);
	
	return 0;
}


/*the init of the hym8563 at first time */
static int hym8563_init_device(struct i2c_client *client)	
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[2];
	int sr;

	mutex_lock(&hym8563->mutex);
	regs[0]=0;
	sr = hym8563_i2c_set_regs(client, RTC_CTL1, regs, 1);		
	if (sr < 0)
		goto exit;
	
	//disable clkout
	regs[0] = 0x80;
	sr = hym8563_i2c_set_regs(client, RTC_CLKOUT, regs, 1);
	if (sr < 0)
		goto exit;

	/*enable alarm && count interrupt*/
	sr = hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	if (sr < 0)
		goto exit;
	/*regs[0] = 0x0;
	regs[0] |= (AIE | TIE);
	sr = hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	if (sr < 0)
		goto exit;
	sr = hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	if (sr < 0)
		goto exit;*/

	sr = hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	if (sr < 0) {
		pr_err("read CTL2 err\n");
		goto exit;
	}
	
	if(regs[0] & (AF|TF))
	{
		regs[0] &= ~(AF|TF);
		sr = hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	}
	
exit:
	mutex_unlock(&hym8563->mutex);
	
	return sr;
}

static int hym8563_read_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[HYM8563_RTC_SECTION_LEN] = { 0, };
	mutex_lock(&hym8563->mutex);
//	for (i = 0; i < HYM8563_RTC_SECTION_LEN; i++) {
//		hym8563_i2c_read_regs(client, RTC_SEC+i, &regs[i], 1);
//	}
	hym8563_i2c_read_regs(client, RTC_SEC, regs, HYM8563_RTC_SECTION_LEN);

	mutex_unlock(&hym8563->mutex);
	
	tm->tm_sec = bcd2bin(regs[0x00] & 0x7F);
	tm->tm_min = bcd2bin(regs[0x01] & 0x7F);
	tm->tm_hour = bcd2bin(regs[0x02] & 0x3F);
	tm->tm_mday = bcd2bin(regs[0x03] & 0x3F);
	tm->tm_wday = bcd2bin(regs[0x04] & 0x07);	
	
	tm->tm_mon = bcd2bin(regs[0x05] & 0x1F) ; 
	tm->tm_mon -= 1;			//inorder to cooperate the systerm time
	
	tm->tm_year = bcd2bin(regs[0x06] & 0xFF);
	if(regs[5] & 0x80)
		tm->tm_year += 1900;
	else
		tm->tm_year += 2000;
		
	tm->tm_year -= 1900;			//inorder to cooperate the systerm time	
	if(tm->tm_year < 0)
		tm->tm_year = 0;	
	tm->tm_isdst = 0;	

	printk("%4d-%02d-%02d(%d) %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday, tm->tm_wday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

static int hym8563_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return hym8563_read_datetime(to_i2c_client(dev), tm);
}

static int hym8563_set_time(struct i2c_client *client, struct rtc_time *tm)	
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[HYM8563_RTC_SECTION_LEN] = { 0, };
	u8 mon_day;
	//u8 ret = 0;

	pr_debug("%4d-%02d-%02d(%d) %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday, tm->tm_wday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	mon_day = rtc_month_days((tm->tm_mon), tm->tm_year + 1900);
	
	if(tm->tm_sec >= 60 || tm->tm_sec < 0 )		//set  sec
		regs[0x00] = bin2bcd(0x00);
	else
		regs[0x00] = bin2bcd(tm->tm_sec);
	
	if(tm->tm_min >= 60 || tm->tm_min < 0 )		//set  min	
		regs[0x01] = bin2bcd(0x00);
	else
		regs[0x01] = bin2bcd(tm->tm_min);

	if(tm->tm_hour >= 24 || tm->tm_hour < 0 )		//set  hour
		regs[0x02] = bin2bcd(0x00);
	else
		regs[0x02] = bin2bcd(tm->tm_hour);
	
	if((tm->tm_mday) > mon_day)				//if the input month day is bigger than the biggest day of this month, set the biggest day 
		regs[0x03] = bin2bcd(mon_day);
	else if((tm->tm_mday) > 0)
		regs[0x03] = bin2bcd(tm->tm_mday);
	else if((tm->tm_mday) <= 0)
		regs[0x03] = bin2bcd(0x01);

	if( tm->tm_year >= 200)		// year >= 2100
		regs[0x06] = bin2bcd(99);	//year = 2099
	else if(tm->tm_year >= 100)			// 2000 <= year < 2100
		regs[0x06] = bin2bcd(tm->tm_year - 100);
	else if(tm->tm_year >= 0){				// 1900 <= year < 2000
		regs[0x06] = bin2bcd(tm->tm_year);	
		regs[0x05] |= 0x80;	
	}else{									// year < 1900
		regs[0x06] = bin2bcd(0);	//year = 1900	
		regs[0x05] |= 0x80;	
	}	
	regs[0x04] = bin2bcd(tm->tm_wday);		//set  the  weekday
	regs[0x05] = (regs[0x05] & 0x80)| (bin2bcd(tm->tm_mon + 1) & 0x7F);		//set  the  month
	
	mutex_lock(&hym8563->mutex);
//	for(i=0;i<HYM8563_RTC_SECTION_LEN;i++){
//		ret = hym8563_i2c_set_regs(client, RTC_SEC+i, &regs[i], 1);
//	}
	hym8563_i2c_set_regs(client, RTC_SEC, regs, HYM8563_RTC_SECTION_LEN);

	mutex_unlock(&hym8563->mutex);

	return 0;
}

static int hym8563_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return hym8563_set_time(to_i2c_client(dev), tm);
}

static int hym8563_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
#ifndef open_8563_mic	
	struct i2c_client *client = to_i2c_client(dev);
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[4] = { 0, };
	
	pr_debug("enter\n");
	mutex_lock(&hym8563->mutex);
	hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);
	regs[0] = 0x0;
	regs[0] |= TIE;
	hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	mutex_unlock(&hym8563->mutex);
#endif	
	return 0;
}

static int hym8563_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{	
#ifndef open_8563_mic
	struct i2c_client *client = to_i2c_client(dev);
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	struct rtc_time now, *tm = &alarm->time;
	u8 regs[4] = { 0, };
	u8 mon_day;	
	unsigned long	alarm_sec, now_sec;
	int diff_sec = 0;
	
	pr_debug("%4d-%02d-%02d(%d) %02d:%02d:%02d enabled %d\n",
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday, tm->tm_wday,
		tm->tm_hour, tm->tm_min, tm->tm_sec, alarm->enabled);
	
	
	hym8563_read_datetime(client, &now);

	
	mutex_lock(&hym8563->mutex);
	rtc_tm_to_time(tm, &alarm_sec);
	rtc_tm_to_time(&now, &now_sec);
	
	diff_sec = alarm_sec - now_sec;
	
	if((diff_sec > 0) && (diff_sec < 256))
	{	
		printk("%s:diff_sec= %ds , use time\n",__func__, diff_sec);	
								
		if (alarm->enabled == 1)
		{
			hym8563_set_count(client, diff_sec);
			hym8563_enable_count(client, 1);
		}
			
		else
		{
			hym8563_enable_count(client, 0);
		}
		
	}
	else
	{				
		printk("%s:diff_sec= %ds , use alarm\n",__func__, diff_sec);
		hym8563_enable_count(client, 0);
		
		if(tm->tm_sec > 0)
		{
			rtc_tm_to_time(tm, &alarm_sec);
			rtc_time_to_tm(alarm_sec, tm);
		}

		hym8563->alarm = *alarm;

		regs[0] = 0x0;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		mon_day = rtc_month_days(tm->tm_mon, tm->tm_year + 1900);
		hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);

		if (tm->tm_min >= 60 || tm->tm_min < 0)		//set  min
		regs[0x00] = bin2bcd(0x00) & 0x7f;
		else
		regs[0x00] = bin2bcd(tm->tm_min) & 0x7f;
		if (tm->tm_hour >= 24 || tm->tm_hour < 0)	//set  hour
		regs[0x01] = bin2bcd(0x00) & 0x7f;
		else
		regs[0x01] = bin2bcd(tm->tm_hour) & 0x7f;
		regs[0x03] = bin2bcd (tm->tm_wday) & 0x7f;

		/* if the input month day is bigger than the biggest day of this month, set the biggest day */
		if (tm->tm_mday > mon_day)
		regs[0x02] = bin2bcd(mon_day) & 0x7f;
		else if (tm->tm_mday > 0)
		regs[0x02] = bin2bcd(tm->tm_mday) & 0x7f;
		else if (tm->tm_mday <= 0)
		regs[0x02] = bin2bcd(0x01) & 0x7f;

		hym8563_i2c_set_regs(client, RTC_A_MIN, regs, 4);	
		hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);	
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
		if (alarm->enabled == 1)
		regs[0] |= AIE;
		else
		regs[0] &= 0x0;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);

		if(diff_sec <= 0)
		{		
			pr_info("alarm sec  <= now sec\n");
		}			

	}
	
	mutex_unlock(&hym8563->mutex);
#endif
	return 0;
}
#ifdef CONFIG_HDMI_SAVE_DATA
int hdmi_get_data(void)
{
    u8 regs=0;
    if(gClient)
        hym8563_i2c_read_regs(gClient, RTC_T_COUNT, &regs, 1);
    else 
    {
        printk("%s rtc has no init\n",__func__);
        return -1;
    }
    if(regs==0 || regs==0xff){
        printk("%s rtc has no hdmi data\n",__func__);
        return -1;
    }
    return (regs-1);
}

int hdmi_set_data(int data)
{
    u8 regs = (data+1)&0xff;
    if(gClient)
        hym8563_i2c_set_regs(gClient, RTC_T_COUNT, &regs, 1);
    else 
    {
        printk("%s rtc has no init\n",__func__);
        return -1;
    }   
    return 0;
}

EXPORT_SYMBOL(hdmi_get_data);
EXPORT_SYMBOL(hdmi_set_data);
#endif
#if defined(CONFIG_RTC_INTF_DEV) || defined(CONFIG_RTC_INTF_DEV_MODULE)
static int hym8563_i2c_open_alarm(struct i2c_client *client)
{
	u8 data;	
	hym8563_i2c_read_regs(client, RTC_CTL2, &data, 1);
	data |= AIE;
	hym8563_i2c_set_regs(client, RTC_CTL2, &data, 1);

	return 0;
}

static int hym8563_i2c_close_alarm(struct i2c_client *client)
{
	u8 data;	
	hym8563_i2c_read_regs(client, RTC_CTL2, &data, 1);
	data &= ~AIE;
	hym8563_i2c_set_regs(client, RTC_CTL2, &data, 1);

	return 0;
}

static int hym8563_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
#ifndef open_8563_mic		
	switch (cmd) {
	case RTC_AIE_OFF:
		if(hym8563_i2c_close_alarm(client) < 0)
			goto err;
		break;
	case RTC_AIE_ON:
		if(hym8563_i2c_open_alarm(client))
			goto err;
		break;
	default:
		return -ENOIOCTLCMD;
	}	
#endif	
	return 0;
err:
	return -EIO;
}
#else
#define hym8563_rtc_ioctl NULL
#endif

#if defined(CONFIG_RTC_INTF_PROC) || defined(CONFIG_RTC_INTF_PROC_MODULE)
static int hym8563_rtc_proc(struct device *dev, struct seq_file *seq)
{
	return 0;
}
#else
#define hym8563_rtc_proc NULL
#endif



static const struct rtc_class_ops hym8563_rtc_ops = {
	.read_time	= hym8563_rtc_read_time,
	.set_time	= hym8563_rtc_set_time,
	.read_alarm	= hym8563_rtc_read_alarm,
	.set_alarm	= hym8563_rtc_set_alarm,
	.ioctl 		= hym8563_rtc_ioctl,
	.proc		= hym8563_rtc_proc
};

#if open_8563_mic
#define	PCF_SET_TIME	1
#define	PCF_RED_TIME	7
#define	PCF_EN_ALARM	3
#define	PCF_DIS_ALARM	4
#define	PCF_SET_ONALARM	5
#define	PCF_RED_ALARM	6
#define	PCF_SET_OFALARM	8
#define PCF_SET_AF      9
#define PCF_POWER      10


#define PCF8563_REG_ST1		0x00 /* status */
#define PCF8563_REG_ST2		0x01

#define PCF8563_REG_SC		0x02 /* datetime */
#define PCF8563_REG_MN		0x03
#define PCF8563_REG_HR		0x04
#define PCF8563_REG_DM		0x05
#define PCF8563_REG_DW		0x06
#define PCF8563_REG_MO		0x07
#define PCF8563_REG_YR		0x08

#define PCF8563_REG_AMN		0x09 /* alarm */
#define PCF8563_REG_AHR		0x0A
#define PCF8563_REG_ADM		0x0B
#define PCF8563_REG_ADW		0x0C

#define PCF8563_REG_CLKO	0x0D /* clock out */
#define PCF8563_REG_TMRC	0x0E /* timer control */
#define PCF8563_REG_TMR		0x0F /* timer */

#define PCF8563_SC_LV		0x80 /* low voltage */
#define PCF8563_MO_C		0x80 /* century */


#define	PCF_SET_TIME	1
#define	PCF_RED_TIME	7
#define	PCF_EN_ALARM	3
#define	PCF_DIS_ALARM	4
#define	PCF_SET_ONALARM	5
#define	PCF_RED_ALARM	6
#define	PCF_SET_OFALARM	8

#define PCF8563_IRQ	1




static int pcf8563_set_datetime_hyman(struct rtc_time *tm)
{
	
	int i, err;
	unsigned char buf[9];
#if 0
	printk( "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* hours, minutes and seconds */
	buf[PCF8563_REG_SC] = bin2bcd(tm->tm_sec);
	buf[PCF8563_REG_MN] = bin2bcd(tm->tm_min);
	buf[PCF8563_REG_HR] = bin2bcd(tm->tm_hour);

	buf[PCF8563_REG_DM] = bin2bcd(tm->tm_mday);

	/* month, 1 - 12 */
	buf[PCF8563_REG_MO] = bin2bcd(tm->tm_mon + 1);

	/* year and century */
	buf[PCF8563_REG_YR] = bin2bcd(tm->tm_year % 100);
	if (pcf8563->c_polarity ? (tm->tm_year >= 100) : (tm->tm_year < 100))
		buf[PCF8563_REG_MO] |= PCF8563_MO_C;

	buf[PCF8563_REG_DW] = tm->tm_wday & 0x07;

	/* write register's data */
	for (i = 0; i < 7; i++) {
		unsigned char data[2] = { PCF8563_REG_SC + i,
						buf[PCF8563_REG_SC + i] };

		err = i2c_master_send(pcf8563->client, data, sizeof(data));
		if (err != sizeof(data)) {
			printk("%s: err=%d addr=%02x, data=%02x\n",__func__, err, data[0], data[1]);
			return -EIO;
		}
	}
#endif
	return 0;
}

void pcf8563_read_datetime_hyman(struct rtc_time *tm)
{
#if 0
	unsigned char buf[13] = { PCF8563_REG_ST1 };

	struct i2c_msg msgs[] = {
		{/* setup read ptr */
			.addr = pcf8563->client->addr,
			.len = 1,
			.buf = buf
		},
		{/* read status + date */
			.addr = pcf8563->client->addr,
			.flags = I2C_M_RD,
			.len = 13,
			.buf = buf
		},
	};

	/* read registers */
	if ((i2c_transfer(pcf8563->client->adapter, msgs, 2)) != 2) {
		printk( "%s: read error\n", __func__);
		return -EIO;
	}

	if (buf[PCF8563_REG_SC] & PCF8563_SC_LV) {
		pcf8563->voltage_low = 1;
		printk(
			"low voltage detected, date/time is not reliable.\n");
	}

	printk(
		"%s: raw data is st1=%02x, st2=%02x, sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, wday=%02x, mon=%02x, year=%02x\n",
		__func__,
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7],
		buf[8]);


	tm->tm_sec = bcd2bin(buf[PCF8563_REG_SC] & 0x7F);
	tm->tm_min = bcd2bin(buf[PCF8563_REG_MN] & 0x7F);
	tm->tm_hour = bcd2bin(buf[PCF8563_REG_HR] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(buf[PCF8563_REG_DM] & 0x3F);
	tm->tm_wday = buf[PCF8563_REG_DW] & 0x07;
	tm->tm_mon = bcd2bin(buf[PCF8563_REG_MO] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(buf[PCF8563_REG_YR]);
	if (tm->tm_year < 70)
		tm->tm_year += 100;	/* assume we are in 1970...2069 */
	/* detect the polarity heuristically. see note above. */
	pcf8563->c_polarity = (buf[PCF8563_REG_MO] & PCF8563_MO_C) ?
		(tm->tm_year >= 100) : (tm->tm_year < 100);

	printk( "%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* the clock can give out invalid datetime, but we cannot return
	 * -EINVAL otherwise hwclock will refuse to set the time on bootup.
	 */
	if (rtc_valid_tm(tm) < 0)
		printk( "retrieved date/time is not valid.\n");
#endif	
}


void pcf8563_read_alarm_hyman(struct rtc_time *tm)
{
#if 0
	unsigned char buf[13] = { PCF8563_REG_ST1 };

	struct i2c_msg msgs[] = {
		{/* setup read ptr */
			.addr = pcf8563->client->addr,
			.len = 1,
			.buf = buf
		},
		{/* read status + date */
			.addr = pcf8563->client->addr,
			.flags = I2C_M_RD,
			.len = 13,
			.buf = buf
		},
	};

	/* read registers */
	if ((i2c_transfer(pcf8563->client->adapter, msgs, 2)) != 2) {
		printk( "%s: read error\n", __func__);
		return -EIO;
	}

	if (buf[PCF8563_REG_SC] & PCF8563_SC_LV) {
		pcf8563->voltage_low = 1;
		printk(
			"low voltage detected, date/time is not reliable.\n");
	}

	printk(
		"%s: raw data is st1=%02x, st2=%02x, sec=%02x, min=%02x, hr=%02x, "
		"mday=%02x, wday=%02x, mon=%02x, year=%02x\n",
		__func__,
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7],
		buf[8]);


	
	tm->tm_min = bcd2bin(buf[PCF8563_REG_AMN] & 0x7F);
	tm->tm_hour = bcd2bin(buf[PCF8563_REG_AHR] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(buf[PCF8563_REG_ADM] & 0x3F);
	tm->tm_wday = buf[PCF8563_REG_ADW] & 0x07;

	printk( "%s: tm is, mins=%d, hours=%d, "
		"mday=%d,, wday=%d\n",
		__func__,
		 tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_wday);
#else
	
	struct i2c_client *client = pcf8563->client;
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[4] = { 0, };
	
	pr_debug("enter\n");
	mutex_lock(&hym8563->mutex);
	hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);
	regs[0] = 0x0;
	regs[0] |= TIE;
	hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	mutex_unlock(&hym8563->mutex);
	return 0;


#endif


}

void pcf8563_set_onalarm_hyman(struct rtc_time *tm)
{
	
	printk("xiao--pcf8563_set_onalarm_hyman\n");
#if 0
	int i, err;
	unsigned char buf[9];

	printk( "%s:  mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		 tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* hours, minutes and seconds */
#if 0	
	buf[PCF8563_REG_AMN - 9] = bin2bcd(tm->tm_min);
	buf[PCF8563_REG_AHR - 9] = bin2bcd(tm->tm_hour);

	buf[PCF8563_REG_ADM - 9] = bin2bcd(tm->tm_mday);

	/* WEEK */
	buf[PCF8563_REG_ADW - 9] = tm->tm_wday & 0x07;



	/* write register's data */
	for (i = 0; i < 4; i++) {
		unsigned char data[2] = { PCF8563_REG_AMN + i,
						buf[PCF8563_REG_AMN - 9  + i] };

		err = i2c_master_send(pcf8563->client, data, sizeof(data));
		if (err != sizeof(data)) {
			printk("%s: err=%d addr=%02x, data=%02x\n",__func__, err, data[0], data[1]);
			return -EIO;
		}
	}
#endif
	pcf8563->onalarm.tm_hour =  tm->tm_hour;
	pcf8563->onalarm.tm_min  =  tm->tm_min;
	pcf8563->onalarm.tm_wday =  tm->tm_wday;
	pcf8563->onalarm.tm_mday =  tm->tm_mday;
	pcf8563->status = 1;
	return 0;
#else

struct i2c_client *client = pcf8563->client;
struct hym8563 *hym8563 = i2c_get_clientdata(client);
struct rtc_time now;
u8 regs[4] = { 0, };
u8 mon_day; 
unsigned long	alarm_sec, now_sec;
int diff_sec = 0;

pr_debug("%4d-%02d-%02d(%d) %02d:%02d:%02d \n",
	1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday, tm->tm_wday,
	tm->tm_hour, tm->tm_min, tm->tm_sec);


hym8563_read_datetime(client, &now);


mutex_lock(&hym8563->mutex);
rtc_tm_to_time(tm, &alarm_sec);
rtc_tm_to_time(&now, &now_sec);

diff_sec = alarm_sec - now_sec;

if((diff_sec > 0) && (diff_sec < 256))
{	
	printk("%s:diff_sec= %ds , use time\n",__func__, diff_sec); 
							
	if ( 1)
	{
		hym8563_set_count(client, diff_sec);
		hym8563_enable_count(client, 1);
	}
		
	else
	{
		hym8563_enable_count(client, 0);
	}
	
}
else
{				
	printk("%s:diff_sec= %ds , use alarm\n",__func__, diff_sec);
	hym8563_enable_count(client, 0);
	
	if(tm->tm_sec > 0)
	{
		rtc_tm_to_time(tm, &alarm_sec);
		rtc_time_to_tm(alarm_sec, tm);
	}

//	hym8563->alarm = *alarm;

	regs[0] = 0x0;
	hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	mon_day = rtc_month_days(tm->tm_mon, tm->tm_year + 1900);
	hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);

	if (tm->tm_min >= 60 || tm->tm_min < 0) 	//set  min
	regs[0x00] = bin2bcd(0x00) & 0x7f;
	else
	regs[0x00] = bin2bcd(tm->tm_min) & 0x7f;
	if (tm->tm_hour >= 24 || tm->tm_hour < 0)	//set  hour
	regs[0x01] = bin2bcd(0x00) & 0x7f;
	else
	regs[0x01] = bin2bcd(tm->tm_hour) & 0x7f;
	regs[0x03] = bin2bcd (tm->tm_wday) & 0x7f;

	/* if the input month day is bigger than the biggest day of this month, set the biggest day */
	if (tm->tm_mday > mon_day)
	regs[0x02] = bin2bcd(mon_day) & 0x7f;
	else if (tm->tm_mday > 0)
	regs[0x02] = bin2bcd(tm->tm_mday) & 0x7f;
	else if (tm->tm_mday <= 0)
	regs[0x02] = bin2bcd(0x01) & 0x7f;

	hym8563_i2c_set_regs(client, RTC_A_MIN, regs, 4);	
	hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);	
	hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	if ( 1)
	regs[0] |= AIE;
	else
	regs[0] &= 0x0;
	hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);

	if(diff_sec <= 0)
	{		
		pr_info("alarm sec	<= now sec\n");
	}			

}

mutex_unlock(&hym8563->mutex);

return 0;


#endif

}

void pcf8563_set_offalarm_hyman(struct rtc_time *tm)
{
	printk("pcf8563_set_offalarm_hyman\n");
	printk( "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);
	struct i2c_client *client = pcf8563->client;
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	struct rtc_time now;
	u8 regs[4] = { 0, };
	u8 mon_day; 
	unsigned long	alarm_sec, now_sec;
	int diff_sec = 0;
	
	printk("%4d-%02d-%02d(%d) %02d:%02d:%02d \n",
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday, tm->tm_wday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
	
	printk("xiao -- test --8563-1\n");
	
	hym8563_read_datetime(client, &now);
	
	printk("xiao -- test --8563-2\n");
	
	printk("%4d-%02d-%02d(%d) %02d:%02d:%02d \n",
		1900 + now.tm_year, now.tm_mon + 1, now.tm_mday, now.tm_wday,
		now.tm_hour, now.tm_min, now.tm_sec);
	mutex_lock(&hym8563->mutex);
	rtc_tm_to_time(tm, &alarm_sec);
	rtc_tm_to_time(&now, &now_sec);
	
	diff_sec = alarm_sec - now_sec;
	
	if(0)
	//if((diff_sec > 0) && (diff_sec < 256))
	{	
		printk("%s:diff_sec= %ds , use time\n",__func__, diff_sec); 
		powerflag=0;
								
		if ( 1)
		{
			hym8563_set_count(client, diff_sec);
			hym8563_enable_count(client, 1);
		}
			
		else
		{
			hym8563_enable_count(client, 0);
		}
		
	}
	else
	{				
		printk("%s:diff_sec= %ds , use alarm\n",__func__, diff_sec);
		powerflag=1;
		hym8563_enable_count(client, 0);
		
		if(tm->tm_sec > 0)
		{
			rtc_tm_to_time(tm, &alarm_sec);
			rtc_time_to_tm(alarm_sec, tm);
		}
	
	//	hym8563->alarm = *alarm;
	
		regs[0] = 0x0;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		mon_day = rtc_month_days(tm->tm_mon, tm->tm_year + 1900);
		hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);
	
		if (tm->tm_min >= 60 || tm->tm_min < 0) 	//set  min
		regs[0x00] = bin2bcd(0x00) & 0x7f;
		else
		regs[0x00] = bin2bcd(tm->tm_min) & 0x7f;
		if (tm->tm_hour >= 24 || tm->tm_hour < 0)	//set  hour
		regs[0x01] = bin2bcd(0x00) & 0x7f;
		else
		regs[0x01] = bin2bcd(tm->tm_hour) & 0x7f;
		regs[0x03] = bin2bcd (tm->tm_wday) & 0x7f;
	
		/* if the input month day is bigger than the biggest day of this month, set the biggest day */
		if (tm->tm_mday > mon_day)
		regs[0x02] = bin2bcd(mon_day) & 0x7f;
		else if (tm->tm_mday > 0)
		regs[0x02] = bin2bcd(tm->tm_mday) & 0x7f;
		else if (tm->tm_mday <= 0)
		regs[0x02] = bin2bcd(0x01) & 0x7f;
	
		hym8563_i2c_set_regs(client, RTC_A_MIN, regs, 4);	
		hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);	
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
		if ( 1)
		regs[0] |= AIE;
		else
		regs[0] &= 0x0;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	
		if(diff_sec <= 0)
		{		
			pr_info("alarm sec	<= now sec\n");
		}			
	
	}
	
	mutex_unlock(&hym8563->mutex);

	/* hours, minutes and seconds */
#if 0	
	buf[PCF8563_REG_AMN - 9] = bin2bcd(tm->tm_min);
	buf[PCF8563_REG_AHR - 9] = bin2bcd(tm->tm_hour);

	buf[PCF8563_REG_ADM - 9] = bin2bcd(tm->tm_mday);

	/* WEEK */
	buf[PCF8563_REG_ADW - 9] = tm->tm_wday & 0x07;



	/* write register's data */
	for (i = 0; i < 4; i++) {
		unsigned char data[2] = { PCF8563_REG_AMN + i,
						buf[PCF8563_REG_AMN - 9  + i] };

		err = i2c_master_send(pcf8563->client, data, sizeof(data));
		if (err != sizeof(data)) {
			printk("%s: err=%d addr=%02x, data=%02x\n",__func__, err, data[0], data[1]);
			return -EIO;
		}
	}
#else
		
	
#endif
}

void pcf8563_en_alarm_hyman(int i)
{
	printk("pcf8563_en_alarm  i = %d \n",i);
	struct i2c_client *client = pcf8563->client;
	u8 data;
#if 1	
	if(i)
	{
			
		hym8563_i2c_read_regs(client, RTC_CTL2, &data, 1);
		data |= AIE;
		hym8563_i2c_set_regs(client, RTC_CTL2, &data, 1);
	
	}
	else{
	
		hym8563_i2c_read_regs(client, RTC_CTL2, &data, 1);
		data &= ~AIE;
		hym8563_i2c_set_regs(client, RTC_CTL2, &data, 1);

	}
#endif	
}

static long pcf8563_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg )
	{
		struct rtc_time tm;	copy_from_user(&tm,arg,sizeof(tm));
		struct i2c_client *client = pcf8563->client;
		printk("%d:%d week = %d cmd = %d \n",tm.tm_hour,tm.tm_min,tm.tm_wday,cmd);
		switch (cmd) {
			case PCF_SET_TIME:
				pcf8563_set_datetime_hyman(&tm);
				return 0;	
			case PCF_SET_OFALARM:
                 ontime=tm;
				 ontimeflag=1;
                 printk("xiao--PCF_SET_OFALARM\n");				 
				//pcf8563_set_offalarm_hyman(&tm);		
				return 0;	
			case PCF_SET_ONALARM:		
				pcf8563_set_onalarm_hyman(&tm);		
				return 0;	
			case PCF_EN_ALARM:		
				pcf8563_en_alarm_hyman(1);		
				return 0;	
			case PCF_DIS_ALARM:		
				pcf8563_en_alarm_hyman(0);		
				return 0;	
			case PCF_RED_ALARM:		
				pcf8563_read_alarm_hyman(&tm);		
				copy_to_user(arg,&tm,sizeof(tm));		
				return 0;
            case PCF_SET_AF:	
				hym8563_set_count(client, 1);
			    hym8563_enable_count(client, 1);
                printk("xiao -- PCF_SET_AF\n"); 
                return 0;	
            case PCF_POWER:
			    printk("xiao -- PCF_POWER\n"); 
                return poweroff;			
			default:		
				return -ENOIOCTLCMD;	
		}
	}


static const struct file_operations pcf8563_misc_fops = {
	.unlocked_ioctl = pcf8563_misc_ioctl,
	};

static struct miscdevice pcf8563_misc = {
	.minor = MISC_DYNAMIC_MINOR,	.name = "pcf8563",
	.fops = &pcf8563_misc_fops,
	};


#endif

static void r_once_work(struct work_struct *work)
{
 	hym8563_i2c_read_regs(mclient, RTC_CTL2, &mval, 1);
	printk("xiao -- hym8563 mval-1=0x%02x\n",mval);
	mdelay(1000);
 gpio_direction_output(10,0);	
 gpio_direction_output(MCU_IO_INIT,0);	
}

static void i_once_work(struct work_struct *work)
{
 gpio_direction_output(MCU_IO_INIT,1);	
 enable_irq_wake(mt8563->irq);
}


/*static void time_handler(unsigned long data) 
{		
	mod_timer(&mtimer, jiffies + HZ);	
	struct i2c_client *client = pcf8563->client;
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	
    //printk("xiao--MCU_IO_INIT set low1\n");		
	//printk("xiao--MCU_IO_INIT set low2\n");		
	//printk("xiao--MCU_IO_INIT set low3\n");	
	if(gpio_get_value(MCU_IO_DET)==1 && ontimeflag==0){
		printk("xiao--MCU_IO_INIT set low4\n");
		pcf8563_set_offalarm_hyman(&ontime);	
		mutex_lock(&hym8563->mutex);
		hym8563_i2c_read_regs(client, RTC_CTL2, &mval, 1);
		mval &= ~(AF|TF);
		hym8563_i2c_set_regs(client, RTC_CTL2, &mval, 1);	
        mutex_unlock(&hym8563->mutex);
		ontimeflag=1;;		
		rtc_update_irq(hym8563->rtc, 1, RTC_IRQF | RTC_AF | RTC_UF);	
	    gpio_direction_output(MCU_IO_INIT,0);
		printk("xiao--MCU_IO_INIT set low8\n");			
	}
}*/

static irqreturn_t hym8563_wakeup_irq(int irq, void *data)
{
	struct hym8563 *hym8563 = data;	
	struct i2c_client *client = hym8563->client;	
	u8 value;
	unsigned long	alarm_sec, now_sec;
	int diff_sec = 0;
	struct rtc_time now;

	disable_irq_nosync(irq);
	printk("xiao--hym8563_wakeup_irq\n");
	//gpio_direction_output(SPK_IO,0);	
	printk("MAC--SPK_IO LOW\n");
	
	hym8563_read_datetime(client, &now);
	rtc_tm_to_time(&ontime, &alarm_sec);
	rtc_tm_to_time(&now, &now_sec);
	
	diff_sec = alarm_sec - now_sec;
	if(diff_sec>0 && ontimeflag==1){
		ontimeflag=0;
	    if(ontime.tm_year!=0){
        pcf8563_set_offalarm_hyman(&ontime);	
		}
	}
	mutex_lock(&hym8563->mutex);
	poweroff=1;
	hym8563_i2c_read_regs(client, RTC_CTL2, &mval, 1);
	printk("xiao -- hym8563 mval-1=0x%02x\n",mval);
	//mval &= ~(AF|TF);
	mval=0x02;
	printk("xiao -- hym8563 mval-2=0x%02x\n",mval);
	hym8563_i2c_set_regs(client, RTC_CTL2, &mval, 1);			
	rtc_update_irq(hym8563->rtc, 1, RTC_IRQF | RTC_AF | RTC_UF);	
	mutex_unlock(&hym8563->mutex);
	printk("xiao--MCU_IO_INIT set low8\n");	
	//gpio_direction_output(10,0);	
	schedule_delayed_work(&r_delayed_work,msecs_to_jiffies(2000));
	//mdelay(2000);
	//gpio_direction_output(MCU_IO_INIT,0);	
	
	return IRQ_HANDLED;
}

static int  hym8563_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	u8 reg = 0;
	struct hym8563 *hym8563;
	struct rtc_device *rtc = NULL;
	struct rtc_time tm_read, tm = {
		.tm_wday = 6,
		.tm_year = 111,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 12,
		.tm_min = 0,
		.tm_sec = 0,
	};	

	struct device_node *np = client->dev.of_node;
	unsigned long irq_flags;
	int result;
	
	ontimeflag=0;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;
		
	hym8563 = devm_kzalloc(&client->dev,sizeof(*hym8563), GFP_KERNEL);
	if (!hym8563) {
		return -ENOMEM;
	}	
	pcf8563 = devm_kzalloc(&client->dev,sizeof(*hym8563), GFP_KERNEL);
	if (!pcf8563) {
		return -ENOMEM;
	}
	pcf8563->client =  client;
	gClient = client;	
	hym8563->client = client;
	hym8563->alarm.enabled = 0;
	client->irq = 0;
	mutex_init(&hym8563->mutex);
	wake_lock_init(&hym8563->wake_lock, WAKE_LOCK_SUSPEND, "rtc_hym8563");
	i2c_set_clientdata(client, hym8563);
	
	gpio_direction_output(MCU_IO_INIT,0);
	gpio_direction_output(10,1);
	gpio_free(MCU_IO_DET); 
	gpio_direction_input(MCU_IO_DET);
	
	//init_timer(&mtimer);
	//mtimer.data = 0;
	//mtimer.expires = jiffies + HZ; //??????
	//mtimer.function = time_handler;
	//add_timer(&mtimer);

	hym8563_init_device(client);	
	hym8563_enable_count(client, 0);	
	
	INIT_DELAYED_WORK(&r_delayed_work, r_once_work);
    INIT_DELAYED_WORK(&i_delayed_work, i_once_work); 	
	schedule_delayed_work(&i_delayed_work,msecs_to_jiffies(2000));
	
	// check power down 
	hym8563_i2c_read_regs(client,RTC_SEC,&reg,1);
	if (reg&0x80) {
		dev_info(&client->dev, "clock/calendar information is no longer guaranteed\n");
		hym8563_set_time(client, &tm);
	}

	hym8563_read_datetime(client, &tm_read);	//read time from hym8563
	
	if(((tm_read.tm_year < 70) | (tm_read.tm_year > 137 )) | (tm_read.tm_mon == -1) | (rtc_valid_tm(&tm_read) != 0)) //if the hym8563 haven't initialized
	{
		hym8563_set_time(client, &tm);	//initialize the hym8563 
	}	
 
    mt8563=hym8563;
	mclient=client;
	
	hym8563_i2c_read_regs(client, RTC_CTL2, &mval, 1);
	printk("xiao -- hym8563 mval-1=0x%02x\n",mval);
	
	mval=0x00;
	hym8563_i2c_set_regs(client, RTC_CTL2, &mval, 1);			
//	rtc_update_irq(hym8563->rtc, 1, RTC_IRQF | RTC_AF | RTC_UF);	
	
	//client->irq = of_get_named_gpio_flags(np, "irq_gpio", 0,(enum of_gpio_flags *)&irq_flags);
	client->irq = MCU_IO_DET;
	if(client->irq >= 0)
        {
	        hym8563->irq = gpio_to_irq(client->irq);
	        result = devm_request_threaded_irq(&client->dev, hym8563->irq, NULL, hym8563_wakeup_irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT, client->dev.driver->name,hym8563 );
	        if (result) {
		        printk(KERN_ERR "%s:fail to request irq = %d, ret = 0x%x\n",__func__, hym8563->irq, result);
		        goto exit;
	        }
	        //enable_irq_wake(hym8563->irq);
	        device_init_wakeup(&client->dev, 1);
        }
	rtc = devm_rtc_device_register(&client->dev,
			client->name,
                       	&hym8563_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		rc = PTR_ERR(rtc);
		rtc = NULL;
		goto exit;
	}
	hym8563->rtc = rtc;

	
#if open_8563_mic
	misc_register(&pcf8563_misc);
#endif 

		
	return 0;

exit:
	if (hym8563) {
		wake_lock_destroy(&hym8563->wake_lock);
	}
	return rc;
}

static int  hym8563_remove(struct i2c_client *client)
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	
	del_timer(&mtimer);

	wake_lock_destroy(&hym8563->wake_lock);

	return 0;
}


void hym8563_shutdown(struct i2c_client * client)
{	u8 regs[2];	
    int ret; 	
	
	//8563 shutdown close irq set poweron time
	printk("---hym8563_shutdown----\n");	
	//power_on_alarm_en();
	//printk("---hym8563_shutdown1----\n");
	
    //disable clkout	
    regs[0] = 0x00;	
    ret=hym8563_i2c_set_regs(client, RTC_CLKOUT, regs, 1);	
    if(ret<0)	
        printk("rtc shutdown is error\n");
}



static const struct i2c_device_id hym8563_id[] = {
	{ "rtc_hym8563", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hym8563_id);

static struct of_device_id rtc_dt_ids[] = {
	{ .compatible = "rtc,hym8563" },
	{},
};

struct i2c_driver hym8563_driver = {
	.driver		= {
		.name	= "rtc_hym8563",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rtc_dt_ids),
	},
	.probe		= hym8563_probe,
	.remove		= hym8563_remove,
	.shutdown= hym8563_shutdown,
	.id_table	= hym8563_id,
};

static int __init hym8563_init(void)
{
    printk("------hym8563_init-------\n");
	return i2c_add_driver(&hym8563_driver);
}

static void __exit hym8563_exit(void)
{
    printk("------hym8563_exit-------\n");
	i2c_del_driver(&hym8563_driver);
}

MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_DESCRIPTION("HYM8563 RTC driver");
MODULE_LICENSE("GPL");

module_init(hym8563_init);
module_exit(hym8563_exit);

