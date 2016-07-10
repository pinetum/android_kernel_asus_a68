/*
 * kernel/drivers/media/video/tegra
 *
 * iCatch SPCA7002A ISP driver
 *
 * Copyright (C) 2012 ASUS Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/syscalls.h>
//#include <yuv_sensor.h>

#undef _CAM_SENSOR_DETECT_
#ifdef _CAM_SENSOR_DETECT_
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <mach/clk.h>
#endif
#include <linux/switch.h>
//#include <mach/board-cardhu-misc.h>
#include <iCatch7002a.h>
#include <linux/proc_fs.h>  //ASUS_BSP LiJen "[A68][ISP][NA][Others]add proc file for AP ISP update"
//#include <linux/pm_qos.h> //ASUS_BSP LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"

extern struct msm_sensor_ctrl_t imx091_s_ctrl;
extern int imx091_power_up(const struct msm_camera_sensor_info *data, bool ISPbootup);
extern int imx091_power_down(const struct msm_camera_sensor_info *data, bool ISPbootup);
extern unsigned char g_mi1040_power;
    
#define I7002A_SDEV_NAME "camera"

#define SPI_CMD_BYTE_READ 	0x03
#define SPI_CMD_RD_ID 		0x9F
#define SPI_CMD_WRT_EN		0x06
#define SPI_CMD_BYTE_PROG 	0x02
#define SPI_CMD_RD_STS		0x05
#define SPI_CMD_BYTE_PROG_AAI	0xAD
#define SPI_CMD_WRT_STS_EN	0x50
#define SPI_CMD_WRT_STS 	0x01
#define SPI_CMD_WRT_DIS 	0x04
#define SPI_CMD_ERASE_ALL	0xC7
#define	SPI_CMD_SECTOR_ERASE		0x20
#define	SPI_CMD_32KB_BLOCK_ERASE	0x52
#define	SPI_CMD_64KB_BLOCK_ERASE	0xD8

#define SENSOR_ID_MI1040	0x2481
#define SENSOR_ID_OV2720	0x2720
#define SENSOR_ID_IMX175	0x175

#define ISP_FLASH_I2C_WRITE_BYTES 64

char debug_buf[1024];

static unsigned int version_num_in_isp = 0xffffff;
static unsigned int b_fw_is_valid = 1;
struct pm_qos_request pm_qos_req_list; //ASUS_BSP LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"

static bool g_spi_is_support_1M = true;
static int ultrapixel_mode = 0;
static char  BIN_FILE_WITH_PATH[] = "/system/etc/firmware/camera/A68-BOOT.BIN";
static char* UPDATE_FILE_WITH_PATH;
static int iCatch_is_updating = 0;
static bool g_enable_roi_debug = false; //ASUS_BSP LiJen "[A68][13M][NA][Others]add 13M camera TAF support

#define NEED_UPDATE		       0x0
#define UPDATE_UNNECESSARY	0x1
#define UPDATE_TIMEOUT			0x2
#define UPDATE_WRONG_FLASHID	0x3

static int dbg_i7002a_page_index = 2047;
static bool g_afmode=0; //0: Auto AF, 1:Full search AF
static int g_flash_mode = 0;	//ASUS_BSP Stimber "Flash mode for EXIF"

/* iCatch Camera Firmware Header
 * It locates on the end of the bin file.
 * Total: 32 bytes.
 * byte[0] ~ byte[7]: 0xFF's
 * byte[8] ~ byte[11]: Compensation for Overall Checksum
 * byte[12] ~ byte[15]: Overall Checksum

 * byte[16] ~ byte[20]: 0xFF's
 * byte[21]: Front Format
 * byte[22]: Rear Lane#
 * byte[23]: Front Lane#
 * byte[24] ~ byte[25]: Rear Sensor ID
 * byte[26] ~ byte[27]: Front sensor ID
 * byte[28] ~ byte[31]: FW Version
 */
#define BIN_FILE_HEADER_SIZE 32

#define ICATCH7002A_DELAY_TEST
#ifdef ICATCH7002A_DELAY_TEST
static u32 iCatch7002a_init_delay= 5;
static u32 iCatch7002a_preview_delay=100;
static u32 touch_focus_enable=0;
#endif
#define CAM_TWO_MODE
#ifdef CAM_TWO_MODE
static unsigned int g_div = 100;
//static int g_initialized_1280_960=0;
//static int g_initialized_1080p=0;
#endif

static bool sensor_opened = false;
extern bool iCatch_first_open;
/* Used for calculating the iCatch fw update progress */
static int g_page_count = -1;
static int g_total_page_count = -1;

unsigned char g_camera_status = 1;	//ASUS_BSP Stimber "Add ATD proc interface"

//ASUS_BSP +++ Stimber "Implement the interface for calibration"
u32 is_calibration = 0;
static u32 calibrating = 0;
bool g_calibrating = false;
//ASUS_BSP --- Stimber "Implement the interface for calibration"
static u32 single_image = 0;	//ASUS_BSP Stimber "Interface for single image"
static bool g_TAEenable = 0;	//0: Disable, 1:Enable

static int g_is_hdr_on = 0;	//0: Disable, 1:Enable //ASUS_BSP stimber "Implement HDR feature"
static int g_is_nr_on = 0;	//0: Disable, 1:Enable //ASUS_BSP stimber "Implement NR feature"
static int g_is_eis_on = 0;    //0: Disable, 1:Enable //ASUS_BSP - Bryant "Implement Electric Image Stabilization(EIS) feature"
static bool  is_calibration_table_set = true;
static bool g_isAFCancel = false;
bool g_isAFDone = true;

//ASUS_BSP +++ Stimber "Modify for preview/video frame rate"
#define DEFAULT_SETTING 0
static int g_LastFixFPS = 0;
static int g_LastMaxExp = 0;
static int g_LastMiniISO = 0;
static int g_LastVideoMode = 0;
//ASUS_BSP --- Stimber "Modify for preview/video frame rate"

static bool g_is_max_exp_on = false;

struct completion g_iCatch_comp;
static bool caf_mode = false;
static int g_pre_res = MSM_SENSOR_INVALID_RES;
int g_cur_res = MSM_SENSOR_INVALID_RES;

enum iCatch_fw_update_status{
	ICATCH_FW_UPDATE_SUCCESS,
	ICATCH_FW_UPDATE_FAILED,        
	ICATCH_FW_IS_BURNING,	
       ICATCH_FW_NO_CMD,
};
static enum iCatch_fw_update_status fw_update_status = ICATCH_FW_NO_CMD;

enum iCatch_flash_type{
	ICATCH_FLASH_TYPE_ST,
	ICATCH_FLASH_TYPE_SST,
};
static enum iCatch_flash_type flash_type = ICATCH_FLASH_TYPE_ST;

struct sensor_reg {
	u16 addr;
	u16 val;
};

int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;

	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, 1);
	*val=*val&0xff;

	return 0;
}

int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;
	if (!client->adapter)
		return -ENODEV;
	
      //pr_info("%s reg(0x%x)=0x%x\n",__func__,addr,val); Jason_yeh debug sensor_write_reg
       if(((addr >> 8 ) != 0x40) && ((addr >> 8 ) != 0x41) && ((addr >> 8 ) != 0x10)){ //LiJen: filter SPI command
            pr_info("%s reg(0x%x)=0x%x",__func__,addr,val);
       }
       
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
//		pr_err("yuv_sensor : i2c transfer failed, count %x, err= 0x%x\n",
//		       __FUNCTION__, __LINE__, msg.addr, err);
		msleep(1); //LiJen: increate i2c retry delay to avoid ISP i2c fail issue
	} while (retry <= SENSOR_MAX_RETRIES);

	if(err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = 0xAAAA;
	}

	return err;
}

static int sensor_sequential_write_reg(struct i2c_client *client, unsigned char *data, u16 datasize)
{
	int err;
	struct i2c_msg msg;
	int retry = 0;

              return 0;
	if (datasize==0)
		return 0;
	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = datasize;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		//pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		      // addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n",
		       msg.addr);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int build_sequential_buffer(unsigned char *pBuf, u16 width, u16 value) {
	u32 count = 0;

	switch (width)
	{
	  case 0:
	  // possibly no address, some focusers use this
	  break;

	  // cascading switch
	  case 32:
	    pBuf[count++] = (u8)((value>>24) & 0xFF);
	  case 24:
	    pBuf[count++] = (u8)((value>>16) & 0xFF);
	  case 16:
	    pBuf[count++] = (u8)((value>>8) & 0xFF);
	  case 8:
	    pBuf[count++] = (u8)(value & 0xFF);
	    break;

	  default:
	    printk("Unsupported Bit Width %d\n", width);
	    break;
	}
	return count;

}

static int sensor_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	int err;
	const struct sensor_reg *next;
	u16 val;
	unsigned char data[10];
	u16 datasize = 0;

	//for (next = table; next->addr != SENSOR_TABLE_END; next++) {
	next = table;
	while (next->addr != SENSOR_TABLE_END) {
		if (next->addr == SENSOR_WAIT_MS) {
			msleep(next->val);
			next +=1;
			continue;
		}
		if (next->addr == SEQ_WRITE_START) {
			next += 1;
			while (next->addr !=SEQ_WRITE_END) {
				if (datasize==0) {//
					datasize += build_sequential_buffer(&data[datasize], 16, next->addr);
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				}
				else
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				if (datasize==10) {
					sensor_sequential_write_reg(client, data, datasize);
					datasize = 0;
				}
				next += 1;
			}
			sensor_sequential_write_reg(client, data, datasize); //flush out the remaining buffer.
			datasize = 0;
		}
		else {
			val = next->val;

			err = sensor_write_reg(client, next->addr, val);
			if (err) {
				printk("%s(%d): isensor_write_reg ret= 0x%x\n", __FUNCTION__, __LINE__, err);
				return err;
			}
		}
		next += 1;
	}
	return 0;
}

int I2C_SPIInit(void)
{
	int ret = 0;
	//  I2CDataWrite(0x0026,0xc0);
	//  I2CDataWrite(0x4051,0x01); /* spien */
	//  I2CDataWrite(0x40e1,0x00); /* spi mode */
	//  I2CDataWrite(0x40e0,0x11); /* spi freq */
	struct sensor_reg SPI_init_seq[] = {
		{0x0026, 0xc0},
		{0x4051, 0x01},
		{0x40e1, 0x00},
		{0x40e0, 0x11},
		{SENSOR_TABLE_END, 0x0000}
	};

	ret = sensor_write_table(imx091_s_ctrl.sensor_i2c_client->client, SPI_init_seq);
	if(ret) {
		printk("%s: init fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIInit
 *  return SUCCESS: normal
           FAIL: if wait spi flash time out
 *------------------------------------------------------------------------*/
u32 I2C_SPIFlashPortWait(void)
{
    //u32 cnt = WAIT_COUNT;
#if 0
    while(I2CDataRead(0x40e6) != 0x00){
        cnt--;
        if(cnt == 0x00)
        {
            printf("serial flash port wait time out!!\n");
            return FAIL;
        }
    }
#endif
    return 0;
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIFlashPortWrite
 *  return SUCCESS: normal
           FAIL:    if wait spi flash time out
 *------------------------------------------------------------------------*/
u32 I2C_SPIFlashPortWrite(u32 wData)
{
    //hsI2CDataWrite(0x40e3,(u8)wData);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)wData);
    return I2C_SPIFlashPortWait();
}

u32 I2C_SPIFlashPortRead(void)
{
	u16 ret;

	// ret = hsI2CDataRead(0x40e4);
      sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e4, &ret);
	/* polling SPI state machine ready */
#if 0
    if (I2C_SPIFlashPortWait() != SUCCESS) {
        return 0;
    }
#endif
	//ret = hsI2CDataRead(0x40e5);
      sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e5, &ret);

    return (u32)ret;
}

u32 I2C_SPIFlashRead(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 err = 0;
	u32 i, size=0;
	u32 pageSize = 0x100;

	addr = addr * pageSize;
	size = pages*pageSize;

	// I2CDataWrite(0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7, 0x00);
	// I2C_SPIFlashPortWrite(SPI_CMD_BYTE_READ);               /* Write one byte command*/
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3, SPI_CMD_BYTE_READ);
	// I2C_SPIFlashPortWrite((u8)(addr >> 16));               /* Send 3 bytes address*/
	// I2C_SPIFlashPortWrite((u8)(addr >> 8));
	// I2C_SPIFlashPortWrite((u8)(addr));
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3, (u8)(addr >> 16));
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3, (u8)(addr >> 8));
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3, (u8)(addr));

	for (i = 0; i < size ; i++) {
		*pbuf = I2C_SPIFlashPortRead();
		if((i%256)==0)
			printk("%s: page count: 0x%x\n", __FUNCTION__, (i/256));
		pbuf ++;
	}

	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7, 0x01);

	return err;
}

u32 I2C_SPIFlashReadId(void)
{
	u8 id[3];
	u32 ID;

	id[0] = 0;
	id[1] = 0;
	id[2] = 0;

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);

	//err = I2C_SPIFlashPortWrite(SPI_CMD_RD_ID); /*read ID command*/
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_RD_ID);

#if 0
	if (err != SUCCESS) {
		printf("Get serial flash ID failed\n");
		return 0;
	}
#endif

	id[0] = I2C_SPIFlashPortRead();    /* Manufacturer's  ID */
	id[1] = I2C_SPIFlashPortRead();    /* Device ID          */
	id[2] = I2C_SPIFlashPortRead();    /* Manufacturer's  ID */

	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	printk("ID %2x %2x %2x\n", id[0], id[1], id[2]);

	ID = ((u32)id[0] << 16) | ((u32)id[1] << 8) | \
    ((u32)id[2] << 0);

	return ID;
}

static const u32 stSpiIdInfo[30] =
{
	/*EON*/
	0x001C3117,
	0x001C2016,
	0x001C3116,
	0x001C3115,
	0x001C3114,
	0x001C3113,
	/*Spansion*/
	0x00012018,
	0x00010216,
	0x00010215,
	0x00010214,
	/*ST*/
	0x00202018,
	0x00202017,
	0x00202016,
	0x00202015,
	0x00202014,
	/*MXIC*/
	0x00C22018,
	0x00C22017,
	0x00C22016,
	0x00C25e16,
	0x00C22015,
	0x00C22014,
	0x00C22013,
	/*Winbond*/
	0x00EF3017,
	0x00EF3016,
	0x00EF3015,
	0x00EF3014,
	0x00EF3013,
	0x00EF5013,
	0x00EF5014,/* For A68 1MB serial flash ID */
	/*Fail*/
	0x00000000,
};

static const u32 sstSpiIdInfo[6] =
{
	/*ESMT*/
	0x008C4016,
	/*SST*/
	0x00BF254A,
	0x00BF2541,
	0x00BF258E,
	0x00BF258D,
	/*Fail*/
	0x00000000,
};

u32
BB_SerialFlashTypeCheck(
	u32 id
)
{
	u32 i=0;
	u32 fullID = 1;
	u32 shift = 0, tblId, type = 0;

	/* check whether SST type serial flash */
	while( 1 ){
		tblId = sstSpiIdInfo[i] >> shift;
		if( id == tblId ) {
			printk("SST type serial flash\n");
			type = 2;
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( sstSpiIdInfo[i] == 0x00000000 ) {
			#if 0
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			#endif
			type = 3;
			break;
		}
		i ++;
	}
	if( type == 2 )
		return type;

	i = 0;
	/* check whether ST type serial flash */
	while( 1 ){
		tblId = stSpiIdInfo[i] >> shift;
		if( id == tblId ) {
			printk("ST Type serial flash\n");
			type = 1;
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( stSpiIdInfo[i] == 0x00000000 ) {
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			type = 3;
			break;
		}
		i ++;
	}

	return type;
}

int I2C_SPIFlashWrEnable(void)
{
	int ret = 0;
	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_EN);
	//hsI2CDataWrite(0x40e7,0x01);
	struct sensor_reg I2C_SPIFlashWrEnable_seq[] = {
		{0x40e7, 0x00},
		{0x40e3, SPI_CMD_WRT_EN},
		{0x40e7, 0x01},
		{SENSOR_TABLE_END, 0x0000}
	};

	ret = sensor_write_table(imx091_s_ctrl.sensor_i2c_client->client, I2C_SPIFlashWrEnable_seq);

	if(ret) {
		printk("%s: fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

u32 I2C_SPIStsRegRead(void)
{
	u32 ret;

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_RD_STS);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_RD_STS);
	ret = I2C_SPIFlashPortRead();

	// hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	return ret;
}

void I2C_SPITimeOutWait(u32 poll, u32 *ptimeOut)
{
    /* MAX_TIME for SECTOR/BLOCK ERASE is 25ms */
    u32 sts;
    u32 time = 0;
    while (1) {
        sts = I2C_SPIStsRegRead();
        if (!(sts & poll))	/* sfStatusRead() > 4.8us */ {
            break;
        }
        time ++;
        if( *ptimeOut < time ) {
            printk("iCatch: TimeOut %d, sts=0x%x, poll=0x%x\n",time,sts,poll);
            break;
        }
    }
}

int I2C_SPIStChipErase(
	void
)
{
	u32 timeout;
	int ret = 0;
	printk("iCatch: ST Chip Erasing...\n");

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS);
	//I2C_SPIFlashPortWrite(0x02);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,0x02);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	ret = I2C_SPIFlashWrEnable();
	if (ret) {
		printk("iCatch: ST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_ERASE_ALL);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_ERASE_ALL);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	timeout = 0xffffffff;
	I2C_SPITimeOutWait(0x01, &timeout);
#if 0
	ros_thread_sleep(1);
#endif
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	printk("iCatch: ST Chip Erased\n");
	return 0;
}

int I2C_SPISstChipErase(void)
{
	u32 timeout;
	int ret = 0;
	printk("iCatch: SST Chip Erasing...\n");

	ret = I2C_SPIFlashWrEnable();
	if (ret) {
		printk("iCatch: SST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN); /*Write Status register command*/
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS_EN);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	//I2C_SPIFlashPortWrite(0x02);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,0x02);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable();

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_ERASE_ALL);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_ERASE_ALL);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	timeout = 0xffffffff;
	I2C_SPITimeOutWait(0x01, &timeout);
	//msleep(500);
	printk("iCatch: SST Chip Erased\n");
	return 0;
}

void writeUpdateProgresstoFile(int page_left, int total_page_num)
{
	struct file *fp_progress = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;
	char str_progress[4];
	int percentage = 0;

	percentage = 100 * (total_page_num - page_left + 1)/total_page_num;

	if(page_left % 32 == 1){
		printk("%s: page:0x%x; percentage= %d;\n", __FUNCTION__, page_left, percentage);
		fp_progress = filp_open("/data/.tmp/isp_fw_update_progress", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
		if ( IS_ERR_OR_NULL(fp_progress) ){
			filp_close(fp_progress, NULL);
			printk("%s: open %s fail\n", __FUNCTION__, "/data/.tmp/isp_fw_update_progress");
		}
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		offset = 0;
		if (fp_progress->f_op != NULL && fp_progress->f_op->write != NULL){
			sprintf(str_progress, "%d\n", percentage);
			fp_progress->f_op->write(fp_progress,
				str_progress,
				strlen(str_progress),
				&offset);
		}else
			pr_err("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp_progress, NULL);
	}
}

void writeUpdateResultFile(void)
{
	struct file *fp_result = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;
	char str_result[4];

	printk("%s: fw_update_status= %d;\n", __FUNCTION__, fw_update_status);
    
	fp_result = filp_open("/data/.tmp/isp_fw_update_result", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
	if ( IS_ERR_OR_NULL(fp_result) ){
		filp_close(fp_result, NULL);
		printk("%s: open %s fail\n", __FUNCTION__, "/data/.tmp/isp_fw_update_result");
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offset = 0;
	if (fp_result->f_op != NULL && fp_result->f_op->write != NULL){
		sprintf(str_result, "%d\n", fw_update_status);
		fp_result->f_op->write(fp_result,
			str_result,
			strlen(str_result),
			&offset);
	}else
		pr_err("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp_result, NULL);

}

#if 0
u32 I2C_SPIFlashWrite(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 i, err = 0;
	u32 pageSize = 0x100;

	addr = addr * pageSize;

	printk("iCatch: ST type writing...\n");
	g_total_page_count = (int)pages;

	while( pages ) {
		g_page_count = (int)pages;
		writeUpdateProgresstoFile(g_page_count, g_total_page_count);

		I2C_SPIFlashWrEnable();
		//hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG); /* Write one byte command*/
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_BYTE_PROG);
		// I2C_SPIFlashPortWrite((u8)(addr >> 16)); /* Send 3 bytes address*/
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr >> 16));
		// I2C_SPIFlashPortWrite((u8)(addr >> 8));
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr >> 8));
		// I2C_SPIFlashPortWrite((u8)(addr));
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr));

		for (i = 0; i < pageSize ; i++) {
			// How about "Early return" here?
			// I2C_SPIFlashPortWrite(*pbuf);
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
			pbuf++;
		}
		// hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
		addr += pageSize;
		pages --;
		// tmrUsWait(2000);
		udelay(2000);
	}
	printk("iCatch: ST type writing Done\n");
	return err;
}
#endif

void I2C_SPISstStatusWrite(u8 dat)
{
	u32 timeout, poll;

	I2C_SPIFlashWrEnable();

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS_EN);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	// hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	//I2C_SPIFlashPortWrite(dat);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS);
	printk("%s: dat=%d\n", __FUNCTION__, dat);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,dat);
	printk("%s: dat=%d; Done.\n", __FUNCTION__, dat);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	poll = 0x01;
#if 0
	if( spiDev.bus != SPI_1BIT_MODE ) {/* 1 bit mode */
		poll = 0x80;
	} else {
		poll = 0x01;
	}
#endif
    timeout = 100000;
    I2C_SPITimeOutWait(poll, &timeout);
    //msleep(500);
    return;
}

u32 I2C_SPISstFlashWrite(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 i, err = 0;
	u32 pageSize = 0x100;
	u32 timeout = 100000;

	addr = addr * pageSize;

	printk("iCatch: SST type writing...\n");
	I2C_SPISstStatusWrite(0x40);

	g_total_page_count = (int)pages;

	while( pages ) {
		g_page_count = (int)pages;
		writeUpdateProgresstoFile(g_page_count, g_total_page_count);

		I2C_SPIFlashWrEnable();
		//hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI); /* Write one byte command*/
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_BYTE_PROG_AAI);
		//I2C_SPIFlashPortWrite((u8)(addr >> 16)); /* Send 3 bytes address*/
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr >> 16));
		//I2C_SPIFlashPortWrite((u8)(addr >> 8));
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr >> 8));
		//I2C_SPIFlashPortWrite((u8)(addr));
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr));
		//I2C_SPIFlashPortWrite(*pbuf);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
		pbuf++;
		//I2C_SPIFlashPortWrite(*pbuf);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
		pbuf++;
		//hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
		timeout = 100000;
		I2C_SPITimeOutWait(0x01,&timeout);

		for (i = 2; i < pageSize ; i = i+2) {
			//hsI2CDataWrite(0x40e7,0x00);
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
			//I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI);
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_BYTE_PROG_AAI);
			// I2C_SPIFlashPortWrite(*pbuf);
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
			pbuf++;
			// I2C_SPIFlashPortWrite(*pbuf);
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
			pbuf++;
			// hsI2CDataWrite(0x40e7,0x01);
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
			timeout = 100000;
			I2C_SPITimeOutWait(0x01,&timeout);
		}

		// hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_DIS);
		//hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

		addr += pageSize;
		pages --;

		//hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_DIS);
		//hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	}
	printk("iCatch: SST type writing Done.\n");
	return err;
}

/* get_one_page_from_i7002a():
 *   Dump the ISP page whose index is "which_page" to "pagebuf".
 *   mclk, power & rst are requisite for getting correct page data.
 */
void get_one_page_from_i7002a(int which_page, u8* pagebuf)
{
	int i = 0;
	int ret = 0;
	//I2CDataWrite(0x70c4,0x00);
	//I2CDataWrite(0x70c5,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x70c4,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x70c5,0x00);

	ret = I2C_SPIInit();
	if (ret) {
		g_camera_status = 0;	//ASUS_BSP Stimber "Add ATD proc interface"
		printk("%s: get nothing. ret= %d", __FUNCTION__, ret);
		return;
	}

	I2C_SPIFlashReadId();

	I2C_SPIFlashRead(which_page, 1, pagebuf);

#if 1 // dump to kmsg ?
	printk("page#%d:\n", which_page);
	for(i=0; i < 0x100; i++) {
		if(i%16 == 0)
			printk("[%04x]", i);
		printk("%02X ",  pagebuf[i]);
		if(i%16 == 15)
			printk("\n");
	}
#endif
}

unsigned int get_fw_version_in_bin(char* binfile_path)
{
	u8 bin_file_header[BIN_FILE_HEADER_SIZE];
	unsigned int version_num_in_bin = 0xFFFFFF;
       struct stat statbuf;

	int fp;
	mm_segment_t old_fs;
	unsigned long bootbin_size = 0;

	/* Calculate BOOT.BIN file size. */
    	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = sys_open(binfile_path, O_RDONLY, 0);
	if (fp > 0){
		pr_info("filp_open success fp:%d\n", fp);
              sys_newfstat(fp, &statbuf);
              bootbin_size = statbuf.st_size;
              sys_lseek(fp, bootbin_size - BIN_FILE_HEADER_SIZE, 0);
              sys_read(fp, bin_file_header, BIN_FILE_HEADER_SIZE);
		sys_close(fp);
	} else{
		pr_err("iCatch \"%s\" open error\n", binfile_path);
		return version_num_in_bin;
	}
    	set_fs(old_fs);
	version_num_in_bin = (bin_file_header[30] << 16) | (bin_file_header[29] << 8) | bin_file_header[28];
       return version_num_in_bin;
}

unsigned int get_fw_version_in_isp(void)
{
	u8 tmp_page[0x100];
	unsigned int vn = 0xABCDEF;
	int i = 0;
	int retry = 3;
	bool b_ok;

	for (i = 0; i < retry; i++) {
		int j =0;
		b_ok = true;

              if(true == g_spi_is_support_1M){
		    /* The fw veriosn is in the page with the index, 4095.*/
                  get_one_page_from_i7002a(4095, tmp_page);
              }else{
		    /* The fw veriosn is in the page with the index, 2047.*/
		    get_one_page_from_i7002a(2047, tmp_page);
              }

		/* The header format looks like:
		 * FF FF FF FF FF FF FF FF XX XX XX XX XX XX XX
		 * FF FF FF FF FF XX XX XX XX XX XX XX XX XX XX
		 */
		for (j = 0; j < 8; j++) {
			if (tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j] != 0xFF) {
				printk("%s: tmp_page[0x%X]= %02X\n", __FUNCTION__,
					0x100 - BIN_FILE_HEADER_SIZE + j,
					tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j]);
				b_ok = false;
				break;
			}
		}
		if (b_ok == true)
			break;
		else {
			printk("%s: wrong page data? Try again (%d).\n", __FUNCTION__, i);
			msleep(10);
		}
	}

	if (b_ok == true)
		vn = (tmp_page[0xFF - 1] <<16) | (tmp_page[0xFF - 2] << 8) | tmp_page[0xFF -3];

       version_num_in_isp = vn;
	printk("%s: vn=0x%X\n", __FUNCTION__, vn);
	return vn;
}

//ASUS_BSP +++ LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"
#define MSM_V4L2_SWFI_LATENCY 3
static void iCatch_pm_qos_update_latency(bool vote)
{
	if (vote){
		pm_qos_add_request(&pm_qos_req_list,
				PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);
		pm_qos_update_request(&pm_qos_req_list,
				MSM_V4L2_SWFI_LATENCY);
       }
	else{
		pm_qos_update_request(&pm_qos_req_list,
				PM_QOS_DEFAULT_VALUE);
		pm_qos_remove_request(&pm_qos_req_list);
       }
}
//ASUS_BSP --- LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"

static int sensor_write_reg_bytes(struct i2c_client *client, u16 addr, unsigned char* val, u32 bytes)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[bytes+2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

       memcpy((data+2), val, bytes);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = bytes+2;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
//		pr_err("yuv_sensor : i2c transfer failed, retrying %x %llu\n", addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n", msg.addr);
	} while (retry <= 3);

	return err;
}

int sensor_read_reg_bytes(struct i2c_client *client, u16 addr, unsigned char* val, u32 bytes)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[bytes+2];
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;

	msg[1].len = bytes;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);
       
	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, bytes);

	return 0;
}

unsigned int get_fw_version_in_isp_fromISP(void)
{
    u32 read_bytes=3;
    unsigned char read_data[read_bytes];
    unsigned int vn = 0xABCDEF;
    int retry = 0;

    //msleep(30); // wait ISP load code
    do{
        msleep(10);
        sensor_read_reg_bytes(imx091_s_ctrl.sensor_i2c_client->client, 0x72ca, read_data, read_bytes);
        vn = (read_data[2] << 16)|(read_data[1] << 8)|read_data[0];        
        retry ++;
        pr_info("%s  wrong version data? Try again (0x%x)(0x%x)(0x%x)\n",__func__,read_data[2],read_data[1],read_data[0]);
    }while(vn==0 && retry<20);
    
    version_num_in_isp = vn;
    printk("%s: vn=0x%X\n", __FUNCTION__, vn);
    return vn;
}

u32 I2C_7002DmemWr(
	u32 bankNum,
	u32 byteNum,
	u8* pbuf
)
{
	u32 i, j, bank;
       const u32 bytes = ISP_FLASH_I2C_WRITE_BYTES;
       unsigned char tmp[bytes];
       int rc=0;

	bank = 0x40+bankNum;
	//I2CDataWrite(0x10A6,bank);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10A6,bank);

	for(i=0;i<byteNum;i+=bytes) {
		//seqI2CDataWrite((0x1800+i),(pbuf+i)); /* sequentially write DMEM */
              memset(&tmp, 0, sizeof(tmp));
        
               for(j=0;j<bytes;j++){
                    tmp[j]=(*(pbuf + i + j));
               }        

              rc=sensor_write_reg_bytes(imx091_s_ctrl.sensor_i2c_client->client, (0x1800+i), tmp, bytes);
              if(rc < 0){
                    pr_err("%s sensor_write_reg_bytes fail rc=%d\n",__func__,rc);
                    return rc;
              }
	}

	bank = 0x40 + ((bankNum+1)%2);
	//hsI2CDataWrite(0x10A6,bank);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10A6,bank);

       return 0;
}

u32 I2C_SPIFlashWrite_DMA(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
    u32 i, err = 0;
    u32 pageSize = 0x100, size;
    u32 rsvSec1, rsvSec2;
    u32 dmemBank = 0;
    u32 chk1=0;
    u16 chk2, temp = 0;
    int rc=0;

    rsvSec1 = pages*pageSize - 0x7000;
    rsvSec2 = pages*pageSize - 0x1000;
    addr = addr * pageSize;
    
    /* Set DMA bytecnt as 256-1 */
    //I2CDataWrite(0x4170,0xff);
    //I2CDataWrite(0x4171,0x00);
    //I2CDataWrite(0x4172,0x00);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4170,0xff);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4171,0x00);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4172,0x00);    

    /* Set DMA bank & DMA start address */
    //I2CDataWrite(0x1084,0x01);
    //I2CDataWrite(0x1080,0x00);
    //I2CDataWrite(0x1081,0x00);
    //I2CDataWrite(0x1082,0x00);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1084,0x01);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1080,0x00);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1081,0x00);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1082,0x00);    

    /* enable DMA checksum and reset checksum */
    //I2CDataWrite(0x4280,0x01);
    //I2CDataWrite(0x4284,0x00);
    //I2CDataWrite(0x4285,0x00);
    //I2CDataWrite(0x4164,0x00);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4280,0x01);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4284,0x00);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4285,0x00);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4164,0x00);    

    size = pages * pageSize;
    for(i=0;i<size;i++)
    {
	if((i>=rsvSec2) || (i <rsvSec1))
	{
		chk1 += *(pbuf+i);
	}
	if(chk1>=0x10000)
	{
		chk1 -= 0x10000;
	}
   }

    g_total_page_count = (int)pages;
    
    while( pages ) {
	g_page_count = (int)pages;
	writeUpdateProgresstoFile(g_page_count, g_total_page_count);        
    
   	 if((pages%0x40)==0)
   	 {
		//printk("page:0x%x\n",pages);
    	}
   	 if((addr>=rsvSec1) && (addr <rsvSec2))
   	 {
		addr += 0x1000;
               		pbuf += 0x1000;
		pages -= 0x10;
		continue;
    	}
    	if((pages==1))
   	 {
		for (i = 0; i < pageSize ; i++) {
            		printk("%2x ",*(pbuf+i));
		if((i%0x10)==0x0f) printk("\n");
    		}
    	}

    	dmemBank = pages % 2;
//    	I2CDataWrite(0x1081,dmemBank*0x20);
//    	I2CDataWrite(0x1084,(1<<dmemBank));
    	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1081,dmemBank*0x20);
    	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1084,(1<<dmemBank));
    	rc=I2C_7002DmemWr(dmemBank,pageSize,pbuf);		
       if(rc<0){
            pr_err("%s fail\n",__func__);
            return rc;
       }		
     	I2C_SPIFlashWrEnable();
     	//I2CDataWrite(0x40e7,0x00);
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
     	I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG);               /* Write one byte command*/
     	I2C_SPIFlashPortWrite((u8)(addr >> 16));               /* Send 3 bytes address*/
     	I2C_SPIFlashPortWrite((u8)(addr >> 8));
     	I2C_SPIFlashPortWrite((u8)(addr));

    	//I2CDataWrite(0x4160,0x01);
    	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4160,0x01);
    	udelay(100);/* wait for DMA done */
    	//I2CDataWrite(0x40e7,0x01);
    	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
    	pbuf += pageSize;
    	addr += pageSize;
    	pages --;
    }
	
    udelay(500);/* wait for DMA done */

    //temp = hsI2CDataRead(0x4285);
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4285, &temp);
    //chk2 = hsI2CDataRead(0x4284);
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4284, &chk2);  
    chk2 = chk2 | (temp<<8);
    pr_info("checksum: 0x%x 0x%x\n",chk1,chk2);

    return err;
}

u32 I2C_SPISectorErase(
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	pr_info("addr:0x%x\n",address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable();
	
		//hsI2CDataWrite(0x40e7,0x00);
    	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
		//hsI2CDataWrite(0x40e7,0x01);
    	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	}
	
	//hsI2CDataWrite(0x40e7,0x00);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable();

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_SECTOR_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(address);			/* A07~A00 */
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	
	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);
	
	return 0;
}

u32 I2C_SPI32KBBlockErase(
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	pr_info("addr:0x%x\n",address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable();
	
		//hsI2CDataWrite(0x40e7,0x00);
    	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
		//hsI2CDataWrite(0x40e7,0x01);
    	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	}

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable();

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_32KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(address);			/* A07~A00 */
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	
	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);
	
	return 0;
}

u32 I2C_SPI64KBBlockErase(
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	pr_info("addr:0x%x\n",address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable();
	
		//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
		//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	}

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable();

	//hsI2CDataWrite(0x40e7,0x00);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_64KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(address);			/* A07~A00 */
	//hsI2CDataWrite(0x40e7,0x01);
  	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	
	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);
	
	return 0;
}

void
BB_EraseSPIFlash(
	u32 type,
	u32 spiSize
)
{
	u8 typeFlag=0;
	u32 i, temp1;
	if( type == 2 )/* SST */
	{
		typeFlag = 0;
	}
	else if( type == 1 || type == 3 )/* ST */
	{
		typeFlag = 1;
	}else{
            pr_err("%s type(%d) is not support\n",__func__,type);
            return;
       }
    
	/*printf("spiSize:0x%x\n",spiSize);*/
	if(spiSize == (512*1024))
	{
		/* skip 0x7B000 ~ 0x7EFF, to keep calibration data */
		temp1 = (spiSize / 0x10000)-1;
		for(i=0;i<temp1;i++)
		{
			I2C_SPI64KBBlockErase(i*0x10000,typeFlag);
		}
		I2C_SPI32KBBlockErase(temp1*0x10000,typeFlag);
		temp1 = temp1*0x10000 + 0x8000;
		for(i=temp1;i<spiSize-0x5000;i+=0x1000)
		{
			I2C_SPISectorErase(i,typeFlag);
		}
		I2C_SPISectorErase(spiSize-0x1000,typeFlag);
	}
	else if(spiSize == (1024*1024))
	{
		/* only erase 256*3KB */
		temp1 = ((spiSize*3/4) / 0x10000);
		for(i=0;i<temp1;i++)
		{
			I2C_SPI64KBBlockErase(i*0x10000,typeFlag);
		}
		I2C_SPI32KBBlockErase((temp1+1),typeFlag);
		I2C_SPISectorErase(spiSize-0x1000,typeFlag);
	}
}

void
BB_WrSPIFlash(char* binfile_path)
{
	u32 id, type;
	u32 pages;

	u8 *pbootBuf;
	u8 bin_file_header[BIN_FILE_HEADER_SIZE];
	u8 checksum1_in_bin[2], checksum2_in_bin[2];
	u8 checksum1_in_isp[2], checksum2_in_isp[2];
	unsigned int version_num_in_bin = 0xFFFFFF;
	int firmware2_offset;
	u8 tmp_page[0x100];

	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
	int bootbin_size = 0;
	int i, ret = 0;

       iCatch_pm_qos_update_latency(true); //ASUS_BSP LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"
	fw_update_status = ICATCH_FW_IS_BURNING;

	/* Calculate BOOT.BIN file size. */
	fp = filp_open(binfile_path, O_RDONLY, 0);

	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, bootbin_size);
		pbootBuf = kmalloc(bootbin_size, GFP_KERNEL);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			int byte_count= 0;
			printk("Start to read %s\n", binfile_path);

			byte_count = fp->f_op->read(fp, pbootBuf, bootbin_size, &fp->f_pos);

			if (byte_count <= 0) {
				printk("iCatch: EOF or error. last byte_count= %d;\n", byte_count);
				kfree(pbootBuf);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				goto end;
			} else
				printk("iCatch: BIN file size= %d bytes\n", bootbin_size);

#if 0
			for(i=0; i < bootbin_size; i++) {
				printk("%c", pbootBuf[i]);
			}
			printk("\n");
#endif
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err("iCatch \"%s\" not found error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	} else{
		pr_err("iCatch \"%s\" open error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	for (i=0; i < BIN_FILE_HEADER_SIZE; i++)
	{
		bin_file_header[i] = pbootBuf[bootbin_size - BIN_FILE_HEADER_SIZE + i];
		printk("%s: bin_file_header[%d]= 0x%x\n", __FUNCTION__, i,bin_file_header[i]);
	}
	version_num_in_bin = (bin_file_header[30] << 16) | (bin_file_header[29] << 8) | bin_file_header[28];

	/* Get the checksum in bin file.
	 *   firmware2_offset
	 *     = fw1 header size
	 *     + fw1 DMEM FICDMEM size
	 *     + fw1 IMEM size
	 */
	memcpy(checksum1_in_bin, pbootBuf + 10, 2);

	firmware2_offset = 16 +
		((pbootBuf[3] << 24) | (pbootBuf[2] << 16) | (pbootBuf[1] << 8) | pbootBuf[0]) +
		((pbootBuf[7] << 24) | (pbootBuf[6] << 16) | (pbootBuf[5] << 8) | pbootBuf[4]);
	memcpy(checksum2_in_bin, pbootBuf + firmware2_offset + 10, 2);

	printk("%s: checksum in bin:%02X %02X; %02X %02X\n", __FUNCTION__,
		checksum1_in_bin[0],checksum1_in_bin[1],checksum2_in_bin[0], checksum2_in_bin[1]);

	ret = I2C_SPIInit();
	if (ret) {
		printk("%s: SPI init fail. ret= 0x%x", __FUNCTION__, ret);
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	id = I2C_SPIFlashReadId();

	if(id==0) {
		printk("read id failed\n");
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	type = BB_SerialFlashTypeCheck(id);
	if(type == 0) {
		printk("BB_SerialFlashTypeCheck(%d) failed\n", id);
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	pages = bootbin_size/0x100;

	printk("%s: pages:0x%x\n", __FUNCTION__, pages);

	BB_EraseSPIFlash(type,bootbin_size);

	/* Writing Flash here */
	if( type == 2 ) {
		flash_type = ICATCH_FLASH_TYPE_SST;
		printk("SST operation\n");
		#if 0
		ret = I2C_SPISstChipErase();
		if(ret) {
			printk("%s: SST erase fail.\n", __FUNCTION__);
			kfree(pbootBuf);
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			goto end;
		}
		#endif
		I2C_SPISstFlashWrite(0, pages, pbootBuf);
	} else if( type == 1 || type == 3 ) {
		flash_type = ICATCH_FLASH_TYPE_ST;
		printk("ST operation\n");
		#if 0
		ret = I2C_SPIStChipErase();
		if(ret) {
			printk("%s: ST erase fail.\n", __FUNCTION__);
			kfree(pbootBuf);
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			goto end;
		}
		#endif
		//I2C_SPIFlashWrite(0, pages, pbootBuf);
		ret=I2C_SPIFlashWrite_DMA(0, pages, pbootBuf);
                if(ret<0){
                    printk("I2C_SPIFlashWrite_DMA fail\n");
                    fw_update_status = ICATCH_FW_UPDATE_FAILED;
               }
	} else {
		printk("type unknown: %d; Won't update iCatch FW.\n", type);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		kfree(pbootBuf);
		goto end;
	}
	kfree(pbootBuf);

	/* Check the update reult. */
	/* Compare Check sum here */
	get_one_page_from_i7002a(0, tmp_page);
	memcpy(checksum1_in_isp, tmp_page + 10, 2);

	if (memcmp(checksum1_in_isp, checksum1_in_bin, 2) == 0) {
		/* checksum1 PASS */
		firmware2_offset = 16 +
			((tmp_page[3] << 24) | (tmp_page[2] << 16) | (tmp_page[1] << 8) | tmp_page[0]) +
			((tmp_page[7] << 24) | (tmp_page[6] << 16) | (tmp_page[5] << 8) | tmp_page[4]);

		get_one_page_from_i7002a(firmware2_offset >> 8, tmp_page);
		memcpy(checksum2_in_isp, tmp_page + 10, 2);

		if (memcmp(checksum2_in_isp, checksum2_in_bin, 2) == 0) {
			/* checksum2 PASS */
			version_num_in_isp = get_fw_version_in_isp();
			if (version_num_in_isp == version_num_in_bin) {
				/* version number PASS */
				fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
				printk("%s: ICATCH FW UPDATE SUCCESS.\n", __FUNCTION__);
			} else {
				/* version number FAIL */
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				printk("%s: check version FAIL: ISP(0x%06X) != BIN(0x%06X)\n", __FUNCTION__, version_num_in_isp, version_num_in_bin);
				version_num_in_isp = 0xABCDEF;
			}
		} else {
			/* checksum2 FAIL */
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			printk("%s: checksum2 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
				__FUNCTION__, checksum2_in_isp[0], checksum2_in_isp[1],
				checksum2_in_bin[0], checksum2_in_bin[1]);
			version_num_in_isp = 0xABCDEF;
		}
	} else {
		/* checksum1 FAIL */
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		printk("%s: checksum1 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
			__FUNCTION__, checksum1_in_isp[0], checksum1_in_isp[1],
			checksum1_in_bin[0], checksum1_in_bin[1]);
		version_num_in_isp = 0xABCDEF;
	}

end:
       writeUpdateResultFile();
       iCatch_pm_qos_update_latency(false); //ASUS_BSP LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"
}

#ifdef WITH_INT		
irqreturn_t iCatch_irq_handler(int irq, void *data)
{  
        pr_info("isp_int\n");
        complete(&g_iCatch_comp);
        //disable_irq(imx091_s_ctrl.sensordata->sensor_platform_info->isp_int);
	return IRQ_HANDLED;
}
#endif 

void iCatch_debug(void)
{
    u16 readval, i;
    u32 read_bytes=128;
    unsigned char read_data[read_bytes];
       
    pr_info("%s +++\n",__func__);

    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x202F, 0x01);
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x202F, 0x00);
    msleep(100);
    
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x007a, &readval);
    pr_info("%s reg(0x007a)=0x%x\n",__func__,readval);
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x007b, &readval);
    pr_info("%s reg(0x007b)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x2054, &readval);
    pr_info("%s reg(0x2054)=0x%x\n",__func__,readval);
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x2055, &readval);
    pr_info("%s reg(0x2055)=0x%x\n",__func__,readval);
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x2056, &readval);
    pr_info("%s reg(0x2056)=0x%x\n",__func__,readval);    
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x2057, &readval);
    pr_info("%s reg(0x2057)=0x%x\n",__func__,readval);    
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x2058, &readval);
    pr_info("%s reg(0x2058)=0x%x\n",__func__,readval);    
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x2059, &readval);
    pr_info("%s reg(0x2059)=0x%x\n",__func__,readval);    
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x205a, &readval);
    pr_info("%s reg(0x205a)=0x%x\n",__func__,readval);    
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x205b, &readval);
    pr_info("%s reg(0x205b)=0x%x\n",__func__,readval);    
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4284, &readval);
    pr_info("%s reg(0x4284)=0x%x\n",__func__,readval);    
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x4285, &readval);
    pr_info("%s reg(0x4285)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9080, &readval);
    pr_info("%s reg(0x9080)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x905c, &readval);
    pr_info("%s reg(0x905c)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7005, &readval);
    pr_info("%s reg(0x7005)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7006, &readval);
    pr_info("%s reg(0x7006)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7070, &readval);
    pr_info("%s reg(0x7070)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7072, &readval);
    pr_info("%s reg(0x7072)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7073, &readval);
    pr_info("%s reg(0x7073)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7074, &readval);
    pr_info("%s reg(0x7074)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7075, &readval);
    pr_info("%s reg(0x7075)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x90cc, &readval);
    pr_info("%s reg(0x90cc)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x90cd, &readval);
    pr_info("%s reg(0x90cd)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x90ce, &readval);
    pr_info("%s reg(0x90ce)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x90cf, &readval);
    pr_info("%s reg(0x90cf)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9400, &readval);
    pr_info("%s reg(0x9400)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9408, &readval);
    pr_info("%s reg(0x9408)=0x%x\n",__func__,readval);   
    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, &readval);
    pr_info("%s reg(0x72f8)=0x%x\n",__func__,readval); 

    //Get ISP Status
    sensor_read_reg_bytes(imx091_s_ctrl.sensor_i2c_client->client, 0x7200, read_data, read_bytes);
    for(i=0;i<read_bytes;i++){
        pr_info("%s reg(0x72%2x)=0x%x\n",__func__,i, read_data[i]);     
    }
    
    pr_info("%s ---\n",__func__);
}

void wait_for_next_frame(void){
#ifdef WITH_INT	
        u32 timeout_count = 1;

        //enable_irq(imx091_s_ctrl.sensordata->sensor_platform_info->isp_int);
        //INIT_COMPLETION(g_iCatch_comp);
        timeout_count = wait_for_completion_timeout(&g_iCatch_comp, 4*HZ);
        if (!timeout_count) {
            pr_err("%s: interrupt timedout\n", __func__);
            iCatch_debug();
        } else {
            pr_debug("%s interrupt done\n",__func__);
            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
        }             
#else
        u16 testval, i;

        for (i=0;i<200;i++)
        {
            sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, &testval);
            printk("testval=0x%X, i=%d\n",testval,i);
            if (testval & 0x04) {
            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
            sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, &testval);
            printk("Clear testval=0x%X, i=%d\n",testval,i);
            break;
            }
            //printk("testval=0x%X, i=%d\n",testval,i);
            msleep(iCatch7002a_init_delay);
        }
        
        if(i==200){
            iCatch_debug();
        }
#endif    
}

void wait_for_AE_ready(void){
    u16 testval, i;
             
    iCatch_first_open = false;
                
    for (i=0;i<200;i++)
    {
        sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72c3, &testval);
        printk("testval=0x%X, i=%d\n",testval,i);
        if (testval & 0x01) {
            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
            sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, &testval);
            printk("Clear testval=0x%X, i=%d\n",testval,i);
            break;
        }
        //printk("testval=0x%X, i=%d\n",testval,i);
        msleep(iCatch7002a_init_delay);
    }
    
    if(i==200){
        iCatch_debug();
    }    
}

void wait_for_AWB_ready(void){
    u16 testval, i;
             
    iCatch_first_open = false;
                
    for (i=0;i<15;i++)
    {
        sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72c3, &testval);
        printk("testval=0x%X, i=%d\n",testval,i);
        if (testval & 0x02) {
            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
            sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, &testval);
            printk("Clear testval=0x%X, i=%d\n",testval,i);
            break;
        }
        //printk("testval=0x%X, i=%d\n",testval,i);
        msleep(20);
    }
}

void enable_isp_interrupt(void){
    //Mask ISP interrupt
    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72fc, 0x04);
    //Enable ISP interrupt 
    init_completion(&g_iCatch_comp);
    INIT_COMPLETION(g_iCatch_comp);              
    enable_irq(imx091_s_ctrl.sensordata->sensor_platform_info->isp_int);  
}

void set_isp_calibration_table(int table){

    if(is_calibration_table_set == false){
        is_calibration_table_set = true;
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x717f, table);
    }
}

//ASUS_BSP +++ Stimber "Modify for preview/video frame rate"
void setFixFPS(int settingVal)
{
    int default_value = 0x00;
    
    if (settingVal == DEFAULT_SETTING) { //restore to default value
        if (g_LastFixFPS != DEFAULT_SETTING) { // Need to restore
            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7124, default_value);    // clear to restore
            g_LastFixFPS = DEFAULT_SETTING;
        } //else: regValue==g_LastMiniISO==default value : do nothing
    } else {
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7124, settingVal);
        g_LastFixFPS = settingVal;
    }
}

void setMaxExp(int settingVal)
{
    int default_value = 0x00;
    
    if (settingVal == DEFAULT_SETTING) { //restore to default value
        if (g_LastMaxExp != DEFAULT_SETTING) { // Need to restore
            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7125, default_value);    // clear to restore
            g_LastMaxExp = DEFAULT_SETTING;
        } //else: regValue==g_LastMiniISO==default value : do nothing
    } else {
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7125, settingVal);
        g_LastMaxExp = settingVal;
    }
}

void setMiniISO(int settingVal)
{
    int default_value = 0x00;
    int reg_val = 0x00;
    
    if (settingVal == DEFAULT_SETTING) { //restore to default value
        if (g_LastMiniISO != DEFAULT_SETTING) { // Need to restore
            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7129, default_value);    // clear to restore
            g_LastMiniISO = DEFAULT_SETTING;
        } //else: regValue==g_LastMiniISO==default value : do nothing
    } else {
        switch(settingVal){
            case 50:
                reg_val = 0x01;
                break;
            case 100:
                reg_val = 0x02;
                break;
            case 200:
                reg_val = 0x03;
                break;
            case 400:
                reg_val = 0x04;
                break;
            case 800:
                reg_val = 0x05;
                break;
            case 1600:
                reg_val = 0x06;
                break;
            default:
                reg_val = 0x00;
                break;
        }
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7129, reg_val);
        g_LastMiniISO = settingVal;
    }
}

void setCaptureVideoMode(int settingVal)
{
    int default_value = 0x00;
    
    if (settingVal == DEFAULT_SETTING) { //restore to default value
        if (g_LastVideoMode != DEFAULT_SETTING) { // Need to restore
            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7132, default_value);    // clear to restore
            g_LastVideoMode = DEFAULT_SETTING;
        } //else: regValue==g_LastVideoMode==default value : do nothing
    } else {
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7132, settingVal);
        g_LastVideoMode = settingVal;
    }
}
//ASUS_BSP --- Stimber "Modify for preview/video frame rate"

int sensor_set_mode(int  res)
{
       bool l_edgeExifEnable = false;
       
       //burst capture abort
       if((MSM_SENSOR_RES_FULL_BURST_CAPTURE == g_pre_res
             || MSM_SENSOR_RES_10M_BURST_CAPTURE == g_pre_res
		|| MSM_SENSOR_RES_MODE_11 == g_pre_res
             || (MSM_SENSOR_RES_FULL_SINGLE_CAPTURE == g_pre_res && g_is_nr_on))
             && g_pre_res != res){
             pr_info("%s: Burst capture abort\n",__func__);
             sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7122, 0x01);
       }

       if(g_is_max_exp_on){
           setMaxExp(0);
			g_is_max_exp_on = false;
           pr_info("%s: Reset max exp to default\n", __func__);
       }
       
       switch(res){
            case MSM_SENSOR_RES_QTR:            //MODE_1
                pr_info("%s: MODE_1\n",__func__);
		if(ultrapixel_mode == 1){
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7134, 0x01);
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x07);
		}
                //LiJen: Workaround: ignore MODE_7->MODE_1 & MODE_8->MODE_1 interrupt. 
                if((g_pre_res == MSM_SENSOR_RES_FULL_SINGLE_CAPTURE)|| MSM_SENSOR_RES_MODE_11 == g_pre_res || (g_pre_res == MSM_SENSOR_RES_FULL_BURST_CAPTURE)){
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }
                
                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(0); //default : capture preview mode
                
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7106, 0x06);//preview resolution
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode  

                break;
            case MSM_SENSOR_RES_4BIN:           //MODE_2
                pr_info("%s: MODE_2\n",__func__);

                setFixFPS(0);
                setMiniISO(400);
                setCaptureVideoMode(2); //High speed video preview mode
                
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7106, 0x05);//preview resolution
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode 
                
                break;
            case MSM_SENSOR_RES_FULL_HD:    //MODE_3
                pr_info("%s: MODE_3\n",__func__);

                setFixFPS(0);
                setMiniISO(400);
                setCaptureVideoMode(2); //High speed video preview mode
                
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7106, 0x02);//preview resolution
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode 

                break;
            case MSM_SENSOR_RES_FULL:           //MODE_4
                pr_info("%s: MODE_4\n",__func__);
				 //Stimber: Workaround: ignore MODE_8->MODE_4 interrupt.
                if((g_pre_res == MSM_SENSOR_RES_FULL_BURST_CAPTURE)){
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }

				 setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture

//ASUS_BSP +++ Stimber "Implement the interface for calibration"
			if(is_calibration){
				pr_info("Enter to calibration mode!!!\n");
				sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1011, 0x01);//cpu reset
				sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x941C, 0x04);
				sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9010, 0x01);
				sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9010, 0x00);
				sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1306, 0x02);//calibration
				sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1011, 0x00);

				sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7106, 0x08);//preview mode
				sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);

                g_calibrating = true;
//ASUS_BSP --- Stimber "Implement the interface for calibration"
             }else{
	            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7106, 0x08);//preview resolution
	            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode          
             }

                break;
            case MSM_SENSOR_RES_10M:            //MODE_5
		        pr_info("%s: MODE_5\n",__func__);

                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture
                
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7106, 0x07);//preview resolution
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode              
        
                break;
            case MSM_SENSOR_RES_HYBRID:     //MODE_6
                pr_info("%s: MODE_6\n",__func__);
                
                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(1); //Normal video preview mode
                
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7106, 0x09);//preview resolution
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode
                              
                break;
            case MSM_SENSOR_RES_FULL_SINGLE_CAPTURE:    //MODE_7
		if (ultrapixel_mode == 0){
                pr_info("%s: MODE_7\n",__func__);
                //LiJen: Workaround: ignore MODE_1->MODE_7 interrupt. 
                if((g_pre_res == MSM_SENSOR_RES_QTR)){
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }         

                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture
                    
                if(g_is_hdr_on){ //HDR
                    pr_info("[Camera] HDR mode\n");
		      sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7127, 0x15);//HDR posotive +1.5EV
		      sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7128, 0x15);//HDR nagative -1.5EV
		      
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710f, 0x01);//capture mode - HDR
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x01);//swtich capture mode
                }else if(g_is_nr_on){ //NR
                    pr_info("[Camera] NR mode\n");
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//swtich burst capture mode
                    if(MSM_SENSOR_RES_FULL == g_pre_res){
                        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode
                    }else{
                     	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x03);//swtich capture flash mode
                    } 
                }else{
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710f, 0x00);//swtich single capture mode
                     if(MSM_SENSOR_RES_FULL == g_pre_res){
                     	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode
                     }else{
                     	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x03);//swtich capture flash mode
                     }
                }    
	}else {   
	        pr_info("%s: MODE_11\n",__func__);
		//Stimber: Workaround: ignore MODE_4->MODE_11 & MODE_1->MODE_11 interrupt.
                if((g_pre_res == MSM_SENSOR_RES_FULL) || (g_pre_res == MSM_SENSOR_RES_QTR)){
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }
		
                setFixFPS(0);
                setMiniISO(0);
 
                setCaptureVideoMode(3); //Zsl capture
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710f, 0x00);//swtich burst capture mode
    
                //ASUS_BSP--- jim3_lin "Implement DIT postprocess-Mode2"
		if(MSM_SENSOR_RES_FULL == g_pre_res){
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode
		}else{
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x03);//swtich capture flash mode
		}
                //sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode
                //data[0]=0x02; //for multiwrite reg(0x7120)
	}
                break;
            case MSM_SENSOR_RES_FULL_BURST_CAPTURE:    //MODE_8
                pr_info("%s: MODE_8\n",__func__);
				 //Stimber: Workaround: ignore MODE_4->MODE_8 & MODE_1->MODE_8 interrupt.
                if((g_pre_res == MSM_SENSOR_RES_FULL) || (g_pre_res == MSM_SENSOR_RES_QTR)){
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }

                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture
                
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//swtich burst capture mode
                     if(MSM_SENSOR_RES_FULL == g_pre_res){
                     	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode
                     }else{
                     	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x03);//swtich capture flash mode
                     }                   

                break;
            case MSM_SENSOR_RES_10M_BURST_CAPTURE:    //MODE_10
                pr_info("%s: MODE_10\n",__func__);

                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture
                
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//swtich burst capture mode
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode  

                break;
		//ASUS_BSP_Evan "implement hi-light mode" +++
	case MSM_SENSOR_RES_MODE_11:    //MODE_11
  
               pr_info("%s: MODE_11\n",__func__);
		//Stimber: Workaround: ignore MODE_4->MODE_11 & MODE_1->MODE_11 interrupt.
                if((g_pre_res == MSM_SENSOR_RES_FULL) || (g_pre_res == MSM_SENSOR_RES_QTR)){
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }
		
                setFixFPS(0);
                setMiniISO(0);
                
                //ASUS_BSP+++ jim3_lin "Implement DIT postprocess-Mode2"
                //setCaptureVideoMode(3); //Zsl capture
                if( g_pre_res == MSM_SENSOR_RES_QTR ){ // for Hi-Light flag1 analog bin MODE_1->MODE_11->MODE_1
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//ASUS_BSP jim3_lin "Fix no torch on Hi-Light burst snapshot force flash mode"
                }
                else{
                    setCaptureVideoMode(3); //Zsl capture
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//swtich burst capture mode
                }
                //ASUS_BSP--- jim3_lin "Implement DIT postprocess-Mode2"

                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode               
                //data[0]=0x02; //for multiwrite reg(0x7120)
             break;   
		//ASUS_BSP_Evan "implement hi-light mode" ---             
            default:
                pr_err("%s: not support resolution res %d\n",__func__, res);
        	  return -EINVAL;                          
                break;
       }

       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7121, 0x01);//stream on

       if(true == iCatch_first_open){                  
            wait_for_AE_ready();
            enable_isp_interrupt();
            set_isp_calibration_table(0x01);
		iCatch_enable_autonight(true);
       }else{
       	wait_for_next_frame();	
	// jim fix HDR recieve 1st pink frame in 20ms +++
	if((g_pre_res == MSM_SENSOR_RES_FULL_SINGLE_CAPTURE || (g_pre_res == MSM_SENSOR_RES_MODE_11)) && g_is_hdr_on )
	{
		//pr_info("sleep 20ms for \n");
		msleep(20); // 20ms
	}
	// jim fix HDR recieve 1st pink frame in 20ms ---
       }

       //Enable EXIF                  
       // enableExif: when res is not equal to mode 1, mode2, mode3, mode6
       if(res == MSM_SENSOR_RES_QTR ||
          res == MSM_SENSOR_RES_4BIN ||
          res == MSM_SENSOR_RES_FULL_HD ||
          res == MSM_SENSOR_RES_HYBRID){
            l_edgeExifEnable = false;
       }else{
            l_edgeExifEnable = true;
       }
    
       if(res == MSM_SENSOR_RES_QTR ||
          res == MSM_SENSOR_RES_FULL ||
          res == MSM_SENSOR_RES_10M){
            iCatch_enable_exif(l_edgeExifEnable  , true);
       }else if(res == MSM_SENSOR_RES_FULL_SINGLE_CAPTURE ||
          res == MSM_SENSOR_RES_FULL_BURST_CAPTURE ||
          res == MSM_SENSOR_RES_10M_BURST_CAPTURE ||
	  res == MSM_SENSOR_RES_MODE_11){
            iCatch_enable_exif(l_edgeExifEnable, false);
       }else{
            iCatch_enable_exif(false, false);
       }
      
       g_pre_res = res;                
	return 0;
}

#define CONFIG_I2C_READ_WRITE
#ifdef CONFIG_I2C_READ_WRITE
#include <linux/debugfs.h>
#include <linux/uaccess.h>
//#include <stdio.h>
#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];
//static u32 i2c_set_value;
static u32 i2c_get_value;

static ssize_t i2c_set_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t i2c_get_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_chip_power_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#if 1
static ssize_t dbg_i7002a_fw_in_isp_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_i7002a_fw_in_isp_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
#if 0 
	int len = 0;
	int tot = 0;
	//char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	/* [Project id]-[FrontSensor]-[FW Version]*/   
	if (front_chip_id == SENSOR_ID_OV2720) {
		len = snprintf(bp, dlen, "%02X-%02X-%06X\n", tegra3_get_project_id(), 1, version_num_in_isp);
		tot += len; bp += len; dlen -= len;
	} else if (front_chip_id == SENSOR_ID_MI1040){
		/* mi1040 chip_id= 0x2481 */
		len = snprintf(bp, dlen, "%02X-%02X-%06X\n", tegra3_get_project_id(), 2, version_num_in_isp);
		tot += len; bp += len; dlen -= len;
	} else {
		len = snprintf(bp, dlen, "%02X-%02X-%06X\n", tegra3_get_project_id(), 0, version_num_in_isp);
		tot += len; bp += len; dlen -= len;
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
#endif    
       return 0;
}
#endif

static ssize_t dbg_i7002a_page_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int dbg_i7002a_page_dump_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
#if 0
    static int pfd  = 0;
    unsigned char buffer[512000];
    
    pfd = open("/data/BOOT_DUMP.BIN", O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    write(pfd , buffer, bytes);
    close(pfd);
#endif        
#if 1
	int len = 0;
	int tot = 0;
//	char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i =0;
	u8 mypage[0x100];

//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

       if(true == g_spi_is_support_1M){
            dbg_i7002a_page_index = 4095;
       }else{
	     dbg_i7002a_page_index = 2047;
       }
              
	len = snprintf(bp, dlen, "page_index=%d (0x%X)\n", dbg_i7002a_page_index, dbg_i7002a_page_index);
	tot += len; bp += len; dlen -= len;

	get_one_page_from_i7002a(dbg_i7002a_page_index, mypage);
	for(i=0; i < 0x100; i++) {
		if(i%16 == 0) {
			len = snprintf(bp, dlen, "[%03X] ", i);
			tot += len; bp += len; dlen -= len;
		}
		len = snprintf(bp, dlen, "%02X ", mypage[i]);
		tot += len; bp += len; dlen -= len;

		if(i%16 == 15) {
			len = snprintf(bp, dlen, "\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	//if (copy_to_user(buf, debug_buf, tot))
	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
#endif 
       return 0;
}
#if 0
void BB_I2CDumpDmem( 
        u32 dmemAddr, 
        u32 size 
) 
{ 
        u32 i, reg10a6=0, reg10a7=0, regData, temp1, temp2; 
        u8* pbuf; 
        u32 fd; 
        u8 ucFileName[30]; 

        pr_info("%s:: size: %d\n",  __func__, size); 

        pbuf = osMemAlloc(size); 
        pr_info("pbuf:0x%x\n",(u32)pbuf); 
        
        strcpy(ucFileName, "/data/DUMPDAT.BIN"); 

        sp5kFsFileDelete(ucFileName); 
        fd = sp5kFsFileOpen(ucFileName, SP5K_FS_OPEN_CREATE); 

        //I2CDataWrite(0x70c4,0x00); 
        //I2CDataWrite(0x70c5,0x00); 
        //I2CDataWrite(0x10a6,0x00); 
        //I2CDataWrite(0x10a7,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x70c4,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x70c5,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a6,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a7,0x00); 

        for(i=0;i<size;i++) 
        { 
                temp1 = 0x40|(dmemAddr / 0x2000); 
                temp2 = (dmemAddr - (temp1-0x40)*0x2000) / 0x800; 
                regData = 0x1800 + dmemAddr - (temp1-0x40)*0x2000 - temp2*0x800; 
                if(temp1 != reg10a6) 
                { 
                        reg10a6 = temp1; 
                        //hsI2CDataWrite(0x10a6,reg10a6); 
                        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a6,reg10a6);
                } 
                if(temp2 != reg10a7) 
                { 
                        reg10a7 = temp2; 
                        //hsI2CDataWrite(0x10a7,reg10a7); 
                        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a7,reg10a7);
                } 
                //*(pbuf+i) = hsI2CDataRead(regData); 
                *(pbuf+i) = sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x40e5);
                dmemAddr++; 
        } 
        /* inform 7002 read operation done */ 
        //I2CDataWrite(0x72C1,0x00); 
        //I2CDataWrite(0x10a6,0x00); 
        //I2CDataWrite(0x10a7,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72C1,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a6,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a7,0x00);         
        
        sp5kFsFileWrite(fd, pbuf, size); 
        sp5kFsFileClose(fd); 

        osMemFree(pbuf); 
} 
#endif
static ssize_t dbg_i7002a_bin_dump_dmem_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_i7002a_bin_dump_dmem_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len, tot = 0;
	//char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
//	int i = 0;
	//int ret = 0;
	char* mybin;
	struct file *fp_bin_dump = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;

        u32 i, reg10a6=0, reg10a7=0, regData, temp1, temp2; 
        u32 dmemAddr=0x28000;//starting dump address 
        u32 size=0x28000;//dump size 
        u16 temp;
        
//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x70c4,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x70c5,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a6,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a7,0x00);

	mybin = kmalloc(size, GFP_KERNEL);

        for(i=0;i<size;i++) 
        { 
                temp1 = 0x40|(dmemAddr / 0x2000); 
                temp2 = (dmemAddr - (temp1-0x40)*0x2000) / 0x800; 
                regData = 0x1800 + dmemAddr - (temp1-0x40)*0x2000 - temp2*0x800; 
                if(temp1 != reg10a6) 
                { 
                        reg10a6 = temp1; 
                        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a6,reg10a6);
                } 
                if(temp2 != reg10a7) 
                { 
                        reg10a7 = temp2; 
                        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a7,reg10a7);
                } 
                sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, regData, &temp);
                if(i <= 32){
                    pr_info("%d ",temp);
                }
                sprintf((mybin+i) , "%d", temp);
                dmemAddr++; 
        } 
        /* inform 7002 read operation done */ 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72C1,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a6,0x00); 
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x10a7,0x00);  

	/* Dump to /data/bin_dump.bin */
	fp_bin_dump = filp_open("/data/DUMPDAT.BIN", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
	if ( IS_ERR_OR_NULL(fp_bin_dump) ){
		filp_close(fp_bin_dump, NULL);
		len = snprintf(bp, dlen, "%s: open %s fail\n", __FUNCTION__, "/data/bin_dump.bin");
		tot += len; bp += len; dlen -= len;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offset = 0;
	if (fp_bin_dump->f_op != NULL && fp_bin_dump->f_op->write != NULL){
		fp_bin_dump->f_op->write(fp_bin_dump,
			mybin,
			size,
			&offset);
	} else {
		len = snprintf(bp, dlen, "%s: f_op might be null\n", __FUNCTION__);
		tot += len; bp += len; dlen -= len;
	}
	set_fs(old_fs);
	filp_close(fp_bin_dump, NULL);
	kfree(mybin);

	len = snprintf(bp, dlen, "%s: Dump Complete.\n", __FUNCTION__);
	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_i7002a_bin_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_i7002a_bin_dump_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len, tot = 0;
	//char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
//	int i = 0;
	int ret = 0;
	char* mybin;
	struct file *fp_bin_dump = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;

//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	//I2CDataWrite(0x70c4,0x00);
	//I2CDataWrite(0x70c5,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x70c4,0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x70c5,0x00);

	ret = I2C_SPIInit();
	if (ret) {
		printk("%s: get nothing. ret= %d", __FUNCTION__, ret);
		return ret;
	}

	I2C_SPIFlashReadId();

	if (true == g_spi_is_support_1M) {
		mybin = kmalloc(1024*1024, GFP_KERNEL);
		I2C_SPIFlashRead(0, 4096, mybin);
	}
	else {
		mybin = kmalloc(512*1024, GFP_KERNEL);
		I2C_SPIFlashRead(0, 2048, mybin);
	}
    
	/* Dump to /data/bin_dump.bin */
	fp_bin_dump = filp_open("/data/bin_dump.bin", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
	if ( IS_ERR_OR_NULL(fp_bin_dump) ){
		filp_close(fp_bin_dump, NULL);
		len = snprintf(bp, dlen, "%s: open %s fail\n", __FUNCTION__, "/data/bin_dump.bin");
		tot += len; bp += len; dlen -= len;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offset = 0;
	if (fp_bin_dump->f_op != NULL && fp_bin_dump->f_op->write != NULL){
		if (true == g_spi_is_support_1M)
			fp_bin_dump->f_op->write(fp_bin_dump, mybin, 1024*1024, &offset);
		else
			fp_bin_dump->f_op->write(fp_bin_dump, mybin, 512*1024, &offset);        
	} else {
		len = snprintf(bp, dlen, "%s: f_op might be null\n", __FUNCTION__);
		tot += len; bp += len; dlen -= len;
	}
	set_fs(old_fs);
	filp_close(fp_bin_dump, NULL);
	kfree(mybin);

	len = snprintf(bp, dlen, "%s: Dump Complete.\n", __FUNCTION__);
	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_i7002a_fw_header_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#define DUMP_HEADER(mypage) do {	\
		for(i = 0; i < 0x100; i++) {	\
			if(i%16 == 0) {	\
				len = snprintf(bp, dlen, "[%02X] ", i);	\
				tot += len; bp += len; dlen -= len;	\
			}	\
			len = snprintf(bp, dlen, "%02X ", mypage[i]);	\
			tot += len; bp += len; dlen -= len;	\
			if(i%16 == 15) {	\
				len = snprintf(bp, dlen, "\n");	\
				tot += len; bp += len; dlen -= len;	\
			}	\
		}	\
	} while (0)


static ssize_t dbg_i7002a_fw_header_dump_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
#if 0
	int len = 0;
	int tot = 0;
	char debug_buf[3072];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i =0;
	u8 fw1page[0x100];
	u8 fw2page[0x100];
	u8 overallpage[0x100];
	int fw2_header_page_index, fw2_offset = 0;

//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	i7002a_isp_on(1);

	/* dump fw1 header */
	get_one_page_from_i7002a(0, fw1page);
	len = snprintf(bp, dlen, "fw1: page[%d]:\n", 0);
	tot += len; bp += len; dlen -= len;
	DUMP_HEADER(fw1page);

	msleep(40);

	/* dump fw2 header */
	fw2_offset = 16 +
		((fw1page[3] << 24) | (fw1page[2] << 16) | (fw1page[1] << 8) | fw1page[0]) +
		((fw1page[7] << 24) | (fw1page[6] << 16) | (fw1page[5] << 8) | fw1page[4]);
	fw2_header_page_index = fw2_offset >> 8;
	get_one_page_from_i7002a(fw2_header_page_index, fw2page);
	len = snprintf(bp, dlen, "fw2: page[%d]:\n", fw2_header_page_index);
	tot += len; bp += len; dlen -= len;
	DUMP_HEADER(fw2page);

	msleep(40);

	/* dump overall header */
	get_one_page_from_i7002a(2047, overallpage);
	len = snprintf(bp, dlen, "Overall: page[%d]:\n", 2047);
	tot += len; bp += len; dlen -= len;
	DUMP_HEADER(overallpage);

	if (front_chip_id == SENSOR_ID_OV2720) {
		if(memcmp(ov2720_fw1page_0_1_21, fw1page, 0x100) == 0) {
			if(memcmp(ov2720_fw2page_0_1_21, fw2page, 0x100) == 0) {
				if(memcmp(ov2720_overallpage_0_1_21, overallpage, 0x100) == 0) {
					b_fw_is_valid = 1;
				} else {
					len = snprintf(bp, dlen, "%s(%d): wrong overall page.\n", __FUNCTION__, __LINE__);
					tot += len; bp += len; dlen -= len;
					b_fw_is_valid = 0;
				}
			} else {
				len = snprintf(bp, dlen, "%s(%d): wrong fw2 page.\n", __FUNCTION__, __LINE__);
				tot += len; bp += len; dlen -= len;
				b_fw_is_valid = 0;
			}
		} else {
			len = snprintf(bp, dlen, "%s(%d): wrong fw1 page.\n", __FUNCTION__, __LINE__);
			tot += len; bp += len; dlen -= len;
			b_fw_is_valid = 0;
		}
	} else if (front_chip_id == SENSOR_ID_MI1040) {
		if(memcmp(mi1040_fw1page_0_1_21, fw1page, 0x100) == 0) {
			if(memcmp(mi1040_fw2page_0_1_21, fw2page, 0x100) == 0) {
				if(memcmp(mi1040_overallpage_0_1_21, overallpage, 0x100) == 0) {
					b_fw_is_valid = 1;
				} else {
					len = snprintf(bp, dlen, "%s(%d): wrong overall page.\n", __FUNCTION__, __LINE__);
					tot += len; bp += len; dlen -= len;
					b_fw_is_valid = 0;
				}
			} else {
				len = snprintf(bp, dlen, "%s(%d): wrong fw2 page.\n", __FUNCTION__, __LINE__);
				tot += len; bp += len; dlen -= len;
				b_fw_is_valid = 0;
			}
		} else {
			len = snprintf(bp, dlen, "%s(%d): wrong fw1 page.\n", __FUNCTION__, __LINE__);
			tot += len; bp += len; dlen -= len;
			b_fw_is_valid = 0;
		}
	} else {
		len = snprintf(bp, dlen, "%s(%d): Unknown Front Camera ID.\n", __FUNCTION__, __LINE__);
		tot += len; bp += len; dlen -= len;
		b_fw_is_valid = 0;
	}

	i7002a_isp_on(0);

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
#endif
        return 0;
}

static ssize_t dbg_fw_update_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_fw_update_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[512];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	//printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	switch(fw_update_status) {
	case ICATCH_FW_NO_CMD:
		len = snprintf(bp, dlen, "Never issue fw update cmd yet.\n");
		tot += len; bp += len; dlen -= len;
		break;

	case ICATCH_FW_IS_BURNING:
		if ((g_page_count >= 0) && (g_page_count <= g_total_page_count)) {
			int time_left = 0;
			if (flash_type == ICATCH_FLASH_TYPE_ST)
				time_left = g_page_count * 8 / 100;
			else
				time_left = g_page_count / 4;

			len = snprintf(bp, dlen, "FW update progress: %d/%d; Timeleft= %d secs\n", g_total_page_count - g_page_count + 1, g_total_page_count, time_left);
			tot += len; bp += len; dlen -= len;
		} else {
			len = snprintf(bp, dlen, "g_page_count=%d; total=%d\n", g_page_count, g_total_page_count);
			tot += len; bp += len; dlen -= len;
		}
		break;

	case ICATCH_FW_UPDATE_SUCCESS:
		len = snprintf(bp, dlen, "FW Update Complete!\n");
		tot += len; bp += len; dlen -= len;
		break;

	case ICATCH_FW_UPDATE_FAILED:
		len = snprintf(bp, dlen, "FW Update FAIL!\n");
		tot += len; bp += len; dlen -= len;
		break;

	default:
		len = snprintf(bp, dlen, "FW Update Status Unknown: %d\n", fw_update_status);
		tot += len; bp += len; dlen -= len;
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_fw_update_write(struct file *file, const char __user *buf, size_t count,
				loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	char bin_path[80];

	//printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;

	debug_buf[count] = '\0';	/* end of string */
	cnt = sscanf(debug_buf, "%s", bin_path);

	/* burning */
	printk("%s: BB_WrSPIFlash()++\n", __FUNCTION__);
	BB_WrSPIFlash(bin_path);
	printk("%s: BB_WrSPIFlash()--\n", __FUNCTION__);
    
	return count;
}

static ssize_t i2c_set_write(struct file *file, const char __user *buf, size_t count,
				loff_t *ppos)
{
  int len;
  int arg[2];
  //int gpio, set;

  //char gpioname[8];

//  printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
  arg[0]=0;

	if (*ppos)
		return 0;	/* the end */

//+ parsing......
  len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
  if (copy_from_user(debugTxtBuf,buf,len))
		return -EFAULT;

  debugTxtBuf[len]=0; //add string end

  sscanf(debugTxtBuf, "%x %x", &arg[0], &arg[1]);
  printk("argument is arg1=0x%x arg2=0x%x\n",arg[0], arg[1]);


  *ppos=len;
  sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, arg[0], arg[1]);

	return len;	/* the end */
}
/*
static ssize_t i2c_config_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{

}
*/

static ssize_t i2c_get_write(struct file *file, const char __user *buf, size_t count,
				loff_t *ppos)
{
  int len;
  int arg = 0;
  //int gpio, set;

  //char gpioname[8];

//  printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);


	if (*ppos)
		return 0;	/* the end */

//+ parsing......
  len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
  if (copy_from_user(debugTxtBuf,buf,len))
		return -EFAULT;

  debugTxtBuf[len]=0; //add string end

  sscanf(debugTxtBuf, "%x", &arg);
  printk("argument is arg=0x%x\n",arg);


  *ppos=len;
  sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, arg, (u16 *)&i2c_get_value);

	return len;	/* the end */
}

static ssize_t i2c_get_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	char *bp = debugTxtBuf;

       if (*ppos)
		return 0;	/* the end */
	len = snprintf(bp, DBG_TXT_BUF_SIZE, "the value is 0x%x\n", i2c_get_value);

	if (copy_to_user(buf, debugTxtBuf, len))
		return -EFAULT;
       *ppos += len;
	return len;

}

static ssize_t dbg_iCatch7002a_vga_status_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_vga_status_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
#if 0
	int len = 0;
	int tot = 0;
	//char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	u16 chip_id, tmp = 0x0;

	//printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	if (sensor_opened == false) {
		if (1) {
			//info->pdata->power_on();
			//tegra_camera_mclk_on_off(1);
			msleep(100);
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_on.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	/*Start - Power on sensor & enable clock - Front I2C (OV2720)*/
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x0084, 0x14); /* To sensor clock divider */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x0034, 0xFF); /* Turn on all clock */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9030, 0x3f);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9031, 0x04);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9034, 0xf3);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9035, 0x04);

	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9032, 0x02);
	msleep(10);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9032, 0x00);
	msleep(10);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9033, 0x00);
	msleep(10);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9033, 0x04);
	msleep(10);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9034, 0xf2);
	/*End - Power on sensor & enable clock */

	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9008, 0x00); /* Need to check with vincent */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9009, 0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x900A, 0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x900B, 0x00);

	/*Start - I2C Read*/
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9138, 0x30); /* Sub address enable */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9140, 0x6C); /* Slave address      */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9100, 0x03); /* Read mode          */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9110, 0x30); /* Register addr MSB  */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9112, 0x0a); /* Register addr LSB  */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9104, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9111, &tmp);

	//printk("0x%x\n", tmp);
	chip_id = (tmp << 8) & 0xFF00;

	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9110, 0x30); /* Register addr MSB  */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9112, 0x0b); /* Register addr LSB  */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9104, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9111, &tmp);
	//printk("0x%x\n", tmp);
	chip_id = chip_id  | (tmp & 0xFF);

	if (chip_id == SENSOR_ID_OV2720) {
		len = snprintf(bp, dlen, "1\n");
		tot += len; bp += len; dlen -= len;
	} else {
#if 0
		len = snprintf(bp, dlen, "back chip_id= 0x%x\n", chip_id);
		tot += len; bp += len; dlen -= len;
#endif
		/* Check if mi1040 is available. */
		sensor_write_table(imx091_s_ctrl.sensor_i2c_client->client, query_mi1040_id_msb_seq);
		sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9111, &tmp);

		chip_id = (tmp << 8) & 0xFF00;

		sensor_write_table(imx091_s_ctrl.sensor_i2c_client->client, query_mi1040_id_lsb_seq);
		sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9111, &tmp);
		chip_id = chip_id  | (tmp & 0xFF);

		printk("0x%x\n", chip_id);

		if (chip_id == SENSOR_ID_MI1040) {
			/* mi1040 chip_id= 0x2481 */
			len = snprintf(bp, dlen, "1\n");
			tot += len; bp += len; dlen -= len;
		} else {
			len = snprintf(bp, dlen, "0\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (sensor_opened == false) {
		if (1) {
			//tegra_camera_mclk_on_off(0);
			//info->pdata->power_off();
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_off.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
#endif
       return 0;
}


static ssize_t dbg_iCatch7002a_camera_status_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_camera_status_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	u16 chip_id, tmp = 0x0;

	//printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	if (sensor_opened == false) {
		if (1) {
			//info->pdata->power_on();
			//tegra_camera_mclk_on_off(1);
			msleep(100);
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_on.\n");
			tot += len; bp += len; dlen -= len;
		}
	}
	/* SONY IMX175 */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x0084, 0x14); /* To sensor clock divider */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x0034, 0xFF); /* Turn on all clock */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9030, 0x3f);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9031, 0x04);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9034, 0xf2);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9035, 0x04);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9032, 0x00);
	msleep(10);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9032, 0x20);
	msleep(10);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9032, 0x30);
	msleep(10);
	/*End - Power on sensor & enable clock */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9008, 0x00); /* Need to check with vincent */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9009, 0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x900A, 0x00);
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x900B, 0x00);

	/*Start - I2C Read*/
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9238, 0x30); /* Sub address enable */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9240, 0x20); /* Slave address      */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9200, 0x03); /* Read mode          */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9210, 0x00); /* Register addr MSB  */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9212, 0x00); /* Register addr LSB  */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9204, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9211, &tmp);
	// printk("0x%x\n", tmp);
	chip_id = (tmp << 8) & 0xFF00;

	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9210, 0x00); /* Register addr MSB  */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9212, 0x01); /* Register addr LSB  */
	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9204, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x9211, &tmp);
	// printk("0x%x\n", tmp);
	chip_id = chip_id  | (tmp & 0xFF);

	if (chip_id == SENSOR_ID_IMX175) {
		len = snprintf(bp, dlen, "1\n");
		tot += len; bp += len; dlen -= len;
	} else {
#if 0
		len = snprintf(bp, dlen, "back chip_id= 0x%x\n", chip_id);
		tot += len; bp += len; dlen -= len;
#endif
		len = snprintf(bp, dlen, "0\n");
		tot += len; bp += len; dlen -= len;
	}

	if (sensor_opened == false) {
		if (1) {
			//tegra_camera_mclk_on_off(0);
			//info->pdata->power_off();
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_off.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_iCatch7002a_chip_power_write(struct file *file, const char __user *buf, size_t count,
				loff_t *ppos)
{
    int len;
    int arg;
    //int gpio, set;

    //char gpioname[8];

    //  printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
    arg=0;

    if (*ppos)
        return 0;	/* the end */

    //+ parsing......
    len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
    if (copy_from_user(debugTxtBuf,buf,len))
    		return -EFAULT;

    debugTxtBuf[len]=0; //add string end

    sscanf(debugTxtBuf, "%x", &arg);
    printk("argument is arg=0x%x\n",arg);


    *ppos=len;
    //sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, arg[0], arg[1]);
    if (arg==1)  //power on
    {
        pr_info("imx091 power_on\n");
        imx091_power_up((struct msm_camera_sensor_info *)&imx091_s_ctrl.sensordata, false);
    }else //power down 
    {
        pr_info("imx091 power_down\n");
        imx091_power_down((struct msm_camera_sensor_info *)&imx091_s_ctrl.sensordata, false);
    }
    	return len;	/* the end */
}

static const struct file_operations dbg_i7002a_fw_in_isp_fops = {
	.open		= dbg_i7002a_fw_in_isp_open,
	.read		= dbg_i7002a_fw_in_isp_read,
};

static const struct file_operations dbg_i7002a_page_dump_fops = {
	.open		= dbg_i7002a_page_dump_open,
	.read		= dbg_i7002a_page_dump_read,
};

static const struct file_operations dbg_i7002a_bin_dump_fops = {
	.open		= dbg_i7002a_bin_dump_open,
	.read		= dbg_i7002a_bin_dump_read,
};

static const struct file_operations dbg_i7002a_bin_dump_dmem_fops = {
	.open		= dbg_i7002a_bin_dump_dmem_open,
	.read		= dbg_i7002a_bin_dump_dmem_read,
};

static const struct file_operations dbg_fw_update_fops = {
	.open		= dbg_fw_update_open,
	.read		= dbg_fw_update_read,
	.write = dbg_fw_update_write,
};

static const struct file_operations i2c_set_fops = {
	.open		= i2c_set_open,
	//.read		= i2c_config_read,
	//.llseek		= seq_lseek,
	//.release	= single_release,
	.write = i2c_set_write,
};

static const struct file_operations i2c_get_fops = {
	.open		= i2c_get_open,
	.read		= i2c_get_read,
	//.llseek		= seq_lseek,
	//.release	= single_release,
	.write = i2c_get_write,
};

static const struct file_operations dbg_iCatch7002a_vga_status_fops = {
	.open		= dbg_iCatch7002a_vga_status_open,
	.read		= dbg_iCatch7002a_vga_status_read,
};

static const struct file_operations dbg_iCatch7002a_camera_status_fops = {
	.open		= dbg_iCatch7002a_camera_status_open,
	.read		= dbg_iCatch7002a_camera_status_read,
};

static const struct file_operations dbg_i7002a_fw_header_dump_fops = {
	.open		= dbg_i7002a_fw_header_dump_open,
	.read		= dbg_i7002a_fw_header_dump_read,
};

static const struct file_operations iCatch7002a_power_fops = {
	.open		= dbg_iCatch7002a_chip_power_open,
	//.read		= i2c_get_read,
	//.llseek		= seq_lseek,
	//.release	= single_release,
	.write = dbg_iCatch7002a_chip_power_write,
};

int icatch_i2c_debuginit(void)
{
       struct dentry *dent = debugfs_create_dir("i7002a", NULL);

	(void) debugfs_create_file("fw_in_isp", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_fw_in_isp_fops);

	(void) debugfs_create_file("page_dump", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_page_dump_fops);

	(void) debugfs_create_file("bin_dump", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_bin_dump_fops);

    	(void) debugfs_create_file("bin_dump_dmem", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_bin_dump_dmem_fops);
        
	(void) debugfs_create_file("fw_header_dump", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_fw_header_dump_fops);

	(void) debugfs_create_file("fw_update", S_IRWXUGO,
					dent, NULL, &dbg_fw_update_fops);

	(void) debugfs_create_file("i2c_set", S_IRWXUGO,
					dent, NULL, &i2c_set_fops);

	debugfs_create_u32("b_fw_is_valid", S_IRWXUGO, dent, &b_fw_is_valid);

	(void) debugfs_create_file("i2c_get", S_IRWXUGO,
					dent, NULL, &i2c_get_fops);
	(void) debugfs_create_file("camera_status", S_IRWXUGO, dent, NULL, &dbg_iCatch7002a_camera_status_fops);
	(void) debugfs_create_file("vga_status", S_IRWXUGO, dent, NULL, &dbg_iCatch7002a_vga_status_fops);
	(void) debugfs_create_file("iCatch_chip_power", S_IRWXUGO, dent, NULL, &iCatch7002a_power_fops);

//ASUS_BSP +++ Stimber "Implement the interface for calibration"
	if (debugfs_create_u32("is_calibration", S_IRWXUGO, dent, &is_calibration) == NULL) 
	{			
		printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",	__FILE__, __LINE__);
		return -1;		
	}	
	
	if (debugfs_create_u32("calibrating", S_IRWXUGO, dent, &calibrating) == NULL) 
	{			
		printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n", __FILE__, __LINE__);
		return -1;		
	}
//ASUS_BSP --- Stimber "Implement the interface for calibration"
//ASUS_BSP +++ Stimber "Interface for single image"
	if (debugfs_create_u32("single_image", S_IRWXUGO, dent, &single_image) == NULL) 
	{			
		printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n", __FILE__, __LINE__);
		return -1;		
	}
//ASUS_BSP --- Stimber "Interface for single image"

#ifdef ICATCH7002A_DELAY_TEST
              if (debugfs_create_u32("iCatch7002a_delay", S_IRWXUGO, dent, &iCatch7002a_init_delay)
		== NULL) {
                printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
		__FILE__, __LINE__);
                return -1;
              }
	if (debugfs_create_u32("iCatch7002a_preview_delay", S_IRWXUGO, dent, &iCatch7002a_preview_delay)
		== NULL) {
                printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
		__FILE__, __LINE__);
                return -1;
              }
              if (debugfs_create_u32("touch_focus_enable", S_IRWXUGO, dent, &touch_focus_enable)
		== NULL) {
                printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
		__FILE__, __LINE__);
                return -1;
              }
#endif
             debugfs_create_u32("page_index",S_IRWXUGO, dent, &dbg_i7002a_page_index);
#ifdef CAM_TWO_MODE
             debugfs_create_u32("div",S_IRWXUGO, dent, &g_div);
	return 0;
#endif
}

#endif

//ASUS_BSP +++ LiJen "[A68][ISP][NA][Others]add proc file for AP ISP update"
// create proc file
#ifdef	CONFIG_PROC_FS
#define	ICATCH_PROC_FILE	"driver/iCatch"
static struct proc_dir_entry *iCatch_proc_file;

#define	ICATCH_FIRMWARE_VERSION_PROC_FILE	"driver/isp_fw_version"
static struct proc_dir_entry *iCatch_fw_version_proc_file;

static ssize_t iCatch_fw_version_proc_read(char *page, char **start, off_t off, int count,
            	int *eof, void *data)
{
	int len=0;
       
	if(*eof == 0){
		if(count>8) {
			len+=sprintf(page+len, "%x\n", version_num_in_isp);
			pr_info("%s:X string=%s", __func__, (char *)page);
		} else {
			len=-1;
		}
		*eof = 1;
	}
    
	return len;
}

static ssize_t iCatch_fw_version_proc_write(struct file *filp, const char __user *buff,
	            unsigned long len, void *data)
{
	pr_info("%s\n",__func__);
	return len;
}

static ssize_t iCatch_proc_read(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	int len=0;

	if(*eof == 0){
		u32 BinVersionNum = 0;
              u32 ISPVersionNum = 0;
              
		ISPVersionNum = version_num_in_isp;
		BinVersionNum = get_fw_version_in_bin(BIN_FILE_WITH_PATH);

		pr_info("ISPVersionNum=%x, BinVersionNum=%x--\n", ISPVersionNum, BinVersionNum);
		if((ISPVersionNum==0xffffff)||(ISPVersionNum<BinVersionNum)) {
                            if(g_A68_hwID >= A80_SR5){ //LiJen: V07.10.xx only support A80_SR5 and later.
				    len+=sprintf(page+len, "%x %x\n%x\n", NEED_UPDATE, ISPVersionNum, BinVersionNum);
                            }else{
                                if(BinVersionNum < 0x071000){
                                    len+=sprintf(page+len, "%x %x\n%x\n", NEED_UPDATE, ISPVersionNum, BinVersionNum);
                                }else{
                                    pr_info("HWID(0x%x) < A80_SR5, UPDATE_UNNECESSARY\n",g_A68_hwID);
                                    len+=sprintf(page+len, "%x %x\n%x\n", UPDATE_UNNECESSARY, ISPVersionNum, BinVersionNum);
                                }
                            }
		}else{
				len+=sprintf(page+len, "%x %x\n%x\n", UPDATE_UNNECESSARY, ISPVersionNum, BinVersionNum);
		}
        
		*eof = 1;
		pr_info("%s:X string=%s", __func__, (char *)page);
	}
	return len;
}

static ssize_t iCatch_proc_write(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	static char messages[256]="";

	if (len > 256)
		len = 256;

	memset(messages, 0, 256);
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
        
	pr_info("%s %s\n", __func__, messages);

	if (strlen(messages)<=0) {
	     pr_info("command not support\n");
	} else {
	       if('d' == messages[0]){
                    if('1' == messages[1]){
                        g_enable_roi_debug = true;
                        pr_info("%s Enable ROI debug\n",__func__);
                    }else if('0' == messages[1]){
                        g_enable_roi_debug = false;
                        pr_info("%s Disable ROI debug\n",__func__);
                    }
              }else{	
        		struct file *fp = NULL;
        		int str_len = strlen(messages);
        		messages[str_len-1] = 0;

        		if(iCatch_is_updating==0) {
        			iCatch_is_updating=1;
        		
        			pr_info("test filp_open %s--\n", messages);
        			fp = filp_open(messages, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
        			if ( !IS_ERR_OR_NULL(fp) ){
        				UPDATE_FILE_WITH_PATH = messages;
        				filp_close(fp, NULL);
        			} else {
        				pr_info("choose system A68-BOOT.BIN\n");
        				UPDATE_FILE_WITH_PATH = BIN_FILE_WITH_PATH;
        			}
        			
        			/* Update ISP firmware*/
        	             pr_info("iCatch firmware update start\n");
                           imx091_power_up((struct msm_camera_sensor_info *)&imx091_s_ctrl.sensordata, true);
        	             BB_WrSPIFlash(UPDATE_FILE_WITH_PATH);
                           imx091_power_down((struct msm_camera_sensor_info *)&imx091_s_ctrl.sensordata, true);
        	             iCatch_is_updating=0;
        		} 
                }
	}

	return len;
}

void create_iCatch_proc_file(void)
{
    iCatch_proc_file = create_proc_entry(ICATCH_PROC_FILE, 0666, NULL);
    if (iCatch_proc_file) {
		iCatch_proc_file->read_proc = iCatch_proc_read;
		iCatch_proc_file->write_proc = iCatch_proc_write;
    } else{
        pr_err("proc file create failed!\n");
    }

    iCatch_fw_version_proc_file = create_proc_entry(ICATCH_FIRMWARE_VERSION_PROC_FILE, 0666, NULL);
    if (iCatch_fw_version_proc_file) {
		iCatch_fw_version_proc_file->read_proc = iCatch_fw_version_proc_read;
		iCatch_fw_version_proc_file->write_proc = iCatch_fw_version_proc_write;
    } else{
        pr_err("proc file iCatch fw version create failed!\n");
    }
}

void remove_iCatch_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    pr_info("%s\n",__func__);
    remove_proc_entry(ICATCH_PROC_FILE, &proc_root);
    remove_proc_entry(ICATCH_FIRMWARE_VERSION_PROC_FILE, &proc_root);
}
#endif // end of CONFIG_PROC_FS
//ASUS_BSP --- LiJen "[A68][ISP][NA][Others]add proc file for AP ISP update"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]add 13M camera TAF support
void iCatch_set_touch_AF(isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w){
	u32 af_w=0x80, af_h=0x80, af_x=0x33, af_y=0x33;
       const u32 roi_bytes = 11;
       unsigned char data[roi_bytes];    
	pr_info("%s +++\n",__func__);
	pr_info("%s: coordinate_x:0x%x coordinate_y:0x%x rectangle_h:0x%x rectangle_w:0x%x\n", __func__, coordinate_x, coordinate_y, rectangle_h, rectangle_w);

	// get preview resolution from ISP
	if(coordinate_x == -1){
		af_x = 0x80;  // ISP default
	}else if(coordinate_x > 0x0400){
		af_x = 0x0400;
	}else if(coordinate_x < 0){
		af_x = 0x0;
	}else{
		af_x = coordinate_x;
	}
	
	if(coordinate_y == -1){
		af_y = 0x80;  // ISP default
	}else if(coordinate_y > 0x0400){
		af_y = 0x0400;
	}else if(coordinate_y < 0){
		af_y = 0x0;
	}else{
		af_y = coordinate_y;
	}

	if(rectangle_w == -1){
		af_w = 0x33;  // ISP default
	}else if(rectangle_w > 0x0400){
		af_w = 0x0400;
	}else if(rectangle_w < 0){
		af_w = 0x0;
	}else{
		af_w = rectangle_w;
	}

	if(rectangle_h == -1){
		af_h = 0x33;  // ISP default
	}else if(rectangle_h > 0x0400){
		af_h = 0x0400;
	}else if(rectangle_h < 0){
		af_h = 0x0;
	}else{
		af_h = rectangle_h;
	}	

	pr_info("%s: af_x:0x%x af_y:0x%x af_w:0x%x af_h:0x%x g_TAEenable:%d\n", __func__, af_x, af_y, af_w, af_h,g_TAEenable);

       // set focus coodinate and retangle
       if(AF_MODE_CAF == mode){
            sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7105, 0x00); // iCatch:ROI only vaild in focus mode = auto
       }
       //AF ROI
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7188, 0x01);//ROI on
#if 0       
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7140, (af_w >> 8));
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7141, (af_w & 0xFF));
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7142, (af_x >> 8));
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7143, (af_x & 0xFF));
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7144, (af_y >> 8));
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7145, (af_y & 0xFF));
#else 
       //0x7140
       data[0] = (af_w >> 8);
       //0x7141
       data[1] = 0x41;
       data[2] = (af_w & 0xFF);
       //0x7142
       data[3] = 0x42;
       data[4] = (af_x >> 8);
       //0x7143
       data[5] = 0x43;
       data[6] = (af_x & 0xFF);
       //0x7144
       data[7] = 0x44;
       data[8] = (af_y >> 8);
       //0x7145
       data[9] = 0x45;
       data[10] = (af_y & 0xFF);
       sensor_write_reg_bytes(imx091_s_ctrl.sensor_i2c_client->client, 0x7140, data, roi_bytes);
#endif
       //AE trigger
       if(g_TAEenable == true){
           sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x714e, 0x02);      
       }
       
       // AF trigger
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7146, 0x01);    
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72a0, 0x01);     // Clean AF state bit
              
	pr_info("%s ---\n",__func__);
}

void iCatch_wait_AF_done(void){
        int retry=0;
        u16 status;
			
        //Wait AF interrupt
        g_isAFDone = false;
        do{
                mutex_unlock(imx091_s_ctrl.msm_sensor_mutex);
                if(retry==0){
                    msleep(120); // LiJen: wait for ISP AF process
                }else{
                    msleep(15);
                }
                mutex_lock(imx091_s_ctrl.msm_sensor_mutex);
                if(g_isAFCancel == true){
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x714f, 0x01); 
                    pr_info("%s g_isAFCancel = ture\n",__func__);
                    break;
                }
                 sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72a0, &status);
                 pr_info("status=0x%X, retry=%d\n",status,retry);
                 retry += 1;
        } while((status != 0x00) && (retry < 135));    
        g_isAFDone = true;
}

void iCatch_start_AF(bool on, isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w)
{
	//int retry=0;	
       //u16 status;
	pr_info("%s +++ Param(%d,%d,%d,%d,%d,%d)\n",__func__,on,mode,coordinate_x,coordinate_y,rectangle_h,rectangle_w);
	g_isAFCancel = false;		
	//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement cancel autofocus in 8M camera with ISP"
	if(on){
            switch(mode) {
                case AF_MODE_MACRO:
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7105, 0x01); 
                    //Any point focus setting
        	      iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);
                    break;
                case AF_MODE_CAF:
                    //Any point focus setting
        	      iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);                    
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7105, 0x03);  
                    break;                         
                case AF_MODE_AUTO:
                    if(g_afmode == 0){
                        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7105, 0x00); // auto af
                    }else{
                        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7105, 0x04); // full search af
                    }
                    //Any point focus setting
        	      iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);                    
                    //Wait AF done
                    iCatch_wait_AF_done();   
                    break;                                              
                case AF_MODE_NORMAL: //LiJen: normal focus instead of full search af
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7105, 0x04);   
                    //Any point focus setting
        	      iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);                    
                    //Wait AF done
                    iCatch_wait_AF_done();                        
                    break;                     
                case AF_MODE_INFINITY:
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7105, 0x02);  
                    goto end;                    
                case AF_MODE_UNCHANGED:    
                case AF_MODE_MAX:  
                default:
                    pr_info("%s mode(%d) is not support \n",__func__,mode);
                    goto end;          
            }

            //Enable ROI debug
            if(true == g_enable_roi_debug){
                if(AF_MODE_AUTO != mode){
                    //Wait AF done
            	      iCatch_wait_AF_done();         
                }
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x243f, 0x01);
            }                  
       
	}else{
		//Cancel AutoFocus
              //AF_START: AF release
              g_isAFCancel = true;
              //sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x714f, 0x01);  
		pr_info("Cancel autofocus\n");
	}
    
end:    
	pr_info("%s ---\n",__func__);
}

uint16_t iCatch_get_AF_result(struct msm_sensor_ctrl_t *s_ctrl)
{
	u16 status;
	bool result=false;
	
	pr_info("%s +++\n",__func__);
	if(!caf_mode){
		//Read AF result
		sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72a1, &status);
		switch(status)
		{
			case 0:
			{
				pr_info("AF success\n");
				result = true;
				break;
			}
			case 1:
			{
				pr_info("AF fail\n");
				result = false;
				break;
			}
		}
	}
	else{
		result = false;
		pr_err("CAF is starting\n");
	}
	
	pr_info("%s result(%d)---\n",__func__,result);
	return result;
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]add 13M camera TAF support

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement LED/Flash mode"
void iCatch_set_led_mode(led_mode_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);
	switch(mode)
	{
		case LED_MODE_ON:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7104, 0x02);
					 g_flash_mode = 1;
			break;
		case LED_MODE_OFF:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7104, 0x01);
					 g_flash_mode = 0;
			break;
		case LED_MODE_AUTO:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7104, 0x00);
					 g_flash_mode = 2;
			break;
		case LED_MODE_TORCH:
                   sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7104, 0x04);
				   g_flash_mode = 3;
			break;
              default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;
	}		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement LED/Flash mode"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement WB mode"	
void iCatch_set_wb_mode(config3a_wb_t mode)
{
        pr_info("%s +++ mode(%d)\n",__func__,mode);
       
        switch(mode)
        {
            case CAMERA_WB_AUTO:
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710A, 0x00);//auto
                break;
            case CAMERA_WB_INCANDESCENT:
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710A, 0x06);//Incandescent
                break;
            case CAMERA_WB_DAYLIGHT:
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710A, 0x01);//Daylight
                break;
            case CAMERA_WB_FLUORESCENT:
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710A, 0x05);//Fluorescent_H
                break;
            case CAMERA_WB_CLOUDY_DAYLIGHT:
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710A, 0x02);//Cloudy
                break;
            case CAMERA_WB_SHADE:
                sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x710A, 0x03);//Shade
                break;
            default:
                pr_info("%s mode(%d) is not support \n",__func__,mode);
                break;
        }
        pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement WB mode"	

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement EV mode"	
void iCatch_set_ev_mode(int16_t mode)
{
	uint16_t ev_val =0;
	pr_info("%s +++ mode(%d)\n",__func__,mode);

	if(mode>=-12 && mode <=12){	 // AP: -2EV=-12, 0EV=0, +2EV=12
		ev_val = 6-(mode/2);		        //ISP: -2EV=12, 0EV=6, +2EV=0
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7103, ev_val);
	}else{
		pr_info("%s -- mode(%d) is not support \n",__func__,mode);
		return;
	}
			
	pr_info("%s --- ev_val(0x%x)\n",__func__,ev_val);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement EV mode"	

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement ISO mode"
void iCatch_set_iso_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);	
	switch(mode)
	{
		case CAMERA_ISO_AUTO:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7110, 0x00);
			break;		
		case CAMERA_ISO_50:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7110, 0x01);
                     break;
		case CAMERA_ISO_100:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7110, 0x02);
			break;	
		case CAMERA_ISO_200:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7110, 0x03);
			break;	
		case CAMERA_ISO_400:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7110, 0x04);
			break;	
		case CAMERA_ISO_800:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7110, 0x05);
			break;	        
		case CAMERA_ISO_1600:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7110, 0x06);
			break;	            
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;            
       }		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement ISO mode"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement FLICKER mode"
void iCatch_set_flicker_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);	
	switch(mode)
	{
#if 0	//iCatch ISP doesn't support auto flicker
		case CAMERA_ANTIBANDING_AUTO:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7101, 0x00);
			break;	
#endif            
		case CAMERA_ANTIBANDING_50HZ:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7101, 0x01);
			break;	            
		case CAMERA_ANTIBANDING_60HZ:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7101, 0x02);
			break;	
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;            
       }		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement FLICKER mode"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement AECLOCK mode"
void iCatch_set_acelock_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);	
	switch(mode)
	{
		case 0:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x05);    //AE lock
			break;		
		case 1:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x03);    //AE unlock
			break;	            
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;            
       }		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement AECLOCK mode"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement AWBLOCK mode"
void iCatch_set_awblock_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);	
	switch(mode)
	{
		case 0:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x06);    //AWB unlock
			break;		
		case 1:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x04);    //AWB lock
			break;	            
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;            
       }		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement AWBLOCK mode"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement CAF mode"
void iCatch_set_caf_mode(bool continuous)
{
	pr_info("%s +++ as %d\n",__func__,continuous);

	if(continuous){
	    caf_mode = true;
	    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7105, 0x03); //CAF mode
	}
	else{
	    caf_mode = false;
	    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7105, 0x00); // Auto mode
	}

	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement CAF mode"

void  iCatch_set_ultrapixel(int16_t mode){
	pr_info("%s +++ mode(%d)\n",__func__,mode);
	if(mode ==1) {
		ultrapixel_mode = 1;
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7134, 0x01);
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x07);
	}else{
		ultrapixel_mode = 0;
		sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7134, 0x00);	
	}
	
}

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement SCENE mode"	
void iCatch_set_scene_mode(camera_bestshot_mode_type mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);
       
	switch(mode)
	{
		case CAMERA_BESTSHOT_AUTO:
              case CAMERA_BESTSHOT_OFF:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x00);         
			break;	
		case CAMERA_BESTSHOT_LANDSCAPE:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x06);
			break;
		case CAMERA_BESTSHOT_SNOW:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0B);
			break;
		case CAMERA_BESTSHOT_SUNSET:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0E);
			break;
		case CAMERA_BESTSHOT_NIGHT:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x07);
			break;			
		case CAMERA_BESTSHOT_PORTRAIT:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0A);
			break;	
		case CAMERA_BESTSHOT_BACKLIGHT:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x16);
			break;				
		case CAMERA_BESTSHOT_SPORTS:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0C);
			break;	
		case CAMERA_BESTSHOT_FLOWERS: //mapping vivid in asusAP
			//sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x13);
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x05);  // set effect vivid
			break;			
		case CAMERA_BESTSHOT_PARTY:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x09);
			break;	
		case CAMERA_BESTSHOT_BEACH:	
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x03);
			break;	            
		case CAMERA_BESTSHOT_ANTISHAKE:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0D);
			break;	            
		case CAMERA_BESTSHOT_CANDLELIGHT:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x04);
			break;	                
		case CAMERA_BESTSHOT_FIREWORKS:			
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x05);
			break;	                
		case CAMERA_BESTSHOT_NIGHT_PORTRAIT:		
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x08);
			break;	                
		case CAMERA_BESTSHOT_ACTION:
                     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x01);
			break;	             
		case CAMERA_BESTSHOT_THEATRE:               
		case CAMERA_BESTSHOT_AR:		
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;
            }	
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement SCENE mode"	

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement EFFECT mode"
void iCatch_set_effect_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);
       
	switch(mode)
	{
		case CAMERA_EFFECT_OFF:	
              case CAMERA_EFFECT_NORMAL:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x00);
			break;
		case CAMERA_EFFECT_MONO:	
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x04);
			break;
		case CAMERA_EFFECT_NEGATIVE:	
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x02);
			break;
              case CAMERA_EFFECT_SEPIA:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x03);
			break;                
              case CAMERA_EFFECT_AQUA:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x01);
			break;                           
              //ASUS_BSP +++
              case CAMERA_EFFECT_AURA:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x06);
			break;                  
              case CAMERA_EFFECT_VINTAGE:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x07);
			break;                     
              case CAMERA_EFFECT_VINTAGE2:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x08);
			break;                     
              case CAMERA_EFFECT_LOMO:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x09);
			break;                     
              case CAMERA_EFFECT_RED:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0A);
			break;                     
              case CAMERA_EFFECT_BLUE:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0B);
			break;                     
              case CAMERA_EFFECT_YELLOW:
			sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0C);
			break;                     
              //ASUS_BSP ---    
              case CAMERA_EFFECT_SOLARIZE:              
              case CAMERA_EFFECT_POSTERIZE:
              case CAMERA_EFFECT_WHITEBOARD:
              case CAMERA_EFFECT_BLACKBOARD:                
              case CAMERA_EFFECT_EMBOSS:
              case CAMERA_EFFECT_SKETCH:
              case CAMERA_EFFECT_NEON:                
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;
            }
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement EFFECT mode"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement AURA mode"
void iCatch_set_aura_value(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);

       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7119, mode);
       
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement AURA mode"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement general command"
void iCatch_set_general_cmd(struct general_cmd_cfg *cmd)
{
       u16 read_byte = 0, write_byte = 0;
	pr_info("%s +++ id(%d), value(%d)\n",__func__,cmd->cmd_id, cmd->cmd_value);
       
	switch(cmd->cmd_id)
	{
		//ASUS_BSP - Bryant : Implement Electric Image Stabilization(EIS) feature +++
		case GENERAL_CMD_EIS:
			g_is_eis_on = cmd->cmd_value;
			if (g_is_eis_on){
				//sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0D);
				iCatch_enable_autonight(false);
				pr_info("Turn On EIS \n");
			} else {
				iCatch_enable_autonight(true);
				pr_info("Turn Off EIS \n");
			}
		break;
		//ASUS_BSP - Bryant : Implement Electric Image Stabilization(EIS) feature ---
		case GENERAL_CMD_WDR:	
                    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x729b, &read_byte);
                    //pr_info("read_byte(0x%x)",read_byte);
                     if (cmd->cmd_value) {
                        write_byte = read_byte | 0x1;
                     } else {
                        write_byte = read_byte & (~0x1);
                     }
                    //pr_info("write_byte(0x%x)",write_byte);
                    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x711b, write_byte);
                    break;
		case GENERAL_CMD_HDR:	
			g_is_hdr_on = cmd->cmd_value;	//ASUS_BSP Stimber "Implement HDR feature"
                    break;  
        case GENERAL_CMD_NR:	
			g_is_nr_on = cmd->cmd_value;	//ASUS_BSP Stimber "Implement NR feature"
                    break;
		case GENERAL_CMD_GYRO:	
                    if(cmd->cmd_value == 1){    // GRYO detect Moving
                        //sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7126, 0x01);
                        setMaxExp(60);//set Preview Max. Exposure Time as 1/60
                        g_is_max_exp_on = true;
					}else{                                  // GYRO detect Stop
                        //sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7126, 0x00);
                        setMaxExp(0);//disable Max. Exposure Time function
                        g_is_max_exp_on = false;
					}
                    break;
		case GENERAL_CMD_TAE:	
                    if(cmd->cmd_value == 1){
                        g_TAEenable = 1;
                    }else{
                        g_TAEenable = 0;
                        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x714e, 0x00);
                    }
                    break;  
		case GENERAL_CMD_FIX_FPS:	//ASUS_BSP Stimber "Implement fixed fps for video"
			if(cmd->cmd_value){
				pr_info("Fixed fps\n");
				setFixFPS(30);  //fix 30 fps
                setMiniISO(0);  //reset min iso to default
                setCaptureVideoMode(2); //High speed video preview mode
			}else{
				pr_info("Dynamic fps\n");
				setFixFPS(0);   //reset fps fix
                setMiniISO(0);  //reset min iso to default
                setCaptureVideoMode(0); //default : capture preview mode
			}
            if(g_is_max_exp_on){
                setMaxExp(0);
                g_is_max_exp_on = false;
            }
			break;
		default:
			pr_info("%s cmd_id(%d) is not support \n",__func__,cmd->cmd_id);
			break;
       }
       
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement general command"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement get general command"
void iCatch_get_general_cmd(struct general_cmd_cfg *cmd)
{
       u16 read_byte = 0;
	pr_info("%s +++ id(%d), value(%d)\n",__func__,cmd->cmd_id, cmd->cmd_value);
       
	switch(cmd->cmd_id)
	{
		case GENERAL_CMD_GET_FLASH_STATUS:	
                    sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x72c3, &read_byte);
                    if(read_byte & 0x04){   // flash status: flash on
                        pr_info("isp flash on\n");
                        cmd->cmd_value = 1;
                    }else{
                        pr_info("isp flash off\n");
                        cmd->cmd_value = 0;                        
                    }
                    break;                  
		default:
			pr_info("%s cmd_id(%d) is not support \n",__func__,cmd->cmd_id);
			break;
       }
       
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement get general command"

void iCatch_checkAFMode(void)
{
    struct file *fp = NULL;

    fp = filp_open("/data/.tmp/fullsearch", O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    if ( !IS_ERR_OR_NULL(fp) ){
        g_afmode = 1;
        filp_close(fp, NULL);
    } else {
        g_afmode = 0;
    }
    pr_info("%s g_afmode(%d)\n",__func__,g_afmode);
}

//ASUS_BSP +++ Stimber "Implement EXIF info for camera with ISP"
void iCatch_enable_edge(bool enable, bool is_preExif)
{
       u16 read_byte = 0, write_byte = 0;

        sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x729b, &read_byte);

		 if(enable){
		     write_byte = read_byte | 0x02;
		 }else{
		     write_byte = read_byte & (~0x02);
		 }
		    
		 if(is_preExif){
		     write_byte = write_byte | 0x04;
		 }else{
		     write_byte = write_byte & (~0x04);
		 }
        
        sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x711b, write_byte);       
}

void iCatch_enable_autonight(bool enable)
{

	u16 read_byte = 0, write_byte = 0;

	sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x729b, &read_byte);

	if (enable) {
		write_byte = write_byte | 0x40;
        } else {
		write_byte = write_byte & (~0x40);
        }

	sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x711b, write_byte);

}


void iCatch_enable_exif(bool enable, bool is_preExif)
{
        if (enable) {
           //enable edge information
           iCatch_enable_edge(true, is_preExif);
           
           /* AE/AWB lock */
    	    sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x01);
           msleep(8);  //LiJen: wait for ISP ready      
        } else {
           iCatch_enable_edge(false, is_preExif);
        } 
}

bool iCatch_exif_flow_control(void)
{
    static int frame_count = 0; 
    int rc = false;

    if(++frame_count == FRAME_PERIOD){
        frame_count = 0;
        rc = true;
    }else{
        rc = false;
    }
    return rc;
}

void iCatch_get_exif(struct exif_cfg *exif)
{
	short iso = 0;
	u16 ISO_lowbyte = 0;
	u16 ISO_highbyte = 0;
	u16 et_numerator = 0;
	u16 et_denominator_byte1 = 1;
	u16 et_denominator_byte2 = 0;
	u16 et_denominator_byte3 = 0;
	u16 flash_mode;
       u32 edge = 0;

       u32 exif_bytes;
       unsigned char exif_data[14];
	   
#ifndef ASUS_SHIP_BUILD	   
       unsigned char info_3a_data[24];
#endif

       if(g_cur_res == MSM_SENSOR_RES_FULL ||
          g_cur_res == MSM_SENSOR_RES_10M ||
          g_cur_res == MSM_SENSOR_RES_FULL_SINGLE_CAPTURE ||
          g_cur_res == MSM_SENSOR_RES_FULL_BURST_CAPTURE ||
          g_cur_res == MSM_SENSOR_RES_10M_BURST_CAPTURE ||
          g_cur_res == MSM_SENSOR_RES_MODE_11||
          (g_mi1040_power == true && g_cur_res ==MSM_SENSOR_RES_FULL) ||
          (g_cur_res == MSM_SENSOR_RES_QTR && iCatch_exif_flow_control())){
            //pr_info("%s\n",__func__);
       }else{
            //pr_info("%s ignore\n",__func__);
            return;
       }
#if 0
       if(g_flash_mode == 0 || g_flash_mode == 1){
            exif_bytes = 9; //0x72b0~0x72b8
       }else{
            exif_bytes = 10; //0x72b0~0x72b9
       }
#else
       exif_bytes = 14; //0x72b0~0x72bd  //enable edge information
#endif

       //Get EXIF information from ISP
       sensor_read_reg_bytes(imx091_s_ctrl.sensor_i2c_client->client, 0x72b0, exif_data, exif_bytes);

	/* EXIF Exposure time */
       et_numerator = exif_data[0];
       et_denominator_byte1 = exif_data[1];
       et_denominator_byte2 = exif_data[2];
       et_denominator_byte3 = exif_data[3];

       /* EXIF ISO */
       ISO_lowbyte = exif_data[7];
       ISO_highbyte = exif_data[8];
       iso = (ISO_highbyte << 8) | ISO_lowbyte;

	/* EXIF Flash mode */
	if(g_flash_mode == 0){
		flash_mode = 0;
	}else if(g_flash_mode ==1){
		flash_mode = 1;
	}else{
		flash_mode = exif_data[9];
	}
    
       /* EXIF Edge */
       edge = (exif_data[13]<<24)|(exif_data[12]<<16)|(exif_data[11]<<8)|exif_data[10];
       //pr_info("(0x%x)(0x%x)(0x%x)(0x%x)\n",exif_data[10],exif_data[11],exif_data[12],exif_data[13]);
        
	exif->iso = iso;
	exif->exp_time_num = et_numerator;
	exif->exp_time_denom = (et_denominator_byte3 << 16)|(et_denominator_byte2 << 8)|et_denominator_byte1;
	exif->flash_mode = flash_mode;
    exif->edge = edge;

#ifndef ASUS_SHIP_BUILD
    exif_bytes = 24;
    sensor_read_reg_bytes(imx091_s_ctrl.sensor_i2c_client->client, 0x72d8, info_3a_data, exif_bytes); 
    memcpy(exif->info_3a, info_3a_data, 24);
#endif

    if(g_cur_res != MSM_SENSOR_RES_QTR){
	    pr_info("[EXIF] ISO(%d), ET(%d/%d), FLASH(%d), EDGE(%d)\n", exif->iso, exif->exp_time_num, exif->exp_time_denom, exif->flash_mode,edge);
    }
}
//ASUS_BSP --- Stimber "Implement EXIF info for camera with ISP"

//ASUS_BSP +++ LiJen "[A86][Camera][NA][Others]add switch file for Camera FW update"
char CAMERA_VERSION_INFO_PATH[] = "camera";
struct switch_dev camera_fw_switch_dev;

static ssize_t camera_fw_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%06x\n", version_num_in_isp);
}

static ssize_t camera_fw_switch_state(struct switch_dev *sdev, char *buf)
{        
	return sprintf(buf, "%s\n", "1");
}

void create_iCatch_switch_file(void)
{
       int ret = 0;
       
	camera_fw_switch_dev.name = CAMERA_VERSION_INFO_PATH;
	camera_fw_switch_dev.print_state = camera_fw_switch_state;
	camera_fw_switch_dev.print_name = camera_fw_switch_name;

       // registered switch device
       ret=switch_dev_register(&camera_fw_switch_dev);
	if (ret < 0){
		pr_err("%s result(%d)\n",__func__, ret);
	}
}
void remove_iCatch_switch_file(void)
{
	// unregistered switch device
	switch_dev_unregister(&camera_fw_switch_dev);
}
//ASUS_BSP --- LiJen "[A86][Camera][NA][Others]add switch file for Camera FW update"

void iCatch_init(void)
{
    //is_calibration = 0;
    single_image = 0;
    g_TAEenable = 0;
    g_is_hdr_on = 0;	//0: Disable, 1:Enable //ASUS_BSP stimber "Implement HDR feature"
    g_is_nr_on = 0;	//0: Disable, 1:Enable //ASUS_BSP stimber "Implement NR feature"
    g_is_eis_on = 0;      //0: Disable, 1:Enable //ASUS_BSP - Bryant "Implement Electric Image Stabilization(EIS) feature"
    is_calibration_table_set = true;    
    g_isAFCancel = false;
    g_isAFDone = true;
    g_LastFixFPS = 0;
    g_LastMaxExp = 0;
    g_LastMiniISO = 0;
    g_cur_res = MSM_SENSOR_INVALID_RES;
}

void iCatch_release_sensor(void)
{    
       //set lens to 120 step. step range is [0-1023]
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x715c, 0x78);
       //triger lens to move
       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x715d, 0x00);
       //wait lens to move
       msleep(200);
}
