/*
        TI bq27520-G3 Fuel Gauge Implementation

*/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>

//#include "axc_TI_HWgauge_A80.h"//ASUS BSP Eason_Chang TIgauge
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#include <linux/microp_api.h>
#endif /*CONFIG_EEPROM_NUVOTON */
#include <linux/platform_device.h>//ASUS_BSP Eason_Chang 1120 porting
#include <linux/i2c.h>//ASUS BSP Eason_Chang TIgauge
#include <linux/module.h>//ASUS BSP Eason_Chang A68101032 porting

//ASUS BSP : Eason Chang check Df_version ++++++++
static struct delayed_work CheckDfVersionWorker;
static bool DffsDown = false;
#define FAKE_TEMP (25)
//ASUS BSP : Eason Chang check Df_version --------

//Eason: TI HW GAUGE IRQ +++
#include <linux/mfd/pm8xxx/pm8921.h>
#define PM8921_GPIO_BASE                NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
static int PMICgpio14;
//Eason: TI HW GAUGE IRQ ---
//ASUS BSP Eason_Chang get Temp from TIgauge+++
static int g_gauge_temp = 25;
#define TI_GAUGE_Error (-1)
//ASUS BSP Eason_Chang get Temp from TIgauge---
//check df version do Cap remapping+++
int g_gauge_df_version = 0;
//check df version do Cap remapping---

//ASUS BSP Eason_Chang TIgauge+++
extern void msleep(unsigned int msecs);
static u32 g_TIgauge_slave_addr=0;
static int g_TIgauge_reg_value=0;
static int g_TIgauge_reg_address=0;
static struct TIgauge_info *g_TIgauge_info=NULL;
struct TIgauge_platform_data{
        int intr_gpio;
};

struct TIgauge_info {
       struct i2c_client *i2c_client;
       struct TIgauge_platform_data *pdata;
};
//ASUS BSP Eason_Chang TIgauge---

struct TIgauge_command{
    char *name;;
    u8 addr;
    u8 len;
    enum readwrite{
		E_READ=0,
		E_WRITE=1,
		E_READWRITE=2,
		E_NOUSE=3,

    }rw;

};

enum TIgauge_CMD_ID {
		TIgauge_SOC=0,
		TIgauge_VOLT,
		TIgauge_AI,
		TIgauge_TEMP,
		TIgauge_FCC,
		TIgauge_CC,
		TIgauge_RM,
		TIgauge_CNTL,
};

struct TIgauge_command TIgauge_CMD_Table[]={
		{"SOC",			0x2C,  1,		E_READ},			//StateOfCharge   
		{"VOLT",			0x08,  2,   	E_READ},			//Votage
		{"AI",			0x14,  2,   	E_READ},			//AverageCurrent
		{"TEMP",			0x06,  2,		E_READ},			//Temprature
		{"FCC",			0x12,  2,		E_READ},		  	//FullChargeCapacity
		{"CC",			0x2A,  2,		E_READ},			//CycleCount
		{"RM",			0x10,  2,		E_READ},			//RemainingCapacity
		{"CNTL",			0x00,  2,		E_READWRITE},	//Control
    	  
};



static int TIgauge_i2c_read(u8 addr, int len, void *data)
{
        int i=0;
        int retries=6;
        int status=0;

	struct i2c_msg msg[] = {
		{
			.addr = g_TIgauge_slave_addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = g_TIgauge_slave_addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

    pr_debug("[BAT]TIgauge_i2c_read+++\n");
    
        if(g_TIgauge_info){
            do{    
                pr_debug("[BAT]TIgauge smb346_i2c_transfer\n");
        		status = i2c_transfer(g_TIgauge_info->i2c_client->adapter,
        			msg, ARRAY_SIZE(msg));
                
        		if ((status < 0) && (i < retries)){
        			    msleep(5);
                      
                                printk("%s retry %d\r\n", __FUNCTION__, i);                                
                                i++;
                     }
        	    } while ((status < 0) && (i < retries));
        }
        if(status < 0){
            printk("TIgauge: i2c read error %d \n", status);
        }
    pr_debug("[BAT]TIgauge_i2c_read---\n");
    

        return status;
        
}


static int TIgauge_read_reg(int cmd, void *data)
{
    int status=0;
    if(cmd>=0 && cmd < ARRAY_SIZE(TIgauge_CMD_Table)){
        if(E_WRITE==TIgauge_CMD_Table[cmd].rw || E_NOUSE==TIgauge_CMD_Table[cmd].rw){ // skip read for these command
            printk("TIgauge: read ignore cmd\r\n");      
        }
        else{   
            pr_debug("[BAT]TIgauge_read_reg\n");
            status=TIgauge_i2c_read(TIgauge_CMD_Table[cmd].addr, TIgauge_CMD_Table[cmd].len, data);
        }    
    }
    else
        printk("TIgauge: unknown read cmd\r\n");
            
    return status;
}

static void TIgauge_proc_read(void)
{    
       int status;
       int16_t reg_value=0;//for 2 byte I2c date should use int16_t instead int, let negative value can show correctly. ex:current
       printk("%s \r\n", __FUNCTION__);
       status=TIgauge_read_reg(g_TIgauge_reg_address,&reg_value);
       g_TIgauge_reg_value = reg_value;

       if(status > 0 && reg_value >= 0){
            printk("[BAT][Chg][TIgauge] found! TIgauge=%d\r\n",reg_value);
       }
}
//ASUS BSP Eason_Chang TIgauge +++

static int TIgauge_i2c_write(u8 addr, int len, void *data)
{
    int i=0;
	int status=0;
	u8 buf[len + 1];
	struct i2c_msg msg[] = {
		{
			.addr = g_TIgauge_slave_addr,
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		},
	};
	int retries = 6;

	buf[0] = addr;
	memcpy(buf + 1, data, len);

	do {
		status = i2c_transfer(g_TIgauge_info->i2c_client->adapter,
			msg, ARRAY_SIZE(msg));
        	if ((status < 0) && (i < retries)){
                    msleep(5);
           
                    printk("%s retry %d\r\n", __FUNCTION__, i);
                    i++;
              }
       } while ((status < 0) && (i < retries));

	if (status < 0) {
                    printk("TIgauge: i2c write error %d \n", status);
	}

	return status;
}

static int TIgauge_write_reg(int cmd, void *data)
{
    int status=0;
    if(cmd>=0 && cmd < ARRAY_SIZE(TIgauge_CMD_Table)){
        if(E_READ==TIgauge_CMD_Table[cmd].rw  || E_NOUSE==TIgauge_CMD_Table[cmd].rw){ // skip read for these command               
        }
        else
            status=TIgauge_i2c_write(TIgauge_CMD_Table[cmd].addr, TIgauge_CMD_Table[cmd].len, data);
    }
    else
        printk("TIgauge: unknown write cmd\r\n");
            
    return status;
}

static void TIgauge_proc_write(void)
{    
       int status;
       uint16_t i2cdata[32]={0};

       i2cdata[0] = g_TIgauge_reg_value;
       printk("%s:%d \r\n", __FUNCTION__,g_TIgauge_reg_value);
       status=TIgauge_write_reg(g_TIgauge_reg_address,i2cdata);

       if(status > 0 ){
            printk("[BAT][Chg][smb346] proc write\r\n");
       }
}

static int TIgauge_read_table(int tableNumber)
{    
       int status;
       int16_t reg_value=0;

       status=TIgauge_read_reg(tableNumber,&reg_value);

       if(status > 0)
   	{
            pr_debug("[BAT][TIgauge][table] found!=%d\r\n",reg_value);
       }
       return reg_value;
}
//ASUS BSP Eason_Chang TIgauge  ---

//ASUS BSP : Eason Chang check Df_version ++++++++
static int TIgauge_DF_VERSION(void);
static void Check_DF_Version(struct work_struct *work)
{
	g_gauge_df_version = TIgauge_DF_VERSION();

	if( (10 >= g_gauge_df_version)&&( 1<= g_gauge_df_version) )
	{
		DffsDown = true;
		printk("[BAT][TIgauge]DffsDown:true\n");
	}else if(0==g_gauge_df_version)
	{
		DffsDown = false;
		printk("[BAT][TIgauge]DffsDown:false\n");
	}else
	{
		schedule_delayed_work(&CheckDfVersionWorker, 20*HZ);
		printk("[BAT][TIgauge]Df version unknown:\n");
	}	
}
//ASUS BSP : Eason Chang check Df_version --------
//Eason report capacity to TIgauge when gauge read error, don't update capacity+++
extern int ReportCapToTIGauge(void);
//Eason report capacity to TIgauge when gauge read error, don't update capacity---
//ASUS BSP Eason_Chang get Cap from TIgauge+++
int get_Cap_from_TIgauge(void)
{
	int TIsoc;

	TIsoc=TIgauge_read_table(TIgauge_SOC);

	//Eason: when gauge read error, don't update capacity+++
	if(TI_GAUGE_Error==TIsoc)
	{
		printk("[BAT][TIgauge][Cap]:Error\n");
		return ReportCapToTIGauge();
	}else{
		printk("[BAT][TIgauge][Cap]:%d\n",TIsoc);
		return TIsoc;
	}
	//Eason: when gauge read error, don't update capacity---
}
//ASUS BSP Eason_Chang get Cap from TIgauge---
//ASUS BSP Eason_Chang get Temp from TIgauge+++
int get_Temp_from_TIgauge(void)
{
	int TItemp;

	TItemp=TIgauge_read_table(TIgauge_TEMP);
	
	printk("[BAT][TIgauge][Temp]:%d\n",TItemp);

	if(TI_GAUGE_Error==TItemp)
		return g_gauge_temp;
	else
	{
		if(true==DffsDown)
		{
			return ((TItemp/10)-273);
		}else{
			return FAKE_TEMP;
		}
	}
}
//ASUS BSP Eason_Chang get Temp from TIgauge---
 //ASUS BSP Eason_Chang dump TIgauge info +++
int get_Cntl_from_TIgauge(int TIcntl)
{
	int TIvalue;

	TIvalue=TIgauge_read_table(TIcntl);
	
	printk("[BAT][TIgauge][CNTL:%d]:%d\n",TIcntl,TIvalue);

	if(TI_GAUGE_Error==TIvalue)
		return TI_GAUGE_Error;
	else
		return TIvalue;
}
 //ASUS BSP Eason_Chang dump TIgauge info ---
 //Eason:A80 slowly drop+++
int get_Curr_from_TIgauge(void)
{
	int ret;
	ret = (-1)*get_Cntl_from_TIgauge(TIgauge_AI);
	return ret;
}
//Eason:A80 slowly drop---
//ASUS BSP Eason_Chang get Temp from TIgauge+++
static ssize_t TIgaugeTemp_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	g_gauge_temp = get_Temp_from_TIgauge();
	return sprintf(page, "%d\n", g_gauge_temp);
}
static ssize_t TIgaugeTemp_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	
	val = (int)simple_strtol(messages, NULL, 10);

	g_gauge_temp = val;
     
        printk("[BAT][Bal]mode:%d\n",val);

	return len;
}

void static create_TIgaugeTemp_proc_file(void)
{
	struct proc_dir_entry *TIgaugeTemp_proc_file = create_proc_entry("driver/BatTemp", 0644, NULL);

	if (TIgaugeTemp_proc_file) {
		TIgaugeTemp_proc_file->read_proc = TIgaugeTemp_read_proc;
		TIgaugeTemp_proc_file->write_proc = TIgaugeTemp_write_proc;
	}
	else {
		printk("[BAT]TIgaugeTemp proc file create failed!\n");
	}

	return;
}
//ASUS BSP Eason_Chang get Temp from TIgauge---
//ASUS BSP Eason_Chang TIgauge +++
static ssize_t TIgauge_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	TIgauge_proc_read();
	return sprintf(page, "%d\n", g_TIgauge_reg_value);
}
static ssize_t TIgauge_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	
	val = (int)simple_strtol(messages, NULL, 10);


	g_TIgauge_reg_value = val;

      TIgauge_proc_write();
    
    printk("[BAT][smb346]mode:%d\n",g_TIgauge_reg_value);
	
	return len;
}

void static create_TIgauge_proc_file(void)
{
	struct proc_dir_entry *TIgauge_proc_file = create_proc_entry("driver/TIgauge", 0644, NULL);

	if (TIgauge_proc_file) {
		TIgauge_proc_file->read_proc = TIgauge_read_proc;
		TIgauge_proc_file->write_proc = TIgauge_write_proc;
	}
    else {
		printk("[BAT][TIgauge]proc file create failed!\n");
    }

	return;
}

static ssize_t TIgaugeAddress_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	return sprintf(page, "%d\n", g_TIgauge_reg_address);
}
static ssize_t TIgaugeAddress_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	
	val = (int)simple_strtol(messages, NULL, 10);


	g_TIgauge_reg_address = val;  
    
    printk("[BAT][TIgauge]TIgaugeAddress:%d\n",val);
	
	return len;
}

void static create_TIgaugeAddress_proc_file(void)
{
	struct proc_dir_entry *TIgaugeAddress_proc_file = create_proc_entry("driver/TIgaugeAddr", 0644, NULL);

	if (TIgaugeAddress_proc_file) {
		TIgaugeAddress_proc_file->read_proc = TIgaugeAddress_read_proc;
		TIgaugeAddress_proc_file->write_proc = TIgaugeAddress_write_proc;
	}
    else {
		printk("[BAT][TIgauge]Addr proc file create failed!\n");
    }

	return;
}

//ASUS BSP Eason_Chang TIgauge ---
//ASUS BSP Eason_Chang dump TIgauge info +++
static int TIgauge_CHEMID(void)
{
       int status;
       uint8_t i2cdata[32]={0};

       printk("%s++++\n", __FUNCTION__);	
       i2cdata[0] = 0x08;
       i2cdata[1] = 0x00;

       status=TIgauge_i2c_write(0x00, 2, i2cdata);//Before read CHEM_ID, need echo 0x0008 to address 0x01/0x00 first 

       if(status > 0 ){
            printk("[BAT][TIgauge] TIgauge_CHEMID\r\n");
       }

	printk("%s----\n", __FUNCTION__);
	return get_Cntl_from_TIgauge(TIgauge_CNTL);
}

static int TIgauge_DF_VERSION(void)
{
	int status;
	uint8_t i2cdata[32]={0};

	printk("%s++++\n", __FUNCTION__);      
	i2cdata[0] = 0x1F;
	i2cdata[1] = 0x00;

	status=TIgauge_i2c_write(0x00, 2, i2cdata);//Before read DF_VERSION, need echo 0x001F to address 0x01/0x00 first

	if(status > 0 ){
		printk("[BAT][TIgauge] TIgauge_DF_VERSION\r\n");
	}

	printk("%s----\n", __FUNCTION__);      
	return get_Cntl_from_TIgauge(TIgauge_CNTL);
}

static ssize_t TIgaugeDump_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	int chemId;
	int df_version;
	
	chemId = TIgauge_CHEMID();
	df_version = TIgauge_DF_VERSION();

	return sprintf(page, "FCC(mAh): %d\n"
						"RM(mAh): %d\n"
						"SOC: %d\n"
						"VOLT(mV): %d\n"
						"AI(mA): %d\n"
						"TEMP(degC): %d\n"
						"CC: %d\n"
						"CHEM_ID: 0x%04X\n"
						"DF_VERSION: 0x%04X\n"
						,get_Cntl_from_TIgauge(TIgauge_FCC)
						,get_Cntl_from_TIgauge(TIgauge_RM)
						,get_Cap_from_TIgauge()
						,get_Cntl_from_TIgauge(TIgauge_VOLT)
						,get_Cntl_from_TIgauge(TIgauge_AI)
						,get_Temp_from_TIgauge()
						,get_Cntl_from_TIgauge(TIgauge_CC)
						,chemId
						,df_version);
	
}

void static create_TIgaugeDump_proc_file(void)
{
	struct proc_dir_entry *TIgaugeDump_proc_file = create_proc_entry("driver/bq27520_test_info_dump", 0444, NULL);

	if (TIgaugeDump_proc_file) {
		TIgaugeDump_proc_file->read_proc = TIgaugeDump_read_proc;
	}
	else {
		printk("[BAT]TIgaugeDump proc file create failed!\n");
	}

	return;
}
 //ASUS BSP Eason_Chang dump TIgauge info ---
//Eason: TI HW GAUGE IRQ +++
#define HW_GAUGE_IRQ_GPIO	14
static int hw_gauge_irq; 

static irqreturn_t  asus_hw_gauge_irq_handler(int irq, void *dev_id)
{
	printk("[BAT]%s() triggered \r\n", __FUNCTION__);

	//schedule_delayed_work(&asus_bat->phone_b.bat_low_work, 0); 
	printk("[BAT][HWgauge]irq:%d\n",gpio_get_value(PMICgpio14));

	return IRQ_HANDLED;
}

static int config_hwGauge_irq_gpio_pin(void)
{
	int ret;

	struct pm_gpio PMICgpio14_param = {
		.direction = PM_GPIO_DIR_IN ,
     	   	.output_buffer = PM_GPIO_OUT_BUF_CMOS,
     	   	.output_value = 0,
       	.pull = PM_GPIO_PULL_UP_30,
   	 	.vin_sel	= PM_GPIO_VIN_S4,
		.out_strength   = PM_GPIO_STRENGTH_MED,
		.function       = PM_GPIO_FUNC_NORMAL,
	};


        PMICgpio14 = PM8921_GPIO_PM_TO_SYS(HW_GAUGE_IRQ_GPIO);
 
	ret = gpio_request(PMICgpio14, "TI_HWGAUGE_IRQ");
	if (ret) {
		pr_err("%s: unable to request pmic's gpio 14 (BAT_HWGAUGE)\n",__func__);
		return ret;
	}

	ret = pm8xxx_gpio_config(PMICgpio14, &PMICgpio14_param);
	if (ret)
		pr_err("%s: Failed to configure gpio\n", __func__);

	//enable TI HW GAUGE IRQ +++
	hw_gauge_irq = gpio_to_irq(PMICgpio14);
	
	enable_irq_wake(hw_gauge_irq);

	ret = request_irq(hw_gauge_irq , asus_hw_gauge_irq_handler, \
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "TI_HWGAUGE_IRQ", NULL);

    	if (ret) {
		printk("request_hw_gauge_irq  failed, ret = %d\n", ret);
    	} 
	//enable TI HW GAUGE IRQ ---
	
	return 0;
}
//Eason: TI HW GAUGE IRQ ---

static int TIgauge_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{

    struct TIgauge_info *info;
    g_TIgauge_info=info = kzalloc(sizeof(struct TIgauge_info), GFP_KERNEL);
    g_TIgauge_slave_addr=client->addr;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);

    printk("%s+++\n",__FUNCTION__);
    //ASUS BSP Eason_Chang TIgauge +++
    create_TIgauge_proc_file();
    create_TIgaugeAddress_proc_file(); 
    //ASUS BSP Eason_Chang TIgauge ---
    //ASUS BSP Eason_Chang get Temp from TIgauge+++
    create_TIgaugeTemp_proc_file();
    //ASUS BSP Eason_Chang get Temp from TIgauge---
    //ASUS BSP Eason_Chang dump TIgauge info +++
    create_TIgaugeDump_proc_file();
    //ASUS BSP Eason_Chang dump TIgauge info ---
    //Eason: TI HW GAUGE IRQ +++
    config_hwGauge_irq_gpio_pin();
    //Eason: TI HW GAUGE IRQ ---
    //ASUS BSP : Eason Chang check Df_version ++++++++
    INIT_DELAYED_WORK(&CheckDfVersionWorker,Check_DF_Version);
    schedule_delayed_work(&CheckDfVersionWorker, 0*HZ);
    //ASUS BSP : Eason Chang check Df_version --------
    printk("%s---\n",__FUNCTION__);
    return 0;
}

static int TIgauge_remove(struct i2c_client *client)
{
	return 0;
}

//ASUS BSP Eason_Chang +++
const struct i2c_device_id TIgauge_id[] = {
    {"TI_bq27520_Gauge", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, TIgauge_id);



static struct i2c_driver TIgauge_driver = {
    .driver = {
        .name = "TI_bq27520_Gauge",
        .owner  = THIS_MODULE,
     },
    .probe = TIgauge_probe,
    .remove = TIgauge_remove,
    //.suspend = smb346_suspend,
    //.resume = smb346_resume,
    .id_table = TIgauge_id,
};    
//ASUS BSP Eason_Chang ---

static int __init TIgauge_init(void)
{
    //ASUS BSP Eason_Chang +++
    printk("[BAT][TIgauge]init\n");
    return i2c_add_driver(&TIgauge_driver);
    //ASUS BSP Eason_Chang ---
}

static void __exit TIgauge_exit(void)
{
    //ASUS BSP Eason_Chang +++
    printk("[BAT][TIgauge]exit\n");
    i2c_del_driver(&TIgauge_driver);
    //ASUS BSP Eason_Chang ---

}
// be be later after usb...
module_init(TIgauge_init);
module_exit(TIgauge_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASUS TIgauge virtual driver");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Eason Chang <eason1_chang@asus.com>");

//ASUS_BSP --- Josh_Liao "add asus battery driver"

