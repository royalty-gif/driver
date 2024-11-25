#include "linux/export.h"
#include "linux/types.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>
#include <linux/delay.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>

#define LCD_IOC_INIT 		123
#define LCD_IOC_SET_POS 	124
#define LCD_IOC_CLEAR       125

#define X_MAX_PIXEL	        128
#define Y_MAX_PIXEL	        128

//为0 表示命令，为1表示数据
#define LCD_CMD 	0
#define LCD_DATA 	1

static struct spi_device* spidev_lcd;
static int major;
static struct gpio_desc* dc_gpio, *rst_gpio;
static struct class *spidev_class;

static void pin_init(void)
{
	gpiod_direction_output(dc_gpio, 1);
	gpiod_direction_output(rst_gpio, 1);
}

static void lcd_set_dc_pin(int val)
{
	gpiod_set_value(dc_gpio, val);
}

static void lcd_set_reset_pin(int val)
{
	gpiod_set_value(rst_gpio, val);
}

static void spi_write_datas(const unsigned char *buf, int len)
{
	spi_write(spidev_lcd, buf, len);
}


/**********************************************************************
	 * 函数名称： lcd_write_cmd
	 * 功能描述： lcd向特定地址写入数据或者命令
	 * 输入参数：@uc_data :要写入的数据
	 			@uc_cmd:为1则表示写入数据，为0表示写入命令
	 * 输出参数：无
	 * 返 回 值： 无
	 * 修改日期 	   版本号	 修改人		  修改内容
	 * -----------------------------------------------
	 * 2020/03/04		 V1.0	  芯晓		  创建
 ***********************************************************************/
static void lcd_write_cmd_data(unsigned char uc_data,unsigned char uc_cmd)
{
	if(uc_cmd==0)
	{
		lcd_set_dc_pin(0); //拉低，表示写入命令
	}
	else
	{
		lcd_set_dc_pin(1); //拉高，表示写入数据
	}

	spi_write_datas(&uc_data, 1);//写入
}

static void lcd_reset(void)
{
	lcd_set_reset_pin(0);
	msleep(100);
	lcd_set_reset_pin(1);
	msleep(50);
}

/**********************************************************************
	 * 函数名称： lcd_init
	 * 功能描述： lcd_init的初始化，包括SPI控制器得初始化
	 * 输入参数：无
	 * 输出参数： 初始化的结果
	 * 返 回 值： 成功则返回0，否则返回-1
	 * 修改日期 	   版本号	 修改人		  修改内容
	 * -----------------------------------------------
	 * 2020/03/15		 V1.0	  芯晓		  创建
 ***********************************************************************/
static int lcd_init(void)
{
	lcd_reset(); //Reset before LCD Init.

	//LCD Init For 1.44Inch LCD Panel with ST7735R.
	lcd_write_cmd_data(0x11, LCD_CMD);//Sleep exit 
	msleep(120);
		
	//ST7735R Frame Rate
	lcd_write_cmd_data(0xB1, LCD_CMD); 
	lcd_write_cmd_data(0x01, LCD_DATA); 
	lcd_write_cmd_data(0x2C, LCD_DATA); 
	lcd_write_cmd_data(0x2D, LCD_DATA); 

	lcd_write_cmd_data(0xB2, LCD_CMD); 
	lcd_write_cmd_data(0x01, LCD_DATA); 
	lcd_write_cmd_data(0x2C, LCD_DATA); 
	lcd_write_cmd_data(0x2D, LCD_DATA); 

	lcd_write_cmd_data(0xB3, LCD_CMD); 
	lcd_write_cmd_data(0x01, LCD_DATA); 
	lcd_write_cmd_data(0x2C, LCD_DATA); 
	lcd_write_cmd_data(0x2D, LCD_DATA); 
	lcd_write_cmd_data(0x01, LCD_DATA); 
	lcd_write_cmd_data(0x2C, LCD_DATA); 
	lcd_write_cmd_data(0x2D, LCD_DATA); 
	
	lcd_write_cmd_data(0xB4, LCD_CMD); //Column inversion 
	lcd_write_cmd_data(0x07, LCD_DATA); 
	
	//ST7735R Power Sequence
	lcd_write_cmd_data(0xC0, LCD_CMD); 
	lcd_write_cmd_data(0xA2, LCD_DATA); 
	lcd_write_cmd_data(0x02, LCD_DATA); 
	lcd_write_cmd_data(0x84, LCD_DATA); 
	lcd_write_cmd_data(0xC1, LCD_CMD); 
	lcd_write_cmd_data(0xC5, LCD_DATA); 

	lcd_write_cmd_data(0xC2, LCD_CMD); 
	lcd_write_cmd_data(0x0A, LCD_DATA); 
	lcd_write_cmd_data(0x00, LCD_DATA); 

	lcd_write_cmd_data(0xC3, LCD_CMD); 
	lcd_write_cmd_data(0x8A, LCD_DATA); 
	lcd_write_cmd_data(0x2A, LCD_DATA); 
	lcd_write_cmd_data(0xC4, LCD_CMD); 
	lcd_write_cmd_data(0x8A, LCD_DATA); 
	lcd_write_cmd_data(0xEE, LCD_DATA); 
	
	lcd_write_cmd_data(0xC5, LCD_CMD); //VCOM 
	lcd_write_cmd_data(0x0E, LCD_DATA); 
	
	lcd_write_cmd_data(0x36, LCD_CMD); //MX, MY, RGB mode 
	lcd_write_cmd_data(0xC8, LCD_DATA); 
	
	//ST7735R Gamma Sequence
	lcd_write_cmd_data(0xe0, LCD_CMD); 
	lcd_write_cmd_data(0x0f, LCD_DATA); 
	lcd_write_cmd_data(0x1a, LCD_DATA); 
	lcd_write_cmd_data(0x0f, LCD_DATA); 
	lcd_write_cmd_data(0x18, LCD_DATA); 
	lcd_write_cmd_data(0x2f, LCD_DATA); 
	lcd_write_cmd_data(0x28, LCD_DATA); 
	lcd_write_cmd_data(0x20, LCD_DATA); 
	lcd_write_cmd_data(0x22, LCD_DATA); 
	lcd_write_cmd_data(0x1f, LCD_DATA); 
	lcd_write_cmd_data(0x1b, LCD_DATA); 
	lcd_write_cmd_data(0x23, LCD_DATA); 
	lcd_write_cmd_data(0x37, LCD_DATA); 
	lcd_write_cmd_data(0x00, LCD_DATA); 	
	lcd_write_cmd_data(0x07, LCD_DATA); 
	lcd_write_cmd_data(0x02, LCD_DATA); 
	lcd_write_cmd_data(0x10, LCD_DATA); 

	lcd_write_cmd_data(0xe1, LCD_CMD); 
	lcd_write_cmd_data(0x0f, LCD_DATA); 
	lcd_write_cmd_data(0x1b, LCD_DATA); 
	lcd_write_cmd_data(0x0f, LCD_DATA); 
	lcd_write_cmd_data(0x17, LCD_DATA); 
	lcd_write_cmd_data(0x33, LCD_DATA); 
	lcd_write_cmd_data(0x2c, LCD_DATA); 
	lcd_write_cmd_data(0x29, LCD_DATA); 
	lcd_write_cmd_data(0x2e, LCD_DATA); 
	lcd_write_cmd_data(0x30, LCD_DATA); 
	lcd_write_cmd_data(0x30, LCD_DATA); 
	lcd_write_cmd_data(0x39, LCD_DATA); 
	lcd_write_cmd_data(0x3f, LCD_DATA); 
	lcd_write_cmd_data(0x00, LCD_DATA); 
	lcd_write_cmd_data(0x07, LCD_DATA); 
	lcd_write_cmd_data(0x03, LCD_DATA); 
	lcd_write_cmd_data(0x10, LCD_DATA); 
	
	lcd_write_cmd_data(0x2a, LCD_CMD);
	lcd_write_cmd_data(0x00, LCD_DATA); 
	lcd_write_cmd_data(0x00, LCD_DATA); 
	lcd_write_cmd_data(0x00, LCD_DATA); 
	lcd_write_cmd_data(0x7f, LCD_DATA); 

	lcd_write_cmd_data(0x2b, LCD_CMD);
	lcd_write_cmd_data(0x00, LCD_DATA); 
	lcd_write_cmd_data(0x00, LCD_DATA); 
	lcd_write_cmd_data(0x00, LCD_DATA); 
	lcd_write_cmd_data(0x9f, LCD_DATA); 
	
	lcd_write_cmd_data(0xF0, LCD_CMD); //Enable test command  
	lcd_write_cmd_data(0x01, LCD_DATA); 
	lcd_write_cmd_data(0xF6, LCD_CMD); //Disable ram power save mode 
	lcd_write_cmd_data(0x00, LCD_DATA); 
	
	lcd_write_cmd_data(0x3A, LCD_CMD); //65k mode 
	lcd_write_cmd_data(0x05, LCD_DATA); 
	
	lcd_write_cmd_data(0x29, LCD_CMD);//Display on	 

	return 0;
}

// 设置lcd显示区域，在此区域写点数据自动换行
static void lcd_set_region(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end)
{ 	
	lcd_write_cmd_data(0x2a, LCD_CMD);
	lcd_write_cmd_data(0x00, LCD_DATA);
	lcd_write_cmd_data(x_start+2, LCD_DATA);
	lcd_write_cmd_data(0x00, LCD_DATA);
	lcd_write_cmd_data(x_end+2, LCD_DATA);

	lcd_write_cmd_data(0x2b, LCD_CMD);
	lcd_write_cmd_data(0x00, LCD_DATA);
	lcd_write_cmd_data(y_start+3, LCD_DATA);
	lcd_write_cmd_data(0x00, LCD_DATA);
	lcd_write_cmd_data(y_end+3, LCD_DATA);
	
	lcd_write_cmd_data(0x2c, LCD_CMD);
}   	      	   			 

// 设置显示起始点
void lcd_setxy(uint16_t x,uint16_t y)
{
  	lcd_set_region(x,y,x,y);
}

static long
spidev_lcd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int x, y;
	
	/* 根据cmd操作硬件 */
	switch (cmd)
	{
		case LCD_IOC_INIT: /* init */
		{
			pin_init();
			lcd_init();
			break;
		}

		case LCD_IOC_SET_POS: /* set pos */
		{
			x = arg & 0xff;
			y = (arg >> 8) & 0xff;
			lcd_setxy(x, y);
			break;
		}

		case LCD_IOC_CLEAR: /* clear*/
		{
			unsigned int i,m;
			lcd_set_region(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
			lcd_write_cmd_data(0x2C, LCD_CMD);
			for(i=0;i<X_MAX_PIXEL;i++) {
				for(m=0;m<Y_MAX_PIXEL;m++)
				{	
					unsigned char buf[2] = {arg>>8, arg};
					lcd_set_dc_pin(1);
					spi_write_datas(buf, 2);
				}  
			}
		}

	}

	return 0;
}

static ssize_t 
spidev_lcd_write(struct file* filp, const char __user* buf, size_t count, loff_t* f_pos)
{
    int err;

    char* pbuf = kmalloc(count, GFP_KERNEL);
    err = copy_from_user(pbuf, buf, count);

    lcd_set_dc_pin(1);

    spi_write_datas(pbuf, count);
    kfree(pbuf);

    return count;
}

static const struct file_operations spidev_lcd_fops = {
	.owner =  THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =   spidev_lcd_write,
	.unlocked_ioctl = spidev_lcd_ioctl,
};

static int spidev_lcd_probe(struct spi_device* spi) 
{
    spidev_lcd = spi;

	pr_emerg("spidev lcd probe!\n");
    major = register_chrdev(0, "spidev_lcd", &spidev_lcd_fops);
    spidev_class = class_create(THIS_MODULE, "spidev_lcd_class");
    device_create(spidev_class, NULL, MKDEV(major, 0), NULL, "spidev_lcd");

    dc_gpio = gpiod_get(&spi->dev, "dc", 0);
	rst_gpio = gpiod_get(&spi->dev, "rst", 0);

    return 0;
}

static int spidev_lcd_remove(struct spi_device* spi)
{
    gpiod_put(dc_gpio);
    gpiod_put(rst_gpio);

    device_destroy(spidev_class, MKDEV(major, 0));
    class_destroy(spidev_class);
    unregister_chrdev(major, "spidev_lcd");

    return 0;
}

static const struct of_device_id spi_lcd_dt_ids[] = {
	{ .compatible = "spi_lcd_drv" },
	{},
};

static struct spi_driver spidev_lcd_driver = {
	.driver = {
		.name =		"spi_lcd_drv",
		.of_match_table = of_match_ptr(spi_lcd_dt_ids),
	},
	.probe =	spidev_lcd_probe,
	.remove =	spidev_lcd_remove,
};

static int __init spidev_lcd_init(void)
{
	int status;

	pr_emerg("init spidev lcd!\n");
	status = spi_register_driver(&spidev_lcd_driver);
	if (status < 0) 
    {
        pr_emerg("spidev register driver failed!\n");
        return -EIO;
    }

	pr_emerg("status: %d\n", status);
	return status;
}
module_init(spidev_lcd_init);

static void __exit spidev_lcd_exit(void)
{
	spi_unregister_driver(&spidev_lcd_driver);
}
module_exit(spidev_lcd_exit);

MODULE_LICENSE("GPL");