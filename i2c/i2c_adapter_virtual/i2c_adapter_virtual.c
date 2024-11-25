#include "linux/device.h"
#include "linux/export.h"
#include "linux/gfp.h"
#include "linux/kernel.h"
#include "linux/printk.h"
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct i2c_adapter_private_data {
    struct i2c_adapter adapter;
};

static unsigned char eeprom_buffer[512];
static int eeprom_cur_addr = 0;

static void eeprom_emulate_xfer(struct i2c_adapter* adap, struct i2c_msg* msg)
{
	int i;
	if (msg->flags & I2C_M_RD)
	{
		for (i = 0; i < msg->len; i++)
		{
			msg->buf[i] = eeprom_buffer[eeprom_cur_addr++];
			if (eeprom_cur_addr == 512)
				eeprom_cur_addr = 0;
		}
	}
	else
	{
		if ( msg->len )
		{
			eeprom_cur_addr = msg->buf[0];
			for (i = 0; i < msg->len; i++)
			{
				eeprom_buffer[eeprom_cur_addr++] = msg->buf[i];
				if (eeprom_cur_addr == 512)
					eeprom_cur_addr = 0;
			}
		}
	}
}

static int i2c_adapter_virtual_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
    int i;

    for(i = 0; i < num; i++) 
    {
        if( msgs[i].addr == 0x50 ) 
        {
            eeprom_emulate_xfer(adap, &msgs[i]);
        }
        else
        {
            i = -EIO;
            break;
        }
    }

    return i;  // 这个返回值会表现为地址值
}

static u32 i2c_adapter_virtual_func(struct i2c_adapter * adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_NOSTART | I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_SMBUS_READ_BLOCK_DATA |
	       I2C_FUNC_SMBUS_BLOCK_PROC_CALL |
	       I2C_FUNC_PROTOCOL_MANGLING;
}

const struct i2c_algorithm i2c_adapter_virtual_algo = {
    .master_xfer = i2c_adapter_virtual_master_xfer,
    .functionality = i2c_adapter_virtual_func,
};

static int i2c_adapter_virtual_probe(struct platform_device *pdev) 
{
    struct device* dev = &pdev->dev;
    struct device_node* np = dev->of_node;
    struct i2c_adapter* adap;

    struct i2c_adapter_private_data *priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if( !priv ) {
        return -ENOMEM;
    }

    adap = &priv->adapter;

    adap->owner = THIS_MODULE;
    adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
    adap->dev.parent = dev;
    adap->dev.of_node = np;
    adap->nr = -1;

    snprintf(adap->name, sizeof(adap->name), "i2c_adapter_virtual");

    adap->algo = &i2c_adapter_virtual_algo;

    platform_set_drvdata(pdev, priv);

    i2c_add_adapter(adap);

    return 0;
}

static int i2c_adapter_virtual_remove(struct platform_device *pdev)
{
    struct i2c_adapter_private_data* priv = platform_get_drvdata(pdev);
    i2c_del_adapter(&priv->adapter);
    return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id i2c_adapter_virtual_dt_ids[] = {
	{ .compatible = "i2c_adapter_virtual", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, i2c_adapter_virtual_dt_ids);
#endif

static struct platform_driver i2c_adapter_virtual_driver = {
	.driver		= {
		.name	= "i2c_adapter_virtual",
		.of_match_table	= of_match_ptr(i2c_adapter_virtual_dt_ids),
	},
	.probe		= i2c_adapter_virtual_probe,
	.remove		= i2c_adapter_virtual_remove,
};

static int __init i2c_adapter_virtual_init(void) 
{
    int ret = 0;

    pr_emerg("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
    ret = platform_driver_register(&i2c_adapter_virtual_driver);
    if (ret)
		pr_emerg(KERN_ERR "i2c-gpio: probe failed: %d\n", ret);

    return ret;
}

module_init(i2c_adapter_virtual_init);

static void __exit i2c_adapter_virtual_exit(void) {
    platform_driver_unregister(&i2c_adapter_virtual_driver);
}

module_exit(i2c_adapter_virtual_exit);

MODULE_LICENSE("GPL");