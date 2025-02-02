
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/property.h>
#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/nvmem-provider.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/gpio/consumer.h>
#include <linux/uaccess.h>
#include <linux/fs.h>


#if 0
static struct i2c_client *switch_chip_client;

static int __init i2c_client_switch_chip_init(void)
{
	struct i2c_adapter *adapter;

	static struct i2c_board_info board_info = {
	  I2C_BOARD_INFO("switch_chip", 0x1e),
	};

	
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	/* register I2C device */
	adapter = i2c_get_adapter(0);
	switch_chip_client = i2c_new_device(adapter, &board_info);
	i2c_put_adapter(adapter);
	return 0;
}

#else

static struct i2c_client *switch_chip_client;

/* Addresses to scan */
static const unsigned short normal_i2c[] = {
	0x1c, I2C_CLIENT_END
};

static int __init i2c_client_switch_chip_init(void)
{
	struct i2c_adapter *adapter;
	struct i2c_board_info board_info;
	memset(&board_info, 0, sizeof(struct i2c_board_info));
	strscpy(board_info.type, "switch_chip", sizeof(board_info.type));
	
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	/* register I2C device */
	adapter = i2c_get_adapter(0);
	switch_chip_client = i2c_new_probed_device(adapter, &board_info,
						   normal_i2c, NULL);
	i2c_put_adapter(adapter);
	return 0;
}
#endif

module_init(i2c_client_switch_chip_init);

static void __exit i2c_client_switch_chip_exit(void)
{
	i2c_unregister_device(switch_chip_client);
}
module_exit(i2c_client_switch_chip_exit);

MODULE_LICENSE("GPL");



