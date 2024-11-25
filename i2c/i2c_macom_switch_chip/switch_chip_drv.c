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

static int major = 0;
static struct class *switch_chip_class;
static struct i2c_client *switch_chip_client;

static ssize_t switch_chip_read (struct file *file, char __user *buf, size_t size, loff_t *offset)
{
	return 0;
}


static int switch_chip_open (struct inode *node, struct file *file)
{
    return 0;
}


static struct file_operations switch_chip_ops = {
	.owner = THIS_MODULE,
	.open  = switch_chip_open,
	.read  = switch_chip_read,
};

static const struct of_device_id of_match_ids_switch_chip[] = {
	{ .compatible = "lite-on,switch_chip",		.data = NULL },
	{ /* END OF LIST */ },
};

static const struct i2c_device_id switch_chip_ids[] = {
	{ "switch_chip",	(kernel_ulong_t)NULL },
	{ /* END OF LIST */ }
};

static int switch_chip_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pr_emerg("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	switch_chip_client = client;
	
	/* register_chrdev */
	major = register_chrdev(0, "switch_chip", &switch_chip_ops);

	switch_chip_class = class_create(THIS_MODULE, "switch_chip_class");
	device_create(switch_chip_class, NULL, MKDEV(major, 0), NULL, "switch_chip"); /* /dev/switch_chip */

	return 0;
}

static int switch_chip_remove(struct i2c_client *client)
{
	pr_emerg("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	device_destroy(switch_chip_class, MKDEV(major, 0));
	class_destroy(switch_chip_class);
	
	/* unregister_chrdev */
	unregister_chrdev(major, "switch_chip");

	return 0;
}

static struct i2c_driver i2c_switch_chip_driver = {
	.driver = {
		.name = "switch_chip",
		.of_match_table = of_match_ids_switch_chip,
	},
	.probe = switch_chip_probe,
	.remove = switch_chip_remove,
	.id_table = switch_chip_ids,
};

static int __init i2c_driver_switch_chip_init(void) 
{
    pr_emerg("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
    return i2c_add_driver(&i2c_switch_chip_driver);
}

module_init(i2c_driver_switch_chip_init);

static void __exit i2c_driver_switch_chip_exit(void) 
{
    i2c_del_driver(&i2c_switch_chip_driver);
}

module_exit(i2c_driver_switch_chip_exit);

MODULE_LICENSE("GPL");