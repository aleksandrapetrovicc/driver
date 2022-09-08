#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/version.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>  // dma access
#include <linux/mm.h>  // dma access


MODULE_LICENSE("GPL");
MODULE_AUTHOR ("g06-2022");
MODULE_DESCRIPTION("Driver for Seam Carving algorithm");
MODULE_ALIAS("custom:seam");
#define DEVICE_NAME "seam"
#define DRIVER_NAME "seam_driver"

// ---------------------------------
// INFO
// ---------------------------------

struct seam_info {
	unsigned long mem_start;
	unsigned long mem_end;
	void __iomem *base_addr;
};

struct file_operations seam_operations = {  ///////////static?
    .owner = THIS_MODULE,
	.open = seam_open,
	.read = seam_read,
	.write = seam_write,
	.release = seam_close,
};

static struct of_device_id seam_of_match[] = {
	{ .compatible = "xlnx,seam", },
	{ .compatible = "xlnx,bram", },  ////////////check this
	{ /* end of list */ },
};

static struct platform_driver seam_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= seam_of_match,
	},
	.probe		= seam_probe,
	.remove		= seam_remove,
};

static struct seam_info *vp = NULL;   /////do i need only 1
static dev_t dev_id;
static struct class *my_class;
static struct device *my_device;
static struct cdev *my_cdev;

MODULE_DEVICE_TABLE(of, seam_of_match);
// ---------------------------------
// FUNCTIONS
// ---------------------------------
static int seam_probe(struct platform_device *pdev);
static int seam_remove(struct platform_device *pdev);
int seam_open(struct inode *pinode, struct file *pfile);
int seam_close(struct inode *pinode, struct file *pfile);
ssize_t seam_read(struct file *pfile, char __user *buffer, size_t length, loff_t *offset);
ssize_t seam_write(struct file *pfile, const char __user *buffer, size_t length, loff_t *offset);

static int __init seam_init(void);
static void __exit seam_exit(void);

//PROBE





///to do

// ------------------------------------------
// OPEN
// ------------------------------------------
int seam_open(struct inode *pinode, struct file *pfile) {
	printk(KERN_INFO "Succesfully opened SEAM\n");
	return 0;
}

// ------------------------------------------
// CLOSE
// ------------------------------------------
int seam_close(struct inode *pinode, struct file *pfile) {
	printk(KERN_INFO "Succesfully closed SEAM\n");
	return 0;
}

//EXIT FUNCTION
static void __exit seam_exit(void)  		
{
  platform_driver_unregister(&seam_driver);
  cdev_del(&my_cdev);
  device_destroy(cl, MKDEV(MAJOR(dev_id),0));
  class_destroy(my_class);
  unregister_chrdev_region(dev_id, 1);
  printk(KERN_INFO "seam_exit: Exit Device Module \"%s\".\n", DEVICE_NAME);
}

module_init(seam_init);
module_exit(seam_exit);