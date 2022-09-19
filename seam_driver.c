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
#include <linux/delay.h>

#include <linux/dma-mapping.h> 
#include <linux/mm.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR ("g06-2022");
MODULE_DESCRIPTION("Driver for Seam Carving algorithm");
MODULE_ALIAS("custom:seam");
#define DEVICE_NAME "seam"
#define DRIVER_NAME "seam_driver"

/////adresses for registers

// INFO

struct seam_info 
{
	unsigned long mem_start;
	unsigned long mem_end;
	void __iomem *base_addr;
};

struct file_operations seam_operations = 
{  ///////////static?
    .owner = THIS_MODULE,
	.open = seam_open,
	.read = seam_read,
	.write = seam_write,
	.release = seam_close,
};

static struct of_device_id seam_of_match[] = 
{
	{ .compatible = "xlnx,seam", },
	{ .compatible = "xlnx,bram", },  ////////////check this
	{ /* end of list */ },
};

static struct platform_driver seam_driver = 
{
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= seam_of_match,
	},
	.probe		= seam_probe,
	.remove		= seam_remove,
};

static struct seam_info *seam = NULL;   
static struct seam_info *dma = NULL;
static dev_t my_dev_id;
static struct class *my_class;
static struct device *my_device;
static struct cdev *my_cdev;

dma_addr_t tx_phy_buffer; ////is this needed
u32 *tx_vir_buffer;

MODULE_DEVICE_TABLE(of, seam_of_match);

// FUNCTIONS

static int seam_probe(struct platform_device *pdev);
static int seam_remove(struct platform_device *pdev);
int seam_open(struct inode *pinode, struct file *pfile);
int seam_close(struct inode *pinode, struct file *pfile);
ssize_t seam_read(struct file *pfile, char __user *buffer, size_t length, loff_t *offset);
ssize_t seam_write(struct file *pfile, const char __user *buffer, size_t length, loff_t *offset);

static int __init seam_init(void);
static void __exit seam_exit(void);

int device_fsm = 0;

//PROBE
static int seam_probe(struct platform_device *pdev) 
{
  struct resource *r_mem;
  int rc = 0;

  printk(KERN_INFO "Probing\n");

  r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!r_mem) {
    printk(KERN_ALERT "invalid address\n");
    return -ENODEV;
  }
  printk(KERN_INFO "Starting probing.\n");
  switch (device_fsm)
    {
      case 0: //device seam_carving
        seam = (struct seam_info *) kmalloc(sizeof(struct seam_info), GFP_KERNEL);
        if (!seam)
        {
          printk(KERN_ALERT "Cound not allocate seam device\n");
          return -ENOMEM;
        }
        seam->mem_start = r_mem->start;
        seam->mem_end = r_mem->end;
        if(!request_mem_region(seam->mem_start, seam->mem_end - seam->mem_start+1, DRIVER_NAME))
        {
          printk(KERN_ALERT "Couldn't lock memory region at %p\n",(void *)seam->mem_start);
          rc = -EBUSY;
          goto error1;
        }
        seam->base_addr = ioremap(seam->mem_start, sema->mem_end-seam->mem_start+1);
        if (!seam->base_addr)
        {
          printk(KERN_ALERT "[PROBE]: Could not allocate seam iomem\n");
          rc = -EIO;
          goto error2;
        }
      ++device_fsm;
      printk(KERN_INFO "[PROBE]: Finished probing seam.\n");
      return 0;
      error2:
      release_mem_region(seam->mem_start, seam->mem_end-sem->mem_start+1);
      error1:
      return rc;
      break;
      case 1: //device dma
      dma = (struct seam_info *) kmalloc(sizeof(struct seam_info), GFP_KERNEL);
      if(!dma)
      {
        printk(KERN_ALERT "Cound not allocate dma device\n");
          return -ENOMEM;
      }
      dma->mem_start = r_mem->mem_start;
      dma->mem_end = r_mem->mem_end;
       if(!request_mem_region(dma->mem_start, dma->mem_end - dma->mem_start+1, DRIVER_NAME))
        {
          printk(KERN_ALERT "Couldn't lock memory region at %p\n",(void *)dma->mem_start);
          rc = -EBUSY;
          goto error3;
        }
        dma->base_addr = ioremap(dma->mem_start. dma->mem_end-dma->mem_start+1);
         if (!dma->base_addr)
        {
          printk(KERN_ALERT "[PROBE]: Could not allocate dma iomem\n");
          rc = -EIO;
          goto error4;
        }
      ++device_fsm;
      printk(KERN_INFO "[PROBE]: Finished probing dma.\n");
      return 0;
      error4:
        release_mem_region(dma->mem_start, dma->mem_end - dma->mem_start + 1);
      error3:
        return rc;
      break;
      default:
      printk(KERN_INFO "[PROBE] Device FSM in illegal state. \n");
      return -1;
    }
  printk(KERN_INFO "Succesfully probed driver\n");
  return 0;
}

//REMOVE

static int seam_remove(struct platform_device *pdev)
{
    switch()
    {
      case 0: //seam device
      printk(KERN_ALERT "seam device platform driver removed\n");
      iowrite32(0, seam->base_addr);
      iounmap(seam->base_addr);
      release_mem_region(seam->mem_start, seam->mem_end - seam->mem_start + 1);
      kfree(seam);
      break;
      case 1: //dma device
      printk(KERN_ALERT "dma platform driver removed\n");
      iowrite32(0, dma->base_addr);
      iounmap(dma->base_addr);
      release_mem_region(dma->mem_start, dma->mem_end - dma->mem_start + 1);
      kfree(dma);
      break;
      default:
      printk(KERN_INFO "[REMOVE] Device FSM in illegal state. \n");
      return -1;
    }
	printk(KERN_INFO "Succesfully removed driver\n");
  return 0;
}

// OPEN

int seam_open(struct inode *pinode, struct file *pfile) 
{
	printk(KERN_INFO "Succesfully opened SEAM\n");
	return 0;
}

// CLOSE

int seam_close(struct inode *pinode, struct file *pfile) 
{
	printk(KERN_INFO "Succesfully closed SEAM\n");
	return 0;
}

//READ AND WRITE FUNCTIONS*********************




//INIT FUNCTION
static int __init seam_init(void)
{
	int ret = 0;
  printk(KERN_INFO "seam_init: Initialize Module \"%s\"\n", DEVICE_NAME);
	ret = alloc_chrdev_region(&my_dev_id, 0, 1, "seam_region");   ////is 1 here ok????
  
  if (ret)
  {
    printk(KERN_ALERT "<1>Failed CHRDEV!.\n");
    return -1;
  }
  printk(KERN_INFO "Succ CHRDEV!.\n");
  my_class = class_create(THIS_MODULE, "seam_class");
  
  if (my_class == NULL)
  {
	  printk(KERN_ALERT "<1>Failed class create!.\n");
    goto fail_0;
  }
  printk(KERN_INFO "Succ class chardev1 create!.\n");
    
  
  if (device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),0), NULL, "xlnx, seam") == NULL)
  {
    printk(KERN_ERR "failed to create device seam\n");
    goto fail_1;
  }
  printk(KERN_INFO "Device created - seam.\n");

if (device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),1), NULL, "xlnx,dma") == NULL) {
     printk(KERN_ERR "failed to create device dma\n");
     goto fail_2;
   }
   printk(KERN_INFO "device created - dma\n");

  my_cdev = cdev_alloc();	
  my_cdev->ops = &seam_operations;
  my_cdev->owner = THIS_MODULE;
  ret = cdev_add(my_cdev, my_dev_id, 1);
  
  if (ret)
  {
    printk(KERN_ERR "seam_init: Failed to add cdev\n");
    goto fail_3;
  }
  printk(KERN_INFO "seam Device init.\n");

  return platform_driver_register(&seam_driver);

  fail_3:
	device_destroy(my_class, MKDEV(MAJOR(my_dev_id),1));
  fail_3:
	device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
  fail_1:
	class_destroy(my_class);
  fail_0:
	unregister_chrdev_region(my_dev_id, 1);
  return -1;
}

//EXIT FUNCTION
static void __exit seam_exit(void)  		
{
  platform_driver_unregister(&seam_driver);
  cdev_del(&my_cdev);
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),1));
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
  class_destroy(my_class);
  unregister_chrdev_region(my_dev_id, 1);
  printk(KERN_INFO "seam_exit: Exit Device Module \"%s\".\n", DEVICE_NAME);
}

module_init(seam_init);
module_exit(seam_exit);