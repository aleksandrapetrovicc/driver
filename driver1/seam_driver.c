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
#define MAX_PKT_LEN 640*480*4 ///how much do we need
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
  .mmap = seam_dma_mmap
};

static struct of_device_id seam_of_match[] = 
{
	{ .compatible = "xlnx,seam", },
	{ .compatible = "xlnx,dma", },  ////////////check this
	{ /* end of list */ },
};

static struct platform_driver seam_driver = 
{
	.driver = 
  {
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
static ssize_t vga_dma_mmap(struct file *f, struct vm_area_struct *vma_s);
static int __init seam_init(void);
static void __exit seam_exit(void);

static irqreturn_t dma_isr(int irq,void*dev_id);
int dma_init(void __iomem *base_address);
u32 dma_simple_write(dma_addr_t TxBufferPtr, u32 max_pkt_len, void __iomem *base_address); // helper function, defined later


int device_fsm = 0;

//PROBE
static int seam_probe(struct platform_device *pdev) 
{
  struct resource *r_mem;
  int rc = 0;

  printk(KERN_INFO "Probing\n");

  r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!r_mem) 
  {
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
//THIS MIGH BE CHANGED
#define BUFF_SIZE 1000
int end_read = 0;
int i = 0;
int j = 0;

ssize_t seam_read(struct file *pfile, char __user *buffer, size_t length, loff_t *offset)
{
  char buf[BUFF_SIZE];
  long int len = 0;
  u32 val;
  int minor = MINOR(pfile->f_inode->i_rdev);

  printk(KERN_INFO "i = %d, len = %ld, end_read = %d\n", i, len, end_read);
  if (end_read == 1)
    {
      end_read = 0;
      return 0;
    }

    switch(minor)
    {
      case 0: //device seam
        printk(KERN_INFO "Succesfully read from seam device.\n");
        //*******************************/
        //////////for our project
        //len = scnprintf(buf, BUFF_SIZE, "nasi signali");
        //*******************************/
        if (copy_to_user(buffer, buf, len))
        return -EFAULT;
        end_read = 1;
      break;
      case 1: //device dma
      break;
      default:
        printk(KERN_ERR "[READ] Invalid minor. \n");
        end_read = 1;
    }
    return len;
}


ssize_t seam_write(struct file *pfile, const char __user *buffer, size_t length, loff_t *offset)
{

}

//DMA FUNCTIONS

static ssize_t vga_dma_mmap(struct file *f, struct vm_area_struct *vma_s)
{
  int ret = 0;
  long length = vma_s->vm_end - vma_s->vm_start;
  //printk(KERN_INFO "DMA TX Buffer is being memory mapped\n");
  if(length > MAX_PKT_LEN)
  {
    return -EIO;
    printk(KERN_ERR "Trying to mmap more space than it's allocated\n");
  }
  ret = dma_mmap_coherent(NULL, vma_s, tx_vir_buffer, tx_phy_buffer, length);
  if(ret<0)
  {
    printk(KERN_ERR "memory map failed\n");
    return ret;
  }
  return 0;
}

static irqreturn_t dma_isr(int irq,void*dev_id)
{
  u32 IrqStatus;
  //??itanje irq_status bita iz MM2S_DMASR registra
  IrqStatus = ioread32(vp->base_addr + 4);
  //clear irq_status bita u MM2S_DMASR registru. (To se radi
  //upisivanjem logi??ke 1 na 13. bit u MM2S_DMASR (IOC_Irq).
  iowrite32(IrqStatus | 0x00007000, vp->base_addr + 4);
  /*Slanje transakcije*/
  dma_simple_write(tx_phy_buffer, MAX_PKT_LEN, vp->base_addr);
  return IRQ_HANDLED;;
}

int dma_init(void __iomem *base_address)
{
  u32 reset = 0x00000004;
  u32 IOC_IRQ_EN;
  u32 ERR_IRQ_EN;
  u32 MM2S_DMACR_reg;
  u32 en_interrupt;
  IOC_IRQ_EN = 1 << 12; // IOC_IrqEn bit u MM2S_DMACR registru
  ERR_IRQ_EN = 1 << 14; // Err_IrqEn bit u MM2S_DMACR registru
  // upisivanje u MM2S_DMACR registar. Postavljanje reset bita na logi??ku jedinicu (3. bit)
  iowrite32(reset, base_address);
  // ??itanje iz MM2S_DMACR registra DMA kontrolera
  MM2S_DMACR_reg = ioread32(base_address);
  // postavljanje na logi??ku jedinicu 13. i 15. bita u MM2S_DMACR
  en_interrupt = MM2S_DMACR_reg | IOC_IRQ_EN | ERR_IRQ_EN;
  iowrite32(en_interrupt, base_address); // upisivanje u MM2S_DMACR registar
  return 0;
}

u32 dma_simple_write(dma_addr_t TxBufferPtr, u32 max_pkt_len, void __iomem *base_address)
{
  u32 MM2S_DMACR_reg;
  MM2S_DMACR_reg = ioread32(base_address); // ??itanje iz MM2S_DMACR registra
  // setovanje RS bita u MM2S_DMACR registru ??ime startujemo DMA.
  iowrite32(0x1 | MM2S_DMACR_reg, base_address);
  // Upisivanje u MM2S_SA registar vrednost odakle DMA kontroler da krene.
  iowrite32((u32)TxBufferPtr, base_address + 24);
  // upisivanje u MM2S_LENGTH registar veli??inu paketa koji DMA kontroler treba da po??alje. U ovom slu??aju ta veli??ina je (640*480*4).
  iowrite32(max_pkt_len, base_address + 40);
  return 0;
}

//INIT FUNCTION
static int __init seam_init(void)
{
  printk(KERN_INFO "seam_init: Initialize Module \"%s\"\n", DEVICE_NAME);
	if(alloc_chrdev_region(&my_dev_id, 0, 2, "seam_region"))   ////is 2 here ok????
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
  printk(KERN_INFO "Class created!.\n");
    
  
  if (device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),0), NULL, "xlnx, seam") == NULL)
  {
    printk(KERN_ERR "failed to create device seam\n");
    goto fail_1;
  }
  printk(KERN_INFO "Device created - seam.\n");

  if (device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),1), NULL, "xlnx,dma") == NULL)
  {
   printk(KERN_ERR "failed to create device dma\n");
       goto fail_2;
  }
   printk(KERN_INFO "device created - dma\n");

  my_cdev = cdev_alloc();	
  my_cdev->ops = &seam_operations;
  my_cdev->owner = THIS_MODULE;
  ret = cdev_add(my_cdev, my_dev_id, 1);
  
  if (cdev_add(my_cdev, my_dev_id, 2) == -1)
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