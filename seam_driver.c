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
#include <linux/dma-mapping.h>
#include <linux/mm.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR ("g06-2022");
MODULE_DESCRIPTION("Driver for Seam Carving algorithm");
MODULE_ALIAS("custom:seam");
#define DEVICE_NAME "seam"
#define DRIVER_NAME "seam_driver"
#define MAX_PKT_LEN 640*480*4 ///how much do we need
#define MAX_X 1000 //example
#define MAX_Y 1000 //example
/////adresses for registers
#define COLSIZE     0x00
#define ROWSIZE     0x04
#define START       0x08
#define READY       0x12 ///here?
// FUNCTIONS

static int seam_probe(struct platform_device *pdev);
static int seam_remove(struct platform_device *pdev);
int seam_open(struct inode *pinode, struct file *pfile);
int seam_close(struct inode *pinode, struct file *pfile);
ssize_t seam_read(struct file *pfile, char __user *buffer, size_t length, loff_t *offset);
ssize_t seam_write(struct file *pfile, const char __user *buffer, size_t length, loff_t *offset);
static ssize_t seam_dma_mmap(struct file *f, struct vm_area_struct *vma_s);
static int __init seam_init(void);
static void __exit seam_exit(void);

static irqreturn_t dma_isr(int irq,void*dev_id);
int dma_init(void __iomem *base_address);
u32 dma_simple_write(dma_addr_t TxBufferPtr, u32 max_pkt_len, void __iomem *base_address);

int colsize, rowsize, start;
int ready;

static struct file_operations seam_operations = 
{
  .owner = THIS_MODULE,
	.open = seam_open,
	.release = seam_close,
	.read = seam_read,
	.write = seam_write
};

static struct of_device_id seam_of_match[] = 
{
	{ .compatible = "xlnx,seam", }, ////////////check this
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

// INFO

struct seam_info 
{
	unsigned long mem_start;
	unsigned long mem_end;
	void __iomem *base_addr;
  int irq_num;
};

static struct seam_info *seam = NULL;   
static dev_t my_dev_id;
static struct class *my_class;
static struct cdev my_cdev;
static int int_cnt;
dma_addr_t tx_phy_buffer;
u32 *tx_vir_buffer;

MODULE_DEVICE_TABLE(of, seam_of_match);


//PROBE
static int seam_probe(struct platform_device *pdev) 
{
  struct resource *r_mem;
  int rc = 0;

  printk(KERN_INFO "Probing sean.\n");

  r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!r_mem) 
  {
    printk(KERN_ALERT "Invalid address for seam.\n");
    return -ENODEV;
  }
  else
  {
    printk(KERN_INFO "Starting probing seam.\n");
  }
  seam = (struct seam_info *) kmalloc(sizeof(struct seam_info), GFP_KERNEL);
  if (!seam) 
  {
    printk(KERN_ALERT "Cound not allocate seam device\n");
    return -ENOMEM;
  }
  else
  {
    printk("Sucessful allocation of seam device.\n");
  }
  seam->mem_start = r_mem->start;
  seam->mem_end = r_mem->end;

  if (!request_mem_region(seam->mem_start, seam->mem_end - seam->mem_start + 1, DRIVER_NAME))
  {
    printk(KERN_ALERT "Couldn't lock memory region at %p for seam device.\n",(void *)seam->mem_start);
    rc = -EBUSY;
    goto error1;
  }
  else 
  {
    printk(KERN_INFO "Successfully allocated memory region for seam\n");
  }

  //mapping physical addresses to virtual addresses
  seam->base_addr = ioremap(seam->mem_start, seam->mem_end - seam->mem_start + 1);
  if (!seam->base_addr)
  {
    printk(KERN_ALERT "Could not allocate iomem for seam\n");
    rc = -EIO;
    goto error2;
  }
  else
  {
    printk(KERN_INFO "Ioremap was a success for seam\n");
  }

  seam->irq_num = platform_get_irq(pdev, 0);
  printk("irq number is: %d\n", seam->irq_num);
  
  if (request_irq(seam->irq_num, dma_isr, 0, DEVICE_NAME, NULL))
  {
    printk(KERN_ERR "Cannot register IRQ %d for seam\n", seam->irq_num);
    return -EIO;
  }
  else 
  {
    printk(KERN_INFO "Registered IRQ %d for seam\n", seam->irq_num);
  }

  //dma init
  dma_init(seam->base_addr);
  dma_simple_write(tx_phy_buffer, MAX_PKT_LEN, seam->base_addr);
  
  printk("Probing done for seam.\n");
  error2:
  release_mem_region(seam->mem_start, seam->mem_end - seam->mem_start + 1);
  error1:
  return rc;
}

//REMOVE

static int seam_remove(struct platform_device *pdev)
{
  u32 reset; //do i need first two lines
  reset = 0x00000004;
  iowrite32(reset, seam->base_addr); //reset or 0
  iounmap(seam->base_addr);
  free_irq(seam->irq_num, NULL);
  release_mem_region(seam->mem_start, seam->mem_end - seam->mem_start + 1);
  kfree(seam);
	printk(KERN_INFO "Succesfully removed seam driver\n");
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

//READ FUNCTION
#define BUFF_SIZE 1000
int end_read = 0;
int i = 0;

ssize_t seam_read(struct file *pfile, char __user *buffer, size_t length, loff_t *offset)
{
  
  char buf[BUFF_SIZE];
  long int len = 0;
/*
  printk(KERN_INFO "i = %d, len = %ld, end_read = %d\n", i, len, end_read);
  if (end_read == 1)
    {
      end_read = 0;
      return 0;
    }
    */
      printk(KERN_INFO "Succesfully read from seam device.\n");
      
    //  len = scnprintf(buf, BUFF_SIZE, "colsize = %d, rowsize = %d, start = %d", colsize, rowsize, start);      //add ready 
  //    if (copy_to_user(buffer, buf, len))
  //      return -EFAULT;
      end_read = 1;
      
    return len;
}

//WRITE FUNCTION
ssize_t seam_write(struct file *pfile, const char __user *buffer, size_t length, loff_t *offset)
{
  char buf[length+1];
  i = copy_from_user(buf, buffer, length);

  if (i)
    return -EFAULT;
  buf[length] = '\0';
/*
  sscanf(buf, "%d", &colsize);
  printk(KERN_INFO "%d, %d, %d\n", colsize, rowsize, start);
  iowrite32(colsize, seam->base_addr+COLSIZE);
  iowrite32(rowsize, seam->base_addr+ROWSIZE);
  iowrite32(start, seam->base_addr+start);
  printk(KERN_INFO, "Sucessfully wrote into seam device. %d, %d, %d\n", colsize, rowsize, start);
*/
  return length;
}

//DMA FUNCTIONS

static ssize_t seam_dma_mmap(struct file *f, struct vm_area_struct *vma_s)
{
  int ret = 0;
  long length = vma_s->vm_end - vma_s->vm_start;
  //printk(KERN_INFO "DMA TX Buffer is being memory mapped\n");
  if(length > MAX_PKT_LEN)
  {
    return -EIO;
    printk(KERN_ERR "Trying to mmap more space than it's allocated for seam\n");
  }
  ret = dma_mmap_coherent(NULL, vma_s, tx_vir_buffer, tx_phy_buffer, length);
  if(ret<0)
  {
    printk(KERN_ERR "memory map failed for seam\n");
    return ret;
  }
  return 0;
}

static irqreturn_t dma_isr(int irq,void*dev_id)
{
  u32 IrqStatus;
  //čitanje irq_status bita iz MM2S_DMASR registra
  IrqStatus = ioread32(seam->base_addr + 4);
  //clear irq_status bita u MM2S_DMASR registru. (To se radi
  //upisivanjem logičke 1 na 13. bit u MM2S_DMASR (IOC_Irq).
  iowrite32(IrqStatus | 0x00007000, seam->base_addr + 4);
  /*Slanje transakcije*/
  dma_simple_write(tx_phy_buffer, MAX_PKT_LEN, seam->base_addr);
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
  // upisivanje u MM2S_DMACR registar. Postavljanje reset bita na logičku jedinicu (3. bit)
  iowrite32(reset, base_address);
  // čitanje iz MM2S_DMACR registra DMA kontrolera
  MM2S_DMACR_reg = ioread32(base_address);
  // postavljanje na logičku jedinicu 13. i 15. bita u MM2S_DMACR
  en_interrupt = MM2S_DMACR_reg | IOC_IRQ_EN | ERR_IRQ_EN;
  iowrite32(en_interrupt, base_address); // upisivanje u MM2S_DMACR registar
  return 0;
}

u32 dma_simple_write(dma_addr_t TxBufferPtr, u32 max_pkt_len, void __iomem *base_address)
{
  u32 MM2S_DMACR_reg;
  MM2S_DMACR_reg = ioread32(base_address); // čitanje iz MM2S_DMACR registra
  // setovanje RS bita u MM2S_DMACR registru čime startujemo DMA.
  iowrite32(0x1 | MM2S_DMACR_reg, base_address);
  // Upisivanje u MM2S_SA registar vrednost odakle DMA kontroler da krene.
  iowrite32((u32)TxBufferPtr, base_address + 24);
  // upisivanje u MM2S_LENGTH registar veličinu paketa koji DMA kontroler treba da pošalje. U ovom slučaju ta veličina je (640*480*4).
  iowrite32(max_pkt_len, base_address + 40);
  return 0;
}

//INIT FUNCTION
static int __init seam_init(void)
{
  int_cnt = 0;

  printk(KERN_INFO "seam_init: Initialize Module \"%s\"\n", DEVICE_NAME);
	
  if(alloc_chrdev_region(&my_dev_id, 0, 1, "seam_region")<0)   
  {
    printk(KERN_ALERT "Failed CHRDEV for seam!\n");
    return -1;
  }
  printk(KERN_INFO "Successful CHRDEV for seam!\n");
  
  if((my_class = class_create(THIS_MODULE, "seam_class")) == NULL)
  {
    printk(KERN_ALERT "Failed to create seam class!\n");
    goto fail_0;
  }
  printk(KERN_INFO "Successfully created seam_class!\n");
  if (device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),0), NULL, "seam") == NULL)
  {
    goto fail_1;
  }

  printk(KERN_INFO "Seam device created.\n");

  //my_cdev = cdev_alloc();
	//my_cdev->ops = &seam_operations;
	//my_cdev->owner = THIS_MODULE;
  cdev_init(&my_cdev, &seam_operations);

  if (cdev_add(&my_cdev, my_dev_id, 1) == -1)
  {
    goto fail_2;
  }

  printk(KERN_INFO "Seam device init.\n");
///DIFERENCE STARTS HERE*************
  tx_vir_buffer = dma_alloc_coherent(NULL, MAX_PKT_LEN, &tx_phy_buffer, GFP_DMA | GFP_KERNEL); //GFP_KERNEL
  if(!tx_vir_buffer)
  {
    printk(KERN_ALERT "Could not allocate dma_alloc_coherent for img");
    goto fail_3;  
  }
  else
  {
    printk("dma_alloc_coherent success img\n");
  }
  for (i = 0; i < MAX_PKT_LEN/4;i++)
  {
    tx_vir_buffer[i] = 0x00000000;
  }
  printk(KERN_INFO "DMA memory reset.\n");

  ////DIFERENCE ENDS HERE************
  return platform_driver_register(&seam_driver);

  fail_3:
    cdev_del(&my_cdev);
  fail_2:
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
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
  class_destroy(my_class);
  unregister_chrdev_region(my_dev_id, 1);
  printk(KERN_INFO "seam_exit: Exit Device Module \"%s\".\n", DEVICE_NAME);
}

module_init(seam_init);
module_exit(seam_exit);