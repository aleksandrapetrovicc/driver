#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/device.h>

#include <linux/io.h> //iowrite ioread
#include <linux/slab.h>//kmalloc kfree
#include <linux/platform_device.h>//platform driver
#include <linux/of.h>//of_match_table
#include <linux/ioport.h>//ioremap

#include <linux/dma-mapping.h>  //dma access
#include <linux/mm.h>  //dma access
#include <linux/interrupt.h>  //interrupt handlers

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR ("g06-2022");
MODULE_DESCRIPTION("Driver for Seam Carving algorithm");
MODULE_ALIAS("custom:seam");
#define DEVICE_NAME "seam"
#define DRIVER_NAME "seam_driver"

#define MAX_PKT_LEN 640*480*4

//adresses for registers
#define COLSIZE     0x00
#define ROWSIZE     0x04
#define START       0x08
#define READY       0x12 ///here?

// ------------------------------------------
// DECLARATIONS
// ------------------------------------------

static dev_t my_dev_id;
static struct class *my_class;
static struct cdev  *my_cdev;

static int seam_probe     (struct platform_device *pdev);
static int seam_remove    (struct platform_device *pdev);
static int seam_open      (struct inode *pinode, struct file *pfile);
static int seam_close     (struct inode *pinode, struct file *pfile);
static ssize_t seam_read  (struct file *pfile, char __user *buffer, size_t length, loff_t *offset);
static ssize_t seam_write (struct file *pfile, const char __user *buffer, size_t length, loff_t *offset);
static ssize_t seam_dma_mmap(struct file *f, struct vm_area_struct *vma_s);

static int  __init seam_init(void);
static void __exit seam_exit(void);


static irqreturn_t dma_isr(int irq,void*dev_id);
int dma_init(void __iomem *base_address);
u32 dma_simple_write(dma_addr_t TxBufferPtr, u32 max_pkt_len, void __iomem *base_address);

int colsize, rowsize, start;
int ready;
dma_addr_t tx_phy_buffer;
u32 *tx_vir_buffer;

struct device_info
{
  unsigned long mem_start;
  unsigned long mem_end;
  void __iomem *base_addr;
  int irq_num;
};

static struct device_info *seam    = NULL;

static struct of_device_id device_of_match[] = {
  { .compatible = "xlnx,seam", },
  { /* end of list */ }
};

MODULE_DEVICE_TABLE(of, device_of_match);

static struct platform_driver my_driver = {
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table	= device_of_match,
  },
  .probe = seam_probe,
  .remove	= seam_remove,
};

struct file_operations my_fops =
{
	.owner   = THIS_MODULE,
	.open    = seam_open,
	.read    = seam_read,
	.write   = seam_write,
	.release = seam_close,
	//.mmap = seam_dma_mmap
};

// ------------------------------------------
// PROBE & REMOVE
// ------------------------------------------


static int seam_probe(struct platform_device *pdev)
{
  printk(KERN_INFO "[PROBE BEGINS");
  struct resource *r_mem;
  int rc = 0;
  r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		printk(KERN_ALERT "invalid address\n");
		return -ENODEV;
	}
  printk(KERN_INFO "[PROBE BEGINS] Starting probing.\n");

      seam = (struct device_info *) kmalloc(sizeof(struct device_info), GFP_KERNEL);
      if (!seam)
        {
          printk(KERN_ALERT "Cound not allocate seam device\n");
          return -ENOMEM;
        }
      seam->mem_start = r_mem->start;
      seam->mem_end   = r_mem->end;

      if(!request_mem_region(seam->mem_start, seam->mem_end - seam->mem_start+1, DRIVER_NAME))
        {
          printk(KERN_ALERT "Couldn't lock memory region at %p\n",(void *)seam->mem_start);
          rc = -EBUSY;
          goto error1;
        }

      seam->base_addr = ioremap(seam->mem_start, seam->mem_end - seam->mem_start + 1);
      if (!seam->base_addr)
        {
          printk(KERN_ALERT "[PROBE]: Could not allocate seam iomem\n");
          rc = -EIO;
          goto error2;
        }

// Get irq num 
	seam->irq_num = platform_get_irq(pdev, 0);
	if(!seam->irq_num)
	{
		printk(KERN_ERR "seam_probe: Could not get IRQ resource\n");
		rc = -ENODEV;
		goto error2;
	}

	if (request_irq(seam->irq_num, dma_isr, 0, DEVICE_NAME, NULL)) {
		printk(KERN_ERR "seam_dma_probe: Could not register IRQ %d\n", seam->irq_num);
		return -EIO;
		goto error3;
	}
	else {
		printk(KERN_INFO "seam_dma_probe: Registered IRQ %d\n", seam->irq_num);
	}

	//INIT DMA 
	dma_init(seam->base_addr);
	dma_simple_write(tx_phy_buffer, MAX_PKT_LEN, seam->base_addr); // helper function, defined later

      printk(KERN_INFO "[PROBE]: Finished probing seam.\n");
      return 0;

      error3:
	      iounmap(seam->base_addr);
      error2:
        release_mem_region(seam->mem_start, seam->mem_end - seam->mem_start + 1);
      error1:
        return rc;
 
  printk(KERN_INFO "[PROBE ENDS] Succesfully probed driver\n");
  return 0;
}

static int seam_remove(struct platform_device *pdev)
{
  u32 reset = 0x00000004;
  printk(KERN_ALERT "[REMOVE BEGINS] seam device platform driver removed\n");
 // iowrite32(0, seam->base_addr); //0 or reset?
  iowrite32(reset, seam->base_addr); 
  free_irq(seam->irq_num, NULL);
  iounmap(seam->base_addr);
  release_mem_region(seam->mem_start, seam->mem_end - seam->mem_start + 1);
  kfree(seam);
  printk(KERN_INFO "[REMOVE ENDS] Succesfully removed driver\n");
  return 0;
}

// ------------------------------------------
// OPEN & CLOSE
// ------------------------------------------

static int seam_open(struct inode *pinode, struct file *pfile)
{
		printk(KERN_INFO "[OPEN] Succesfully opened file\n");
		return 0;
}

static int seam_close(struct inode *pinode, struct file *pfile)
{
		printk(KERN_INFO "[CLOSE] Succesfully closed file\n");
		return 0;
}

// ------------------------------------------
// READ & WRITE
// ------------------------------------------

#define BUFF_SIZE 40
int i = 0;
int end_read = 0;


ssize_t seam_read(struct file *pfile, char __user *buffer, size_t length, loff_t *offset)
{
  char buf[BUFF_SIZE];
  long int len=0;
  //u32 val;
  int minor = MINOR(pfile->f_inode->i_rdev);

  printk(KERN_INFO "[READ BEGINS] Seam read entered \n"); 
  printk(KERN_INFO "i = %d, len = %ld, end_read = %d\n", i, len, end_read);
  if (end_read == 1)
    {
      end_read = 0;
      return 0;
    }

  switch (minor)
    {
    case 0: //device seam
      printk(KERN_INFO "[READ ENDS] Succesfully read from seam device.\n");
      //signals here
      
      len = scnprintf(buf, BUFF_SIZE, "[READ] colsize = %d, rowsize = %d, start = %d, ready = %d\n", colsize, rowsize, start, ready);
      if (copy_to_user(buffer, buf, len))
        return -EFAULT;
      end_read = 1;
      break;

    default:
      printk(KERN_ERR "[READ] Invalid minor. \n");
      end_read = 1;
    }

  return len;
}

ssize_t seam_write(struct file *pfile, const char __user *buffer, size_t length, loff_t *offset)
{
  char buf[length+1];
  int minor = MINOR(pfile->f_inode->i_rdev);
  if (copy_from_user(buf, buffer, length))
    return -EFAULT;
  buf[length]='\0';

  switch (minor)
    {
    case 0: //device seam
      sscanf(buf, "%d", &colsize);
      printk(KERN_INFO "[WRITE BEINGS] ");
      printk(KERN_INFO "%d, %d, %d\n", colsize, rowsize, start);
      iowrite32(colsize, seam->base_addr+COLSIZE);
      iowrite32(rowsize, seam->base_addr+ROWSIZE);
      iowrite32(start, seam->base_addr+START);
      printk(KERN_INFO "[WRITE ENDS] Succesfully wrote into seam device.\n");
      break;

    default:
      printk(KERN_INFO "[WRITE] Invalid minor. \n");
  }

  return length;
}

static ssize_t seam_dma_mmap(struct file *f, struct vm_area_struct *vma_s)
{
	int ret = 0;
	long length = vma_s->vm_end - vma_s->vm_start;

	printk(KERN_INFO "DMA TX Buffer is being memory mapped\n");

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


/****************************************************/
// IMPLEMENTATION OF DMA related functions

static irqreturn_t dma_isr(int irq,void*dev_id)
{
	u32 IrqStatus;  
	//Read pending interrupts
	IrqStatus = ioread32(seam->base_addr + 4);//read irq status from MM2S_DMASR register
	iowrite32(IrqStatus | 0x00007000, seam->base_addr + 4);//clear irq status in MM2S_DMASR register
	//(clearing is done by writing 1 on 13. bit in MM2S_DMASR (IOC_Irq)

	//Send a transaction
	dma_simple_write(tx_phy_buffer, MAX_PKT_LEN, seam->base_addr); //My function that starts a DMA transaction
	return IRQ_HANDLED;;
}

int dma_init(void __iomem *base_address)
{
	u32 reset = 0x00000004;
	u32 IOC_IRQ_EN; 
	u32 ERR_IRQ_EN;
	u32 MM2S_DMACR_reg;
	u32 en_interrupt;

	IOC_IRQ_EN = 1 << 12; // this is IOC_IrqEn bit in MM2S_DMACR register
	ERR_IRQ_EN = 1 << 14; // this is Err_IrqEn bit in MM2S_DMACR register

	iowrite32(reset, base_address); // writing to MM2S_DMACR register. Seting reset bit (3. bit)

	MM2S_DMACR_reg = ioread32(base_address); // Reading from MM2S_DMACR register inside DMA
	en_interrupt = MM2S_DMACR_reg | IOC_IRQ_EN | ERR_IRQ_EN;// seting 13. and 15.th bit in MM2S_DMACR
	iowrite32(en_interrupt, base_address); // writing to MM2S_DMACR register  
	return 0;
}

u32 dma_simple_write(dma_addr_t TxBufferPtr, u32 max_pkt_len, void __iomem *base_address) {
	u32 MM2S_DMACR_reg;

	MM2S_DMACR_reg = ioread32(base_address); // READ from MM2S_DMACR register

	iowrite32(0x1 |  MM2S_DMACR_reg, base_address); // set RS bit in MM2S_DMACR register (this bit starts the DMA)

	iowrite32((u32)TxBufferPtr, base_address + 24); // Write into MM2S_SA register the value of TxBufferPtr.
	// With this, the DMA knows from where to start.

	iowrite32(max_pkt_len, base_address + 40); // Write into MM2S_LENGTH register. This is the length of a tranaction.
	// In our case this is the size of the image (640*480*4)
	return 0;
}

// ------------------------------------------
// INIT & EXIT
// ------------------------------------------

static int __init seam_init(void)
{
   printk(KERN_INFO "\n");
   printk(KERN_INFO "[INIT BEGIN] SEAM driver starting insmod.\n");

   if (alloc_chrdev_region(&my_dev_id, 0, 1, "seam_region") < 0){  
      printk(KERN_ERR "failed to register char device\n");
      return -1;
   }
   printk(KERN_INFO "char device region allocated\n");

   my_class = class_create(THIS_MODULE, "seam_class");
   if (my_class == NULL){
      printk(KERN_ERR "failed to create class\n");
      goto fail_0;
   }
   printk(KERN_INFO "class created\n");

   if (device_create(my_class, NULL, MKDEV(MAJOR(my_dev_id),0), NULL, "xlnx,seam") == NULL) {
      printk(KERN_ERR "failed to create device seam\n");
      goto fail_1;
   }
   printk(KERN_INFO "device created - seam\n");

	my_cdev = cdev_alloc();
	my_cdev->ops = &my_fops;
	my_cdev->owner = THIS_MODULE;

	if (cdev_add(my_cdev, my_dev_id, 1) == -1) 
	{
      printk(KERN_ERR "failed to add cdev\n");
      goto fail_2;
	}
   printk(KERN_INFO "cdev added\n");
   printk(KERN_INFO "[INIT 1 ENDS] Seam driver initialized.\n");

//DMA INIT 
/*

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

  printk(KERN_INFO "[INIT 2 ENDS] DMA memory reset.\n");
}*/
   return platform_driver_register(&my_driver);

    fail_3:
      cdev_del(my_cdev);
    fail_2:
     device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));  
    fail_1:
      class_destroy(my_class);
   fail_0:
      unregister_chrdev_region(my_dev_id, 1);
   return -1;
}

static void __exit seam_exit(void)
{
  printk(KERN_INFO "[EXIT BEGINS] Seam driver starting rmmod.\n");
	platform_driver_unregister(&my_driver);
	cdev_del(my_cdev);
  device_destroy(my_class, MKDEV(MAJOR(my_dev_id),0));
  class_destroy(my_class);
  unregister_chrdev_region(my_dev_id,1);
  //dma_free_coherent(NULL, MAX_PKT_LEN, tx_vir_buffer, tx_phy_buffer);
	printk(KERN_INFO "[EXIT ENDS] Seam driver exited.\n");
}

module_init(seam_init);
module_exit(seam_exit);