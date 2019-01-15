/******************************************************************************

                              Copyright (c) 2014
                            Lantiq Deutschland GmbH

  For licensing information, see the file 'LICENSE' in the root folder of
  this software module.

******************************************************************************/

/**
   \file dxs_irq_linux.c
   This file contains the implementation of DUSLIC XS interrupt handling driver.
*/

/* ========================================================================= */
/*                                 Includes                                  */
/* ========================================================================= */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/delay.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0))
#include <linux/irqdomain.h>
#endif


#include "dxs_irq.h"


/* ========================================================================= */
/*                             Macro definitions                             */
/* ========================================================================= */

#define DXS_EXINT_MODE 1 /* Interrupt on falling edge */

#ifdef CONFIG_SEC_TETRA_PROJECT
#define SLIC_RST	(902 + 53)
#endif

struct class *slic_class;
struct device *dxs_dev;


/* ========================================================================= */
/*                             Type definitions                              */
/* ========================================================================= */

/* status of the device */
typedef struct {
   /* interrupt handler has been requested */
   int irq_handler_requested;
   /* interrupt number */
   int irq;
   /* interrupt has occurred */
   int interrupted;
} dxs_irq_device_t;

/* main driver structure */
typedef struct {
   wait_queue_head_t queue;
   dxs_irq_device_t dxs_irq_device[DXS_MAX_DEVICES];
   struct miscdevice dxs_irq_miscdev;
} dxs_irq_t;

/* ========================================================================= */
/*                          Function declarations                            */
/* ========================================================================= */

static int dxs_irq_open(struct inode *inode, struct file *filp);
static int dxs_irq_release(struct inode *inode, struct file *filp);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
static int dxs_irq_ioctl(struct inode *inode, struct file *filp,
   unsigned int cmd, unsigned long arg);
#else
static long dxs_irq_ioctl(struct file *filp,
   unsigned int cmd, unsigned long arg);
#endif
static unsigned int dxs_irq_poll(struct file *filp, poll_table *wait);

/* ========================================================================== */
/*                             Global variables                               */
/* ========================================================================== */

int dxs_irq_minor =   0;

dxs_irq_t dxs_irq;

module_param(dxs_irq_minor, int, S_IRUGO);


/*lint -restore */

/** The driver callbacks which will be registered with the kernel*/
static struct file_operations dxs_irq_fops = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,0)
   .owner =    THIS_MODULE,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,0) */
   .llseek =   NULL,
   .read =     NULL,
   .write =    NULL,
   .poll =     dxs_irq_poll,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
   .ioctl =    dxs_irq_ioctl,
#else
   .unlocked_ioctl = dxs_irq_ioctl,
#endif
   .open =     dxs_irq_open,
   .release =  dxs_irq_release
};



/* ========================================================================== */
/*                                Local functions                             */
/* ========================================================================== */

/*
 * Interrupt handler.
 */
#if   (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
static void dxs_irq_handler(int irq, void *pDev, struct pt_regs *regs);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static irqreturn_t dxs_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t dxs_irq_handler(int irq, void *dev_id)
#endif
{
   /* dev_id points to "interrupted" field of the device */
   *((int*)dev_id) = 1;
   wake_up_interruptible(&dxs_irq.queue);

   return IRQ_HANDLED;
}

/**
   Request interrupt on given device

   \param  pointer to DXS_INT_CONF_t structure

   \return
   0 on success or error code
*/
static int dxs_irq_handler_register(DXS_INT_CONF_t* pIntCfg)
{
   int ret;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0))
   unsigned int virq;
#endif

   if (pIntCfg->dev < 0 || pIntCfg->dev >= DXS_MAX_DEVICES)
      return -EINVAL;
   if (dxs_irq.dxs_irq_device[pIntCfg->dev].irq_handler_requested)
      return 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0))
   virq = irq_create_mapping(NULL, pIntCfg->irq);
   if (!virq)
      return -1;
   ret = request_irq(virq, dxs_irq_handler, IRQF_SHARED,
      DXS_IRQ_NAME,
      &dxs_irq.dxs_irq_device[pIntCfg->dev].interrupted);
#else
   ret = request_irq(pIntCfg->irq, dxs_irq_handler, IRQF_SHARED,
      DXS_IRQ_NAME,
      &dxs_irq.dxs_irq_device[pIntCfg->dev].interrupted);
#endif
   if (ret)
   {
      printk(KERN_INFO "%s: can't get assigned irq %i for device %d, error "
         "%d\n",
         DXS_IRQ_NAME,
         pIntCfg->irq,
         pIntCfg->dev,
         ret);
      return ret;
   }
   dxs_irq.dxs_irq_device[pIntCfg->dev].irq_handler_requested = 1;
   dxs_irq.dxs_irq_device[pIntCfg->dev].irq = pIntCfg->irq;
   return 0;
}

/**
   Get device that has signaled an interrupt

   \param  none

   \return
   Device number or -1 if no interupts since last call
*/
static int dxs_irq_device_get(void)
{
   static int dev = 0;
   int from, ret = -1;

   /* Save device number we started checking from, end cycle when we get to
    * the same device. It means no interrupts occurred.
    */
   from = dev;
   do
   {
      if (dxs_irq.dxs_irq_device[dev].interrupted)
      {
         /* found a device with interrupt */
         dxs_irq.dxs_irq_device[dev].interrupted = 0;
         /* save device number in ret */
         ret = dev;
         /* increase dev to avoid device starvation */
         if (++dev >= DXS_MAX_DEVICES)
            dev = 0;
         break;
      }
      if (++dev >= DXS_MAX_DEVICES)
         dev = 0;
   } while (dev != from);
   return ret;
}

static int dxs_irq_open(struct inode *inode, struct file *filp)
{
   return 0;
}

static int dxs_irq_release(struct inode *inode, struct file *filp)
{
   return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
int dxs_irq_ioctl(struct inode *inode, struct file *filp,
   unsigned int cmd, unsigned long arg)
#else
static long dxs_irq_ioctl(struct file *filp,
   unsigned int cmd, unsigned long arg)
#endif
{
   int ret = 0;

   switch (cmd)
   {
     case DXS_INT_CONF:
     {
        DXS_INT_CONF_t int_cfg;

        if(copy_from_user(&int_cfg, (DXS_INT_CONF_t *) arg, sizeof(int_cfg)))
			return -EFAULT;
        ret = dxs_irq_handler_register(&int_cfg);
        break;
     }
     case DXS_INT_DEV_GET:
     {
        int dev;

        dev = dxs_irq_device_get();
        ret = __put_user(dev, (int __user *)arg);
        break;
     }
     default:
      return -EINVAL;
   }
   return ret;
}

static unsigned int dxs_irq_poll(struct file *filp, poll_table *wait)
{
   int i;
   unsigned int mask = 0;

   poll_wait(filp, &dxs_irq.queue, wait);
   for(i = 0; i < DXS_MAX_DEVICES; i++)
   {
      if (dxs_irq.dxs_irq_device[i].interrupted)
      {
         mask = POLLIN | POLLRDNORM;
         break;
      }
   }
   return mask;
}

/* ========================================================================== */
/*                                Public functions                            */
/* ========================================================================== */

ssize_t dxs_reset_store(struct device *dev,
		      struct device_attribute *attr, const char *buf,
		      size_t count)
{
   int value = 0;

   if ((buf == NULL) || kstrtouint(buf, 10, &value))
		return count;

   gpio_set_value(SLIC_RST, 0);
   usleep_range(100, 100);
   gpio_set_value(902+105, 0);
   gpio_set_value(SLIC_RST, 1);
   usleep_range(3000, 4000);
   pr_info("====================SLIC RESET====================\n");

   return count;
}

ssize_t dxs_reset_show(struct device *dev,
		     struct device_attribute *attr, char *buf)
{
   return snprintf(buf, sizeof(buf),
			"%d\n", gpio_get_value(SLIC_RST));
}

static DEVICE_ATTR(dxs_reset, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
		   dxs_reset_show, dxs_reset_store);


int dxs_irq_module_init(void)
{
   int ret;

   memset(&dxs_irq, 0, sizeof(dxs_irq));
   /* Initialize the device. */
   dxs_irq.dxs_irq_miscdev.minor = dxs_irq_minor;
   dxs_irq.dxs_irq_miscdev.name = DXS_IRQ_NAME;
   dxs_irq.dxs_irq_miscdev.fops = &dxs_irq_fops;

   slic_class = class_create(THIS_MODULE, "slic");
   if (IS_ERR(slic_class))
	pr_err("failed to create device slic!\n");
   
   dxs_dev = device_create(slic_class, NULL, 0, NULL, "dxs");
   if (IS_ERR(dxs_dev))
	pr_err("Failed to create device(dxs_dev)!\n");

   if (device_create_file(dxs_dev, &dev_attr_dxs_reset) < 0) {
	pr_err("failed to create device file, %s\n",
				 dev_attr_dxs_reset.attr.name);
   }

   ret = misc_register(&dxs_irq.dxs_irq_miscdev);
   if (ret != 0)
   {
      printk(KERN_ERR "%s: cannot register DUSLIC XS interrupt handler device "
         "node.\n",
         dxs_irq.dxs_irq_miscdev.name);
      return ret;
   }
   init_waitqueue_head(&dxs_irq.queue);

   printk("%s, (c) 2015 Lantiq Deutschland GmbH\n\r", DXS_IRQ_NAME);
   return 0;
}



void dxs_irq_module_exit(void)
{
   int i;

   for (i = 0; i < DXS_MAX_DEVICES; i++)
      if (dxs_irq.dxs_irq_device[i].irq_handler_requested)
      {
         free_irq(dxs_irq.dxs_irq_device[i].irq,
            &dxs_irq.dxs_irq_device[i].interrupted);
      }
   misc_deregister(&dxs_irq.dxs_irq_miscdev);
   device_remove_file(dxs_dev, &dev_attr_dxs_reset);
   device_destroy(slic_class, 0);
   class_destroy(slic_class);
}


/*lint -save -e{528, 546} */
module_init(dxs_irq_module_init);
module_exit(dxs_irq_module_exit);

/****************************************************************************/

MODULE_DESCRIPTION      ("DUSLIC XS interrupt handling driver - www.lantiq.com");
MODULE_AUTHOR           ("Lantiq Deutschland GmbH");
MODULE_SUPPORTED_DEVICE ("DUSLIC XS");
MODULE_LICENSE          ("Dual BSD/GPL");

