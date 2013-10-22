 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/ahern_pump.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>

#define PUMP_DEBUG

#define AHERN_PUMP_1_ENABLE       AT91_PIN_PC26
#define AHERN_PUMP_2_ENABLE       AT91_PIN_PC27
#define AHERN_PUMP_1_PULSE        AT91_PIN_PC28
#define AHERN_PUMP_2_PULSE        AT91_PIN_PC29

int pump_major = 0, pump_minor = 0;
int pump_1_authorized, pump_2_authorized;

struct pump_dev {
  struct cdev cdev;
};

struct pump_dev pump_device;

int pump_release(struct inode *inode, struct file *filp)
{
  return 0;
}

int pump_open(struct inode *inode, struct file *filp)
{
  return 0;
}

/*int pump_authorized()
{
  return pump_1_authorized || pump_2_authorized;
}*/

long pump_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int ret = 0;
    
#if defined PUMP_DEBUG
  printk(KERN_INFO "pump.ioctl(): ");
#endif

  switch (cmd) {
  case AHERN_PUMP_IOCTL_1_ENABLE:
    at91_set_gpio_value(AHERN_PUMP_1_ENABLE, 1);
    break;
  case AHERN_PUMP_IOCTL_2_ENABLE:
    at91_set_gpio_value(AHERN_PUMP_2_ENABLE, 1);
    break;
  case AHERN_PUMP_IOCTL_1_DISABLE:
    at91_set_gpio_value(AHERN_PUMP_1_ENABLE, 0);
    break;
  case AHERN_PUMP_IOCTL_2_DISABLE:
    at91_set_gpio_value(AHERN_PUMP_2_ENABLE, 0);
    break;
  }

  return ret;
}

ssize_t pump_read(struct file *filp, char __user *buf, size_t count,
                    loff_t *f_pos)
{
  int rc = 2;
    
  uint16_t ticks = 0x5FF5;
    
  if (copy_to_user(buf, &ticks, 2)) {
    printk(KERN_ERR "pump_read(): Error copying data to userspace!\n");
    return -EFAULT;
  }

  if (pump_1_authorized || pump_2_authorized) {
    rc = AHERN_PUMP_NOT_AUTHORIZED;
  }

  return rc;
}

struct file_operations pump_fops = {
  .owner          = THIS_MODULE,
  .read           = pump_read,
  .compat_ioctl   = pump_ioctl,
  .open           = pump_open,
  .release        = pump_release,
};

static void pump_setup_cdev(struct pump_dev *dev)
{
  int err;
    
  cdev_init(&dev->cdev, &pump_fops);
  dev->cdev.owner = THIS_MODULE;
  dev->cdev.ops = &pump_fops;
  err = cdev_add (&dev->cdev, MKDEV(pump_major, pump_minor), 1);
    
  if (err) {
    printk(KERN_NOTICE "Error %d adding ahern_pump!\n", err);
  }
}

static void __exit pump_exit(void)
{
  dev_t devno = MKDEV(pump_major, pump_minor);
    
  cdev_del(&pump_device.cdev);
    
  unregister_chrdev_region(devno, 1);
}

static int __init pump_init(void)
{
  int ret;
  dev_t dev = 0;
    
  ret = alloc_chrdev_region(&dev, pump_minor, 1, "pump");
  pump_major = MAJOR(dev);
    
  if (ret < 0) {
    printk(KERN_WARNING "ahern_pump: can't get major %d\n", pump_major);
    return ret;
  }
    
  pump_setup_cdev(&pump_device);

#ifdef PUMP_DEBUG
  printk(KERN_INFO "pump_init(): loaded Ahern Fuel Pump driver successfully!\n");
#endif

  return 0;
}

module_init(pump_init);
module_exit(pump_exit);

MODULE_AUTHOR("Jesse L. Zamora");
MODULE_LICENSE("GPL");
