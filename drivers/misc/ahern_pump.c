 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <linux/ahern_pump.h>

#include <asm/uaccess.h>

#ifndef PUMP_DESKTOP_DEBUG
  #include <asm/io.h>
  #include <asm/gpio.h>

  #define AHERN_PUMP_1_ENABLE   AT91_PIN_PC26
  #define AHERN_PUMP_2_ENABLE   AT91_PIN_PC27
  #define AHERN_PUMP_1_PULSE    AT91_PIN_PC28
  #define AHERN_PUMP_2_PULSE    AT91_PIN_PC29
#endif

#define AHERN_PUMP_WORK_DELAY 1
#define AHERN_PUMP_BUF_SIZE   64

#define AHERN_PUMP_MAJOR      55
#define AHERN_PUMP_MINOR      1
#define AHERN_PUMP_NUM        2

struct pump_interface {
  int enable_pin;
  int pulse_pin;

  int val;
  int prev;

  int authorized;

  uint32_t ticks;

  struct cdev cdev;
};

int pump_major = AHERN_PUMP_MAJOR;
int pump_minor = AHERN_PUMP_MINOR;
int pump_num = AHERN_PUMP_NUM;

module_param(pump_major, int, S_IRUGO);
module_param(pump_minor, int, S_IRUGO);
module_param(pump_num, int, S_IRUGO);

int pump_interface_authorize(struct pump_interface * );
int pump_interface_deauthorize(struct pump_interface * );

struct pump_interface * pumps;

char pump_buf[AHERN_PUMP_BUF_SIZE];

static struct delayed_work pump_work;

int pump_release(struct inode *inode, struct file *filp)
{
  return 0;
}

int pump_open(struct inode *inode, struct file *filp)
{
  struct pump_interface *pump; /* device information */

	pump = container_of(inode->i_cdev, struct pump_interface, cdev);
  filp->private_data = pump;

  return 0;
}

long pump_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int ret = 0;

  struct pump_interface * pump = filp->private_data;

  if (pump) {
    switch (cmd) {
    case AHERN_PUMP_IOCTL_AUTHORIZE:
      pump_interface_authorize(pump);
      break;
    case AHERN_PUMP_IOCTL_DEAUTHORIZE:
      pump_interface_deauthorize(pump);
      break;
    }
  } else {
    ret = -ENODEV;
  }

  return ret;
}

void pump_interface_handle(struct pump_interface * pump)
{
#ifndef PUMP_DESKTOP_DEBUG
  pump->val = at91_get_gpio_value(pump->pulse_pin);
#else
  pump->val = !pump->val;
#endif
  if (pump->val != pump->prev) {
    if (pump->val == 0 && pump->authorized) {
      pump->ticks++;
    }
    pump->prev = pump->val;
  }
}

int pump_interface_authorize(struct pump_interface * pump)
{
  if (pump->authorized) return -EBUSY;

#ifndef PUMP_DESKTOP_DEBUG
  at91_set_gpio_value(pump->enable_pin, 1);
#endif

  pump->ticks = 0;
  pump->authorized = true;

  return 0;
}

int pump_interface_deauthorize(struct pump_interface * pump)
{
  if (!pump->authorized) return -EACCES;

#ifndef PUMP_DESKTOP_DEBUG
  at91_set_gpio_value(pump->enable_pin, 0);
#endif

  pump->authorized = false;

  return 0;
}

static void pump_task(struct work_struct *taskp)
{
  int i;

  for (i = 0; i < pump_num; i++) {
    pump_interface_handle(&pumps[i]);
  }

  schedule_delayed_work(&pump_work, AHERN_PUMP_WORK_DELAY);
}

ssize_t pump_read(struct file *filp, char __user *buf, size_t count,
                    loff_t *f_pos)
{
  int rc = 0;
  struct pump_interface * pump = filp->private_data;

  memset(pump_buf, 0, sizeof(pump_buf) - 1);

  if (pump) {
    if (!pump->authorized) {
      rc = -EACCES;
    } else {
      if (pump->ticks > 0) {
        sprintf(pump_buf, "%u\n", pump->ticks);

        if (strlen(pump_buf) > 0) {
          if (copy_to_user(buf, pump_buf, strlen(pump_buf))) {
            printk(KERN_ERR "pump_read(): Error copying data to userspace!\n");
            return -EFAULT;
          }
          rc = strlen(pump_buf);
        }
      }
    }
  } else {
    rc = -ENODEV;
  }

  return rc;
}

struct file_operations pump_fops = {
  .owner          = THIS_MODULE,
  .read           = pump_read,
  .unlocked_ioctl = pump_ioctl,
  .open           = pump_open,
  .release        = pump_release,
};

void pump_cleanup_module(void)
{
	int i;
	dev_t devno = MKDEV(pump_major, pump_minor);

	if (pumps) {
		for (i = 0; i < pump_num; i++) {
			cdev_del(&pumps[i].cdev);
		}
		kfree(pumps);
	}

	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(devno, pump_num);
}

void pump_interface_init(struct pump_interface * pump, int index)
{
  int err, devno = MKDEV(pump_major, pump_minor + index);

  //init CDEV interface
  cdev_init(&pump->cdev, &pump_fops);
  pump->cdev.owner = THIS_MODULE;
  pump->cdev.ops = &pump_fops;
  err = cdev_add(&pump->cdev, devno, 1);
  if (err) {
    printk(KERN_NOTICE "Error %d adding pump%d\n", err, index);
  }

  printk(KERN_INFO "ahern_pump: added pump interface %d\n", pump_minor + index);

  //init IO
#ifndef PUMP_DESKTOP_DEBUG
  at91_set_gpio_output(pump->enable_pin, 0);
  at91_set_gpio_input(pump->pulse_pin, 1);
#endif

  //init variables
  pump->authorized = false;
  pump->val = 1;
  pump->prev = 1;
  pump->ticks = 0;
}

static void __exit pump_exit(void)
{    
  cancel_delayed_work(&pump_work);

  pump_cleanup_module();
}

static int __init pump_init(void)
{
  int i, ret;
  dev_t dev = 0;

	if (pump_major > 0) {
		dev = MKDEV(pump_major, pump_minor);
		ret = register_chrdev_region(dev, pump_num, "pump");
  } else {
    ret = alloc_chrdev_region(&dev, pump_minor, pump_num, "pump");
    pump_major = MAJOR(dev);
  }
    
  if (ret < 0) {
    printk(KERN_WARNING "ahern_pump: can't get major %d\n", pump_major);
    return ret;
  }

  printk(KERN_INFO "ahern_pump: registered major %d\n", pump_major);

  //allocate memory for pumps
  pumps = kmalloc(sizeof(struct pump_interface) * pump_num, GFP_KERNEL);
  if (!pumps) {
    ret = -ENOMEM;
    goto fail;
  }

  //TODO: initialize default IO
#ifndef PUMP_DESKTOP_DEBUG
  pumps[0].enable_pin = AHERN_PUMP_1_ENABLE;
  pumps[0].pulse_pin = AHERN_PUMP_1_PULSE;

  pumps[1].enable_pin = AHERN_PUMP_2_ENABLE;
  pumps[1].pulse_pin = AHERN_PUMP_2_PULSE;
#endif

  //init interfaces
  for (i = 0; i < pump_num; i++) {
    pump_interface_init(&pumps[i], i);
  }

#ifdef PUMP_DEBUG
  printk(KERN_INFO "pump_init(): loaded Ahern Fuel Pump driver successfully!\n");
#endif

  //start delayed timer
  INIT_DELAYED_WORK(&pump_work, pump_task);

  //TODO: REMOVE
  //pump_interface_authorize(&pumps[0]);
  schedule_delayed_work(&pump_work, AHERN_PUMP_WORK_DELAY);
  //END REMOVE
  
  return 0;

fail:
  pump_cleanup_module();
  return ret;
}

module_init(pump_init);
module_exit(pump_exit);

MODULE_AUTHOR("Jesse L. Zamora");
MODULE_LICENSE("GPL");
