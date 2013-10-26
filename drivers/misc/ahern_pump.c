 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <linux/ahern_pump.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>


#define PUMP_DEBUG

#define AHERN_PUMP_WORK_DELAY 1
#define AHERN_PUMP_BUF_SIZE   64

#define AHERN_PUMP_1_ENABLE   AT91_PIN_PC26
#define AHERN_PUMP_2_ENABLE   AT91_PIN_PC27
#define AHERN_PUMP_1_PULSE    AT91_PIN_PC28
#define AHERN_PUMP_2_PULSE    AT91_PIN_PC29

int pump_interface_authorize(struct pump_interface * );
int pump_interface_deauthorize(struct pump_interface * );

char pump_buf[AHERN_PUMP_BUF_SIZE];

static struct delayed_work pump_work;

struct pump_interface pump_1 = {
  .enable_pin = AHERN_PUMP_1_ENABLE,
  .pulse_pin = AHERN_PUMP_1_PULSE,
  .number = 1
};

struct pump_interface pump_2 = {
  .enable_pin = AHERN_PUMP_2_ENABLE,
  .pulse_pin = AHERN_PUMP_2_PULSE,
  .number = 2
};

int pump_authorized(void)
{
  return pump_1.authorized || pump_2.authorized;
}

int pump_release(struct inode *inode, struct file *filp)
{
  return 0;
}

int pump_open(struct inode *inode, struct file *filp)
{
  return 0;
}

long pump_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int ret = 0;
    
#if defined PUMP_DEBUG
  printk(KERN_INFO "pump.ioctl(): ");
#endif

  if (cmd == AHERN_PUMP_IOCTL_1_ENABLE || 
      cmd == AHERN_PUMP_IOCTL_2_ENABLE) {
    if (pump_authorized()) return AHERN_PUMP_INTERFACE_BUSY;
  }

  switch (cmd) {
  case AHERN_PUMP_IOCTL_1_ENABLE:
    ret = pump_interface_authorize(&pump_1);
    break;
  case AHERN_PUMP_IOCTL_1_DISABLE:
    ret = pump_interface_deauthorize(&pump_1);
    break;
  case AHERN_PUMP_IOCTL_2_ENABLE:
    ret = pump_interface_authorize(&pump_2);
    break;
  case AHERN_PUMP_IOCTL_2_DISABLE:
    ret = pump_interface_deauthorize(&pump_2);
    break;
  }

  return ret;
}

void pump_interface_init(struct pump_interface * pump)
{
  at91_set_gpio_output(pump->enable_pin, 1);
  at91_set_gpio_input(pump->pulse_pin, 1);

  pump->val = 1;
  pump->prev = 1;
  pump->authorized = false;
  pump->ticks = 0;
}

void pump_interface_handle(struct pump_interface * pump)
{
  pump->val = at91_get_gpio_value(pump->pulse_pin);
  if (pump->val != pump->prev) {
    if (pump->val == 0 && pump->authorized) {
#ifdef PUMP_DEBUG
    printk(KERN_INFO "Pump %d Tick\n", pump->number);
#endif
      pump->ticks++;
    }
    pump->prev = pump->val;
  }
}

int pump_interface_authorize(struct pump_interface * pump)
{
  if (pump->authorized) return AHERN_PUMP_INTERFACE_BUSY;

  at91_set_gpio_value(pump->enable_pin, 1);
  pump->authorized = true;
  pump->ticks = 0;

  //start delayed timer
  schedule_delayed_work(&pump_work, AHERN_PUMP_WORK_DELAY);

  return 0;
}

int pump_interface_deauthorize(struct pump_interface * pump)
{
  if (!pump->authorized) return AHERN_PUMP_NOT_AUTHORIZED;

  at91_set_gpio_value(pump->enable_pin, 0);
  pump->authorized = false;

  //cancel delayed timer
  cancel_delayed_work(&pump_work);

  return 0;
}

static void pump_task(struct work_struct *taskp)
{
  pump_interface_handle(&pump_1);
  pump_interface_handle(&pump_2);

  schedule_delayed_work(&pump_work, AHERN_PUMP_WORK_DELAY);
}

ssize_t pump_read(struct file *fi, char __user *buf, size_t count,
                    loff_t *f_pos)
{
  int rc = 0;

  memset(pump_buf, 0, sizeof(pump_buf) - 1);
  if (pump_1.authorized) {
    sprintf(pump_buf, "%d\n", pump_1.ticks);
    rc = strlen(pump_buf);
  } else if (pump_2.authorized) {
    sprintf(pump_buf, "%d\n", pump_2.ticks);
    rc = strlen(pump_buf);
  } else {
    rc = AHERN_PUMP_NOT_AUTHORIZED;
  }

  if (copy_to_user(buf, pump_buf, strlen(pump_buf))) {
    printk(KERN_ERR "pump_read(): Error copying data to userspace!\n");
    return -EFAULT;
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

static void __exit pump_exit(void)
{    
  cancel_delayed_work(&pump_work);

  unregister_chrdev(AHERN_PUMP_MAJOR_NUM, "pump");
}

static int __init pump_init(void)
{
  int ret;

  ret = register_chrdev(AHERN_PUMP_MAJOR_NUM, "pump", &pump_fops);
    
  if (ret < 0) {
    printk(KERN_WARNING "ahern_pump: error registering character device: %d\n", ret);
    return ret;
  }

#ifdef PUMP_DEBUG
  printk(KERN_INFO "pump_init(): loaded Ahern Fuel Pump driver successfully!\n");
#endif

  //start delayed timer
  INIT_DELAYED_WORK(&pump_work, pump_task);

  //initialize interfaces
  pump_interface_init(&pump_1);
  pump_interface_init(&pump_2);

  return 0;
}

module_init(pump_init);
module_exit(pump_exit);

MODULE_AUTHOR("Jesse L. Zamora");
MODULE_LICENSE("GPL");
