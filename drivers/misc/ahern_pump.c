 
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
bool pump_interface_enabled(struct pump_interface * pump);

char pump_buf[AHERN_PUMP_BUF_SIZE];

static struct delayed_work pump_work;

struct pump_interface pump_1 = {
  .enable_pin = AHERN_PUMP_1_ENABLE,
  .pulse_pin = AHERN_PUMP_1_PULSE
};

struct pump_interface pump_2 = {
  .enable_pin = AHERN_PUMP_2_ENABLE,
  .pulse_pin = AHERN_PUMP_2_PULSE
};

int pump_release(struct inode *inode, struct file *filp)
{
  return 0;
}

int pump_open(struct inode *inode, struct file *filp)
{
  return 0;
}

struct pump_interface * pump_interface_select(int number)
{
  switch (number) {
  case AHERN_PUMP_1:
    return &pump_1;
  case AHERN_PUMP_2:
    return &pump_2;  
  }
  
  return NULL;
}

long pump_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int ret = 0;

  struct pump_interface * pump;

  switch (cmd) {
  case AHERN_PUMP_IOCTL_AUTHORIZE:
    pump = pump_interface_select(arg);
    if (pump == NULL) {
      ret = -ENODEV;
    } else {
      ret = pump_interface_authorize(pump);
    }
    break;
  case AHERN_PUMP_IOCTL_DEAUTHORIZE:
    pump = pump_interface_select(arg);
    if (pump == NULL) {
      ret = -ENODEV;
    } else {
      ret = pump_interface_deauthorize(pump);
    }
    break;
  default:
    ret = -EINVAL;
    break;
  }

  return ret;
}

void pump_interface_init(struct pump_interface * pump)
{
  at91_set_gpio_output(pump->enable_pin, 0);
  at91_set_gpio_input(pump->pulse_pin, 1);

  pump->val = 1;
  pump->prev = 1;
  pump->ticks = 0;
}

void pump_interface_handle(struct pump_interface * pump)
{
  bool authorized = pump_interface_enabled(pump);

  pump->val = at91_get_gpio_value(pump->pulse_pin);
  if (pump->val != pump->prev) {
    if (pump->val == 0 && authorized) {
      pump->ticks++;
    }
    pump->prev = pump->val;
  }
}

bool pump_interface_enabled(struct pump_interface * pump)
{
  return at91_get_gpio_value(pump->enable_pin);
}

int pump_interface_authorize(struct pump_interface * pump)
{
  if (pump_interface_enabled(pump)) return -EBUSY;

  at91_set_gpio_value(pump->enable_pin, 1);
  pump->ticks = 0;

  //start delayed timer
  schedule_delayed_work(&pump_work, AHERN_PUMP_WORK_DELAY);

  return 0;
}

int pump_interface_deauthorize(struct pump_interface * pump)
{
  if (!pump_interface_enabled(pump)) return -EACCES;

  at91_set_gpio_value(pump->enable_pin, 0);

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
  uint32_t ticks = 0;

  memset(pump_buf, 0, sizeof(pump_buf) - 1);

  if (pump_interface_enabled(&pump_1)) {
    ticks = pump_1.ticks;
  } else if (pump_interface_enabled(&pump_2)) {
    ticks = pump_2.ticks;
  } else {
    rc = -EACCES;
  }

  if (ticks > 0) {
    sprintf(pump_buf, "%u\n", pump_1.ticks);
    rc = strlen(pump_buf);
  }

  if (strlen(pump_buf) > 0) {
    if (copy_to_user(buf, pump_buf, strlen(pump_buf))) {
      printk(KERN_ERR "pump_read(): Error copying data to userspace!\n");
      return -EFAULT;
    }
    rc = strlen(pump_buf);
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
