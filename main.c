/********************************************************
 *      mini6410                 STM8
 * -------------------       ------------
 * |      (GPE1) rst |>----->| rst      |
 * |  (GPE2) swim_in |<----+-| swim     |
 * | (GPE3) swim_out |>---/  |          |
 * -------------------       ------------
 *******************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <plat/regs-timer.h>
#include <mach/regs-irq.h>
#include <asm/mach/time.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>

#include <plat/gpio-cfg.h>
#include <mach/gpio-bank-e.h>

#include "stm8_swim.h"


#define DEVICE_NAME     "swim"

#define SWIM_IOCTL_RESET    1


static struct semaphore lock;
static swim_method *swim;
static swim_input input;


/* 
 * pclk = 66.5MHz, min valid delay = 15ns, and max deviation = 15ns
 */
static void s3c6410_ndelay(unsigned int ns)
{
	unsigned long tcon;
	unsigned long tcnt;
	unsigned long tcfg1;
	unsigned long tcfg0;
	unsigned long tint_cstat;

	struct clk *clk_p;
	unsigned long pclk;

	tcon = __raw_readl(S3C_TCON);
	tcfg1 = __raw_readl(S3C_TCFG1);
	tcfg0 = __raw_readl(S3C_TCFG0);
	tint_cstat = __raw_readl(S3C_TINT_CSTAT);

	//prescaler = 0
	tcfg0 &= ~S3C_TCFG_PRESCALER0_MASK;
	//tcfg0 |= (50 - 1); // add

	//mux = 1/1
	tcfg1 &= ~S3C_TCFG1_MUX0_MASK;
	tcfg1 |= S3C_TCFG1_MUX0_DIV1; // add

	__raw_writel(tcfg1, S3C_TCFG1);
	__raw_writel(tcfg0, S3C_TCFG0);

	clk_p = clk_get(NULL, "pclk");
	pclk  = clk_get_rate(clk_p);
    //printk("pclk = %u\n", (unsigned int)pclk);
	tcnt  = ns * (pclk / 100000) / 10000;
    //printk("tcnt = %u\n", (unsigned int)tcnt);	
	__raw_writel(tcnt, S3C_TCNTB(0));
	//__raw_writel(tcnt/2, S3C_TCMPB(0)); // add

	tint_cstat &= ~1; 
    tint_cstat |= (1<<5);
	__raw_writel(tint_cstat, S3C_TINT_CSTAT);

    tcon &= ~0x1f;
    tcon |= 0xb;		//update TCNTB0, start timer 0
    __raw_writel(tcon, S3C_TCON);

    tcon &= ~2;			//clear manual update bit
    __raw_writel(tcon, S3C_TCON);

    //printk("S3C_TINT_TCNTO = %d\n", __raw_readl(S3C_TIMERREG(0x14)));
    //printk("S3C_TINT_TCNTO = %d\n", __raw_readl(S3C_TIMERREG(0x14)));

    //printk("before S3C_TINT_CSTAT 0x%X\n", __raw_readl(S3C_TINT_CSTAT));
    while (!(__raw_readl(S3C_TINT_CSTAT) & (1<<5)));
    //printk("after S3C_TINT_CSTAT 0x%X\n", __raw_readl(S3C_TINT_CSTAT));
}


static void swim_init_io(void)
{
    unsigned long tmp;

    // GPE1 -> RST (open drain); GPE2 -> SWIM_OUT (open drain); GPE3 -> SWIM_IN (floating)
    tmp = readl(S3C64XX_GPECON);
    tmp &= ~(0xF << 4);
    tmp |= 1 << 4;
    tmp &= ~(0xF << 8);
    tmp |= 1 << 8;
    tmp &= ~(0xF << 12);
    writel(tmp, S3C64XX_GPECON);
    
    tmp = readl(S3C64XX_GPEPUD);
    tmp &= ~(0x3F << 2);
    writel(tmp, S3C64XX_GPEPUD);

    tmp = readl(S3C64XX_GPEDAT);
    tmp |= (0x3 << 1);
    writel(tmp, S3C64XX_GPEDAT);
}

void swim_io_set(io_name io, swim_level level)
{
    unsigned long tmp = readl(S3C64XX_GPEDAT);

    switch (io)
    {
        case RST:
            if (level == HIGH)
            {
                tmp |= 1 << 1;
            }
            else
            {
                tmp &= ~(1 << 1);
            } 
            break;
        case SWIM:
            if (level == HIGH)
            {
                tmp |= 1 << 2;
            }
            else
            {
                tmp &= ~(1 << 2);
            } 
            break;
        default:
            printk("No define this io: %d!\n", io);
            break;
    }
    
    writel(tmp, S3C64XX_GPEDAT);
}

swim_level swim_io_get(io_name io)
{
    return (readl(S3C64XX_GPEDAT) & (1 << 3)) ? HIGH : LOW;
}

#if 0 // io test
static void swim_io_test(void)
{
    swim_io_set(RST, HIGH);
    printk("RST(GPE1) => HIGH\n");
    mdelay(5000);
    swim_io_set(RST, LOW);
    printk("RST(GPE1) => LOW\n");
    mdelay(5000);
    swim_io_set(SWIM, HIGH);
    printk("SWIM(GPE2) => HIGH\n");
    mdelay(5000);
    swim_io_set(SWIM, LOW);
    printk("SWIM(GPE2) => LOW\n");
    mdelay(5000);
    while (swim_io_get(SWIM) == HIGH);
    printk("SWIM(GPE3) <= LOW\n");
    while (swim_io_get(SWIM) == LOW);
    printk("SWIM(GPE3) <= HIGH\n");
}
#endif

static ssize_t s3c64xx_swim_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    swim_ret ret;
    unsigned int addr = *ppos;
    unsigned char *kbuf = NULL;
    
    kbuf = kmalloc(count, GFP_KERNEL);
    if (!kbuf)
    {
        printk("kmalloc %d bytes fail!\n", count);
        return 0;
    }

    printk("swim read addr = 0x%x, size = 0x%x\n", addr, count);

    local_irq_disable();
    ret = swim->read(addr, kbuf, count);
    if (ret)
    {
        printk("resd fail! ret = %d, return_line = %d\n", ret, return_line);
    }
    local_irq_enable();

    if (copy_to_user(buf, kbuf, count))
    {
        printk("copy_to_user fail!\n");
        return 0;
    }

    kfree(kbuf);

    return count;
}

static ssize_t s3c64xx_swim_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    swim_ret ret;
    unsigned int addr = *ppos;
    unsigned char *kbuf = NULL;
    
    kbuf = kmalloc(count, GFP_KERNEL);
    if (!kbuf)
    {
        printk("kmalloc %d bytes fail!\n", count);
        return 0;
    }

    if (copy_from_user(kbuf, buf, count))
    {
        printk("copy_from_user fail!\n");
        return 0;
    }

    printk("swim write addr = 0x%x, size = 0x%x\n", addr, count);

    local_irq_disable();
    ret = swim->write(addr, kbuf, count);
    if (ret)
    {
        printk("write fail! ret = %d, return_line = %d\n", ret, return_line);
    }
    local_irq_enable();

    kfree(kbuf);

    return count;
}

static loff_t s3c64xx_swim_seek(struct file *file, loff_t offset, int orig)
{
    loff_t ret;

    switch (orig) 
    {
        case SEEK_CUR:
            offset += file->f_pos;
        case SEEK_SET:
            file->f_pos = offset;
            ret = file->f_pos;
            break;
        default:
            ret = -EINVAL;
    }

    return ret;
}

static long s3c64xx_swim_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    swim_ret ret = 0;

    switch (cmd)
    {
        case SWIM_IOCTL_RESET:
            local_irq_disable();
            ret = swim->reset();
            if (ret)
            {
                printk("reset fail! ret = %d, return_line = %d\n", ret, return_line);
            }
            local_irq_enable();
            break;
        default:
            break;
    }

    return ret;
}

static int s3c64xx_swim_open(struct inode *inode, struct file *file)
{
    swim_ret ret;

	if (down_trylock(&lock))
    {
        return -EBUSY;
    }

    file->f_pos = 0;

    local_irq_disable();
    ret = swim->entry();
    if (ret)
    {
        printk("entry fail! ret = %d, return_line = %d\n", ret, return_line);
    }
    local_irq_enable();

    return 0;
}

static int s3c64xx_swim_close(struct inode *inode, struct file *file)
{
    swim_ret ret;

    local_irq_disable();
    ret = swim->reset();
    if (ret)
    {
        printk("reset fail! ret = %d, return_line = %d\n", ret, return_line);
    }
    local_irq_enable();

	up(&lock);

	return 0;
}

static struct file_operations dev_fops = {
    .owner			= THIS_MODULE,
    .open			= s3c64xx_swim_open,
    .release		= s3c64xx_swim_close, 
    .write          = s3c64xx_swim_write,
    .read           = s3c64xx_swim_read,
    .llseek         = s3c64xx_swim_seek,
    .unlocked_ioctl = s3c64xx_swim_ioctl,
};

static struct miscdevice misc = {
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};


static int __init dev_init(void)
{
	swim_ret ret;
    struct clk *timer_clock;

	init_MUTEX(&lock);
	ret = misc_register(&misc);

    timer_clock = clk_get(NULL, "timers");
    if (!timer_clock) {
        printk(KERN_ERR "failed to get adc clock source\n");
        return -ENOENT;
    }
    clk_enable(timer_clock);

    swim_init_io();

    input.ndelay = s3c6410_ndelay;
    input.io_set = swim_io_set;
    input.io_get = swim_io_get;
    input.private = NULL;

    swim = swim_register(&input);
    if (!swim)
    {
        printk("swim = NULL!\n");
        return -1;
    }

    printk (DEVICE_NAME"\tinitialized\n");
    return ret;
}

static void __exit dev_exit(void)
{
    swim_unregister(swim);
	misc_deregister(&misc);
}


module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cole3");
MODULE_DESCRIPTION("S3C6410 SWIM Driver");
