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

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>

#include <plat/gpio-cfg.h>
#include <mach/gpio-bank-e.h>
#include <mach/gpio-bank-f.h>
#include <mach/gpio-bank-k.h>

#define DEVICE_NAME     "swim"

#define swim_IOCTL_SET_FREQ		1
#define swim_IOCTL_STOP			0

static struct semaphore lock;

/* freq:  pclk/50/16/65536 ~ pclk/50/16 
  * if pclk = 50MHz, freq is 1Hz to 62500Hz
  * human ear : 20Hz~ 20000Hz
  */
static void swim_ndelay(unsigned long ns)
{
	unsigned long tcon;
	unsigned long tcnt;
	unsigned long tcfg1;
	unsigned long tcfg0;
	unsigned long tint_cstat;

	struct clk *clk_p;
	unsigned long pclk;

	unsigned tmp;

#if 0 // PWM IO
	tmp = readl(S3C64XX_GPFCON);
	tmp &= ~(0x3U << 28);
	tmp |=  (0x2U << 28);
	writel(tmp, S3C64XX_GPFCON);
#endif

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

static swim_udelay(unsigned int us)
{
    while (us--)
    {
        swim_ndelay(1000);
    }
}

static swim_mdelay(unsigned int ms)
{
    while (ms--)
    {
        swim_ndelay(1000000);
    }
}

static void swim_init_io(void)
{
    
}

void swim_stop( void )
{
	unsigned tmp;
	tmp = readl(S3C64XX_GPFCON);
	tmp &= ~(0x3U << 28);
	writel(tmp, S3C64XX_GPFCON);
}

static int s3c64xx_swim_open(struct inode *inode, struct file *file)
{
	if (!down_trylock(&lock))
		return 0;
	else
		return -EBUSY;
}


static int s3c64xx_swim_close(struct inode *inode, struct file *file)
{
	up(&lock);
	return 0;
}


static long s3c64xx_swim_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
		case swim_IOCTL_SET_FREQ:
			if (arg == 0)
				return -EINVAL;
			//swim_set_freq(arg);
			break;

		case swim_IOCTL_STOP:
		default:
			swim_stop();
			break;
	}

	return 0;
}


static struct file_operations dev_fops = {
    .owner			= THIS_MODULE,
    .open			= s3c64xx_swim_open,
    .release		= s3c64xx_swim_close, 
    .unlocked_ioctl	= s3c64xx_swim_ioctl,
};

static struct miscdevice misc = {
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};


static int __init dev_init(void)
{
	int ret;
    struct clk   *timer_clock;

	init_MUTEX(&lock);
	ret = misc_register(&misc);

    timer_clock = clk_get(NULL, "timers");
    if (!timer_clock) {
        printk(KERN_ERR "failed to get adc clock source\n");
        return -ENOENT;
    }
    clk_enable(timer_clock);


    local_irq_disable();
    printk("start delay\n");
    swim_ndelay(1250);
    printk("end delay\n");
    swim_mdelay(30000);
    printk("end delay\n");
    local_irq_enable();

	printk (DEVICE_NAME"\tinitialized\n");
    	return ret;
}

static void __exit dev_exit(void)
{
	misc_deregister(&misc);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cole3");
MODULE_DESCRIPTION("S3C6410 SWIM Driver");
