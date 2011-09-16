/*
 *	MultiPorts 1/2/4/8/32 device driver for linux(kernel ver. 2.4.2.
 */
static char *multi_version = "3.0";
static char *multi_revdate = "2009-06-22"; // md by shlee

#include <linux/config.h>

#define SERIAL_DO_RESTART

#define RS_STROBE_TIME (10*HZ)
#define _ISR_PASS_LIMIT 256

#if defined(__i386__) && (defined(CONFIG_M386) || defined(CONFIG_M486))
#define SERIAL_INLINE
#endif
  
/*
 * End of serial driver configuration section.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_reg.h>
#include <asm/serial.h>
#define LOCAL_VERSTRING ""

#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
//#include <linux/malloc.h> 
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/pci.h>

//#include "multi.h"

///////////////////////////////////////////////////////////////////////////////
/* 
 * multi.h
 */
#include <linux/termios.h>
#include <linux/tqueue.h>

#include "mp_register.h"
#include "multiport.h"

#define MPORT_MAGIC	0x525070
#define SSTATE_MAGIC 0x5302

/*
 * The size of the serial xmit buffer is 1 page, or 4096 bytes
 */
#define MULTI_XMIT_SIZE 4096 

#define MP_EVENT_WRITE_WAKEUP	0

#define MP_BH	20
#define MP_TIMER	20
#define MAX_MP_DEV	8

#define PCI_VENDOR_ID_MULTIPORT	0x14A1
#define PCI_DEVICE_ID_MP1		0x4d01
#define PCI_DEVICE_ID_MP2		0x4d02
#define PCI_DEVICE_ID_MP4		0x0004
#define PCI_DEVICE_ID_MP8		0x0008
#define PCI_DEVICE_ID_MP32		0x0032

#define ITR_MASK_REG	0x0C
#define POLL_REG	0x10

#define TTY_MP_MAJOR	54
#define CUA_MP_MAJOR	55

struct mp_device_t;

struct sb_serial_state {
	int	magic;
	int	baud_base;
	unsigned long	port;
	int	irq;
	int	flags;
	int	hub6;
	int	type;
	int	line;
	int	revision;	/* Chip revision (950) */
	int	xmit_fifo_size;
	int	custom_divisor;
	int	count;
	u8	*iomem_base;
	u16	iomem_reg_shift;
	unsigned short	close_delay;
	unsigned short	closing_wait; /* time to wait before closing */
	struct async_icount	icount;	
	struct termios		normal_termios;
	struct termios		callout_termios;
	int	io_type;
	struct sb_async_struct *info;
	struct pci_dev	*dev;
	struct mp_device_t	*device;
	unsigned long	interface_config_addr;
	unsigned long	option_base_addr;
	unsigned char interface;
};

struct sb_async_struct {
	int			magic;
	unsigned long		port;
	int			hub6;
	int			flags;
	int			xmit_fifo_size;
	struct sb_serial_state	*state;
	struct tty_struct 	*tty;
	int			read_status_mask;
	int			ignore_status_mask;
	int			timeout;
	int			quot;
	int			x_char;	/* xon/xoff character */
	int			close_delay;
	unsigned short		closing_wait;
	unsigned short		closing_wait2;
	int			IER; 	/* Interrupt Enable Register */
	int			MCR; 	/* Modem control register */
	int			LCR; 	/* Line control register */
	int			ACR;	 /* 16950 Additional Control Reg. */
	unsigned long		event;
	unsigned long		last_active;
	int			line;
	int			blocked_open; /* # of blocked opens */
	long			session; /* Session of opening process */
	long			pgrp; /* pgrp of opening process */
 	struct circ_buf		xmit;
 	spinlock_t		xmit_lock;
 	spinlock_t		irq_spinlock;
	u8			*iomem_base;
	u16			iomem_reg_shift;
	int			io_type;
	struct tq_struct	tqueue;
#ifdef DECLARE_WAITQUEUE
	wait_queue_head_t	open_wait;
	wait_queue_head_t	close_wait;
	wait_queue_head_t	delta_msr_wait;
#else	
	struct wait_queue	*open_wait;
	struct wait_queue	*close_wait;
	struct wait_queue	*delta_msr_wait;
#endif	
	struct sb_async_struct	*next_port; /* For the linked list */
	struct sb_async_struct	*prev_port;
};

/////////////////////////////////////////////////////////////////////////////

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/bitops.h>

#ifdef SERIAL_INLINE
#define _INLINE_ inline
#else
#define _INLINE_
#endif

static DECLARE_TASK_QUEUE(tq_multi);

static struct tty_driver multi_driver, callout_driver;
static int multi_refcount;
static int NR_DEV=0;
static int NR_PORTS=0;
static unsigned char mp_char_buf[N_TTY_BUF_SIZE];
static unsigned char mp_flag_buf[N_TTY_BUF_SIZE];


static struct timer_list multi_timer;

/* serial subtype definitions */
#ifndef SERIAL_TYPE_NORMAL
#define SERIAL_TYPE_NORMAL	1
#define SERIAL_TYPE_CALLOUT	2
#endif

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256

/*
 * IRQ_timeout		- How long the timeout should be for each IRQ
 * 				should be after the IRQ has been active.
 */

static struct sb_async_struct *IRQ_ports[NR_IRQS];
static int IRQ_timeout[NR_IRQS];

static unsigned detect_uart_irq (struct sb_serial_state * state);
static void autoconfig(struct sb_serial_state * state);
static void change_speed(struct sb_async_struct *info, struct termios *old);
static void mp_wait_until_sent(struct tty_struct *tty, int timeout);
// static int get_num_ports(unsigned int *value);

/*
 * Here we define the default xmit fifo size used for each type of
 * UART
 */
static const struct serial_uart_config uart_config[] = {
	{ "unknown",    1,  0 },
	{ "16550A", 16, UART_CLEAR_FIFO | UART_USE_FIFO },
	{ "SB16C1050",    128,    UART_CLEAR_FIFO | UART_USE_FIFO | UART_STARTECH },
};

struct mp_device_t {
	unsigned short	device_id;
	unsigned char	revision;
	char			*name;
	unsigned long	uart_access_addr;
	unsigned long	option_reg_addr;
	unsigned long	reserved_addr[4];
	int				irq;
	int				nr_ports;
};

static struct mp_device_t mp_devs[MAX_MP_DEV];

typedef struct mppcibrd {
	char			*name;
	unsigned short	vendor_id;
	unsigned short	device_id;
} mppcibrd_t;

static mppcibrd_t mp_pciboards[] = {
	{ "Multi-1 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP1} ,
	{ "Multi-2 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP2} ,
	{ "Multi-4 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP4} ,
	{ "Multi-8 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP8} ,
	{ "Multi-32 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP32} ,
};

static int mp_nrpcibrds = sizeof(mp_pciboards)/sizeof(mppcibrd_t);


#ifndef PREPARE_FUNC
#define PREPARE_FUNC(dev)  (dev->prepare)
#define ACTIVATE_FUNC(dev)  (dev->activate)
#define DEACTIVATE_FUNC(dev)  (dev->deactivate)
#endif

#define HIGH_BITS_OFFSET ((sizeof(long)-sizeof(int))*8)


static struct sb_serial_state *mp_table;
static struct tty_struct **multi_table;
static struct termios **multi_termios;
static struct termios **multi_termios_locked;

/*
 * tmp_buf is used as a temporary buffer by multi_write.  We need to
 * lock it in case the copy_from_user blocks while swapping in a page,
 * and some other program tries to do a serial write at the same time.
 * Since the lock will only come under contention when the system is
 * swapping and available memory is low, it makes sense to share one
 * buffer across all the serial ports, since it significantly saves
 * memory if large numbers of serial ports are open.
 */
static unsigned char *tmp_buf;
#ifdef DECLARE_MUTEX
static DECLARE_MUTEX(tmp_buf_sem);
#else
static struct semaphore tmp_buf_sem = MUTEX;
#endif


static _INLINE_ unsigned int multi_in(struct sb_async_struct *info, int offset)
{
		return inb(info->port + offset);
}

static _INLINE_ void multi_out(struct sb_async_struct *info, int offset,
				int value)
{
		outb(value, info->port+offset);
}

static _INLINE_ unsigned int read_option_register(struct sb_serial_state *state, int offset)
{
	return inb(state->option_base_addr + offset);
}

static _INLINE_ void write_option_register(struct sb_serial_state *state, int offset, int value)
{
	outb(value, state->option_base_addr + offset);
}

/* SB16C1050 read functions by John Lee */

static int sb1054_get_register(struct sb_async_struct * info, int page, int reg)
{
	int ret = 0;
	unsigned int lcr = 0;
	unsigned int mcr = 0;
	unsigned int tmp = 0;

	if( page <= 0)
	{
		printk(" page 0 can not use this fuction\n");
		return -1;
	}

	switch(page)
	{
		case 1:
			lcr = SB105X_GET_LCR(info);
			tmp = lcr | SB105X_LCR_DLAB;
			SB105X_PUT_LCR(info, tmp);

			tmp = SB105X_GET_LCR(info);

			ret = SB105X_GET_REG(info,reg);
			SB105X_PUT_LCR(info,lcr);
			break;
		case 2:
			mcr = SB105X_GET_MCR(info);
			tmp = mcr | SB105X_MCR_P2S;
			SB105X_PUT_MCR(info,tmp);

			ret = SB105X_GET_REG(info,reg);

			SB105X_PUT_MCR(info,mcr);
			break;
		case 3:
			lcr = SB105X_GET_LCR(info);
			tmp = lcr | SB105X_LCR_BF;
			SB105X_PUT_LCR(info,tmp);
			SB105X_PUT_REG(info,SB105X_PSR,SB105X_PSR_P3KEY);

			ret = SB105X_GET_REG(info,reg);

			SB105X_PUT_LCR(info,lcr);
			break;
		case 4:
			lcr = SB105X_GET_LCR(info);
			tmp = lcr | SB105X_LCR_BF;
			SB105X_PUT_LCR(info,tmp);
			SB105X_PUT_REG(info,SB105X_PSR,SB105X_PSR_P4KEY);

			ret = SB105X_GET_REG(info,reg);

			SB105X_PUT_LCR(info,lcr);
			break;
		default:
			printk(" error invalid page number \n");
			return -1;
	}

	return ret;
}

static int sb1054_set_register(struct sb_async_struct * info, int page, int reg, int value)
{  
	int lcr = 0;
	int mcr = 0;
	int ret = 0;

	if( page <= 0)
	{
		printk(" page 0 can not use this fuction\n");
		return -1;
	}
	switch(page)
	{
		case 1:
			lcr = SB105X_GET_LCR(info);
			SB105X_PUT_LCR(info, lcr | SB105X_LCR_DLAB);

			SB105X_PUT_REG(info,reg,value);

			SB105X_PUT_LCR(info, lcr);
			ret = 0;
			break;
		case 2:
			mcr = SB105X_GET_MCR(info);
			SB105X_PUT_MCR(info, mcr | SB105X_MCR_P2S);

			SB105X_PUT_REG(info,reg,value);

			SB105X_PUT_MCR(info, mcr);
			ret = 0;
			break;
		case 3:
			lcr = SB105X_GET_LCR(info);
			SB105X_PUT_LCR(info, lcr | SB105X_LCR_BF);
			SB105X_PUT_PSR(info, SB105X_PSR_P3KEY);

			SB105X_PUT_REG(info,reg,value);

			SB105X_PUT_LCR(info, lcr);
			ret = 0;
			break;
		case 4:
			lcr = SB105X_GET_LCR(info);
			SB105X_PUT_LCR(info, lcr | SB105X_LCR_BF);
			SB105X_PUT_PSR(info, SB105X_PSR_P4KEY);

			SB105X_PUT_REG(info,reg,value);

			SB105X_PUT_LCR(info, lcr);
			ret = 0;
			break;
		default:
			printk(" error invalid page number \n");
			return -1;
	}

	return ret;
}

//// additional setting functions ////

static int set_deep_fifo(struct sb_async_struct * info, int status)
{
	int afr_status = 0;
	afr_status = sb1054_get_register(info, PAGE_4, SB105X_AFR);

	if(status == ENABLE)
	{
		afr_status |= SB105X_AFR_AFEN;
	}
	else
	{
		afr_status &= ~SB105X_AFR_AFEN;
	}
	
	sb1054_set_register(info,PAGE_4,SB105X_AFR,afr_status);
	
	afr_status = sb1054_get_register(info, PAGE_4, SB105X_AFR);
	
	return afr_status;
}

static int set_auto_rts(struct sb_async_struct *info, int status)
{
	int efr_status = 0;
	efr_status = sb1054_get_register(info, PAGE_3, SB105X_EFR);

	if(status == ENABLE)
	{
		efr_status |= SB105X_EFR_ARTS;
	}
	else
	{
		efr_status &= ~SB105X_EFR_ARTS;
	}
	sb1054_set_register(info,PAGE_3,SB105X_EFR,efr_status);
	
	efr_status = sb1054_get_register(info, PAGE_3, SB105X_EFR);
	
	return efr_status;
}

static int set_auto_cts(struct sb_async_struct *info, int status)
{
	int efr_status = 0;
	efr_status = sb1054_get_register(info, PAGE_3, SB105X_EFR);

	if(status == ENABLE)
	{
		efr_status |= SB105X_EFR_ACTS;
	}
	else
	{
		efr_status &= ~SB105X_EFR_ACTS;
	}
	sb1054_set_register(info,PAGE_3,SB105X_EFR,efr_status);
	
	efr_status = sb1054_get_register(info, PAGE_3, SB105X_EFR);
	
	return efr_status;
	
}

/*
 * We used to support using pause I/O for certain machines.  We
 * haven't supported this for a while, but just in case it's badly
 * needed for certain old 386 machines, I've left these #define's
 * in....
 */

/*
 * For the 16C950
 */
void multi_icr_write(struct sb_async_struct *info, int offset, int  value)
{
	multi_out(info, UART_SCR, offset);
	multi_out(info, UART_ICR, value);
}

unsigned int multi_icr_read(struct sb_async_struct *info, int offset)
{
	int	value;

	multi_icr_write(info, UART_ACR, info->ACR | UART_ACR_ICRRD);
	multi_out(info, UART_SCR, offset);
	value = multi_in(info, UART_ICR);
	multi_icr_write(info, UART_ACR, info->ACR);
	return value;
}

/*
 * ------------------------------------------------------------
 * mp_stop() and mp_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * They enable or disable transmitter interrupts, as necessary.
 * ------------------------------------------------------------
 */
static void mp_stop(struct tty_struct *tty)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;

	
	save_flags(flags); cli();
	if (info->IER & UART_IER_THRI) {
		info->IER &= ~UART_IER_THRI;
		multi_out(info, UART_IER, info->IER);
	}
	restore_flags(flags);
}

static void mp_start(struct tty_struct *tty)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;
	
	
	save_flags(flags); cli();
	if (info->xmit.head != info->xmit.tail
	    && info->xmit.buf
	    && !(info->IER & UART_IER_THRI)) {
		info->IER |= UART_IER_THRI;
		multi_out(info, UART_IER, info->IER);
	}
	restore_flags(flags);
}

/*
 * This routine is used by the interrupt handler to schedule
 * processing in the software interrupt portion of the driver.
 */
static _INLINE_ void mp_sched_event(struct sb_async_struct *info,
				  int event)
{
	info->event |= 1 << event;
	queue_task(&info->tqueue, &tq_multi);
	mark_bh(MP_BH);
}

static _INLINE_ void receive_chars(struct sb_async_struct *info,
				 int *status, struct pt_regs * regs)
{
	struct tty_struct *tty = info->tty;
	//unsigned char ch;
	int ignored = 0;
	struct	async_icount *icount;
	int count = 0,room;
	unsigned char *cbuf, *fbuf;

	cbuf = mp_char_buf;
	fbuf = mp_flag_buf;

	icount = &info->state->icount;
	do {
		//ch = multi_in(info, UART_RX);
		*cbuf = multi_in(info,UART_RX);
		if (count >= N_TTY_BUF_SIZE -1)
			goto ignore_char;
		//*tty->flip.char_buf_ptr = ch;
		icount->rx++;
		
		//*tty->flip.flag_buf_ptr = 0;
		*fbuf = 0;
		if (*status & (UART_LSR_BI | UART_LSR_PE |
			       UART_LSR_FE | UART_LSR_OE)) {
			/*
			 * For statistics only
			 */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				icount->brk++;
				if (info->flags & ASYNC_SAK)
					do_SAK(tty);
			} else if (*status & UART_LSR_PE)
				icount->parity++;
			else if (*status & UART_LSR_FE)
				icount->frame++;
			if (*status & UART_LSR_OE)
				icount->overrun++;

			/*
			 * Now check to see if character should be
			 * ignored, and mask off conditions which
			 * should be ignored.
			 */
			if (*status & info->ignore_status_mask) {
				if (++ignored > 100)
					break;
				goto ignore_char;
			}
			*status &= info->read_status_mask;

			if (*status & (UART_LSR_BI)) {
				//*tty->flip.flag_buf_ptr = TTY_BREAK;
				*fbuf = TTY_BREAK;
			} else if (*status & UART_LSR_PE)
				//*tty->flip.flag_buf_ptr = TTY_PARITY;
				*fbuf = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				//*tty->flip.flag_buf_ptr = TTY_FRAME;
				*fbuf = TTY_FRAME;
			if (*status & UART_LSR_OE) {
				/*
				 * Overrun is special, since it's
				 * reported immediately, and doesn't
				 * affect the current character
				 */
				//tty->flip.count++;
				//tty->flip.flag_buf_ptr++;
				//tty->flip.char_buf_ptr++;
				//*tty->flip.flag_buf_ptr = TTY_OVERRUN;
				count++;
				fbuf++;
				cbuf++;
				*fbuf = TTY_OVERRUN;
				if (count >= N_TTY_BUF_SIZE)
					goto ignore_char;
			}
		}
		//tty->flip.flag_buf_ptr++;
		//tty->flip.char_buf_ptr++;
		//tty->flip.count++;
		fbuf++;
		cbuf++;
		count++;
	ignore_char:
		*status = multi_in(info, UART_LSR);
	} while (*status & UART_LSR_DR);
	//tty_flip_buffer_push(tty);
	room = tty->ldisc.receive_room(tty);
	count = count < room ? count : room;
	if(count>0)
		tty->ldisc.receive_buf(tty,mp_char_buf,mp_flag_buf,count);

}

static _INLINE_ void transmit_chars(struct sb_async_struct *info, int *intr_done)
{
	int count;
	if (info->x_char) {
		multi_out(info, UART_TX, info->x_char);
		info->state->icount.tx++;
		info->x_char = 0;
		if (intr_done)
			*intr_done = 0;
		return;
	}
	if (info->xmit.head == info->xmit.tail
	    || info->tty->stopped
	    || info->tty->hw_stopped) {
		info->IER &= ~UART_IER_THRI;
		multi_out(info, UART_IER, info->IER);
		return;
	}
	
	count = info->xmit_fifo_size;
	do {
		multi_out(info, UART_TX, info->xmit.buf[info->xmit.tail]);
		info->xmit.tail = (info->xmit.tail + 1) & (MULTI_XMIT_SIZE-1);
		info->state->icount.tx++;
		if (info->xmit.head == info->xmit.tail)
			break;
	} while (--count > 0);
	
	if (CIRC_CNT(info->xmit.head,
		     info->xmit.tail,
		     MULTI_XMIT_SIZE) < WAKEUP_CHARS)
		mp_sched_event(info, MP_EVENT_WRITE_WAKEUP);

	if (intr_done)
		*intr_done = 0;

	if (info->xmit.head == info->xmit.tail) {
		info->IER &= ~UART_IER_THRI;
		multi_out(info, UART_IER, info->IER);
	}
}

static _INLINE_ void check_modem_status(struct sb_async_struct *info)
{
	int	status;
	struct	async_icount *icount;
	
	status = multi_in(info, UART_MSR);

	if (status & UART_MSR_ANY_DELTA) {
		icount = &info->state->icount;
		/* update input line counters */
		if (status & UART_MSR_TERI)
			icount->rng++;
		if (status & UART_MSR_DDSR)
			icount->dsr++;
		if (status & UART_MSR_DDCD) {
			icount->dcd++;
#ifdef CONFIG_HARD_PPS
			if ((info->flags & ASYNC_HARDPPS_CD) &&
			    (status & UART_MSR_DCD))
				hardpps();
#endif
		}
		if (status & UART_MSR_DCTS)
			icount->cts++;
		wake_up_interruptible(&info->delta_msr_wait);
	}

	if ((info->flags & ASYNC_CHECK_CD) && (status & UART_MSR_DDCD)) {
		if (status & UART_MSR_DCD)
			wake_up_interruptible(&info->open_wait);
		else if (!((info->flags & ASYNC_CALLOUT_ACTIVE) &&
			   (info->flags & ASYNC_CALLOUT_NOHUP))) {
			if (info->tty)
				tty_hangup(info->tty);
		}
	}
	if (info->flags & ASYNC_CTS_FLOW) {
		if (info->tty->hw_stopped) {
			if (status & UART_MSR_CTS) {
				info->tty->hw_stopped = 0;
				info->IER |= UART_IER_THRI;
				multi_out(info, UART_IER, info->IER);
				mp_sched_event(info, MP_EVENT_WRITE_WAKEUP);
				return;
			}
		} else {
			if (!(status & UART_MSR_CTS)) {
				info->tty->hw_stopped = 1;
				info->IER &= ~UART_IER_THRI;
				multi_out(info, UART_IER, info->IER);
			}
		}
	}
}

/*
 * This is the serial driver's generic interrupt routine
 */
static void mp_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	int status,i,j;
	struct sb_async_struct * info;
	int pass_counter = 0;
	struct sb_async_struct *end_mark = 0;
	unsigned long addr;

	for(i=0; i<NR_DEV; i++){
		addr = mp_devs[i].option_reg_addr + ITR_MASK_REG;
		for(j=0; j < mp_devs[i].nr_ports/8; j++)
			outb(0x00,addr +j);
	}

	info = IRQ_ports[irq];
	if (!info)
		return;

	do {
		if (!info->tty ||
		    (multi_in(info, UART_IIR) & UART_IIR_NO_INT)) {
			if (!end_mark)
				end_mark = info;
			goto next;
		}
		end_mark = 0;

		info->last_active = jiffies;

		status = multi_in(info, UART_LSR);
		if (status & UART_LSR_DR)
			receive_chars(info, &status, regs);
		check_modem_status(info);
		if (status & UART_LSR_THRE)
			transmit_chars(info, 0);

	next:
		info = info->next_port;
		if (!info) {
			info = IRQ_ports[irq];
			if (pass_counter++ > _ISR_PASS_LIMIT) {
				break; 	/* Prevent infinite loops */
			}
			continue;
		}
	} while (end_mark != info);

	for(i=0; i<NR_DEV; i++){
		addr = mp_devs[i].option_reg_addr + ITR_MASK_REG;
		for(j=0; j < mp_devs[i].nr_ports/8; j++)
			outb(0xff,addr +j);
	}

}

/*
 * -------------------------------------------------------------------
 * Here ends the serial interrupt routines.
 * -------------------------------------------------------------------
 */

/*
 * This routine is used to handle the "bottom half" processing for the
 * serial driver, known also the "software interrupt" processing.
 * This processing is done at the kernel interrupt level, after the
 * mp_interrupt() has returned, BUT WITH INTERRUPTS TURNED ON.  This
 * is where time-consuming activities which can not be done in the
 * interrupt driver proper are done; the interrupt driver schedules
 * them using mp_sched_event(), and they get done here.
 */
static void do_multi_bh(void)
{
	run_task_queue(&tq_multi);
}

static void do_softint(void *private_)
{
	struct sb_async_struct	*info = (struct sb_async_struct *) private_;
	struct tty_struct	*tty;
	
	tty = info->tty;
	if (!tty)
		return;

	if (test_and_clear_bit(MP_EVENT_WRITE_WAKEUP, &info->event)) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    tty->ldisc.write_wakeup)
			(tty->ldisc.write_wakeup)(tty);
		wake_up_interruptible(&tty->write_wait);
#ifdef SERIAL_HAVE_POLL_WAIT
		wake_up_interruptible(&tty->poll_wait);
#endif
	}
}

/*
 * This subroutine is called when the RS_TIMER goes off.  It is used
 * by the serial driver to handle ports that do not have an interrupt
 * (irq=0).  This doesn't work very well for 16450's, but gives barely
 * passable results for a 16550A.  (Although at the expense of much
 * CPU overhead).
 */
static void mp_timer(unsigned long dummy)
{
	static unsigned long last_strobe;
	struct sb_async_struct *info;
	unsigned int	i;
	unsigned long flags;

	if ((jiffies - last_strobe) >= RS_STROBE_TIME) {
		for (i=0; i < NR_IRQS; i++) {
			info = IRQ_ports[i];
			if (!info)
				continue;
			save_flags(flags); cli();
			if (info->next_port) {
				do {
					multi_out(info, UART_IER, 0);
					info->IER |= UART_IER_THRI;
					multi_out(info, UART_IER, info->IER);
					info = info->next_port;
				} while (info);
				mp_interrupt(i, NULL, NULL);
			} 
			restore_flags(flags);
		}
	}
	last_strobe = jiffies;
	mod_timer(&multi_timer, jiffies + RS_STROBE_TIME);

	if (IRQ_ports[0]) {
		save_flags(flags); cli();
		mp_interrupt(0, NULL, NULL);
		restore_flags(flags);
		mod_timer(&multi_timer, jiffies + IRQ_timeout[0]);
	}
}

/*
 * ---------------------------------------------------------------
 * Low level utility subroutines for the serial driver:  routines to
 * figure out the appropriate timeout for an interrupt chain, routines
 * to initialize and startup a serial port, and routines to shutdown a
 * serial port.  Useful stuff like that.
 * ---------------------------------------------------------------
 */

/*
 * This routine figures out the correct timeout for a particular IRQ.
 * It uses the smallest timeout of all of the serial ports in a
 * particular interrupt chain.  Now only used for IRQ 0....
 */
static void figure_IRQ_timeout(int irq)
{
	struct	sb_async_struct	*info;
	int	timeout = 60*HZ;	/* 60 seconds === a long time :-) */

	info = IRQ_ports[irq];
	if (!info) {
		IRQ_timeout[irq] = 60*HZ;
		return;
	}
	while (info) {
		if (info->timeout < timeout)
			timeout = info->timeout;
		info = info->next_port;
	}
	if (!irq)
		timeout = timeout / 2;
	IRQ_timeout[irq] = (timeout > 3) ? timeout-2 : 1;
}

static int startup(struct sb_async_struct * info)
{
	unsigned long flags;
	int	retval=0;
	struct sb_serial_state *state= info->state;
	unsigned long page;

	page = get_zeroed_page(GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	save_flags(flags); cli();

	if (info->flags & ASYNC_INITIALIZED) {
		free_page(page);
		goto errout;
	}

	if (!CONFIGURED_SERIAL_PORT(state) || !state->type) {
		if (info->tty)
			set_bit(TTY_IO_ERROR, &info->tty->flags);
		free_page(page);
		goto errout;
	}
	if (info->xmit.buf)
		free_page(page);
	else
		info->xmit.buf = (unsigned char *) page;

	if (uart_config[state->type].flags & UART_STARTECH) {
		/* Wake up UART */
		multi_out(info, UART_LCR, 0xBF);
		multi_out(info, UART_EFR, UART_EFR_ECB);
		/*
		 * Turn off LCR == 0xBF so we actually set the IER
		 * register on the XR16C850
		 */
		multi_out(info, UART_LCR, 0);
		multi_out(info, UART_IER, 0);
		/*
		 * Now reset LCR so we can turn off the ECB bit
		 */
		multi_out(info, UART_LCR, 0xBF);
		multi_out(info, UART_EFR, 0);
		/*
		 * For a XR16C850, we need to set the trigger levels
		 */
		multi_out(info, UART_LCR, 0);
	}

	/*
	 * Clear the FIFO buffers and disable them
	 * (they will be reenabled in change_speed())
	 */
	if (uart_config[state->type].flags & UART_CLEAR_FIFO) {
		multi_out(info, UART_FCR, UART_FCR_ENABLE_FIFO);
		multi_out(info, UART_FCR, (UART_FCR_ENABLE_FIFO |
					     UART_FCR_CLEAR_RCVR |
					     UART_FCR_CLEAR_XMIT));
		multi_out(info, UART_FCR, 0);
	}

	/*
	 * Clear the interrupt registers.
	 */
	(void) multi_in(info, UART_LSR);
	(void) multi_in(info, UART_RX);
	(void) multi_in(info, UART_IIR);
	(void) multi_in(info, UART_MSR);

	/*
	 * At this point there's no way the LSR could still be 0xFF;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if (!(info->flags & ASYNC_BUGGY_UART) &&
	    (multi_in(info, UART_LSR) == 0xff)) {
		printk("LSR safety check engaged!\n");
		if (capable(CAP_SYS_ADMIN)) {
			if (info->tty)
				set_bit(TTY_IO_ERROR, &info->tty->flags);
		} else
			retval = -ENODEV;
		goto errout;
	}
	
	/*
	 * Allocate the IRQ if necessary
	 */
	if (state->irq && (!IRQ_ports[state->irq] ||
			  !IRQ_ports[state->irq]->next_port)) {
		if (IRQ_ports[state->irq]) {
			free_irq(state->irq, &IRQ_ports[state->irq]);
			//	handler = mp_interrupt;
		}  
		retval = request_irq(state->irq, mp_interrupt, SA_SHIRQ,
				     "MultiPorts", &IRQ_ports[state->irq]);
		if (retval) {
			if (capable(CAP_SYS_ADMIN)) {
				if (info->tty)
					set_bit(TTY_IO_ERROR,
						&info->tty->flags);
				retval = 0;
			}
			goto errout;
		}
	}

	/*
	 * Insert serial port into IRQ chain.
	 */
	info->prev_port = 0;
	info->next_port = IRQ_ports[state->irq];
	if (info->next_port)
		info->next_port->prev_port = info;
	IRQ_ports[state->irq] = info;
	figure_IRQ_timeout(state->irq);

	/*
	 * Now, initialize the UART 
	 */
	multi_out(info, UART_LCR, UART_LCR_WLEN8);	/* reset DLAB */

	info->MCR = 0;
	if (info->tty->termios->c_cflag & CBAUD)
		info->MCR = UART_MCR_DTR | UART_MCR_RTS;
	{
		if (state->irq != 0)
			info->MCR |= UART_MCR_OUT2;
	}
	info->MCR |= ALPHA_KLUDGE_MCR; 		/* Don't ask */
	multi_out(info, UART_MCR, info->MCR);
	
	/*
	 * Finally, enable interrupts
	 */
	info->IER = UART_IER_MSI | UART_IER_RLSI | UART_IER_RDI;
	multi_out(info, UART_IER, info->IER);	/* enable interrupts */
	

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void)multi_in(info, UART_LSR);
	(void)multi_in(info, UART_RX);
	(void)multi_in(info, UART_IIR);
	(void)multi_in(info, UART_MSR);

	if (info->tty)
		clear_bit(TTY_IO_ERROR, &info->tty->flags);
	info->xmit.head = info->xmit.tail = 0;

	/*
	 * Set up serial timers...
	 */
	mod_timer(&multi_timer, jiffies + 2*HZ/100);

	/*
	 * Set up the tty->alt_speed kludge
	 */
#if (LINUX_VERSION_CODE >= 131394) /* Linux 2.1.66 */
	if (info->tty) {
		if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_HI)
			info->tty->alt_speed = 57600;
		if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_VHI)
			info->tty->alt_speed = 115200;
		if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_SHI)
			info->tty->alt_speed = 230400;
		if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_WARP)
			info->tty->alt_speed = 460800;
	}
#endif
	
	/*
	 * and set the speed of the serial port
	 */
	change_speed(info, 0);

	info->flags |= ASYNC_INITIALIZED;
	restore_flags(flags);
	return 0;
	
errout:
	restore_flags(flags);
	return retval;
}

/*
 * This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */
static void shutdown(struct sb_async_struct * info)
{
	unsigned long	flags;
	struct sb_serial_state *state;
	int		retval;

	if (!(info->flags & ASYNC_INITIALIZED))
		return;

	state = info->state;

	
	save_flags(flags); cli(); /* Disable interrupts */

	/*
	 * clear delta_msr_wait queue to avoid mem leaks: we may free the irq
	 * here so the queue might never be waken up
	 */
	wake_up_interruptible(&info->delta_msr_wait);
	
	/*
	 * First unlink the serial port from the IRQ chain...
	 */
	if (info->next_port)
		info->next_port->prev_port = info->prev_port;
	if (info->prev_port)
		info->prev_port->next_port = info->next_port;
	else
		IRQ_ports[state->irq] = info->next_port;
	figure_IRQ_timeout(state->irq);
	
	/*
	 * Free the IRQ, if necessary
	 */
	if (state->irq && (!IRQ_ports[state->irq] ||
			  !IRQ_ports[state->irq]->next_port)) {
		if (IRQ_ports[state->irq]) {
			free_irq(state->irq, &IRQ_ports[state->irq]);
			retval = request_irq(state->irq, mp_interrupt,
					     SA_SHIRQ, "MultiPorts",
					     &IRQ_ports[state->irq]);
			
			if (retval)
				printk("multiports shutdown: request_irq: error %d"
				       "  Couldn't reacquire IRQ.\n", retval);
		} else
			free_irq(state->irq, &IRQ_ports[state->irq]);
	}

	if (info->xmit.buf) {
		unsigned long pg = (unsigned long) info->xmit.buf;
		info->xmit.buf = 0;
		free_page(pg);
	}

	info->IER = 0;
	multi_out(info, UART_IER, 0x00);	/* disable all intrs */
		info->MCR &= ~UART_MCR_OUT2;
	info->MCR |= ALPHA_KLUDGE_MCR; 		/* Don't ask */
	
	/* disable break condition */
	multi_out(info, UART_LCR, multi_in(info, UART_LCR) & ~UART_LCR_SBC);
	
	if (!info->tty || (info->tty->termios->c_cflag & HUPCL))
		info->MCR &= ~(UART_MCR_DTR|UART_MCR_RTS);
	multi_out(info, UART_MCR, info->MCR);

	/* disable FIFO's */	
	multi_out(info, UART_FCR, (UART_FCR_ENABLE_FIFO |
				     UART_FCR_CLEAR_RCVR |
				     UART_FCR_CLEAR_XMIT));
	multi_out(info, UART_FCR, 0);

	(void)multi_in(info, UART_RX);    /* read data port to reset things */
	
	if (info->tty)
		set_bit(TTY_IO_ERROR, &info->tty->flags);

	if (uart_config[info->state->type].flags & UART_STARTECH) {
		/* Arrange to enter sleep mode */
		multi_out(info, UART_LCR, 0xBF);
		multi_out(info, UART_EFR, UART_EFR_ECB);
		multi_out(info, UART_IER, UART_IERX_SLEEP);
		multi_out(info, UART_LCR, 0);
	}
	info->flags &= ~ASYNC_INITIALIZED;
	restore_flags(flags);
}

/*
 * This routine is called to set the UART divisor registers to match
 * the specified baud rate for a serial port.
 */
static void change_speed(struct sb_async_struct *info,
			 struct termios *old_termios)
{
	int	quot = 0, baud_base, baud;
	unsigned cflag, cval, fcr = 0;
	int	bits;
	unsigned long	flags;

	if (!info->tty || !info->tty->termios)
		return;
	cflag = info->tty->termios->c_cflag;
	if (!CONFIGURED_SERIAL_PORT(info))
		return;

	/* byte size and parity */
	switch (cflag & CSIZE) {
	      case CS5: cval = 0x00; bits = 7; break;
	      case CS6: cval = 0x01; bits = 8; break;
	      case CS7: cval = 0x02; bits = 9; break;
	      case CS8: cval = 0x03; bits = 10; break;
	      /* Never happens, but GCC is too dumb to figure it out */
	      default:  cval = 0x00; bits = 7; break;
	      }
	if (cflag & CSTOPB) {
		cval |= 0x04;
		bits++;
	}
	if (cflag & PARENB) {
		cval |= UART_LCR_PARITY;
		bits++;
	}
	if (!(cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/* Determine divisor based on baud rate */
	baud = tty_get_baud_rate(info->tty);
	if (!baud)
		baud = 9600;	/* B0 transition handled in mp_set_termios */
	baud_base = info->state->baud_base;
	if (baud == 38400 &&
	    ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_CUST))
		quot = info->state->custom_divisor;
	else {
		if (baud == 134)
			/* Special case since 134 is really 134.5 */
			quot = (2*baud_base / 269);
		else if (baud)
			quot = baud_base / baud;
	}
	/* If the quotient is zero refuse the change */
	if (!quot && old_termios) {
		info->tty->termios->c_cflag &= ~CBAUD;
		info->tty->termios->c_cflag |= (old_termios->c_cflag & CBAUD);
		baud = tty_get_baud_rate(info->tty);
		if (!baud)
			baud = 9600;
		if (baud == 38400 &&
		    ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_CUST))
			quot = info->state->custom_divisor;
		else {
			if (baud == 134)
				/* Special case since 134 is really 134.5 */
				quot = (2*baud_base / 269);
			else if (baud)
				quot = baud_base / baud;
		}
	}
	/* As a last resort, if the quotient is zero, default to 9600 bps */
	if (!quot)
		quot = baud_base / 9600;
	/*
	 * Work around a bug in the Oxford Semiconductor 952 rev B
	 * chip which causes it to seriously miscalculate baud rates
	 * when DLL is 0.
	 */
	info->quot = quot;
	info->timeout = ((info->xmit_fifo_size*HZ*bits*quot) / baud_base);
	info->timeout += HZ/50;		/* Add .02 seconds of slop */

	/* Set up FIFO's */
	if (uart_config[info->state->type].flags & UART_USE_FIFO) {
		if ((info->state->baud_base / quot) < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
		else
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_8;
	}
	
	/* CTS flow control flag and modem status interrupts */
	info->IER &= ~UART_IER_MSI;
	if (info->flags & ASYNC_HARDPPS_CD)
		info->IER |= UART_IER_MSI;
	if (cflag & CRTSCTS) {
		info->flags |= ASYNC_CTS_FLOW;
		info->IER |= UART_IER_MSI;
	} else
		info->flags &= ~ASYNC_CTS_FLOW;
	if (cflag & CLOCAL)
		info->flags &= ~ASYNC_CHECK_CD;
	else {
		info->flags |= ASYNC_CHECK_CD;
		info->IER |= UART_IER_MSI;
	}
	multi_out(info, UART_IER, info->IER);

	/*
	 * Set up parity check flag
	 */
#define RELEVANT_IFLAG(iflag) (iflag & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

	info->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (I_INPCK(info->tty))
		info->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (I_BRKINT(info->tty) || I_PARMRK(info->tty))
		info->read_status_mask |= UART_LSR_BI;
	
	/*
	 * Characters to ignore
	 */
	info->ignore_status_mask = 0;
	if (I_IGNPAR(info->tty))
		info->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (I_IGNBRK(info->tty)) {
		info->ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignore parity and break indicators, ignore 
		 * overruns too.  (For real raw support).
		 */
		if (I_IGNPAR(info->tty))
			info->ignore_status_mask |= UART_LSR_OE;
	}
	/*
	 * !!! ignore all characters if CREAD is not set
	 */
	if ((cflag & CREAD) == 0)
		info->ignore_status_mask |= UART_LSR_DR;
	save_flags(flags); cli();
	if (uart_config[info->state->type].flags & UART_STARTECH) {
		multi_out(info, UART_LCR, 0xBF);
		multi_out(info, UART_EFR,
			    (cflag & CRTSCTS) ? UART_EFR_CTS : 0);
	}
	multi_out(info, UART_LCR, cval | UART_LCR_DLAB);	/* set DLAB */
	multi_out(info, UART_DLL, quot & 0xff);	/* LS of divisor */
	multi_out(info, UART_DLM, quot >> 8);		/* MS of divisor */
	multi_out(info, UART_LCR, cval);		/* reset DLAB */
	info->LCR = cval;				/* Save LCR */
 		if (fcr & UART_FCR_ENABLE_FIFO) {
 			/* emulated UARTs (Lucent Venus 167x) need two steps */
 			multi_out(info, UART_FCR, UART_FCR_ENABLE_FIFO);
 		}
		multi_out(info, UART_FCR, fcr); 	/* set fcr */
	restore_flags(flags);
}

static void mp_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;


	if (!tty || !info->xmit.buf)
		return;

	save_flags(flags); cli();
	if (CIRC_SPACE(info->xmit.head,
		       info->xmit.tail,
		       MULTI_XMIT_SIZE) == 0) {
		restore_flags(flags);
		return;
	}

	info->xmit.buf[info->xmit.head] = ch;
	info->xmit.head = (info->xmit.head + 1) & (MULTI_XMIT_SIZE-1);
	restore_flags(flags);
}

static void mp_flush_chars(struct tty_struct *tty)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;
				

	if (info->xmit.head == info->xmit.tail
	    || tty->stopped
	    || tty->hw_stopped
	    || !info->xmit.buf)
		return;

	save_flags(flags); cli();
	info->IER |= UART_IER_THRI;
	multi_out(info, UART_IER, info->IER);
	restore_flags(flags);
}

static int mp_write(struct tty_struct * tty, int from_user,
		    const unsigned char *buf, int count)
{
	int	c, ret = 0;
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;
				

	if (!tty || !info->xmit.buf || !tmp_buf)
		return 0;

	save_flags(flags);
	if (from_user) {
		down(&tmp_buf_sem);
		while (1) {
			int c1;
			c = CIRC_SPACE_TO_END(info->xmit.head,
					      info->xmit.tail,
					      MULTI_XMIT_SIZE);
			if (count < c)
				c = count;
			if (c <= 0)
				break;

			c -= copy_from_user(tmp_buf, buf, c);
			if (!c) {
				if (!ret)
					ret = -EFAULT;
				break;
			}
			cli();
			c1 = CIRC_SPACE_TO_END(info->xmit.head,
					       info->xmit.tail,
					       MULTI_XMIT_SIZE);
			if (c1 < c)
				c = c1;
			memcpy(info->xmit.buf + info->xmit.head, tmp_buf, c);
			info->xmit.head = ((info->xmit.head + c) &
					   (MULTI_XMIT_SIZE-1));
			restore_flags(flags);
			buf += c;
			count -= c;
			ret += c;
		}
		up(&tmp_buf_sem);
	} else {
		cli();
		while (1) {
			c = CIRC_SPACE_TO_END(info->xmit.head,
					      info->xmit.tail,
					      MULTI_XMIT_SIZE);
			if (count < c)
				c = count;
			if (c <= 0) {
				break;
			}
			memcpy(info->xmit.buf + info->xmit.head, buf, c);
			info->xmit.head = ((info->xmit.head + c) &
					   (MULTI_XMIT_SIZE-1));
			buf += c;
			count -= c;
			ret += c;
		}
		restore_flags(flags);
	}
	if (info->xmit.head != info->xmit.tail
	    && !tty->stopped
	    && !tty->hw_stopped
	    && !(info->IER & UART_IER_THRI)) {
		info->IER |= UART_IER_THRI;
		multi_out(info, UART_IER, info->IER);
	}
	return ret;
}

static int mp_write_room(struct tty_struct *tty)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	return CIRC_SPACE(info->xmit.head, info->xmit.tail, MULTI_XMIT_SIZE);
}

static int mp_chars_in_buffer(struct tty_struct *tty)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
				
	return CIRC_CNT(info->xmit.head, info->xmit.tail, MULTI_XMIT_SIZE);
}

static void mp_flush_buffer(struct tty_struct *tty)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;
	
	save_flags(flags); cli();
	info->xmit.head = info->xmit.tail = 0;
	restore_flags(flags);
	wake_up_interruptible(&tty->write_wait);
#ifdef SERIAL_HAVE_POLL_WAIT
	wake_up_interruptible(&tty->poll_wait);
#endif
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup)(tty);
}

/*
 * This function is used to send a high-priority XON/XOFF character to
 * the device
 */
static void mp_send_xchar(struct tty_struct *tty, char ch)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;


	info->x_char = ch;
	if (ch) {
		/* Make sure transmit interrupts are on */
		info->IER |= UART_IER_THRI;
		multi_out(info, UART_IER, info->IER);
	}
}

/*
 * ------------------------------------------------------------
 * mp_throttle()
 * 
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 * ------------------------------------------------------------
 */
static void mp_throttle(struct tty_struct * tty)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;

	
	if (I_IXOFF(tty))
		mp_send_xchar(tty, STOP_CHAR(tty));

	if (tty->termios->c_cflag & CRTSCTS)
		info->MCR &= ~UART_MCR_RTS;

	save_flags(flags); cli();
	multi_out(info, UART_MCR, info->MCR);
	restore_flags(flags);
}

static void mp_unthrottle(struct tty_struct * tty)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;

	
	if (I_IXOFF(tty)) {
		if (info->x_char)
			info->x_char = 0;
		else
			mp_send_xchar(tty, START_CHAR(tty));
	}
	if (tty->termios->c_cflag & CRTSCTS)
		info->MCR |= UART_MCR_RTS;
	save_flags(flags); cli();
	multi_out(info, UART_MCR, info->MCR);
	restore_flags(flags);
}

/*
 * ------------------------------------------------------------
 * mp_ioctl() and friends
 * ------------------------------------------------------------
 */

static int get_multi_info(struct sb_async_struct * info,
			   struct serial_struct * retinfo)
{
	struct serial_struct tmp;
	struct sb_serial_state *state = info->state;
   
	if (!retinfo)
		return -EFAULT;
	memset(&tmp, 0, sizeof(tmp));
	tmp.type = state->type;
	tmp.line = state->line;
	tmp.port = state->port;
	if (HIGH_BITS_OFFSET)
		tmp.port_high = state->port >> HIGH_BITS_OFFSET;
	else
		tmp.port_high = 0;
	tmp.irq = state->irq;
	tmp.flags = state->flags;
	tmp.xmit_fifo_size = state->xmit_fifo_size;
	tmp.baud_base = state->baud_base;
	tmp.close_delay = state->close_delay;
	tmp.closing_wait = state->closing_wait;
	tmp.custom_divisor = state->custom_divisor;
	tmp.hub6 = state->hub6;
	tmp.io_type = state->io_type;
	if (copy_to_user(retinfo,&tmp,sizeof(*retinfo)))
		return -EFAULT;
	return 0;
}

static int set_multi_info(struct sb_async_struct * info,
			   struct serial_struct * new_info)
{
	struct serial_struct new_serial;
 	struct sb_serial_state old_state, *state;
	unsigned int		i,change_irq,change_port;
	int 			retval = 0;
	unsigned long		new_port;

	if (copy_from_user(&new_serial,new_info,sizeof(new_serial)))
		return -EFAULT;
	state = info->state;
	old_state = *state;

	new_port = new_serial.port;
	if (HIGH_BITS_OFFSET)
		new_port += (unsigned long) new_serial.port_high << HIGH_BITS_OFFSET;

	change_irq = new_serial.irq != state->irq;
	change_port = (new_port != ((int) state->port)) ||
		(new_serial.hub6 != state->hub6);
  
	if (!capable(CAP_SYS_ADMIN)) {
		if (change_irq || change_port ||
		    (new_serial.baud_base != state->baud_base) ||
		    (new_serial.type != state->type) ||
		    (new_serial.close_delay != state->close_delay) ||
		    (new_serial.xmit_fifo_size != state->xmit_fifo_size) ||
		    ((new_serial.flags & ~ASYNC_USR_MASK) !=
		     (state->flags & ~ASYNC_USR_MASK)))
			return -EPERM;
		state->flags = ((state->flags & ~ASYNC_USR_MASK) |
			       (new_serial.flags & ASYNC_USR_MASK));
		info->flags = ((info->flags & ~ASYNC_USR_MASK) |
			       (new_serial.flags & ASYNC_USR_MASK));
		state->custom_divisor = new_serial.custom_divisor;
		goto check_and_exit;
	}

	new_serial.irq = irq_cannonicalize(new_serial.irq);

	if ((new_serial.irq >= NR_IRQS) || (new_serial.irq < 0) || 
	    (new_serial.baud_base < 9600)|| (new_serial.type < PORT_UNKNOWN) ||
	    (new_serial.type > PORT_MAX) || (new_serial.type == PORT_STARTECH)) {
		return -EINVAL;
	}

	if ((new_serial.type != state->type) ||
	    (new_serial.xmit_fifo_size <= 0))
		new_serial.xmit_fifo_size =
			uart_config[new_serial.type].dfl_xmit_fifo_size;

	/* Make sure address is not already in use */
	if (new_serial.type) {
		for (i = 0 ; i < NR_PORTS; i++)
			if ((state != &mp_table[i]) &&
			    (mp_table[i].port == new_port) &&
			    mp_table[i].type)
				return -EADDRINUSE;
	}

	if ((change_port || change_irq) && (state->count > 1))
		return -EBUSY;

	/*
	 * OK, past this point, all the error checking has been done.
	 * At this point, we start making changes.....
	 */

	state->baud_base = new_serial.baud_base;
	state->flags = ((state->flags & ~ASYNC_FLAGS) |
			(new_serial.flags & ASYNC_FLAGS));
	info->flags = ((state->flags & ~ASYNC_INTERNAL_FLAGS) |
		       (info->flags & ASYNC_INTERNAL_FLAGS));
	state->custom_divisor = new_serial.custom_divisor;
	state->close_delay = new_serial.close_delay * HZ/100;
	state->closing_wait = new_serial.closing_wait * HZ/100;
#if (LINUX_VERSION_CODE > 0x20100)
	info->tty->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;
#endif
	info->xmit_fifo_size = state->xmit_fifo_size =
		new_serial.xmit_fifo_size;

//	if ((state->type != PORT_UNKNOWN) && state->port) {
//		release_region(state->port,8);
//	}
	state->type = new_serial.type;
	if (change_port || change_irq) {
		/*
		 * We need to shutdown the serial port at the old
		 * port/irq combination.
		 */
		shutdown(info);
		state->irq = new_serial.irq;
		info->port = state->port = new_port;
		info->hub6 = state->hub6 = new_serial.hub6;
		if (info->hub6)
			info->io_type = state->io_type = SERIAL_IO_HUB6;
		else if (info->io_type == SERIAL_IO_HUB6)
			info->io_type = state->io_type = SERIAL_IO_PORT;
	}
//	if ((state->type != PORT_UNKNOWN) && state->port) {
//			request_region(state->port,8,"serial(set)");
//	}

	
check_and_exit:
	if (!state->port || !state->type)
		return 0;
	if (info->flags & ASYNC_INITIALIZED) {
		if (((old_state.flags & ASYNC_SPD_MASK) !=
		     (state->flags & ASYNC_SPD_MASK)) ||
		    (old_state.custom_divisor != state->custom_divisor)) {
#if (LINUX_VERSION_CODE >= 131394) /* Linux 2.1.66 */
			if ((state->flags & ASYNC_SPD_MASK) == ASYNC_SPD_HI)
				info->tty->alt_speed = 57600;
			if ((state->flags & ASYNC_SPD_MASK) == ASYNC_SPD_VHI)
				info->tty->alt_speed = 115200;
			if ((state->flags & ASYNC_SPD_MASK) == ASYNC_SPD_SHI)
				info->tty->alt_speed = 230400;
			if ((state->flags & ASYNC_SPD_MASK) == ASYNC_SPD_WARP)
				info->tty->alt_speed = 460800;
#endif
			change_speed(info, 0);
		}
	} else
		retval = startup(info);
	return retval;
}


/*
 * get_lsr_info - get line status register info
 *
 * Purpose: Let user call ioctl() to get info when the UART physically
 * 	    is emptied.  On bus types like RS485, the transmitter must
 * 	    release the bus after transmitting. This must be done when
 * 	    the transmit shift register is empty, not be done when the
 * 	    transmit holding register is empty.  This functionality
 * 	    allows an RS485 driver to be written in user space. 
 */
static int get_lsr_info(struct sb_async_struct * info, unsigned int *value)
{
	unsigned char status;
	unsigned int result;
	unsigned long flags;

	save_flags(flags); cli();
	status = multi_in(info, UART_LSR);
	restore_flags(flags);
	result = ((status & UART_LSR_TEMT) ? TIOCSER_TEMT : 0);

	/*
	 * If we're about to load something into the transmit
	 * register, we'll pretend the transmitter isn't empty to
	 * avoid a race condition (depending on when the transmit
	 * interrupt happens).
	 */
	if (info->x_char || 
	    ((CIRC_CNT(info->xmit.head, info->xmit.tail,
		       MULTI_XMIT_SIZE) > 0) &&
	     !info->tty->stopped && !info->tty->hw_stopped))
		result &= ~TIOCSER_TEMT;

	if (copy_to_user(value, &result, sizeof(int)))
		return -EFAULT;
	return 0;
}

/*
static int get_num_ports(unsigned int *value)
{
	int	nport = NR_PORTS;

	if (copy_to_user(value, &nport, sizeof(int)))
		return -EFAULT;

	return 0;

}
*/

static int get_modem_info(struct sb_async_struct * info, unsigned int *value)
{
	unsigned char control, status;
	unsigned int result;
	unsigned long flags;

	control = info->MCR;
	save_flags(flags); cli();
	status = multi_in(info, UART_MSR);
	restore_flags(flags);
	result =  ((control & UART_MCR_RTS) ? TIOCM_RTS : 0)
		| ((control & UART_MCR_DTR) ? TIOCM_DTR : 0)
#ifdef TIOCM_OUT1
		| ((control & UART_MCR_OUT1) ? TIOCM_OUT1 : 0)
		| ((control & UART_MCR_OUT2) ? TIOCM_OUT2 : 0)
#endif
		| ((status  & UART_MSR_DCD) ? TIOCM_CAR : 0)
		| ((status  & UART_MSR_RI) ? TIOCM_RNG : 0)
		| ((status  & UART_MSR_DSR) ? TIOCM_DSR : 0)
		| ((status  & UART_MSR_CTS) ? TIOCM_CTS : 0);

	if (copy_to_user(value, &result, sizeof(int)))
		return -EFAULT;
	return 0;
}

static int set_modem_info(struct sb_async_struct * info, unsigned int cmd,
			  unsigned int *value)
{
	unsigned int arg;
	unsigned long flags;

	if (copy_from_user(&arg, value, sizeof(int)))
		return -EFAULT;

	switch (cmd) {
	case TIOCMBIS: 
		if (arg & TIOCM_RTS)
			info->MCR |= UART_MCR_RTS;
		if (arg & TIOCM_DTR)
			info->MCR |= UART_MCR_DTR;
#ifdef TIOCM_OUT1
		if (arg & TIOCM_OUT1)
			info->MCR |= UART_MCR_OUT1;
		if (arg & TIOCM_OUT2)
			info->MCR |= UART_MCR_OUT2;
#endif
		if (arg & TIOCM_LOOP)
			info->MCR |= UART_MCR_LOOP;
		break;
	case TIOCMBIC:
		if (arg & TIOCM_RTS)
			info->MCR &= ~UART_MCR_RTS;
		if (arg & TIOCM_DTR)
			info->MCR &= ~UART_MCR_DTR;
#ifdef TIOCM_OUT1
		if (arg & TIOCM_OUT1)
			info->MCR &= ~UART_MCR_OUT1;
		if (arg & TIOCM_OUT2)
			info->MCR &= ~UART_MCR_OUT2;
#endif
		if (arg & TIOCM_LOOP)
			info->MCR &= ~UART_MCR_LOOP;
		break;
	case TIOCMSET:
		info->MCR = ((info->MCR & ~(UART_MCR_RTS |
#ifdef TIOCM_OUT1
					    UART_MCR_OUT1 |
					    UART_MCR_OUT2 |
#endif
					    UART_MCR_LOOP |
					    UART_MCR_DTR))
			     | ((arg & TIOCM_RTS) ? UART_MCR_RTS : 0)
#ifdef TIOCM_OUT1
			     | ((arg & TIOCM_OUT1) ? UART_MCR_OUT1 : 0)
			     | ((arg & TIOCM_OUT2) ? UART_MCR_OUT2 : 0)
#endif
			     | ((arg & TIOCM_LOOP) ? UART_MCR_LOOP : 0)
			     | ((arg & TIOCM_DTR) ? UART_MCR_DTR : 0));
		break;
	default:
		return -EINVAL;
	}
	save_flags(flags); cli();
	info->MCR |= ALPHA_KLUDGE_MCR; 		/* Don't ask */
	multi_out(info, UART_MCR, info->MCR);
	restore_flags(flags);
	return 0;
}

static int do_autoconfig(struct sb_async_struct * info)
{
	int irq, retval;
	
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	
	if (info->state->count > 1)
		return -EBUSY;
	
	shutdown(info);

	autoconfig(info->state);
	if ((info->state->flags & ASYNC_AUTO_IRQ) &&
	    (info->state->port != 0) &&
	    (info->state->type != PORT_UNKNOWN)) {
		irq = detect_uart_irq(info->state);
		if (irq > 0)
			info->state->irq = irq;
	}

	retval = startup(info);
	if (retval)
		return retval;
	return 0;
}

/*
 * mp_break() --- routine which turns the break handling on or off
 */
static void mp_break(struct tty_struct *tty, int break_state)
{
	struct sb_async_struct * info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;
	

	if (!CONFIGURED_SERIAL_PORT(info))
		return;
	save_flags(flags); cli();
	if (break_state == -1)
		info->LCR |= UART_LCR_SBC;
	else
		info->LCR &= ~UART_LCR_SBC;
	multi_out(info, UART_LCR, info->LCR);
	restore_flags(flags);
}

static int mp_ioctl(struct tty_struct *tty, struct file * file,
		    unsigned int cmd, unsigned long arg)
{
	struct sb_async_struct * info = (struct sb_async_struct *)tty->driver_data;
	struct async_icount cprev, cnow;	/* kernel counter temps */
	struct serial_icounter_struct icount;
	unsigned long flags;
	

	if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
	    (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGSTRUCT) &&
	    (cmd != TIOCMGET) && ( cmd != TIOCMBIS) && ( cmd != TIOCMBIC ) &&
	    (cmd != TIOCSERGETLSR) && ( cmd != TIOCGNUMOFPORT ) &&
		(cmd != TIOCSMULTIECHO) && ( cmd != TIOCSPTPNOECHO) &&
	    (cmd != TIOCMIWAIT) && (cmd != TIOCGICOUNT)) {
		if (tty->flags & (1 << TTY_IO_ERROR))
		    return -EIO;
	}
	
	switch (cmd) {
		case TIOCMGET:
			return get_modem_info(info, (unsigned int *) arg);
		case TIOCMBIS:
		case TIOCMBIC:
		case TIOCMSET:
			return set_modem_info(info, cmd, (unsigned int *) arg);
		case TIOCGSERIAL:
			return get_multi_info(info,
					       (struct serial_struct *) arg);
		case TIOCSSERIAL:
			return set_multi_info(info,
					       (struct serial_struct *) arg);
		case TIOCSERCONFIG:
			return do_autoconfig(info);

		case TIOCSERGETLSR: /* Get line status register */
			return get_lsr_info(info, (unsigned int *) arg);
		case TIOCGNUMOFPORT: /* Get number of ports */
			return NR_PORTS;
			//return get_num_ports((unsigned int *)arg);
		case TIOCSMULTIECHO: /* set to multi-drop mode(RS422) or echo mode(RS485) */
			save_flags(flags); cli();
			outb( ( inb(info->state->interface_config_addr) & ~0x03 ) | 0x01 , info->state->interface_config_addr );
			restore_flags(flags);
			return 0;
		case TIOCSPTPNOECHO: /* set to multi-drop mode(RS422) or echo mode(RS485) */
			save_flags(flags); cli();
			outb( ( inb(info->state->interface_config_addr) & ~0x03 )  , info->state->interface_config_addr );
			restore_flags(flags);
			return 0;

		case TIOCSERGSTRUCT:
			if (copy_to_user((struct sb_async_struct *) arg,
					 info, sizeof(struct sb_async_struct)))
				return -EFAULT;
			return 0;
				
		/*
		 * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
		 * - mask passed in arg for lines of interest
 		 *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
		 * Caller should use TIOCGICOUNT to see which one it was
		 */
		case TIOCMIWAIT:
			save_flags(flags); cli();
			/* note the counters on entry */
			cprev = info->state->icount;
			restore_flags(flags);
			/* Force modem status interrupts on */
			info->IER |= UART_IER_MSI;
			multi_out(info, UART_IER, info->IER);
			while (1) {
				interruptible_sleep_on(&info->delta_msr_wait);
				/* see if a signal did it */
				if (signal_pending(current))
					return -ERESTARTSYS;
				save_flags(flags); cli();
				cnow = info->state->icount; /* atomic copy */
				restore_flags(flags);
				if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr && 
				    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
					return -EIO; /* no change => error */
				if ( ((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
				     ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
				     ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
				     ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
					return 0;
				}
				cprev = cnow;
			}
			/* NOTREACHED */

		/* 
		 * Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
		 * Return: write counters to the user passed counter struct
		 * NB: both 1->0 and 0->1 transitions are counted except for
		 *     RI where only 0->1 is counted.
		 */
		case TIOCGICOUNT:
			save_flags(flags); cli();
			cnow = info->state->icount;
			restore_flags(flags);
			icount.cts = cnow.cts;
			icount.dsr = cnow.dsr;
			icount.rng = cnow.rng;
			icount.dcd = cnow.dcd;
			icount.rx = cnow.rx;
			icount.tx = cnow.tx;
			icount.frame = cnow.frame;
			icount.overrun = cnow.overrun;
			icount.parity = cnow.parity;
			icount.brk = cnow.brk;
			icount.buf_overrun = cnow.buf_overrun;
			
			if (copy_to_user((void *)arg, &icount, sizeof(icount)))
				return -EFAULT;
			return 0;
		case TIOCSERGWILD:
		case TIOCSERSWILD:
			/* "setserial -W" is called in Debian boot */
			printk ("TIOCSER?WILD ioctl obsolete, ignored.\n");
			return 0;

		default:
			return -ENOIOCTLCMD;
		}
	return 0;
}

static void mp_set_termios(struct tty_struct *tty, struct termios *old_termios)
{
	struct sb_async_struct *info = (struct sb_async_struct *)tty->driver_data;
	unsigned long flags;
	unsigned int cflag = tty->termios->c_cflag;
	
	if (   (cflag == old_termios->c_cflag)
	    && (   RELEVANT_IFLAG(tty->termios->c_iflag) 
		== RELEVANT_IFLAG(old_termios->c_iflag)))
	  return;

	change_speed(info, old_termios);

	/* Handle transition to B0 status */
	if ((old_termios->c_cflag & CBAUD) &&
	    !(cflag & CBAUD)) {
		info->MCR &= ~(UART_MCR_DTR|UART_MCR_RTS);
		save_flags(flags); cli();
		multi_out(info, UART_MCR, info->MCR);
		restore_flags(flags);
	}
	
	/* Handle transition away from B0 status */
	if (!(old_termios->c_cflag & CBAUD) &&
	    (cflag & CBAUD)) {
		info->MCR |= UART_MCR_DTR;
		if (!(tty->termios->c_cflag & CRTSCTS) || 
		    !test_bit(TTY_THROTTLED, &tty->flags)) {
			info->MCR |= UART_MCR_RTS;
		}
		save_flags(flags); cli();
		multi_out(info, UART_MCR, info->MCR);
		restore_flags(flags);
	}
	
	/* Handle turning off CRTSCTS */
	if ((old_termios->c_cflag & CRTSCTS) &&
	    !(tty->termios->c_cflag & CRTSCTS)) {
		tty->hw_stopped = 0;
		mp_start(tty);
	}
	
	if (info->state->type == PORT_16C105X)
	{
		set_deep_fifo(info, ENABLE);
	}

	if (info->state->interface >= RS485NE)
	{
		set_auto_rts(info,ENABLE);
	}

}

/*
 * ------------------------------------------------------------
 * mp_close()
 * 
 * This routine is called when the serial port gets closed.  First, we
 * wait for the last remaining data to be sent.  Then, we unlink its
 * async structure from the interrupt chain if necessary, and we free
 * that IRQ if nothing is left in the chain.
 * ------------------------------------------------------------
 */
static void mp_close(struct tty_struct *tty, struct file * filp)
{
	struct sb_async_struct * info = (struct sb_async_struct *)tty->driver_data;
	struct sb_serial_state *state;
	unsigned long flags;

	if (!info )
		return;

	state = info->state;
	
	save_flags(flags); cli();
	
	if (tty_hung_up_p(filp)) {
		MOD_DEC_USE_COUNT;
		restore_flags(flags);
		return;
	}
	
	if ((tty->count == 1) && (state->count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  state->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		printk("mp_close: bad multiports port count; tty->count is 1, "
		       "state->count is %d\n", state->count);
		state->count = 1;
	}
	if (--state->count < 0) {
		printk("mp_close: bad multiports port count for ttyMP%d: %d\n",
		       info->line, state->count);
		state->count = 0;
	}
	if (state->count) {
		MOD_DEC_USE_COUNT;
		restore_flags(flags);
		return;
	}
	info->flags |= ASYNC_CLOSING;
	restore_flags(flags);
	/*
	 * Save the termios structure, since this port may have
	 * separate termios for callout and dialin.
	 */
	if (info->flags & ASYNC_NORMAL_ACTIVE)
		info->state->normal_termios = *tty->termios;
	if (info->flags & ASYNC_CALLOUT_ACTIVE)
		info->state->callout_termios = *tty->termios;
	/*
	 * Now we wait for the transmit buffer to clear; and we notify 
	 * the line discipline to only process XON/XOFF characters.
	 */
	tty->closing = 1;
	if (info->closing_wait != ASYNC_CLOSING_WAIT_NONE)
		tty_wait_until_sent(tty, info->closing_wait);
	/*
	 * At this point we stop accepting input.  To do this, we
	 * disable the receive line status interrupts, and tell the
	 * interrupt driver to stop checking the data ready bit in the
	 * line status register.
	 */
	info->IER &= ~UART_IER_RLSI;
	info->read_status_mask &= ~UART_LSR_DR;
	if (info->flags & ASYNC_INITIALIZED) {
		multi_out(info, UART_IER, info->IER);
		/*
		 * Before we drop DTR, make sure the UART transmitter
		 * has completely drained; this is especially
		 * important if there is a transmit FIFO!
		 */
		mp_wait_until_sent(tty, info->timeout);
	}
	shutdown(info);
	if (tty->driver.flush_buffer)
		tty->driver.flush_buffer(tty);
	if (tty->ldisc.flush_buffer)
		tty->ldisc.flush_buffer(tty);
	tty->closing = 0;
	info->event = 0;
	info->tty = 0;
	if (info->blocked_open) {
		if (info->close_delay) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(info->close_delay);
		}
		wake_up_interruptible(&info->open_wait);
	}
	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CALLOUT_ACTIVE|
			 ASYNC_CLOSING);
	wake_up_interruptible(&info->close_wait);
	MOD_DEC_USE_COUNT;
}

/*
 * mp_wait_until_sent() --- wait until the transmitter is empty
 */
static void mp_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct sb_async_struct * info = (struct sb_async_struct *)tty->driver_data;
	unsigned long orig_jiffies, char_time;
	int lsr;
	

	if (info->state->type == PORT_UNKNOWN)
		return;

	if (info->xmit_fifo_size == 0)
		return; /* Just in case.... */

	orig_jiffies = jiffies;
	/*
	 * Set the check interval to be 1/5 of the estimated time to
	 * send a single character, and make it at least 1.  The check
	 * interval should also be less than the timeout.
	 * 
	 * Note: we have to use pretty tight timings here to satisfy
	 * the NIST-PCTS.
	 */
	char_time = (info->timeout - HZ/50) / info->xmit_fifo_size;
	char_time = char_time / 5;
	if (char_time == 0)
		char_time = 1;
	if (timeout && timeout < char_time)
		char_time = timeout;
	/*
	 * If the transmitter hasn't cleared in twice the approximate
	 * amount of time to send the entire FIFO, it probably won't
	 * ever clear.  This assumes the UART isn't doing flow
	 * control, which is currently the case.  Hence, if it ever
	 * takes longer than info->timeout, this is probably due to a
	 * UART bug of some kind.  So, we clamp the timeout parameter at
	 * 2*info->timeout.
	 */
	if (!timeout || timeout > 2*info->timeout)
		timeout = 2*info->timeout;
	while (!((lsr = multi_in(info, UART_LSR)) & UART_LSR_TEMT)) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(char_time);
		if (signal_pending(current))
			break;
		if (timeout && time_after(jiffies, orig_jiffies + timeout))
			break;
	}
	set_current_state(TASK_RUNNING);
}

/*
 * mp_hangup() --- called by tty_hangup() when a hangup is signaled.
 */
static void mp_hangup(struct tty_struct *tty)
{
	struct sb_async_struct * info = (struct sb_async_struct *)tty->driver_data;
	struct sb_serial_state *state = info->state;
	

	state = info->state;
	
	mp_flush_buffer(tty);
	if (info->flags & ASYNC_CLOSING)
		return;
	shutdown(info);
	info->event = 0;
	state->count = 0;
	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CALLOUT_ACTIVE);
	info->tty = 0;
	wake_up_interruptible(&info->open_wait);
}

/*
 * ------------------------------------------------------------
 * mp_open() and friends
 * ------------------------------------------------------------
 */
static int block_til_ready(struct tty_struct *tty, struct file * filp,
			   struct sb_async_struct *info)
{
	DECLARE_WAITQUEUE(wait, current);
	struct sb_serial_state *state = info->state;
	int		retval;
	int		do_clocal = 0, extra_count = 0;
	unsigned long	flags;

	/*
	 * If the device is in the middle of being closed, then block
	 * until it's done, and then try again.
	 */
	if (tty_hung_up_p(filp) ||
	    (info->flags & ASYNC_CLOSING)) {
		if (info->flags & ASYNC_CLOSING)
			interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
		return ((info->flags & ASYNC_HUP_NOTIFY) ?
			-EAGAIN : -ERESTARTSYS);
#else
		return -EAGAIN;
#endif
	}

	/*
	 * If this is a callout device, then just make sure the normal
	 * device isn't being used.
	 */
	if (tty->driver.subtype == SERIAL_TYPE_CALLOUT) {
		if (info->flags & ASYNC_NORMAL_ACTIVE)
			return -EBUSY;
		if ((info->flags & ASYNC_CALLOUT_ACTIVE) &&
		    (info->flags & ASYNC_SESSION_LOCKOUT) &&
		    (info->session != current->session))
		    return -EBUSY;
		if ((info->flags & ASYNC_CALLOUT_ACTIVE) &&
		    (info->flags & ASYNC_PGRP_LOCKOUT) &&
		    (info->pgrp != current->pgrp))
		    return -EBUSY;
		info->flags |= ASYNC_CALLOUT_ACTIVE;
		return 0;
	}
	
	/*
	 * If non-blocking mode is set, or the port is not enabled,
	 * then make the check up front and then exit.
	 */
	if ((filp->f_flags & O_NONBLOCK) ||
	    (tty->flags & (1 << TTY_IO_ERROR))) {
		if (info->flags & ASYNC_CALLOUT_ACTIVE)
			return -EBUSY;
		info->flags |= ASYNC_NORMAL_ACTIVE;
		return 0;
	}

	if (info->flags & ASYNC_CALLOUT_ACTIVE) {
		if (state->normal_termios.c_cflag & CLOCAL)
			do_clocal = 1;
	} else {
		if (tty->termios->c_cflag & CLOCAL)
			do_clocal = 1;
	}
	
	/*
	 * Block waiting for the carrier detect and the line to become
	 * free (i.e., not in use by the callout).  While we are in
	 * this loop, state->count is dropped by one, so that
	 * mp_close() knows when to free things.  We restore it upon
	 * exit, either normal or abnormal.
	 */
	retval = 0;
	add_wait_queue(&info->open_wait, &wait);
	save_flags(flags); cli();
	if (!tty_hung_up_p(filp)) {
		extra_count = 1;
		state->count--;
	}
	restore_flags(flags);
	info->blocked_open++;
	while (1) {
		save_flags(flags); cli();
		if (!(info->flags & ASYNC_CALLOUT_ACTIVE) &&
		    (tty->termios->c_cflag & CBAUD))
			multi_out(info, UART_MCR,
				   multi_in(info, UART_MCR) |
				   (UART_MCR_DTR | UART_MCR_RTS));
		restore_flags(flags);
		set_current_state(TASK_INTERRUPTIBLE);
		if (tty_hung_up_p(filp) ||
		    !(info->flags & ASYNC_INITIALIZED)) {
#ifdef SERIAL_DO_RESTART
			if (info->flags & ASYNC_HUP_NOTIFY)
				retval = -EAGAIN;
			else
				retval = -ERESTARTSYS;	
#else
			retval = -EAGAIN;
#endif
			break;
		}
		if (!(info->flags & ASYNC_CALLOUT_ACTIVE) &&
		    !(info->flags & ASYNC_CLOSING) &&
		    (do_clocal || (multi_in(info, UART_MSR) &
				   UART_MSR_DCD)))
			break;
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}
		schedule();
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&info->open_wait, &wait);
	if (extra_count)
		state->count++;
	info->blocked_open--;
	if (retval)
		return retval;
	info->flags |= ASYNC_NORMAL_ACTIVE;
	return 0;
}

static int get_sb_async_struct(int line, struct sb_async_struct **ret_info)
{
	struct sb_async_struct *info;
	struct sb_serial_state *sstate;

	sstate = mp_table + line;
	sstate->count++;
	if (sstate->info) {
		*ret_info = sstate->info;
		return 0;
	}
	info = kmalloc(sizeof(struct sb_async_struct), GFP_KERNEL);
	if (!info) {
		sstate->count--;
		return -ENOMEM;
	}
	memset(info, 0, sizeof(struct sb_async_struct));
	init_waitqueue_head(&info->open_wait);
	init_waitqueue_head(&info->close_wait);
	init_waitqueue_head(&info->delta_msr_wait);
	info->magic = MPORT_MAGIC;
	info->port = sstate->port;
	info->flags = sstate->flags;
	info->io_type = sstate->io_type;
	info->iomem_base = sstate->iomem_base;
	info->iomem_reg_shift = sstate->iomem_reg_shift;
	info->xmit_fifo_size = sstate->xmit_fifo_size;
	info->line = line;
	info->tqueue.routine = do_softint;
	info->tqueue.data = info;
	info->state = sstate;
	if (sstate->info) {
		kfree(info);
		*ret_info = sstate->info;
		return 0;
	}
	*ret_info = sstate->info = info;
	return 0;
}

/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its async structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
int mp_open(struct tty_struct *tty, struct file * filp)
{
	struct sb_async_struct	*info;
	int 			retval, line;
	unsigned long		page;
	MOD_INC_USE_COUNT;
	line = MINOR(tty->device) - tty->driver.minor_start;
	if ((line < 0) || (line >= NR_PORTS)) {
		MOD_DEC_USE_COUNT;
		return -ENODEV;
	}
	retval = get_sb_async_struct(line, &info);
	if (retval) {
		MOD_DEC_USE_COUNT;
		return retval;
	}
	tty->driver_data = info;
	info->tty = tty;

#if (LINUX_VERSION_CODE > 0x20100)
	info->tty->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;
#endif

	if (!tmp_buf) {
		page = get_zeroed_page(GFP_KERNEL);
		if (!page) {
			MOD_DEC_USE_COUNT;
			return -ENOMEM;
		}
		if (tmp_buf)
			free_page(page);
		else
			tmp_buf = (unsigned char *) page;
	}

	/*
	 * If the port is the middle of closing, bail out now
	 */
	if (tty_hung_up_p(filp) ||
	    (info->flags & ASYNC_CLOSING)) {
		if (info->flags & ASYNC_CLOSING)
			interruptible_sleep_on(&info->close_wait);
		MOD_DEC_USE_COUNT;
#ifdef SERIAL_DO_RESTART
		return ((info->flags & ASYNC_HUP_NOTIFY) ?
			-EAGAIN : -ERESTARTSYS);
#else
		return -EAGAIN;
#endif
	}

	/*
	 * Start up serial port
	 */
	retval = startup(info);
	if (retval) {
		MOD_DEC_USE_COUNT;
		return retval;
	}

	retval = block_til_ready(tty, filp, info);
	if (retval) {
		MOD_DEC_USE_COUNT;
		return retval;
	}

	if ((info->state->count == 1) &&
	    (info->flags & ASYNC_SPLIT_TERMIOS)) {
		if (tty->driver.subtype == SERIAL_TYPE_NORMAL)
			*tty->termios = info->state->normal_termios;
		else 
			*tty->termios = info->state->callout_termios;
		change_speed(info, 0);
	}
	info->session = current->session;
	info->pgrp = current->pgrp;

	return 0;
}

/*
 * /proc fs routines....
 */

static inline int line_info(char *buf, struct sb_serial_state *state)
{
	struct sb_async_struct *info = state->info, scr_info;
	char	stat_buf[30], control, status;
	int	ret;
	unsigned long flags;
	struct mp_device_t	*sb_dev;

	sb_dev = state->device;

	ret = sprintf(buf, "%d: %s(rev %x) uart:%s port:%lX irq:%d",
			state->line, sb_dev->name, sb_dev->revision,
		       uart_config[state->type].name, 
			  state->port, state->irq);

      
	switch( state->baud_base / BASE_BAUD ){
		case 1 :
			ret += sprintf(buf+ret, " osc:1.8432Mhz");
			break;
		case 2 :
			ret += sprintf(buf+ret, " osc:3.6864Mhz");
			break;
		case 4 :
			ret += sprintf(buf+ret, " osc:7.3728Mhz");
			break;
		case 8 :
			ret += sprintf(buf+ret, " osc:14.7456Mhz");
			break;
	}


	if (!state->port || (state->type == PORT_UNKNOWN)) {
		ret += sprintf(buf+ret, "\n");
		return ret;
	}

	/*
	 * Figure out the current RS-232 lines
	 */
	if (!info) {
		info = &scr_info;	/* This is just for multi_{in,out} */

		info->magic = MPORT_MAGIC;
		info->port = state->port;
		info->flags = state->flags;
		info->quot = 0;
		info->tty = 0;
	}
	save_flags(flags); cli();
	status = multi_in(info, UART_MSR);
	control = info != &scr_info ? info->MCR : multi_in(info, UART_MCR);
	restore_flags(flags); 

	stat_buf[0] = 0;
	stat_buf[1] = 0;
	if (control & UART_MCR_RTS)
		strcat(stat_buf, "|RTS");
	if (status & UART_MSR_CTS)
		strcat(stat_buf, "|CTS");
	if (control & UART_MCR_DTR)
		strcat(stat_buf, "|DTR");
	if (status & UART_MSR_DSR)
		strcat(stat_buf, "|DSR");
	if (status & UART_MSR_DCD)
		strcat(stat_buf, "|CD");
	if (status & UART_MSR_RI)
		strcat(stat_buf, "|RI");

	if (info->quot) {
		ret += sprintf(buf+ret, " baud:%d",
			       state->baud_base / info->quot);
	}

	ret += sprintf(buf+ret, " tx:%d rx:%d",
		      state->icount.tx, state->icount.rx);

	if (state->icount.frame)
		ret += sprintf(buf+ret, " fe:%d", state->icount.frame);
	
	if (state->icount.parity)
		ret += sprintf(buf+ret, " pe:%d", state->icount.parity);
	
	if (state->icount.brk)
		ret += sprintf(buf+ret, " brk:%d", state->icount.brk);	

	if (state->icount.overrun)
		ret += sprintf(buf+ret, " oe:%d", state->icount.overrun);

#if 0
	if ( mp_devs[state->line].device_id == PCI_DEVICE_ID_MP4JA) {
		unsigned char IIR_STATE;
		IIR_STATE = inb(mp_devs[state->line].option_reg_addr + 0x08 + state->line);
			if((IIR_STATE & 0xf0) == 0x00)
				ret += sprintf(buf+ret," Type: RS232");
			else if ((IIR_STATE & 0xf0) == 0x10) {
				if((IIR_STATE & 0x03) == 0x00)
					ret += sprintf(buf+ret," Type: RS422(PTP)");
				else
					ret += sprintf(buf+ret," Type: RS422(MD)");
			}
			else if ((IIR_STATE & 0xf0) == 0x20) {
				if((IIR_STATE & 0x03) == 0x00)
					ret += sprintf(buf+ret," Type: RS485(NON-ECHO)");
				else
					ret += sprintf(buf+ret," Type: RS485(ECHO)");
			}
	}
#endif
	/*
	 * Last thing is the RS-232 status lines
	 */
	ret += sprintf(buf+ret, " %s\n", stat_buf+1);
	return ret;
}

int mp_read_proc(char *page, char **start, off_t off, int count,
		 int *eof, void *data)
{
	int i, len = 0, l;
	off_t	begin = 0;

	len += sprintf(page, " *** MultiPorts Information :1.1 driver:%s%s revision:%s\n",
		       multi_version, LOCAL_VERSTRING, multi_revdate);
	for (i = 0; i < NR_PORTS && len < 4000; i++) {
		l = line_info(page + len, &mp_table[i]);
		len += l;
		if (len+begin > off+count)
			goto done;
		if (len+begin < off) {
			begin += len;
			len = 0;
		}
	}
	*eof = 1;
done:
	if (off >= len+begin)
		return 0;
	*start = page + (off-begin);
	return ((count < begin+len-off) ? count : begin+len-off);
}

/*
 * ---------------------------------------------------------------------
 * mp_init() and friends
 *
 * mp_init() is called at boot-time to initialize the serial driver.
 * ---------------------------------------------------------------------
 */

/*
 * This routine prints out the appropriate serial driver version
 * number, and identifies which options were configured into this
 * driver.
 */


/*
 * This routine detect the IRQ of a serial port by clearing OUT2 when
 * no UART interrupt are requested (IER = 0) (*GPL*). This seems to work at
 * each time, as long as no other device permanently request the IRQ.
 * If no IRQ is detected, or multiple IRQ appear, this function returns 0.
 * The variable "state" and the field "state->port" should not be null.
 */
static unsigned detect_uart_irq (struct sb_serial_state * state)
{
	int irq;
	unsigned long irqs;
	unsigned char save_mcr, save_ier;
	struct sb_async_struct scr_info; /* multi_{in,out} because HUB6 */

	scr_info.magic = MPORT_MAGIC;
	scr_info.state = state;
	scr_info.port = state->port;
	scr_info.flags = state->flags;
	scr_info.io_type = state->io_type;
	scr_info.iomem_base = state->iomem_base;
	scr_info.iomem_reg_shift = state->iomem_reg_shift;

	/* forget possible initially masked and pending IRQ */
	probe_irq_off(probe_irq_on());
	save_mcr = multi_in(&scr_info, UART_MCR);
	save_ier = multi_in(&scr_info, UART_IER);
	multi_out(&scr_info, UART_MCR, UART_MCR_OUT1 | UART_MCR_OUT2);
	
	irqs = probe_irq_on();
	multi_out(&scr_info, UART_MCR, 0);
	udelay (10);
	if (state->flags & ASYNC_FOURPORT)  {
		multi_out(&scr_info, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS);
	} else {
		multi_out(&scr_info, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2);
	}
	multi_out(&scr_info, UART_IER, 0x0f);	/* enable all intrs */
	(void)multi_in(&scr_info, UART_LSR);
	(void)multi_in(&scr_info, UART_RX);
	(void)multi_in(&scr_info, UART_IIR);
	(void)multi_in(&scr_info, UART_MSR);
	multi_out(&scr_info, UART_TX, 0xFF);
	udelay (20);
	irq = probe_irq_off(irqs);

	multi_out(&scr_info, UART_MCR, save_mcr);
	multi_out(&scr_info, UART_IER, save_ier);
	return (irq > 0)? irq : 0;
}

/*
 * This is a quickie test to see how big the FIFO is.
 * It doesn't work at all the time, more's the pity.
 */
static int size_fifo(struct sb_async_struct *info)
{
	unsigned char old_fcr, old_mcr, old_dll, old_dlm;
	int count;

	old_fcr = multi_in(info, UART_FCR);
	old_mcr = multi_in(info, UART_MCR);
	multi_out(info, UART_FCR, UART_FCR_ENABLE_FIFO |
		    UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	multi_out(info, UART_MCR, UART_MCR_LOOP);
	multi_out(info, UART_LCR, UART_LCR_DLAB);
	old_dll = multi_in(info, UART_DLL);
	old_dlm = multi_in(info, UART_DLM);
	multi_out(info, UART_DLL, 0x01);
	multi_out(info, UART_DLM, 0x00);
	multi_out(info, UART_LCR, 0x03);
	for (count = 0; count < 256; count++)
		multi_out(info, UART_TX, count);
	mdelay(20);
	for (count = 0; (multi_in(info, UART_LSR) & UART_LSR_DR) &&
	     (count < 256); count++)
		multi_in(info, UART_RX);
	multi_out(info, UART_FCR, old_fcr);
	multi_out(info, UART_MCR, old_mcr);
	multi_out(info, UART_LCR, UART_LCR_DLAB);
	multi_out(info, UART_DLL, old_dll);
	multi_out(info, UART_DLM, old_dlm);

	return count;
}


/*
 * This routine is called by mp_init() to initialize a specific serial
 * port.  It determines what type of UART chip this serial port is
 * using: 8250, 16450, 16550, 16550A.  The important question is
 * whether or not this UART is a 16550A or not, since this will
 * determine whether or not we can use its FIFO features or not.
 */
static void autoconfig(struct sb_serial_state * state)
{
	unsigned char status1, scratch, scratch2, scratch3;
	unsigned char save_lcr, save_mcr;
	struct sb_async_struct *info, scr_info;
	unsigned long flags;
	
	unsigned char u_type;
	unsigned char b_ret = 0;

	state->type = PORT_UNKNOWN;

	if (!CONFIGURED_SERIAL_PORT(state)){
		return;
	}
		
	info = &scr_info;	/* This is just for multi_{in,out} */

	info->magic = MPORT_MAGIC;
	info->state = state;
	info->port = state->port;
	info->flags = state->flags;
	info->io_type = state->io_type;
	info->iomem_base = state->iomem_base;
	info->iomem_reg_shift = state->iomem_reg_shift;

	save_flags(flags); cli();
	
	if (!(state->flags & ASYNC_BUGGY_UART) &&
	    !state->iomem_base) {
		/*
		 * Do a simple existence test first; if we fail this,
		 * there's no point trying anything else.
		 * 
		 * 0x80 is used as a nonsense port to prevent against
		 * false positives due to ISA bus float.  The
		 * assumption is that 0x80 is a non-existent port;
		 * which should be safe since include/asm/io.h also
		 * makes this assumption.
		 */
		scratch = multi_in(info, UART_IER);
		multi_out(info, UART_IER, 0);
#ifdef __i386__
		outb(0xff, 0x080);
#endif
		scratch2 = multi_in(info, UART_IER);
		multi_out(info, UART_IER, 0x0F);
#ifdef __i386__
		outb(0, 0x080);
#endif
		scratch3 = multi_in(info, UART_IER);
		multi_out(info, UART_IER, scratch);
		if (scratch2 || scratch3 != 0x0F) {
			restore_flags(flags);
			return;		/* We failed; there's nothing here */
		}
	}

	save_mcr = multi_in(info, UART_MCR);
	save_lcr = multi_in(info, UART_LCR);

	/* 
	 * Check to see if a UART is really there.  Certain broken
	 * internal modems based on the Rockwell chipset fail this
	 * test, because they apparently don't implement the loopback
	 * test mode.  So this test is skipped on the COM 1 through
	 * COM 4 ports.  This *should* be safe, since no board
	 * manufacturer would be stupid enough to design a board
	 * that conflicts with COM 1-4 --- we hope!
	 */
	if (!(state->flags & ASYNC_SKIP_TEST)) {
		multi_out(info, UART_MCR, UART_MCR_LOOP | 0x0A);
		status1 = multi_in(info, UART_MSR) & 0xF0;
		multi_out(info, UART_MCR, save_mcr);
		if (status1 != 0x90) {
			restore_flags(flags);
			return;
		}
	}
	multi_out(info, UART_LCR, 0xBF); /* set up for StarTech test */
	multi_out(info, UART_EFR, 0);	/* EFR is the same as FCR */
	multi_out(info, UART_LCR, 0);
	multi_out(info, UART_FCR, UART_FCR_ENABLE_FIFO);
	scratch = multi_in(info, UART_IIR) >> 6;
	
	b_ret = read_option_register(state, MP_OPTR_DIR0);

	u_type = (b_ret & 0xf0) >> 4;
	
	switch (u_type) {
			
			case DIR_UART_16C550:
				info->state->type = PORT_16C55X;
				break;
			case DIR_UART_16C1050:
				info->state->type = PORT_16C105X;
				break;
			default:	
				info->state->type = PORT_UNKNOWN;
				break;
	}
	
	multi_out(info, UART_LCR, save_lcr);

	state->xmit_fifo_size =	uart_config[state->type].dfl_xmit_fifo_size;


	
	if (state->type == PORT_UNKNOWN) {
		restore_flags(flags);
		return;
	}

//	if (info->port) {
//			request_region(info->port,8,"serial(auto)");
//	}

	/*
	 * Reset the UART.
	 */
	multi_out(info, UART_MCR, save_mcr);
	multi_out(info, UART_FCR, (UART_FCR_ENABLE_FIFO |
				     UART_FCR_CLEAR_RCVR |
				     UART_FCR_CLEAR_XMIT));
	multi_out(info, UART_FCR, 0);
	(void)multi_in(info, UART_RX);
	multi_out(info, UART_IER, 0);
	
	restore_flags(flags);
}

int register_serial(struct serial_struct *req);
void unregister_serial(int line);

#if (LINUX_VERSION_CODE > 0x20100)
EXPORT_SYMBOL(register_serial);
EXPORT_SYMBOL(unregister_serial);
#else
static struct symbol_table multi_syms = {
#include <linux/symtab_begin.h>
	X(register_serial),
	X(unregister_serial),
#include <linux/symtab_end.h>
};
#endif

static int init_mp_dev(struct pci_dev *pcidev, mppcibrd_t brd)
{
	static struct mp_device_t * sbdev = mp_devs;
	unsigned long itr_addr;

	sbdev->device_id = brd.device_id;
	pci_read_config_byte(pcidev, PCI_CLASS_REVISION, &(sbdev->revision));
	sbdev->name = brd.name;
	sbdev->uart_access_addr = pcidev->resource[0].start & PCI_BASE_ADDRESS_IO_MASK;
	sbdev->option_reg_addr = pcidev->resource[1].start & PCI_BASE_ADDRESS_IO_MASK;
	sbdev->irq = pcidev->irq;

	/* codes which is specific to each board*/
	switch(brd.device_id){
		case PCI_DEVICE_ID_MP1 :
				sbdev->nr_ports = 1;
				break;
		case PCI_DEVICE_ID_MP2 :
				sbdev->nr_ports = 2;
				break;
		case PCI_DEVICE_ID_MP4 :
				sbdev->nr_ports = 4;
				if(sbdev->revision == 0x91){
					sbdev->reserved_addr[0] = pcidev->resource[0].start & PCI_BASE_ADDRESS_IO_MASK;
					outb(0x03 , sbdev->reserved_addr[0] + 0x01);
					outb(0x03 , sbdev->reserved_addr[0] + 0x02);
					outb(0x01 , sbdev->reserved_addr[0] + 0x20);
					outb(0x00 , sbdev->reserved_addr[0] + 0x21);
					request_region(sbdev->reserved_addr[0], 32, sbdev->name);
					sbdev->uart_access_addr = pcidev->resource[1].start & PCI_BASE_ADDRESS_IO_MASK;
					sbdev->option_reg_addr = pcidev->resource[2].start & PCI_BASE_ADDRESS_IO_MASK;
				}
				break;
		case PCI_DEVICE_ID_MP8 :
				sbdev->nr_ports = 8;
				break;
		case PCI_DEVICE_ID_MP32 :
				{
					int portnum_hex=0;
					portnum_hex = inb(sbdev->option_reg_addr);  
					sbdev->nr_ports = ((portnum_hex/16)*10) + (portnum_hex % 16);
				}
				break;
	}

	request_region(sbdev->uart_access_addr, 8*sbdev->nr_ports, sbdev->name);
	request_region(sbdev->option_reg_addr, 32, sbdev->name);

	NR_DEV++;
	NR_PORTS += sbdev->nr_ports;

	/* Enable PCI interrupt */
	itr_addr = sbdev->option_reg_addr + ITR_MASK_REG;
	outb(0xff,itr_addr);
	outb(0xff,itr_addr+1);
	outb(0xff,itr_addr+2);
	outb(0xff,itr_addr+3);
	sbdev++;

	return 0;
}

static int init_mp_table(struct mp_device_t * sbdev)
{
	static int n = 0;
	struct sb_serial_state * state;
	int	i,j;
	int osc;
	unsigned char b_ret = 0;

	state = mp_table+n;

	for(i=0; i<sbdev->nr_ports; i++,n++,state++){
		state->magic = 0;

		/* get baud_base */
		state->baud_base = BASE_BAUD;
		osc = inb(sbdev->option_reg_addr + 4 + i/8) & 0x0f;
		for(j=0; j < osc; j++)
			state->baud_base *= 2;

		state->port = sbdev->uart_access_addr + 8*i;
		state->irq = sbdev->irq;
		state->flags = STD_COM_FLAGS;
		state->info = NULL;
		state->interface_config_addr = sbdev->option_reg_addr + 0x08 + i/8;
		state->option_base_addr = sbdev->option_reg_addr;
		
		state->device = sbdev;
		switch(sbdev->device_id){
			case PCI_DEVICE_ID_MP1 :
				break;
			case PCI_DEVICE_ID_MP2 :
				break;
			case PCI_DEVICE_ID_MP4 :
				if ( sbdev->revision == 0x91 ){
					state->interface_config_addr = sbdev->option_reg_addr + 0x08 + i;
				}
				break;
			case PCI_DEVICE_ID_MP8 :
				break;
			case PCI_DEVICE_ID_MP32 :
				break;
		}
		b_ret = read_option_register(state,(MP_OPTR_IIR0 + i/8));
		if(IIR_RS232 == (b_ret & IIR_RS232))
		{
			state->interface = RS232;
		}
		if(IIR_RS422 == (b_ret & IIR_RS422))
		{
			state->interface = RS422PTP;
		}
		if(IIR_RS485 == (b_ret & IIR_RS485))
		{
			state->interface = RS485NE;
		}
	}

	return 0;
}

static void print_info(void)
{
	int i,n=0,port,IIR;

	printk("==============================================\n");
	printk("      MultiPorts/PCI Board Installation      \n");
	printk("     version: %s  revision: %s\n",multi_version,multi_revdate);
	printk("        email :  <tech@mp.com>           \n");
	printk("==============================================\n");
	printk(" %d board(s) installed.   %d ports availabe. \n",NR_DEV,NR_PORTS);

	for(i=0; i<NR_DEV;i++){
		printk(" Board No.%d %s (rev %x) ttyMP%d ~ ttyMP%d using IRQ%d\n",
				i, mp_devs[i].name, mp_devs[i].revision, n , (n + mp_devs[i].nr_ports)-1, mp_devs[i].irq);

		for (port=0; port < mp_devs[i].nr_ports; port++) 
		{
			IIR = inb(mp_devs[i].option_reg_addr + 0x08 + port);
			if((IIR & 0xf0) == 0x00)
				printk(" %d port --- RS232\n", port+1);
			else if ((IIR & 0xf0) == 0x10) {
				if((IIR & 0x03) == 0x00)
					printk(" %d port --- RS422 and Point to Point mode\n", port+1);
				else
					printk(" %d port --- RS422 and Multi Drop mode\n", port+1);
			}
			else if ((IIR & 0xf0) == 0x20) {
				if((IIR & 0x03) == 0x00)
					printk(" %d port --- RS485 and Non Echo mode\n", port+1); 			
				else
					printk(" %d port --- RS485 and Echo mode\n", port+1);
			}
		}
		n += mp_devs[i].nr_ports;
	}
}

/*
 * The serial driver boot-time initialization code!
 */
static int __init mp_init(void)
{
	int i;
	struct sb_serial_state * state;
	struct pci_dev	*dev = NULL;

	////////////////////////

	/* find PCI bios and PCI device*/
	if(!pcibios_present()){
		printk("No PCI BIOS found!\n");
		return -ENODEV;
	}

	for( i=0; i< mp_nrpcibrds; i++){
		while( (dev = pci_find_device(mp_pciboards[i].vendor_id, mp_pciboards[i].device_id, dev) ) ) {
			init_mp_dev(dev, mp_pciboards[i]);
		}
	}

	if(!NR_PORTS){
		printk("ERROR : Multiport PCI board is not fount!\n");
		return -ENODEV;
	}

	/* set up IRQ structure */
	for (i = 0; i < NR_IRQS; i++) {
		IRQ_ports[i] = 0;
		IRQ_timeout[i] = 0;
	}

	init_bh(MP_BH, do_multi_bh);
	init_timer(&multi_timer);
	multi_timer.function = mp_timer;
	mod_timer(&multi_timer, jiffies + RS_STROBE_TIME);

	/* allocate and initialize mp_table, multi_table,multi_termios,multi_termios_locked */
	mp_table = (struct sb_serial_state *) kmalloc(sizeof(struct serial_state)*NR_PORTS , GFP_KERNEL);
	memset(mp_table, 0, sizeof(struct sb_serial_state)*NR_PORTS);
	for(i=0; i< NR_DEV; i++)
		init_mp_table(&(mp_devs[i]));

	multi_table  = (struct tty_struct **) kmalloc(sizeof(struct tty_struct *)*NR_PORTS , GFP_KERNEL);
	memset(multi_table, 0, sizeof(struct tty_struct *)*NR_PORTS);

	multi_termios = (struct termios **) kmalloc(sizeof(struct termios *)*NR_PORTS , GFP_KERNEL);
	memset(multi_termios, 0, sizeof(struct termios *)*NR_PORTS);

	multi_termios_locked = (struct termios **) kmalloc(sizeof(struct termios *)*NR_PORTS , GFP_KERNEL);
	memset(multi_termios_locked, 0, sizeof(struct termios *)*NR_PORTS);

	/////////////////////

	for (i = 0; i < NR_IRQS; i++) {
		IRQ_ports[i] = 0;
		IRQ_timeout[i] = 0;
	}

	/* Initialize the tty_driver structure */

	memset(&multi_driver, 0, sizeof(struct tty_driver));
	multi_driver.magic = TTY_DRIVER_MAGIC;
	multi_driver.driver_name = "multiports";
	multi_driver.major = TTY_MP_MAJOR;
	multi_driver.minor_start = 0;
	multi_driver.name = "ttyMP";
	multi_driver.name_base = 0;
	multi_driver.num = NR_PORTS;
	multi_driver.type = TTY_DRIVER_TYPE_SERIAL;
	multi_driver.subtype = SERIAL_TYPE_NORMAL;
	multi_driver.init_termios = tty_std_termios;
	multi_driver.init_termios.c_cflag =
		B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	multi_driver.flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_NO_DEVFS;
	multi_driver.refcount = &multi_refcount;
	multi_driver.table = multi_table;
	multi_driver.termios = multi_termios;
	multi_driver.termios_locked = multi_termios_locked;

	multi_driver.open = mp_open;
	multi_driver.close = mp_close;
	multi_driver.write = mp_write;
	multi_driver.put_char = mp_put_char;
	multi_driver.flush_chars = mp_flush_chars;
	multi_driver.write_room = mp_write_room;
	multi_driver.chars_in_buffer = mp_chars_in_buffer;
	multi_driver.flush_buffer = mp_flush_buffer;
	multi_driver.ioctl = mp_ioctl;
	multi_driver.throttle = mp_throttle;
	multi_driver.unthrottle = mp_unthrottle;
	multi_driver.set_termios = mp_set_termios;
	multi_driver.stop = mp_stop;
	multi_driver.start = mp_start;
	multi_driver.hangup = mp_hangup;
	multi_driver.break_ctl = mp_break;
	multi_driver.send_xchar = mp_send_xchar;
	multi_driver.wait_until_sent = mp_wait_until_sent;
	multi_driver.read_proc = mp_read_proc;
	
	/*
	 * The callout device is just like normal device except for
	 * major number and the subtype code.
	 */
	callout_driver = multi_driver;
	callout_driver.name = "cuaMP";
	callout_driver.major = CUA_MP_MAJOR;
	callout_driver.subtype = SERIAL_TYPE_CALLOUT;
	callout_driver.read_proc = 0;
	callout_driver.proc_entry = 0;

	if (tty_register_driver(&multi_driver))
		panic("Couldn't register multiports driver\n");
	if (tty_register_driver(&callout_driver))
		panic("Couldn't register callout driver\n");
	
	for (i = 0, state = mp_table; i < NR_PORTS; i++,state++) {
		state->magic = SSTATE_MAGIC;
		state->line = i;
		state->type = PORT_UNKNOWN;
		state->custom_divisor = 0;
		state->close_delay = 5*HZ/10;
		state->closing_wait = 30*HZ;
		state->callout_termios = callout_driver.init_termios;
		state->normal_termios = multi_driver.init_termios;
		state->icount.cts = state->icount.dsr = 
			state->icount.rng = state->icount.dcd = 0;
		state->icount.rx = state->icount.tx = 0;
		state->icount.frame = state->icount.parity = 0;
		state->icount.overrun = state->icount.brk = 0;
		state->irq = irq_cannonicalize(state->irq);
		if (state->flags & ASYNC_BOOT_AUTOCONF)
			autoconfig(state);
	}

	print_info();

	return 0;
}

/*
 * This is for use by architectures that know their serial port
 * attributes only at run time. Not to be invoked after mp_init().
 */

int __init early_multi_setup(struct serial_struct *req)
{
	int i = req->line;

	if (i >= NR_IRQS)
		return(-ENOENT);
	mp_table[i].magic = 0;
	mp_table[i].baud_base = req->baud_base;
	mp_table[i].port = req->port;
	if (HIGH_BITS_OFFSET)
		mp_table[i].port += (unsigned long) req->port_high << 
							HIGH_BITS_OFFSET;
	mp_table[i].irq = req->irq;
	mp_table[i].flags = req->flags;
	mp_table[i].close_delay = req->close_delay;
	mp_table[i].io_type = req->io_type;
	mp_table[i].hub6 = req->hub6;
	mp_table[i].iomem_base = req->iomem_base;
	mp_table[i].iomem_reg_shift = req->iomem_reg_shift;
	mp_table[i].type = req->type;
	mp_table[i].xmit_fifo_size = req->xmit_fifo_size;
	mp_table[i].custom_divisor = req->custom_divisor;
	mp_table[i].closing_wait = req->closing_wait;
	return(0);
}

/*
 * register_serial and unregister_serial allows for 16x50 serial ports to be
 * configured at run-time, to support PCMCIA modems.
 */
 
/**
 *	register_serial - configure a 16x50 serial port at runtime
 *	@req: request structure
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use an error is returned. If the port
 *	is not currently in the table it is added.
 *
 *	The port is then probed and if neccessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */

int register_serial(struct serial_struct *req)
{
	int i;
	unsigned long flags;
	struct sb_serial_state *state;
	struct sb_async_struct *info;
	unsigned long port;

	port = req->port;
	if (HIGH_BITS_OFFSET)
		port += (unsigned long) req->port_high << HIGH_BITS_OFFSET;

	save_flags(flags); cli();
	for (i = 0; i < NR_PORTS; i++) {
		if ((mp_table[i].port == port) &&
				(mp_table[i].iomem_base == req->iomem_base))
			break;
	}
	if (i == NR_PORTS) {
		for (i = 0; i < NR_PORTS; i++)
			if ((mp_table[i].type == PORT_UNKNOWN) &&
					(mp_table[i].count == 0))
				break;
	}
	if (i == NR_PORTS) {
		restore_flags(flags);
		return -1;
	}
	state = &mp_table[i];
	if (mp_table[i].count) {
		restore_flags(flags);
		printk("Couldn't configure serial #%d (port=%ld,irq=%d): "
				"device already open\n", i, port, req->irq);
		return -1;
	}
	state->irq = req->irq;
	state->port = port;
	state->flags = req->flags;
	state->io_type = req->io_type;
	state->iomem_base = req->iomem_base;
	state->iomem_reg_shift = req->iomem_reg_shift;
	if (req->baud_base)
		state->baud_base = req->baud_base;
	if ((info = state->info) != NULL) {
		info->port = port;
		info->flags = req->flags;
		info->io_type = req->io_type;
		info->iomem_base = req->iomem_base;
		info->iomem_reg_shift = req->iomem_reg_shift;
	}
	autoconfig(state);
	if (state->type == PORT_UNKNOWN) {
		restore_flags(flags);
		printk("register_serial(): autoconfig failed\n");
		return -1;
	}
	restore_flags(flags);

	if ((state->flags & ASYNC_AUTO_IRQ) && CONFIGURED_SERIAL_PORT(state))
		state->irq = detect_uart_irq(state);

	printk(KERN_INFO "ttyMP%02d at %s 0x%04lx (irq = %d) is a %s\n",
			state->line,
			state->iomem_base ? "iomem" : "port",
			state->iomem_base ? (unsigned long)state->iomem_base :
			state->port, state->irq, uart_config[state->type].name);
	tty_register_devfs(&multi_driver, 0,
			multi_driver.minor_start + state->line); 
	tty_register_devfs(&callout_driver, 0,
			callout_driver.minor_start + state->line);
	return state->line;
}

/**
 *	unregister_serial - deconfigure a 16x50 serial port
 *	@line: line to deconfigure
 *
 *	The port specified is deconfigured and its resources are freed. Any
 *	user of the port is disconnected as if carrier was dropped. Line is
 *	the port number returned by register_serial().
 */

void unregister_serial(int line)
{
	unsigned long flags;
	struct sb_serial_state *state = &mp_table[line];

	save_flags(flags); cli();
	if (state->info && state->info->tty)
		tty_hangup(state->info->tty);
	state->type = PORT_UNKNOWN;
	printk(KERN_INFO "ttyMP%02d unloaded\n", state->line);
	/* These will be hidden, because they are devices that will no longer
	 * be available to the system. (ie, PCMCIA modems, once ejected)
	 */
	tty_unregister_devfs(&multi_driver,
			multi_driver.minor_start + state->line);
	tty_unregister_devfs(&callout_driver,
			callout_driver.minor_start + state->line);
	restore_flags(flags);
}

static void release_mp_dev(void)
{
	int i;
	unsigned long itr_addr;
	struct mp_device_t	*sbdev;

	for(i=0; i< NR_DEV; i++){
		sbdev = &(mp_devs[i]);
		/* codes which is specific to each board*/
		switch(sbdev->device_id){
			case PCI_DEVICE_ID_MP1 :
				break;
			case PCI_DEVICE_ID_MP2 :
				break;
			case PCI_DEVICE_ID_MP4 :
				if(sbdev->revision == 0x91)
					release_region(sbdev->reserved_addr[0], 32);
				break;
			case PCI_DEVICE_ID_MP8 :
				break;
			case PCI_DEVICE_ID_MP32 :
				break;
		}

		release_region(sbdev->uart_access_addr, 8*sbdev->nr_ports);
		release_region(sbdev->option_reg_addr, 32);


		/* Disable PCI interrupt */
		itr_addr = sbdev->option_reg_addr + ITR_MASK_REG;
		outb(0xff,itr_addr);
		outb(0xff,itr_addr+1);
		outb(0xff,itr_addr+2);
		outb(0xff,itr_addr+3);
	}

}

static void __exit mp_fini(void) 
{
	unsigned long flags;
	int e1, e2;
	int i;
	struct sb_async_struct *info;

	del_timer_sync(&multi_timer);
	save_flags(flags); cli();
	remove_bh(MP_BH);
	if ((e1 = tty_unregister_driver(&multi_driver)))
		printk("multiports: failed to unregister multiports driver (%d)\n",
				e1);
	if ((e2 = tty_unregister_driver(&callout_driver)))
		printk("multiports: failed to unregister callout driver (%d)\n", 
				e2);
	restore_flags(flags);

	release_mp_dev();

	for (i = 0; i < NR_PORTS; i++) {
		if ((info = mp_table[i].info)) {
			mp_table[i].info = NULL;
			kfree(info);
		}
		//	if ((mp_table[i].type != PORT_UNKNOWN) && mp_table[i].port) {
		//			release_region(mp_table[i].port, 8);
		//	}
	}
	kfree(mp_table);
	kfree(multi_table);
	kfree(multi_termios);
	kfree(multi_termios_locked);
	if (tmp_buf) {
		unsigned long pg = (unsigned long) tmp_buf;
		tmp_buf = NULL;
		free_page(pg);
	}
}

module_init(mp_init);
module_exit(mp_fini);
MODULE_DESCRIPTION("Multiport PCI driver");
MODULE_AUTHOR("Systembase Co,. LTD.");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,10)
MODULE_LICENSE("GPL");
#endif


/*
 * ------------------------------------------------------------
 * Serial console driver
 * ------------------------------------------------------------
 */

/*
   Local variables:
   compile-command: "gcc -D__KERNEL__ -I../../include -Wall -Wstrict-prototypes -O2 -fomit-frame-pointer -fno-strict-aliasing -pipe -fno-strength-reduce -march=i586 -DMODULE -DMODVERSIONS -include ../../include/linux/modversions.h   -DEXPORT_SYMTAB -c serial.c"
End:
 */
