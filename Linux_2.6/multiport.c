/*
 *  multiport.c
 *
 *  Driver core for serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright 1999 ARM Limited
 *  Copyright (C) 2000-2001 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/version.h>
#define VERSION_CODE(ver,rel,seq)   ((ver << 16) | (rel << 8) | seq)
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,18))
#include <linux/config.h>
#endif
#include <linux/vermagic.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/mca.h>		//FEDORA 4
#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <asm/serial.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>

#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>

#include "mp_register.h"
#include "multiport.h"

#undef MP_DEBUG

#ifdef MP_DEBUG
#define DPRINTK(x...)	printk(x)
#else
#define DPRINTK(x...)	do { } while (0)
#endif

#ifdef MP_DEBUG
#define DEBUG_AUTOCONF(fmt...)  printk(fmt)
#else
#define DEBUG_AUTOCONF(fmt...)  do { } while (0)
#endif

#ifdef MP_DEBUG
#define DEBUG_INTR(fmt...)  printk(fmt)
#else
#define DEBUG_INTR(fmt...)  do { } while (0)
#endif

///////////// TSUM //////////////////
static char *multi_version = "3.0";
static char *multi_revdate = "2009-06-22"; 

#if defined(__i386__) && (defined(CONFIG_M386) || defined(CONFIG_M486))
#define SERIAL_INLINE
#endif
#ifdef SERIAL_INLINE
#define _INLINE_ inline
#else
#define _INLINE_
#endif


struct mp_device_t {
	unsigned short  device_id;
	unsigned char   revision;
	char            *name;
	unsigned long   uart_access_addr;
	unsigned long   option_reg_addr;
	unsigned long   reserved_addr[4];
	int             irq;
	int             nr_ports;
};
static struct mp_device_t mp_devs[MAX_MP_DEV];
typedef struct mppcibrd {
	char            *name;
	unsigned short  vendor_id;
	unsigned short  device_id;
} mppcibrd_t;

static mppcibrd_t mp_pciboards[] = {
	{ "Multi-1 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP1} ,
	{ "Multi-2 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP2} ,
	{ "Multi-4 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP4} ,
	{ "Multi-8 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP8} ,
	{ "Multi-32 PCI", PCI_VENDOR_ID_MULTIPORT , PCI_DEVICE_ID_MP32} ,
};
static int mp_nrpcibrds = sizeof(mp_pciboards)/sizeof(mppcibrd_t);
static int NR_BOARD=0;
static int NR_PORTS=0;

struct mp_port {
	struct uart_port port;

	struct timer_list   timer;      /* "no irq" timer */
	struct list_head    list;       /* ports on this IRQ */
	unsigned int        capabilities;   /* port capabilities */
	unsigned short      rev;
	unsigned char       acr;
	unsigned char       ier;
	unsigned char       lcr;
	unsigned char       mcr;
	unsigned char       mcr_mask;   /* mask of user bits */
	unsigned char       mcr_force;  /* mask of forced bits */
	unsigned char       lsr_break_flag;

	/*
	 * We provide a per-port pm hook.
	 */
	void            (*pm)(struct uart_port *port,
			unsigned int state, unsigned int old);
	struct mp_device_t *device;
	unsigned long interface_config_addr;
	unsigned long option_base_addr;
	unsigned char	interface;
};

static struct mp_port multi_ports[128];

struct irq_info {
	spinlock_t      lock;
	struct list_head    *head;
};

static struct irq_info irq_lists[NR_IRQS];




static const struct serial_uart_config uart_config[] = {
	{ "unknown",    1,  0 },
	{ "16550A", 16, UART_CLEAR_FIFO | UART_USE_FIFO },
	{ "SB16C1050",    128,    UART_CLEAR_FIFO | UART_USE_FIFO | UART_STARTECH },
};

#define PASS_LIMIT  256
#define is_real_interrupt(irq)  ((irq) != 0)

#define PROBE_ANY   (~0)

unsigned int share_irqs = 1;

//////////////////////////////////////


/*
 * This is used to lock changes in serial line configuration.
 */
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,12))
static DECLARE_MUTEX(mp_mutex);
#define MP_MUTEX_LOCK(x) down(&(x))
#define MP_MUTEX_UNLOCK(x) up(&(x))
#define MP_STATE_LOCK(x) down(&((x)->sem))
#define MP_STATE_UNLOCK(x) up(&((x)->sem))

#else
static DEFINE_MUTEX(mp_mutex);
#define MP_MUTEX_LOCK(x) mutex_lock(&(x)) 
#define MP_MUTEX_UNLOCK(x) mutex_unlock(&(x)) 
#define MP_STATE_LOCK(x) mutex_lock(&((x)->mutex)) 
#define MP_STATE_UNLOCK(x) mutex_unlock(&((x)->mutex)) 
#endif

#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,19))
#define MP_TERMIOS  ktermios
#else
#define MP_TERMIOS  termios
#endif

#define UART_LSR_SPECIAL    0x1E

#define HIGH_BITS_OFFSET	((sizeof(long)-sizeof(int))*8)
#define uart_users(state)	((state)->count + ((state)->info ? (state)->info->blocked_open : 0))

static void mp_change_speed(struct uart_state *state, struct MP_TERMIOS *old_termios);
static void mp_wait_until_sent(struct tty_struct *tty, int timeout);
static void mp_change_pm(struct uart_state *state, int pm_state);
static void multi_shutdown(struct uart_port *port);
static int sb1054_get_register(struct uart_port * port, int page, int reg);
static int sb1054_set_register(struct uart_port * port, int page, int reg, int value);


//// access uart register ////
static _INLINE_ unsigned int serial_in(struct mp_port *mtpt, int offset)
{
	return inb(mtpt->port.iobase + offset);
}

static _INLINE_ void serial_out(struct mp_port *mtpt, int offset, int value)
{
	outb(value, mtpt->port.iobase + offset);
}

//// access option register ////
static _INLINE_ unsigned int read_option_register(struct mp_port *mtpt, int offset)
{
	return inb(mtpt->option_base_addr + offset);
}

static _INLINE_ void write_option_register(struct mp_port *mtpt, int offset, int value)
{
	outb(value, mtpt->option_base_addr + offset);
}

/* SB16C1050 read functions by John Lee */

static int sb1054_get_register(struct uart_port * port, int page, int reg)
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
			lcr = SB105X_GET_LCR(port);
			tmp = lcr | SB105X_LCR_DLAB;
			SB105X_PUT_LCR(port, tmp);

			tmp = SB105X_GET_LCR(port);

			ret = SB105X_GET_REG(port,reg);
			SB105X_PUT_LCR(port,lcr);
			break;
		case 2:
			mcr = SB105X_GET_MCR(port);
			tmp = mcr | SB105X_MCR_P2S;
			SB105X_PUT_MCR(port,tmp);

			ret = SB105X_GET_REG(port,reg);

			SB105X_PUT_MCR(port,mcr);
			break;
		case 3:
			lcr = SB105X_GET_LCR(port);
			tmp = lcr | SB105X_LCR_BF;
			SB105X_PUT_LCR(port,tmp);
			SB105X_PUT_REG(port,SB105X_PSR,SB105X_PSR_P3KEY);

			ret = SB105X_GET_REG(port,reg);

			SB105X_PUT_LCR(port,lcr);
			break;
		case 4:
			lcr = SB105X_GET_LCR(port);
			tmp = lcr | SB105X_LCR_BF;
			SB105X_PUT_LCR(port,tmp);
			SB105X_PUT_REG(port,SB105X_PSR,SB105X_PSR_P4KEY);

			ret = SB105X_GET_REG(port,reg);

			SB105X_PUT_LCR(port,lcr);
			break;
		default:
			printk(" error invalid page number \n");
			return -1;
	}

	return ret;
}

static int sb1054_set_register(struct uart_port * port, int page, int reg, int value)
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
			lcr = SB105X_GET_LCR(port);
			SB105X_PUT_LCR(port, lcr | SB105X_LCR_DLAB);

			SB105X_PUT_REG(port,reg,value);

			SB105X_PUT_LCR(port, lcr);
			ret = 0;
			break;
		case 2:
			mcr = SB105X_GET_MCR(port);
			SB105X_PUT_MCR(port, mcr | SB105X_MCR_P2S);

			SB105X_PUT_REG(port,reg,value);

			SB105X_PUT_MCR(port, mcr);
			ret = 0;
			break;
		case 3:
			lcr = SB105X_GET_LCR(port);
			SB105X_PUT_LCR(port, lcr | SB105X_LCR_BF);
			SB105X_PUT_PSR(port, SB105X_PSR_P3KEY);

			SB105X_PUT_REG(port,reg,value);

			SB105X_PUT_LCR(port, lcr);
			ret = 0;
			break;
		case 4:
			lcr = SB105X_GET_LCR(port);
			SB105X_PUT_LCR(port, lcr | SB105X_LCR_BF);
			SB105X_PUT_PSR(port, SB105X_PSR_P4KEY);

			SB105X_PUT_REG(port,reg,value);

			SB105X_PUT_LCR(port, lcr);
			ret = 0;
			break;
		default:
			printk(" error invalid page number \n");
			return -1;
	}

	return ret;
}

//// additional setting functions ////

static int set_deep_fifo(struct uart_port * port, int status)
{
	int afr_status = 0;
	afr_status = sb1054_get_register(port, PAGE_4, SB105X_AFR);

	if(status == ENABLE)
	{
		afr_status |= SB105X_AFR_AFEN;
	}
	else
	{
		afr_status &= ~SB105X_AFR_AFEN;
	}
	
	sb1054_set_register(port,PAGE_4,SB105X_AFR,afr_status);
	
	afr_status = sb1054_get_register(port, PAGE_4, SB105X_AFR);
	
	return afr_status;
}

static int set_auto_rts(struct uart_port *port, int status)
{
	int efr_status = 0;
	efr_status = sb1054_get_register(port, PAGE_3, SB105X_EFR);

	if(status == ENABLE)
	{
		efr_status |= SB105X_EFR_ARTS;
	}
	else
	{
		efr_status &= ~SB105X_EFR_ARTS;
	}
	sb1054_set_register(port,PAGE_3,SB105X_EFR,efr_status);
	
	efr_status = sb1054_get_register(port, PAGE_3, SB105X_EFR);
	
	return efr_status;
}

static int set_auto_cts(struct uart_port *port, int status)
{
	int efr_status = 0;
	efr_status = sb1054_get_register(port, PAGE_3, SB105X_EFR);

	if(status == ENABLE)
	{
		efr_status |= SB105X_EFR_ACTS;
	}
	else
	{
		efr_status &= ~SB105X_EFR_ACTS;
	}
	sb1054_set_register(port,PAGE_3,SB105X_EFR,efr_status);
	
	efr_status = sb1054_get_register(port, PAGE_3, SB105X_EFR);
	
	return efr_status;
	
}

/*
 * This routine is used by the interrupt handler to schedule processing in
 * the software interrupt portion of the driver.
 */
void mp_write_wakeup(struct uart_port *port)
{
	struct uart_info *info = port->info;
	tasklet_schedule(&info->tlet);
}

static void mp_stop(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,14))
		port->ops->stop_tx(port, 1);
#else
		port->ops->stop_tx(port);
#endif 
	spin_unlock_irqrestore(&port->lock, flags);
}

static void __mp_start(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;

	if (!uart_circ_empty(&state->info.xmit) && state->info.xmit.buf &&
			!tty->stopped && !tty->hw_stopped)
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,14))
			port->ops->start_tx(port, 1);
#else
			port->ops->start_tx(port);
#endif
}

static void mp_start(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	__mp_start(tty);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void mp_tasklet_action(unsigned long data)
{
	struct uart_state *state = (struct uart_state *)data;
	struct tty_struct *tty;

	tty = state->info.port.tty;
#if (LINUX_VERSION_CODE <  VERSION_CODE(2,6,9))
	if (tty) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc.write_wakeup)
		{
			tty->ldisc.write_wakeup(tty);
		}
		wake_up_interruptible(&tty->write_wait);
	}
#else
	tty_wakeup(tty);
#endif
}

	static inline void
mp_update_mctrl(struct uart_port *port, unsigned int set, unsigned int clear)
{
	unsigned long flags;
	unsigned int old;

	spin_lock_irqsave(&port->lock, flags);
	old = port->mctrl;
	port->mctrl = (old & ~clear) | set;
	if (old != port->mctrl)
		port->ops->set_mctrl(port, port->mctrl);
	spin_unlock_irqrestore(&port->lock, flags);
}

#define uart_set_mctrl(port,set)	mp_update_mctrl(port,set,0)
#define uart_clear_mctrl(port,clear)	mp_update_mctrl(port,0,clear)

/*
 * Startup the port.  This will be called once per open.  All calls
 * will be serialised by the per-port semaphore.
 */
static int mp_startup(struct uart_state *state, int init_hw)
{
	struct uart_info *info = &state->info;
	struct uart_port *port = state->port;
	unsigned long page;
	int retval = 0;

	if (info->flags & UIF_INITIALIZED)
		return 0;

	/*
	 * Set the TTY IO error marker - we will only clear this
	 * once we have successfully opened the port.  Also set
	 * up the tty->alt_speed kludge
	 */
	if (info->port.tty)
		set_bit(TTY_IO_ERROR, &info->port.tty->flags);

	if (port->type == PORT_UNKNOWN)
		return 0;

	/*
	 * Initialise and allocate the transmit and temporary
	 * buffer.
	 */
	if (!info->xmit.buf) {
		page = get_zeroed_page(GFP_KERNEL);
		if (!page)
			return -ENOMEM;

		info->xmit.buf = (unsigned char *) page;
		
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,10))
		info->tmpbuf = info->xmit.buf + UART_XMIT_SIZE;
		init_MUTEX(&info->tmpbuf_sem);
#endif
		uart_circ_clear(&info->xmit);
	}

	retval = port->ops->startup(port);
	if (retval == 0) {
		if (init_hw) {
			/*
			 * Initialise the hardware port settings.
			 */
			mp_change_speed(state, NULL);

			/*
			 * Setup the RTS and DTR signals once the
			 * port is open and ready to respond.
			 */
			if (info->port.tty->termios->c_cflag & CBAUD)
				uart_set_mctrl(port, TIOCM_RTS | TIOCM_DTR);
		}

		info->flags |= UIF_INITIALIZED;


		clear_bit(TTY_IO_ERROR, &info->port.tty->flags);
	}

	if (retval && capable(CAP_SYS_ADMIN))
		retval = 0;

	return retval;
}

/*
 * This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.  Calls to
 * mp_shutdown are serialised by the per-port semaphore.
 */
static void mp_shutdown(struct uart_state *state)
{
	struct uart_info *info = &state->info;
	struct uart_port *port = state->port;

	/*
	 * Set the TTY IO error marker
	 */
	if (info->port.tty)
		set_bit(TTY_IO_ERROR, &info->port.tty->flags);

	if (info->flags & UIF_INITIALIZED) {
		info->flags &= ~UIF_INITIALIZED;

		/*
		 * Turn off DTR and RTS early.
		 */
		if (!info->port.tty || (info->port.tty->termios->c_cflag & HUPCL))
			uart_clear_mctrl(port, TIOCM_DTR | TIOCM_RTS);

		/*
		 * clear delta_msr_wait queue to avoid mem leaks: we may free
		 * the irq here so the queue might never be woken up.  Note
		 * that we won't end up waiting on delta_msr_wait again since
		 * any outstanding file descriptors should be pointing at
		 * hung_up_tty_fops now.
		 */
		wake_up_interruptible(&info->delta_msr_wait);

		/*
		 * Free the IRQ and disable the port.
		 */
		port->ops->shutdown(port);

		/*
		 * Ensure that the IRQ handler isn't running on another CPU.
		 */
		synchronize_irq(port->irq);
	}
	/*
	 * kill off our tasklet
	 */
	tasklet_kill(&info->tlet);

	/*
	 * Free the transmit buffer page.
	 */
	if (info->xmit.buf) {
		free_page((unsigned long)info->xmit.buf);
		info->xmit.buf = NULL;
	}
}

/**
 *	mp_update_timeout - update per-port FIFO timeout.
 *	@port:  uart_port structure describing the port
 *	@cflag: termios cflag value
 *	@baud:  speed of the port
 *
 *	Set the port FIFO timeout value.  The @cflag value should
 *	reflect the actual hardware settings.
 */
	void
mp_update_timeout(struct uart_port *port, unsigned int cflag,
		unsigned int baud)
{
	unsigned int bits;

	/* byte size and parity */
	switch (cflag & CSIZE) {
		case CS5:
			bits = 7;
			break;
		case CS6:
			bits = 8;
			break;
		case CS7:
			bits = 9;
			break;
		default:
			bits = 10;
			break; // CS8
	}

	if (cflag & CSTOPB)
		bits++;
	if (cflag & PARENB)
		bits++;

	/*
	 * The total number of bits to be transmitted in the fifo.
	 */
	bits = bits * port->fifosize;

	/*
	 * Figure the timeout to send the above number of bits.
	 * Add .02 seconds of slop
	 */
	port->timeout = (HZ * bits) / baud + HZ/50;
}

EXPORT_SYMBOL(mp_update_timeout);

/**
 *	mp_get_baud_rate - return baud rate for a particular port
 *	@port: uart_port structure describing the port in question.
 *	@termios: desired termios settings.
 *	@old: old termios (or NULL)
 *	@min: minimum acceptable baud rate
 *	@max: maximum acceptable baud rate
 *
 *	Decode the termios structure into a numeric baud rate,
 *	taking account of the magic 38400 baud rate (with spd_*
 *	flags), and mapping the %B0 rate to 9600 baud.
 *
 *	If the new baud rate is invalid, try the old termios setting.
 *	If it's still invalid, we try 9600 baud.
 *
 *	Update the @termios structure to reflect the baud rate
 *	we're actually going to be using.
 */
	unsigned int
mp_get_baud_rate(struct uart_port *port, struct MP_TERMIOS *termios,
		struct MP_TERMIOS *old, unsigned int min, unsigned int max)
{
	unsigned int try, baud, altbaud = 38400;
	unsigned int flags = port->flags & UPF_SPD_MASK;

	if (flags == UPF_SPD_HI)
		altbaud = 57600;
	if (flags == UPF_SPD_VHI)
		altbaud = 115200;
	if (flags == UPF_SPD_SHI)
		altbaud = 230400;
	if (flags == UPF_SPD_WARP)
		altbaud = 460800;

	for (try = 0; try < 2; try++) {
		baud = tty_termios_baud_rate(termios);

		/*
		 * The spd_hi, spd_vhi, spd_shi, spd_warp kludge...
		 * Die! Die! Die!
		 */
		if (baud == 38400)
			baud = altbaud;

		/*
		 * Special case: B0 rate.
		 */
		if (baud == 0)
			baud = 9600;

		if (baud >= min && baud <= max)
			return baud;

		/*
		 * Oops, the quotient was zero.  Try again with
		 * the old baud rate if possible.
		 */
		termios->c_cflag &= ~CBAUD;
		if (old) {
			termios->c_cflag |= old->c_cflag & CBAUD;
			old = NULL;
			continue;
		}

		/*
		 * As a last resort, if the quotient is zero,
		 * default to 9600 bps
		 */
		termios->c_cflag |= B9600;
	}

	return 0;
}

EXPORT_SYMBOL(mp_get_baud_rate);

/**
 *	mp_get_divisor - return uart clock divisor
 *	@port: uart_port structure describing the port.
 *	@baud: desired baud rate
 *
 *	Calculate the uart clock divisor for the port.
 */
	unsigned int
mp_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	/*
	 * Old custom speed handling.
	 */
	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)
		quot = port->custom_divisor;
	else
		quot = port->uartclk / (16 * baud);

	return quot;
}
EXPORT_SYMBOL(mp_get_divisor);

	static void
mp_change_speed(struct uart_state *state, struct MP_TERMIOS *old_termios)
{
	struct tty_struct *tty = state->info.port.tty;
	struct uart_port *port = state->port;
	struct MP_TERMIOS *termios;

	/*
	 * If we have no tty, termios, or the port does not exist,
	 * then we can't set the parameters for this port.
	 */
	if (!tty || !tty->termios || port->type == PORT_UNKNOWN)
		return;

	termios = tty->termios;

	/*
	 * Set flags based on termios cflag
	 */
	if (termios->c_cflag & CRTSCTS)
		state->info.flags |= UIF_CTS_FLOW;
	else
		state->info.flags &= ~UIF_CTS_FLOW;

	if (termios->c_cflag & CLOCAL)
		state->info.flags &= ~UIF_CHECK_CD;
	else
		state->info.flags |= UIF_CHECK_CD;

	port->ops->set_termios(port, termios, old_termios);
}

	static inline void
__mp_put_char(struct uart_port *port, struct circ_buf *circ, unsigned char c)
{
	unsigned long flags;

	if (!circ->buf)
		return;

	spin_lock_irqsave(&port->lock, flags);
	if (uart_circ_chars_free(circ) != 0) {
		circ->buf[circ->head] = c;
		circ->head = (circ->head + 1) & (UART_XMIT_SIZE - 1);
	}
	spin_unlock_irqrestore(&port->lock, flags);
}

static void mp_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct uart_state *state = tty->driver_data;

	__mp_put_char(state->port, &state->info.xmit, ch);
}

static void mp_put_chars(struct tty_struct *tty)
{
	mp_start(tty);
}

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,10))
static int mp_write(struct tty_struct *tty, int from_user, const unsigned char * buf,
		int count)
#else
static int mp_write(struct tty_struct *tty, const unsigned char * buf,
		int count)
#endif
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port;
	struct circ_buf *circ;
	unsigned long flags;
	int c, ret = 0;

	/*
	 * This means you called this function _after_ the port was
	 * closed.  No cookie for you.
	 */

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,10))
	
	if (!state->info->xmit.buf)
		return 0;
	
	port = state->port;
	circ = &state->info->xmit;
		
	
	if ( from_user )
	{
		if (down_interruptible(&port->info->tmpbuf_sem))
		{
			printk("down interrupt error \n");
			return -EINTR;
		}

		while (1) 
		{
			int c1;
			c = CIRC_SPACE_TO_END(circ->head, circ->tail, UART_XMIT_SIZE);
			if (count < c)
				c = count;
			if (c <= 0)
				break;

			c -= copy_from_user(port->info->tmpbuf, buf, c);
			if (!c) 
			{
				if (!ret)
					ret = -EFAULT;
				break;
			}
			spin_lock_irqsave(&port->lock, flags);
			c1 = CIRC_SPACE_TO_END(circ->head, circ->tail, UART_XMIT_SIZE);
			if (c1 < c)
				c = c1;
			memcpy(circ->buf + circ->head, port->info->tmpbuf, c);
			circ->head = (circ->head + c) & (UART_XMIT_SIZE - 1);
			buf += c;
			count -= c;
			ret += c;
			spin_unlock_irqrestore(&port->lock, flags);
		}
	
		up(&port->info->tmpbuf_sem);
	}

	else
	{

		spin_lock_irqsave(&port->lock, flags);
		while (1) {
			c = CIRC_SPACE_TO_END(circ->head, circ->tail, UART_XMIT_SIZE);
			if (count < c)
				c = count;
			if (c <= 0)
				break;
			memcpy(circ->buf + circ->head, buf, c);
			circ->head = (circ->head + c) & (UART_XMIT_SIZE - 1);
			buf += c;
			count -= c;
			ret += c;
		}
		spin_unlock_irqrestore(&port->lock, flags);

	}
#else
	
	if (!state || !state->port) {
		WARN_ON(1);
		return -EL3HLT;
	}

	port = state->port;
	circ = &state->info.xmit;

	if (!circ->buf)
		return 0;
	spin_lock_irqsave(&port->lock, flags);
	while (1) {
		c = CIRC_SPACE_TO_END(circ->head, circ->tail, UART_XMIT_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;

		memcpy(circ->buf + circ->head, buf, c);
		circ->head = (circ->head + c) & (UART_XMIT_SIZE - 1);
		buf += c;
		count -= c;
		ret += c;
	}
	spin_unlock_irqrestore(&port->lock, flags);
#endif
	mp_start(tty);
	return ret;
}

static int mp_write_room(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;

	return uart_circ_chars_free(&state->info.xmit);
}

static int mp_chars_in_buffer(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;

	return uart_circ_chars_pending(&state->info.xmit);
}

static void mp_flush_buffer(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;
	unsigned long flags;

	/*
	 * This means you called this function _after_ the port was
	 * closed.  No cookie for you.
	 */
	if (!state || !state->port) {
		WARN_ON(1);
		return;
	}

	DPRINTK("mp_flush_buffer(%d) called\n", tty->index);

	spin_lock_irqsave(&port->lock, flags);
	uart_circ_clear(&state->info.xmit);
	spin_unlock_irqrestore(&port->lock, flags);
#if (LINUX_VERSION_CODE <  VERSION_CODE(2,6,9))
	wake_up_interruptible(&tty->write_wait);
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc.write_wakeup)
	{
		(tty->ldisc.write_wakeup)(tty);
	}
#else
	tty_wakeup(tty);
#endif
}

/*
 * This function is used to send a high-priority XON/XOFF character to
 * the device
 */
static void mp_send_xchar(struct tty_struct *tty, char ch)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;
	unsigned long flags;

	if (port->ops->send_xchar)
		port->ops->send_xchar(port, ch);
	else {
		port->x_char = ch;
		if (ch) {
			spin_lock_irqsave(&port->lock, flags);
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,14))
			port->ops->start_tx(port, 0);
#else
			port->ops->start_tx(port);
#endif
			spin_unlock_irqrestore(&port->lock, flags);
		}
	}
}

static void mp_throttle(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;

	if (I_IXOFF(tty))
		mp_send_xchar(tty, STOP_CHAR(tty));

	if (tty->termios->c_cflag & CRTSCTS)
		uart_clear_mctrl(state->port, TIOCM_RTS);
}

static void mp_unthrottle(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;

	if (I_IXOFF(tty)) {
		if (port->x_char)
			port->x_char = 0;
		else
			mp_send_xchar(tty, START_CHAR(tty));
	}

	if (tty->termios->c_cflag & CRTSCTS)
		uart_set_mctrl(port, TIOCM_RTS);
}

static int mp_get_info(struct uart_state *state, struct serial_struct *retinfo)
{
	struct uart_port *port = state->port;
	struct serial_struct tmp;

	memset(&tmp, 0, sizeof(tmp));
	tmp.type	    = port->type;
	tmp.line	    = port->line;
	tmp.port	    = port->iobase;
	if (HIGH_BITS_OFFSET)
		tmp.port_high = (long) port->iobase >> HIGH_BITS_OFFSET;
	tmp.irq		    = port->irq;
	tmp.flags	    = port->flags;
	tmp.xmit_fifo_size  = port->fifosize;
	tmp.baud_base	    = port->uartclk / 16;
	tmp.close_delay	    = state->close_delay;
	tmp.closing_wait    = state->closing_wait == USF_CLOSING_WAIT_NONE ?
		ASYNC_CLOSING_WAIT_NONE :
		state->closing_wait;
	tmp.custom_divisor  = port->custom_divisor;
	tmp.hub6	    = port->hub6;
	tmp.io_type         = port->iotype;
	tmp.iomem_reg_shift = port->regshift;
	tmp.iomem_base      = (void *)port->mapbase;

	if (copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;
	return 0;
}

	static int
mp_set_info(struct uart_state *state, struct serial_struct *newinfo)
{
	struct serial_struct new_serial;
	struct uart_port *port = state->port;
	unsigned long new_port;
	unsigned int change_irq, change_port, closing_wait;
	unsigned int old_custom_divisor;
	unsigned int old_flags, new_flags;
	int retval = 0;

	if (copy_from_user(&new_serial, newinfo, sizeof(new_serial)))
		return -EFAULT;

	new_port = new_serial.port;
	if (HIGH_BITS_OFFSET)
		new_port += (unsigned long) new_serial.port_high << HIGH_BITS_OFFSET;

	new_serial.irq = irq_canonicalize(new_serial.irq);

	closing_wait = new_serial.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
		USF_CLOSING_WAIT_NONE : new_serial.closing_wait;
	/*
	 * This semaphore protects state->count.  It is also
	 * very useful to prevent opens.  Also, take the
	 * port configuration semaphore to make sure that a
	 * module insertion/removal doesn't change anything
	 * under us.
	 */
	MP_STATE_LOCK(state);

	change_irq  = new_serial.irq != port->irq;

	/*
	 * Since changing the 'type' of the port changes its resource
	 * allocations, we should treat type changes the same as
	 * IO port changes.
	 */
	change_port = new_port != port->iobase ||
		(unsigned long)new_serial.iomem_base != port->mapbase ||
		new_serial.hub6 != port->hub6 ||
		new_serial.io_type != port->iotype ||
		new_serial.iomem_reg_shift != port->regshift ||
		new_serial.type != port->type;
	old_flags = port->flags;
	new_flags = new_serial.flags;
	old_custom_divisor = port->custom_divisor;

	if (!capable(CAP_SYS_ADMIN)) {
		retval = -EPERM;
		if (change_irq || change_port ||
				(new_serial.baud_base != port->uartclk / 16) ||
				(new_serial.close_delay != state->close_delay) ||
				(closing_wait != state->closing_wait) ||
				(new_serial.xmit_fifo_size != port->fifosize) ||
				(((new_flags ^ old_flags) & ~UPF_USR_MASK) != 0))
			goto exit;
		port->flags = ((port->flags & ~UPF_USR_MASK) |
				(new_flags & UPF_USR_MASK));
		port->custom_divisor = new_serial.custom_divisor;
		goto check_and_exit;
	}

	/*
	 * Ask the low level driver to verify the settings.
	 */
	if (port->ops->verify_port)
		retval = port->ops->verify_port(port, &new_serial);

	if ((new_serial.irq >= NR_IRQS) || (new_serial.irq < 0) ||
			(new_serial.baud_base < 9600))
		retval = -EINVAL;

	if (retval)
		goto exit;

	if (change_port || change_irq) {
		retval = -EBUSY;

		/*
		 * Make sure that we are the sole user of this port.
		 */
		if (uart_users(state) > 1)
			goto exit;

		/*
		 * We need to shutdown the serial port at the old
		 * port/type/irq combination.
		 */
		mp_shutdown(state);
	}

	if (change_port) {
		unsigned long old_iobase, old_mapbase;
		unsigned int old_type, old_iotype, old_hub6, old_shift;

		old_iobase = port->iobase;
		old_mapbase = port->mapbase;
		old_type = port->type;
		old_hub6 = port->hub6;
		old_iotype = port->iotype;
		old_shift = port->regshift;

		/*
		 * Free and release old regions
		 */
		if (old_type != PORT_UNKNOWN)
			port->ops->release_port(port);

		port->iobase = new_port;
		port->type = new_serial.type;
		port->hub6 = new_serial.hub6;
		port->iotype = new_serial.io_type;
		port->regshift = new_serial.iomem_reg_shift;
		port->mapbase = (unsigned long)new_serial.iomem_base;

		/*
		 * Claim and map the new regions
		 */
		if (port->type != PORT_UNKNOWN) {
			retval = port->ops->request_port(port);
		} else {
			/* Always success - Jean II */
			retval = 0;
		}

		/*
		 * If we fail to request resources for the
		 * new port, try to restore the old settings.
		 */
		if (retval && old_type != PORT_UNKNOWN) {
			port->iobase = old_iobase;
			port->type = old_type;
			port->hub6 = old_hub6;
			port->iotype = old_iotype;
			port->regshift = old_shift;
			port->mapbase = old_mapbase;
			retval = port->ops->request_port(port);
			/*
			 * If we failed to restore the old settings,
			 * we fail like this.
			 */
			if (retval)
				port->type = PORT_UNKNOWN;

			/*
			 * We failed anyway.
			 */
			retval = -EBUSY;
		}
	}

	port->irq              = new_serial.irq;
	port->uartclk          = new_serial.baud_base * 16;
	port->flags            = (port->flags & ~UPF_CHANGE_MASK) |
		(new_flags & UPF_CHANGE_MASK);
	port->custom_divisor   = new_serial.custom_divisor;
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,12))
	state->close_delay     = new_serial.close_delay * HZ / 100;
	state->closing_wait    = closing_wait * HZ / 100;;
#else
	state->close_delay     = new_serial.close_delay;
	state->closing_wait    = closing_wait;
#endif
	port->fifosize         = new_serial.xmit_fifo_size;
	if (state->info.port.tty)
		state->info.port.tty->low_latency =
			(port->flags & UPF_LOW_LATENCY) ? 1 : 0;

check_and_exit:
	retval = 0;
	if (port->type == PORT_UNKNOWN)
		goto exit;
	if (state->info.flags & UIF_INITIALIZED) {
		if (((old_flags ^ port->flags) & UPF_SPD_MASK) ||
				old_custom_divisor != port->custom_divisor) {
			/*
			 * If they're setting up a custom divisor or speed,
			 * instead of clearing it, then bitch about it. No
			 * need to rate-limit; it's CAP_SYS_ADMIN only.
			 */
			if (port->flags & UPF_SPD_MASK) {
				char buf[64];
				printk(KERN_NOTICE
						"%s sets custom speed on %s. This "
						"is deprecated.\n", current->comm,
						tty_name(state->info.port.tty, buf));
			}
			mp_change_speed(state, NULL);
		}
	} else
		retval = mp_startup(state, 1);
exit:
	MP_STATE_UNLOCK(state);
	return retval;
}


/*
 * mp_get_lsr_info - get line status register info.
 * Note: mp_ioctl protects us against hangups.
 */
static int mp_get_lsr_info(struct uart_state *state, unsigned int *value)
{
	struct uart_port *port = state->port;
	unsigned int result;

	result = port->ops->tx_empty(port);

	/*
	 * If we're about to load something into the transmit
	 * register, we'll pretend the transmitter isn't empty to
	 * avoid a race condition (depending on when the transmit
	 * interrupt happens).
	 */
	if (port->x_char ||
			((uart_circ_chars_pending(&state->info.xmit) > 0) &&
			 !state->info.port.tty->stopped && !state->info.port.tty->hw_stopped))
		result &= ~TIOCSER_TEMT;

	return put_user(result, value);
}

static int mp_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;
	int result = -EIO;

	MP_STATE_LOCK(state);
	if ((!file || !tty_hung_up_p(file)) &&
			!(tty->flags & (1 << TTY_IO_ERROR))) {
		result = port->mctrl;
		DPRINTK("test result = 0x%x \n",result);
		spin_lock_irq(&port->lock);
		result |= port->ops->get_mctrl(port);
		spin_unlock_irq(&port->lock);
	}
	MP_STATE_UNLOCK(state);
#ifdef MP_DEBUG
	if(result != 0x4000)
	{
		DPRINTK("%s: out result = 0x%x \n",__FUNCTION__,result);
	}
#endif
	return result;
}

static int
mp_tiocmset(struct tty_struct *tty, struct file *file,
		unsigned int set, unsigned int clear)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;
	int ret = -EIO;


	MP_STATE_LOCK(state);
	if ((!file || !tty_hung_up_p(file)) &&
			!(tty->flags & (1 << TTY_IO_ERROR))) {
		mp_update_mctrl(port, set, clear);
		ret = 0;
	}
	MP_STATE_UNLOCK(state);

	return ret;
}

static void mp_break_ctl(struct tty_struct *tty, int break_state)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;

	BUG_ON(!kernel_locked());

	MP_STATE_LOCK(state);

	if (port->type != PORT_UNKNOWN)
		port->ops->break_ctl(port, break_state);

	MP_STATE_UNLOCK(state);
}

static int mp_do_autoconfig(struct uart_state *state)
{
	struct uart_port *port = state->port;
	int flags, ret;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	/*
	 * Take the per-port semaphore.  This prevents count from
	 * changing, and hence any extra opens of the port while
	 * we're auto-configuring.
	 */
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,12))
	if (down_interruptible(&state->sem))
		return -ERESTARTSYS;
#else
	if (mutex_lock_interruptible(&state->mutex))
		return -ERESTARTSYS;
#endif
	ret = -EBUSY;
	if (uart_users(state) == 1) {
		mp_shutdown(state);

		/*
		 * If we already have a port type configured,
		 * we must release its resources.
		 */
		if (port->type != PORT_UNKNOWN)
			port->ops->release_port(port);

		flags = UART_CONFIG_TYPE;
		if (port->flags & UPF_AUTO_IRQ)
			flags |= UART_CONFIG_IRQ;

		/*
		 * This will claim the ports resources if
		 * a port is found.
		 */
		port->ops->config_port(port, flags);

		ret = mp_startup(state, 1);
	}
	MP_STATE_UNLOCK(state);
	return ret;
}

/*
 * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
 * - mask passed in arg for lines of interest
 *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
 * Caller should use TIOCGICOUNT to see which one it was
 */
	static int
mp_wait_modem_status(struct uart_state *state, unsigned long arg)
{
	struct uart_port *port = state->port;
	DECLARE_WAITQUEUE(wait, current);
	struct uart_icount cprev, cnow;
	int ret;

	/*
	 * note the counters on entry
	 */
	spin_lock_irq(&port->lock);
	memcpy(&cprev, &port->icount, sizeof(struct uart_icount));

	/*
	 * Force modem status interrupts on
	 */
	port->ops->enable_ms(port);
	spin_unlock_irq(&port->lock);

	add_wait_queue(&state->info.delta_msr_wait, &wait);
	for (;;) {
		spin_lock_irq(&port->lock);
		memcpy(&cnow, &port->icount, sizeof(struct uart_icount));
		spin_unlock_irq(&port->lock);

		set_current_state(TASK_INTERRUPTIBLE);

		if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
				((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
				((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
				((arg & TIOCM_CTS) && (cnow.cts != cprev.cts))) {
			ret = 0;
			break;
		}

		schedule();

		/* see if a signal did it */
		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		cprev = cnow;
	}

	current->state = TASK_RUNNING;
	remove_wait_queue(&state->info.delta_msr_wait, &wait);

	return ret;
}

/*
 * Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
 * Return: write counters to the user passed counter struct
 * NB: both 1->0 and 0->1 transitions are counted except for
 *     RI where only 0->1 is counted.
 */
	static int
mp_get_count(struct uart_state *state, struct serial_icounter_struct *icnt)
{
	struct serial_icounter_struct icount;
	struct uart_icount cnow;
	struct uart_port *port = state->port;

	spin_lock_irq(&port->lock);
	memcpy(&cnow, &port->icount, sizeof(struct uart_icount));
	spin_unlock_irq(&port->lock);

	icount.cts         = cnow.cts;
	icount.dsr         = cnow.dsr;
	icount.rng         = cnow.rng;
	icount.dcd         = cnow.dcd;
	icount.rx          = cnow.rx;
	icount.tx          = cnow.tx;
	icount.frame       = cnow.frame;
	icount.overrun     = cnow.overrun;
	icount.parity      = cnow.parity;
	icount.brk         = cnow.brk;
	icount.buf_overrun = cnow.buf_overrun;

	return copy_to_user(icnt, &icount, sizeof(icount)) ? -EFAULT : 0;
}

/*
 * Called via sys_ioctl under the BKL.  We can use spin_lock_irq() here.
 */
static int
mp_ioctl(struct tty_struct *tty, struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct uart_state *state = tty->driver_data;
	struct mp_port *info = (struct mp_port *)state->port;
	int ret = -ENOIOCTLCMD;

	BUG_ON(!kernel_locked());

	/*
	 * These ioctls don't rely on the hardware to be present.
	 */
	switch (cmd) {
		case TIOCGSERIAL:
			ret = mp_get_info(state, (struct serial_struct *)arg);
			break;

		case TIOCSSERIAL:
			ret = mp_set_info(state, (struct serial_struct *)arg);
			break;

		case TIOCSERCONFIG:
			ret = mp_do_autoconfig(state);
			break;

		case TIOCSERGWILD: /* obsolete */
		case TIOCSERSWILD: /* obsolete */
			ret = 0;
			break;
			/* TSUM for Multiport */
		case TIOCGNUMOFPORT: /* Get number of ports */
			return NR_PORTS;
		case TIOCSMULTIECHO: /* set to multi-drop mode(RS422) or echo mode(RS485)*/
			outb( ( inb(info->interface_config_addr) & ~0x03 ) | 0x01 ,  
					info->interface_config_addr);
			return 0;
		case TIOCSPTPNOECHO: /* set to multi-drop mode(RS422) or echo mode(RS485) */
			outb( ( inb(info->interface_config_addr) & ~0x03 )  ,             
					info->interface_config_addr);
			return 0;
	}

	if (ret != -ENOIOCTLCMD)
		goto out;

	if (tty->flags & (1 << TTY_IO_ERROR)) {
		ret = -EIO;
		goto out;
	}

	/*
	 * The following should only be used when hardware is present.
	 */
	switch (cmd) {
		case TIOCMIWAIT:
			ret = mp_wait_modem_status(state, arg);
			break;

		case TIOCGICOUNT:
			ret = mp_get_count(state, (struct serial_icounter_struct *)arg);
			break;
	}

	if (ret != -ENOIOCTLCMD)
		goto out;

	MP_STATE_LOCK(state);

	if (tty_hung_up_p(filp)) {
		ret = -EIO;
		goto out_up;
	}

	/*
	 * All these rely on hardware being present and need to be
	 * protected against the tty being hung up.
	 */
	switch (cmd) {
		case TIOCSERGETLSR: /* Get line status register */
			ret = mp_get_lsr_info(state, (unsigned int *)arg);
			break;

		default: {
					 struct uart_port *port = state->port;
					 if (port->ops->ioctl)
						 ret = port->ops->ioctl(port, cmd, arg);
					 break;
				 }
	}
out_up:
	MP_STATE_UNLOCK(state);
out:
	return ret;
}

static void mp_set_termios(struct tty_struct *tty, struct MP_TERMIOS *old_termios)
{
	struct uart_state *state = tty->driver_data;
	unsigned long flags;
	unsigned int cflag = tty->termios->c_cflag;

	BUG_ON(!kernel_locked());

	/*
	 * These are the bits that are used to setup various
	 * flags in the low level driver.
	 */
#define RELEVANT_IFLAG(iflag)	((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

	if ((cflag ^ old_termios->c_cflag) == 0 &&
			RELEVANT_IFLAG(tty->termios->c_iflag ^ old_termios->c_iflag) == 0)
		return;

	mp_change_speed(state, old_termios);

	/* Handle transition to B0 status */
	if ((old_termios->c_cflag & CBAUD) && !(cflag & CBAUD))
		uart_clear_mctrl(state->port, TIOCM_RTS | TIOCM_DTR);

	/* Handle transition away from B0 status */
	if (!(old_termios->c_cflag & CBAUD) && (cflag & CBAUD)) {
		unsigned int mask = TIOCM_DTR;
		if (!(cflag & CRTSCTS) ||
				!test_bit(TTY_THROTTLED, &tty->flags))
			mask |= TIOCM_RTS;
		uart_set_mctrl(state->port, mask);
	}

	/* Handle turning off CRTSCTS */
	if ((old_termios->c_cflag & CRTSCTS) && !(cflag & CRTSCTS)) {
		spin_lock_irqsave(&state->port->lock, flags);
		tty->hw_stopped = 0;
		__mp_start(tty);
		spin_unlock_irqrestore(&state->port->lock, flags);
	}

	/* Handle turning on CRTSCTS */
	if (!(old_termios->c_cflag & CRTSCTS) && (cflag & CRTSCTS)) {
		spin_lock_irqsave(&state->port->lock, flags);
		if (!(state->port->ops->get_mctrl(state->port) & TIOCM_CTS)) {
			tty->hw_stopped = 1;
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,14))
			state->port->ops->stop_tx(state->port, 0);
#else
			state->port->ops->stop_tx(state->port);
#endif 
		}
		spin_unlock_irqrestore(&state->port->lock, flags);
	}
}

/*
 * In 2.4.5, calls to this will be serialized via the BKL in
 *  linux/drivers/char/tty_io.c:tty_release()
 *  linux/drivers/char/tty_io.c:do_tty_handup()
 */
static void mp_close(struct tty_struct *tty, struct file *filp)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port;

	BUG_ON(!kernel_locked());

	if (!state || !state->port)
		return;

	port = state->port;

	DPRINTK("mp_close(%d) called\n", port->line);

	MP_STATE_LOCK(state);

	if (tty_hung_up_p(filp))
		goto done;

	if ((tty->count == 1) && (state->count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  state->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		printk("mp_close: bad serial port count; tty->count is 1, "
				"state->count is %d\n", state->count);
		state->count = 1;
	}
	if (--state->count < 0) {
		printk("rs_close: bad serial port count for %s: %d\n",
				tty->name, state->count);
		state->count = 0;
	}
	if (state->count)
		goto done;

	/*
	 * Now we wait for the transmit buffer to clear; and we notify
	 * the line discipline to only process XON/XOFF characters by
	 * setting tty->closing.
	 */
	tty->closing = 1;

	if (state->closing_wait != USF_CLOSING_WAIT_NONE)
		tty_wait_until_sent(tty, state->closing_wait);

	/*
	 * At this point, we stop accepting input.  To do this, we
	 * disable the receive line status interrupts.
	 */
	if (state->info.flags & UIF_INITIALIZED) {
		unsigned long flags;
		spin_lock_irqsave(&port->lock, flags);
		port->ops->stop_rx(port);
		spin_unlock_irqrestore(&port->lock, flags);
		/*
		 * Before we drop DTR, make sure the UART transmitter
		 * has completely drained; this is especially
		 * important if there is a transmit FIFO!
		 */
		mp_wait_until_sent(tty, port->timeout);
	}

	mp_shutdown(state);
	mp_flush_buffer(tty);
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,9))
	if (tty->ldisc.flush_buffer)
		tty->ldisc.flush_buffer(tty);
#else
	tty_ldisc_flush(tty);
#endif
	tty->closing = 0;
	state->info.port.tty = NULL;
	if (state->info.port.blocked_open) 
	{
		if (state->close_delay)
		{
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(state->close_delay);
		}
	}
	else
	{
		mp_change_pm(state, 3);
	}

	/*
	 * Wake up anyone trying to open this port.
	 */
	state->info.flags &= ~UIF_NORMAL_ACTIVE;
	wake_up_interruptible(&state->info->open_wait);

done:
	MP_STATE_UNLOCK(state);
}

static void mp_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct uart_state *state = tty->driver_data;
	struct uart_port *port = state->port;
	unsigned long char_time, expire;

	BUG_ON(!kernel_locked());

	if (port->type == PORT_UNKNOWN || port->fifosize == 0)
		return;

	/*
	 * Set the check interval to be 1/5 of the estimated time to
	 * send a single character, and make it at least 1.  The check
	 * interval should also be less than the timeout.
	 *
	 * Note: we have to use pretty tight timings here to satisfy
	 * the NIST-PCTS.
	 */
	char_time = (port->timeout - HZ/50) / port->fifosize;
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
	 * takes longer than port->timeout, this is probably due to a
	 * UART bug of some kind.  So, we clamp the timeout parameter at
	 * 2*port->timeout.
	 */
	if (timeout == 0 || timeout > 2 * port->timeout)
		timeout = 2 * port->timeout;

	expire = jiffies + timeout;

	DPRINTK("mp_wait_until_sent(%d), jiffies=%lu, expire=%lu...\n",
			port->line, jiffies, expire);

	/*
	 * Check whether the transmitter is empty every 'char_time'.
	 * 'timeout' / 'expire' give us the maximum amount of time
	 * we wait.
	 */
	while (!port->ops->tx_empty(port)) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(char_time);
		if (signal_pending(current))
			break;
		if (time_after(jiffies, expire))
			break;
	}
	set_current_state(TASK_RUNNING); /* might not be needed */
}

/*
 * This is called with the BKL held in
 *  linux/drivers/char/tty_io.c:do_tty_hangup()
 * We're called from the eventd thread, so we can sleep for
 * a _short_ time only.
 */
static void mp_hangup(struct tty_struct *tty)
{
	struct uart_state *state = tty->driver_data;
	struct uart_info *info = &state->info;

	BUG_ON(!kernel_locked());
	DPRINTK("mp_hangup(%d)\n", state->port->line);

	MP_STATE_LOCK(state);
	if (info->flags & UIF_NORMAL_ACTIVE) {
		mp_flush_buffer(tty);
		mp_shutdown(state);
		state->count = 0;
		info->flags &= ~UIF_NORMAL_ACTIVE;
		info->port.tty = NULL;
		wake_up_interruptible(&info->port.open_wait);
		wake_up_interruptible(&info->delta_msr_wait);
	}
	MP_STATE_UNLOCK(state);
}

/*
 * Copy across the serial console cflag setting into the termios settings
 * for the initial open of the port.  This allows continuity between the
 * kernel settings, and the settings init adopts when it opens the port
 * for the first time.
 */
static void mp_update_termios(struct uart_state *state)
{
	struct tty_struct *tty = state->info.port.tty;
	struct uart_port *port = state->port;

	/*
	 * If the device failed to grab its irq resources,
	 * or some other error occurred, don't try to talk
	 * to the port hardware.
	 */
	if (!(tty->flags & (1 << TTY_IO_ERROR))) {
		/*
		 * Make termios settings take effect.
		 */
		mp_change_speed(state, NULL);

		/*
		 * And finally enable the RTS and DTR signals.
		 */
		if (tty->termios->c_cflag & CBAUD)
			uart_set_mctrl(port, TIOCM_DTR | TIOCM_RTS);
	}
}

/*
 * Block the open until the port is ready.  We must be called with
 * the per-port semaphore held.
 */
	static int
mp_block_til_ready(struct file *filp, struct uart_state *state)
{
	DECLARE_WAITQUEUE(wait, current);
	struct uart_info *info = &state->info;
	struct uart_port *port = state->port;
	unsigned int mctrl;

	info->port.blocked_open++;
	state->count--;

	add_wait_queue(&info->port.open_wait, &wait);
	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);

		/*
		 * If we have been hung up, tell userspace/restart open.
		 */
		if (tty_hung_up_p(filp) || info->port.tty == NULL)
			break;

		/*
		 * If the port has been closed, tell userspace/restart open.
		 */
		if (!(info->flags & UIF_INITIALIZED))
			break;

		/*
		 * If non-blocking mode is set, or CLOCAL mode is set,
		 * we don't want to wait for the modem status lines to
		 * indicate that the port is ready.
		 *
		 * Also, if the port is not enabled/configured, we want
		 * to allow the open to succeed here.  Note that we will
		 * have set TTY_IO_ERROR for a non-existant port.
		 */
		if ((filp->f_flags & O_NONBLOCK) ||
				(info->port.tty->termios->c_cflag & CLOCAL) ||
				(info->port.tty->flags & (1 << TTY_IO_ERROR))) {
			break;
		}

		/*
		 * Set DTR to allow modem to know we're waiting.  Do
		 * not set RTS here - we want to make sure we catch
		 * the data from the modem.
		 */
		if (info->port.tty->termios->c_cflag & CBAUD)
			uart_set_mctrl(port, TIOCM_DTR);

		/*
		 * and wait for the carrier to indicate that the
		 * modem is ready for us.
		 */
		spin_lock_irq(&port->lock);
		port->ops->enable_ms(port);
		mctrl = port->ops->get_mctrl(port);
		spin_unlock_irq(&port->lock);
		if (mctrl & TIOCM_CAR)
			break;

		MP_STATE_UNLOCK(state);
		schedule();
		MP_STATE_LOCK(state);

		if (signal_pending(current))
			break;
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&info->port.open_wait, &wait);

	state->count++;
	info->port.blocked_open--;

	if (signal_pending(current))
		return -ERESTARTSYS;

	if (!info->port.tty || tty_hung_up_p(filp))
		return -EAGAIN;

	return 0;
}

static struct uart_state *uart_get(struct uart_driver *drv, int line)
{
	struct uart_state *state;

	MP_MUTEX_LOCK(mp_mutex);
	state = drv->state + line;
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,12))
	if (down_interruptible(&state->sem)) {

		state = ERR_PTR(-ERESTARTSYS);
		goto out;
	}
#else
	if (mutex_lock_interruptible(&state->mutex)) {
		state = ERR_PTR(-ERESTARTSYS);
		goto out;
	}
#endif
	state->count++;
	if (!state->port) {
		state->count--;
		MP_STATE_UNLOCK(state);
		state = ERR_PTR(-ENXIO);
		goto out;
	}

	if (!state->info) {
		state->info = kmalloc(sizeof(struct uart_info), GFP_KERNEL);
		if (state->info) {
			memset(state->info, 0, sizeof(struct uart_info));
			init_waitqueue_head(&state->info->port.open_wait);
			init_waitqueue_head(&state->info->port.delta_msr_wait);

			/*
			 * Link the info into the other structures.
			 */
			state->port->info = state->info;

			tasklet_init(&state->info->tlet, mp_tasklet_action,
					(unsigned long)state);
		} else {
			state->count--;
			MP_STATE_UNLOCK(state);
			state = ERR_PTR(-ENOMEM);
		}
	}

out:
	MP_MUTEX_UNLOCK(mp_mutex);
	return state;
}

/*
 * In 2.4.5, calls to mp_open are serialised by the BKL in
 *   linux/fs/devices.c:chrdev_open()
 * Note that if this fails, then mp_close() _will_ be called.
 *
 * In time, we want to scrap the "opening nonpresent ports"
 * behaviour and implement an alternative way for setserial
 * to set base addresses/ports/types.  This will allow us to
 * get rid of a certain amount of extra tests.
 */
static int mp_open(struct tty_struct *tty, struct file *filp)
{
	struct uart_driver *drv = (struct uart_driver *)tty->driver->driver_state;
	struct uart_state *state;
	int retval, line = tty->index;
	struct mp_port *mtpt;

	BUG_ON(!kernel_locked());
	DPRINTK("mp_open(%d) called\n", line);

	/*
	 * tty->driver->num won't change, so we won't fail here with
	 * tty->driver_data set to something non-NULL (and therefore
	 * we won't get caught by mp_close()).
	 */
	retval = -ENODEV;
	if (line >= tty->driver->num)
		goto fail;

	/*
	 * We take the semaphore inside uart_get to guarantee that we won't
	 * be re-entered while allocating the info structure, or while we
	 * request any IRQs that the driver may need.  This also has the nice
	 * side-effect that it delays the action of mp_hangup, so we can
	 * guarantee that info->tty will always contain something reasonable.
	 */
	state = uart_get(drv, line);

	mtpt  = (struct mp_port *)state->port;

	if (IS_ERR(state)) {
		retval = PTR_ERR(state);
		goto fail;
	}

	/*
	 * Once we set tty->driver_data here, we are guaranteed that
	 * mp_close() will decrement the driver module use count.
	 * Any failures from here onwards should not touch the count.
	 */
	tty->driver_data = state;
	tty->low_latency = (state->info.flags & UPF_LOW_LATENCY) ? 1 : 0;
	tty->alt_speed = 0;
	state->info.port.tty = tty;

	/*
	 * If the port is in the middle of closing, bail out now.
	 */
	if (tty_hung_up_p(filp)) {
		retval = -EAGAIN;
		state->count--;
		MP_STATE_UNLOCK(state);
		goto fail;
	}

	/*
	 * Make sure the device is in D0 state.
	 */
	if (state->count == 1)
		mp_change_pm(state, 0);

	/*
	 * Start up the serial port.
	 */
	retval = mp_startup(state, 0);

	/*
	 * If we succeeded, wait until the port is ready.
	 */
	if (retval == 0)
		retval = mp_block_til_ready(filp, state);
	MP_STATE_UNLOCK(state);

	/*
	 * If this is the first open to succeed, adjust things to suit.
	 */
	if (retval == 0 && !(state->info->flags & UIF_NORMAL_ACTIVE)) {
		state->info->flags |= UIF_NORMAL_ACTIVE;

		mp_update_termios(state);
	}

	uart_clear_mctrl(state->port, TIOCM_RTS);
fail:
	return retval;
}

static const char *mp_type(struct uart_port *port)
{
	const char *str = NULL;

	if (port->ops->type)
		str = port->ops->type(port);

	if (!str)
		str = "unknown";

	return str;
}

#ifdef CONFIG_PROC_FS

static int mp_line_info(char *buf, struct uart_driver *drv, int i)
{
	struct uart_state *state = drv->state + i;
	struct uart_port *port = state->port;
	char stat_buf[32];
	unsigned int status;
	int ret;

	if (!port)
		return 0;

	ret = sprintf(buf, "%d: uart:%s %s%08lX irq:%d",
			port->line, mp_type(port),
			port->iotype == UPIO_MEM ? "mmio:0x" : "port:",
			port->iotype == UPIO_MEM ? port->mapbase :
			(unsigned long) port->iobase,
			port->irq);

	if (port->type == PORT_UNKNOWN) {
		strcat(buf, "\n");
		return ret + 1;
	}

	if(capable(CAP_SYS_ADMIN))
	{
		spin_lock_irq(&port->lock);
		status = port->ops->get_mctrl(port);
		spin_unlock_irq(&port->lock);

		ret += sprintf(buf + ret, " tx:%d rx:%d",
				port->icount.tx, port->icount.rx);
		if (port->icount.frame)
			ret += sprintf(buf + ret, " fe:%d",
					port->icount.frame);
		if (port->icount.parity)
			ret += sprintf(buf + ret, " pe:%d",
					port->icount.parity);
		if (port->icount.brk)
			ret += sprintf(buf + ret, " brk:%d",
					port->icount.brk);
		if (port->icount.overrun)
			ret += sprintf(buf + ret, " oe:%d",
					port->icount.overrun);

#define INFOBIT(bit,str) \
		if (port->mctrl & (bit)) \
			strncat(stat_buf, (str), sizeof(stat_buf) - \
					strlen(stat_buf) - 2)
#define STATBIT(bit,str) \
				if (status & (bit)) \
					strncat(stat_buf, (str), sizeof(stat_buf) - \
							strlen(stat_buf) - 2)

						stat_buf[0] = '\0';
		stat_buf[1] = '\0';
		INFOBIT(TIOCM_RTS, "|RTS");
		STATBIT(TIOCM_CTS, "|CTS");
		INFOBIT(TIOCM_DTR, "|DTR");
		STATBIT(TIOCM_DSR, "|DSR");
		STATBIT(TIOCM_CAR, "|CD");
		STATBIT(TIOCM_RNG, "|RI");
		if (stat_buf[0])
			stat_buf[0] = ' ';
		strcat(stat_buf, "\n");

		ret += sprintf(buf + ret, stat_buf);
	} else {
		strcat(buf, "\n");
		ret++;
	}
#undef STATBIT
#undef INFOBIT
	return ret;
}

static int mp_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	struct tty_driver *ttydrv = data;
	struct uart_driver *drv = ttydrv->driver_state;
	int i, len = 0, l;
	off_t begin = 0;

	len += sprintf(page, "serinfo:1.0 driver%s%s revision:%s\n",
			"", "", "");
	for (i = 0; i < drv->nr && len < PAGE_SIZE - 96; i++) {
		l = mp_line_info(page + len, drv, i);
		len += l;
		if (len + begin > off + count)
			goto done;
		if (len + begin < off) {
			begin += len;
			len = 0;
		}
	}
	*eof = 1;
done:
	if (off >= len + begin)
		return 0;
	*start = page + (off - begin);
	return (count < begin + len - off) ? count : (begin + len - off);
}
#endif

static void mp_change_pm(struct uart_state *state, int pm_state)
{
	struct uart_port *port = state->port;
	if (port->ops->pm)
		port->ops->pm(port, pm_state, state->pm_state);
	state->pm_state = pm_state;
}

int mp_suspend_port(struct uart_driver *drv, struct uart_port *port)
{
	struct uart_state *state = drv->state + port->line;

	MP_STATE_LOCK(state);

	if (state->info.flags & UIF_INITIALIZED) {
		struct uart_ops *ops = (struct uart_ops *) port->ops;

		spin_lock_irq(&port->lock);
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,14))
		ops->stop_tx(port, 0);
#else
		ops->stop_tx(port);
#endif 
		ops->set_mctrl(port, 0);
		ops->stop_rx(port);
		spin_unlock_irq(&port->lock);

		/*
		 * Wait for the transmitter to empty.
		 */
		while (!ops->tx_empty(port)) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(10*HZ/1000);
		}
		set_current_state(TASK_RUNNING);

		ops->shutdown(port);
	}

	mp_change_pm(state, 3);

	MP_STATE_UNLOCK(state);

	return 0;
}

int mp_resume_port(struct uart_driver *drv, struct uart_port *port)
{
	struct uart_state *state = drv->state + port->line;
	MP_STATE_LOCK(state);
	mp_change_pm(state, 0);

	if (state->info.flags & UIF_INITIALIZED) {
		struct uart_ops *ops = (struct uart_ops *) port->ops;
		int ret = 0;

		ops->set_mctrl(port, 0);
		ret = ops->startup(port);
		if (ret == 0) {
			mp_change_speed(state, NULL);
			spin_lock_irq(&port->lock);
			ops->set_mctrl(port, port->mctrl);
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,14))
			ops->start_tx(port, 0);
#else
			ops->start_tx(port);
#endif
			spin_unlock_irq(&port->lock);
		}
		else
		{
			/*
			 * Failed to resume - maybe hardware went away?
			 * Clear the "initialized" flag so we won't try
			 * to call the low level drivers shutdown method.
			 */
			state->info.flags &= ~UIF_INITIALIZED;
			mp_shutdown(state);
		}
	}

	MP_STATE_UNLOCK(state);

	return 0;
}

static inline void mp_report_port(struct uart_driver *drv, struct uart_port *port)
{
	char address[64];
	DPRINTK("%s in \n",__FUNCTION__);

	switch (port->iotype) {
		case UPIO_PORT:
			snprintf(address, sizeof(address),"I/O 0x%lx", port->iobase);
			break;
		case UPIO_HUB6:
			snprintf(address, sizeof(address),"I/O 0x%lx offset 0x%x", port->iobase, port->hub6);
			break;
		case UPIO_MEM:
			snprintf(address, sizeof(address),"MMIO 0x%llx", (unsigned long long) port->mapbase);
			break;
		default:
			strlcpy(address, "*unknown*", sizeof(address));
			break;
	}

	printk(KERN_INFO "%s%s%s%d at %s (irq = %d) is a %s\n",
			port->dev ? dev_name(port->dev) : "",
			port->dev ? ": " : "",
			drv->dev_name, port->line, address, port->irq, mp_type(port));
	DPRINTK("%s out \n",__FUNCTION__);
}

static void
mp_configure_port(struct uart_driver *drv, struct uart_state *state,
		struct uart_port *port)
{
	unsigned int flags;

	DPRINTK("%s in \n",__FUNCTION__);

	/*
	 * If there isn't a port here, don't do anything further.
	 */
	if (!port->iobase && !port->mapbase && !port->membase)
	{
		DPRINTK("%s error \n",__FUNCTION__);
		return;
	}
	/*
	 * Now do the auto configuration stuff.  Note that config_port
	 * is expected to claim the resources and map the port for us.
	 */
	flags = UART_CONFIG_TYPE;
	if (port->flags & UPF_AUTO_IRQ)
		flags |= UART_CONFIG_IRQ;
	if (port->flags & UPF_BOOT_AUTOCONF) {
		port->type = PORT_UNKNOWN;
		port->ops->config_port(port, flags);
	}

	if (port->type != PORT_UNKNOWN) {
		unsigned long flags;

		mp_report_port(drv, port);

		/*
		 * Ensure that the modem control lines are de-activated.
		 * We probably don't need a spinlock around this, but
		 */
		spin_lock_irqsave(&port->lock, flags);
		port->ops->set_mctrl(port, 0);
		spin_unlock_irqrestore(&port->lock, flags);

		/*
		 * Power down all ports by default, except the
		 * console if we have one.
		 */
		mp_change_pm(state, 3);
	}
	DPRINTK("%s out \n",__FUNCTION__);
}

/*
 * This reverses the effects of mp_configure_port, hanging up the
 * port before removal.
 */
	static void
mp_unconfigure_port(struct uart_driver *drv, struct uart_state *state)
{
	struct uart_port *port = state->port;
	struct uart_info *info = &state->info;

	if (info && info->port.tty)
		tty_vhangup(info->port.tty);

	MP_STATE_LOCK(state);

	state->port = NULL;

	/*
	 * Free the port IO and memory resources, if any.
	 */
	DPRINTK("%s: b\n",__FUNCTION__);
	if (port->type != PORT_UNKNOWN)
		port->ops->release_port(port);
	DPRINTK("%s: a\n",__FUNCTION__);

	/*
	 * Indicate that there isn't a port here anymore.
	 */
	port->type = PORT_UNKNOWN;

	/*
	 * Kill the tasklet, and free resources.
	 */
	if (info) {
		tasklet_kill(&info->tlet);
		kfree(info);
	}

	MP_STATE_UNLOCK(state);
}

static struct tty_operations mp_ops = {
	.open		= mp_open,
	.close		= mp_close,
	.write		= mp_write,
	.put_char	= mp_put_char,
	.flush_chars	= mp_put_chars,
	.write_room	= mp_write_room,
	.chars_in_buffer= mp_chars_in_buffer,
	.flush_buffer	= mp_flush_buffer,
	.ioctl		= mp_ioctl,
	.throttle	= mp_throttle,
	.unthrottle	= mp_unthrottle,
	.send_xchar	= mp_send_xchar,
	.set_termios	= mp_set_termios,
	.stop		= mp_stop,
	.start		= mp_start,
	.hangup		= mp_hangup,
	.break_ctl	= mp_break_ctl,
	.wait_until_sent= mp_wait_until_sent,
#ifdef CONFIG_PROC_FS
	.read_proc	= mp_read_proc,
#endif
	.tiocmget	= mp_tiocmget,
	.tiocmset	= mp_tiocmset,
};

/**
 *	mp_register_driver - register a driver with the uart core layer
 *	@drv: low level driver structure
 *
 *	Register a uart driver with the core driver.  We in turn register
 *	with the tty layer, and initialise the core driver per-port state.
 *
 *	We have a proc file in /proc/tty/driver which is named after the
 *	normal driver.
 *
 *	drv->port should be NULL, and the per-port structures should be
 *	registered using mp_add_one_port after this call has succeeded.
 */
int mp_register_driver(struct uart_driver *drv)
{
	struct tty_driver *normal = NULL;
	int i, retval;

	DPRINTK("%s in \n",__FUNCTION__);

	BUG_ON(drv->state);

	/*
	 * Maybe we should be using a slab cache for this, especially if
	 * we have a large number of ports to handle.
	 */
	drv->state = kmalloc(sizeof(struct uart_state) * drv->nr, GFP_KERNEL);
	retval = -ENOMEM;
	if (!drv->state)
		goto out;

	memset(drv->state, 0, sizeof(struct uart_state) * drv->nr);

	normal  = alloc_tty_driver(drv->nr);
	if (!normal)
		goto out;

	drv->tty_driver = normal;

	normal->owner		= drv->owner;
	normal->driver_name	= drv->driver_name;
	normal->name		= drv->dev_name;
	normal->major		= drv->major;
	normal->minor_start	= drv->minor;
	normal->type		= TTY_DRIVER_TYPE_SERIAL;
	normal->subtype		= SERIAL_TYPE_NORMAL;
	normal->init_termios	= tty_std_termios;
	normal->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,18))
	normal->flags		= TTY_DRIVER_REAL_RAW | TTY_DRIVER_NO_DEVFS;
#else
	normal->flags		= TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
#endif
	normal->driver_state    = drv;
	tty_set_operations(normal, &mp_ops);

	/*
	 * Initialise the UART state(s).
	 */
	for (i = 0; i < drv->nr; i++) {
		struct uart_state *state = drv->state + i;

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,12))
		state->close_delay     = 5 * HZ / 10;
		state->closing_wait    = 30 * HZ;
		init_MUTEX(&state->sem);
#else

		state->close_delay     = 500;   /* .5 seconds */
		state->closing_wait    = 30000; /* 30 seconds */

		mutex_init(&state->mutex);
#endif
	}

	retval = tty_register_driver(normal);
out:
	if (retval < 0) {
		put_tty_driver(normal);
		kfree(drv->state);
	}

	DPRINTK("%s out ret = %d\n",__FUNCTION__,retval);
	return retval;
}

/**
 *	mp_unregister_driver - remove a driver from the uart core layer
 *	@drv: low level driver structure
 *
 *	Remove all references to a driver from the core driver.  The low
 *	level driver must have removed all its ports via the
 *	mp_remove_one_port() if it registered them with mp_add_one_port().
 *	(ie, drv->port == NULL)
 */
void mp_unregister_driver(struct uart_driver *drv)
{
	struct tty_driver *p = drv->tty_driver;
	DPRINTK("%s: b\n",__FUNCTION__);
	tty_unregister_driver(p);
	put_tty_driver(p);
	kfree(drv->state);
	drv->tty_driver = NULL;
	DPRINTK("%s: a\n",__FUNCTION__);
}

struct tty_driver *uart_console_device(struct console *co, int *index)
{
	struct uart_driver *p = co->data;
	*index = co->index;
	return p->tty_driver;
}

/**
 *	mp_add_one_port - attach a driver-defined port structure
 *	@drv: pointer to the uart low level driver structure for this port
 *	@port: uart port structure to use for this port.
 *
 *	This allows the driver to register its own uart_port structure
 *	with the core driver.  The main purpose is to allow the low
 *	level uart drivers to expand uart_port, rather than having yet
 *	more levels of structures.
 */
int mp_add_one_port(struct uart_driver *drv, struct uart_port *port)
{
	struct uart_state *state;
	int ret = 0;

	DPRINTK("%s in \n",__FUNCTION__);
	BUG_ON(in_interrupt());

	if (port->line >= drv->nr)
		return -EINVAL;

	state = drv->state + port->line;

	MP_MUTEX_LOCK(mp_mutex);
	if (state->port) {
		ret = -EINVAL;
		goto out;
	}

	state->port = port;

	spin_lock_init(&port->lock);
	port->cons = drv->cons;
	port->info = &state->info;

	mp_configure_port(drv, state, port);

	/*
	 * Register the port whether it's detected or not.  This allows
	 * setserial to be used to alter this ports parameters.
	 */
	tty_register_device(drv->tty_driver, port->line, port->dev);

out:
	MP_MUTEX_UNLOCK(mp_mutex);

	DPRINTK("%s out \n",__FUNCTION__);

	return ret;
}

/**
 *	mp_remove_one_port - detach a driver defined port structure
 *	@drv: pointer to the uart low level driver structure for this port
 *	@port: uart port structure for this port
 *
 *	This unhooks (and hangs up) the specified port structure from the
 *	core driver.  No further calls will be made to the low-level code
 *	for this port.
 */
int mp_remove_one_port(struct uart_driver *drv, struct uart_port *port)
{
	struct uart_state *state = drv->state + port->line;

	BUG_ON(in_interrupt());

	if (state->port != port)
		printk(KERN_ALERT "Removing wrong port: %p != %p\n",
				state->port, port);

	MP_MUTEX_LOCK(mp_mutex);

	//	multi_shutdown(port);

	/*
	 * Remove the devices from devfs
	 */
	tty_unregister_device(drv->tty_driver, port->line);

	mp_unconfigure_port(drv, state);
	state->port = NULL;
	MP_MUTEX_UNLOCK(mp_mutex);

	return 0;
}

/*
 *	Are the two ports equivalent?
 */
static int mp_match_port(struct uart_port *port1, struct uart_port *port2)
{
	if (port1->iotype != port2->iotype)
		return 0;

	switch (port1->iotype) {
		case UPIO_PORT:
			return (port1->iobase == port2->iobase);
		case UPIO_HUB6:
			return (port1->iobase == port2->iobase) &&
				(port1->hub6   == port2->hub6);
		case UPIO_MEM:
			return (port1->membase == port2->membase);
	}
	return 0;
}

/*
 *	Try to find an unused uart_state slot for a port.
 */
	static struct uart_state *
mp_find_match_or_unused(struct uart_driver *drv, struct uart_port *port)
{
	int i;

	/*
	 * First, find a port entry which matches.  Note: if we do
	 * find a matching entry, and it has a non-zero use count,
	 * then we can't register the port.
	 */
	for (i = 0; i < drv->nr; i++)
		if (mp_match_port(drv->state[i].port, port))
			return &drv->state[i];

	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < drv->nr; i++)
		if (drv->state[i].port->type == PORT_UNKNOWN &&
				drv->state[i].port->iobase == 0 &&
				drv->state[i].count == 0)
			return &drv->state[i];

	/*
	 * That also failed.  Last resort is to find any currently
	 * entry which doesn't have a real port associated with it.
	 */
	for (i = 0; i < drv->nr; i++)
		if (drv->state[i].port->type == PORT_UNKNOWN &&
				drv->state[i].count == 0)
			return &drv->state[i];

	return NULL;
}

/**
 *	mp_register_port: register uart settings with a port
 *	@drv: pointer to the uart low level driver structure for this port
 *	@port: uart port structure describing the port
 *
 *	Register UART settings with the specified low level driver.  Detect
 *	the type of the port if UPF_BOOT_AUTOCONF is set, and detect the
 *	IRQ if UPF_AUTO_IRQ is set.
 *
 *	We try to pick the same port for the same IO base address, so that
 *	when a modem is plugged in, unplugged and plugged back in, it gets
 *	allocated the same port.
 *
 *	Returns negative error, or positive line number.
 */
int mp_register_port(struct uart_driver *drv, struct uart_port *port)
{
	struct uart_state *state;
	int ret;

	MP_MUTEX_LOCK(mp_mutex);

	state = mp_find_match_or_unused(drv, port);

	if (state) {
		/*
		 * Ok, we've found a line that we can use.
		 *
		 * If we find a port that matches this one, and it appears
		 * to be in-use (even if it doesn't have a type) we shouldn't
		 * alter it underneath itself - the port may be open and
		 * trying to do useful work.
		 */
		if (uart_users(state) != 0) {
			ret = -EBUSY;
			goto out;
		}

		/*
		 * If the port is already initialised, don't touch it.
		 */
		if (state->port->type == PORT_UNKNOWN) {
			state->port->iobase   = port->iobase;
			state->port->membase  = port->membase;
			state->port->irq      = port->irq;
			state->port->uartclk  = port->uartclk;
			state->port->fifosize = port->fifosize;
			state->port->regshift = port->regshift;
			state->port->iotype   = port->iotype;
			state->port->flags    = port->flags;
			state->port->line     = state - drv->state;
			state->port->mapbase  = port->mapbase;

			mp_configure_port(drv, state, state->port);
		}

		ret = state->port->line;
	} else
		ret = -ENOSPC;
out:
	MP_MUTEX_UNLOCK(mp_mutex);
	return ret;
}

/**
 *	mp_unregister_port - de-allocate a port
 *	@drv: pointer to the uart low level driver structure for this port
 *	@line: line index previously returned from mp_register_port()
 *
 *	Hang up the specified line associated with the low level driver,
 *	and mark the port as unused.
 */
void mp_unregister_port(struct uart_driver *drv, int line)
{
	struct uart_state *state;

	if (line < 0 || line >= drv->nr) {
		printk(KERN_ERR "Attempt to unregister ");
		printk("%s%d", drv->dev_name, line);
		printk("\n");
		return;
	}

	state = drv->state + line;

	MP_MUTEX_LOCK(mp_mutex);
	mp_unconfigure_port(drv, state);
	MP_MUTEX_UNLOCK(mp_mutex);
}


static void serial_icr_write(struct mp_port *mtpt, int offset, int value)
{
	serial_out(mtpt, UART_SCR, offset);
	serial_out(mtpt, UART_ICR, value);
}


static void autoconfig(struct mp_port *mtpt, unsigned int probeflags)
{
	unsigned char status1, scratch, scratch2, scratch3;
	unsigned char save_lcr, save_mcr;
	unsigned long flags;

	unsigned char u_type;
	unsigned char b_ret = 0;

	if (!mtpt->port.iobase && !mtpt->port.mapbase && !mtpt->port.membase)
		return;

	DEBUG_AUTOCONF("ttyMP%d: autoconf (0x%04x, 0x%p): ",
			mtpt->port.line, mtpt->port.iobase, mtpt->port.membase);

	spin_lock_irqsave(&mtpt->port.lock, flags);

	if (!(mtpt->port.flags & UPF_BUGGY_UART)) {
		scratch = serial_inp(mtpt, UART_IER);
		serial_outp(mtpt, UART_IER, 0);
#ifdef __i386__
		outb(0xff, 0x080);
#endif
		scratch2 = serial_inp(mtpt, UART_IER) & 0x0f;
		serial_outp(mtpt, UART_IER, 0x0F);
#ifdef __i386__
		outb(0, 0x080);
#endif
		scratch3 = serial_inp(mtpt, UART_IER) & 0x0F;
		serial_outp(mtpt, UART_IER, scratch);
		if (scratch2 != 0 || scratch3 != 0x0F) {
			/*
			 * We failed; there's nothing here
			 */
			DEBUG_AUTOCONF("IER test failed (%02x, %02x) ",
					scratch2, scratch3);
			goto out;
		}
	}

	save_mcr = serial_in(mtpt, UART_MCR);
	save_lcr = serial_in(mtpt, UART_LCR);

	if (!(mtpt->port.flags & UPF_SKIP_TEST)) {
		serial_outp(mtpt, UART_MCR, UART_MCR_LOOP | 0x0A);
		status1 = serial_inp(mtpt, UART_MSR) & 0xF0;
		serial_outp(mtpt, UART_MCR, save_mcr);
		if (status1 != 0x90) {
			DEBUG_AUTOCONF("LOOP test failed (%02x) ",
					status1);
			goto out;
		}
	}

	serial_outp(mtpt, UART_LCR, 0xBF);
	serial_outp(mtpt, UART_EFR, 0);
	serial_outp(mtpt, UART_LCR, 0);

	serial_outp(mtpt, UART_FCR, UART_FCR_ENABLE_FIFO);
	scratch = serial_in(mtpt, UART_IIR) >> 6;

	DEBUG_AUTOCONF("iir=%d ", scratch);

	b_ret = read_option_register(mtpt,(MP_OPTR_DIR0 + ((mtpt->port.line)/8)));

	u_type = (b_ret & 0xf0) >> 4;
	/* get uart type infomation */
	if(mtpt->port.type == PORT_UNKNOWN )
	{
		switch (u_type)
		{
			case DIR_UART_16C550:
				mtpt->port.type = PORT_16C55X;
				break;
			case DIR_UART_16C1050:
				mtpt->port.type = PORT_16C105X;
				break;
			default:	
				mtpt->port.type = PORT_UNKNOWN;
				break;
		}
	}
	DPRINTK("%s)%d: uart = 0x%x \n ", __FUNCTION__,mtpt->port.line,mtpt->port.type);

	if(mtpt->port.type == PORT_UNKNOWN )
	{
		switch (scratch) {
			case 0:
			case 1:
				mtpt->port.type = PORT_UNKNOWN;
				break;
			case 2:
			case 3:
				mtpt->port.type = PORT_16C55X;
				break;
		}
	}

	serial_outp(mtpt, UART_LCR, save_lcr);

	mtpt->port.fifosize = uart_config[mtpt->port.type].dfl_xmit_fifo_size;
	mtpt->capabilities = uart_config[mtpt->port.type].flags;

	if (mtpt->port.type == PORT_UNKNOWN)
		goto out;
	/*
	 * Reset the UART.
	 */
	serial_outp(mtpt, UART_MCR, save_mcr);
	serial_outp(mtpt, UART_FCR, (UART_FCR_ENABLE_FIFO |
				UART_FCR_CLEAR_RCVR |
				UART_FCR_CLEAR_XMIT));
	serial_outp(mtpt, UART_FCR, 0);
	(void)serial_in(mtpt, UART_RX);
	serial_outp(mtpt, UART_IER, 0);

out:
	spin_unlock_irqrestore(&mtpt->port.lock, flags);
	//  restore_flags(flags);
	DEBUG_AUTOCONF("type=%s\n", uart_config[mtpt->port.type].name);
}

static void autoconfig_irq(struct mp_port *mtpt)
{
	unsigned char save_mcr, save_ier;
	unsigned char save_ICP = 0;
	unsigned int ICP = 0;
	unsigned long irqs;
	int irq;

	if (mtpt->port.flags & UPF_FOURPORT) {
		ICP = (mtpt->port.iobase & 0xfe0) | 0x1f;
		save_ICP = inb_p(ICP);
		outb_p(0x80, ICP);
		(void) inb_p(ICP);
	}

	/* forget possible initially masked and pending IRQ */
	probe_irq_off(probe_irq_on());
	save_mcr = serial_inp(mtpt, UART_MCR);
	save_ier = serial_inp(mtpt, UART_IER);
	serial_outp(mtpt, UART_MCR, UART_MCR_OUT1 | UART_MCR_OUT2);

	irqs = probe_irq_on();
	serial_outp(mtpt, UART_MCR, 0);
	udelay (10);
	if (mtpt->port.flags & UPF_FOURPORT)  {
		serial_outp(mtpt, UART_MCR,
				UART_MCR_DTR | UART_MCR_RTS);
	} else {
		serial_outp(mtpt, UART_MCR,
				UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2);
	}
	serial_outp(mtpt, UART_IER, 0x0f);    /* enable all intrs */
	(void)serial_inp(mtpt, UART_LSR);
	(void)serial_inp(mtpt, UART_RX);
	(void)serial_inp(mtpt, UART_IIR);
	(void)serial_inp(mtpt, UART_MSR);
	serial_outp(mtpt, UART_TX, 0xFF);
	udelay (20);
	irq = probe_irq_off(irqs);

	serial_outp(mtpt, UART_MCR, save_mcr);
	serial_outp(mtpt, UART_IER, save_ier);

	if (mtpt->port.flags & UPF_FOURPORT)
		outb_p(save_ICP, ICP);

	mtpt->port.irq = (irq > 0) ? irq : 0;
}


#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,14))
static void multi_stop_tx(struct uart_port *port, unsigned int tty_stop)
#else
static void multi_stop_tx(struct uart_port *port)
#endif
{
	struct mp_port *mtpt = (struct mp_port *)port;

	if (mtpt->ier & UART_IER_THRI) {
		mtpt->ier &= ~UART_IER_THRI;
		serial_out(mtpt, UART_IER, mtpt->ier);
	}
}

#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,14))
static void multi_start_tx(struct uart_port *port, unsigned int tty_start)
#else
static void multi_start_tx(struct uart_port *port)
#endif
{
	struct mp_port *mtpt = (struct mp_port *)port;

	if (!(mtpt->ier & UART_IER_THRI)) {
		mtpt->ier |= UART_IER_THRI;
		serial_out(mtpt, UART_IER, mtpt->ier);
	}
}

static void multi_stop_rx(struct uart_port *port)
{
	struct mp_port *mtpt = (struct mp_port *)port;

	mtpt->ier &= ~UART_IER_RLSI;
	mtpt->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(mtpt, UART_IER, mtpt->ier);
}

static void multi_enable_ms(struct uart_port *port)
{
	struct mp_port *mtpt = (struct mp_port *)port;

	mtpt->ier |= UART_IER_MSI;
	serial_out(mtpt, UART_IER, mtpt->ier);
}


static _INLINE_ void receive_chars(struct mp_port *mtpt, int *status )
{
	struct tty_struct *tty = mtpt->port.info.tty;
	unsigned char lsr = *status;
	int max_count = 256;
	unsigned char ch;
	char flag;

	lsr &= mtpt->port.read_status_mask;

	do {

		if (lsr & UART_LSR_SPECIAL) {
			flag = 0;
			ch = serial_inp(mtpt, UART_RX);

			if (lsr & UART_LSR_BI) 
			{

				mtpt->port.icount.brk++;
				flag = TTY_BREAK;

				if (uart_handle_break(&mtpt->port))
					goto ignore_char;
			} 
			if (lsr & UART_LSR_PE)
			{
				mtpt->port.icount.parity++;
				flag = TTY_PARITY;
			}
			if (lsr & UART_LSR_FE)
			{
				mtpt->port.icount.frame++;
				flag = TTY_FRAME;
			}
			if (lsr & UART_LSR_OE)
			{
				mtpt->port.icount.overrun++;
				flag = TTY_OVERRUN;
			}
			tty_insert_flip_char(tty, ch, flag);
		}
		else
		{
			ch = serial_inp(mtpt, UART_RX);
			tty_insert_flip_char(tty, ch, 0);
		}
ignore_char:
		lsr = serial_inp(mtpt, UART_LSR);
	} while ((lsr & UART_LSR_DR) && (max_count-- > 0));

	tty_flip_buffer_push(tty);
}




static _INLINE_ void transmit_chars(struct mp_port *mtpt)
{
	struct circ_buf *xmit = &mtpt->port.info->xmit;
	int count;

	if (mtpt->port.x_char) {
		serial_outp(mtpt, UART_TX, mtpt->port.x_char);
		mtpt->port.icount.tx++;
		mtpt->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&mtpt->port)) {
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,14))
		multi_stop_tx(&mtpt->port, 0);
#else
		multi_stop_tx(&mtpt->port);
#endif
		return;
	}

	count = uart_circ_chars_pending(xmit);
	if(count > mtpt->port.fifosize)
	{
		count = mtpt->port.fifosize;
	}

	do {
		serial_out(mtpt, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		mtpt->port.icount.tx++;
	} while (--count > 0);
}



static _INLINE_ void check_modem_status(struct mp_port *mtpt)
{
	int status;

	status = serial_in(mtpt, UART_MSR);

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return;

	if (status & UART_MSR_TERI)
		mtpt->port.icount.rng++;
	if (status & UART_MSR_DDSR)
		mtpt->port.icount.dsr++;
	if (status & UART_MSR_DDCD)
		uart_handle_dcd_change(&mtpt->port, status & UART_MSR_DCD);
	if (status & UART_MSR_DCTS)
		uart_handle_cts_change(&mtpt->port, status & UART_MSR_CTS);

	// //wake_up_interruptible(&mtpt->info.delta_msr_wait);
}

static inline void multi_handle_port(struct mp_port *mtpt)
{
	unsigned int status = serial_inp(mtpt, UART_LSR);
	int count=0;

	DEBUG_INTR("status = %x...", status);

	if ((status & UART_LSR_DR) || (status & UART_LSR_SPECIAL))
		receive_chars(mtpt, &status);
	check_modem_status(mtpt);
	if (status & UART_LSR_THRE)
	{
		if (mtpt->interface > 3)
			uart_set_mctrl(&mtpt->port, TIOCM_RTS);

		transmit_chars(mtpt);
		if (mtpt->interface > 3)
			while((serial_in(mtpt,UART_LSR) &0x60)!=0x60)
			{
				if (count++>5000) break;
			}

		if (mtpt->interface > 3)
			uart_clear_mctrl(&mtpt->port, TIOCM_RTS);
	}
}



#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,18))
static irqreturn_t multi_interrupt(int irq, void *dev_id)
#else
static irqreturn_t multi_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
	struct irq_info *iinfo = dev_id;
	struct list_head *lhead, *end = NULL;
	int pass_counter = 0;

	unsigned long addr;
	int j,k;

	for(k=0; k<NR_BOARD; k++){
		addr = mp_devs[k].option_reg_addr + MP_OPTR_IMR0;
		for(j=0; j < mp_devs[k].nr_ports/8; j++)
			outb(0x00,addr +j);
	}
	DEBUG_INTR("multi_interrupt(%d)...", irq);

	spin_lock(&iinfo->lock);

	lhead = iinfo->head;
	do {
		struct mp_port *mtpt;
		unsigned int iir;

		mtpt = list_entry(lhead, struct mp_port, list);

		iir = serial_in(mtpt, UART_IIR);
		if (!(iir & UART_IIR_NO_INT)) {
			spin_lock(&mtpt->port.lock);
			multi_handle_port(mtpt);
			spin_unlock(&mtpt->port.lock);

			end = NULL;
		} else if (end == NULL)
			end = lhead;

		lhead = lhead->next;

		if (lhead == iinfo->head && pass_counter++ > PASS_LIMIT) {
			/* If we hit this, we're dead. */
			printk(KERN_ERR "multi: too much work for "
					"irq%d\n", irq);
			break;
		}
	} while (lhead != end);

	spin_unlock(&iinfo->lock);

	for(k=0; k<NR_BOARD; k++){
		addr = mp_devs[k].option_reg_addr + MP_OPTR_IMR0;
		for(j=0; j < mp_devs[k].nr_ports/8; j++)
			outb(0xff,addr +j);
	}

	DEBUG_INTR("end.\n");
	/* FIXME! Was it really ours? */
	return IRQ_HANDLED;
}



static void serial_do_unlink(struct irq_info *i, struct mp_port *mtpt)
{
	spin_lock_irq(&i->lock);

	if (!list_empty(i->head)) {
		if (i->head == &mtpt->list)
			i->head = i->head->next;
		list_del(&mtpt->list);
	} else {
		BUG_ON(i->head != &mtpt->list);
		i->head = NULL;
	}

	spin_unlock_irq(&i->lock);
}

static int serial_link_irq_chain(struct mp_port *mtpt)
{
	struct irq_info *i = irq_lists + mtpt->port.irq;
#if (LINUX_VERSION_CODE > VERSION_CODE(2,6,18))
	int ret, irq_flags = mtpt->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;
#else
	int ret, irq_flags = mtpt->port.flags & UPF_SHARE_IRQ ? SA_SHIRQ : 0;
#endif	
	spin_lock_irq(&i->lock);

	if (i->head) {
		list_add(&mtpt->list, i->head);
		spin_unlock_irq(&i->lock);

		ret = 0;
	} else {
		INIT_LIST_HEAD(&mtpt->list);
		i->head = &mtpt->list;
		spin_unlock_irq(&i->lock);

		ret = request_irq(mtpt->port.irq, multi_interrupt,
				irq_flags, "serial", i);
		if (ret < 0)
			serial_do_unlink(i, mtpt);
	}

	return ret;
}




static void serial_unlink_irq_chain(struct mp_port *mtpt)
{
	struct irq_info *i = irq_lists + mtpt->port.irq;

	BUG_ON(i->head == NULL);

	DPRINTK("1:%s: free_irq %d \n",__FUNCTION__,mtpt->port.irq);

	if (list_empty(i->head))
	{
		DPRINTK("2:%s: free_irq %d \n",__FUNCTION__,mtpt->port.irq);
		free_irq(mtpt->port.irq, i);
	}
	serial_do_unlink(i, mtpt);
}

static void multi_timeout(unsigned long data)
{
	struct mp_port *mtpt = (struct mp_port *)data;
	unsigned int timeout;
	unsigned int iir;

	iir = serial_in(mtpt, UART_IIR);
	if (!(iir & UART_IIR_NO_INT)) {
		spin_lock(&mtpt->port.lock);
		multi_handle_port(mtpt);
		spin_unlock(&mtpt->port.lock);
	}

	timeout = mtpt->port.timeout;
	timeout = timeout > 6 ? (timeout / 2 - 2) : 1;
	mod_timer(&mtpt->timer, jiffies + timeout);
}

static unsigned int multi_tx_empty(struct uart_port *port)
{
	struct mp_port *mtpt = (struct mp_port *)port;
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&mtpt->port.lock, flags);
	ret = serial_in(mtpt, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&mtpt->port.lock, flags);

	return ret;
}


static unsigned int multi_get_mctrl(struct uart_port *port)
{
	struct mp_port *mtpt = (struct mp_port *)port;
	unsigned char status;
	unsigned int ret;

	status = serial_in(mtpt, UART_MSR);

#ifdef MP_DEBUG
	if(status != 0)
	{
		DPRINTK("%s: in status = 0x%x\n",__FUNCTION__,status);
	}
#endif

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
#ifdef MP_DEBUG
	if(ret != 0)
	{
		DPRINTK("%s: out msr = 0x%x\n",__FUNCTION__,ret);
	}
#endif
	return ret;
}

static void multi_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct mp_port *mtpt = (struct mp_port *)port;
	unsigned char mcr = 0;

	DPRINTK("%s: in mctrl = 0x%x\n",__FUNCTION__,mctrl);
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & mtpt->mcr_mask) | mtpt->mcr_force | mtpt->mcr;

	serial_out(mtpt, UART_MCR, mcr);
	DPRINTK("1:%s: out mcr = 0x%x \n",__FUNCTION__,mcr);
	mcr = serial_in(mtpt, UART_MCR);
	DPRINTK("2:%s: out mcr = 0x%x \n",__FUNCTION__,mcr);
}


static void multi_break_ctl(struct uart_port *port, int break_state)
{
	struct mp_port *mtpt = (struct mp_port *)port;
	unsigned long flags;

	spin_lock_irqsave(&mtpt->port.lock, flags);
	if (break_state == -1)
		mtpt->lcr |= UART_LCR_SBC;
	else
		mtpt->lcr &= ~UART_LCR_SBC;
	serial_out(mtpt, UART_LCR, mtpt->lcr);
	spin_unlock_irqrestore(&mtpt->port.lock, flags);
}



static int multi_startup(struct uart_port *port)
{
	struct mp_port *mtpt = (struct mp_port *)port;
	unsigned long flags;
	int retval;

	mtpt->capabilities = uart_config[mtpt->port.type].flags;
	mtpt->mcr = 0;

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reeanbled in set_termios())
	 */
	if (mtpt->capabilities & UART_CLEAR_FIFO) {
		serial_outp(mtpt, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_outp(mtpt, UART_FCR, UART_FCR_ENABLE_FIFO |
				UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_outp(mtpt, UART_FCR, 0);
	}

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_inp(mtpt, UART_LSR);
	(void) serial_inp(mtpt, UART_RX);
	(void) serial_inp(mtpt, UART_IIR);
	(void) serial_inp(mtpt, UART_MSR);


	if (!(mtpt->port.flags & UPF_BUGGY_UART) &&
			(serial_inp(mtpt, UART_LSR) == 0xff)) {
		printk("ttyS%d: LSR safety check engaged!\n", mtpt->port.line);
		return -ENODEV;
	}

	/*
	 * If the "interrupt" for this port doesn't correspond with any
	 * hardware interrupt, we use a timer-based system.  The original
	 * driver used to do this with IRQ0.
	 */
	if (!is_real_interrupt(mtpt->port.irq)) {
		unsigned int timeout = mtpt->port.timeout;

		timeout = timeout > 6 ? (timeout / 2 - 2) : 1;

		mtpt->timer.data = (unsigned long)up;
		mod_timer(&mtpt->timer, jiffies + timeout);
	} 
	else 
	{
		retval = serial_link_irq_chain(mtpt);
		if (retval)
			return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_outp(mtpt, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&mtpt->port.lock, flags);
	if (mtpt->port.flags & UPF_FOURPORT) {
		if (!is_real_interrupt(mtpt->port.irq))
			mtpt->port.mctrl |= TIOCM_OUT1;
	} else
		/*
		 * Most PC uarts need OUT2 raised to enable interrupts.
		 */
		if (is_real_interrupt(mtpt->port.irq))
			mtpt->port.mctrl |= TIOCM_OUT2;

	multi_set_mctrl(&mtpt->port, mtpt->port.mctrl);
	spin_unlock_irqrestore(&mtpt->port.lock, flags);

	mtpt->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_outp(mtpt, UART_IER, mtpt->ier);

	if (mtpt->port.flags & UPF_FOURPORT) {
		unsigned int icp;
		/*
		 * Enable interrupts on the AST Fourport board
		 */
		icp = (mtpt->port.iobase & 0xfe0) | 0x01f;
		outb_p(0x80, icp);
		(void) inb_p(icp);
	}

	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void) serial_inp(mtpt, UART_LSR);
	(void) serial_inp(mtpt, UART_RX);
	(void) serial_inp(mtpt, UART_IIR);
	(void) serial_inp(mtpt, UART_MSR);

	return 0;
}



static void multi_shutdown(struct uart_port *port)
{
	struct mp_port *mtpt = (struct mp_port *)port;
	unsigned long flags;

	DPRINTK("%s in \n",__FUNCTION__);

	/*
	 * Disable interrupts from this port
	 */
	mtpt->ier = 0;
	serial_outp(mtpt, UART_IER, 0);

	spin_lock_irqsave(&mtpt->port.lock, flags);
	if (mtpt->port.flags & UPF_FOURPORT) {
		/* reset interrupts on the AST Fourport board */
		inb((mtpt->port.iobase & 0xfe0) | 0x1f);
		mtpt->port.mctrl |= TIOCM_OUT1;
	} else
		mtpt->port.mctrl &= ~TIOCM_OUT2;

	multi_set_mctrl(&mtpt->port, mtpt->port.mctrl);
	spin_unlock_irqrestore(&mtpt->port.lock, flags);

	release_region(mtpt->port.iobase,8*NR_PORTS);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(mtpt, UART_LCR, serial_inp(mtpt, UART_LCR) & ~UART_LCR_SBC);
	serial_outp(mtpt, UART_FCR, UART_FCR_ENABLE_FIFO |
			UART_FCR_CLEAR_RCVR |
			UART_FCR_CLEAR_XMIT);
	serial_outp(mtpt, UART_FCR, 0);


	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	(void) serial_in(mtpt, UART_RX);

	if (!is_real_interrupt(mtpt->port.irq))
	{
		DPRINTK("%s: this doesn't use interrupt %d \n",__FUNCTION__,mtpt->port.irq);
		del_timer_sync(&mtpt->timer);
	}
	else
	{
		DPRINTK("%s: this use interrupt %d \n",__FUNCTION__,mtpt->port.irq);
		serial_unlink_irq_chain(mtpt);
	}
	DPRINTK("%s out \n",__FUNCTION__);
}



static unsigned int multi_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	/*
	 * Handle magic divisors for baud rates above baud_base on
	 * SMSC SuperIO chips.
	 */
	if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
			baud == (port->uartclk/4))
		quot = 0x8001;
	else if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
			baud == (port->uartclk/8))
		quot = 0x8002;
	else
		quot = uart_get_divisor(port, baud);

	return quot;
}




static void
multi_set_termios(struct uart_port *port, struct MP_TERMIOS *termios,
		struct MP_TERMIOS *old)
{
	struct mp_port *mtpt = (struct mp_port *)port;
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
		case CS5:
			cval = 0x00;
			break;
		case CS6:
			cval = 0x01;
			break;
		case CS7:
			cval = 0x02;
			break;
		default:
		case CS8:
			cval = 0x03;
			break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= 0x04;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = multi_get_divisor(port, baud);

	/*
	 * Work around a bug in the Oxford Semiconductor 952 rev B
	 * chip which causes it to seriously miscalculate baud rates
	 * when DLL is 0.
	 */
	if (mtpt->capabilities & UART_USE_FIFO) {
		if (baud < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
		else
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_8;
	}

	/*
	 * TI16C750: hardware flow control and 64 byte FIFOs. When AFE is
	 * enabled, RTS will be deasserted when the receive FIFO contains
	 * more characters than the trigger, or the MCR RTS bit is cleared.
	 */
	spin_lock_irqsave(&mtpt->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	mtpt->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		mtpt->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		mtpt->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	mtpt->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		mtpt->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		mtpt->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			mtpt->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		mtpt->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	mtpt->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&mtpt->port, termios->c_cflag))
		mtpt->ier |= UART_IER_MSI;

	serial_out(mtpt, UART_IER, mtpt->ier);

	if (mtpt->capabilities & UART_STARTECH) {
		serial_outp(mtpt, UART_LCR, 0xBF);
		serial_outp(mtpt, UART_EFR,
				termios->c_cflag & CRTSCTS ? UART_EFR_CTS :0);
	}

	if (mtpt->capabilities & UART_NATSEMI) {
		/* Switch to bank 2 not bank 1, to avoid resetting EXCR2 */
		serial_outp(mtpt, UART_LCR, 0xe0);
	} else {
		serial_outp(mtpt, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */
	}

	serial_outp(mtpt, UART_DLL, quot & 0xff);     /* LS of divisor */
	serial_outp(mtpt, UART_DLM, quot >> 8);       /* MS of divisor */

	/*
	 * LCR DLAB must be set to enable 64-byte FIFO mode. If the FCR
	 * is written without DLAB set, this mode will be disabled.
	 */

	serial_outp(mtpt, UART_LCR, cval);        /* reset DLAB */
	mtpt->lcr = cval;                 /* Save LCR */

	if (fcr & UART_FCR_ENABLE_FIFO) {
		/* emulated UARTs (Lucent Venus 167x) need two steps */
		serial_outp(mtpt, UART_FCR, UART_FCR_ENABLE_FIFO);
	}

	serial_outp(mtpt, UART_FCR, fcr);     /* set fcr */

	/*
	 * if 16C1050 core, enable the deep fifo
	 */
	if (mtpt->port.type == PORT_16C105X)
	{
		set_deep_fifo(port, ENABLE);
	}

	if (mtpt->interface >= RS485NE)
	{
		set_auto_rts(port,ENABLE);
	}

	multi_set_mctrl(&mtpt->port, mtpt->port.mctrl);
	spin_unlock_irqrestore(&mtpt->port.lock, flags);
}




static void
multi_pm(struct uart_port *port, unsigned int state,
		unsigned int oldstate)
{
	struct mp_port *mtpt = (struct mp_port *)port;
	if (state) {
		/* sleep */
		if (mtpt->capabilities & UART_STARTECH) {
			/* Arrange to enter sleep mode */
			serial_outp(mtpt, UART_LCR, 0xBF);
			serial_outp(mtpt, UART_EFR, UART_EFR_ECB);
			serial_outp(mtpt, UART_LCR, 0);
			serial_outp(mtpt, UART_IER, UART_IERX_SLEEP);
			serial_outp(mtpt, UART_LCR, 0xBF);
			serial_outp(mtpt, UART_EFR, 0);
			serial_outp(mtpt, UART_LCR, 0);
		}

		if (mtpt->pm)
			mtpt->pm(port, state, oldstate);
	} 
	else 
	{
		/* wake */
		if (mtpt->capabilities & UART_STARTECH) {
			/* Wake up UART */
			serial_outp(mtpt, UART_LCR, 0xBF);
			serial_outp(mtpt, UART_EFR, UART_EFR_ECB);
			/*
			 * Turn off LCR == 0xBF so we actually set the IER
			 * register on the XR16C850
			 */
			serial_outp(mtpt, UART_LCR, 0);
			serial_outp(mtpt, UART_IER, 0);


			serial_outp(mtpt, UART_LCR, 0xBF);
			serial_outp(mtpt, UART_EFR, 0);
			serial_outp(mtpt, UART_LCR, 0);
		}

		if (mtpt->pm)
			mtpt->pm(port, state, oldstate);
	}
}

static void multi_release_std_resource(struct mp_port *mtpt)
{
	unsigned int size = 8 << mtpt->port.regshift;

	switch (mtpt->port.iotype) {
		case UPIO_MEM:
			if (!mtpt->port.mapbase)
				break;

			if (mtpt->port.flags & UPF_IOREMAP) {
				iounmap(mtpt->port.membase);
				mtpt->port.membase = NULL;
			}

			release_mem_region(mtpt->port.mapbase, size);
			break;

		case UPIO_HUB6:
		case UPIO_PORT:
			release_region(mtpt->port.iobase,size);
			break;
	}
}

static void multi_release_port(struct uart_port *port)
{
}

static int multi_request_port(struct uart_port *port)
{
	return 0;
}

static void multi_config_port(struct uart_port *port, int flags)
{
	struct mp_port *mtpt = (struct mp_port *)port;
	int probeflags = PROBE_ANY;

	DPRINTK("%s : %d \n",__FUNCTION__,port->line);
	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	if (flags & UART_CONFIG_TYPE)
		autoconfig(mtpt, probeflags);
	if (mtpt->port.type != PORT_UNKNOWN && flags & UART_CONFIG_IRQ)
		autoconfig_irq(mtpt);

	if (mtpt->port.type == PORT_UNKNOWN)
		multi_release_std_resource(mtpt);
}

	static int
multi_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->irq >= NR_IRQS || ser->irq < 0 ||
			ser->baud_base < 9600 || ser->type < PORT_UNKNOWN ||
			ser->type == PORT_STARTECH)
		return -EINVAL;
	return 0;
}

	static const char *
multi_type(struct uart_port *port)
{
	int type = port->type;

	if (type >= ARRAY_SIZE(uart_config))
		type = 0;
	return uart_config[type].name;
}

static struct uart_ops multi_pops = {
	.tx_empty   = multi_tx_empty,
	.set_mctrl  = multi_set_mctrl,
	.get_mctrl  = multi_get_mctrl,
	.stop_tx    = multi_stop_tx,
	.start_tx   = multi_start_tx,
	.stop_rx    = multi_stop_rx,
	.enable_ms  = multi_enable_ms,
	.break_ctl  = multi_break_ctl,
	.startup    = multi_startup,
	.shutdown   = multi_shutdown,
	.set_termios    = multi_set_termios,
	.pm     = multi_pm,
	.type       = multi_type,
	.release_port   = multi_release_port,
	.request_port   = multi_request_port,
	.config_port    = multi_config_port,
	.verify_port    = multi_verify_port,
};

static struct uart_driver multi_reg = {
	.owner          = THIS_MODULE,
	.driver_name    = "multiport",
	.dev_name       = "ttyMP",
	.major          = TTY_MP_MAJOR,
	.minor          = 0,
	.nr             = 128, // TSUM
	.cons           = NULL,
};

static void __init multi_init_ports(void)
{
	struct mp_port *mtpt;
	static int first = 1;
	int i,j,k;
	unsigned char osc;
	unsigned char b_ret = 0;
	static struct mp_device_t * sbdev; 

	if (!first)
		return;
	first = 0;

	mtpt = multi_ports; // TSUM

	for (k=0;k<NR_BOARD;k++)
	{
		sbdev = &mp_devs[k];

		for (i = 0/*, mtpt = multi_ports*/; i < sbdev->nr_ports; i++, mtpt++) 
		{
			mtpt->port.iobase   = sbdev->uart_access_addr + 8*i;
			mtpt->port.irq      = sbdev->irq;
			if ( ((sbdev->device_id == PCI_DEVICE_ID_MP4)&&(sbdev->revision==0x91)))
				mtpt->interface_config_addr = sbdev->option_reg_addr + 0x08 + i;
			else
				mtpt->interface_config_addr = sbdev->option_reg_addr + 0x08 + i/8;

			mtpt->option_base_addr = sbdev->option_reg_addr;

			mtpt->port.uartclk  = BASE_BAUD * 16;
			/* get input clock infomation */
			osc = inb(sbdev->option_reg_addr + MP_OPTR_DIR0 + i/8) & 0x0F;
			if (osc==0x0f)
				osc = 0;
			for(j=0;j<osc;j++)
				mtpt->port.uartclk *= 2;
			mtpt->port.flags    |= STD_COM_FLAGS | UPF_SHARE_IRQ ;
			mtpt->port.iotype   = UPIO_PORT;
			mtpt->port.ops      = &multi_pops;

			b_ret = read_option_register(mtpt,(MP_OPTR_IIR0 + i/8));
			if(IIR_RS232 == (b_ret & IIR_RS232))
			{
				mtpt->interface = RS232;
			}
			if(IIR_RS422 == (b_ret & IIR_RS422))
			{
				mtpt->interface = RS422PTP;
			}
			if(IIR_RS485 == (b_ret & IIR_RS485))
			{
				mtpt->interface = RS485NE;
			}
		}
	}
}

static void print_info(void)
{
	int i,n=0,port,IIR;

	printk("==============================================\n");
	printk("      MultiPorts/PCI Board Installation      \n");
	printk("     version: %s  revision: %s\n",multi_version,multi_revdate);
	printk("        email :  <tech@mp.com>           \n");
	printk("==============================================\n");
	printk(" %d board(s) installed.   %d ports availabe. \n",NR_BOARD,NR_PORTS);

	for(i=0; i<NR_BOARD;i++)
	{
		printk(" Board No.%d %s (rev %x) ttyMP%d ~ ttyMP%d using IRQ%d\n",
				i, mp_devs[i].name, mp_devs[i].revision, n , (n + mp_devs[i].nr_ports)-1,
				mp_devs[i].irq);

		for (port=0; port < mp_devs[i].nr_ports; port++) 
		{
			IIR = inb(mp_devs[i].option_reg_addr + MP_OPTR_IIR0 + (port/8));
			if((IIR & 0xf0) == 0x00)
			{
				printk(" %d port --- RS232\n", port+1);
			}
			else if ((IIR & 0xf0) == 0x10) 
			{
				printk(" %d port --- RS422 and Point to Point mode\n", port+1);
			}
			else if ((IIR & 0xf0) == 0x20) {
				printk(" %d port --- RS485 and Non Echo mode\n", port+1);
			}
		}
		n += mp_devs[i].nr_ports;
	}
}

static void __init multi_register_ports(struct uart_driver *drv)
{
	int i;

	print_info();

	multi_init_ports();

	for (i = 0; i < NR_PORTS; i++) {
		struct mp_port *mtpt = &multi_ports[i];

		mtpt->port.line = i;
		mtpt->port.ops = &multi_pops;
		init_timer(&mtpt->timer);
		mtpt->timer.function = multi_timeout;

		/*
		 * ALPHA_KLUDGE_MCR needs to be killed.
		 */
		mtpt->mcr_mask = ~ALPHA_KLUDGE_MCR;
		mtpt->mcr_force = ALPHA_KLUDGE_MCR;

		mp_add_one_port(drv, &mtpt->port);
	}
}

static int __register_serial(struct serial_struct *req, int line)
{
	struct uart_port port;

	port.iobase   = req->port;
	port.membase  = req->iomem_base;
	port.irq      = req->irq;
	port.uartclk  = req->baud_base * 16;
	port.fifosize = req->xmit_fifo_size;
	port.regshift = req->iomem_reg_shift;
	port.iotype   = req->io_type;
	port.flags    = req->flags | UPF_BOOT_AUTOCONF;
	port.mapbase  = req->iomap_base;
	port.line     = line;

	if (share_irqs)
		port.flags |= UPF_SHARE_IRQ;

	if (HIGH_BITS_OFFSET)
		port.iobase |= (long) req->port_high << HIGH_BITS_OFFSET;

	if (port.uartclk == 0)
		port.uartclk = BASE_BAUD * 16;

	return mp_register_port(&multi_reg, &port);
}
int register_serial(struct serial_struct *req)
{
	return __register_serial(req, -1);
}

int __init early_serial_setup(struct uart_port *port)
{
	if (port->line >= ARRAY_SIZE(multi_ports))
		return -ENODEV;

	multi_init_ports();
	multi_ports[port->line].port    = *port;
	multi_ports[port->line].port.ops    = &multi_pops;
	return 0;
}

void unregister_serial(int line)
{
	mp_unregister_port(&multi_reg, line);
}

/*
 * This is for ISAPNP only.
 */
void multi_get_irq_map(unsigned int *map)
{
	int i;

	for (i = 0; i < NR_PORTS; i++) {
		if (multi_ports[i].port.type != PORT_UNKNOWN &&
				multi_ports[i].port.irq < 16)
			*map |= 1 << multi_ports[i].port.irq;
	}
}

/**
 *  multi_suspend_port - suspend one serial port
 *  @line:  serial line number
 *      @level: the level of port suspension, as per uart_suspend_port
 *
 *  Suspend one serial port.
 */
void multi_suspend_port(int line)
{
	mp_suspend_port(&multi_reg, &multi_ports[line].port);
}

/**
 *  multi_resume_port - resume one serial port
 *  @line:  serial line number
 *      @level: the level of port resumption, as per uart_resume_port
 *
 *  Resume one serial port.
 */
void multi_resume_port(int line)
{
	mp_resume_port(&multi_reg, &multi_ports[line].port);
}

static int init_mp_dev(struct pci_dev *pcidev, mppcibrd_t brd)
{
	static struct mp_device_t * sbdev = mp_devs;
	unsigned long addr = 0;
	int j;
	struct resource * ret = NULL;

	sbdev->device_id = brd.device_id;
	pci_read_config_byte(pcidev, PCI_CLASS_REVISION, &(sbdev->revision));
	sbdev->name = brd.name;
	sbdev->uart_access_addr = pcidev->resource[0].start & PCI_BASE_ADDRESS_IO_MASK;
	sbdev->option_reg_addr = pcidev->resource[1].start & PCI_BASE_ADDRESS_IO_MASK;
	sbdev->irq = pcidev->irq;

	DPRINTK("%s: uart_access_addr=%ld,option_reg_addr=%ld,irq = %d, name= %s,device_id = %d,revision = %d \n",__FUNCTION__,sbdev->uart_access_addr,sbdev->option_reg_addr,sbdev->irq,sbdev->name,sbdev->device_id,sbdev->revision);

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

	ret = request_region(sbdev->uart_access_addr, 8*sbdev->nr_ports, sbdev->name);
	DPRINTK("uart_access_addr ret = %p \n",ret);

	ret = request_region(sbdev->option_reg_addr, 32, sbdev->name);
	DPRINTK("option_reg_addr ret = %p \n",ret);


	NR_BOARD++;
	NR_PORTS += sbdev->nr_ports;

	/* Enable PCI interrupt */

	addr = sbdev->option_reg_addr + MP_OPTR_IMR0;
	for(j=0; j < sbdev->nr_ports/8; j++)
		outb(0xff,addr +j);

	sbdev++;

	return 0;
}

static int __init multi_init(void)
{
	int ret, i;
	struct pci_dev  *dev = NULL;

	/* TSUM */
	for( i=0; i< mp_nrpcibrds; i++)
	{
#if (LINUX_VERSION_CODE < VERSION_CODE(2,6,21))
		while( (dev = pci_find_device(mp_pciboards[i].vendor_id, mp_pciboards[i].device_id, dev) ) )
#else
			while( (dev = pci_get_device(mp_pciboards[i].vendor_id, mp_pciboards[i].device_id, dev) ) )
#endif
			{
				init_mp_dev(dev, mp_pciboards[i]);
			}
	}

	for (i = 0; i < NR_IRQS; i++)
		spin_lock_init(&irq_lists[i].lock);

	ret = mp_register_driver(&multi_reg);

	if (ret >= 0)
		multi_register_ports(&multi_reg);

	return ret;
}

static void __exit multi_exit(void)
{
	int i;

	for (i = 0; i < NR_PORTS; i++)
		mp_remove_one_port(&multi_reg, &multi_ports[i].port);

	mp_unregister_driver(&multi_reg);
}

module_init(multi_init);
module_exit(multi_exit);

EXPORT_SYMBOL(register_serial);
EXPORT_SYMBOL(unregister_serial);
EXPORT_SYMBOL(multi_get_irq_map);
EXPORT_SYMBOL(multi_suspend_port);
EXPORT_SYMBOL(multi_resume_port);
////////////////////////////////////////////////////


EXPORT_SYMBOL(mp_write_wakeup);
EXPORT_SYMBOL(mp_register_driver);
EXPORT_SYMBOL(mp_unregister_driver);
EXPORT_SYMBOL(mp_suspend_port);
EXPORT_SYMBOL(mp_resume_port);
EXPORT_SYMBOL(mp_register_port);
EXPORT_SYMBOL(mp_unregister_port);
EXPORT_SYMBOL(mp_add_one_port);
EXPORT_SYMBOL(mp_remove_one_port);

MODULE_DESCRIPTION("Multiport PCI CORE");
MODULE_LICENSE("GPL");
