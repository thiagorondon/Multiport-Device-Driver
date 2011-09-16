#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x12771549, "struct_module" },
	{ 0x852abecf, "__request_region" },
	{ 0x90b997e1, "per_cpu__current_task" },
	{ 0xe9cc5ed0, "kmalloc_caches" },
	{ 0xe284b00, "pci_bus_read_config_byte" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0x9b388444, "get_zeroed_page" },
	{ 0x608c2831, "warn_on_slowpath" },
	{ 0x87e426cb, "uart_write_wakeup" },
	{ 0x1f9cfe83, "iomem_resource" },
	{ 0xd0d8621b, "strlen" },
	{ 0x84e663f9, "remove_wait_queue" },
	{ 0xf36d1823, "alloc_tty_driver" },
	{ 0xacee843f, "tty_hung_up_p" },
	{ 0x900e0d6, "_spin_lock" },
	{ 0x25ed77d9, "tty_name" },
	{ 0x3cd0bf13, "uart_get_divisor" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x26914d86, "tty_register_driver" },
	{ 0x73bcd634, "mutex_unlock" },
	{ 0x8a318f40, "put_tty_driver" },
	{ 0xeaa456ed, "_spin_lock_irqsave" },
	{ 0xe23b72eb, "uart_update_timeout" },
	{ 0x1d26aa98, "sprintf" },
	{ 0x7d11c268, "jiffies" },
	{ 0xe363828, "tty_set_operations" },
	{ 0xffd5a395, "default_wake_function" },
	{ 0xfa593d6a, "del_timer_sync" },
	{ 0xb407b205, "ioport_resource" },
	{ 0x1f0b43cc, "do_SAK" },
	{ 0x70d1f8f3, "strncat" },
	{ 0x7b9eb732, "mutex_lock_interruptible" },
	{ 0x91d6536d, "__mutex_init" },
	{ 0x1b7d4074, "printk" },
	{ 0x57f46cef, "_spin_lock_irq" },
	{ 0xa456c2bb, "tty_ldisc_flush" },
	{ 0xfaef0ed, "__tasklet_schedule" },
	{ 0x2da418b5, "copy_to_user" },
	{ 0x73e20c1c, "strlcpy" },
	{ 0xda3d7ad, "mutex_lock" },
	{ 0x27147e64, "_spin_unlock_irqrestore" },
	{ 0x9545af6d, "tasklet_init" },
	{ 0xb38e248b, "mod_timer" },
	{ 0x16fd6633, "tty_vhangup" },
	{ 0xd8903a41, "tty_insert_flip_string_flags" },
	{ 0xe523ad75, "synchronize_irq" },
	{ 0x5aaee706, "tty_register_device" },
	{ 0x61651be, "strcat" },
	{ 0x82072614, "tasklet_kill" },
	{ 0x7dceceac, "capable" },
	{ 0x2601baf6, "tty_unregister_device" },
	{ 0x90e276b8, "kmem_cache_alloc" },
	{ 0xed633abc, "pv_irq_ops" },
	{ 0xab600421, "probe_irq_off" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0xb2fd5ceb, "__put_user_4" },
	{ 0xc643085c, "tty_wait_until_sent" },
	{ 0x1fc91fb2, "request_irq" },
	{ 0xd62c833f, "schedule_timeout" },
	{ 0x4292364c, "schedule" },
	{ 0x58b80a29, "pv_cpu_ops" },
	{ 0xef79ac56, "__release_region" },
	{ 0x5967a56f, "tty_unregister_driver" },
	{ 0xffd3c7, "init_waitqueue_head" },
	{ 0x76c31941, "init_timer" },
	{ 0xb27ad967, "tty_hangup" },
	{ 0x4302d0eb, "free_pages" },
	{ 0x994e1983, "__wake_up" },
	{ 0x409873e3, "tty_termios_baud_rate" },
	{ 0xab2cd386, "add_wait_queue" },
	{ 0xb121390a, "probe_irq_on" },
	{ 0x37a0cba, "kfree" },
	{ 0xedc03953, "iounmap" },
	{ 0x381da1, "up" },
	{ 0x74a5dfc6, "pci_get_device" },
	{ 0x2d727820, "tty_flip_buffer_push" },
	{ 0x25da070, "snprintf" },
	{ 0xc17ff76d, "tty_wakeup" },
	{ 0xf2a644fb, "copy_from_user" },
	{ 0xcab0e5bd, "uart_get_baud_rate" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

