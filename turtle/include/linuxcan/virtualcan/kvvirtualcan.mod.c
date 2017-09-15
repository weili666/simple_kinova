#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
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
	{ 0x1e94b2a0, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x651c85c2, __VMLINUX_SYMBOL_STR(os_if_spin_unlock) },
	{ 0x7489367b, __VMLINUX_SYMBOL_STR(vCanCleanup) },
	{ 0x906447e8, __VMLINUX_SYMBOL_STR(queue_length) },
	{ 0x91831d70, __VMLINUX_SYMBOL_STR(seq_printf) },
	{ 0xc49e8552, __VMLINUX_SYMBOL_STR(vCanTime) },
	{ 0x170220be, __VMLINUX_SYMBOL_STR(queue_empty) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0xd5065308, __VMLINUX_SYMBOL_STR(queue_front) },
	{ 0xc229a07c, __VMLINUX_SYMBOL_STR(vCanDispatchEvent) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x691087f9, __VMLINUX_SYMBOL_STR(queue_wakeup_on_space) },
	{ 0x6f580dd5, __VMLINUX_SYMBOL_STR(queue_pop) },
	{ 0x7e02d9a9, __VMLINUX_SYMBOL_STR(vCanInitData) },
	{ 0x451c7bd4, __VMLINUX_SYMBOL_STR(os_if_kernel_malloc) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0x9338dd27, __VMLINUX_SYMBOL_STR(os_if_kernel_free) },
	{ 0xfa49442f, __VMLINUX_SYMBOL_STR(os_if_set_task_uninterruptible) },
	{ 0xc2cb47a4, __VMLINUX_SYMBOL_STR(vCanInit) },
	{ 0x1a737dbd, __VMLINUX_SYMBOL_STR(vCanFlushSendBuffer) },
	{ 0x6e94147d, __VMLINUX_SYMBOL_STR(queue_release) },
	{ 0x4a2261d9, __VMLINUX_SYMBOL_STR(os_if_wait_for_event_timeout_simple) },
	{ 0xdb5bb4d0, __VMLINUX_SYMBOL_STR(os_if_spin_lock) },
	{ 0xbac4b01c, __VMLINUX_SYMBOL_STR(os_if_wake_up_interruptible) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=kvcommon";


MODULE_INFO(srcversion, "D9F0F9EBF7793B9EADAA902");
