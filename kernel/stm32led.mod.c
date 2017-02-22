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
	{ 0x380122c1, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x3186a04b, __VMLINUX_SYMBOL_STR(param_ops_bool) },
	{ 0x45f4466b, __VMLINUX_SYMBOL_STR(param_ops_uint) },
	{ 0xf3342c6e, __VMLINUX_SYMBOL_STR(class_unregister) },
	{ 0x65e14d2d, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0xcf6fa016, __VMLINUX_SYMBOL_STR(spi_bus_type) },
	{ 0x264586be, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0x1757ab0d, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0x6bc3fbc0, __VMLINUX_SYMBOL_STR(__unregister_chrdev) },
	{ 0x32f82cba, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0xaece8396, __VMLINUX_SYMBOL_STR(__register_chrdev) },
	{ 0x8574ca6c, __VMLINUX_SYMBOL_STR(gpio_request_array) },
	{ 0xc94afd85, __VMLINUX_SYMBOL_STR(device_unregister) },
	{ 0x9dfdf722, __VMLINUX_SYMBOL_STR(gpio_free_array) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x6f5fd15e, __VMLINUX_SYMBOL_STR(gpiod_set_raw_value) },
	{ 0x2c1e148, __VMLINUX_SYMBOL_STR(gpio_to_desc) },
	{ 0x605eb862, __VMLINUX_SYMBOL_STR(spi_setup) },
	{ 0xcdbb0bec, __VMLINUX_SYMBOL_STR(spi_new_device) },
	{ 0x7936eef7, __VMLINUX_SYMBOL_STR(device_del) },
	{ 0xf35160aa, __VMLINUX_SYMBOL_STR(bus_find_device_by_name) },
	{ 0xb81960ca, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x3efae5dc, __VMLINUX_SYMBOL_STR(spi_busnum_to_master) },
	{ 0x12da5bb2, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x153e0540, __VMLINUX_SYMBOL_STR(spi_sync) },
	{ 0xfa2a45e, __VMLINUX_SYMBOL_STR(__memzero) },
	{ 0x5f754e5a, __VMLINUX_SYMBOL_STR(memset) },
	{ 0x2e5810c6, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr1) },
	{ 0xb1ad28e0, __VMLINUX_SYMBOL_STR(__gnu_mcount_nc) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "72AF29216068E39F57CC9C8");
