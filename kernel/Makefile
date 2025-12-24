CROSS_ := riscv64-linux-gnu-
export GCC := $(CROSS_)gcc
export LD := $(CROSS_)ld
export OBJCOPY := $(CROSS_)objcopy
export OBJDUMP := $(CROSS_)objdump
export NM := $(CROSS_)nm

ISA := rv64ia_zicsr_zifencei
ABI := lp64

export CPPFLAGS := -I$(CURDIR)/include
export CFLAGS := -march=$(ISA) -mabi=$(ABI) -mcmodel=medany \
	-ffreestanding -fno-builtin -ffunction-sections -fdata-sections \
	-nostartfiles -nostdlib -nostdinc -static -ggdb -Og \
	-Wall -Wextra -std=gnu11
export LDFLAGS := -lgcc -Wl,--nmagic -Wl,--gc-sections

.PHONY: all run debug gdb snprintf clean spike_run spike_debug spike_bridge

all:
	$(MAKE) -C lib all
	$(MAKE) -C arch/riscv all
	$(LD) -T arch/riscv/kernel/vmlinux.lds arch/riscv/kernel/*.o lib/*.o -o vmlinux
	mkdir -p arch/riscv/boot
	$(OBJCOPY) -O binary vmlinux arch/riscv/boot/Image
	$(OBJDUMP) -S vmlinux > vmlinux.asm
	$(NM) vmlinux > System.map
	# Build finished!

SPIKE_CONF := $(realpath $(CURDIR)/../../../repo/sys-project/spike)

run: all
	# Launch the qemu ......
	qemu-system-riscv64 -nographic -machine virt -kernel vmlinux -bios $(SPIKE_CONF)/fw_jump.bin

gdb:all
# 请将../../../../code/.gdbinit 替换为你的代码目录下的 .gdbinit 文件
	uv run gdb-multiarch -x ../../../../code/.gdbinit vmlinux  

debug: all
	# Launch the qemu for debug ......
	qemu-system-riscv64 -nographic -machine virt -kernel vmlinux -bios $(SPIKE_CONF)/fw_jump.bin -S -s

SNPRINTF_TEST_DIR := $(realpath $(CURDIR)/../../../repo/sys-project/testcode/snprintf)
SNPRINTF_MAKEFILE := $(CURDIR)/lib/Makefile
snprintf: all
	$(MAKE) -C "$(SNPRINTF_TEST_DIR)" -f "$(SNPRINTF_MAKEFILE)" all
	$(LD) -T "$(SNPRINTF_TEST_DIR)/link.ld" arch/riscv/kernel/sbi.o arch/riscv/kernel/printk.o lib/*.o "$(SNPRINTF_TEST_DIR)"/*.o -o snprintf_test
	qemu-system-riscv64 -smp 8 -nographic -machine virt -kernel ./snprintf_test -bios $(SPIKE_CONF)/fw_jump.bin

spike_run: all
	spike --kernel=arch/riscv/boot/Image $(SPIKE_CONF)/fw_jump.elf

spike_debug: all
	spike -H --rbb-port=9824 --kernel=arch/riscv/boot/Image $(SPIKE_CONF)/fw_jump.elf

spike_bridge:
	openocd -f $(SPIKE_CONF)/spike.cfg

clean:
	$(MAKE) -C lib clean
	$(MAKE) -C arch/riscv clean
	$(MAKE) -C "$(SNPRINTF_TEST_DIR)" -f "$(SNPRINTF_MAKEFILE)" clean
	rm -rf vmlinux vmlinux.asm snprintf_test System.map arch/riscv/boot
	# Clean finished!
