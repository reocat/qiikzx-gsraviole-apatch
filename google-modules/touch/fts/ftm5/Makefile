KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_FTS=m
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_TBN
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_HEATMAP
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_OFFLOAD
EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/display
EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/touch/common
EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/touch/common/include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(@)
