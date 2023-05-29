dtb-$(CONFIG_SOC_GS101) += \
	gs101-b0.dtb

soc-dtbo-$(CONFIG_SOC_GS101) += \
	gs101-dpm-user.dtbo \

DTC_FLAGS_gs101-dpm-eng += -a 4096
DTC_FLAGS_gs101-dpm-user += -a 4096

dtbo-y += \
	gs101-bluejay-mp1_0.dtbo

gs101-bluejay-mp1_0.dtbo-base := gs101-b0.dtb

gs101-dpm-user.dtbo-base := gs101-b0.dtb

MKDTIMG_FLAGS += --page_size=4096  --id=/:board_id --rev=/:board_rev
