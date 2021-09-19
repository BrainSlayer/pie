# SPDX-License-Identifier: GPL-2.0-only

define Device/zyxel_gs1900-48
  SOC := rtl8393
  IMAGE_SIZE := 13952k
  DEVICE_VENDOR := ZyXEL
  UIMAGE_MAGIC := 0x83800000
  ZYXEL_VERS := AAHO
  DEVICE_MODEL := GS1900-48
# KERNEL := kernel-bin | append-dtb | gzip | zyxel-vers $$$$(ZYXEL_VERS) | \
#	uImage gzip
  KERNEL_INITRAMFS := kernel-bin | append-dtb | gzip | zyxel-vers $$$$(ZYXEL_VERS) | \
	uImage gzip
#  IMAGES := sysupgrade.bin
#  IMAGE/sysupgrade.bin := append-kernel | pad-to 64k | append-rootfs | pad-rootfs | \
#	check-size | append-metadata
endef
TARGET_DEVICES += zyxel_gs1900-48

define Device/edgecore_ecs4100-12ph
  SOC := rtl8392
  IMAGE_SIZE := 14336k
  DEVICE_VENDOR := Edgecore
  DEVICE_MODEL := ECS4100-12PH
  DEVICE_PACKAGES += lua-rs232
endef
TARGET_DEVICES += edgecore_ecs4100-12ph
