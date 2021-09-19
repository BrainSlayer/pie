# SPDX-License-Identifier: GPL-2.0-only

define Device/zyxel_xgs1210-12
  SOC := rtl9302
  UIMAGE_MAGIC := 0x93001210
  DEVICE_VENDOR := Zyxel
  DEVICE_MODEL := XGS1210-12
  KERNEL := kernel-bin | append-dtb | gzip | custom-uimage OpenWRT
  KERNEL_INITRAMFS := kernel-bin | append-dtb | gzip | custom-uimage OpenWRT
  DEVICE_PACKAGES := ip-bridge ethtool
  IMAGES := sysupgrade.bin
  IMAGE/sysupgrade.bin := append-kernel | pad-to 64k | append-rootfs | pad-rootfs | \
	check-size | append-metadata
endef
TARGET_DEVICES += zyxel_xgs1210-12

define Device/zyxel_xgs1250-12
  SOC := rtl9302
  UIMAGE_MAGIC := 0x93001250
  DEVICE_VENDOR := Zyxel
  DEVICE_MODEL := XGS1250-12
  KERNEL := kernel-bin | append-dtb | gzip | custom-uimage OpenWRT
  KERNEL_INITRAMFS := kernel-bin | append-dtb | gzip | custom-uimage OpenWRT
  DEVICE_PACKAGES := ip-bridge ethtool
endef
TARGET_DEVICES += zyxel_xgs1250-12
