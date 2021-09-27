# SPDX-License-Identifier: GPL-2.0-only

define Device/edgecore_ecs4125-10p
  KERNEL_LOADADDR = 0x80220000
  KERNEL_ENTRY = 0x80220000
  SOC := rtl9313
  IMAGE_SIZE := 14336k
  DEVICE_VENDOR := Edgecore
  DEVICE_MODEL := ECS4125-10P
  DEVICE_PACKAGES := ip-bridge ethtool
  DEVICE_PACKAGES += lua-rs232
endef
TARGET_DEVICES += edgecore_ecs4125-10p

define Device/zyxel_xs1900-10
  KERNEL_LOADADDR = 0x80220000
  KERNEL_ENTRY = 0x80220000
  SOC := rtl9313
  IMAGE_SIZE := 14336k
  DEVICE_MODEL := XS1900-10
  KERNEL := kernel-bin | append-dtb | gzip | custom-uimage OpenWRT
  KERNEL_INITRAMFS := kernel-bin | append-dtb | gzip | custom-uimage OpenWRT
  DEVICE_PACKAGES := ip-bridge ethtool
endef
TARGET_DEVICES += zyxel_xs1900-10

