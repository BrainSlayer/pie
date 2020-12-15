#
# Copyright (C) 2008-2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

KERNEL_LOADADDR =	0x80000000
KERNEL_ENTRY =		0x80000400

define Device/Default
  PROFILES = Default
  KERNEL := kernel-bin | append-dtb | gzip | uImage gzip
  KERNEL_INITRAMFS := kernel-bin | append-dtb | gzip | uImage gzip
  DEVICE_DTS_DIR := ../dts
  DEVICE_DTS = $$(SOC)_$(1)
  IMAGES := sysupgrade.bin
  SUPPORTED_DEVICES := $(subst _,$(comma),$(1))
  sysupgrade_bin := append-kernel | pad-to 64k | append-rootfs | pad-rootfs
  IMAGE/sysupgrade.bin := append-kernel | pad-to 64k | append-rootfs | pad-rootfs | append-metadata | check-size $$$$(IMAGE_SIZE)
endef

define Build/netgear-uImage
	$(call Build/uImage,$(1) -M $(NETGEAR_KERNEL_MAGIC))
endef

define Build/custom-uimage
	mkimage -A $(LINUX_KARCH) \
		-O linux -T kernel \
		-C gzip -a $(KERNEL_LOADADDR) $(if $(UIMAGE_MAGIC),-M $(UIMAGE_MAGIC),) \
		-e $(if $(KERNEL_ENTRY),$(KERNEL_ENTRY),$(KERNEL_LOADADDR)) \
		-n '$(1)' -d $@ $@.new
	mv $@.new $@
endef

define Build/relocate-kernel
	rm -rf $@.relocate
	$(CP) ../../generic/image/relocate $@.relocate
	$(MAKE) -C $@.relocate KERNEL_ADDR=$(KERNEL_LOADADDR) CROSS_COMPILE=$(TARGET_CROSS)
	( \
		dd if=$@.relocate/loader.bin bs=32 conv=sync && \
		perl -e '@s = stat("$@"); print pack("V", @s[7])' && \
		cat $@ \
	) > $@.new
	mv $@.new $@
	rm -rf $@.relocate
endef

#define Device/zyxel_gs1900-48
#  SOC := rtl8393
#  IMAGE_SIZE := 6976k
#  DEVICE_VENDOR := Zyxel
#  DEVICE_MODEL := GS1900-48
#  DEVICE_PACKAGES := ip-bridge ethtool
#endef
#TARGET_DEVICES += zyxel_gs1900-48

define Device/zyxel_xs1900-10
#  KERNEL_LOADADDR = 0x80300000
#  KERNEL_ENTRY = 0x80300000
#  KERNEL := kernel-bin | append-dtb | uImage none
#  KERNEL_INITRAMFS := kernel-bin | append-dtb | uImage none
  SOC := rtl9313
#  IMAGE_SIZE := 8000k
  DEVICE_VENDOR := Zyxel
  DEVICE_MODEL := XS1900-10
  DEVICE_PACKAGES := ip-bridge ethtool
endef
TARGET_DEVICES += zyxel_xs1900-10

define Device/zyxel_xgs1210-12
  SOC := rtl9302
  UIMAGE_MAGIC := 0x93001210
#  IMAGE_SIZE := 8000k
  DEVICE_VENDOR := Zyxel
  DEVICE_MODEL := XGS1210-12  
  KERNEL := kernel-bin | append-dtb | gzip | custom-uimage OpenWRT
  KERNEL_INITRAMFS := kernel-bin | append-dtb | gzip | custom-uimage OpenWRT

  DEVICE_PACKAGES := ip-bridge ethtool
endef
TARGET_DEVICES += zyxel_xgs1210-12

$(eval $(call BuildImage))
