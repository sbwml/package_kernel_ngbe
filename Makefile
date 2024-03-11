include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=ngbe
PKG_VERSION:=1.2.5.1
PKG_RELEASE:=1

PKG_LICENSE:=GPLv2
PKG_LICENSE_FILE:=COPYING
PKG_MAINTAINER:=sbwml <admin@cooluc.com>

PKG_BUILD_DIR:=$(KERNEL_BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)

include $(INCLUDE_DIR)/package.mk

define KernelPackage/ngbe
  TITLE:=Driver for WangXun GbE PCI Express ethernet
  SUBMENU:=Network Devices
  VERSION:=$(LINUX_VERSION)+$(PKG_VERSION)-$(BOARD)-$(PKG_RELEASE)
  DEPENDS:=@PCI_SUPPORT
  FILES:= $(PKG_BUILD_DIR)/src/ngbe.ko
  AUTOLOAD:=$(call AutoProbe,ngbe)
endef

define Package/ngbe/description
  WangXun(R) Gigabit PCI Express ethernet driver
endef

define Build/Compile
	+$(KERNEL_MAKE) M=$(PKG_BUILD_DIR)/src modules
endef

$(eval $(call KernelPackage,ngbe))
