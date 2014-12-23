#
# DRAGINO v2 si3217x FXS Daughter Board support
#
# Copyright (C) 2013-2014 Village Telco Ltd.
# Copyright (C) 2013-2014 Dimitar Penev <dpn at switchvoice dot com>
# Copyright (C) 2014 Vittorio Gambaletta <openwrt@vittgam.net>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=dragino2-si3217x
PKG_VERSION:=0.2
PKG_RELEASE:=1

# Remove the Build/Prepare section below if using Git.

#PKG_SOURCE_PROTO:=git
#PKG_SOURCE_URL:=https://github.com/VittGam/dragino2-si3217x.git
#PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
#PKG_SOURCE_VERSION:=put_commit_id_here
#PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION)-$(PKG_SOURCE_VERSION).tar.gz

PKG_LICENSE:=GPLv2, except for Silicon Labs binary blob
PKG_LICENSE_FILES:=si3217x/LICENSE
PKG_MAINTAINER:=Vittorio Gambaletta <openwrt@vittgam.net>

include $(INCLUDE_DIR)/package.mk

define KernelPackage/dragino2-si3217x
  SUBMENU:=Voice over IP
  TITLE:=DRAGINO v2 si3217x FXS Daughter Board driver
  VERSION:=$(LINUX_VERSION)+$(PKG_VERSION)-$(PKG_RELEASE)
  FILES:= $(PKG_BUILD_DIR)/proslic-api/dragino2_si3217x_proslic_api.$(LINUX_KMOD_SUFFIX) \
          $(PKG_BUILD_DIR)/si3217x/dragino2_si3217x.$(LINUX_KMOD_SUFFIX)
  AUTOLOAD:=$(call AutoProbe,dragino2_si3217x_proslic_api)
  DEPENDS:=@TARGET_ar71xx_generic_DRAGINO2 @GPIO_SUPPORT +kmod-spi-bitbang +kmod-spi-gpio +kmod-dahdi
endef

define KernelPackage/dragino2-si3217x/description
  This package contains the DRAGINO v2 si3217x FXS Daughter Board driver.
endef

define Package/dragino2-si3217x
  SECTION:=utils
  CATEGORY:=Utilities
  SUBMENU:=Telephony
  TITLE:=DRAGINO v2 si3217x FXS Daughter Board userspace support
  VERSION:=$(PKG_VERSION)-$(PKG_RELEASE)
  DEPENDS:=@TARGET_ar71xx_generic_DRAGINO2 +kmod-dragino2-si3217x +dahdi-cfg +kmod-dahdi-echocan-oslec
endef

define Package/dragino2-si3217x/description
  This package contains userspace support files for the DRAGINO v2 si3217x FXS Daughter Board.
endef

include $(INCLUDE_DIR)/kernel-defaults.mk

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) $(KERNEL_MAKEOPTS) SUBDIRS="$(PKG_BUILD_DIR)" modules
endef

define Package/dragino2-si3217x/conffiles
/etc/config/dragino2-si3217x
endef

define Package/dragino2-si3217x/install
	$(INSTALL_DIR) $(1)/etc

	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/userspace/dragino2-si3217x.init $(1)/etc/init.d/dragino2-si3217x

	$(INSTALL_DIR) $(1)/etc/config
	$(CP) $(PKG_BUILD_DIR)/userspace/dragino2-si3217x.config $(1)/etc/config/dragino2-si3217x
endef

$(eval $(call KernelPackage,dragino2-si3217x))
$(eval $(call BuildPackage,dragino2-si3217x))
