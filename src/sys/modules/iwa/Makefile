# $FreeBSD$

.PATH:  ${.CURDIR}/../../dev/iwa

KMOD    = if_iwa
SRCS    = if_iwa.c if_iwa_firmware.c if_iwa_pci.c \
	    device_if.h bus_if.h pci_if.h opt_iwn.h opt_wlan.h

.PATH:  ${.CURDIR}/../../dev/iwa/iwl/
SRCS	+= iwl-7000.c iwl-8000.c

.include <bsd.kmod.mk>
