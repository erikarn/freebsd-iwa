# $FreeBSD$

.PATH:  ${.CURDIR}/../../dev/iwa

KMOD    = if_iwa
SRCS    = if_iwa.c if_iwa_firmware.c if_iwa_pci.c \
	    if_iwa_trans.c if_iwa_rx.c if_iwa_fw_util.c \
	    if_iwa_nvm.c

SRCS+=	device_if.h bus_if.h pci_if.h opt_iwn.h opt_wlan.h

CFLAGS+= -I${.CURDIR}/../../

.PATH:  ${.CURDIR}/../../dev/iwa/iwl/
SRCS	+= iwl-7000.c iwl-8000.c

.include <bsd.kmod.mk>
