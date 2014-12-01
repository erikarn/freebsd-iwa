/*-
 * Copyright (c) 2014 Adrian Chadd <adrian@FreeBSD.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Driver for Intel WiFi Link 7260 series 802.11n/802.11ac adapters.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_wlan.h"
//#include "opt_iwa.h"

#include <sys/param.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/endian.h>
#include <sys/firmware.h>
#include <sys/limits.h>
#include <sys/module.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/clock.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_ratectl.h>

#include <dev/iwa/drv-compat.h>

#include <dev/iwa/iwl/iwl-config.h>

#include <dev/iwa/if_iwa_debug.h>
#include <dev/iwa/if_iwa_firmware.h>
#include <dev/iwa/if_iwa_trans.h>
#include <dev/iwa/if_iwavar.h>



struct iwa_ident {
	uint16_t	vendor;
	uint16_t	device;
	uint16_t	sub_vendor;
	uint16_t	sub_device;
	const void	*cfg;
};

#define	IWL_PCI_DEVICE(d, sd, p) \
	.vendor = 0x8086, .device = (d), .sub_vendor = 0x8086, .sub_device = (sd), .cfg = &(p)

static const struct iwa_ident iwa_ident_table[] = {
	{IWL_PCI_DEVICE(0x08B1, 0x4070, iwl7260_2ac_cfg)},
	{IWL_PCI_DEVICE(0x08B1, 0x4072, iwl7260_2ac_cfg)},
	{IWL_PCI_DEVICE(0x08B1, 0x4170, iwl7260_2ac_cfg)},
	{IWL_PCI_DEVICE(0x08B1, 0x4060, iwl7260_2n_cfg)},
	{IWL_PCI_DEVICE(0x08B1, 0x406A, iwl7260_2n_cfg)},
	{IWL_PCI_DEVICE(0x08B1, 0x4160, iwl7260_2n_cfg)},
	{IWL_PCI_DEVICE(0x08B1, 0x4062, iwl7260_n_cfg)},
	{IWL_PCI_DEVICE(0x08B1, 0x4162, iwl7260_n_cfg)},

	{IWL_PCI_DEVICE(0x08B2, 0x4270, iwl7260_2ac_cfg)},
	{IWL_PCI_DEVICE(0x08B2, 0x4272, iwl7260_2ac_cfg)},
	{IWL_PCI_DEVICE(0x08B2, 0x4260, iwl7260_2n_cfg)},
	{IWL_PCI_DEVICE(0x08B2, 0x426A, iwl7260_2n_cfg)},
	{IWL_PCI_DEVICE(0x08B2, 0x4262, iwl7260_n_cfg)},

	{ 0, 0, 0, 0, NULL }
};

static int iwa_pci_detach(device_t dev);

static void
iwa_pci_intr(void *arg)
{
	int ret;
	struct iwa_softc *sc = arg;

	device_printf(sc->sc_dev, "%s: called\n", __func__);

	ret = iwa_intr(sc);
}

static int
iwa_pci_probe(device_t dev)
{
	struct iwa_softc *sc = (struct iwa_softc *)device_get_softc(dev);

	const struct iwa_ident *ident;

	for (ident = iwa_ident_table; ident->cfg != NULL; ident++) {
		if (pci_get_vendor(dev) == ident->vendor &&
		    pci_get_device(dev) == ident->device &&
		    pci_get_subvendor(dev) == ident->sub_vendor &&
		    pci_get_subdevice(dev) == ident->sub_device) {
			/* Assign the hardware type early during probe */
			sc->sc_cfg = ident->cfg;
			device_set_desc(dev, sc->sc_cfg->name);
			return (BUS_PROBE_DEFAULT);
		}
	}
	return (ENXIO);
}

/* XXX for now */
#define	IWA_MAX_SCATTER		1

static int
iwa_pci_attach(device_t dev)
{
	struct iwa_softc *sc = (struct iwa_softc *)device_get_softc(dev);
	int i, error, rid;

	sc->sc_dev = dev;

#ifdef	IWA_DEBUG
	error = resource_int_value(device_get_name(sc->sc_dev),
	    device_get_unit(sc->sc_dev), "debug", &(sc->sc_debug));
	if (error != 0)
		sc->sc_debug = 0;
#else
	sc->sc_debug = 0;
#endif

	IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "->%s: begin\n",__func__);

	sc->subdevice_id = pci_get_subdevice(dev);

	/*
	 * Get the offset of the PCI Express Capability Structure in PCI
	 * Configuration Space.
	 */
	error = pci_find_cap(dev, PCIY_EXPRESS, &sc->sc_cap_off);
	if (error != 0) {
		device_printf(dev, "PCIe capability structure not found!\n");
		return error;
	}

	/* Clear device-specific "PCI retry timeout" register (41h). */
	pci_write_config(dev, 0x41, 0, 1);

	/* Enable bus-mastering. */
	pci_enable_busmaster(dev);

	/* XXX from OpenBSD port: disable interrupts when enabling busmaster? */
#if 0
        reg |= PCI_COMMAND_MASTER_ENABLE;
        /* if !MSI */
        if (reg & PCI_COMMAND_INTERRUPT_DISABLE) {
                reg &= ~PCI_COMMAND_INTERRUPT_DISABLE;
        }
#endif

	rid = PCIR_BAR(0);
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "can't map mem space\n");
		error = ENOMEM;
		return error;
	}
	sc->sc_st = rman_get_bustag(sc->mem);
	sc->sc_sh = rman_get_bushandle(sc->mem);

	i = 1;
	rid = 0;
	if (pci_alloc_msi(dev, &i) == 0)
		rid = 1;

	/* Allocate interrupt resource */
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE |
	    (rid != 0 ? 0 : RF_SHAREABLE));
	if (sc->irq == NULL) {
		device_printf(dev, "can't map interrupt\n");
		error = ENOMEM;
		goto bad;
	}

	/* Setup interrupt handler */
	if (bus_setup_intr(dev, sc->irq,
	    INTR_TYPE_NET | INTR_MPSAFE,
	    NULL,
	    iwa_pci_intr,
	    sc,
	    &sc->sc_ih)) {
		device_printf(dev, "could not establish interrupt\n");
		error = ENXIO;
		goto bad;
	}

	/* Setup DMA descriptor area */

	/*
	 * This chip apparently supports 36-bit DMA; let's limit it
	 * to 32 bit DMA for now.
	 *
	 * Also, segments should be aligned at 8 bit boundaries;
	 * the descriptor rings take addresses that are >> 8.
	 */
	if (bus_dma_tag_create(bus_get_dma_tag(dev),    /* parent */
	    256,		/* alignment (8 bit) */
	    0,			/* bounds (none?) */
	    BUS_SPACE_MAXADDR_32BIT, /* lowaddr */
	    BUS_SPACE_MAXADDR,       /* highaddr */
	    NULL, NULL,              /* filter, filterarg */
	    0x3ffff,                 /* maxsize XXX */
	    IWA_MAX_SCATTER,         /* nsegments */
	    0x3ffff,                 /* maxsegsize XXX */
	    BUS_DMA_ALLOCNOW,        /* flags */
	    NULL,                    /* lockfunc */
	    NULL,                    /* lockarg */
	    &sc->sc_dmat)) {
		device_printf(dev, "cannot allocate DMA tag\n");
		error = ENXIO;
		goto bad;
	}

	IWA_LOCK_INIT(sc);

	if ((error = iwa_attach(sc)) == 0) {
		IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "->%s: end\n",__func__);
		return (0);
	}

	/* iwa_attach() failed */
	device_printf(sc->sc_dev, "%s: iwa_attach_failed\n", __func__);
	IWA_LOCK_DESTROY(sc);

	bus_dma_tag_destroy(sc->sc_dmat);

bad:
	if (sc->irq) {
		if (sc->sc_ih)
			bus_teardown_intr(dev, sc->irq, sc->sc_ih);
		bus_release_resource(dev,
		    SYS_RES_IRQ,
		    rman_get_rid(sc->irq),
		    sc->irq);
		pci_release_msi(dev);
	}
	if (sc->mem)
		bus_release_resource(dev,
		    SYS_RES_MEMORY,
		    rman_get_rid(sc->mem),
		    sc->mem);

	IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "->%s: end in error\n",__func__);
	return (error);
}

static int
iwa_pci_detach(device_t dev)
{
	struct iwa_softc *sc = device_get_softc(dev);

	(void) iwa_detach(sc);

	/* Uninstall interrupt handler. */
	if (sc->irq != NULL) {
		bus_teardown_intr(dev, sc->irq, sc->sc_ih);
		bus_release_resource(dev, SYS_RES_IRQ, rman_get_rid(sc->irq),
		    sc->irq);
		pci_release_msi(dev);
	}


	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem), sc->mem);

	IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "->%s: end\n", __func__);
	IWA_LOCK_DESTROY(sc);

	return (0);
}

static int
iwa_pci_shutdown(device_t dev)
{
	struct iwa_softc *sc = device_get_softc(dev);

	return (iwa_shutdown(sc));
}

static int
iwa_pci_suspend(device_t dev)
{
	struct iwa_softc *sc = device_get_softc(dev);
#if 0
	struct ieee80211com *ic = sc->sc_ifp->if_l2com;

#endif
	return (iwa_suspend(sc));
}

static int
iwa_pci_resume(device_t dev)
{
	struct iwa_softc *sc = device_get_softc(dev);
#if 0
	struct ieee80211com *ic = sc->sc_ifp->if_l2com;

	/* Clear device-specific "PCI retry timeout" register (41h). */
	pci_write_config(dev, 0x41, 0, 1);
#endif

	return (iwa_resume(sc));
}

static device_method_t iwa_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		iwa_pci_probe),
	DEVMETHOD(device_attach,	iwa_pci_attach),
	DEVMETHOD(device_detach,	iwa_pci_detach),
	DEVMETHOD(device_shutdown,	iwa_pci_shutdown),
	DEVMETHOD(device_suspend,	iwa_pci_suspend),
	DEVMETHOD(device_resume,	iwa_pci_resume),

	DEVMETHOD_END
};

static driver_t iwa_driver = {
	"iwa",
	iwa_methods,
	sizeof(struct iwa_softc)
};
static devclass_t iwa_devclass;

DRIVER_MODULE(iwa, pci, iwa_driver, iwa_devclass, NULL, NULL);

MODULE_VERSION(iwa, 1);

MODULE_DEPEND(iwa, firmware, 1, 1, 1);
MODULE_DEPEND(iwa, pci, 1, 1, 1);
MODULE_DEPEND(iwa, wlan, 1, 1, 1);
