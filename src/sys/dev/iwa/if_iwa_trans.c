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

#include <dev/iwa/if_iwa_debug.h>

#include <dev/iwa/drv-compat.h>

#include <dev/iwa/iwl/iwl-config.h>
#include <dev/iwa/iwl/iwl-csr.h>
#include <dev/iwa/iwl/iwl-fh.h>
#include <dev/iwa/iwl/iwl-trans.h>
#include <dev/iwa/iwl/iwl-prph.h>

/*
 * XXX wtf - anything referencing this isn't supposed
 * to be in the PCIe transaction layer!
 */
#include <dev/iwa/iwl/iwl-fw.h>	/* XXX */
#include <dev/iwa/iwl/mvm/fw-api.h>
#include <dev/iwa/iwl/mvm/fw-api-tx.h>

#include <dev/iwa/if_iwa_firmware.h>
#include <dev/iwa/if_iwa_trans.h>
#include <dev/iwa/if_iwavar.h>
#include <dev/iwa/if_iwareg.h>


/*
 * basic device access - pcie
 *
 * TODO: implement the iwlwifi 'trans' layer so we can
 * start using more of the code from Linux!
 */

/*
 * This code seems to straddle both the low level access
 * and knowledge about how big things are like the commands.
 * In the linux driver code the iwl_tx_cmd def is in both
 * mvm/ and dvm/ - the 7260 NIC is mvm/, but the code in pcie/tx.c
 * that knows about tx commands .. uses the dvm version? I dunno.
 * It's all a bit of a mess.
 */

static void iwa_dma_contig_free(struct iwa_dma_info *dma);

static uint32_t
iwa_read_prph(struct iwa_softc *sc, uint32_t addr)
{

	IWA_REG_WRITE(sc,
	    HBUS_TARG_PRPH_RADDR, ((addr & 0x000fffff) | (3 << 24)));
	IWA_REG_BARRIER_READ_WRITE(sc);
	return (IWA_REG_READ(sc, HBUS_TARG_PRPH_RDAT));
}

static void
iwa_write_prph(struct iwa_softc *sc, uint32_t addr, uint32_t val)
{

	IWA_REG_WRITE(sc,
	    HBUS_TARG_PRPH_WADDR, ((addr & 0x000fffff) | (3 << 24)));
	IWA_REG_BARRIER_WRITE(sc);
	IWA_REG_WRITE(sc, HBUS_TARG_PRPH_WDAT, val);
}

/* iwlwifi: pcie/trans.c */
static int
iwa_read_mem(struct iwa_softc *sc, uint32_t addr, void *buf, int dwords)
{
	int offs, ret = 0;
	uint32_t *vals = buf;

	if (iwa_grab_nic_access(sc)) {
		IWA_REG_WRITE(sc, HBUS_TARG_MEM_RADDR, addr);
		for (offs = 0; offs < dwords; offs++)
			vals[offs] = IWA_REG_READ(sc, HBUS_TARG_MEM_RDAT);
		iwa_release_nic_access(sc);
	} else {
		ret = EBUSY;
	}
	return ret;
}

#if 0
/* debugging */
static uint32_t
iwa_read_mem32(struct iwa_softc *sc, uint32_t addr)
{
	uint32_t rv;

	iwa_read_mem(sc, addr, &rv, 1);
	return rv;
}
#endif

/* iwlwifi: pcie/trans.c */
static int
iwa_write_mem(struct iwa_softc *sc, uint32_t addr, const void *buf, int dwords)
{
	int offs, ret = 0;
	const uint32_t *vals = buf;

	if (iwa_grab_nic_access(sc)) {
		IWA_REG_WRITE(sc, HBUS_TARG_MEM_WADDR, addr);
		/* WADDR auto-increments */
		for (offs = 0; offs < dwords; offs++) {
			uint32_t val = vals ? vals[offs] : 0;
			IWA_REG_WRITE(sc, HBUS_TARG_MEM_WDAT, val);
		}
		iwa_release_nic_access(sc);
	} else {
		/* let's just say that it's good to notice this failure */
		device_printf(sc->sc_dev, "WARNING: write_mem failed\n");
		ret = EBUSY;
	}
	return ret;
}

static int
iwa_write_mem32(struct iwa_softc *sc, uint32_t addr, uint32_t val)
{

	return iwa_write_mem(sc, addr, &val, 1);
}

/*
 * Convenience routines for bit and burger flipping
 */

static void
iwa_set_bits_mask(struct iwa_softc *sc,
	int reg, uint32_t mask, uint32_t bits)
{
	uint32_t val;

	val = IWA_REG_READ(sc, reg);
	val &= ~mask;
	val |= bits;
	IWA_REG_WRITE(sc, reg, val);
}

static void
iwa_set_bit(struct iwa_softc *sc, int reg, uint32_t bit)
{

	iwa_set_bits_mask(sc, reg, bit, bit);
}

static void
iwa_clear_bit(struct iwa_softc *sc, int reg, uint32_t bit)
{

	iwa_set_bits_mask(sc, reg, bit, 0);
}

static bool
iwa_poll_bit(struct iwa_softc *sc, int reg,
	uint32_t bits, uint32_t mask, int timo)
{

	for (;;) {
		if ((IWA_REG_READ(sc, reg) & mask) == (bits & mask)) {
			return true;
		}
		if (timo < 10) {
			return false;
		}
		timo -= 10;
		DELAY(10);
	}
}

bool
iwa_grab_nic_access(struct iwa_softc *sc)
{
	bool rv = false;

	iwa_set_bit(sc, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

	if (iwa_poll_bit(sc, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_VAL_MAC_ACCESS_EN,
	    CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY
	     | CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP, 15000)) {
	    	rv = true;
	} else {
		/* jolt */
		IWA_REG_WRITE(sc, CSR_RESET, CSR_RESET_REG_FLAG_FORCE_NMI);
	}

	return rv;
}

void
iwa_release_nic_access(struct iwa_softc *sc)
{

	iwa_clear_bit(sc, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
}

/*
 * whatta beep?  does the linux driver really have different semantics for
 * this bitsetting operation over iwl_set_bits()???
 */
static void
iwa_set_bits_mask_prph(struct iwa_softc *sc,
	uint32_t reg, uint32_t bits, uint32_t mask)
{
	uint32_t val;

	/* XXX: no error path? */
	if (iwa_grab_nic_access(sc)) {
		val = iwa_read_prph(sc, reg) & mask;
		val |= bits;
		iwa_write_prph(sc, reg, val);
		iwa_release_nic_access(sc);
	}
}

static void
iwa_set_bits_prph(struct iwa_softc *sc, uint32_t reg, uint32_t bits)
{

	iwa_set_bits_mask_prph(sc, reg, bits, ~0);
}

static void
iwa_clear_bits_prph(struct iwa_softc *sc, uint32_t reg, uint32_t bits)
{

	iwa_set_bits_mask_prph(sc, reg, 0, ~bits);
}

/*
 * DMA resource routines
 */

static void
iwa_dma_map_addr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (error != 0)
		return;
	KASSERT(nsegs == 1, ("too many DMA segments, %d should be 1", nsegs));
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

static int
iwa_dma_contig_alloc(bus_dma_tag_t tag, struct iwa_dma_info *dma,
    bus_size_t size, bus_size_t alignment)
{
	int error;

	dma->tag = NULL;
	dma->size = size;

	error = bus_dma_tag_create(tag,
	    alignment,
	    0,
	    BUS_SPACE_MAXADDR_32BIT,
	    BUS_SPACE_MAXADDR,
	    NULL, NULL,
	    size,
            1,
	    size,
	    BUS_DMA_NOWAIT,
	    NULL, NULL,
	    &dma->tag);
	if (error != 0)
		goto fail;

	error = bus_dmamem_alloc(dma->tag, (void **)&dma->vaddr,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT, &dma->map);
	if (error != 0)
		goto fail;

	error = bus_dmamap_load(dma->tag, dma->map, dma->vaddr, size,
	    iwa_dma_map_addr, &dma->paddr, BUS_DMA_NOWAIT);
	if (error != 0)
		goto fail;

	memset(dma->vaddr, 0, size);
	bus_dmamap_sync(dma->tag, dma->map, BUS_DMASYNC_PREWRITE);

	return 0;

fail:   iwa_dma_contig_free(dma);
        return error;

}

static void
iwa_dma_contig_free(struct iwa_dma_info *dma)
{
	if (dma->vaddr != NULL) {
		bus_dmamap_sync(dma->tag, dma->map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(dma->tag, dma->map);
		bus_dmamem_free(dma->tag, dma->vaddr, dma->map);
		dma->vaddr = NULL;
	}
	if (dma->tag != NULL) {
		bus_dma_tag_destroy(dma->tag);
		dma->tag = NULL;
	}
}

/* fwmem is used to load firmware onto the card */
int
iwa_alloc_fwmem(struct iwa_softc *sc)
{

	/* Must be aligned on a 16-byte boundary. */
	return iwa_dma_contig_alloc(sc->sc_dmat, &sc->fw_dma,
	    sc->sc_fw_dmasegsz, 16);
}

void
iwa_free_fwmem(struct iwa_softc *sc)
{

	iwa_dma_contig_free(&sc->fw_dma);
}

/* tx scheduler rings.  not used? */
int
iwa_alloc_sched(struct iwa_softc *sc)
{
	int rv;
#define	N(a)    (sizeof(a)/sizeof(a[0]))

	/* TX scheduler rings must be aligned on a 1KB boundary. */
	rv = iwa_dma_contig_alloc(sc->sc_dmat, &sc->sched_dma,
	   N(sc->txq) * sizeof(struct iwlagn_scd_bc_tbl), 1024);
	return rv;
#undef N
}

void
iwa_free_sched(struct iwa_softc *sc)
{
	iwa_dma_contig_free(&sc->sched_dma);
}

/* keep-warm page is used internally by the card.  see iwl-fh.h for more info */
int
iwa_alloc_kw(struct iwa_softc *sc)
{

	return iwa_dma_contig_alloc(sc->sc_dmat, &sc->kw_dma, 4096, 4096);
}

void
iwa_free_kw(struct iwa_softc *sc)
{

	iwa_dma_contig_free(&sc->kw_dma);
}

/* interrupt cause table */
int
iwa_alloc_ict(struct iwa_softc *sc)
{

	return iwa_dma_contig_alloc(sc->sc_dmat, &sc->ict_dma,
	    IWM_ICT_SIZE, 1<<IWM_ICT_PADDR_SHIFT);
}

void
iwa_free_ict(struct iwa_softc *sc)
{
	iwa_dma_contig_free(&sc->ict_dma);
}


/*
 * Allocate an RX buffer for the given RX ring slot.
 *
 * Returns 0 if it's done and OK, non-zero on error.
 *
 * XXX TODO: unmapping/freeing if there's already a slot?
 */
int
iwa_rx_addbuf(struct iwa_softc *sc, struct iwa_rx_ring *ring,
    size_t mbuf_size, int idx)
{
	struct mbuf *m = NULL;
	bus_addr_t paddr;
	int error;

	m = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, mbuf_size);
	if (m == NULL) {
		device_printf(sc->sc_dev,
		    "%s: could not allocate RX mbuf\n", __func__);
		ring->data[idx].m = NULL;
		error = ENOBUFS;
		goto fail;
	}

	error = bus_dmamap_load(ring->data_dmat, ring->data[idx].map,
	    mtod(m, void *), IWA_RBUF_SIZE, iwa_dma_map_addr,
	    &paddr, BUS_DMA_NOWAIT);
	if (error != 0 && error != EFBIG) {
		device_printf(sc->sc_dev,
		    "%s: can't not map mbuf, error %d\n", __func__,
		    error);
		goto fail;
	}

	/* Set mbuf */
	ring->data[idx].m = m;
	/* Set physical address of RX buffer (256-byte aligned). */
	ring->desc[idx] = htole32(paddr >> 8);

	return (0);
fail:
	if (m)
		m_freem(m);
	return (error);
}

/* and finally, the rx/tx ring alloc/reset/free routines */
int
iwa_alloc_rx_ring(struct iwa_softc *sc, struct iwa_rx_ring *ring)
{
	bus_size_t size;
	int i, error;

	ring->cur = 0;

	/* Allocate RX descriptors (256-byte aligned). */
	size = IWA_RX_RING_COUNT * sizeof(uint32_t);
	error = iwa_dma_contig_alloc(sc->sc_dmat, &ring->desc_dma, size, 256);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate RX ring DMA memory\n");
		goto fail;
	}
	ring->desc = (void *) ring->desc_dma.vaddr;

	/* Allocate RX status area (16-byte aligned). */
	error = iwa_dma_contig_alloc(sc->sc_dmat, &ring->stat_dma,
	    sizeof(*ring->stat), 16);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate RX status DMA memory\n");
		goto fail;
	}
	ring->stat = (void *) ring->stat_dma.vaddr;

        /* Create RX buffer DMA tag. */
        error = bus_dma_tag_create(sc->sc_dmat, 1, 0,
            BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
            IWA_RBUF_SIZE, 1, IWA_RBUF_SIZE, BUS_DMA_NOWAIT, NULL, NULL,
            &ring->data_dmat);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "%s: could not create RX buf DMA tag, error %d\n",
                    __func__, error);
                goto fail;
        }

	/*
	 * Allocate and map RX buffers.
	 */
	for (i = 0; i < IWA_RX_RING_COUNT; i++) {
		struct iwa_rx_data *data = &ring->data[i];

		memset(data, 0, sizeof(*data));

		error = bus_dmamap_create(ring->data_dmat, 0, &data->map);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not create RX buf DMA map, error %d\n",
			    __func__,
			    error);
			goto fail;
		}

		if ((error = iwa_rx_addbuf(sc, ring, IWA_RBUF_SIZE, i)) != 0) {
			device_printf(sc->sc_dev,
			    "could not add mbuf to ring");
			goto fail;
		}
	}
	return 0;

fail:	iwa_free_rx_ring(sc, ring);
	return error;
}

void
iwa_reset_rx_ring(struct iwa_softc *sc, struct iwa_rx_ring *ring)
{
	int ntries;

	if (iwa_grab_nic_access(sc)) {
		IWA_REG_WRITE(sc, FH_MEM_RCSR_CHNL0_CONFIG_REG, 0);
		for (ntries = 0; ntries < 1000; ntries++) {
			if (IWA_REG_READ(sc, FH_MEM_RSSR_RX_STATUS_REG) &
			    FH_RSSR_CHNL0_RX_STATUS_CHNL_IDLE)
				break;
			DELAY(10);
		}
		if (ntries == 1000) {
			device_printf(sc->sc_dev,
			    "unable to detect idle rx chan after reset\n");
		}
		iwa_release_nic_access(sc);
	}
	ring->cur = 0;
}

void
iwa_free_rx_ring(struct iwa_softc *sc, struct iwa_rx_ring *ring)
{
	int i;

	iwa_dma_contig_free(&ring->desc_dma);
	iwa_dma_contig_free(&ring->stat_dma);

	for (i = 0; i < IWA_RX_RING_COUNT; i++) {
		struct iwa_rx_data *data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(sc->sc_dmat, data->map,
			    BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(sc->sc_dmat, data->map);
			m_freem(data->m);
		}
		if (data->map != NULL)
			bus_dmamap_destroy(sc->sc_dmat, data->map);
	}
	bus_dma_tag_destroy(ring->data_dmat);
}

int
iwa_alloc_tx_ring(struct iwa_softc *sc, struct iwa_tx_ring *ring, int qid)
{
	bus_addr_t paddr;
	bus_size_t size;
	int i, error;

	ring->qid = qid;
	ring->queued = 0;
	ring->cur = 0;

	/* Allocate TX descriptors (256-byte aligned). */
	size = IWA_TX_RING_COUNT * sizeof (struct iwl_tfd);
	error = iwa_dma_contig_alloc(sc->sc_dmat, &ring->desc_dma, size, 256);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate TX ring DMA memory\n");
		goto fail;
	}
	ring->desc = (void *) ring->desc_dma.vaddr;

#if 0
	/*
	 * We only use rings 0 through 9 (4 EDCA + cmd) so there is no need
	 * to allocate commands space for other rings.
	 */
	if (qid > IWL_MVM_CMD_QUEUE)
		return 0;
#endif

	size = IWA_TX_RING_COUNT * sizeof(struct iwl_device_cmd);
	error = iwa_dma_contig_alloc(sc->sc_dmat, &ring->cmd_dma, size, 4);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate TX cmd DMA memory\n");
		goto fail;
	}
	ring->cmd = (void *) ring->cmd_dma.vaddr;

	/* Allocate tag for the TX ring */
	/* XXX why are we limiting maxsegs to num(bufs) - 1 here? */
	error = bus_dma_tag_create(sc->sc_dmat, 1, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL, MCLBYTES,
	    IWL_NUM_OF_TBS - 1, MCLBYTES, BUS_DMA_NOWAIT, NULL, NULL,
	    &ring->data_dmat);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not create TX buf DMA tag, error %d\n",
		    __func__, error);
		goto fail;
	}

	paddr = ring->cmd_dma.paddr;
	for (i = 0; i < IWA_TX_RING_COUNT; i++) {
		struct iwa_tx_data *data = &ring->data[i];

		data->cmd_paddr = paddr;
		data->scratch_paddr = paddr + sizeof(struct iwl_cmd_header)
		    + offsetof(struct iwl_tx_cmd, scratch);
		paddr += sizeof(struct iwl_device_cmd);

		error = bus_dmamap_create(ring->data_dmat, 0, &data->map);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "could not create TX buf DMA map\n");
			goto fail;
		}
	}
	KASSERT(paddr == ring->cmd_dma.paddr + size, (""));
	return 0;

fail:
	iwa_free_tx_ring(sc, ring);
	return error;
}

void
iwa_reset_tx_ring(struct iwa_softc *sc, struct iwa_tx_ring *ring)
{
	int i;

	for (i = 0; i < IWA_TX_RING_COUNT; i++) {
		struct iwa_tx_data *data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(sc->sc_dmat, data->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->sc_dmat, data->map);
			m_freem(data->m);
			data->m = NULL;
		}
	}
	/* Clear TX descriptors. */
	memset(ring->desc, 0, ring->desc_dma.size);
	bus_dmamap_sync(sc->sc_dmat, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);
	sc->qfullmsk &= ~(1 << ring->qid);
	ring->queued = 0;
	ring->cur = 0;
}

void
iwa_free_tx_ring(struct iwa_softc *sc, struct iwa_tx_ring *ring)
{
	int i;

	iwa_dma_contig_free(&ring->desc_dma);
	iwa_dma_contig_free(&ring->cmd_dma);

	for (i = 0; i < IWA_TX_RING_COUNT; i++) {
		struct iwa_tx_data *data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(sc->sc_dmat, data->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->sc_dmat, data->map);
			m_freem(data->m);
		}
		if (data->map != NULL)
			bus_dmamap_destroy(sc->sc_dmat, data->map);
	}
	bus_dma_tag_destroy(ring->data_dmat);
}

/*
 * High-level hardware frobbing routines
 */

static void
iwa_enable_rfkill_int(struct iwa_softc *sc)
{

	sc->sc_intmask = CSR_INT_BIT_RF_KILL;
	IWA_REG_WRITE(sc, CSR_INT_MASK, sc->sc_intmask);
}

bool
iwa_check_rfkill(struct iwa_softc *sc)
{
	uint32_t v;
	bool rv;

	/* XXX lock? */

	/*
	 * "documentation" is not really helpful here:
	 *  27:	HW_RF_KILL_SW
	 *	Indicates state of (platform's) hardware RF-Kill switch
	 *
	 * But apparently when it's off, it's on ...
	 */
	v = IWA_REG_READ(sc, CSR_GP_CNTRL);
	rv = (v & CSR_GP_CNTRL_REG_FLAG_HW_RF_KILL_SW) == 0;
	if (rv) {
		sc->sc_flags |= IWM_FLAG_RFKILL;
	} else {
		sc->sc_flags &= ~IWM_FLAG_RFKILL;
	}

	return rv;
}

/* all ints */

void
iwa_enable_interrupts(struct iwa_softc *sc)
{

	sc->sc_intmask = CSR_INI_SET_MASK;
	IWA_REG_WRITE(sc, CSR_INT_MASK, sc->sc_intmask);
}

void
iwa_restore_interrupts(struct iwa_softc *sc)
{

	IWA_REG_WRITE(sc, CSR_INT_MASK, sc->sc_intmask);
}

void
iwa_disable_interrupts(struct iwa_softc *sc)
{
	/* XXX lock? */

	/* disable interrupts */
	IWA_REG_WRITE(sc, CSR_INT_MASK, 0);

	/* acknowledge all interrupts */
	IWA_REG_WRITE(sc, CSR_INT, ~0);
	IWA_REG_WRITE(sc, CSR_FH_INT_STATUS, ~0);
}

static void
iwa_ict_reset(struct iwa_softc *sc)
{

	/* XXX lock */

	iwa_disable_interrupts(sc);

	/* Reset ICT table. */
	memset(sc->ict_dma.vaddr, 0, IWM_ICT_SIZE);
	sc->ict_cur = 0;

	/* Set physical address of ICT table (4KB aligned). */
	IWA_REG_WRITE(sc, CSR_DRAM_INT_TBL_REG,
	    CSR_DRAM_INT_TBL_ENABLE
	    | CSR_DRAM_INIT_TBL_WRAP_CHECK
	    | sc->ict_dma.paddr >> IWM_ICT_PADDR_SHIFT);

	/* Switch to ICT interrupt mode in driver. */
	sc->sc_flags |= IWM_FLAG_USE_ICT;

	/* Re-enable interrupts. */
	IWA_REG_WRITE(sc, CSR_INT, ~0);
	iwa_enable_interrupts(sc);
}

/* misc */

#define HW_READY_TIMEOUT 50
static bool
iwa_set_hw_ready(struct iwa_softc *sc)
{

	/* XXX lock */

	iwa_set_bit(sc, CSR_HW_IF_CONFIG_REG,
	    CSR_HW_IF_CONFIG_REG_BIT_NIC_READY);

        return iwa_poll_bit(sc, CSR_HW_IF_CONFIG_REG,
	    CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
	    CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
	    HW_READY_TIMEOUT);
}
#undef HW_READY_TIMEOUT

int
iwa_prepare_card_hw(struct iwa_softc *sc)
{
	int rv = 0;
	int t = 0;

	if (!iwa_set_hw_ready(sc))
		goto out;

	/* If HW is not ready, prepare the conditions to check again */
	iwa_set_bit(sc, CSR_HW_IF_CONFIG_REG, CSR_HW_IF_CONFIG_REG_PREPARE);

	do {
		if (iwa_set_hw_ready(sc))
			goto out;
		DELAY(200);
		t += 200;
	} while (t < 150000);

	rv = ETIMEDOUT;

 out:
	return rv;
}

#if 0
#ifndef PCI_PCIE_LCSR
#define PCI_PCIE_LCSR PCIE_LCSR
#define PCI_PCIE_LCSR_ASPM_L1 PCIE_LCSR_ASPM_L1
#endif
#endif

/* XXX TODO: put the pcie capability stuff in freebsd? */
#define PCI_PCIE_LCSR           0x10
#define PCI_PCIE_LCSR_ASPM_L0S  0x00000001
#define PCI_PCIE_LCSR_ASPM_L1   0x00000002

static void
iwa_apm_config(struct iwa_softc *sc)
{
	uint32_t reg;

	/* XXX 32 bit register, right? */
	reg = pci_read_config(sc->sc_dev,
	    sc->sc_cap_off + PCI_PCIE_LCSR,
	    4);
	if (reg & PCI_PCIE_LCSR_ASPM_L1) {
		/* Um the Linux driver prints "Disabling L0S for this one ... */
		iwa_set_bit(sc, CSR_GIO_REG, CSR_GIO_REG_VAL_L0S_ENABLED);
	} else {
		/* ... and "Enabling" here */
		iwa_clear_bit(sc, CSR_GIO_REG, CSR_GIO_REG_VAL_L0S_ENABLED);
	}
}

/*
 * Start up NIC's basic functionality after it has been reset
 * (e.g. after platform boot, or shutdown via iwl_pcie_apm_stop())
 * NOTE:  This does not load uCode nor start the embedded processor
 */
static int
iwa_apm_init(struct iwa_softc *sc)
{
	int error = 0;

	IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "%s: iwa apm start\n", __func__);

	/* Disable L0S exit timer (platform NMI Work/Around) */
	iwa_set_bit(sc, CSR_GIO_CHICKEN_BITS,
	    CSR_GIO_CHICKEN_BITS_REG_BIT_DIS_L0S_EXIT_TIMER);

	/*
	 * Disable L0s without affecting L1;
	 *  don't wait for ICH L0s (ICH bug W/A)
	 */
	iwa_set_bit(sc, CSR_GIO_CHICKEN_BITS,
	    CSR_GIO_CHICKEN_BITS_REG_BIT_L1A_NO_L0S_RX);

	/* Set FH wait threshold to maximum (HW error during stress W/A) */
	iwa_set_bit(sc, CSR_DBG_HPET_MEM_REG, CSR_DBG_HPET_MEM_REG_VAL);

	/*
	 * Enable HAP INTA (interrupt from management bus) to
	 * wake device's PCI Express link L1a -> L0s
	 */
	iwa_set_bit(sc, CSR_HW_IF_CONFIG_REG,
	    CSR_HW_IF_CONFIG_REG_BIT_HAP_WAKE_L1A);

	iwa_apm_config(sc);

#if 0 /* not for 7k */
	/* Configure analog phase-lock-loop before activating to D0A */
	if (trans->cfg->base_params->pll_cfg_val)
		iwl_set_bit(trans, CSR_ANA_PLL_CFG,
		    trans->cfg->base_params->pll_cfg_val);
#endif

	/*
	 * Set "initialization complete" bit to move adapter from
	 * D0U* --> D0A* (powered-up active) state.
	 */
	iwa_set_bit(sc, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	/*
	 * Wait for clock stabilization; once stabilized, access to
	 * device-internal resources is supported, e.g. iwl_write_prph()
	 * and accesses to uCode SRAM.
	 */
	if (!iwa_poll_bit(sc, CSR_GP_CNTRL,
	    CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
	    CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY, 25000)) {
		device_printf(sc->sc_dev, "Failed to init the card\n");
		goto out;
	}

	/*
	 * This is a bit of an abuse - This is needed for 7260 / 3160
	 * only check host_interrupt_operation_mode even if this is
	 * not related to host_interrupt_operation_mode.
	 *
	 * Enable the oscillator to count wake up time for L1 exit. This
	 * consumes slightly more power (100uA) - but allows to be sure
	 * that we wake up from L1 on time.
	 *
	 * This looks weird: read twice the same register, discard the
	 * value, set a bit, and yet again, read that same register
	 * just to discard the value. But that's the way the hardware
	 * seems to like it.
	 */
	iwa_read_prph(sc, OSC_CLK);
	iwa_read_prph(sc, OSC_CLK);
	iwa_set_bits_prph(sc, OSC_CLK, OSC_CLK_FORCE_CONTROL);
	iwa_read_prph(sc, OSC_CLK);
	iwa_read_prph(sc, OSC_CLK);

	/*
	 * Enable DMA clock and wait for it to stabilize.
	 *
	 * Write to "CLK_EN_REG"; "1" bits enable clocks, while "0" bits
	 * do not disable clocks.  This preserves any hardware bits already
	 * set by default in "CLK_CTRL_REG" after reset.
	 */
	iwa_write_prph(sc, APMG_CLK_EN_REG, APMG_CLK_VAL_DMA_CLK_RQT);
	//kpause("iwaapm", false, mstohz(20), NULL);
	DELAY(20);

	/* Disable L1-Active */
	iwa_set_bits_prph(sc, APMG_PCIDEV_STT_REG,
	    APMG_PCIDEV_STT_VAL_L1_ACT_DIS);

	/* Clear the interrupt in APMG if the NIC is in RFKILL */
	iwa_write_prph(sc, APMG_RTC_INT_STT_REG, APMG_RTC_INT_STT_RFKILL);

 out:
	if (error)
		device_printf(sc->sc_dev, "apm init error %d\n", error);
	return error;
}

/* iwlwifi/pcie/trans.c */
static void
iwa_apm_stop(struct iwa_softc *sc)
{

	/* stop device's busmaster DMA activity */
	iwa_set_bit(sc, CSR_RESET, CSR_RESET_REG_FLAG_STOP_MASTER);

	if (!iwa_poll_bit(sc, CSR_RESET,
	    CSR_RESET_REG_FLAG_MASTER_DISABLED,
	    CSR_RESET_REG_FLAG_MASTER_DISABLED, 100))
		device_printf(sc->sc_dev,
		    "Master Disable Timed Out, 100 usec\n");
	IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "%s: iwa apm stop\n", __func__);
}

/* iwlwifi pcie/trans.c */
int
iwa_start_hw(struct iwa_softc *sc)
{
	int error;

	if ((error = iwa_prepare_card_hw(sc)) != 0)
		return error;

        /* Reset the entire device */
	IWA_REG_WRITE(sc, CSR_RESET,
	    CSR_RESET_REG_FLAG_SW_RESET | CSR_RESET_REG_FLAG_NEVO_RESET);
	DELAY(10);

	if ((error = iwa_apm_init(sc)) != 0)
		return error;

	iwa_enable_rfkill_int(sc);
	iwa_check_rfkill(sc);

	return 0;
}

/* iwlwifi pcie/trans.c */
/* philosophy: why is it "start hw" but "stop device"? */

void
iwa_stop_device(struct iwa_softc *sc)
{
	int chnl, ntries;
	int qid;

	/* tell the device to stop sending interrupts */
	iwa_disable_interrupts(sc);

	/* device going down, Stop using ICT table */
	sc->sc_flags &= ~IWM_FLAG_USE_ICT;

	/* stop tx and rx.  tx and rx bits, as usual, are from if_iwn */

	iwa_write_prph(sc, SCD_TXFACT, 0);

	/* Stop all DMA channels. */
	if (iwa_grab_nic_access(sc)) {
		for (chnl = 0; chnl < FH_TCSR_CHNL_NUM; chnl++) {
			IWA_REG_WRITE(sc,
			    FH_TCSR_CHNL_TX_CONFIG_REG(chnl), 0);
			for (ntries = 0; ntries < 200; ntries++) {
				uint32_t r;

				r = IWA_REG_READ(sc, FH_TSSR_TX_STATUS_REG);
				if (r & FH_TSSR_TX_STATUS_REG_MSK_CHNL_IDLE(chnl))
					break;
				DELAY(20);
			}
			if (ntries == 200) {
				device_printf(sc->sc_dev,
				    "unable to detect idle tx "
				    "chan after reset\n");
			}
		}
		iwa_release_nic_access(sc);
	}

	/* Stop RX ring. */
	iwa_reset_rx_ring(sc, &sc->rxq);

	/* Reset all TX rings. */
#define	N(a)    (sizeof(a)/sizeof(a[0]))
	for (qid = 0; qid < N(sc->txq); qid++)
		iwa_reset_tx_ring(sc, &sc->txq[qid]);
#undef N
	/*
	 * Power-down device's busmaster DMA clocks
	 */
	iwa_write_prph(sc, APMG_CLK_DIS_REG, APMG_CLK_VAL_DMA_CLK_RQT);
	DELAY(5);

	/* Make sure (redundant) we've released our request to stay awake */
	iwa_clear_bit(sc, CSR_GP_CNTRL,
	    CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

	/* Stop the device, and put it in low power state */
	iwa_apm_stop(sc);

        /* Upon stop, the APM issues an interrupt if HW RF kill is set.
         * Clean again the interrupt here
         */
	iwa_disable_interrupts(sc);
	/* stop and reset the on-board processor */
	IWA_REG_WRITE(sc, CSR_RESET, CSR_RESET_REG_FLAG_NEVO_RESET);

	/*
	 * Even if we stop the HW, we still want the RF kill
	 * interrupt
	 */
	iwa_enable_rfkill_int(sc);
	iwa_check_rfkill(sc);
}

/* iwlwifi pcie/trans.c (always main power) */
void
iwa_set_pwr(struct iwa_softc *sc)
{

	iwa_set_bits_mask_prph(sc, APMG_PS_CTRL_REG,
	    APMG_PS_CTRL_VAL_PWR_SRC_VMAIN, ~APMG_PS_CTRL_MSK_PWR_SRC);
}

/* iwlwifi: mvm/ops.c */
/*
 * XXX shouldn't be in this file; this doesn't really
 * have anything to do with the PCIe layer.
 */
static void
iwa_mvm_nic_config(struct iwa_softc *sc)
{
	uint8_t radio_cfg_type, radio_cfg_step, radio_cfg_dash;
	uint32_t reg_val = 0;

	IWA_LOCK_ASSERT(sc);

	radio_cfg_type = (sc->sc_fw_phy_config & FW_PHY_CFG_RADIO_TYPE) >>
	    FW_PHY_CFG_RADIO_TYPE_POS;
	radio_cfg_step = (sc->sc_fw_phy_config & FW_PHY_CFG_RADIO_STEP) >>
	    FW_PHY_CFG_RADIO_STEP_POS;
	radio_cfg_dash = (sc->sc_fw_phy_config & FW_PHY_CFG_RADIO_DASH) >>
	    FW_PHY_CFG_RADIO_DASH_POS;

	/* SKU control */
	reg_val |= CSR_HW_REV_STEP(sc->sc_hw_rev) <<
	    CSR_HW_IF_CONFIG_REG_POS_MAC_STEP;
	reg_val |= CSR_HW_REV_DASH(sc->sc_hw_rev) <<
	    CSR_HW_IF_CONFIG_REG_POS_MAC_DASH;

	/* radio configuration */
	reg_val |= radio_cfg_type << CSR_HW_IF_CONFIG_REG_POS_PHY_TYPE;
	reg_val |= radio_cfg_step << CSR_HW_IF_CONFIG_REG_POS_PHY_STEP;
	reg_val |= radio_cfg_dash << CSR_HW_IF_CONFIG_REG_POS_PHY_DASH;

	/* silicon bits */
	reg_val |= CSR_HW_IF_CONFIG_REG_BIT_RADIO_SI;

	iwa_set_bits_mask(sc, CSR_HW_IF_CONFIG_REG,
	    CSR_HW_IF_CONFIG_REG_MSK_MAC_DASH |
	    CSR_HW_IF_CONFIG_REG_MSK_MAC_STEP |
	    CSR_HW_IF_CONFIG_REG_MSK_PHY_TYPE |
	    CSR_HW_IF_CONFIG_REG_MSK_PHY_STEP |
	    CSR_HW_IF_CONFIG_REG_MSK_PHY_DASH |
	    CSR_HW_IF_CONFIG_REG_BIT_RADIO_SI |
	    CSR_HW_IF_CONFIG_REG_BIT_MAC_SI,
	    reg_val);

	device_printf(sc->sc_dev,
	    "Radio type=0x%x-0x%x-0x%x\n", radio_cfg_type,
	    radio_cfg_step, radio_cfg_dash);

	/*
	 * W/A : NIC is stuck in a reset state after Early PCIe power off
	 * (PCIe power is lost before PERST# is asserted), causing ME FW
	 * to lose ownership and not being able to obtain it back.
	 */
	iwa_set_bits_mask_prph(sc, APMG_PS_CTRL_REG,
	    APMG_PS_CTRL_EARLY_PWR_OFF_RESET_DIS,
	    ~APMG_PS_CTRL_EARLY_PWR_OFF_RESET_DIS);
}

static void
iwa_enable_txq(struct iwa_softc *sc, int qid, int fifo)
{

	IWA_LOCK_ASSERT(sc);

	if (!iwa_grab_nic_access(sc)) {
		device_printf(sc->sc_dev, "cannot enable txq %d\n", qid);
		return;
	}

	/* unactivate before configuration */
	iwa_write_prph(sc, SCD_QUEUE_STATUS_BITS(qid),
	    (0 << SCD_QUEUE_STTS_REG_POS_ACTIVE)
	    | (1 << SCD_QUEUE_STTS_REG_POS_SCD_ACT_EN));

	if (qid != IWL_MVM_CMD_QUEUE) {
		iwa_set_bits_prph(sc, SCD_QUEUECHAIN_SEL, BIT(qid));
	}

	iwa_clear_bits_prph(sc, SCD_AGGR_SEL, BIT(qid));

	IWA_REG_WRITE(sc, HBUS_TARG_WRPTR, qid << 8 | 0);
	iwa_write_prph(sc, SCD_QUEUE_RDPTR(qid), 0);

	iwa_write_mem32(sc, sc->sched_base + SCD_CONTEXT_QUEUE_OFFSET(qid), 0);
	/* Set scheduler window size and frame limit. */
	iwa_write_mem32(sc,
	    sc->sched_base + SCD_CONTEXT_QUEUE_OFFSET(qid) + sizeof(uint32_t),
		((IWL_FRAME_LIMIT << SCD_QUEUE_CTX_REG2_WIN_SIZE_POS) &
		  SCD_QUEUE_CTX_REG2_WIN_SIZE_MSK) |
		((IWL_FRAME_LIMIT << SCD_QUEUE_CTX_REG2_FRAME_LIMIT_POS) &
		  SCD_QUEUE_CTX_REG2_FRAME_LIMIT_MSK));

	iwa_write_prph(sc, SCD_QUEUE_STATUS_BITS(qid),
	    (1 << SCD_QUEUE_STTS_REG_POS_ACTIVE) |
	    (fifo << SCD_QUEUE_STTS_REG_POS_TXF) |
	    (1 << SCD_QUEUE_STTS_REG_POS_WSL) |
	    SCD_QUEUE_STTS_REG_MSK);

	iwa_release_nic_access(sc);

	device_printf(sc->sc_dev,
	    "enabled txq %d FIFO %d\n",
	    qid,
	    fifo);
}

int
iwa_nic_rx_init(struct iwa_softc *sc)
{

	IWA_LOCK_ASSERT(sc);

	if (!iwa_grab_nic_access(sc))
		return EBUSY;

	/*
	 * Initialize RX ring.  This is from the iwn driver.
	 */
	memset(sc->rxq.stat, 0, sizeof(*sc->rxq.stat));

	/* stop DMA */
	IWA_REG_WRITE(sc, FH_MEM_RCSR_CHNL0_CONFIG_REG, 0);
	IWA_REG_WRITE(sc, FH_MEM_RCSR_CHNL0_RBDCB_WPTR, 0);
	IWA_REG_WRITE(sc, FH_MEM_RCSR_CHNL0_FLUSH_RB_REQ, 0);
	IWA_REG_WRITE(sc, FH_RSCSR_CHNL0_RDPTR, 0);
	IWA_REG_WRITE(sc, FH_RSCSR_CHNL0_RBDCB_WPTR_REG, 0);

	/* Set physical address of RX ring (256-byte aligned). */
	IWA_REG_WRITE(sc,
	    FH_RSCSR_CHNL0_RBDCB_BASE_REG, sc->rxq.desc_dma.paddr >> 8);

	/* Set physical address of RX status (16-byte aligned). */
	IWA_REG_WRITE(sc,
	    FH_RSCSR_CHNL0_STTS_WPTR_REG, sc->rxq.stat_dma.paddr >> 4);

	/* Enable RX. */
	/*
	 * Note: Linux driver also sets this:
	 *  (RX_RB_TIMEOUT << FH_RCSR_RX_CONFIG_REG_IRQ_RBTH_POS) |
	 *
	 * It causes weird behavior.  YMMV.
	 */
	IWA_REG_WRITE(sc, FH_MEM_RCSR_CHNL0_CONFIG_REG,
	    FH_RCSR_RX_CONFIG_CHNL_EN_ENABLE_VAL		  |
	    FH_RCSR_CHNL0_RX_IGNORE_RXF_EMPTY			  |  /* HW bug */
	    FH_RCSR_CHNL0_RX_CONFIG_IRQ_DEST_INT_HOST_VAL	  |
	    FH_RCSR_RX_CONFIG_REG_VAL_RB_SIZE_4K		  |
	    RX_QUEUE_SIZE_LOG << FH_RCSR_RX_CONFIG_RBDCB_SIZE_POS);

	IWA_REG_WRITE_1(sc, CSR_INT_COALESCING, IWL_HOST_INT_TIMEOUT_DEF);
	iwa_set_bit(sc, CSR_INT_COALESCING, IWL_HOST_INT_OPER_MODE);

	/*
	 * Thus sayeth el jefe (iwlwifi) via a comment:
	 *
	 * This value should initially be 0 (before preparing any
	 * RBs), should be 8 after preparing the first 8 RBs (for example)
	 */
	IWA_REG_WRITE(sc, FH_RSCSR_CHNL0_WPTR, 8);

	iwa_release_nic_access(sc);

	return 0;
}

int
iwa_nic_tx_init(struct iwa_softc *sc)
{
	int qid;

	IWA_LOCK_ASSERT(sc);

	if (!iwa_grab_nic_access(sc))
		return EBUSY;

	/* Deactivate TX scheduler. */
	iwa_write_prph(sc, SCD_TXFACT, 0);

	/* Set physical address of "keep warm" page (16-byte aligned). */
	IWA_REG_WRITE(sc, FH_KW_MEM_ADDR_REG, sc->kw_dma.paddr >> 4);

	/* Initialize TX rings. */
#define	N(a)    (sizeof(a)/sizeof(a[0]))
	for (qid = 0; qid < N(sc->txq); qid++) {
		struct iwa_tx_ring *txq = &sc->txq[qid];

		/* Set physical address of TX ring (256-byte aligned). */
		IWA_REG_WRITE(sc, FH_MEM_CBBC_QUEUE(qid),
		    txq->desc_dma.paddr >> 8);
		IWA_DPRINTF(sc, IWA_DEBUG_RESET | IWA_DEBUG_TX,
		    "loading ring %d descriptors (%p) at %llx\n",
		    qid, txq->desc, (long long) (txq->desc_dma.paddr >> 8));
	}
	iwa_release_nic_access(sc);
#undef N

	return 0;
}

int
iwa_nic_init(struct iwa_softc *sc)
{
	int error;

	IWA_LOCK_ASSERT(sc);

	iwa_apm_init(sc);
	iwa_set_pwr(sc);

	iwa_mvm_nic_config(sc);

	if ((error = iwa_nic_rx_init(sc)) != 0)
		return error;

	/*
	 * Ditto for TX, from iwn
	 */
	if ((error = iwa_nic_tx_init(sc)) != 0)
		return error;

	device_printf(sc->sc_dev,
	    "shadow registers enabled\n");
	iwa_set_bit(sc, CSR_MAC_SHADOW_REG_CTRL, 0x800fffff);

        return 0;
}

int
iwa_post_alive(struct iwa_softc *sc)
{
	int nwords;
	int error, chnl;

	IWA_LOCK_ASSERT(sc);

	if (!iwa_grab_nic_access(sc))
		return EBUSY;

	if (sc->sched_base != iwa_read_prph(sc, SCD_SRAM_BASE_ADDR)) {
		device_printf(sc->sc_dev, "sched addr mismatch");
		error = EINVAL;
		goto out;
	}

	iwa_ict_reset(sc);

	/* Clear TX scheduler state in SRAM. */
	nwords = (SCD_TRANS_TBL_MEM_UPPER_BOUND - SCD_CONTEXT_MEM_LOWER_BOUND)
	    / sizeof(uint32_t);
	error = iwa_write_mem(sc, sc->sched_base + SCD_CONTEXT_MEM_LOWER_BOUND,
	    NULL, nwords);
	if (error)
		goto out;

	/* Set physical address of TX scheduler rings (1KB aligned). */
	iwa_write_prph(sc, SCD_DRAM_BASE_ADDR, sc->sched_dma.paddr >> 10);

	iwa_write_prph(sc, SCD_CHAINEXT_EN, 0);

	/* enable command channel */
	iwa_enable_txq(sc, IWL_MVM_CMD_QUEUE, 7);

	iwa_write_prph(sc, SCD_TXFACT, 0xff);

	/* Enable DMA channels. */
	for (chnl = 0; chnl < FH_TCSR_CHNL_NUM; chnl++) {
		IWA_REG_WRITE(sc, FH_TCSR_CHNL_TX_CONFIG_REG(chnl),
		    FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_ENABLE |
		    FH_TCSR_TX_CONFIG_REG_VAL_DMA_CREDIT_ENABLE);
	}

	iwa_set_bit(sc,
	    FH_TX_CHICKEN_BITS_REG, FH_TX_CHICKEN_BITS_SCD_AUTO_RETRY_EN);

        /* Enable L1-Active */
	iwa_clear_bits_prph(sc, APMG_PCIDEV_STT_REG,
	    APMG_PCIDEV_STT_VAL_L1_ACT_DIS);

out:
	iwa_release_nic_access(sc);
	return error;
}
