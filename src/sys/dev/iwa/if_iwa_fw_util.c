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
#include <dev/iwa/iwl/iwl-fw.h>
#include <dev/iwa/iwl/iwl-fh.h>
#include <dev/iwa/iwl/iwl-trans.h>

#include <dev/iwa/iwl/mvm/fw-api.h>

#include <dev/iwa/if_iwa_firmware.h>
#include <dev/iwa/if_iwa_trans.h>
#include <dev/iwa/if_iwa_nvm.h>
#include <dev/iwa/if_iwavar.h>
#include <dev/iwa/if_iwareg.h>

#include <dev/iwa/if_iwa_fw_util.h>


/*
 * Send a command to the firmware.  We try to implement the Linux
 * driver interface for the routine.
 * mostly from if_iwn (iwn_cmd()).
 *
 * For now, we always copy the first part and map the second one (if it exists).
 *
 * This requires the IWA lock to be held.
 *
 * If the command requires a response (CMD_WANT_SKB is set); then
 * we wait until the command response comes through or the NIC is reset.
 */
int
iwa_send_cmd(struct iwa_softc *sc, struct iwl_host_cmd *hcmd)
{
	struct iwa_tx_ring *ring = &sc->txq[IWL_MVM_CMD_QUEUE];
	struct iwa_tx_ring_meta *meta = &sc->txq_meta[IWL_MVM_CMD_QUEUE];
	struct iwl_tfd *desc;
	struct iwa_tx_data *data;
	struct iwl_device_cmd *cmd;
	struct mbuf *m;
	bus_addr_t paddr;
	uint32_t addr_lo;
	int error = 0, i, paylen, off;
	int code;
	bool async, wantresp;
	int ring_idx;

	code = hcmd->id;
	async = hcmd->flags & CMD_ASYNC;
	wantresp = hcmd->flags & CMD_WANT_SKB;

	IWA_LOCK_ASSERT(sc);

#define	N(a)	(sizeof(a)/sizeof(a[0]))
	for (i = 0, paylen = 0; i < N(hcmd->len); i++) {
		paylen += hcmd->len[i];
	}
#undef	N

	/*
	 * Unlike openbsd, we're going to sleep on the individual
	 * command slot entries.
	 */
#if 0
	/* if the command wants an answer, busy sc_cmd_resp */
	if (wantresp) {
		KASSERT(!async, (""));
		while (sc->sc_wantresp != -1)
			tsleep(&sc->sc_wantresp, 0, "iwacmdsl", 0);
		sc->sc_wantresp = ring->qid << 16 | ring->cur;
		DPRINTFN(12, ("wantresp is %x\n", sc->sc_wantresp));
	}
#endif

	/*
	 * XXX TODO: need to migrate this code to actually
	 * use the per-command response buffer - if one is available.
	 */

	/*
	 * Is the hardware still available?  (after e.g. above wait).
	 */
	if (sc->sc_flags & IWM_FLAG_STOPPED) {
		error = ENXIO;
		goto out;
	}

	/*
	 * Make sure we have a copy of the original ring
	 * pointer that contains our metadata
	 */
	ring_idx = ring->cur;
	desc = &ring->desc[ring->cur];
	data = &ring->data[ring->cur];

	/*
	 * Handle the payload being larger than the provided buffer
	 */
	if (paylen > sizeof(cmd->payload)) {
		/* Command is too large */
		if (sizeof(cmd->hdr) + paylen > IWA_RBUF_SIZE) {
			device_printf(sc->sc_dev, "%s: payload too big?\n",
			    __func__);
			error = EINVAL;
			goto out;
		}

		/* XXX FreeBSD specific */
		/* XXX TODO: this should be moved into an OS specific chunk */
		m = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, IWA_RBUF_SIZE);
		if (m == NULL) {
			error = ENOMEM;
			goto out;
		}
		IWA_DPRINTF(sc, IWA_DEBUG_TX, "%s: m=%p; m_data=%p\n", __func__, m, mtod(m, void *));
		m->m_pkthdr.len = m->m_len = m->m_ext.ext_size;

		cmd = mtod(m, struct iwl_device_cmd *);
		/* XXX TODO hcmd->len[0] is wrong? It's not the full length */
		error = bus_dmamap_load(sc->sc_dmat, data->map, cmd,
		    hcmd->len[0], iwa_dma_map_addr, &paddr, BUS_DMA_NOWAIT);
		if (error != 0) {
			m_freem(m);
			goto out;
		}
		data->m = m;
	} else {
		/* XXX TODO: is this even right here? */
		cmd = &ring->cmd[ring->cur];
		paddr = data->cmd_paddr;
	}

	cmd->hdr.cmd = code;
	cmd->hdr.flags = 0;
	cmd->hdr.sequence = htole16(IWA_IDX_QID_TO_SEQ(ring->cur, ring->qid));
#if 0
	cmd->hdr.qid = ring->qid;
	cmd->hdr.idx = ring->cur;
#endif

#define	N(a)	(sizeof(a)/sizeof(a[0]))
	for (i = 0, off = 0; i < N(hcmd->data); i++) {
		if (hcmd->len[i] == 0)
			continue;
		memcpy(cmd->payload + off, hcmd->data[i], hcmd->len[i]);
		off += hcmd->len[i];
	}
	KASSERT(off == paylen, (""));
#undef	N

	/* XXX validate this! */
	/* lo field is not aligned */
	addr_lo = htole32((uint32_t)paddr);
	memcpy(&desc->tbs[0].lo, &addr_lo, sizeof(uint32_t));
	desc->tbs[0].hi_n_len  = htole16(iwl_get_dma_hi_addr(paddr)
	    | ((sizeof(cmd->hdr) + paylen) << 4));
	desc->num_tbs = 1;

	IWA_DPRINTF(sc, IWA_DEBUG_CMD,
	    "iwa_send_cmd 0x%x size=%lu %s\n",
	    code,
	    ((unsigned long) hcmd->len[0] + hcmd->len[1] + sizeof(cmd->hdr)),
	    async ? " (async)" : "");

	device_printf(sc->sc_dev, "%s: ", __func__);
	for (i = 0; i < hcmd->len[0] + hcmd->len[1] + sizeof(cmd->hdr); i++) {
	    printf("%02x ", ((char *) cmd)[i] & 0xff);
	}
	printf("\n");

	if (hcmd->len[0] > sizeof(cmd->payload)) {
		bus_dmamap_sync(sc->sc_dmat, data->map, BUS_DMASYNC_PREWRITE);
	} else {
		bus_dmamap_sync(sc->sc_dmat, ring->cmd_dma.map, BUS_DMASYNC_PREWRITE);
	}
	bus_dmamap_sync(sc->sc_dmat, ring->desc_dma.map, BUS_DMASYNC_PREWRITE);

	/* XXX this is 'iwa_grab_nic_access() */
	iwa_set_bit(sc, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
	if (!iwa_poll_bit(sc, CSR_GP_CNTRL,
	    CSR_GP_CNTRL_REG_VAL_MAC_ACCESS_EN,
	    (CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY |
	     CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP), 15000)) {
		device_printf(sc->sc_dev, "acquiring device failed\n");
		error = EBUSY;
		goto out;
	}

#if 0
	iwa_update_sched(sc, ring->qid, ring->cur, 0, 0);
#endif
	IWA_DPRINTF(sc, IWA_DEBUG_CMD,
	    "sending command 0x%x qid %d, idx %d\n",
	    code, ring->qid, ring->cur);

	/*
	 * If we're waiting for a response, then setup the
	 * meta pointer to point to us.  Else, set it to NULL
	 * so things don't get confused.
	 */
	if (wantresp) {
		meta->meta[ring->cur] = hcmd;
	} else {
		meta->meta[ring->cur] = NULL;
	}

	/* Kick command ring. */
	ring->cur = (ring->cur + 1) % IWA_TX_RING_COUNT;
	IWA_REG_WRITE(sc, HBUS_TARG_WRPTR, ring->qid << 8 | ring->cur);

	/*
	 * sync: wait for wakeup from RX completion path
	 *
	 * XXX when you get to making this code work,
	 * ensure we do a wakeup() on
	 * all of those descriptors _when_ we free the driver,
	 * or we may end up with stuck threads on unload!
	 */
	if (! async) {
		int cnt = 0;
		/*
		 * For now, we're going to do a timed sleep
		 * on the transmit ring entry.
		 *
		 * XXX The OpenBSD driver implemented a generation
		 * count so commands sent to the hardware across
		 * things like configuration changes and such
		 * would error out.
		 *
		 * I'm not going to do that just yet (iwn didn't
		 * implement this) but it's entirely possible
		 * that I'm going to have to.
		 */
		/* XXX 5 second command wait for now */
		for (cnt = 0; cnt < 5; cnt++) {
			device_printf(sc->sc_dev,
			    "%s: msleep'ing on %p\n",
			    __func__,
			    desc);
			error = msleep(desc, &sc->sc_mtx,
			    PCATCH,
			    "iwacmd",
			    hz);
			if (error == EWOULDBLOCK)
				continue;
			if (error == 0)
				break;
		}
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: msleep failed; error=%d\n",
			    __func__,
			    error);
			/*
			 * We've failed; so we won't be
			 * handling any command status that
			 * are handed to us past this point.
			 */
			meta->meta[ring_idx] = NULL;
			goto out;
		}

		device_printf(sc->sc_dev,
		    "%s: sleep completed; continue!\n",
		    __func__);
		/* We've received our command response */
		meta->meta[ring_idx] = NULL;
	}

	/*
	 * At this point iwa_cmd_done() has set resp_pkt / resp_obj
	 * for us as appropriate.
	 */

out:
	/* If we hit an error and we have a response, free it here */
	if (wantresp && error != 0) {
		iwa_free_resp(sc, hcmd);
	}

	/*
	 * If we were waiting for a response and
	 * the response data is NULL, return ENOMEM
	 * instead of okay but with NULL pointers.
	 *
	 * This way the caller doesn't think everything
	 * is okay and tries dereferencing things.
	 */
	if (wantresp && error == 0 && hcmd->resp_pkt == NULL) {
		device_printf(sc->sc_dev,
		    "%s: woke up OK but resp_pkt == NULL\n",
		    __func__);
		error = ENOMEM;
	}

	return error;
}

/* iwlwifi: mvm/utils.c */
int
iwa_mvm_send_cmd_pdu(struct iwa_softc *sc, uint8_t id,
	uint32_t flags, uint16_t len, const void *data)
{
	struct iwl_host_cmd cmd = {
		.id = id,
		.len = { len, },
		.data = { data, },
		.flags = flags,
	};

	return iwa_send_cmd(sc, &cmd);
}

/* iwlwifi: mvm/utils.c */
int
iwa_mvm_send_cmd_status(struct iwa_softc *sc,
	struct iwl_host_cmd *cmd, uint32_t *status)
{
	struct iwl_rx_packet *pkt;
	struct iwl_cmd_response *resp;
	int error, resp_len;

	//lockdep_assert_held(&mvm->mutex);

	/* Caller: sync command, but didn't provide a buffer */
	KASSERT((cmd->flags & CMD_WANT_SKB) == 0, (""));
	KASSERT((cmd->flags & CMD_ASYNC) == 0, (""));

	/* sync command; want response */
	cmd->flags |= CMD_WANT_SKB;

	if ((error = iwa_send_cmd(sc, cmd)) != 0)
		return error;
	pkt = cmd->resp_pkt;

	/* Can happen if RFKILL is asserted */
	if (!pkt) {
		error = 0;
		goto out_free_resp;
	}

	if (pkt->hdr.flags & IWL_CMD_FAILED_MSK) {
		device_printf(sc->sc_dev, "%s: failed_msk\n",
		    __func__);
		error = EIO;
		goto out_free_resp;
	}

	resp_len = iwl_rx_packet_payload_len(pkt);
	if (resp_len != sizeof(*resp)) {
		device_printf(sc->sc_dev,
		    "%s: response len %d != expected %d len\n",
		    __func__,
		    resp_len,
		    (int) sizeof(*resp));
		error = EIO;
		goto out_free_resp;
	}

	resp = (void *)pkt->data;
	*status = le32toh(resp->status);

 out_free_resp:
	iwa_free_resp(sc, cmd);
	return error;
}

/* iwlwifi/mvm/utils.c */
int
iwa_mvm_send_cmd_pdu_status(struct iwa_softc *sc, uint8_t id,
	uint16_t len, const void *data, uint32_t *status)
{
	struct iwl_host_cmd cmd = {
		.id = id,
		.len = { len, },
		.data = { data, },
	};

	return iwa_mvm_send_cmd_status(sc, &cmd, status);
}

/*
 * XXX this is some kind of wakeup response thing; it should be
 * done by the received message actually doing a wakeup() as appropriate.
 *
 * XXX TODO: this needs to be split up into a TX side only thing;
 *           which frees any response mbuf associated with a transmit
 *           command.
 *
 * What we really need to do here is:
 *
 * + Whenever the command completion occurs, the rx slot mbuf is given
 *   to the command response, and a new mbuf is put in that RX slot
 * + .. so if we end up receiving a lot of rx frames whilst doing
 *   some deferred processing in another thread, things don't get
 *   trodden over.
 * + Then once the command is completed, the code sleeping on the
 *   command can do what it wants with the buffer and then
 *   free the received mbuf associated with this transmit slot.
 */
void
iwa_free_resp(struct iwa_softc *sc, struct iwl_host_cmd *hcmd)
{

	IWA_LOCK_ASSERT(sc);

	device_printf(sc->sc_dev, "%s: called!; pkt=%p, m=%p\n",
	    __func__,
	    hcmd->resp_pkt,
	    hcmd->resp_obj);
	if (hcmd->resp_obj) {
		struct mbuf *m = (struct mbuf *) hcmd->resp_obj;
		IWA_DPRINTF(sc, IWA_DEBUG_TX, "%s: free m=%p, data=%p\n", __func__, m, mtod(m, void *));
		m_freem(m);
	}
}

/*
 * Process a "command done" firmware notification.  This is where we wakeup
 * processes waiting for a synchronous command completion.
 *
 * XXX TODO: we should likely be gifting the RX packet (ie, mbuf) to the
 * transmit response and replenishing the RX slot with an mbuf.
 *
 * _OR_ we should have the caller pre-allocate a buffer that we memcpy()
 * the command response into, so we don't end up having to replenish
 * mbufs during interrupt handling.
 *
 * from if_iwn
 */
void
iwa_cmd_done(struct iwa_softc *sc, struct iwl_rx_packet *pkt, struct mbuf *m)
{
	struct iwa_tx_ring *ring = &sc->txq[IWL_MVM_CMD_QUEUE];
	struct iwa_tx_ring_meta *meta = &sc->txq_meta[IWL_MVM_CMD_QUEUE];
	struct iwa_tx_data *data;
	int qid, idx;

	IWA_LOCK_ASSERT(sc);

	qid = IWA_SEQ_TO_QID(le16toh(pkt->hdr.sequence));
	idx = IWA_SEQ_TO_IDX(le16toh(pkt->hdr.sequence));

	device_printf(sc->sc_dev, "%s: called; qid=%d, idx=%d; pkt=%p, m=%p\n",
	    __func__,
	    qid,
	    idx,
	    pkt,
	    m);

	if (qid != IWL_MVM_CMD_QUEUE) {
		device_printf(sc->sc_dev,
		    "%s: seq=%04x; not cmd queue?\n",
		    __func__,
		    le16toh(pkt->hdr.sequence));
		/* Not a command ack. */
		goto error;
	}

	/*
	 * Get the tx buffer for the original sent command.
	 */
	data = &ring->data[idx];

	/* If the command was mapped in an mbuf, free it. */
	if (data->m != NULL) {
		bus_dmamap_sync(sc->sc_dmat, data->map, BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_dmat, data->map);
		IWA_DPRINTF(sc, IWA_DEBUG_TX, "%s: free m=%p; data=%p\n", __func__, data->m, mtod(data->m, void *));
		m_freem(data->m);
		data->m = NULL;
	}

	/*
	 * Put the mbuf into the original request, so the
	 * responder can use it as appropriate and then
	 * free it.
	 *
	 * There's no link at the moment from the TX command
	 * ring slot entry to the original command that originated
	 * things.  I'm going to have to add this.
	 */
	if (meta->meta[idx] != NULL) {
		/* Need to squirrel things away as appropriate */
		struct iwl_host_cmd *hcmd = meta->meta[idx];

		/*
		 * If m is NULL, we can't hand the buffer to our
		 * caller.  So send NULL pointers back; the
		 * rest of iwa_cmd_done() will check this and
		 * return an error as appropriate.
		 */
		if (m == NULL) {
			hcmd->resp_pkt = NULL;
			hcmd->resp_obj = NULL;
		} else {
			hcmd->resp_pkt = pkt;
			hcmd->resp_obj = (void *) m;
		}

		/*
		 * The wakeup will now kick the waiting
		 * client to free things.
		 */
	} else {
		/* Nothing to squirrel away; just free */
		if (m != NULL) {
			IWA_DPRINTF(sc, IWA_DEBUG_TX, "%s: free m=%p; data=%p\n", __func__, m, mtod(m, void *));
			m_freem(m);
		}
	}

	/* This wakes up anything sleeping on the specific slot */
	device_printf(sc->sc_dev,
	    "%s: waking up %p\n",
	    __func__,
	    &ring->desc[idx]);

	wakeup(&ring->desc[idx]);
	return;

error:
	/* If we have an mbuf, we have to free it */
	if (m != NULL) {
		IWA_DPRINTF(sc, IWA_DEBUG_TX, "%s: free m=%p; data=%p\n", __func__, m, mtod(m, void *));
		m_freem(m);
	}
	return;
}
