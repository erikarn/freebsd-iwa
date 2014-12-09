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
#include <dev/iwa/iwl/iwl-fw.h>
#include <dev/iwa/iwl/iwl-fh.h>
#include <dev/iwa/iwl/iwl-trans.h>

#include <dev/iwa/iwl/mvm/fw-api.h>

#include <dev/iwa/if_iwa_firmware.h>
#include <dev/iwa/if_iwa_trans.h>
#include <dev/iwa/if_iwa_nvm.h>
#include <dev/iwa/if_iwavar.h>
#include <dev/iwa/if_iwareg.h>
#include <dev/iwa/if_iwa_rx.h>
#include <dev/iwa/if_iwa_fw_util.h>

#define SYNC_RESP_STRUCT(_var_, _pkt_)					\
do {									\
	bus_dmamap_sync(sc->sc_dmat, data->map, \
	    BUS_DMASYNC_POSTREAD);			\
	_var_ = (void *)((_pkt_)+1);					\
} while (/*CONSTCOND*/0)

#define SYNC_RESP_PTR(_ptr_, _len_, _pkt_)				\
do {									\
	bus_dmamap_sync(sc->sc_dmat, data->map, BUS_DMASYNC_POSTREAD);	\
	_ptr_ = (void *)((_pkt_)+1);					\
} while (/*CONSTCOND*/0)

#define ADVANCE_RXQ(sc) (sc->rxq.cur = (sc->rxq.cur + 1) % IWA_RX_RING_COUNT);

/*
 * Process an CSR_INT_BIT_FH_RX or CSR_INT_BIT_SW_RX interrupt.
 * Basic structure from if_iwn
 *
 * Requires: IWA lock held
 */
void
iwa_notif_intr(struct iwa_softc *sc)
{
	uint16_t hw;

	IWA_LOCK_ASSERT(sc);

	bus_dmamap_sync(sc->sc_dmat, sc->rxq.stat_dma.map,
	    BUS_DMASYNC_POSTREAD);

	hw = le16toh(sc->rxq.stat->closed_rb_num) & 0xfff;
	while (sc->rxq.cur != hw) {
		int slot_idx = sc->rxq.cur;
		struct iwa_rx_data *data = &sc->rxq.data[sc->rxq.cur];
		struct iwl_rx_packet *pkt;
		struct iwl_cmd_response *cresp;
		int qid, idx;

		bus_dmamap_sync(sc->sc_dmat, data->map,
		    BUS_DMASYNC_POSTREAD);
		pkt = mtod(data->m, struct iwl_rx_packet *);

		/*
		 * iwl_cmd_header has a le16 sequence field
		 * whose bits 0:14 are user settable.
		 * Look at iwl/iwl-trans.h iwl_cmd_header
		 * for a description of what Linux is doing.
		 */
#if 0
		qid = pkt->hdr.qid & ~0x80;
		idx = pkt->hdr.idx;
#endif
		qid = IWA_SEQ_TO_QID(le16toh(pkt->hdr.sequence));
		idx = IWA_SEQ_TO_IDX(le16toh(pkt->hdr.sequence));

		IWA_DPRINTF(sc,
		    IWA_DEBUG_RX,
		    "rx packet seq=0x%04x len=%d qid=%d idx=%d flags=%x cmd=%x cur=%d hw=%d\n",
		    (int) le16toh(pkt->hdr.sequence),
		    (int) (le32toh(pkt->len_n_flags) & 0x3fff),
		    qid, idx,
		    pkt->hdr.flags,
		    pkt->hdr.cmd,
		    sc->rxq.cur,
		    hw);

		/*
		 * randomly get these from the firmware, no idea why.
		 * they at least seem harmless, so just ignore them for now
		 *
		 * XXX TODO: dump the contents out; let's figure it out!
		 */
		if (__predict_false((pkt->hdr.cmd == 0 && qid == 0 && idx == 0)
		    || pkt->len_n_flags == htole32(0x55550000))) {
			ADVANCE_RXQ(sc);
			continue;
		}

		switch (pkt->hdr.cmd) {
		case REPLY_RX_PHY_CMD:
#if 0
			iwa_mvm_rx_rx_phy_cmd(sc, pkt, data);
#endif
			break;

		case REPLY_RX_MPDU_CMD:
#if 0
			iwa_mvm_rx_rx_mpdu(sc, pkt, data);
#endif
			break;

		case TX_CMD:
#if 0
			iwa_mvm_rx_tx_cmd(sc, pkt, data);
#endif
			break;

		case MISSED_BEACONS_NOTIFICATION:
#if 0
			iwa_mvm_rx_missed_beacons_notif(sc, pkt, data);
#endif
			break;

		case MVM_ALIVE: {
			struct mvm_alive_resp *resp;
			SYNC_RESP_STRUCT(resp, pkt);

			sc->sc_uc.uc_error_event_table
			    = le32toh(resp->error_event_table_ptr);
			sc->sc_uc.uc_log_event_table
			    = le32toh(resp->log_event_table_ptr);
			sc->sched_base = le32toh(resp->scd_base_ptr);
			sc->sc_uc.uc_ok = resp->status == IWL_ALIVE_STATUS_OK;

			sc->sc_uc.uc_intr = true;
			wakeup(&sc->sc_uc);
			break; }

		case CALIB_RES_NOTIF_PHY_DB: {
#if 0
			struct iwl_calib_res_notif_phy_db *phy_db_notif;
			SYNC_RESP_STRUCT(phy_db_notif, pkt);

			iwa_phy_db_set_section(sc, phy_db_notif);
#endif

			break; }

		case STATISTICS_NOTIFICATION: {
#if 0
			struct iwl_notif_statistics *stats;
			SYNC_RESP_STRUCT(stats, pkt);
			memcpy(&sc->sc_stats, stats, sizeof(sc->sc_stats));
#endif
			break; }

		case NVM_ACCESS_CMD:
			/*
			 * XXX TODO: should we put it into the origin
			 * command structure now, and do the wakeup?
			 * Or just handle it after this chunk?
			 */
			/*
			 * XXX TODO: why is this differently treated
			 * to the command block below?
			 */
#if 0
			if (sc->sc_wantresp == ((qid << 16) | idx)) {
				bus_dmamap_sync(sc->sc_dmat, data->map,
				    BUS_DMASYNC_POSTREAD);
				memcpy(sc->sc_cmd_resp,
				    pkt, sizeof(sc->sc_cmd_resp));
			}
#endif
			break;

		case PHY_CONFIGURATION_CMD:
		case TX_ANT_CONFIGURATION_CMD:
		case ADD_STA:
		case MAC_CONTEXT_CMD:
		case REPLY_SF_CFG_CMD:
		case POWER_TABLE_CMD:
		case PHY_CONTEXT_CMD:
		case BINDING_CONTEXT_CMD:
		case TIME_EVENT_CMD:
		case SCAN_REQUEST_CMD:
		case REPLY_BEACON_FILTERING_CMD:
		case MAC_PM_POWER_TABLE:
		case TIME_QUOTA_CMD:
		case REMOVE_STA:
		case TXPATH_FLUSH:
		case LQ_CMD:
			SYNC_RESP_STRUCT(cresp, pkt);
			/*
			 * XXX TODO: should we put it into the origin
			 * command structure now, and do the wakeup?
			 * Or just handle it after this chunk?
			 */
#if 0
			if (sc->sc_wantresp == ((qid << 16) | idx)) {
				memcpy(sc->sc_cmd_resp,
				    pkt, sizeof(*pkt)+sizeof(*cresp));
			}
#endif
			break;

		/* ignore */
		case 0x6c: /* PHY_DB_CMD, no idea why it's not in fw-api.h */
			break;

		case INIT_COMPLETE_NOTIF:
			sc->sc_init_complete = true;
			wakeup(&sc->sc_init_complete);
			break;

		case SCAN_COMPLETE_NOTIFICATION: {
#if 0
			struct iwl_scan_complete_notif *notif;
			SYNC_RESP_STRUCT(notif, pkt);

			iwa_workq_enqueue(sc->sc_eswq, &sc->sc_eswk,
			    iwa_endscan_cb, sc);
#endif
			break; }

		case REPLY_ERROR: {
#if 0
			struct iwl_error_resp *resp;
			SYNC_RESP_STRUCT(resp, pkt);

			aprint_error_dev(sc->sc_dev,
			    "Firmware error 0x%x, cmd 0x%x\n",
			        le32toh(resp->error_type), resp->cmd_id);
#endif
			break; }

		case TIME_EVENT_NOTIFICATION: {
#if 0
			struct iwl_time_event_notif *notif;
			SYNC_RESP_STRUCT(notif, pkt);
			
			if (notif->status) {
				if (le32toh(notif->action) &
				    TE_V2_NOTIF_HOST_EVENT_START)
					sc->sc_auth_prot = 2;
				else
					sc->sc_auth_prot = 0;
			} else {
				sc->sc_auth_prot = -1;
			}
			wakeup(&sc->sc_auth_prot);
#endif
			break; }

		default:
			device_printf(sc->sc_dev,
			    "frame %d/%d %x UNHANDLED (this should not happen)\n",
			    qid, idx, pkt->len_n_flags);
			break;
		}

		device_printf(sc->sc_dev, "%s: checking flags=0x%04x, is_cmd=%d\n",
		    __func__,
		    le16toh(pkt->hdr.sequence),
		    IWA_SEQ_TO_UCODE_RX(le16toh(pkt->hdr.sequence)));

		/*
		 * Right now this unconditionally gives the response
		 * to the command layer and replenishes the mbuf.
		 *
		 * This is inefficient but cleaner.
		 *
		 * Later on we can optimise things by only doing this
		 * if the command layer decides it wants ownership of
		 * the mbuf (ie, the requesting command wanted the
		 * response, and not just to wait for the command to
		 * complete.)
		 */
		if (IWA_SEQ_TO_UCODE_RX(le16toh(pkt->hdr.sequence)) == 0) {
			struct mbuf *m;
			int error;

			device_printf(sc->sc_dev,
			    "%s: command response!\n", __func__);

			SYNC_RESP_STRUCT(cresp, pkt);
			m = data->m;
			/*
			 * Attempt to replace the buffer that we're about
			 * give to the command.
			 *
			 * If we can't replenish things, then we pass in
			 * a NULL mbuf pointer so the caller knows that
			 * it can't steal the buffer.
			 */
			error = iwa_rx_addbuf(sc, &sc->rxq, IWA_RBUF_SIZE,
			    slot_idx);
			if (error != 0) {
				device_printf(sc->sc_dev,
				    "%s: failed to replenish buffer!\n",
				    __func__);
				m = NULL;
			}

			/*
			 * Notify the command layer about the buffer
			 * we're giving to them to free.
			 */
			iwa_cmd_done(sc, pkt, m);
		}

		ADVANCE_RXQ(sc);
	}

	iwa_clear_bit(sc, CSR_GP_CNTRL, CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

	/*
	 * Tell the firmware what we have processed.
	 * Seems like the hardware gets upset unless we align
	 * the write by 8??
	 */
	hw = (hw == 0) ? IWA_RX_RING_COUNT - 1 : hw - 1;
	IWA_REG_WRITE(sc, FH_RSCSR_CHNL0_WPTR, hw & ~7);
}
