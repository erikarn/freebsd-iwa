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

#include <dev/iwa/if_iwa_firmware.h>
#include <dev/iwa/if_iwa_trans.h>
#include <dev/iwa/if_iwavar.h>
#include <dev/iwa/if_iwareg.h>


/*
 * Populate the hardware ID.
 */
static void
iwa_populate_hw_id(struct iwa_softc *sc)
{

	sc->sc_hw_rev = IWA_REG_READ(sc, CSR_HW_REV);
	device_printf(sc->sc_dev, "%s: hw_rev=0x%08x\n", __func__, sc->sc_hw_rev);

	/*
	 * In the 8000 HW family the format of the 4 bytes of CSR_HW_REV have
	 * changed, and now the revision step also includes bit 0-1 (no more
	 * "dash" value). To keep hw_rev backwards compatible - we'll store it
	 * in the old format.
	 */
	if (sc->sc_cfg->device_family == IWL_DEVICE_FAMILY_8000)
		sc->sc_hw_rev = (sc->sc_hw_rev & 0xfff0) |
		    (CSR_HW_REV_STEP(sc->sc_hw_rev << 2) << 2);
}

/*
 * follows iwlwifi/fw.c
 */
static int
iwa_run_init_mvm_ucode(struct iwa_softc *sc, bool justnvm)
{
        int error;

	IWA_LOCK_ASSERT(sc);

	device_printf(sc->sc_dev, "%s: started\n", __func__);

        /* do not operate with rfkill switch turned on */
        if ((sc->sc_flags & IWM_FLAG_RFKILL) && !justnvm) {
                device_printf(sc->sc_dev, "rfkill active, no go\n");
                return EPERM;
        }

	/*
	 * Note: the firmware must be loaded by the caller.
	 */

        sc->sc_init_complete = false;
        if ((error = iwa_mvm_load_ucode_wait_alive(sc,
            IWL_UCODE_INIT)) != 0)
                return error;

        if (justnvm) {
		device_printf(sc->sc_dev, "%s: TODO: iwa_nvm_init\n", __func__);
#if 0
                if ((error = iwa_nvm_init(sc)) != 0) {
                        device_printf(sc->sc_dev, "failed to read nvm\n");
                        return error;
                }
                device_printf(sc->sc_dev, "MAC address: %s\n",
                    ether_sprintf(sc->sc_nvm.hw_addr));
                memcpy(&sc->sc_ic.ic_myaddr,
                    &sc->sc_nvm.hw_addr, ETHER_ADDR_LEN);

                sc->sc_scan_cmd_len = sizeof(struct iwl_scan_cmd)
                    + sc->sc_capa_max_probe_len
                    + MAX_NUM_SCAN_CHANNELS * sizeof(struct iwl_scan_channel);
                sc->sc_scan_cmd = iwa_malloc(sc->sc_scan_cmd_len, true);
#endif
                return 0;
        }

#if 0
        /* Send TX valid antennas before triggering calibrations */
        if ((error = iwa_send_tx_ant_cfg(sc, IWM_FW_VALID_TX_ANT(sc))) != 0)
                return error;

        /*
        * Send phy configurations command to init uCode
        * to start the 16.0 uCode init image internal calibrations.
        */
        if ((error = iwa_send_phy_cfg_cmd(sc)) != 0 ) {
                device_printf(sc->sc_dev, "Failed to run INIT "
                    "calibrations: %d\n", error);
                return error;
        }

        /*
         * Nothing to do but wait for the init complete notification
         * from the firmware
         */
        while (!sc->sc_init_complete)
                if ((error = tsleep(&sc->sc_init_complete,
                    0, "iwminit", 2*hz)) != 0)
                        break;

        return error;
#endif
        return (EINVAL);
}

/*
 * Pre-initialise the hardware.
 *
 * The firmware must already have been loaded.
 */
static int
iwa_preinit(struct iwa_softc *sc)
{
	int error;

	IWA_LOCK_ASSERT(sc);

	if ((error = iwa_prepare_card_hw(sc)) != 0)
		return error;

	if ((error = iwa_start_hw(sc)) != 0)
		return error;

	if ((error = iwa_run_init_mvm_ucode(sc, true)) != 0) {
		return error;
	}

	iwa_stop_device(sc);
	return 0;
}

static void
iwa_notif_intr(struct iwa_softc *sc)
{

	device_printf(sc->sc_dev, "%s: called\n", __func__);
}

static void
iwa_stop_locked(struct iwa_softc *sc, int disable)
{

	IWA_LOCK_ASSERT(sc);

#if 0
        struct ieee80211com *ic = &sc->sc_ic;

        sc->sc_flags &= ~IWM_FLAG_HW_INITED;
        sc->sc_flags |= IWM_FLAG_STOPPED;
        sc->sc_generation++;
        sc->sc_scanband = 0;
        sc->sc_auth_prot = 0;
#ifdef __OpenBSD__
        ic->ic_scan_lock = IEEE80211_SCAN_UNLOCKED;
#endif
        ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);

        if (ic->ic_state != IEEE80211_S_INIT)
                ieee80211_new_state(ic, IEEE80211_S_INIT, -1);

        ifp->if_timer = sc->sc_tx_timer = 0;
#endif
	iwa_stop_device(sc);
}

static void
iwa_stop(struct ifnet *ifp, int disable)
{
	struct iwa_softc *sc = ifp->if_softc;

	IWA_LOCK(sc);
	iwa_stop_locked(sc, disable);
	IWA_UNLOCK(sc);
}

int
iwa_intr(struct iwa_softc *sc)
{
//	struct ifnet *ifp = sc->sc_ifp;
	int handled = 0;
	int r1, r2, rv = 0;
	bool isperiodic = false;

	IWA_LOCK(sc);

	IWA_REG_WRITE(sc, CSR_INT_MASK, 0);

	if (sc->sc_flags & IWM_FLAG_USE_ICT) {
		uint32_t *ict = (void *) sc->ict_dma.vaddr;
		int tmp;

		tmp = htole32(ict[sc->ict_cur]);
		if (!tmp)
			goto out_ena;

		/*
		 * ok, there was something.  keep plowing until we have all.
		 */
		r1 = r2 = 0;
		while (tmp) {
			r1 |= tmp;
			ict[sc->ict_cur] = 0;
			sc->ict_cur = (sc->ict_cur+1) % IWM_ICT_COUNT;
			tmp = htole32(ict[sc->ict_cur]);
		}

		/* this is where the fun begins.  don't ask */
		if (r1 == 0xffffffff)
			r1 = 0;

		/* i am not expected to understand this */
		if (r1 & 0xc0000)
			r1 |= 0x8000;
		r1 = (0xff & r1) | ((0xff00 & r1) << 16);
	} else {
		r1 = IWA_REG_READ(sc, CSR_INT);
		/* "hardware gone" (where, fishing?) */
		if (r1 == 0xffffffff || (r1 & 0xfffffff0) == 0xa5a5a5a0)
			goto out;
		r2 = IWA_REG_READ(sc, CSR_FH_INT_STATUS);
	}
	if (r1 == 0 && r2 == 0) {
		goto out_ena;
	}

	IWA_REG_WRITE(sc, CSR_INT, r1 | ~sc->sc_intmask);

	/* ignored */
	handled |= (r1 & (CSR_INT_BIT_ALIVE /*| CSR_INT_BIT_SCD*/));

	if (r1 & CSR_INT_BIT_SW_ERR) {
#if 0
		int i;

		iwm_nic_error(sc);

		/* Dump driver status (TX and RX rings) while we're here. */
		aprint_error("driver status:\n");
		for (i = 0; i < IWL_MVM_MAX_QUEUES; i++) {
			struct iwm_tx_ring *ring = &sc->txq[i];
			aprint_error("  tx ring %2d: qid=%-2d cur=%-3d "
			    "queued=%-3d\n",
			    i, ring->qid, ring->cur, ring->queued);
		}
		aprint_error("  rx ring: cur=%d\n", sc->rxq.cur);
		aprint_error("  802.11 state %d\n", sc->sc_ic.ic_state);
#endif

		device_printf(sc->sc_dev,
		    "firmware error, stopping device\n");
//		ifp->if_flags &= ~IFF_UP;
		iwa_stop_locked(sc, 1);
		rv = 1;
		goto out;

	}

	if (r1 & CSR_INT_BIT_HW_ERR) {
		handled |= CSR_INT_BIT_HW_ERR;
		device_printf(sc->sc_dev,
		    "hardware error, stopping device \n");
//		ifp->if_flags &= ~IFF_UP;
		iwa_stop_locked(sc, 1);
		rv = 1;
		goto out;
	}

	/* firmware chunk loaded */
	if (r1 & CSR_INT_BIT_FH_TX) {
		IWA_REG_WRITE(sc, CSR_FH_INT_STATUS, CSR_FH_INT_TX_MASK);
		handled |= CSR_INT_BIT_FH_TX;
		device_printf(sc->sc_dev, "%s: called; FH_TX set\n", __func__);
		sc->sc_fw_chunk_done = true;
		wakeup(&sc->sc_fw);
	}

	if (r1 & CSR_INT_BIT_RF_KILL) {
		handled |= CSR_INT_BIT_RF_KILL;
//		if (iwa_check_rfkill(sc) && (ifp->if_flags & IFF_UP)) {
		if (iwa_check_rfkill(sc)) {
			device_printf(sc->sc_dev,
			    "rfkill switch, disabling interface\n");
//			ifp->if_flags &= ~IFF_UP;
			iwa_stop_locked(sc, 1);
		}
	}

	/*
	 * The Linux driver uses periodic interrupts to avoid races.
	 * We cargo-cult like it's going out of fashion.
	 */
	if (r1 & CSR_INT_BIT_RX_PERIODIC) {
		handled |= CSR_INT_BIT_RX_PERIODIC;
		IWA_REG_WRITE(sc, CSR_INT, CSR_INT_BIT_RX_PERIODIC);
		if ((r1 & (CSR_INT_BIT_FH_RX | CSR_INT_BIT_SW_RX)) == 0)
			IWA_REG_WRITE_1(sc,
			    CSR_INT_PERIODIC_REG, CSR_INT_PERIODIC_DIS);
		isperiodic = true;
	}

	if ((r1 & (CSR_INT_BIT_FH_RX | CSR_INT_BIT_SW_RX)) || isperiodic) {
		handled |= (CSR_INT_BIT_FH_RX | CSR_INT_BIT_SW_RX);
		IWA_REG_WRITE(sc, CSR_FH_INT_STATUS, CSR_FH_INT_RX_MASK);
		iwa_notif_intr(sc);

		/* enable periodic interrupt, see above */
		if (r1 & (CSR_INT_BIT_FH_RX | CSR_INT_BIT_SW_RX) && !isperiodic)
			IWA_REG_WRITE_1(sc, CSR_INT_PERIODIC_REG, CSR_INT_PERIODIC_ENA);
	}

	if (__predict_false(r1 & ~handled))
		device_printf(sc->sc_dev, "unhandled interrupts: %x\n", r1);
	rv = 1;

out_ena:
	iwa_restore_interrupts(sc);
out:

	IWA_UNLOCK(sc);

	return rv;
}

/*
 * XXX TODO - we can't assume interrupts are available at attach() time,
 * so this whole "load initial firmware at attach time" has to either stop
 * and be deferred until interface up, or it needs to use some polling
 * of the status registers to verify things are up.
 *
 * .. grr.
 */
int
iwa_attach(struct iwa_softc *sc)
{
#if 0
	struct ieee80211com *ic;
	struct ifnet *ifp;
	uint8_t macaddr[IEEE80211_ADDR_LEN];
#endif
	int error;
	int i;

#ifdef	IWA_DEBUG
	error = resource_int_value(device_get_name(sc->sc_dev),
	    device_get_unit(sc->sc_dev), "debug", &(sc->sc_debug));
	if (error != 0)
		sc->sc_debug = 0xffffffff;
#else
	sc->sc_debug = 0xffffffff;
#endif

	IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "->%s: begin\n",__func__);

	/* Setup initial firmware details */
	sc->sc_fw_dmasegsz = IWM_FWDMASEGSZ;

	/* Read hardware revision */
	iwa_populate_hw_id(sc);

	device_printf(sc->sc_dev,
	    "hw rev: 0x%x, dash: %d, step: %d\n",
	    sc->sc_hw_rev & CSR_HW_REV_TYPE_MSK,
	    CSR_HW_REV_DASH(sc->sc_hw_rev),
	    CSR_HW_REV_STEP(sc->sc_hw_rev));

	/* XXX do we need this; we do it down in preinit() */
	if ((error = iwa_prepare_card_hw(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "hardware not ready, error %d\n",
		    error);
		goto fail;
	}

	/* Allocate DMA memory for firmware transfers. */
	if ((error = iwa_alloc_fwmem(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate memory for firmware, error %d\n",
		    error);
		goto fail;
	}

	/* Allocate "Keep Warm" page. */
	if ((error = iwa_alloc_kw(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate keep warm page, error %d\n", error);
		goto fail;
	}

	if ((error = iwa_alloc_ict(sc)) != 0) {
		device_printf(sc->sc_dev, "could not allocate ICT table, error %d\n",
		    error);
		goto fail;
	}

	/* Allocate TX scheduler "rings". */
	if ((error = iwa_alloc_sched(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate TX scheduler rings, error %d\n", error);
		goto fail;
	}

	for (i = 0; i < sc->sc_cfg->base_params->num_of_queues; i++) {
		if ((error = iwa_alloc_tx_ring(sc, &sc->txq[i], i)) != 0) {
			device_printf(sc->sc_dev,
			    "could not allocate TX ring %d, error %d\n", i,
			    error);
			goto fail;
		}
	}

	/* Allocate RX ring. */
	if ((error = iwa_alloc_rx_ring(sc, &sc->rxq)) != 0) {
		device_printf(sc->sc_dev, "could not allocate RX ring, error %d\n",
		    error);
		goto fail;
	}

	/*
	 * Load initial firmware - do this before the lock is grabbed.
	 */
	if ((error = iwa_find_firmware(sc)) != 0) {
		device_printf(sc->sc_dev, "firmware load failed; error %d\n",
		    error);
		goto fail;
	}

	IWA_LOCK(sc);

	/* Clear pending interrupts. */
	IWA_REG_WRITE(sc, CSR_INT, 0xffffffff);

	if ((error = iwa_preinit(sc)) != 0) {
		device_printf(sc->sc_dev, "failed to init firmware: %d\n",
		    error);
		IWA_UNLOCK(sc);
		goto fail;
	}

	IWA_UNLOCK(sc);

#if 0
	ifp = sc->sc_ifp = if_alloc(IFT_IEEE80211);
	if (ifp == NULL) {
		device_printf(dev, "can not allocate ifnet structure\n");
		goto fail;
	}

	ic = ifp->if_l2com;
	ic->ic_ifp = ifp;
	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */

	/* Set device capabilities. */
	ic->ic_caps =
		  IEEE80211_C_STA		/* station mode supported */
		| IEEE80211_C_MONITOR		/* monitor mode supported */
		| IEEE80211_C_BGSCAN		/* background scanning */
		| IEEE80211_C_TXPMGT		/* tx power management */
		| IEEE80211_C_SHSLOT		/* short slot time supported */
		| IEEE80211_C_WPA
		| IEEE80211_C_SHPREAMBLE	/* short preamble supported */
#if 0
		| IEEE80211_C_IBSS		/* ibss/adhoc mode */
#endif
		| IEEE80211_C_WME		/* WME */
		| IEEE80211_C_PMGT		/* Station-side power mgmt */
		;

	/* Read MAC address, channels, etc from EEPROM. */
	if ((error = iwn_read_eeprom(sc, macaddr)) != 0) {
		device_printf(dev, "could not read EEPROM, error %d\n",
		    error);
		goto fail;
	}

	/* Count the number of available chains. */
	sc->ntxchains =
	    ((sc->txchainmask >> 2) & 1) +
	    ((sc->txchainmask >> 1) & 1) +
	    ((sc->txchainmask >> 0) & 1);
	sc->nrxchains =
	    ((sc->rxchainmask >> 2) & 1) +
	    ((sc->rxchainmask >> 1) & 1) +
	    ((sc->rxchainmask >> 0) & 1);
	if (bootverbose) {
		device_printf(dev, "MIMO %dT%dR, %.4s, address %6D\n",
		    sc->ntxchains, sc->nrxchains, sc->eeprom_domain,
		    macaddr, ":");
	}

	if (sc->sc_flags & IWN_FLAG_HAS_11N) {
		ic->ic_rxstream = sc->nrxchains;
		ic->ic_txstream = sc->ntxchains;

		/*
		 * Some of the 3 antenna devices (ie, the 4965) only supports
		 * 2x2 operation.  So correct the number of streams if
		 * it's not a 3-stream device.
		 */
		if (! iwn_is_3stream_device(sc)) {
			if (ic->ic_rxstream > 2)
				ic->ic_rxstream = 2;
			if (ic->ic_txstream > 2)
				ic->ic_txstream = 2;
		}

		ic->ic_htcaps =
			  IEEE80211_HTCAP_SMPS_OFF	/* SMPS mode disabled */
			| IEEE80211_HTCAP_SHORTGI20	/* short GI in 20MHz */
			| IEEE80211_HTCAP_CHWIDTH40	/* 40MHz channel width*/
			| IEEE80211_HTCAP_SHORTGI40	/* short GI in 40MHz */
#ifdef notyet
			| IEEE80211_HTCAP_GREENFIELD
#if IWN_RBUF_SIZE == 8192
			| IEEE80211_HTCAP_MAXAMSDU_7935	/* max A-MSDU length */
#else
			| IEEE80211_HTCAP_MAXAMSDU_3839	/* max A-MSDU length */
#endif
#endif
			/* s/w capabilities */
			| IEEE80211_HTC_HT		/* HT operation */
			| IEEE80211_HTC_AMPDU		/* tx A-MPDU */
#ifdef notyet
			| IEEE80211_HTC_AMSDU		/* tx A-MSDU */
#endif
			;
	}

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_init = iwn_init;
	ifp->if_ioctl = iwn_ioctl;
	ifp->if_start = iwn_start;
	IFQ_SET_MAXLEN(&ifp->if_snd, ifqmaxlen);
	ifp->if_snd.ifq_drv_maxlen = ifqmaxlen;
	IFQ_SET_READY(&ifp->if_snd);

	ieee80211_ifattach(ic, macaddr);
	ic->ic_vap_create = iwn_vap_create;
	ic->ic_vap_delete = iwn_vap_delete;
	ic->ic_raw_xmit = iwn_raw_xmit;
	ic->ic_node_alloc = iwn_node_alloc;
	sc->sc_ampdu_rx_start = ic->ic_ampdu_rx_start;
	ic->ic_ampdu_rx_start = iwn_ampdu_rx_start;
	sc->sc_ampdu_rx_stop = ic->ic_ampdu_rx_stop;
	ic->ic_ampdu_rx_stop = iwn_ampdu_rx_stop;
	sc->sc_addba_request = ic->ic_addba_request;
	ic->ic_addba_request = iwn_addba_request;
	sc->sc_addba_response = ic->ic_addba_response;
	ic->ic_addba_response = iwn_addba_response;
	sc->sc_addba_stop = ic->ic_addba_stop;
	ic->ic_addba_stop = iwn_ampdu_tx_stop;
	ic->ic_newassoc = iwn_newassoc;
	ic->ic_wme.wme_update = iwn_updateedca;
	ic->ic_update_mcast = iwn_update_mcast;
	ic->ic_scan_start = iwn_scan_start;
	ic->ic_scan_end = iwn_scan_end;
	ic->ic_set_channel = iwn_set_channel;
	ic->ic_scan_curchan = iwn_scan_curchan;
	ic->ic_scan_mindwell = iwn_scan_mindwell;
	ic->ic_setregdomain = iwn_setregdomain;

	iwn_radiotap_attach(sc);

	callout_init_mtx(&sc->calib_to, &sc->sc_mtx, 0);
	callout_init_mtx(&sc->watchdog_to, &sc->sc_mtx, 0);
	TASK_INIT(&sc->sc_reinit_task, 0, iwn_hw_reset, sc);
	TASK_INIT(&sc->sc_radioon_task, 0, iwn_radio_on, sc);
	TASK_INIT(&sc->sc_radiooff_task, 0, iwn_radio_off, sc);
	TASK_INIT(&sc->sc_panic_task, 0, iwn_panicked, sc);

	sc->sc_tq = taskqueue_create("iwn_taskq", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->sc_tq);
	error = taskqueue_start_threads(&sc->sc_tq, 1, 0, "iwn_taskq");
	if (error != 0) {
		device_printf(dev, "can't start threads, error %d\n", error);
		goto fail;
	}

	iwn_sysctlattach(sc);

	/*
	 * Hook our interrupt after all initialization is complete.
	 */
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, iwn_intr, sc, &sc->sc_ih);
	if (error != 0) {
		device_printf(dev, "can't establish interrupt, error %d\n",
		    error);
		goto fail;
	}

#if 0
	device_printf(sc->sc_dev, "%s: rx_stats=%d, rx_stats_bt=%d\n",
	    __func__,
	    sizeof(struct iwn_stats),
	    sizeof(struct iwn_stats_bt));
#endif

	if (bootverbose)
		ieee80211_announce(ic);
#endif

	IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "->%s: end\n",__func__);
	return 0;

fail:
	iwa_detach(sc);
	IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "->%s: end in error\n",__func__);
	return (error);
}

int
iwa_detach(struct iwa_softc *sc)
{
	struct ifnet *ifp = sc->sc_ifp;
	int qid;
#if 0
	struct ieee80211com *ic;

	DPRINTF(sc, IWN_DEBUG_TRACE, "->%s begin\n", __func__);

	if (ifp != NULL) {
		ic = ifp->if_l2com;

		ieee80211_draintask(ic, &sc->sc_reinit_task);
		ieee80211_draintask(ic, &sc->sc_radioon_task);
		ieee80211_draintask(ic, &sc->sc_radiooff_task);

		iwn_stop(sc);

		taskqueue_drain_all(sc->sc_tq);
		taskqueue_free(sc->sc_tq);

		callout_drain(&sc->watchdog_to);
		callout_drain(&sc->calib_to);
		ieee80211_ifdetach(ic);
	}
#endif

	IWA_LOCK(sc);

	/* Free DMA resources. */
	iwa_free_rx_ring(sc, &sc->rxq);
	for (qid = 0; qid < sc->sc_cfg->base_params->num_of_queues; qid++)
		iwa_free_tx_ring(sc, &sc->txq[qid]);
	iwa_free_sched(sc);
	iwa_free_kw(sc);
	iwa_free_ict(sc);
	iwa_free_fwmem(sc);

	IWA_UNLOCK(sc);

	if (ifp != NULL)
		if_free(ifp);

	IWA_DPRINTF(sc, IWA_DEBUG_TRACE, "->%s: end\n", __func__);

	return (0);
}

int
iwa_shutdown(struct iwa_softc *sc)
{
#if 0
	struct iwa_softc *sc = device_get_softc(dev);

	iwn_stop(sc);
#endif
	return 0;
}

int
iwa_suspend(struct iwa_softc *sc)
{
#if 0
	struct iwa_softc *sc = device_get_softc(dev);
	struct ieee80211com *ic = sc->sc_ifp->if_l2com;

	ieee80211_suspend_all(ic);
#endif
	return 0;
}

int
iwa_resume(struct iwa_softc *sc)
{
#if 0
	struct iwa_softc *sc = device_get_softc(dev);
	struct ieee80211com *ic = sc->sc_ifp->if_l2com;

	ieee80211_resume_all(ic);
#endif
	return 0;
}

