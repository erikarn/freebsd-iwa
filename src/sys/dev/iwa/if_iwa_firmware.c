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

#include <dev/iwa/iwl/iwl-fh.h>
#include <dev/iwa/iwl/iwl-fw.h>
#include <dev/iwa/iwl/iwl-csr.h>
#include <dev/iwa/iwl/iwl-config.h>

#include <dev/iwa/if_iwa_debug.h>

#include <dev/iwa/if_iwa_firmware.h>
#include <dev/iwa/if_iwa_trans.h>
#include <dev/iwa/if_iwareg.h>
#include <dev/iwa/if_iwavar.h>

/*
 * XXX TODO: pull out the firmware bits completely from the
 * softc so this file doesn't require if_athvar.h to load in.
 */


/*
 * Load in the firmware for the NIC.
 *
 * Returns 0 if the firmware was grabbed, non-zero if there
 * was a problem.
 *
 * This may sleep in the firmware(9) API - no locks must be
 * held.
 */
static int
if_iwa_firmware_load(struct iwa_softc *sc, struct iwa_fw_info *fw,
    const char *fw_name)
{
	const struct firmware *fwh;
	int error;

	IWA_UNLOCK_ASSERT(sc);

	/* Open */
	fwh = firmware_get(fw_name);
	if (fwh == NULL) {
		device_printf(sc->sc_dev,
		    "%s: failed to read firmware (%s)\n",
		    __func__,
		    fw_name);
		return (EINVAL);
	}

	fw->fw_rawsize = fwh->datasize;
	/*
	 * Well, this is how the Linux driver checks it ....
	 */
	if (fw->fw_rawsize < sizeof(uint32_t)) {
		device_printf(sc->sc_dev,
		    "firmware too short: %zd bytes\n", fw->fw_rawsize);
		error = EINVAL;
		goto out;
	}

	/* some sanity */
	if (fw->fw_rawsize > IWM_FWMAXSIZE) {
		device_printf(sc->sc_dev,
		    "firmware size is ridiculous: %zd bytes\n",
		    fw->fw_rawsize);
		error = EINVAL;
		goto out;
	}

	/* Read the firmware. */
	fw->fw_rawdata = malloc(fw->fw_rawsize, M_TEMP, M_WAITOK);
	if (fw->fw_rawdata == NULL) {
		device_printf(sc->sc_dev,
		    "not enough memory to stock firmware %s\n", fw_name);
		error = ENOMEM;
		goto out;
	}

	memcpy(fw->fw_rawdata, fwh->data, fw->fw_rawsize);
	error = 0;

out:
	/* caller will release memory, if necessary */
	firmware_put(fwh, FIRMWARE_UNLOAD);
	return (error);
}

/*
 * look for a suitable firmware version to use.
 */
int
iwa_find_firmware(struct iwa_softc *sc)
{
#if 0
	int i, error;
	char fwname[64];
#endif

	if (sc->sc_cfg == NULL) {
		device_printf(sc->sc_dev, "%s: called; cfg=NULL; no config?\n",
		    __func__);
		return (EINVAL);
	}

#if 0
	/* XXX firmware name is likely not right */
	for (i = sc->sc_cfg->ucode_api_min; i <= sc->sc_cfg->ucode_api_max; i++) {
		snprintf(fwname, 32, "%s%d",
		    sc->sc_cfg->fw_name_pre,
		    i);
		device_printf(sc->sc_dev, "%s: trying to load firmware '%s'\n",
		    __func__,
		    fwname);
		error = if_iwa_firmware_load(sc, &sc->sc_fw, fwname);
		if (error == 0)
			return (0);
	}

	return (ENOENT);
#else
	return if_iwa_firmware_load(sc, &sc->sc_fw, "iwa_fw_7260_9");
#endif
}

/*
 * Firmware parser.
 */
static int
iwa_store_cscheme(struct iwa_softc *sc, uint8_t *data, size_t dlen)
{
	struct iwl_fw_cscheme_list *l = (void *)data;

	if (dlen < sizeof(*l) ||
	    dlen < sizeof(l->size) + l->size * sizeof(*l->cs))
		return EINVAL;

	/* we don't actually store anything for now, always use s/w crypto */
	/* XXX [adrian] - sigh. */

	return 0;
}

static int
iwa_firmware_store_section(struct iwa_softc *sc,
	enum iwl_ucode_type type, uint8_t *data, size_t dlen)
{
	struct fw_sects *fws;
	struct fw_onesect *fwone;

	if (type >= IWL_UCODE_TYPE_MAX)
		return EINVAL;
	if (dlen < sizeof(uint32_t))
		return EINVAL;

	fws = &sc->sc_fw.fw_sects[type];
	if (fws->fw_count >= IWL_UCODE_SECTION_MAX)
		return EINVAL;

	fwone = &fws->fw_sect[fws->fw_count];

	/* first 32bit are device load offset */
	memcpy(&fwone->fws_devoff, data, sizeof(uint32_t));

	/* rest is data */
	fwone->fws_data = data + sizeof(uint32_t);
	fwone->fws_len = dlen - sizeof(uint32_t);

	/* for freeing the buffer during driver unload */
	fwone->fws_alloc = data;
	fwone->fws_allocsize = dlen;

	fws->fw_count++;
	fws->fw_totlen += fwone->fws_len;

	return 0;
}

/* iwlwifi: iwl-drv.c */
struct iwl_tlv_calib_data {
	uint32_t ucode_type;
	struct iwl_tlv_calib_ctrl calib;
} __packed;

static int
iwa_set_default_calib(struct iwa_softc *sc, const void *data)
{
	const struct iwl_tlv_calib_data *def_calib = data;
	uint32_t ucode_type = le32toh(def_calib->ucode_type);

	if (ucode_type >= IWL_UCODE_TYPE_MAX) {
		device_printf(sc->sc_dev, "Wrong ucode_type %u for default "
		    "calibration.\n", ucode_type);
		return EINVAL;
	}

	sc->sc_default_calib[ucode_type].flow_trigger =
	    def_calib->calib.flow_trigger;
	sc->sc_default_calib[ucode_type].event_trigger =
	    def_calib->calib.event_trigger;

	return 0;
}

static int
iwa_read_firmware(struct iwa_softc *sc)
{
	struct iwa_fw_info *fw = &sc->sc_fw;
        struct iwl_tlv_ucode_header *uhdr;
        struct iwl_ucode_tlv tlv;
	enum iwl_ucode_tlv_type tlv_type;
	uint8_t *data;
	int error, len;
#if 0
	int status;
#endif

	IWA_LOCK_ASSERT(sc);

#if 0
	/*
	 * Wait for the firmware load to complete.
	 */
	if (fw->fw_status == FW_STATUS_NONE) {
		fw->fw_status = FW_STATUS_INPROGRESS;
	} else {
		while (fw->fw_status == FW_STATUS_INPROGRESS)
			tsleep(&sc->sc_fw, 0, "iwmfwp", 0);
	}
	status = fw->fw_status;

	if (status == FW_STATUS_DONE)
		return 0;
	else if (status < 0)
		return -status;

	KASSERT(status == FW_STATUS_INPROGRESS);

	/*
	 * Load firmware into driver memory.
	 * fw_rawdata and fw_rawsize will be set.
	 */
	if ((error = iwa_firmload(sc)) != 0)
		goto out;
#endif
	if (fw->fw_rawdata == NULL) {
		device_printf(sc->sc_dev, "%s: no firmware in memory?\n",
		    __func__);
		return (EINVAL);
	}

	/*
	 * Parse firmware contents
	 */
	uhdr = (void *)fw->fw_rawdata;
	if (*(uint32_t *)fw->fw_rawdata != 0
	    || le32toh(uhdr->magic) != IWL_TLV_UCODE_MAGIC) {
		device_printf(sc->sc_dev, "invalid firmware magic/empty firmware\n");
		error = EINVAL;
		goto out;
	}

	sc->sc_fwver = le32toh(uhdr->ver);
	device_printf(sc->sc_dev, "microcode version %d.%d (API ver %d)\n",
	    IWL_UCODE_MAJOR(sc->sc_fwver),
	    IWL_UCODE_MINOR(sc->sc_fwver),
	    IWL_UCODE_API(sc->sc_fwver));

	data = uhdr->data;
	len = fw->fw_rawsize - sizeof(*uhdr);

	while (len >= sizeof(tlv)) {
		uint32_t tlv_len;
		void *tlv_data;

		memcpy(&tlv, data, sizeof(tlv));
		tlv_len = le32toh(tlv.length);
		tlv_type = le32toh(tlv.type);

		len -= sizeof(tlv);
		data += sizeof(tlv);
		tlv_data = data;

		if (len < tlv_len) {
			device_printf(sc->sc_dev,
			    "firmware image invalid length\n");
			error = EINVAL;
			goto parse_out;
		}

		switch ((int)tlv_type) {
		case IWL_UCODE_TLV_PROBE_MAX_LEN:
			if (tlv_len < sizeof(uint32_t)) {
				error = EINVAL;
				goto parse_out;
			}
			sc->sc_capa_max_probe_len
			    = le32toh(*(uint32_t *)tlv_data);
			/* limit it to something sensible */
			if (sc->sc_capa_max_probe_len > (1<<16)) {
				device_printf(sc->sc_dev,
				    "IWL_UCODE_TLV_PROBE_MAX_LEN ridiculous\n");
				error = EINVAL;
				goto parse_out;
			}
			break;
		case IWL_UCODE_TLV_PAN:
			if (tlv_len) {
				error = EINVAL;
				goto parse_out;
			}
			sc->sc_capaflags |= IWL_UCODE_TLV_FLAGS_PAN;
			break;
		case IWL_UCODE_TLV_FLAGS:
			if (tlv_len < sizeof(uint32_t)) {
				error = EINVAL;
				goto parse_out;
			}
			/*
			 * Apparently there can be many flags, but Linux driver
			 * parses only the first one, and so do we.
			 *
			 * XXX: why does this override IWL_UCODE_TLV_PAN?
			 * Intentional or a bug?  Observations from
			 * current firmware file:
			 *  1) TLV_PAN is parsed first
			 *  2) TLV_FLAGS contains TLV_FLAGS_PAN
			 * ==> this resets TLV_PAN to itself... hnnnk
			 */
			sc->sc_capaflags = le32toh(*(uint32_t *)tlv_data);
			break;
		case IWL_UCODE_TLV_CSCHEME:
			if ((error = iwa_store_cscheme(sc,
			    tlv_data, tlv_len)) != 0)
				goto parse_out;
			break;
		case IWL_UCODE_TLV_NUM_OF_CPU:
			if (tlv_len != sizeof(uint32_t)) {
				error = EINVAL;
				goto parse_out;
			}
			if (le32toh(*(uint32_t*)tlv_data) != 1) {
				device_printf(sc->sc_dev, "driver supports "
				    "only TLV_NUM_OF_CPU == 1");
				error = EINVAL;
				goto parse_out;
			}
			break;
		case IWL_UCODE_TLV_SEC_RT:
			if ((error = iwa_firmware_store_section(sc,
			    IWL_UCODE_REGULAR, tlv_data, tlv_len)) != 0)
				goto parse_out;
			break;
		case IWL_UCODE_TLV_SEC_INIT:
			if ((error = iwa_firmware_store_section(sc,
			    IWL_UCODE_INIT, tlv_data, tlv_len)) != 0)
				goto parse_out;
			break;
		case IWL_UCODE_TLV_SEC_WOWLAN:
			if ((error = iwa_firmware_store_section(sc,
			    IWL_UCODE_WOWLAN, tlv_data, tlv_len)) != 0)
				goto parse_out;
			break;
		case IWL_UCODE_TLV_DEF_CALIB:
			if (tlv_len != sizeof(struct iwl_tlv_calib_data)) {
				error = EINVAL;
				goto parse_out;
			}
			if ((error = iwa_set_default_calib(sc, tlv_data)) != 0)
				goto parse_out;
			break;
		case IWL_UCODE_TLV_PHY_SKU:
			if (tlv_len != sizeof(uint32_t)) {
				error = EINVAL;
				goto parse_out;
			}
			sc->sc_fw_phy_config = le32toh(*(uint32_t *)tlv_data);
			break;

		case IWL_UCODE_TLV_API_CHANGES_SET:
		case IWL_UCODE_TLV_ENABLED_CAPABILITIES:
			/* ignore, not used by current driver */
			break;

		default:
			device_printf(sc->sc_dev,
			    "unknown firmware section %d, abort\n", tlv_type);
			error = EINVAL;
			goto parse_out;
		}

		len -= roundup(tlv_len, 4);
		data += roundup(tlv_len, 4);
	}

	if (error != 0) {
		device_printf(sc->sc_dev, "%s: parsed firmware ok but error != 0?\n",
		    __func__);
		error = EINVAL;
		goto out;
	}

 parse_out:
	if (error) {
		device_printf(sc->sc_dev, "firmware parse error, "
		    "section type %d\n", tlv_type);
	}

	if (!(sc->sc_capaflags & IWL_UCODE_TLV_FLAGS_PM_CMD_SUPPORT)) {
		device_printf(sc->sc_dev,
		    "device uses unsupported power ops\n");
		error = ENOTSUP;
	}

out:

	/* Wakeup anyone waiting for the firmware load to complete */
	if (error) {
		fw->fw_status = -error;
	} else {
		fw->fw_status = FW_STATUS_DONE;
	}
	wakeup(&sc->sc_fw);

#if 0
	/* Free the firmware image if there was a problem */
	if (error) {
		iwa_free(fw->fw_rawdata, fw->fw_rawsize);
		fw->fw_rawdata = NULL;
	}
#endif
	return error;
}


/*
 * Firmware loading gunk.  This is kind of a weird hybrid between the
 * old iwn driver and the Linux iwlwifi driver.
 *
 * This requires the IWA_LOCK to be held.
 */
static int
iwa_firmware_load_chunk(struct iwa_softc *sc, uint32_t dst_addr,
	const uint8_t *section, uint32_t byte_cnt)
{
	struct iwa_dma_info *dma = &sc->fw_dma;
	int error;

	IWA_LOCK_ASSERT(sc);

	/* Copy firmware section into pre-allocated DMA-safe memory. */
	memcpy(dma->vaddr, section, byte_cnt);
	bus_dmamap_sync(sc->sc_dmat, dma->map, BUS_DMASYNC_PREWRITE);

	if (!iwa_grab_nic_access(sc))
		return EBUSY;

	sc->sc_fw_chunk_done = false;

	IWA_REG_WRITE(sc, FH_TCSR_CHNL_TX_CONFIG_REG(FH_SRVC_CHNL),
	    FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_PAUSE);
	IWA_REG_WRITE(sc, FH_SRVC_CHNL_SRAM_ADDR_REG(FH_SRVC_CHNL), dst_addr);
	IWA_REG_WRITE(sc, FH_TFDIB_CTRL0_REG(FH_SRVC_CHNL),
	    dma->paddr & FH_MEM_TFDIB_DRAM_ADDR_LSB_MSK);
	IWA_REG_WRITE(sc, FH_TFDIB_CTRL1_REG(FH_SRVC_CHNL),
	    (iwl_get_dma_hi_addr(dma->paddr)
	      << FH_MEM_TFDIB_REG1_ADDR_BITSHIFT) | byte_cnt);
	IWA_REG_WRITE(sc, FH_TCSR_CHNL_TX_BUF_STS_REG(FH_SRVC_CHNL),
	    1 << FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_NUM |
	    1 << FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_IDX |
	    FH_TCSR_CHNL_TX_BUF_STS_REG_VAL_TFDB_VALID);
	IWA_REG_WRITE(sc, FH_TCSR_CHNL_TX_CONFIG_REG(FH_SRVC_CHNL),
	    FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_ENABLE    |
	    FH_TCSR_TX_CONFIG_REG_VAL_DMA_CREDIT_DISABLE |
	    FH_TCSR_TX_CONFIG_REG_VAL_CIRQ_HOST_ENDTFD);

	iwa_release_nic_access(sc);

	/* wait 1s for this segment to load */
	while (!sc->sc_fw_chunk_done)
		if ((error = msleep(&sc->sc_fw, &sc->sc_mtx, 0, "iwmfw", hz)) != 0)
			break;

        return error;
}

static int
iwa_load_firmware(struct iwa_softc *sc, enum iwl_ucode_type ucode_type)
{
	struct fw_sects *fws;
	int error, i, w;
	void *data;
	uint32_t dlen;
	uint32_t offset;

	sc->sc_uc.uc_intr = false;

	fws = &sc->sc_fw.fw_sects[ucode_type];
	for (i = 0; i < fws->fw_count; i++) {
		data = fws->fw_sect[i].fws_data;
		dlen = fws->fw_sect[i].fws_len;
		offset = fws->fw_sect[i].fws_devoff;
		IWA_DPRINTF(sc,
		    IWA_DEBUG_FIRMWARE,
		    "LOAD FIRMWARE type %d offset %u len %d\n",
		    ucode_type, offset, dlen);
		error = iwa_firmware_load_chunk(sc, offset, data, dlen);
		if (error) {
			device_printf(sc->sc_dev,
			    "iwa_firmware_load_chunk() returned error %02x\n",
			    error);
			return error;
		}
	}

	/* wait for the firmware to load */
	IWA_REG_WRITE(sc, CSR_RESET, 0);

	for (w = 0; !sc->sc_uc.uc_intr && w < 10; w++) {
		error = msleep(&sc->sc_uc, &sc->sc_mtx, 0, "iwmuc", hz/10);
	}

	return error;
}

/* iwlwifi: pcie/trans.c */
static int
iwa_start_fw(struct iwa_softc *sc, enum iwl_ucode_type ucode_type)
{
	int error;

	IWA_REG_WRITE(sc, CSR_INT, ~0);

	if ((error = iwa_nic_init(sc)) != 0) {
		device_printf(sc->sc_dev, "Unable to init nic\n");
		return error;
	}

	/* make sure rfkill handshake bits are cleared */
	IWA_REG_WRITE(sc, CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_SW_BIT_RFKILL);
	IWA_REG_WRITE(sc, CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_DRV_GP1_BIT_CMD_BLOCKED);

	/* clear (again), then enable host interrupts */
	IWA_REG_WRITE(sc, CSR_INT, ~0);
	iwa_enable_interrupts(sc);

	/* really make sure rfkill handshake bits are cleared */
	/* maybe we should write a few times more?  just to make sure */
	IWA_REG_WRITE(sc, CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_SW_BIT_RFKILL);
	IWA_REG_WRITE(sc, CSR_UCODE_DRV_GP1_CLR, CSR_UCODE_SW_BIT_RFKILL);

	/* Load the given image to the HW */
	return iwa_load_firmware(sc, ucode_type);
}

static int
iwa_fw_alive(struct iwa_softc *sc, uint32_t sched_base)
{

	return iwa_post_alive(sc);
}

#if 0
static int
iwa_send_tx_ant_cfg(struct iwa_softc *sc, uint8_t valid_tx_ant)
{
	struct iwa_tx_ant_cfg_cmd tx_ant_cmd = {
		.valid = htole32(valid_tx_ant),
	};

	return iwa_mvm_send_cmd_pdu(sc, TX_ANT_CONFIGURATION_CMD, CMD_SYNC,
	    sizeof(tx_ant_cmd), &tx_ant_cmd);
}

/* iwlwifi: mvm/fw.c */
static int
iwa_send_phy_cfg_cmd(struct iwa_softc *sc)
{
	struct iwa_phy_cfg_cmd phy_cfg_cmd;
	enum iwl_ucode_type ucode_type = sc->sc_uc_current;

	/* Set parameters */
	phy_cfg_cmd.phy_cfg = htole32(sc->sc_fw_phy_config);
	phy_cfg_cmd.calib_control.event_trigger =
	    sc->sc_default_calib[ucode_type].event_trigger;
	phy_cfg_cmd.calib_control.flow_trigger =
	    sc->sc_default_calib[ucode_type].flow_trigger;

	DPRINTFN(10, ("Sending Phy CFG command: 0x%x\n", phy_cfg_cmd.phy_cfg));
	return iwa_mvm_send_cmd_pdu(sc, PHY_CONFIGURATION_CMD, CMD_SYNC,
	    sizeof(phy_cfg_cmd), &phy_cfg_cmd);
}
#endif

/*
 * Called to load in the firmware and bring the NIC up.
 *
 * The firmware must aleady have been loaded.
 */
int
iwa_mvm_load_ucode_wait_alive(struct iwa_softc *sc,
	enum iwl_ucode_type ucode_type)
{
	enum iwl_ucode_type old_type = sc->sc_uc_current;
	int error;

	IWA_LOCK_ASSERT(sc);

	/*
	 * The firmware has been loaded - iwa_read_firmware()
	 * parses out the TLV structures.
	 */
	if ((error = iwa_read_firmware(sc)) != 0) {
		device_printf(sc->sc_dev, "%s: iwa_read_firmware failed: %d\n",
		    __func__,
		    error);
		return error;
	}

	sc->sc_uc_current = ucode_type;
        error = iwa_start_fw(sc, ucode_type);
	if (error) {
		device_printf(sc->sc_dev, "%s: iwa_start_fw failed: %d\n",
		    __func__,
		    error);
		sc->sc_uc_current = old_type;
		return error;
	}

	return iwa_fw_alive(sc, sc->sched_base);
}
