
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
#include <dev/iwa/iwl/iwl-trans.h>
#include <dev/iwa/iwl/iwl-csr.h>
#include <dev/iwa/iwl/iwl-fw.h>

#include <dev/iwa/iwl/mvm/fw-api.h>

#include <dev/iwa/if_iwa_firmware.h>
#include <dev/iwa/if_iwa_nvm.h>
#include <dev/iwa/if_iwa_trans.h>
#include <dev/iwa/if_iwavar.h>
#include <dev/iwa/if_iwareg.h>

#include <dev/iwa/if_iwa_fw_util.h>

/*
 * NVM read access and content parsing.  We do not support
 * external NVM or writing NVM.
 * iwlwifi/mvm/nvm.c
 */

/*
 * This is specific to the 7260 series stuff.
 * The 8xxx series stuff is in a different location.
 *
 * XXX TODO: use sc->cfg->nvm_hw_section_num instead.
 * XXX TODO: add 8xxx series stuff to this driver!
 */
#define	NVM_SECTION_TYPE_HW	0

/* list of NVM sections we are allowed/need to read */
static const int nvm_to_read[] = {
	NVM_SECTION_TYPE_HW,
	NVM_SECTION_TYPE_SW,
	NVM_SECTION_TYPE_CALIBRATION,
	NVM_SECTION_TYPE_PRODUCTION,
};

/* Default NVM size to read */
#define IWL_NVM_DEFAULT_CHUNK_SIZE (2*1024)
#define IWL_MAX_NVM_SECTION_SIZE 7000

#define NVM_WRITE_OPCODE 1
#define NVM_READ_OPCODE 0

static int
iwa_nvm_read_chunk(struct iwa_softc *sc, uint16_t section,
	uint16_t offset, uint16_t length, uint8_t *data, uint16_t *len)
{
	offset = 0;
	struct iwl_nvm_access_cmd nvm_access_cmd = {
		.offset = htole16(offset),
		.length = htole16(length),
		.type = htole16(section),
		.op_code = NVM_READ_OPCODE,
	};
	struct iwl_nvm_access_resp *nvm_resp;
	struct iwl_rx_packet *pkt;
	struct iwl_host_cmd cmd = {
		.id = NVM_ACCESS_CMD,
		.flags = CMD_WANT_SKB | CMD_SEND_IN_RFKILL,
		.data = { &nvm_access_cmd, },
	};
	int ret, bytes_read, offset_read;
	uint8_t *resp_data;

	device_printf(sc->sc_dev, "%s: section=%d; offset=%d; length=%d\n",
	    __func__,
	    (int) section,
	    (int) offset,
	    (int) length);

	cmd.len[0] = sizeof(struct iwl_nvm_access_cmd);

	/* Sync request */
	ret = iwa_send_cmd(sc, &cmd);
	if (ret) {
		device_printf(sc->sc_dev,
		    "%s: iwa_send_cmd failed; error=%d\n",
		    __func__,
		    ret);
		return ret;
	}

	pkt = cmd.resp_pkt;
	if (pkt->hdr.flags & IWL_CMD_FAILED_MSK) {
		device_printf(sc->sc_dev,
		    "Bad return from NVM_ACCES_COMMAND (0x%08X)\n",
		    pkt->hdr.flags);
		ret = EIO;
		goto exit;
	}

	/* Extract NVM response */
	nvm_resp = (void *)pkt->data;

	ret = le16toh(nvm_resp->status);
	bytes_read = le16toh(nvm_resp->length);
	offset_read = le16toh(nvm_resp->offset);
	device_printf(sc->sc_dev, "%s: called; ret=%d, bytes=%d, offset=%d\n",
	    __func__,
	    ret,
	    bytes_read,
	    offset_read);
	resp_data = nvm_resp->data;
	if (ret) {
		device_printf(sc->sc_dev,
		    "NVM access command failed with status %d\n", ret);
		ret = EINVAL;
		goto exit;
	}

	if (offset_read != offset) {
		device_printf(sc->sc_dev,
		    "NVM ACCESS response with invalid offset %d\n", offset_read);
		ret = EINVAL;
		goto exit;
	}

	memcpy(data + offset, resp_data, bytes_read);
	*len = bytes_read;

 exit:
	iwa_free_resp(sc, &cmd);
	return ret;
}

/*
 * Reads an NVM section completely.
 * NICs prior to 7000 family doesn't have a real NVM, but just read
 * section 0 which is the EEPROM. Because the EEPROM reading is unlimited
 * by uCode, we need to manually check in this case that we don't
 * overflow and try to read more than the EEPROM size.
 * For 7000 family NICs, we supply the maximal size we can read, and
 * the uCode fills the response with as much data as we can,
 * without overflowing, so no check is needed.
 */
static int
iwa_nvm_read_section(struct iwa_softc *sc,
	uint16_t section, uint8_t *data, uint16_t *len)
{
	uint16_t length, seglen;
	int error;

	/* Set nvm section read length */
	length = seglen = IWL_NVM_DEFAULT_CHUNK_SIZE;
	*len = 0;

	/* Read the NVM until exhausted (reading less than requested) */
	while (seglen == length) {
		error = iwa_nvm_read_chunk(sc,
		    section, *len, length, data, &seglen);
		if (error) {
			device_printf(sc->sc_dev,
			    "Cannot read NVM from section "
			    "%d offset %d, length %d\n",
			    section, *len, length);
			return (error);
		}
		*len += seglen;
	}

	IWA_DPRINTF(sc, IWA_DEBUG_NVRAM,
	    "NVM section %d read completed\n", section);
	return 0;
}

/*
 * BEGIN NVM_PARSE
 */

/* iwlwifi/iwl-nvm-parse.c */

/* NVM offsets (in words) definitions */
enum wkp_nvm_offsets {
	/* NVM HW-Section offset (in words) definitions */
	HW_ADDR = 0x15,

/* NVM SW-Section offset (in words) definitions */
	NVM_SW_SECTION = 0x1C0,
	NVM_VERSION = 0,
	RADIO_CFG = 1,
	SKU = 2,
	N_HW_ADDRS = 3,
	NVM_CHANNELS = 0x1E0 - NVM_SW_SECTION,

/* NVM calibration section offset (in words) definitions */
	NVM_CALIB_SECTION = 0x2B8,
	XTAL_CALIB = 0x316 - NVM_CALIB_SECTION
};

/* SKU Capabilities (actual values from NVM definition) */
enum nvm_sku_bits {
	NVM_SKU_CAP_BAND_24GHZ	= BIT(0),
	NVM_SKU_CAP_BAND_52GHZ	= BIT(1),
	NVM_SKU_CAP_11N_ENABLE	= BIT(2),
	NVM_SKU_CAP_11AC_ENABLE	= BIT(3),
};

/* radio config bits (actual values from NVM definition) */
#define NVM_RF_CFG_DASH_MSK(x)   (x & 0x3)         /* bits 0-1   */
#define NVM_RF_CFG_STEP_MSK(x)   ((x >> 2)  & 0x3) /* bits 2-3   */
#define NVM_RF_CFG_TYPE_MSK(x)   ((x >> 4)  & 0x3) /* bits 4-5   */
#define NVM_RF_CFG_PNUM_MSK(x)   ((x >> 6)  & 0x3) /* bits 6-7   */
#define NVM_RF_CFG_TX_ANT_MSK(x) ((x >> 8)  & 0xF) /* bits 8-11  */
#define NVM_RF_CFG_RX_ANT_MSK(x) ((x >> 12) & 0xF) /* bits 12-15 */

#define DEFAULT_MAX_TX_POWER 16

/**
 * enum iwl_nvm_channel_flags - channel flags in NVM
 * @NVM_CHANNEL_VALID: channel is usable for this SKU/geo
 * @NVM_CHANNEL_IBSS: usable as an IBSS channel
 * @NVM_CHANNEL_ACTIVE: active scanning allowed
 * @NVM_CHANNEL_RADAR: radar detection required
 * @NVM_CHANNEL_DFS: dynamic freq selection candidate
 * @NVM_CHANNEL_WIDE: 20 MHz channel okay (?)
 * @NVM_CHANNEL_40MHZ: 40 MHz channel okay (?)
 * @NVM_CHANNEL_80MHZ: 80 MHz channel okay (?)
 * @NVM_CHANNEL_160MHZ: 160 MHz channel okay (?)
 */
enum iwl_nvm_channel_flags {
	NVM_CHANNEL_VALID = BIT(0),
	NVM_CHANNEL_IBSS = BIT(1),
	NVM_CHANNEL_ACTIVE = BIT(3),
	NVM_CHANNEL_RADAR = BIT(4),
	NVM_CHANNEL_DFS = BIT(7),
	NVM_CHANNEL_WIDE = BIT(8),
	NVM_CHANNEL_40MHZ = BIT(9),
	NVM_CHANNEL_80MHZ = BIT(10),
	NVM_CHANNEL_160MHZ = BIT(11),
};

#define CHECK_AND_PRINT_I(x)	\
	((ch_flags & NVM_CHANNEL_##x) ? # x " " : "")

static const uint8_t iwa_nvm_channels[] = {
	/* 2.4GHz */
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,

	/* 5GHz */
	36, 40, 44, 48, 52, 56, 60, 64,
	100, 104, 108, 112, 116, 120,
	124, 128, 132, 136, 140, 144,

	149, 153, 157, 161, 165,
};

#define	NUM_2GHZ_CHANNELS	14

static uint32_t
iwa_eeprom_channel_flags(uint16_t ch_flags)
{
	uint32_t nflags = 0;

	if ((ch_flags & NVM_CHANNEL_ACTIVE) == 0)
		nflags |= IEEE80211_CHAN_PASSIVE;
	if ((ch_flags & NVM_CHANNEL_IBSS) == 0)
		nflags |= IEEE80211_CHAN_NOADHOC;
	if (ch_flags & NVM_CHANNEL_DFS) {
		nflags |= IEEE80211_CHAN_DFS;
		nflags |= IEEE80211_CHAN_NOADHOC;
	}

	return (nflags);
}

static void
iwa_init_print_channel_flags(uint16_t ch_flags)
{
	if (ch_flags & NVM_CHANNEL_VALID)
		printf("(valid)");
	if (ch_flags & NVM_CHANNEL_IBSS)
		printf("(ibss)");
	if (ch_flags & NVM_CHANNEL_ACTIVE)
		printf("(active)");
	if (ch_flags & NVM_CHANNEL_RADAR)
		printf("(radar)");
	if (ch_flags & NVM_CHANNEL_DFS)
		printf("(dfs)");
	if (ch_flags & NVM_CHANNEL_WIDE)
		printf("(wide)");
	if (ch_flags & NVM_CHANNEL_40MHZ)
		printf("(40mhz)");
	if (ch_flags & NVM_CHANNEL_80MHZ)
		printf("(80mhz)");
	if (ch_flags & NVM_CHANNEL_160MHZ)
		printf("(160mhz)");
}

static void
iwa_init_channel_map(struct iwa_softc *sc, const uint16_t * const nvm_ch_flags)
{
	device_printf(sc->sc_dev, "%s: TODO\n", __func__);
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211_channel *c;
	struct iwa_nvm_data *data = &sc->sc_nvm;
	uint16_t ch_flags;
	uint32_t nflags;
	bool is_5ghz;
	int ch_idx;

	/* Initial setup - legacy, HT20 */
	for (ch_idx = 0; ch_idx < nitems(iwa_nvm_channels); ch_idx++) {
		ch_flags = le16_to_cpup(nvm_ch_flags + ch_idx);

		if (! (ch_flags & NVM_CHANNEL_VALID))
			continue;
		if (ch_idx >= NUM_2GHZ_CHANNELS && (! data->sku_cap_band_52GHz_enable))
			continue;

		device_printf(sc->sc_dev, "%s: [%d]: ch %d, flags 0x%04x ",
		    __func__,
		    ch_idx,
		    iwa_nvm_channels[ch_idx],
		    ch_flags);
		iwa_init_print_channel_flags(ch_flags);
		printf("\n");

		is_5ghz = !! (ch_idx >= NUM_2GHZ_CHANNELS);

		/* Grab net80211 flags */
		nflags = iwa_eeprom_channel_flags(ch_flags);

		/* Allocate an ieee80211_channel entry */

		/* Setup! */
		c = &ic->ic_channels[ic->ic_nchans++];

		c->ic_ieee = iwa_nvm_channels[ch_idx];
		/* XXX maxregpower */
		/* XXX maxpower */
		
		if (is_5ghz) {
			/* 5GHz */
			c->ic_freq = ieee80211_ieee2mhz(iwa_nvm_channels[ch_idx], IEEE80211_CHAN_A);
			c->ic_flags = IEEE80211_CHAN_A | nflags;
		} else {
			/* 2GHz */
			c->ic_freq = ieee80211_ieee2mhz(iwa_nvm_channels[ch_idx], IEEE80211_CHAN_G);
			c->ic_flags = IEEE80211_CHAN_B | nflags;

			/* Channel 13 - 11b only */
			if (iwa_nvm_channels[ch_idx] != 13) {
				c = &ic->ic_channels[ic->ic_nchans++];
				c[0] = c[-1];
				c->ic_flags = IEEE80211_CHAN_G | nflags;
			}
		}

		/* HT20 - create duplicate channel; set HT20 */
		/* XXX TODO */

	}

	/* Next - add HT40 flags as appropriate */
	/* XXX TODO */
}

void
iwa_eeprom_init_channel_map(struct iwa_softc *sc)
{
	const uint16_t *sw;

	sw = (const uint16_t *)sc->sc_nvm_sections[NVM_SECTION_TYPE_SW].data;

	iwa_init_channel_map(sc, &sw[NVM_CHANNELS]);
}

static int
iwa_parse_nvm_data(struct iwa_softc *sc,
	const uint16_t *nvm_hw, const uint16_t *nvm_sw,
	const uint16_t *nvm_calib, uint8_t tx_chains, uint8_t rx_chains)
{
	struct iwa_nvm_data *data = &sc->sc_nvm;
	uint8_t hw_addr[ETHER_ADDR_LEN];
	uint16_t radio_cfg, sku;

	data->nvm_version = le16_to_cpup(nvm_sw + NVM_VERSION);

	radio_cfg = le16_to_cpup(nvm_sw + RADIO_CFG);
	data->radio_cfg_type = NVM_RF_CFG_TYPE_MSK(radio_cfg);
	data->radio_cfg_step = NVM_RF_CFG_STEP_MSK(radio_cfg);
	data->radio_cfg_dash = NVM_RF_CFG_DASH_MSK(radio_cfg);
	data->radio_cfg_pnum = NVM_RF_CFG_PNUM_MSK(radio_cfg);
	data->valid_tx_ant = NVM_RF_CFG_TX_ANT_MSK(radio_cfg);
	data->valid_rx_ant = NVM_RF_CFG_RX_ANT_MSK(radio_cfg);

	sku = le16_to_cpup(nvm_sw + SKU);
	data->sku_cap_band_24GHz_enable = sku & NVM_SKU_CAP_BAND_24GHZ;
	data->sku_cap_band_52GHz_enable = sku & NVM_SKU_CAP_BAND_52GHZ;
	data->sku_cap_11n_enable = 0;

	if (!data->valid_tx_ant || !data->valid_rx_ant) {
		device_printf(sc->sc_dev, "invalid antennas (0x%x, 0x%x)\n",
			    data->valid_tx_ant, data->valid_rx_ant);
		return EINVAL;
	}

	data->n_hw_addrs = le16_to_cpup(nvm_sw + N_HW_ADDRS);

	data->xtal_calib[0] = *(nvm_calib + XTAL_CALIB);
	data->xtal_calib[1] = *(nvm_calib + XTAL_CALIB + 1);

	/* The byte order is little endian 16 bit, meaning 214365 */
	memcpy(hw_addr, nvm_hw + HW_ADDR, ETHER_ADDR_LEN);
	data->hw_addr[0] = hw_addr[1];
	data->hw_addr[1] = hw_addr[0];
	data->hw_addr[2] = hw_addr[3];
	data->hw_addr[3] = hw_addr[2];
	data->hw_addr[4] = hw_addr[5];
	data->hw_addr[5] = hw_addr[4];

//	iwa_init_channel_map(sc, &nvm_sw[NVM_CHANNELS]);
	data->calib_version = 255;   /* TODO:
					this value will prevent some checks from
					failing, we need to check if this
					field is still needed, and if it does,
					where is it in the NVM*/

	return 0;
}

/*
 * END NVM PARSE
 */

#define IWM_FW_VALID_TX_ANT(sc) \
    ((sc->sc_fw_phy_config & FW_PHY_CFG_TX_CHAIN) >> FW_PHY_CFG_TX_CHAIN_POS)
#define IWM_FW_VALID_RX_ANT(sc) \
    ((sc->sc_fw_phy_config & FW_PHY_CFG_RX_CHAIN) >> FW_PHY_CFG_RX_CHAIN_POS)

static int
iwa_parse_nvm_sections(struct iwa_softc *sc, struct iwa_nvm_section *sections)
{
	const uint16_t *hw, *sw, *calib;

	/* Checking for required sections */
	if (!sections[NVM_SECTION_TYPE_SW].data ||
	    !sections[NVM_SECTION_TYPE_HW].data) {
		device_printf(sc->sc_dev,
		    "%s: Can't parse empty NVM sections\n", __func__);
		return ENOENT;
	}

	hw = (const uint16_t *)sections[NVM_SECTION_TYPE_HW].data;
	sw = (const uint16_t *)sections[NVM_SECTION_TYPE_SW].data;
	calib = (const uint16_t *)sections[NVM_SECTION_TYPE_CALIBRATION].data;
	return iwa_parse_nvm_data(sc, hw, sw, calib,
	    IWM_FW_VALID_TX_ANT(sc), IWM_FW_VALID_RX_ANT(sc));
}

/*
 * Initialise the NVRAM section - this involves sending commands
 * to wake the hardware up.
 *
 * For now, do this with the iwa lock held.
 */
int
iwa_nvm_init(struct iwa_softc *sc)
{
	int i, section, error;
	uint16_t len;
	uint8_t *nvm_buffer, *temp;

	IWA_LOCK_ASSERT(sc);

	/* Read From FW NVM */
	IWA_DPRINTF(sc, IWA_DEBUG_NVRAM, "%s: Read NVM\n", __func__);

	/* TODO: find correct NVM max size for a section */
	nvm_buffer = malloc(sc->sc_cfg->base_params->eeprom_size,
	    M_TEMP, M_NOWAIT);
	if (nvm_buffer == NULL) {
		device_printf(sc->sc_dev,
		    "%s: nvmbuffer malloc failed\n", __func__);
		error = ENOMEM;
		return (error);
	}
#define	N(a)	(sizeof(a)/sizeof(a[0]))
	for (i = 0; i < N(nvm_to_read); i++) {
		section = nvm_to_read[i];
		KASSERT(section <= N(sc->sc_nvm_sections), (""));

		error = iwa_nvm_read_section(sc, section, nvm_buffer, &len);
		if (error)
			break;

		temp = malloc(len, M_TEMP, M_NOWAIT);
		if (temp == NULL) {
			device_printf(sc->sc_dev,
			    "%s: nvram temp malloc failed\n",
			    __func__);
			error = ENOMEM;
			break;
		}
		memcpy(temp, nvm_buffer, len);
		sc->sc_nvm_sections[section].data = temp;
		sc->sc_nvm_sections[section].length = len;
	}
#undef	N
	free(nvm_buffer, M_TEMP);
	if (error)
		return error;

	/* XXX TODO: make sure we free the sections when we're done */
	return iwa_parse_nvm_sections(sc, sc->sc_nvm_sections);
}
