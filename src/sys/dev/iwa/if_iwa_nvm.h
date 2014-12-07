#ifndef	__IF_IWA_NVM_H__
#define	__IF_IWA_NVM_H__

struct iwa_softc;


struct iwa_nvm_data {
	int n_hw_addrs;
	uint8_t hw_addr[ETHER_ADDR_LEN];

	uint8_t calib_version;
	uint16_t calib_voltage;

	uint16_t raw_temperature;
	uint16_t kelvin_temperature;
	uint16_t kelvin_voltage;
	uint16_t xtal_calib[2];

	bool sku_cap_band_24GHz_enable;
	bool sku_cap_band_52GHz_enable;
	bool sku_cap_11n_enable;
	bool sku_cap_amt_enable;
	bool sku_cap_ipan_enable;

	uint8_t radio_cfg_type;
	uint8_t radio_cfg_step;
	uint8_t radio_cfg_dash;
	uint8_t radio_cfg_pnum;
	uint8_t valid_tx_ant, valid_rx_ant;

	uint16_t nvm_version;
	uint8_t max_tx_pwr_half_dbm;
};

extern	int iwa_nvm_init(struct iwa_softc *sc);

#endif	/* __IF_IWA_NVM_H__ */
