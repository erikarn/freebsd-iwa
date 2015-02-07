#ifndef	__IF_IWA_FIRMWARE_H__
#define	__IF_IWA_FIRMWARE_H__


#define	IWM_FWNAME		"iwa_fw_7260_7"
#define	IWM_FWDMASEGSZ		(192*1024)
/* sanity check value */
#define IWM_FWMAXSIZE		(2*1024*1024)

/*
 * fw_status is used to determine if we've already parsed the firmware file
 *
 * In addition to the following, status < 0 ==> -error
 */
#define FW_STATUS_NONE          0
#define FW_STATUS_INPROGRESS    1
#define FW_STATUS_DONE          2

struct iwa_softc;

struct iwa_ucode_status {
	uint32_t uc_error_event_table;
	uint32_t uc_log_event_table;
	bool uc_ok;
	bool uc_intr;
};

struct iwa_fw_info {
        void *fw_rawdata;
        size_t fw_rawsize;
        int fw_status;

        struct fw_sects {
                struct fw_onesect {
                        void *fws_data;
                        uint32_t fws_len;
                        uint32_t fws_devoff; 

                        void *fws_alloc;
                        size_t fws_allocsize;
                } fw_sect[IWL_UCODE_SECTION_MAX];
                size_t fw_totlen;
                int fw_count;
        } fw_sects[IWL_UCODE_TYPE_MAX];
};

extern	int iwa_find_firmware(struct iwa_softc *sc);
extern	int iwa_mvm_load_ucode_wait_alive(struct iwa_softc *sc,
	    enum iwl_ucode_type ucode_type);
extern	int iwa_send_tx_ant_cfg(struct iwa_softc *sc, uint8_t valid_tx_ant);
extern	int iwa_send_phy_cfg_cmd(struct iwa_softc *sc);
#endif	/* __IF_IWA_FIRMWARE_H__ */
