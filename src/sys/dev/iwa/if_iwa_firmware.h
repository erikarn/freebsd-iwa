#ifndef	__IF_IWA_FIRMWARE_H__
#define	__IF_IWA_FIRMWARE_H__


#define	IWM_UCODE_SECT_MAX	6
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

enum iwm_ucode_type {
        IWM_UCODE_TYPE_INIT,
        IWM_UCODE_TYPE_REGULAR,
        IWM_UCODE_TYPE_WOW,
        IWM_UCODE_TYPE_MAX
};

struct iwa_softc;

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
                } fw_sect[IWM_UCODE_SECT_MAX];
                size_t fw_totlen;
                int fw_count;
        } fw_sects[IWM_UCODE_TYPE_MAX];
};

extern	int if_iwa_firmware_load(struct iwa_softc *sc, struct iwa_fw_info *fw,
	    const char *fwname);

#endif	/* __IF_IWA_FIRMWARE_H__ */
