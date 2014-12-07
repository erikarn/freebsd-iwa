#ifndef	__IF_IWA_FW_UTIL_H__
#define	__IF_IWA_FW_UTIL_H__

extern	int iwa_send_cmd(struct iwa_softc *, struct iwl_host_cmd *);
extern	int iwa_mvm_send_cmd_pdu(struct iwa_softc *, uint8_t,
	    uint32_t, uint16_t, const void *);
extern	int iwa_mvm_send_cmd_status(struct iwa_softc *,
	    struct iwl_host_cmd *, uint32_t *);
extern	int iwa_mvm_send_cmd_pdu_status(struct iwa_softc *, uint8_t,
	    uint16_t, const void *, uint32_t *);
extern	void iwa_free_resp(struct iwa_softc *sc, struct iwl_host_cmd *hcmd);
extern	void iwa_cmd_done(struct iwa_softc *sc, struct iwl_rx_packet *pkt,
	    struct mbuf *m);

#endif	/* __IF_IWA_FW_UTIL_H__ */
