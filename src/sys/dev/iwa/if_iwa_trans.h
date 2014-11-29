#ifndef	__IF_IWA_TRANS_H__
#define	__IF_IWA_TRANS_H__

/*
 * This file defines the bus-facing primitives and functions.
 * It's very PCIe oriented at the moment, but there's apparently
 * some Intel SDIO wifi parts that will work with the same
 * driver code as the other 7xxx/8xxx chips.
 *
 * So for now let's try to keep as much of the bus specific
 * and transmit/receive/command ring management code here.
 */

struct iwa_dma_info {
        bus_dma_tag_t           tag;
        bus_dmamap_t            map;
        bus_dma_segment_t       seg;
        bus_addr_t              paddr;
        caddr_t                 vaddr;
        bus_size_t              size;
};

#define IWM_ICT_SIZE            4096
#define IWM_ICT_COUNT           (IWM_ICT_SIZE / sizeof (uint32_t))
#define IWM_ICT_PADDR_SHIFT     12

#define IWA_TX_RING_COUNT       256
#define IWA_TX_RING_LOMARK      192
#define IWA_TX_RING_HIMARK      224

struct iwa_tx_data {
        bus_dmamap_t    map;
        bus_addr_t      cmd_paddr;
        bus_addr_t      scratch_paddr;
        struct mbuf     *m;
        struct iwa_node *in;
        bool done;
};

struct iwa_tx_ring {
        struct iwa_dma_info     desc_dma;
        struct iwa_dma_info     cmd_dma;
        struct iwl_tfd          *desc;
        struct iwl_device_cmd   *cmd;
        struct iwa_tx_data      data[IWA_TX_RING_COUNT];
	bus_dma_tag_t           data_dmat;
        int                     qid;
        int                     queued;
        int                     cur;
};

#define IWA_RX_RING_COUNT       256
#define IWA_RBUF_COUNT          (IWA_RX_RING_COUNT + 32)
/* Linux driver optionally uses 8k buffer */
#define IWA_RBUF_SIZE           4096

struct iwa_softc;
struct iwa_rbuf {
        struct iwa_softc        *sc;
        void                    *vaddr;
        bus_addr_t              paddr;
};

struct iwa_rx_data {
        struct mbuf     *m;
        bus_dmamap_t    map;
        int             wantresp;
};

struct iwa_rx_ring {
        struct iwa_dma_info     desc_dma;
        struct iwa_dma_info     stat_dma;
        struct iwa_dma_info     buf_dma;
        uint32_t                *desc;
        struct iwl_rb_status    *stat;
        struct iwa_rx_data      data[IWA_RX_RING_COUNT];
	bus_dma_tag_t           data_dmat;
        int                     cur;
};

/*
 * External facing functions - should be trans ops methods!
 */
extern	void iwa_set_pwr(struct iwa_softc *sc);
extern	int iwa_prepare_card_hw(struct iwa_softc *sc);

extern	int iwa_alloc_fwmem(struct iwa_softc *sc);
extern	void iwa_free_fwmem(struct iwa_softc *sc);
extern	int iwa_alloc_sched(struct iwa_softc *sc);
extern	void iwa_free_sched(struct iwa_softc *sc);
extern	int iwa_alloc_kw(struct iwa_softc *sc);
extern	void iwa_free_kw(struct iwa_softc *sc);
extern	int iwa_alloc_ict(struct iwa_softc *sc);
extern	void iwa_free_ict(struct iwa_softc *sc);
extern	int iwa_rx_addbuf(struct iwa_softc *sc, struct iwa_rx_ring *ring,
	    size_t mbuf_size, int idx);
extern	int iwa_alloc_rx_ring(struct iwa_softc *sc, struct iwa_rx_ring *ring);
extern	void iwa_reset_rx_ring(struct iwa_softc *sc, struct iwa_rx_ring *ring);
extern	void iwa_free_rx_ring(struct iwa_softc *sc, struct iwa_rx_ring *ring);
extern	int iwa_alloc_tx_ring(struct iwa_softc *sc, struct iwa_tx_ring *ring,
	    int qid);
extern	void iwa_reset_tx_ring(struct iwa_softc *sc, struct iwa_tx_ring *ring);
extern	void iwa_free_tx_ring(struct iwa_softc *sc, struct iwa_tx_ring *ring);







#endif	/* __IF_IWA_TRANS_H__ */
