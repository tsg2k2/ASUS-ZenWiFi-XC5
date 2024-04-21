#include <common.h>

/*====move this to a .h file===============================================*/
#define SYSPORT_NUM_RX_PKT_DESC_LOG2  (7)

#define SYSPORT_NUM_TX_PKT_DESC_LOG2  (0)

#define SYSPORT_PKT_LEN_LOG2 (11)

/* Number of RX DMA buffers */
#define NUM_RX_DMA_BUFFERS		(1<<SYSPORT_NUM_RX_PKT_DESC_LOG2)

/* Number of RX DMA buffers */
#define NUM_TX_DMA_BUFFERS		(1<<SYSPORT_NUM_TX_PKT_DESC_LOG2)

/* Total number of DMA buffers */
#define NUM_DMA_BUFFERS			(NUM_RX_DMA_BUFFERS + NUM_TX_DMA_BUFFERS)

/* RX/TX DMA buffer size */
#define NET_DMA_BUFSIZE			2048

/* DMA buffer size for SP_RDMA_BSRS_REG register (2048 bytes) */
#define NET_DMA_SHIFT			11

#define ENET_ZLEN                               60

typedef struct PktDesc {
    uint32_t address ;
    uint32_t address_hi :8;
    uint32_t status :10;
    uint32_t length :14;
}PktDesc;

/*===================================================*/

typedef struct sys_port_topctrl {
	uint32_t SYSTEMPORT_TOPCTRL_REV_CNTL;
	uint32_t SYSTEMPORT_TOPCTRL_RX_FLUSH_CNTL;
	uint32_t SYSTEMPORT_TOPCTRL_TX_FLUSH_CNTL;
	uint32_t SYSTEMPORT_TOPCTRL_MISC_CNTL;
}sys_port_topctrl, SYSTEMPORT_TOPCTRL;

typedef struct sys_port_mib
{
    uint32_t Pkts64Octets;            /* SYSTEMPORT_UMAC_GR64   */
    uint32_t Pkts65to127Octets;       /* SYSTEMPORT_UMAC_GR127   */
    uint32_t Pkts128to255Octets;      /* SYSTEMPORT_UMAC_GR255   */
    uint32_t Pkts256to511Octets;      /* SYSTEMPORT_UMAC_GR511   */
    uint32_t Pkts512to1023Octets;     /* SYSTEMPORT_UMAC_GR1023   */
    uint32_t Pkts1024to1518Octets;    /* SYSTEMPORT_UMAC_GR1518   */
    uint32_t Pkts1519to1522;          /* SYSTEMPORT_UMAC_GRMGV   */
    uint32_t Pkts1523to2047;          /* SYSTEMPORT_UMAC_GR2047   */
    uint32_t Pkts2048to4095;          /* SYSTEMPORT_UMAC_GR4095   */
    uint32_t Pkts4096to8191;          /* SYSTEMPORT_UMAC_GR9216   */
    uint32_t RxPkts;                  /* SYSTEMPORT_UMAC_GRPKT   */
    uint32_t RxOctetsLo;              /* SYSTEMPORT_UMAC_GRBYT   */
    uint32_t RxMulticastPkts;         /* SYSTEMPORT_UMAC_GRMCA   */
    uint32_t RxBroadcastPkts;         /* SYSTEMPORT_UMAC_GRBCA   */
    uint32_t RxFCSErrs;               /* SYSTEMPORT_UMAC_GRFCS   */
    uint32_t RxCtrlFrame;             /* SYSTEMPORT_UMAC_GRXCF   */
    uint32_t RxPausePkts;             /* SYSTEMPORT_UMAC_GRXPF   */
    uint32_t RxUnknown;               /* SYSTEMPORT_UMAC_GRXUO   */
    uint32_t RxAlignErrs;             /* SYSTEMPORT_UMAC_GRALN   */
    uint32_t RxExcessSizeDisc;        /* SYSTEMPORT_UMAC_GRFLR   */
    uint32_t RxSymbolError;           /* SYSTEMPORT_UMAC_GRCDE   */
    uint32_t RxCarrierSenseErrs;      /* SYSTEMPORT_UMAC_GRFCR   */
    uint32_t RxOversizePkts;          /* SYSTEMPORT_UMAC_GROVR   */
    uint32_t RxJabbers;               /* SYSTEMPORT_UMAC_GRJBR   */
    uint32_t RxMtuErrs;               /* SYSTEMPORT_UMAC_GRMTUE   */
    uint32_t RxGoodPkts;              /* SYSTEMPORT_UMAC_GRPOK   */
    uint32_t RxUnicastPkts;           /* SYSTEMPORT_UMAC_GRUC   */
    uint32_t RxPPPPkts;               /* SYSTEMPORT_UMAC_GRPPP   */
    uint32_t RxCRCMatchPkts;          /* SYSTEMPORT_UMAC_GRCRC   */
    uint32_t dummy1[3];               /* uint8_t dummy6[12]   */
    uint32_t TxPkts64Octets;          /* SYSTEMPORT_UMAC_TR64   */
    uint32_t TxPkts65to127Octets;     /* SYSTEMPORT_UMAC_TR127   */
    uint32_t TxPkts128to255Octets;    /* SYSTEMPORT_UMAC_TR255   */
    uint32_t TxPkts256to511Octets;    /* SYSTEMPORT_UMAC_TR511   */
    uint32_t TxPkts512to1023Octets;   /* SYSTEMPORT_UMAC_TR1023   */
    uint32_t TxPkts1024to1518Octets;  /* SYSTEMPORT_UMAC_TR1518   */
    uint32_t TxPkts1519to1522;        /* SYSTEMPORT_UMAC_TRMGV   */
    uint32_t TxPkts1523to2047;        /* SYSTEMPORT_UMAC_TR2047   */
    uint32_t TxPkts2048to4095;        /* SYSTEMPORT_UMAC_TR4095   */
    uint32_t TxPkts4096to8191;        /* SYSTEMPORT_UMAC_TR9216   */
    uint32_t TxPkts;                  /* SYSTEMPORT_UMAC_GTPKT   */
    uint32_t TxMulticastPkts;         /* SYSTEMPORT_UMAC_GTMCA   */
    uint32_t TxBroadcastPkts;         /* SYSTEMPORT_UMAC_GTBCA   */
    uint32_t TxPausePkts;             /* SYSTEMPORT_UMAC_GTXPF   */
    uint32_t TxCtrlFrame;             /* SYSTEMPORT_UMAC_GTXCF   */
    uint32_t TxFCSErrs;               /* SYSTEMPORT_UMAC_GTFCS   */
    uint32_t TxOversizePkts;          /* SYSTEMPORT_UMAC_GTOVR   */
    uint32_t TxDeferredTx;            /* SYSTEMPORT_UMAC_GTDRF   */
    uint32_t TxExcessiveDef;          /* SYSTEMPORT_UMAC_GTEDF   */
    uint32_t TxSingleCol;             /* SYSTEMPORT_UMAC_GTSCL   */
    uint32_t TxMultipleCol;           /* SYSTEMPORT_UMAC_GTMCL   */
    uint32_t TxLateCol;               /* SYSTEMPORT_UMAC_GTLCL   */
    uint32_t TxExcessiveCol;          /* SYSTEMPORT_UMAC_GTXCL   */
    uint32_t TxFragments;             /* SYSTEMPORT_UMAC_GTFRG   */
    uint32_t TxCol;                   /* SYSTEMPORT_UMAC_GTNCL   */
    uint32_t TxJabber;                /* SYSTEMPORT_UMAC_GTJBR   */
    uint32_t TxOctetsLo;              /* SYSTEMPORT_UMAC_GTBYT   */
    uint32_t TxGoodPkts;              /* SYSTEMPORT_UMAC_GTPOK   */
    uint32_t TxUnicastPkts;           /* SYSTEMPORT_UMAC_GTUC   */
    uint32_t dummy2[3];               /* uint8_t dummy7[12]   */
    uint32_t RxRuntPkts;              /* SYSTEMPORT_UMAC_RRPKT   */
    uint32_t RxRuntValidFCSPkts;      /* SYSTEMPORT_UMAC_RRUND   */
    uint32_t RxRuntInvalidFCSPkts;    /* SYSTEMPORT_UMAC_RRFRG   */
    uint32_t RxRuntOctets;            /* SYSTEMPORT_UMAC_RRBYT   */

    /* No mapping for below counters */
    /* uint32_t RxUndersizePkts; */
    /* uint32_t RxFragments; */

}sys_port_mib;


#define FIELD_MASK(bits, shift)  ( ( (1ULL<<(bits)) - 1 ) << shift )

#define SYSPORT_UMAC_CMD_TX_ENA_M     FIELD_MASK(1,0)
#define SYSPORT_UMAC_CMD_RX_ENA_M     FIELD_MASK(1,1)

#define SYSPORT_UMAC_MPD_CTRL_MPD_EN  FIELD_MASK(1,0)

typedef struct sys_port_umac {
	uint32_t SYSTEMPORT_UMAC_UMAC_DUMMY;
	uint32_t SYSTEMPORT_UMAC_HD_BKP_CNTL;
	uint32_t SYSTEMPORT_UMAC_CMD;
	uint32_t SYSTEMPORT_UMAC_MAC0;
	uint32_t SYSTEMPORT_UMAC_MAC1;
	uint32_t SYSTEMPORT_UMAC_FRM_LEN;
	uint32_t SYSTEMPORT_UMAC_PAUSE_QUNAT;
	uint32_t dummy1[9];
	uint32_t SYSTEMPORT_UMAC_SFD_OFFSET;
	uint32_t SYSTEMPORT_UMAC_MODE;
	uint32_t SYSTEMPORT_UMAC_FRM_TAG0;
	uint32_t SYSTEMPORT_UMAC_FRM_TAG1;
	uint32_t dummy2[3];
	uint32_t SYSTEMPORT_UMAC_TX_IPG_LEN;
	uint32_t dummy3;
	uint32_t SYSTEMPORT_UMAC_EEE_CTRL;
	uint32_t SYSTEMPORT_UMAC_EEE_LPI_TIMER;
	uint32_t SYSTEMPORT_UMAC_EEE_WAKE_TIMER;
	uint32_t SYSTEMPORT_UMAC_EEE_REF_COUNT;
	uint32_t dummy4;
	uint32_t SYSTEMPORT_UMAC_RX_PKT_DROP_STATUS;
	uint32_t SYSTEMPORT_UMAC_SYMMETRIC_IDLE_THRESHOLD;
	uint32_t dummy5[164];
	uint32_t SYSTEMPORT_UMAC_MACSEC_PROG_TX_CRC;
	uint32_t SYSTEMPORT_UMAC_MACSEC_CNTRL;
	uint32_t SYSTEMPORT_UMAC_TS_STATUS_CNTRL;
	uint32_t SYSTEMPORT_UMAC_TX_TS_DATA;
	uint32_t dummy6[4];
	uint32_t SYSTEMPORT_UMAC_PAUSE_CNTRL;
	uint32_t SYSTEMPORT_UMAC_TXFIFO_FLUSH;
	uint32_t SYSTEMPORT_UMAC_RXFIFO_STAT;
	uint32_t SYSTEMPORT_UMAC_TXFIFO_STAT;
	uint32_t SYSTEMPORT_UMAC_PPP_CNTRL;
	uint32_t SYSTEMPORT_UMAC_PPP_REFRESH_CNTRL;
	uint32_t SYSTEMPORT_UMAC_TX_PAUSE_PREL0;
	uint32_t SYSTEMPORT_UMAC_TX_PAUSE_PREL1;
	uint32_t SYSTEMPORT_UMAC_TX_PAUSE_PREL2;
	uint32_t SYSTEMPORT_UMAC_TX_PAUSE_PREL3;
	uint32_t SYSTEMPORT_UMAC_RX_PAUSE_PREL0;
	uint32_t SYSTEMPORT_UMAC_RX_PAUSE_PREL1;
	uint32_t SYSTEMPORT_UMAC_RX_PAUSE_PREL2;
	uint32_t SYSTEMPORT_UMAC_RX_PAUSE_PREL3;
	uint32_t dummy7[38];
	sys_port_mib sp_mib;
	uint32_t dummy8[28];
	uint32_t SYSTEMPORT_UMAC_MIB_CNTRL;
	uint32_t dummy9[32];
	uint32_t SYSTEMPORT_UMAC_RXERR_MASK;
	uint32_t SYSTEMPORT_UMAC_RX_MAX_PKT_SIZE;
	uint32_t dummy10[5];
	uint32_t SYSTEMPORT_UMAC_MPD_CTRL;
	uint32_t SYSTEMPORT_UMAC_PSW_MS;
	uint32_t SYSTEMPORT_UMAC_PSW_LS;
	uint32_t dummy11[8];
	}sys_port_umac, SYSTEMPORT_UMAC;

#define SYSPORT_RDMA_CTRL_RDMA_EN_M             FIELD_MASK(1,0)
#define SYSPORT_RDMA_CTRL_RING_CFG_M            FIELD_MASK(1,1)
#define SYSPORT_RDMA_CTRL_DISCARD_EN_M          FIELD_MASK(1,2)
#define SYSPORT_RDMA_CTRL_DATA_OFFSET_M         FIELD_MASK(10,4)
#define SYSPORT_RDMA_CTRL_DDR_DESC_RD_EN_M      FIELD_MASK(1,14)
#define SYSPORT_RDMA_CTRL_DDR_DESC_WR_EN_M      FIELD_MASK(1,15)
#define SYSPORT_RDMA_CTRL_DDR_DESC_SWAP_M       FIELD_MASK(2,16)

#define SYSPORT_RDMA_BSRS_BUF_SIZE_LOG2_S       (0)
#define SYSPORT_RDMA_BSRS_BUF_SIZE_LOG2_M       FIELD_MASK(4,0)
#define SYSPORT_RDMA_BSRS_RING_SIZE_S           (16)
#define SYSPORT_RDMA_BSRS_RING_SIZE_M           FIELD_MASK(16,16)

#define SYSPORT_RDMA_STATUS_RDMA_DISABLED_M     FIELD_MASK(1,0)
#define SYSPORT_RDMA_STATUS_DESC_RAM_BUSY_M     FIELD_MASK(1,1)

#define SYSTEMPORT_RDMA_LOCRAM_DESCRING_SIZE_MAX  512

#define SYSPORT_RDMA_PRODUCER_INDEX_PROD_IDX_M  FIELD_MASK(16,0)
#define SYSPORT_RDMA_CONSUMER_INDEX_CONS_IDX_M  FIELD_MASK(16,0)
#define SYSTEMPORT_RDMA_DDR_DESC_RING_PUSH_TIMER_TIMEOUT_M  FIELD_MASK(16,0)

typedef struct sys_port_rdma {
	uint32_t SYSTEMPORT_RDMA_DESCRIPTOR_WORD[1024];
	uint32_t SYSTEMPORT_RDMA_CONTROL;
	uint32_t SYSTEMPORT_RDMA_STATUS;
	uint32_t SYSTEMPORT_RDMA_SYSBUS_BURST;
	uint32_t SYSTEMPORT_RDMA_BSRS;
	uint32_t SYSTEMPORT_RDMA_WRITE_POINTER_LOW;
	uint32_t SYSTEMPORT_RDMA_WRITE_POINTER_HIGH;
	uint32_t SYSTEMPORT_RDMA_PRODUCER_INDEX;
	uint32_t SYSTEMPORT_RDMA_CONSUMER_INDEX;
	uint32_t SYSTEMPORT_RDMA_START_ADDRESS_LOW;
	uint32_t SYSTEMPORT_RDMA_START_ADDRESS_HIGH;
	uint32_t SYSTEMPORT_RDMA_MULTIPLE_BUFFERS_DONE_INTERRUPT_THRESHOLD_PUSH_TIMER;
	uint32_t SYSTEMPORT_RDMA_XON_XOFF_THRESHOLD;
	uint32_t SYSTEMPORT_RDMA_READ_POINTER_LOW;
	uint32_t SYSTEMPORT_RDMA_READ_POINTER_HIGH;
	uint32_t SYSTEMPORT_RDMA_DESC_RAM_ARB_CONTROL;
	uint32_t SYSTEMPORT_RDMA_DESC_RAM_RD_ARB_CFG;
	uint32_t SYSTEMPORT_RDMA_DESC_RAM_WR_ARB_CFG;
	uint32_t SYSTEMPORT_RDMA_DDR_DESC_RING_START_LOW;
	uint32_t SYSTEMPORT_RDMA_DDR_DESC_RING_START_HIGH;
	uint32_t SYSTEMPORT_RDMA_DDR_DESC_RING_SIZE;
	uint32_t SYSTEMPORT_RDMA_DDR_DESC_RING_CTRL;
	uint32_t SYSTEMPORT_RDMA_DDR_DESC_RING_PUSH_TIMER;
	uint32_t SYSTEMPORT_RDMA_TEST;
	uint32_t SYSTEMPORT_RDMA_DEBUG;

}sys_port_rdma, SYSTEMPORT_RDMA;

#define SYSPORT_RBUF_CTRL_RSB_EN_M            FIELD_MASK(1,0)
#define SYSPORT_RBUF_CTRL_4B_ALIGN_M          FIELD_MASK(1,1)
#define SYSPORT_RBUF_CTRL_BTAG_STRIP_M        FIELD_MASK(1,2)
#define SYSPORT_RBUF_CTRL_BAD_PKT_DISCARD_M   FIELD_MASK(1,3)
#define SYSPORT_RBUF_CTRL_CRC_REPLACE_M       FIELD_MASK(1,20)
#define SYSPORT_RBUF_CTRL_RSB_SWAP_M          FIELD_MASK(2,22)
#define SYSPORT_RBUF_CTRL_RSB_SWAP_S          22

#define SYSPORT_RBUF_CTRL_RSB_SWAP_NONE       0
#define SYSPORT_RBUF_CTRL_RSB_SWAP_32         1
#define SYSPORT_RBUF_CTRL_RSB_SWAP_64         2
#define SYSPORT_RBUF_CTRL_RSB_SWAP_32_64      3

typedef struct sys_port_rbuf {
	uint32_t SYSTEMPORT_RBUF_RBUF_CONTROL;
	uint32_t SYSTEMPORT_RBUF_RBUF_PACKET_READY_THRESHOLD;
	uint32_t SYSTEMPORT_RBUF_RBUF_STATUS;
	uint32_t SYSTEMPORT_RBUF_RBUF_OVERFLOW_PACKET_DISCARD_COUNT;
	uint32_t SYSTEMPORT_RBUF_RBUF_ERROR_PACKET_COUNT;
}sys_port_rbuf, SYSTEMPORT_RBUF;


typedef struct sys_port_tbuf {
	uint32_t SYSTEMPORT_TBUF_TBUF_CONTROL;
	uint32_t SYSTEMPORT_TBUF_TBUF_STATUS;
}sys_port_tbuf, SYSTEMPORT_TBUF;

#define SYSTEMPORT_TDMA_DESC_RING_MAX        16
#define SYSTEMPORT_TDMA_LOCRAM_DESCRING_MAX  1024

#define SYSTEMPORT_TDMA_TIMEOUT_TICK_NSEC    4096 // For 250MHz clock: 1024 * 4ns

typedef struct systemport_tdma_descriptor_write_port
{
		uint32_t SYSTEMPORT_TDMA_DESCRIPTOR_XX_WRITE_PORT_LO;
		uint32_t SYSTEMPORT_TDMA_DESCRIPTOR_XX_WRITE_PORT_HI;
}systemport_tdma_descriptor_write_port;

typedef struct systemport_tdma_descriptor_read_port
{
		uint32_t SYSTEMPORT_TDMA_DESCRIPTOR_XX_READ_PORT_LO;
		uint32_t SYSTEMPORT_TDMA_DESCRIPTOR_XX_READ_PORT_HI;
}systemport_tdma_descriptor_read_port;

#define SYSPORT_TDMA_DESC_RING_XX_HEAD_TAIL_PTR_RING_EN_M                          FIELD_MASK(1,25)
#define SYSTEMPORT_TDMA_DESC_RING_XX_PRODUCER_CONSUMER_INDEX_CONSUMER_INDEX_S      16
#define SYSTEMPORT_TDMA_DESC_RING_XX_PRODUCER_CONSUMER_INDEX_CONSUMER_INDEX_M      FIELD_MASK(16,16)
#define SYSTEMPORT_TDMA_DESC_RING_XX_PRODUCER_CONSUMER_INDEX_PRODUCER_INDEX_M      FIELD_MASK(16,0)

#define SYSTEMPORT_TDMA_DESC_RING_XX_INTR_CONTROL_TIMEOUT_S                        16
#define SYSTEMPORT_TDMA_DESC_RING_XX_INTR_CONTROL_TIMEOUT_M                        FIELD_MASK(16,16)
#define SYSTEMPORT_TDMA_DESC_RING_XX_INTR_CONTROL_RING_EMPTY_INTR_EN_M             FIELD_MASK(1,15)
#define SYSTEMPORT_TDMA_DESC_RING_XX_INTR_CONTROL_INTR_THRESHOLD_S                 0
#define SYSTEMPORT_TDMA_DESC_RING_XX_INTR_CONTROL_INTR_THRESHOLD_M                 FIELD_MASK(15,0)

typedef struct SYSTEMPORT_TDMA_DESC
{
	uint32_t SYSTEMPORT_TDMA_DESC_RING_XX_HEAD_TAIL_PTR;
	uint32_t SYSTEMPORT_TDMA_DESC_RING_XX_COUNT;
	uint32_t SYSTEMPORT_TDMA_DESC_RING_XX_MAX_HYST_THRESHOLD;
	uint32_t SYSTEMPORT_TDMA_DESC_RING_XX_INTR_CONTROL;
	uint32_t SYSTEMPORT_TDMA_DESC_RING_XX_PRODUCER_CONSUMER_INDEX;
	uint32_t SYSTEMPORT_TDMA_DESC_RING_XX_MAPPING;
	uint32_t SYSTEMPORT_TDMA_DESC_RING_XX_PCP_DEI_VID;
	uint32_t SYSTEMPORT_TDMA_DDR_DESC_RING_XX_START_LOW;
	uint32_t SYSTEMPORT_TDMA_DDR_DESC_RING_XX_START_HIGH;
	uint32_t SYSTEMPORT_TDMA_DDR_DESC_RING_XX_SIZE;
	uint32_t SYSTEMPORT_TDMA_DDR_DESC_RING_XX_CTRL;
	uint32_t SYSTEMPORT_TDMA_DDR_DESC_RING_XX_PUSH_TIMER;
}systemport_tdma_desc;

#define SYSPORT_TDMA_CONTROL_TDMA_EN_M            FIELD_MASK(1,0)
#define SYSPORT_TDMA_CONTROL_TSB_EN_M             FIELD_MASK(1,1)
#define SYSPORT_TDMA_CONTROL_TSB_SWAP_M           FIELD_MASK(2,2)
#define SYSPORT_TDMA_CONTROL_ACB_ALGO_M           FIELD_MASK(1,4)
#define SYSPORT_TDMA_CONTROL_DATA_OFFSET_M        FIELD_MASK(10,5)
#define SYSPORT_TDMA_CONTROL_VLAN_EN_M            FIELD_MASK(1,15)
#define SYSPORT_TDMA_CONTROL_SW_BRCM_TAG_M        FIELD_MASK(1,16)
#define SYSPORT_TDMA_CONTROL_DDR_DESC_RING_EN_M   FIELD_MASK(1,19)
#define SYSPORT_TDMA_CONTROL_DDR_DESC_SWAP_M      FIELD_MASK(2,20)
#define SYSPORT_TDMA_CONTROL_DDR_DESC_SWAP_S      20
#define SYSPORT_TDMA_CONTROL_NO_ACB_M             FIELD_MASK(1,27)

#define SYSPORT_TDMA_CONTROL_DDR_DESC_SWAP_NONE   0
#define SYSPORT_TDMA_CONTROL_DDR_DESC_SWAP_32     1
#define SYSPORT_TDMA_CONTROL_DDR_DESC_SWAP_64     2
#define SYSPORT_TDMA_CONTROL_DDR_DESC_SWAP_32_64  3

#define SYSPORT_TDMA_STATUS_TDMA_DISABLED_M       FIELD_MASK(1,0)
#define SYSPORT_TDMA_STATUS_LL_RAM_INIT_BUSY_M    FIELD_MASK(1,1)

#define SYSTEMPORT_TDMA_DDR_DESC_RING_PUSH_TIMER_TIMEOUT_M  FIELD_MASK(16,0)

typedef struct sys_port_tdma {
	systemport_tdma_descriptor_write_port SYSTEMPORT_TDMA_DESCRIPTOR_WRITE_PORT[16];
	uint8_t dummy1[128];
	systemport_tdma_descriptor_read_port  SYSTEMPORT_TDMA_DESCRIPTOR_READ_PORT[16];
	uint8_t dummy2[128];
	uint32_t SYSTEMPORT_TDMA_DESCRIPTOR_XX_READ_PORT_CMD[16];
	uint8_t dummy3[64];
	systemport_tdma_desc SYSTEMPORT_TDMA_DESC[16];
	uint8_t dummy4[128];
	uint32_t SYSTEMPORT_TDMA_CONTROL;
	uint32_t SYSTEMPORT_TDMA_STATUS;
	uint32_t SYSTEMPORT_TDMA_SYSBUS_BURST;
	uint32_t SYSTEMPORT_TDMA_OVER_MAX_THRESHOLD_STATUS;
	uint32_t SYSTEMPORT_TDMA_OVER_HYST_THRESHOLD_STATUS;
	uint32_t SYSTEMPORT_TDMA_TPID;
	uint32_t SYSTEMPORT_TDMA_FREE_LIST_HEAD_TAIL_PTR;
	uint32_t SYSTEMPORT_TDMA_FREE_LIST_COUNT;
	uint32_t SYSTEMPORT_TDMA_TIER_2_ARBITER_CTRL;
	uint32_t SYSTEMPORT_TDMA_TIER_1_ARBITER_0_CTRL;
	uint32_t SYSTEMPORT_TDMA_TIER_1_ARBITER_0_QUEUE_ENABLE;
	uint32_t SYSTEMPORT_TDMA_TIER_1_ARBITER_1_CTRL;
	uint32_t SYSTEMPORT_TDMA_TIER_1_ARBITER_1_QUEUE_ENABLE;
	uint32_t SYSTEMPORT_TDMA_TIER_1_ARBITER_2_CTRL;
	uint32_t SYSTEMPORT_TDMA_TIER_1_ARBITER_2_QUEUE_ENABLE;
	uint32_t SYSTEMPORT_TDMA_TIER_1_ARBITER_3_CTRL;
	uint32_t SYSTEMPORT_TDMA_TIER_1_ARBITER_3_QUEUE_ENABLE;
	uint32_t SYSTEMPORT_TDMA_TEST;
	uint32_t SYSTEMPORT_TDMA_DEBUG;

}sys_port_tdma, SYSTEMPORT_TDMA;
