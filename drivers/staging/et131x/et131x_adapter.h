/*
 * Agere Systems Inc.
 * 10/100/1000 Base-T Ethernet Driver for the ET1301 and ET131x series MACs
 *
 * Copyright ? 2005 Agere Systems Inc.
 * All rights reserved.
 *   http://www.agere.com
 *
 *------------------------------------------------------------------------------
 *
 * et131x_adapter.h - Header which includes the private adapter structure, along
 *                    with related support structures, macros, definitions, etc.
 *
 *------------------------------------------------------------------------------
 *
 * SOFTWARE LICENSE
 *
 * This software is provided subject to the following terms and conditions,
 * which you should read carefully before using the software.  Using this
 * software indicates your acceptance of these terms and conditions.  If you do
 * not agree with these terms and conditions, do not use the software.
 *
 * Copyright ? 2005 Agere Systems Inc.
 * All rights reserved.
 *
 * Redistribution and use in source or binary forms, with or without
 * modifications, are permitted provided that the following conditions are met:
 *
 * . Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following Disclaimer as comments in the code as
 *    well as in the documentation and/or other materials provided with the
 *    distribution.
 *
 * . Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following Disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * . Neither the name of Agere Systems Inc. nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Disclaimer
 *
 * THIS SOFTWARE IS PROVIDED ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, INFRINGEMENT AND THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  ANY
 * USE, MODIFICATION OR DISTRIBUTION OF THIS SOFTWARE IS SOLELY AT THE USERS OWN
 * RISK. IN NO EVENT SHALL AGERE SYSTEMS INC. OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, INCLUDING, BUT NOT LIMITED TO, CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#ifndef __ET131X_ADAPTER_H__
#define __ET131X_ADAPTER_H__

#include "et1310_address_map.h"
#include "et1310_tx.h"
#include "et1310_rx.h"

/*
 * Do not change these values: if changed, then change also in respective
 * TXdma and Rxdma engines
 */
#define NUM_DESC_PER_RING_TX         512	// TX Do not change these values
#define NUM_TCB                      64

/*
 * These values are all superseded by registry entries to facilitate tuning.
 * Once the desired performance has been achieved, the optimal registry values
 * should be re-populated to these #defines:
 */
#define NUM_TRAFFIC_CLASSES          1

/*
 * There are three ways of counting errors - if there are more than X errors
 * in Y packets (represented by the "SAMPLE" macros), if there are more than
 * N errors in a S mSec time period (the "PERIOD" macros), or if there are
 * consecutive packets with errors (CONSEC_ERRORED_THRESH).  This last covers
 * for "Bursty" errors, and the errored packets may well not be contiguous,
 * but several errors where the packet counter has changed by less than a
 * small amount will cause this count to increment.
 */
#define TX_PACKETS_IN_SAMPLE        10000
#define TX_MAX_ERRORS_IN_SAMPLE     50

#define TX_ERROR_PERIOD             1000
#define TX_MAX_ERRORS_IN_PERIOD     10

#define LINK_DETECTION_TIMER        5000

#define TX_CONSEC_RANGE             5
#define TX_CONSEC_ERRORED_THRESH    10

#define LO_MARK_PERCENT_FOR_PSR     15
#define LO_MARK_PERCENT_FOR_RX      15

/* Macros for flag and ref count operations        */
#define MP_SET_FLAG(_M, _F)         ((_M)->Flags |= (_F))
#define MP_CLEAR_FLAG(_M, _F)       ((_M)->Flags &= ~(_F))
#define MP_CLEAR_FLAGS(_M)          ((_M)->Flags = 0)
#define MP_TEST_FLAG(_M, _F)        (((_M)->Flags & (_F)) != 0)
#define MP_TEST_FLAGS(_M, _F)       (((_M)->Flags & (_F)) == (_F))
#define MP_IS_FLAG_CLEAR(_M, _F)    (((_M)->Flags & (_F)) == 0)

#define MP_INC_RCV_REF(_A)          atomic_inc(&(_A)->RcvRefCount)
#define MP_DEC_RCV_REF(_A)          atomic_dec(&(_A)->RcvRefCount)
#define MP_GET_RCV_REF(_A)          atomic_read(&(_A)->RcvRefCount)

/* Macros specific to the private adapter structure */
#define MP_TCB_RESOURCES_AVAILABLE(_M) ((_M)->TxRing.nBusySend < NUM_TCB)
#define MP_TCB_RESOURCES_NOT_AVAILABLE(_M) ((_M)->TxRing.nBusySend >= NUM_TCB)

#define MP_SHOULD_FAIL_SEND(_M)   ((_M)->Flags & fMP_ADAPTER_FAIL_SEND_MASK)
#define MP_IS_NOT_READY(_M)       ((_M)->Flags & fMP_ADAPTER_NOT_READY_MASK)
#define MP_IS_READY(_M)           !((_M)->Flags & fMP_ADAPTER_NOT_READY_MASK)

#define MP_HAS_CABLE(_M)           !((_M)->Flags & fMP_ADAPTER_NO_CABLE)
#define MP_LINK_DETECTED(_M)       !((_M)->Flags & fMP_ADAPTER_LINK_DETECTION)

/* Counters for error rate monitoring */
typedef struct _MP_ERR_COUNTERS {
	u32 PktCountTxPackets;
	u32 PktCountTxErrors;
	u32 TimerBasedTxErrors;
	u32 PktCountLastError;
	u32 ErredConsecPackets;
} MP_ERR_COUNTERS, *PMP_ERR_COUNTERS;

/* RFD (Receive Frame Descriptor) */
typedef struct _MP_RFD {
	struct list_head list_node;
	struct sk_buff *Packet;
	u32 PacketSize;	// total size of receive frame
	u16 iBufferIndex;
	u8 iRingIndex;
} MP_RFD, *PMP_RFD;

/* Enum for Flow Control */
typedef enum _eflow_control_t {
	Both = 0,
	TxOnly = 1,
	RxOnly = 2,
	None = 3
} eFLOW_CONTROL_t, *PeFLOW_CONTROL_t;

/* Struct to define some device statistics */
typedef struct _ce_stats_t {
	/* Link Input/Output stats */
	uint64_t ipackets;	// # of in packets
	uint64_t opackets;	// # of out packets

	/* MIB II variables
	 *
	 * NOTE: atomic_t types are only guaranteed to store 24-bits; if we
	 * MUST have 32, then we'll need another way to perform atomic
	 * operations
	 */
	u32 unircv;	// # multicast packets received
	atomic_t unixmt;	// # multicast packets for Tx
	u32 multircv;	// # multicast packets received
	atomic_t multixmt;	// # multicast packets for Tx
	u32 brdcstrcv;	// # broadcast packets received
	atomic_t brdcstxmt;	// # broadcast packets for Tx
	u32 norcvbuf;	// # Rx packets discarded
	u32 noxmtbuf;	// # Tx packets discarded

	/* Transciever state informations. */
	u8 xcvr_addr;
	u32 xcvr_id;

	/* Tx Statistics. */
	u32 tx_uflo;		// Tx Underruns

	u32 collisions;
	u32 excessive_collisions;
	u32 first_collision;
	u32 late_collisions;
	u32 max_pkt_error;
	u32 tx_deferred;

	/* Rx Statistics. */
	u32 rx_ov_flow;	// Rx Over Flow

	u32 length_err;
	u32 alignment_err;
	u32 crc_err;
	u32 code_violations;
	u32 other_errors;

#ifdef CONFIG_ET131X_DEBUG
	u32 UnhandledInterruptsPerSec;
	u32 RxDmaInterruptsPerSec;
	u32 TxDmaInterruptsPerSec;
	u32 WatchDogInterruptsPerSec;
#endif	/* CONFIG_ET131X_DEBUG */

	u32 SynchrounousIterations;
	INTERRUPT_t InterruptStatus;
} CE_STATS_t, *PCE_STATS_t;

/* The private adapter structure */
struct et131x_adapter {
	struct net_device *netdev;
	struct pci_dev *pdev;

	struct work_struct task;

	/* Flags that indicate current state of the adapter */
	u32 Flags;
	u32 HwErrCount;

	/* Configuration  */
	u8 PermanentAddress[ETH_ALEN];
	u8 CurrentAddress[ETH_ALEN];
	bool bOverrideAddress;
	bool bEepromPresent;
	u8 eepromData[2];

	/* Spinlocks */
	spinlock_t Lock;

	spinlock_t TCBSendQLock;
	spinlock_t TCBReadyQLock;
	spinlock_t SendHWLock;
	spinlock_t SendWaitLock;

	spinlock_t RcvLock;
	spinlock_t RcvPendLock;
	spinlock_t FbrLock;

	spinlock_t PHYLock;

	/* Packet Filter and look ahead size */
	u32 PacketFilter;
	u32 ulLookAhead;
	u32 uiLinkSpeed;
	u32 uiDuplexMode;
	u32 uiAutoNegStatus;
	u8 ucLinkStatus;

	/* multicast list */
	u32 MCAddressCount;
	u8 MCList[NIC_MAX_MCAST_LIST][ETH_ALEN];

	/* MAC test */
	TXMAC_TXTEST_t TxMacTest;

	/* Pointer to the device's PCI register space */
	ADDRESS_MAP_t __iomem *CSRAddress;

	/* PCI config space info, for debug purposes only. */
	u8 RevisionID;
	u16 VendorID;
	u16 DeviceID;
	u16 SubVendorID;
	u16 SubSystemID;
	u32 CacheFillSize;
	u16 PciXDevCtl;
	u8 pci_lat_timer;
	u8 pci_hdr_type;
	u8 pci_bist;
	u32 pci_cfg_state[64 / sizeof(u32)];

	/* Registry parameters */
	u8 SpeedDuplex;		// speed/duplex
	eFLOW_CONTROL_t RegistryFlowControl;	// for 802.3x flow control
	u8 RegistryWOLMatch;	// Enable WOL pattern-matching
	u8 RegistryWOLLink;	// Link state change is independant
	u8 RegistryPhyComa;	// Phy Coma mode enable/disable

	u32 RegistryRxMemEnd;	// Size of internal rx memory
	u8 RegistryMACStat;	// If set, read MACSTAT, else don't
	u32 RegistryVlanTag;	// 802.1q Vlan TAG
	u32 RegistryJumboPacket;	// Max supported ethernet packet size

	u32 RegistryTxNumBuffers;
	u32 RegistryTxTimeInterval;

	u32 RegistryRxNumBuffers;
	u32 RegistryRxTimeInterval;

	/* Validation helpers */
	u8 RegistryPMWOL;
	u8 RegistryNMIDisable;
	u32 RegistryDMACache;
	u32 RegistrySCGain;
	u8 RegistryPhyLoopbk;	// Enable Phy loopback

	/* Derived from the registry: */
	u8 AiForceDpx;		// duplex setting
	u16 AiForceSpeed;		// 'Speed', user over-ride of line speed
	eFLOW_CONTROL_t FlowControl;	// flow control validated by the far-end
	enum {
		NETIF_STATUS_INVALID = 0,
		NETIF_STATUS_MEDIA_CONNECT,
		NETIF_STATUS_MEDIA_DISCONNECT,
		NETIF_STATUS_MAX
	} MediaState;
	u8 DriverNoPhyAccess;

	/* Minimize init-time */
	bool bQueryPending;
	bool bSetPending;
	bool bResetPending;
	struct timer_list ErrorTimer;
	bool bLinkTimerActive;
	MP_POWER_MGMT PoMgmt;
	INTERRUPT_t CachedMaskValue;

	atomic_t RcvRefCount;	// Num packets not yet returned

	/* Xcvr status at last poll */
	MI_BMSR_t Bmsr;

	/* Tx Memory Variables */
	TX_RING_t TxRing;

	/* Rx Memory Variables */
	RX_RING_t RxRing;

	/* ET1310 register Access */
	JAGCORE_ACCESS_REGS JagCoreRegs;
	PCI_CFG_SPACE_REGS PciCfgRegs;

	/* Loopback specifics */
	u8 ReplicaPhyLoopbk;	// Replica Enable
	u8 ReplicaPhyLoopbkPF;	// Replica Enable Pass/Fail

	/* Stats */
	CE_STATS_t Stats;

	struct net_device_stats net_stats;
	struct net_device_stats net_stats_prev;
};

#define MPSendPacketsHandler  MPSendPackets
#define MP_FREE_SEND_PACKET_FUN(Adapter, pMpTcb) \
	et131x_free_send_packet(Adapter, pMpTcb)
#define MpSendPacketFun(Adapter, Packet) MpSendPacket(Adapter, Packet)

#endif /* __ET131X_ADAPTER_H__ */
