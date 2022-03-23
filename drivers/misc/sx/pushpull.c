/*
 * SpaceX FPGA Push-Pull driver.
 *
 * The Push and Pull IP cores are used to provide DMA interfaces for packet
 * communication in the Satellite network subsystem. While they are separate
 * cores, they are logically paired in the implementation to provide a
 * bi-directional channel.
 *
 * The "Push" refers to receive and "Pull" refers to transmit from the
 * perspective of software.
 *
 * These IP blocks have limited depth hardware FIFOs, so to provide better
 * performance we manage a larger pool of buffers and descriptors then the
 * FIFO depth supports.
 *
 * Organizationally, the driver is broken into four separate levels:
 * 1 - The chan.  This is the basic work structure of either a push
 *	or pull block, and contains the common functionality including most
 *	fifos, workqueues, interrupts, etc.
 * 2 - The pchan.  This is a superset of chan for either a push or a pull block.
 *	Functionality that is different between push and pull but common among
 *	all blocks of that type should live here.
 * 3 - The dchan.  This is a superset of either a pchan.  Any information that
 *	that is specific to the type of push/pull channel should live here.
 * 4 - The pushpull.  This is the aggregation of a push, pull, or both block and
 *	the character device that acts as the userspace interface.
 *
 * The IP specifications can be found here:
 *
 * @see https://stash/projects/SAT_RTL/repos/dma_push_pull/browse/docs/DMA_Push_Specification.docx
 * @see https://stash/projects/SAT_RTL/repos/dma_push_pull/browse/docs/DMA_Pull_Specification.docx
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/pushpull.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/workqueue.h>

/* Auto-generated register headers. */
#include "apb_dma_push.h"
#include "apb_dma_pull.h"

#define DRIVER_NAME "pushpull"
#define CDEV_COMPAT_ID "sx,apb-dma-push-pull-2.00a"
#define NET_COMPAT_ID "sx,apb-dma-push-pull-net-2.00a"
#define HYBRID_COMPAT_ID "sx,apb-dma-push-pull-hybrid-2.00a"

#define PUSHPULL_ENABLE_TIMING
#define    PUSHPULL_TIMING_BUCKET_CNT	 8
#define    PUSHPULL_TIMING_BUCKET_STRIDE 4

/*
 * MAX_PUSHPULL_MINORS - max number of push-pull character devices we can have
 *			 at once
 */
#define MAX_PUSHPULL_MINORS 256

#define PUSHPULL_DESC_ALIGNMENT 8 /* 8-byte alignment for descriptors */

/**
 * Interrupt fields.
 */
#define PUSH_RX_MAX_CHANNEL_COUNT 4
#define PUSH_RX_INTC0_ALMOST_FULL_BIT 11 /* work_fifo_almost_full[0] */
#define PUSH_RX_INTC0_CHAN_BIT 25 /* !work_fifo_empty[0] */
#define PUSH_RX_INTC1_MASK 0x0       /* Unused */
#define PULL_TX_INTC0_MASK BIT(9) /* free_fifo_almost_full */

/**
 * Error bits for struct eut_push_descriptor.dma_status and
 * cfe_push_descriptor.dma_status
 */
#define RX_DESC_DMA_STATUS_LARGE_PKT_ERR BIT(0)
#define RX_DESC_DMA_STATUS_ERR_PACKET BIT(1)
#define RX_DESC_DMA_STATUS_SMALL_PKT_ERR BIT(2)

/**
 * Length to use for common linux descriptors.  Arbitrarily chosen as 32.
 */
#define NAME_LEN 32

/**
 * Static globals
 */
static dev_t major_dev_id;

/**
 * The resource index for the push block.  The pull block resource index depends
 * on whether a push block was defined and as such cannot be constant.
 */
#define PUSHPULL_PUSH_PARAM_IDX	0
#define MAX_LINKID_DFS_COUNT	16

/**
 * The length of a SpaceX header. This is used for the phynet push/pull.
 */
#define SPACEX_HEADER_LEN		20

/**
 * struct cdec_push_descriptor - common hardware receive descriptor.
 * @link_id:		identifier used for source identification.
 * @dma_status:		status bits for the data received.
 * @hw_fifo:		work fifo the descriptor was assigned to.
 * @payload_length:	length of received data in bytes.
 * @sw_ext:	software field sent over modemlink.
 */
struct cdec_push_descriptor {
	u16 link_id;
	u8 dma_status;
	u8 hw_fifo;
	u32 payload_length;
	u64 sw_ext;
} __packed;
#define CDEC_PUSH_DESCRIPTOR_SIZE 16

/**
 * struct c4eth_push_descriptor - common hardware receive descriptor for cut4.
 *
 * @dmac:			Destination MAC address.
 * @smac:			Source MAC address.
 * @ethertype:			Ethertype of this packet.
 * @payload_length:		12 bit payload length.
 * @status:			Error flags from hardware.
 */
struct c4eth_push_descriptor {
	u8 dmac[6];
	u8 smac[6];
	u16 ethertype;
	u8 payload_length_upper : 8;
	u8 status : 4;
	u8 payload_length_lower : 4;
} __packed;
#define C4ETH_PUSH_DESCRIPTOR_SIZE 16

/**
 * struct cdec_pull_descriptor - common hardware transmit descriptor.
 * @link_id:	identifier used for source identification.
 * @desc_tag:   tag to identify descriptor for software.
 * @reserved0:	reserved region.
 * @sw_ext:	software field sent over modemlink.
 * @address:	physical address of the data to transmit.
 * @length:	length of data to send in bytes.
 */
struct cdec_pull_descriptor {
	u16 link_id;
	u16 desc_tag;
	u8 reserved0[4];
	u64 sw_ext;
	u32 address;
	u32 length;
} __packed;
#define CDEC_PULL_DESCRIPTOR_SIZE 24

/**
 * struct c4sdu_push_descriptor - common hardware receive descriptor for cut4.
 * @link_id:			identifier used for source identification.
 * @dma_status:			error status from DMA hardware.
 * @reserved0:			reserved_region.
 * @reserved1:			reserved_region.
 * @playload_length:	length of the message in Bytes.
 * @reserved2:			reserved region.
 * @desc_type:			descriptor type.
 * @reserved3:			reserved region.
 */
struct c4sdu_push_descriptor {
	u16 link_id;
	u8 dma_status : 3;
	u8 reserved0 : 5;
	u8 reserved1;
	u16 payload_length;
	u8 reserved2[2];
	u8 desc_type;
	u8 reserved3[7];
} __packed;
#define C4SDU_PUSH_DESCRIPTOR_SIZE 16

/**
 * struct c4sdu_pull_descriptor - common hardware receive descriptor for cut4.
 * @link_id:			identifier used for source identification.
 * @desc_tag:			DMA Descriptor Tag.
 * @is_data:			flag for data injection.
 * @reserved0:			reserved region.
 * @reserved1:			reserved region.
 * @address:			start address of the packet to be transmitted.
 * @length:				length of the message in Bytes.
 */
struct c4sdu_pull_descriptor {
	u16 link_id;
	u16 desc_tag;
	u8 is_data : 1;
	u8 reserved0 : 7;
	u8 reserved1[11];
	u32 address;
	u32 length;
} __packed;
#define C4SDU_PULL_DESCRIPTOR_SIZE 24

#define CFE_MSG_TYPE_DATA			0
#define CFE_MSG_TYPE_PHY_CTRL			1
#define CFE_MSG_TYPE_SCP_MSG			2
#define CFE_MSG_TYPE_MAC_CTRL_MSG		3
#define CFE_MSG_TYPE_MAC_CTRL_SDU		4

/*
 * MAC does not care about SDU vs MSG distinction.  Instead, they care about
 * one beam per hw fifo.  Hardware does the following map:
 *  Beam 0 and 2 MSG type to WF0
 *  Beam 1 and 3 MSG type to WF1
 *  Beam 0 and 2 SDU type to WF2
 *  Beam 1 and 3 SDU type to WF3
 * Therefore, by setting beam 0 and 1 to use MSG exclusively and beam 2 and 3 to
 * use SDU exclusively, each beam will arrive on exactly one work fifo.
 */
#define CFE_MAC_MSG_TYPE(__linkid)	(((__linkid) & 2) ? \
						CFE_MSG_TYPE_MAC_CTRL_SDU : \
						CFE_MSG_TYPE_MAC_CTRL_MSG)

enum cfe_pp_type {
	CFE_TYPE_SCP,
	CFE_TYPE_PHY,
	CFE_TYPE_MAC
};

/**
 * struct cfe_pull_descriptor - hardware tx descriptor for CFE pull blocks.
 * @link_id:	identifier used for source identification.
 * @desc_tag:	tag for the descriptor.
 * @type:	message type to send.
 * @tag:	alternate tag for the descriptor.
 * @reserved0:		reserved region.
 * @address:	physical address of the data to transmit.
 * @length:	length of data to send in bytes.
 */
struct cfe_pull_descriptor {
	u16 link_id;
	u8 desc_tag;
	u8 type;
	u16 tag;
	u8 reserved0[2];
	u32 address;
	u32 length;
} __packed;
#define CFE_PULL_DESCRIPTOR_SIZE 16

/**
 * struct aap_push_descriptor - hardware receive descriptor for AAP push blocks.
 * @payload_length:	Length of the received buffer.
 * @dma_status:		status bits for the data received.
 * @reserved0:		reserved region.
 */
struct aap_push_descriptor {
	u16 payload_length;
	u8 dma_status;
	u8 reserved0[5];
} __packed;
#define AAP_PUSH_DESCRIPTOR_SIZE 8

/**
 * struct aap_pull_descriptor - AAP pull hardware transmit descriptor for AAP
				pull blocks.
 * @slice_bytes:	The size of a sliced AAP message when combined.
 * @reserved0:		reserved region.
 * @tag:		Tag for the descriptor.
 * @address:		physical address of the data to transmit.
 * @length:		length of data to send in bytes.
 */
struct aap_pull_descriptor {
	u8 slice_bytes;
	u8 reserved0[6];
	u8 tag;
	u32 address;
	u32 length;
} __packed;
#define AAP_PULL_DESCRIPTOR_SIZE 16

/**
 * struct l3_push_descriptor - hardware receive descriptor for L3 push blocks.
 * @length:		Length of the received buffer.
 * @reason:		Reason the hardware is forwarding the buffer.
 * @ether_type:		The ether type of this packet.
 * @counter:		Free running counter.
 * @reserved0:		reserved region.
 */
struct l3_push_descriptor {
	u32 length	:13;
	u32 reason	:11;
	u16 ether_type;
	u64 counter;
	u8 reserved0[3];
} __packed;
#define L3_PUSH_DESCRIPTOR_SIZE 16

/**
 * struct l3dl_pull_descriptor - hardware transmit descriptor for L3DL pull
				 blocks.
 * @ether_type:		The ether type of this packet.
 * @address:		physical address of the data to transmit.
 * @length:		length of data to send in bytes.
 */
struct l3dl_pull_descriptor {
	u16 ether_type;
	u8 reserved[6];		/* Docs list this as 5 bytes in 2-7 */
	u32 address;
	u32 length;
} __packed;
#define L3DL_PULL_DESCRIPTOR_SIZE 16

/**
 * struct l3ul_pull_descriptor - hardware transmit descriptor for L3UL pull
 *                               blocks.  (Catson only.)
 * @security_tunnel_valid:	True if passing through a security tunnel.
 * @security_tunnel:		Security tunnel indentifier.
 * @cos:			Packet's class of service.
 * @beam_id:			Beam used to transmit the packet.
 * @phs_low:			Payload header supression index. (lower bits)
 * @phs_high:			Payload header supression index. (upper bits)
 * @phs_valid:			True if payload header suppression required.
 * @address:		physical address of the data to transmit.
 * @length:		length of data to send in bytes.
 */
struct l3ul_pull_descriptor {
	u8 security_tunnel_valid : 1;
	u8 security_tunnel : 7;
	u8 cos : 3;
	u8 beam_id : 1;
	u8 phs_low : 4;
	u8 phs_high : 3;
	u8 phs_valid : 1;
	u8 reserved0 : 4;
	u8 reserved1[5];
	u32 address;
	u32 length;
} __packed;
#define L3UL_PULL_DESCRIPTOR_SIZE 16

/**
 * struct npu_push_descriptor - hardware receive descriptor for Satellite and
 *   Gateway NPU push blocks.
 * @dma_status:	status bits for the data received.
 * @pkt_type:   gnpu_packet_type of packet source
 * @reason_id:	reason why this packet was pushed to RAM.
 * @reserved:	reserved field.
 * @length:	received packet length in bytes.
 */

struct npu_push_descriptor {
	u8 dma_status;
	u8 pkt_type;
	u8 reason_id;
	u8 reserved;
	u32 length;
} __packed;
#define NPU_PUSH_DESCRIPTOR_SIZE 8

struct pktproc_push_descriptor {
	u32 filter_track :5;
	u8 rsvd1 : 3;
	u8 dma_status;
	u16 rsvd2;
	u32 length;
} __packed;
#define PKTPROC_PUSH_DESCRIPTOR_SIZE 8

/**
 * struct snpu_pull_descriptor - hardware transmit descriptor for Satellite NPU
 *   pull blocks.
 * @dest0:	first destination port.
 * @valid0:	1 if the first destination is valid.
 * @sid0:	first destination MAC session ID.
 * @lag0:	1 if the first destination is a LAG (link aggregation group).
 * @cos:	packet class of service.
 * @dp:		packet drop precedence.
 * @reserved0:	reserved region.
 * @dest1:	second destination port.
 * @valid1:	1 if the second destination is valid.
 * @sid1:	second destination MAC session ID.
 * @lag1:	1 if the second destination is a LAG.
 * @reserved1:	reserved region.
 * @address:	physical address of the packet to transmit.
 * @length:	packet length in bytes.
 */
struct snpu_pull_descriptor {
	u8 dest0 : 7;
	u8 valid0: 1;
	u16 sid0;
	u8 lag0: 1;
	u8 cos: 3;
	u8 dp: 2;
	u8 reserved0: 2;
	u8 dest1: 7;
	u8 valid1: 1;
	u16 sid1;
	u8 lag1: 1;
	u8 reserved1: 7;
	u32 address;
	u32 length;
} __packed;
#define SNPU_PULL_DESCRIPTOR_SIZE 16

/**
 * struct gnpu_pull_descriptor - hardware transmit descriptor for Gateway NPU
 *    pull blocks.
 * @op_index:         frame editor operation index.
 * @dest_port:        destination port on the NPU.
 * @bicast_op_index:  Frame editor operation index for a copy of the packet.
 * @bicast_dest_port: destination port for a copy of the packet.
 * @bicast_valid:     1 if a copy of the packet is requested.
 * @reserved1:        reserved region.
 * @pkt_type:         packet type (Ethernet, SpaceX etc.)
 * @reserved2:        reserved region.
 * @reserved3:        reserved region.
 * @address:          physical address of the packet to transmit.
 * @length:	          packet length in bytes.
 */
struct gnpu_pull_descriptor {
	u32 op_index : 8;
	u32 dest_port : 4;
	u32 bicast_op_index: 8;
	u32 bicast_dest_port: 4;
	u32 bicast_valid: 1;
	u32 reserved1: 3;
	u32 pkt_type: 2;
	u32 reserved2: 2;
	u32 reserved3;
	u32 address;
	u32 length;
} __packed;
#define GNPU_PULL_DESCRIPTOR_SIZE 16

struct pktproc_pull_descriptor {
	u16 rsvd;
	u8 lswitch;
	u8 encap_hdr;
	u32 rsvd2;
	u32 address;
	u32 length;
} __packed;
#define PKTPROC_PULL_DESCRIPTOR_SIZE 16

/**
 * Gateway NPU destination ports.
 * See https://rtm.spacex.corp/docs/moka-fpga/en/latest/destinations.html
 */
enum gnpu_destination_port {
	GNPU_DEST_MAC_0,
	GNPU_DEST_MAC_1,
	GNPU_DEST_MAC_2,
	GNPU_DEST_MAC_3,
	GNPU_DEST_MAC_4,
	GNPU_DEST_MAC_5,
	GNPU_DEST_MAC_6,
	GNPU_DEST_MAC_7,
	GNPU_DEST_CPU,
	GNPU_DEST_ETH_0,
	GNPU_DEST_ETH_1,
	GNPU_DEST_UNUSED_0,
	GNPU_DEST_UNUSED_1,
	GNPU_DEST_UNUSED_2,
	GNPU_DEST_ETH_LOAD_BALANCE,
	GNPU_DEST_SX_LOAD_BALANCE,
};

/**
 * Gateway Pull packet types.
 * https://stash/projects/FPGASAT/repos/gnet_fpga/browse/gnet_top/sub/l3_sw_dp/src/l3sw_ingress.sv#144
 */
enum gnpu_packet_type {
	GNPU_PACKET_TYPE_NONE,
	GNPU_PACKET_TYPE_SPACEX,
	GNPU_PACKET_TYPE_ETHERNET,
	GNPU_PACKET_TYPE_CPU,
	GPNU_PACKET_TYPE_MAX
};

/**
 * A generic descriptor which defines all possible descriptors.  This is only
 * used to determine the size of allocations.
 */
union pp_descriptor {
	union {
		struct cdec_push_descriptor cdec;
		struct c4eth_push_descriptor c4dec;
		struct c4sdu_push_descriptor c4sdu;
		struct aap_push_descriptor aap;
		struct l3_push_descriptor l3;
		struct npu_push_descriptor npu;
	} __packed push;

	union {
		struct cdec_pull_descriptor cdec;
		struct c4sdu_push_descriptor c4sdu;
		struct cfe_pull_descriptor cfe;
		struct aap_pull_descriptor aap;
		struct l3dl_pull_descriptor l3dl;
		struct l3ul_pull_descriptor l3ul;
		struct snpu_pull_descriptor snpu;
		struct gnpu_pull_descriptor gnpu;
	} __packed pull;
} __packed;
#define PP_DESCRIPTOR_SIZE		sizeof(union pp_descriptor)

/**
 * struct pp_fifo_entry - describes a single buffer.
 * @list:		reference to any list entry might belong to.
 * @data_len:		length of valid data present in buffer. May be zero.
 * @desc:		virtual address of descriptor memory.
 * @desc_phys:		physical address of descriptor memory.
 * @buffer:		virtual address of buffer memory to transfer.
 * @buffer_phys:	physical address of buffer memory to transfer.
 * @priv:		Private entry specific information.
 */
struct pp_fifo_entry {
	struct list_head list;

	size_t data_len;

	void *desc;
	dma_addr_t desc_phys;

	void *buffer;
	dma_addr_t buffer_phys;

	void *priv;

#ifdef PUSHPULL_ENABLE_TIMING
	ktime_t timestamp;
#endif
};

/**
 * struct fifo_wait - Describes a FIFO and data related to that fifo.
 * @fifo:		the fifo of data.
 * @depth:		list length of fifo.
 */
struct fifo_wait {
	struct list_head fifo;
	u32 depth;
};

/*
 * struct pp_dev_ctx - Information that is associated with a particular
 * character device.  From this, different parameters are set on pull and
 * different user lists are filled for push.
 * @cdev:	The backing char device.
 * @priv:	Pointer to the pushpull structure.
 * @rd_lock:	The mutex that prevents multiple readers of the file.
 * @user_queue:	Pointer to the completion queue for this file.
 * @wait_queue:	Wait queue for notifying users of available data.
 * @minor:	The cdev minor number associated with this message.
 * @link_id:	The linkid associated with the file.  Only used in some modes.
 * @cfe_msg_type: CFE specific metadata.
 */
struct pp_dev_ctx {
	struct cdev cdev;

	struct pp_priv *priv;

	volatile unsigned long rd_lock;

	struct fifo_wait *user_queue;
	wait_queue_head_t *wait_queue;

	size_t minor;

	/*
	 * This is such a common and configurable/non-static field, put it in
	 * the context for convienence.
	 */
	u32 link_id;
};

/*
 * struct pp_file_ctx - Information that is associated with a particular
 * file.  From this, different parameters are set per-fd.
 * @dev_ctx:	The backing char device's context.
 * @security_tunnel_valid:	L3UL-specific metadata.
 * @security_tunnel:		L3UL-specific metadata.
 * @cos:					L3UL-specific metadata.
 */
struct pp_file_ctx {
	struct pp_dev_ctx *dev_ctx;

	u8 security_tunnel_valid : 1;
	u8 security_tunnel : 7;
	u8 cos : 3;
};

/*
 * struct pp_skb_ctx - Context information for a SKB.
 * @skb:		The SKB either being sent or received.
 * @transmit_idx:	The index for sequence through skb transmission.
 */
struct pp_skb_ctx {
	struct sk_buff *skb;

	int transmit_idx;
};

#ifdef PUSHPULL_ENABLE_TIMING
/**
 * struct pp_timing - timing statistics tracking structure.
 * @lock:		Lock for updating the statistics.
 * @start_interval:	The start time for which these stats are recorded.
 * @packet_cnt:		Number of packets that have been timed.
 * @total_latency:	Sum of latency for packet_cnt.
 * @min_latency:	The minimum latency seen.
 * @max_latency:	The maximum latency seen.
 * @bucket:		An array indicated how many packets are in each bracket.
 */
struct pp_timing {
	spinlock_t lock;
	ktime_t start_interval;
	u64 packet_cnt;
	u64 total_latency;
	u64 min_latency;
	u64 max_latency;
	u64 bucket[PUSHPULL_TIMING_BUCKET_CNT];
};
#endif

/**
 * struct pp_channel - channel tracking structure.
 * @dev:		The platform device this channel is attached to.
 * @regs:		Data related to the register region for the channel.
 * @irq:		Data related to the irq for the channel.
 * @dynamic_buffers:	Whether buffers have been preallocated.
 * @buffer_size:	Size of each buffer allocated.
 * @hw_entries:		Data related to the entries to be given to hardware.
 * @hw_dma_min_transfer: Minimum transfer size hardware supports.
 * @timing:		Statistics related to driver timing.
 * @lock:		Lock for handling linked lists.
 * @wait_queue:		Wait queue for notifying users of available data.
 * @free:		The FIFO data of entries ready to be used.
 * @used:		The FIFO data of entries pending hardware.
 * @complete:		The FIFO data of entries pending software.
 */
struct pp_channel {
	struct platform_device  *dev;

	struct {
		void __iomem *regs;
		struct resource *resource;
		char name[NAME_LEN];
	} regs;

	struct {
		int num;
		char name[NAME_LEN];
	} irq;

	bool dynamic_buffers;
	u32 buffer_size;

	u16 hw_fifo_max_depth;

	u32 hw_dma_min_transfer;

	struct {
		struct pp_fifo_entry *fifo_entries;
		struct dma_pool *desc_pool;
		struct dma_pool *buffer_pool;

		unsigned int fifo_count;
	} hw_entries;

#ifdef PUSHPULL_ENABLE_TIMING
	struct pp_timing timing;
#endif

	spinlock_t lock;

	wait_queue_head_t wait_queue;

	struct fifo_wait free; /* Acts as stack not queue */
	struct fifo_wait used;
	struct fifo_wait complete;
};

/**
 * struct pp_push_channel - push channel tracking structure.
 * @chan:		The generic pp channel.
 * @push_handle_pkt:	Function pointer for attaching data from hardware to a
 *			user fifo.
 * @stats:		sysfs exposed stats about the push channel.
 */
struct pp_push_channel {
	struct pp_channel chan;

	int (*push_prepare_desc)(struct pp_push_channel *pchan,
				 struct pp_fifo_entry *entry);

	void (*push_handle_pkt)(struct pp_push_channel *pchan,
				struct pp_fifo_entry *entry);

	struct {
		u64 usr_reads;
		u64 usr_pkts_read;
		u64 pkts_received;
		u64 hw_blocked;
		u64 usr_queued;
		u64 irq_queued;

		u64 cdev_pkts;
		u64 network_pkts;

		u64 discards;
	} stats;
};

/**
 * struct pp_linkid_push_channel - push channel for linkid based packets.
 * @pchan:		The generic push channel.
 * @user_queue:		A list of user queues for file contexts.
 * @link_id:		An array listing the linkid associated with user_queue.
 */
struct pp_linkid_push_channel {
	struct pp_push_channel pchan;

	wait_queue_head_t wait_queue[MAX_LINKID_DFS_COUNT];
	struct fifo_wait user_queue[MAX_LINKID_DFS_COUNT];
	u16 link_id[MAX_LINKID_DFS_COUNT];
};

/**
 * struct pp_net_push_channel - push channel for network interfaces
 * @pchan:		The generic push channel.
 * @netdev:		Backing network device.
 * @napi:		NAPI structure for poll completion.
 * @header_len	The length of data to remove from incoming packets.
 */
struct pp_net_push_channel {
	struct pp_push_channel pchan;

	struct net_device *netdev;

	struct napi_struct napi;

	u16 header_len;
};

/**
 * struct pp_net_push_channel - push channel capable of handling a network and
 *		character device simultaneously.
 * @net_dev:		The network interface device.
 * @header_len:		The length of data to remove from incoming packets.
 * @cdev_queue:		The queue that the character device looks at.
 */
struct pp_hybrid_push_channel {
	struct pp_net_push_channel net_dev; /* MUST REMAIN FIRST */

	wait_queue_head_t cdev_wait_queue;
	struct fifo_wait cdev_queue;
};

/**
 * union pp_push_master_channel - Combination of all possible dchans.  Only used
 *				  for size determination.
 */
union pp_push_master_channel {
	struct pp_push_channel pchan;
	struct pp_linkid_push_channel linkid;
	struct pp_net_push_channel net_dev;
	struct pp_hybrid_push_channel hybrid_dev;
};

/**
 * The maximum size of all push dchans.
 */
#define PP_PUSH_CHANNEL_SIZE		sizeof(union pp_push_master_channel)

enum pp_pull_source {
	PULL_SOURCE_NETWORK,
	PULL_SOURCE_CHR_DEV,
};

/**
 * struct pp_pull_channel - pull channel tracking structure.
 * @chan:		The generic pushpull channel.
 * @pad_size		How much to pad out.
 * @encode_descriptor:	Function pointer for filling out a pull descriptor based
 *			off user file descriptor.
 * @extra_work:		Extra work to perform as part of pull thread.
 * @stats:		sysfs exposed stats about the push channel.
 * @endpoints:		The number of virtual endpoints attached to channel.
 */
struct pp_pull_channel {
	struct pp_channel chan;

	/* TX buffer pad size */
	u32 pad_size;

	bool (*encode_header)(struct pp_pull_channel *pchan,
				 void *buffer,
				 const struct sk_buff *skb);
	int (*encode_descriptor)(struct pp_pull_channel *pchan,
				 void *ctx,
				 size_t count,
				 struct pp_fifo_entry *entry,
				 enum pp_pull_source source);
	void (*extra_work)(struct pp_pull_channel *pchan);

	struct {
		u64 usr_writes;
		u64 pkts_sent;
		u64 usr_blocked;
		u64 hw_full;
		u64 usr_queued;
		u64 irq_queued;

		u64 network_pkts;
		u64 cdev_pkts;
	} stats;

	int endpoints;
};

/**
 * struct pp_cfe_pull_channel - cfe pull channel tracking structure.
 * @pchan:		The generic pp pull channel.
 * @cfe_type:		The type of cfe interface this is.
 */
struct pp_cfe_pull_channel {
	struct pp_pull_channel pchan;

	enum cfe_pp_type cfe_type;
};

/**
 * struct pp_net_pull_channel - linkid pull channel tracking structure.
 * @pchan:		The generic pp pull channel.
 * @linkid_count:	The number of associated linkids.
 * @link_id:		List of linkid based endpoints.
 * @netdev:		The backing network interface.
 * @header_len:	The length to reserve for headers.
 */
struct pp_net_pull_channel {
	struct pp_pull_channel pchan;

	int linkid_count;
	u32 link_id[MAX_LINKID_DFS_COUNT];

	struct net_device *netdev;

	u16 header_len;
};

/**
 * union pp_pull_master_channel - Combination of all possible dchans.  Only used
 *					for size determination.
 */
union pp_pull_master_channel {
	struct pp_pull_channel pchan;
	struct pp_cfe_pull_channel cfe;
	struct pp_net_pull_channel linkid_net;
};

/**
 * The maximum size of all pull dchans.
 */
#define PP_PULL_CHANNEL_SIZE		sizeof(union pp_pull_master_channel)

/*
 * bitmap of registered minor numbers
 */
static DECLARE_BITMAP(registered_minors, MAX_PUSHPULL_MINORS);

/*
 * mutex for protecting that bitmap (only required if pp_probe() can be
 * called from multiple threads at once -- while unlikely, it can't hurt)
 */
static DEFINE_MUTEX(registered_minors_lock);

static struct class *pp_class;

/* forward-declaration */
static const struct file_operations pp_pushpull_fops;
static const struct file_operations pp_push_fops;
static const struct file_operations pp_pull_fops;
static const struct attribute_group pp_push_sysfs_regs_group;
static const struct attribute_group pp_pull_sysfs_regs_group;
static const struct attribute_group pp_push_sysfs_stats_group;
static const struct attribute_group pp_pull_sysfs_stats_group;
static const struct attribute_group pp_cfe_sysfs_group;
static const struct attribute_group pp_cfenet_sysfs_group;

static irqreturn_t pp_push_irq(int irq, void *d);
static irqreturn_t pp_pull_irq(int irq, void *d);

/*
 * Descriptor specific implementation information.
 * @name:			Name matching the device tree entry.
 * @pull_extra_init:		FP for extra init for the pull channel.
 * @pull_extra_work:		FP for performing extra work in pull thread.
 * @pull_encode_descriptor:	FP for setting up the pull descriptor.
 * @push_extra_init:		FP for extra init for the push channel.
 * @push_handle_pkt:		FP for handling a push packet.
 * @if_register:		FP for creating user interface.
 * @if_unregister:		FP for removing user interface.
 * @if_ioctl			FP for ioctls.
 * @attrs:			Descriptor specific sysfs files.
 * @dynamic_push_buffers:	Set true to prevent preallocated push buffers.
 */
struct pp_desc_info {
	const char *name;

	int (*pull_extra_init)(struct pp_pull_channel *pchan,
			       struct platform_device *dev,
			       struct device_node *of_node);

	void (*pull_extra_work)(struct pp_pull_channel *pchan);

	bool (*net_pull_encode_header)(struct pp_pull_channel *pchan,
				    void *buffer,
					const struct sk_buff *skb);

	int (*pull_encode_descriptor)(struct pp_pull_channel *pchan,
				      void *ctx,
				      size_t count,
				      struct pp_fifo_entry *entry,
				      enum pp_pull_source source);

	int (*push_extra_init)(struct pp_push_channel *pchan,
			       struct platform_device *dev,
			       struct device_node *of_node);

	void (*push_extra_deinit)(struct pp_push_channel *pchan);

	int (*push_prepare_desc)(struct pp_push_channel *pchan,
				 struct pp_fifo_entry *entry);

	void (*push_handle_pkt)(struct pp_push_channel *pchan,
				struct pp_fifo_entry *entry);

	int (*if_register)(struct pp_priv *priv);

	void (*if_unregister)(struct pp_priv *priv);

	long (*if_ioctl)(struct file *file, unsigned int cmd, unsigned long arg);

	const struct attribute_group *attrs;

	bool dynamic_push_buffers;
};

/**
 * struct pp_priv - Private driver data structure for the push-pull cores.
 * @pdev:		back-reference to the owning &struct platform_device.
 * @dev:		back-reference to the owning &struct device.
 * @push:		push channel context.
 * @pull:		pull channel context.
 * @dev_ctx_count:	The array size of dev_ctx.
 * @dev_ctx:		An array of all the character device structures
 *			attached to the pushpull.
 * @netdev:		Net device for network interface.
 * @ip_dev:		Whether this device is an IP (tun) device.
 * @desc_info:		Descriptor specific information about pushpull.
 */
struct pp_priv {
	struct platform_device *pdev;
	struct device *dev;

	/* Channel context */
	struct pp_push_channel *push;
	struct pp_pull_channel *pull;

	struct {
		size_t dev_ctx_count;
		struct pp_dev_ctx *dev_ctx;
	};

	struct {
		/* How is there not a backcast macro? */
		struct net_device *netdev;
		bool ip_dev;
	};

	const struct pp_desc_info *desc_info;
};

/**
 * Helper functions to convert between the various structures.
 */
static inline struct pp_channel *push_chan_to_chan(struct pp_push_channel *pchan)
{
	return &pchan->chan;
}

static inline struct pp_push_channel *chan_to_push_chan(struct pp_channel *chan)
{
	return container_of(chan, struct pp_push_channel, chan);
}

static inline struct pp_channel *pull_chan_to_chan(struct pp_pull_channel *pchan)
{
	return &pchan->chan;
}

static inline struct pp_pull_channel *chan_to_pull_chan(struct pp_channel *chan)
{
	return container_of(chan, struct pp_pull_channel, chan);
}

static inline struct pp_push_channel *pp_priv_to_push(struct pp_priv *priv)
{
	return priv->push;
}

static inline struct pp_push_channel *dev_to_push(struct device *dev)
{
	struct pp_priv *priv = dev_get_drvdata(dev);

	return pp_priv_to_push(priv);
}

static inline struct pp_pull_channel *pp_priv_to_pull(struct pp_priv *priv)
{
	return priv->pull;
}

static inline struct pp_pull_channel *dev_to_pull(struct device *dev)
{
	struct pp_priv *priv = dev_get_drvdata(dev);

	return pp_priv_to_pull(priv);
}

/**
 * pp_reg_read() - reads a value from a push or pull block register.
 * @chan:	pushpull channel data structure.
 * @offset:	which register to read.
 *
 * @return	contents of register
 */
static u32 pp_reg_read(struct pp_channel *chan, u32 offset)
{
	char __iomem *regs = chan->regs.regs;

	return ioread32(regs + offset);
}

/**
 * push_reg_read() - reads a value from a push block register.
 * @pchan:	push channel data structure.
 * @offset:	which register to read.
 *
 * @return	contents of register.
 */
static u32 push_reg_read(struct pp_push_channel *pchan, u32 offset)
{
	return pp_reg_read(push_chan_to_chan(pchan), offset);
}

/**
 * pull_reg_read() - reads a value from a pull block register.
 * @pchan:	pull channel data structure.
 * @offset:	which register to read.
 *
 * @return	contents of register.
 */
static u32 pull_reg_read(struct pp_pull_channel *pchan, u32 offset)
{
	return pp_reg_read(pull_chan_to_chan(pchan), offset);
}

/**
 * pp_reg_write() - writes a value to a push or pull block register.
 * @chan:	pushpull channel data structure.
 * @offset:	which register to write.
 * @value:	the value to write to the register
 */
static void pp_reg_write(struct pp_channel *chan, u32 offset, u32 value)
{
	char __iomem *regs = chan->regs.regs;

	return iowrite32(value, regs + offset);
}

/**
 * push_reg_write() - writes a value to a push block register.
 * @pchan:	push channel data structure.
 * @offset:	which register to write.
 * @value:	the value to write to the register.
 */
static void push_reg_write(struct pp_push_channel *pchan, u32 offset, u32 value)
{
	return pp_reg_write(push_chan_to_chan(pchan), offset, value);
}

/**
 * pull_reg_write() - writes a value to a pull block register.
 * @pchan:	pull channel data structure.
 * @offset:	which register to write.
 * @value:	the value to write to the register.
 */
static void pull_reg_write(struct pp_pull_channel *pchan, u32 offset, u32 value)
{
	return pp_reg_write(pull_chan_to_chan(pchan), offset, value);
}

/**
 * push_clear_interrupts() - clear any pending push interrupts that are enabled.
 * @pchan:	push channel data structure.
 */
static void push_clear_interrupts(struct pp_push_channel *pchan)
{
	push_reg_write(pchan, DMA_PUSH_INTR0_STATUS_REG_OFFSET,
	    BIT(PUSH_RX_INTC0_CHAN_BIT));
	/* INTR1 register does not need to be written if not used. */
	#if PUSH_RX_INTC1_MASK > 0
	push_reg_write(pchan, DMA_PUSH_INTR1_STATUS_REG_OFFSET,
	    PUSH_RX_INTC1_MASK);
	#endif
}

/**
 * pull_clear_interrupts() - clear any pending pull interrupts that are enabled.
 * @pchan:	pull channel data structure.
 */
static void pull_clear_interrupts(struct pp_pull_channel *pchan)
{
	pull_reg_write(pchan, DMA_PULL_INTR0_STATUS_REG_OFFSET,
	    PULL_TX_INTC0_MASK);
}

#ifdef PUSHPULL_ENABLE_TIMING
/**
 * pp_timing_clear() - Resets all timing stats.
 * @chan:	pushpull channel data structure.
 */
static void pp_timing_clear(struct pp_timing *timing)
{
	timing->packet_cnt = 0;
	timing->total_latency = 0;
	timing->min_latency = U64_MAX;
	timing->max_latency = 0;
	memset(timing->bucket, 0, sizeof(timing->bucket));

	timing->start_interval = ktime_get();
}

/**
 * pp_timing_update() - Triggers an update of timing stats.
 * @chan:	pushpull channel data structure.
 */
static void pp_timing_update(struct pp_timing *timing, ktime_t start_time)
{
	unsigned long flags;
	unsigned long bucket_idx;
	u64 bucket_top = PUSHPULL_TIMING_BUCKET_STRIDE;
	u64 time_delta_us = ktime_us_delta(ktime_get(), start_time);

	for (bucket_idx = 0; bucket_idx < PUSHPULL_TIMING_BUCKET_CNT - 1;
	     bucket_idx++) {
		if (time_delta_us < bucket_top)
			break;

		bucket_top *= PUSHPULL_TIMING_BUCKET_STRIDE;
	}

	spin_lock_irqsave(&timing->lock, flags);

	if (unlikely(time_delta_us < timing->min_latency)) {
		timing->min_latency = time_delta_us;
	}

	if (unlikely(time_delta_us > timing->max_latency)) {
		timing->max_latency = time_delta_us;
	}

	timing->packet_cnt++;
	timing->total_latency += time_delta_us;
	timing->bucket[bucket_idx]++;

	spin_unlock_irqrestore(&timing->lock, flags);
}
#endif /* PUSHPULL_ENABLE_TIMING */

/**
 * push_can_fill_used_fifo() - test if the hardware receive fifo can be filled.
 * @chan:	pushpull channel data structure.
 *
 * @return	true if ready to fill, else false.
 *
 * Read the current fifo of free buffers to determine if a new buffer can be
 * pushed onto the hardware free fifo to use for receive.
 */
static bool push_can_fill_used_fifo(struct pp_channel *chan)
{
	return !list_empty(&chan->free.fifo) &&
		(chan->used.depth < chan->hw_fifo_max_depth);
}

/**
 * pp_net_push_prepare_desc() - Prepare a push descriptor for RX.
 * @pchan:	Push channel to send the entry on.
 * @entry:	The entry about to be transmitted to hardware.
 *
 * @return	0 on success, <0 on failure.
 */
static int pp_net_push_prepare_desc(struct pp_push_channel *pchan,
				    struct pp_fifo_entry *entry)
{
	struct pp_net_push_channel *dchan = container_of(pchan,
						struct pp_net_push_channel,
						pchan);
	struct pp_channel *chan = push_chan_to_chan(pchan);
	struct sk_buff *skb = netdev_alloc_skb(dchan->netdev,
				chan->buffer_size + PUSHPULL_DESC_ALIGNMENT);
	if (!skb) {
		return -1;
	}

	/* Push has an 8 byte alignment requirement. */
	skb_reserve(skb, ALIGN((uintptr_t)skb->data, PUSHPULL_DESC_ALIGNMENT) -
			 (uintptr_t)skb->data);

	entry->priv = skb;
	entry->buffer = skb->data;
	entry->buffer_phys = dma_map_single(&chan->dev->dev, entry->buffer,
					    chan->buffer_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&chan->dev->dev, entry->buffer_phys)) {
		dev_kfree_skb_any(skb);
		return -1;
	}

	return 0;
}

/**
 * gnpu_push_prepare_desc() - Prepare a push descriptor for RX.  gnpu will not
 *			      free the skb in the cdev path.
 * @pchan:	Push channel to send the entry on.
 * @entry:	The entry about to be transmitted to hardware.
 *
 * @return	0 on success, <0 on failure.
 */
static int gnpu_push_prepare_desc(struct pp_push_channel *pchan,
				  struct pp_fifo_entry *entry)
{
	if (!entry->priv)
		return pp_net_push_prepare_desc(pchan, entry);

	return 0;
}

/**
 * pp_push_start_receive() - Feed fifo entries to hardware.
 * NOTE: CHANNEL LOCK MUST BE HELD
 * @pchan:	The push channel to perform work on.
 *
 * @return	number of entries fed.
 */
static int pp_push_start_receive(struct pp_push_channel *pchan)
{
	struct pp_channel *chan = push_chan_to_chan(pchan);
	int ret = 0;

	while (push_can_fill_used_fifo(chan)) {
		struct pp_fifo_entry *entry = list_first_entry(&chan->free.fifo,
							struct pp_fifo_entry,
							list);

		/* Make sure entry->data_len is zero. */
		entry->data_len = 0;

		/*
		 * Revisit: Ideally this is done before the entry is placed
		 * on the free fifo, so theres less overhead when hardware is
		 * ready to be serviced.
		 */
		if (pchan->push_prepare_desc &&
		    dev_WARN_ONCE(&chan->dev->dev,
				  pchan->push_prepare_desc(pchan,
							   entry),
				  "Failed to prepare push descriptor\n")) {
			break;
		}

		/* Mark as in-use. */
		list_move_tail(&entry->list, &chan->used.fifo);

		/* Drain any outstanding transactions first. */
		wmb();
		push_reg_write(pchan,
			       DMA_PUSH_FREE_FIFO_WR_DESC_REG_OFFSET,
			       (u32)entry->desc_phys);

		/* Ensure buffer is written after descriptor. */
		wmb();
		push_reg_write(pchan,
			       DMA_PUSH_FREE_FIFO_WR_DATA_REG_OFFSET,
			       (u32)((uintptr_t)entry->buffer_phys));

		chan->free.depth--;
		chan->used.depth++;
		ret++;
	}

	return ret;
}

/**
 * pull_can_fill_work_fifo() - test if the hardware transmit fifo can be filled.
 * @pchan:	pull channel to read from.
 *
 * @return	true if ready to fill, else false.
 *
 * Read the current fifo of used buffers to determine if a new buffer can be
 * pushed onto the hardware work fifo to transmit.
 */
static bool pull_can_fill_work_fifo(struct pp_pull_channel *pchan)
{
	struct pp_channel *chan = pull_chan_to_chan(pchan);

	if (!list_empty(&chan->used.fifo)) {
		if (chan->complete.depth < chan->hw_fifo_max_depth)
			return true;

		pchan->stats.hw_full++;
	}

	return false;
}

/**
 * network_pull_extra_work() - Performs network work.
 * @pchan:	The pull channel to perform work on.
 */
static void network_pull_extra_work(struct pp_pull_channel *pchan)
{
	struct pp_channel *chan = pull_chan_to_chan(pchan);
	struct pp_net_pull_channel *dchan = container_of(pchan,
					      struct pp_net_pull_channel,
					      pchan);

	if (netif_queue_stopped(dchan->netdev) &&
	    chan->free.depth >= pchan->endpoints) {
		netif_wake_queue(dchan->netdev);
	}
}

/**
 * pp_pull_start_transmit() - Drain fifo entries to hardware.
 * NOTE: CHANNEL LOCK MUST BE HELD
 * @pchan:	The pull channel to perform work on.
 *
 * @return	number of packets transmitted.
 */
static int pp_pull_start_transmit(struct pp_pull_channel *pchan)
{
	struct pp_channel *chan = pull_chan_to_chan(pchan);
	int ret = 0;

	while (pull_can_fill_work_fifo(pchan)) {
		struct pp_fifo_entry *entry = list_first_entry(&chan->used.fifo,
							struct pp_fifo_entry,
							list);
#ifdef PUSHPULL_ENABLE_TIMING
		ktime_t timestamp = entry->timestamp;
#endif
		/* Move it to the completed FIFO. */
		list_move_tail(&entry->list, &chan->complete.fifo);

		/* Queue descriptor for transmit. */
		wmb();
		pull_reg_write(pchan,
			       DMA_PULL_WORK_FIFO_WR_DESC_REG_OFFSET,
			       (u32)entry->desc_phys);
#ifdef PUSHPULL_ENABLE_TIMING
		pp_timing_update(&chan->timing, timestamp);
#endif

		chan->complete.depth++;
		ret++;
	}

	chan->used.depth -= ret;
	pchan->stats.pkts_sent += ret;

	return ret;
}

/**
 * pp_deinit_channel() - Free channel resources.
 * @dev		platform device.
 * @chan	channel to free.
 *
 * Note the dma pools are managed, so no frees are required.
 */
static void pp_deinit_channel(struct platform_device *dev,
			      struct pp_channel *chan)
{
	int i;

	free_irq(chan->irq.num, chan);

	iounmap(chan->regs.regs);
	release_mem_region(chan->regs.resource->start,
			   resource_size(chan->regs.resource));

	for (i = 0; i < chan->hw_entries.fifo_count; i++) {
		struct pp_fifo_entry *entry = &chan->hw_entries.fifo_entries[i];

		if (!chan->dynamic_buffers) {
			dma_pool_free(chan->hw_entries.buffer_pool,
				      entry->buffer,
				      entry->buffer_phys);
		}

		dma_pool_free(chan->hw_entries.desc_pool,
			      entry->desc,
			      entry->desc_phys);
	}
}

/**
 * net_push_chan_deinit() - Frees network channel resources.
 *
 * @pchan	push channel to free.
 */
static void net_push_chan_deinit(struct pp_push_channel *pchan)
{
	struct pp_channel *chan = push_chan_to_chan(pchan);
	struct pp_net_push_channel *dchan = container_of(pchan,
						struct pp_net_push_channel,
						pchan);
	int i;

	netif_napi_del(&dchan->napi);

	for (i = 0; i < chan->hw_entries.fifo_count; i++) {
		struct pp_fifo_entry *entry = &chan->hw_entries.fifo_entries[i];
		struct sk_buff *skb = entry->priv;

		if (skb) {
			dma_unmap_single(&chan->dev->dev, entry->buffer_phys,
					 chan->buffer_size, DMA_FROM_DEVICE);
			dev_kfree_skb_any(skb);
			entry->priv = NULL;
		}
	}
}

/**
 * pp_push_free_channel() - Frees push channel resources.
 *
 * @dev		platform device.
 * @pchan	push channel to free.
 * @desc_info	descriptor info about the channel.
 */
static void pp_push_free_channel(struct platform_device *dev,
				 struct pp_push_channel *pchan,
				 const struct pp_desc_info *desc_info)
{
	struct pp_channel *chan = push_chan_to_chan(pchan);

	/* Disable hardware interrupts. */
	push_reg_write(pchan, DMA_PUSH_INTR0_EN_REG_OFFSET, 0);
	push_reg_write(pchan, DMA_PUSH_INTR1_EN_REG_OFFSET, 0);

	if (desc_info->push_extra_deinit) {
		desc_info->push_extra_deinit(pchan);
	}

	sysfs_remove_group(&chan->dev->dev.kobj, &pp_push_sysfs_regs_group);
	sysfs_remove_group(&chan->dev->dev.kobj, &pp_push_sysfs_stats_group);

	pp_deinit_channel(dev, chan);
}

/**
 * pp_pull_free_channel() - Frees pull channel resources.
 *
 * @dev		platform device.
 * @pchan	pull channel to free.
 * @desc_info	descriptor info about the channel.
 */
static void pp_pull_free_channel(struct platform_device *dev,
				 struct pp_pull_channel *pchan,
				 const struct pp_desc_info *desc_info)
{
	struct pp_channel *chan = pull_chan_to_chan(pchan);

	/* Disable hardware interrupts */
	pull_reg_write(pchan, DMA_PULL_INTR0_EN_REG_OFFSET, 0);

	pp_deinit_channel(dev, pull_chan_to_chan(pchan));

	sysfs_remove_group(&chan->dev->dev.kobj, &pp_pull_sysfs_regs_group);
	sysfs_remove_group(&chan->dev->dev.kobj, &pp_pull_sysfs_stats_group);
}

/**
 * pp_chan_init_common() - Common code for initializing a channel.
 * @dev		platform device.
 * @of_node	The device_node containing channel specific metadata.
 * @chan	The channel to initialize.
 * @name	Pretty name to identify the channel.
 * @resouce_idx	The resource number to use for regs and interrupts.
 * @dynamic_buffers When true, do not allocate DMA buffers.
 * @irq_handler The interrupt handler for this channel.
 *
 * @return	zero on successful allocation, non-zero on failure.
 */
static int pp_chan_init_common(struct platform_device *dev,
			       struct device_node *of_node,
			       struct pp_channel *chan,
			       const char *name,
			       u32 resource_idx,
			       bool dynamic_buffers,
			       irq_handler_t irq_handler)
{
	int rc = 0;
	u32 i;
	char dmam_name[NAME_LEN];

	chan->dev = dev;

	spin_lock_init(&chan->lock);

#ifdef PUSHPULL_ENABLE_TIMING
	spin_lock_init(&chan->timing.lock);
	pp_timing_clear(&chan->timing);
#endif

	/*
	 * Determine the size of the buffer for each fifo entry.
	 */
	rc = of_property_read_u32(of_node, "buffer-size",
				  &chan->buffer_size);
	if (rc) {
		dev_err(&dev->dev,
			"Missing required parameter 'buffer-size'\n");
		return rc;
	}

	/*
	 * Determine the minimum dma transfer size.
	 */
	rc = of_property_read_u32(of_node, "min-transfer",
				  &chan->hw_dma_min_transfer);
	if (rc) {
		chan->hw_dma_min_transfer = 0;
	}

	chan->dynamic_buffers = dynamic_buffers;

	/*
	 * Determine the number of fifo entries for this channel.
	 */
	rc = of_property_read_u32(of_node, "fifo-depth",
				  &chan->hw_entries.fifo_count);
	if (rc) {
		dev_err(&dev->dev, "Missing required parameter 'fifo-depth'\n");
		return rc;
	}

	/*
	 * Create DMA pools for the descriptors and buffers.
	 */
	snprintf(dmam_name, sizeof(dmam_name), "%s_desc", name);
	chan->hw_entries.desc_pool = dmam_pool_create(dmam_name, &dev->dev,
						      PP_DESCRIPTOR_SIZE,
						      PUSHPULL_DESC_ALIGNMENT,
						      0);
	if (!chan->hw_entries.desc_pool) {
		dev_err(&dev->dev, "desc dmam_pool_create failed\n");
		return -ENOMEM;
	}

	if (!chan->dynamic_buffers) {
		snprintf(dmam_name, sizeof(dmam_name), "%s_buff", name);
		chan->hw_entries.buffer_pool = dmam_pool_create(dmam_name,
							&dev->dev,
							chan->buffer_size,
							PUSHPULL_DESC_ALIGNMENT,
							0);
		if (!chan->hw_entries.buffer_pool) {
			dev_err(&dev->dev, "buff dmam_pool_create failed\n");
			return -ENOMEM;
		}
	}

	init_waitqueue_head(&chan->wait_queue);

	INIT_LIST_HEAD(&chan->free.fifo);
	chan->free.depth = 0;

	INIT_LIST_HEAD(&chan->used.fifo);
	chan->used.depth = 0;

	INIT_LIST_HEAD(&chan->complete.fifo);
	chan->complete.depth = 0;

	/*
	 * Map in hardware registers.
	 */
	scnprintf(chan->regs.name, sizeof(chan->regs.name), "%s_regs", name);
	chan->regs.resource = platform_get_resource(dev, IORESOURCE_MEM,
						    resource_idx);
	if (!chan->regs.resource) {
		dev_err(&dev->dev, "No memory region specified for %s!\n",
			chan->regs.name);
		goto resource_failed;
	}

	if (!request_mem_region(chan->regs.resource->start,
				resource_size(chan->regs.resource),
				chan->regs.name)) {
		dev_err(&dev->dev, "Could not map memory region %s!\n",
			chan->regs.name);
		goto resource_failed;
	}

	chan->regs.regs = ioremap(chan->regs.resource->start,
				  resource_size(chan->regs.resource));
	if (!chan->regs.regs) {
		dev_err(&dev->dev, "IO Remap failed for region %s!\n",
			chan->regs.name);
		rc = -EACCES;
		goto ioremap_failed;
	}

	/*
	 * Set up hardware interrupts.
	 */
	scnprintf(chan->irq.name, sizeof(chan->irq.name), "pp_%s_irq_%s",
		  name, dev->name);
	chan->irq.num = platform_get_irq(dev, resource_idx);
	if (chan->irq.num < 0) {
		dev_err(&dev->dev, "No IRQ specified for '%s!\n",
			chan->irq.name);
		rc = chan->irq.num;
		goto irq_failed;
	}

	rc = request_irq(chan->irq.num, irq_handler, 0, chan->irq.name, chan);
	if (rc) {
		dev_err(&dev->dev, "Could not register %d ('%s')!\n",
			chan->irq.num, chan->irq.name);
		goto irq_failed;
	}

	/*
	 * Allocate the software based fifo entries.
	 */
	chan->hw_entries.fifo_entries = devm_kzalloc(&dev->dev,
		sizeof(struct pp_fifo_entry) * chan->hw_entries.fifo_count,
		GFP_KERNEL);
	if (!chan->hw_entries.fifo_entries) {
		goto fifo_alloc_failed;
	}

	/* Allocate and add all buffers to the free fifo. */
	for (i = 0; i < chan->hw_entries.fifo_count; i++) {
		struct pp_fifo_entry *entry = &chan->hw_entries.fifo_entries[i];

		entry->desc = dma_pool_zalloc(chan->hw_entries.desc_pool,
					      GFP_KERNEL,
					      &entry->desc_phys);
		if (!entry->desc) {
			rc = -ENOMEM;
			goto dma_alloc_failed;
		}

		if (!chan->dynamic_buffers) {
			entry->buffer =
			    dma_pool_zalloc(chan->hw_entries.buffer_pool,
					    GFP_KERNEL,
					    &entry->buffer_phys);
			if (!entry->buffer) {
				rc = -ENOMEM;
				goto dma_alloc_failed;
			}
		}

		INIT_LIST_HEAD(&entry->list);

		/*
		 * Add to free fifo. Since this is init, defer signalling the
		 * wait queue.
		 */
		list_add(&entry->list,
			 &chan->free.fifo);
		chan->free.depth++;
	}

	return 0;

dma_alloc_failed:
	for (i = 0; i < chan->hw_entries.fifo_count; i++) {
		struct pp_fifo_entry *entry = &chan->hw_entries.fifo_entries[i];

		if (entry->desc) {
			dma_pool_free(chan->hw_entries.desc_pool,
				      entry->desc,
				      entry->desc_phys);
		}

		if (entry->buffer) {
			dma_pool_free(chan->hw_entries.buffer_pool,
				      entry->buffer,
				      entry->buffer_phys);
		}
	}
fifo_alloc_failed:
	free_irq(chan->irq.num, chan);
irq_failed:
	release_mem_region(chan->regs.resource->start,
			   resource_size(chan->regs.resource));
ioremap_failed:
	iounmap(chan->regs.regs);
resource_failed:

	return rc;
}

/**
 * cdev_handle_pkt_common() - Common handler for network packets.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 * @complete:	The fifo_wait associated with this packet.
 * @wait_queue:	The waitqueue to wake up with this packet.
 * @length:	The length of the payload in the entry.
 * @failure:	Whether hardware reported an error on the packet.
 */
static void cdev_handle_pkt_common(struct pp_push_channel *pchan,
				   struct pp_fifo_entry *entry,
				   struct fifo_wait *complete,
				   wait_queue_head_t *wait_queue,
				   int length,
				   int failure)
{
	struct pp_channel *chan = push_chan_to_chan(pchan);
	unsigned long flags;
	int error = 0;

	dev_WARN_ONCE(&chan->dev->dev, entry->data_len,
		      "Entry has non-zero length %zu\n", entry->data_len);

	/*
	 * Extract the data length and store it.
	 */
	entry->data_len = length;

	/*
	 * In the case where we have a descriptor with an
	 * unexpected length, truncate it to zero. This could
	 * indicate corrupted data from the FPGA.
	 */
	if (dev_WARN_ONCE(&chan->dev->dev, entry->data_len > chan->buffer_size,
			  "Entry (%zd) exceeds maximum buffer size (%d)\n",
			  entry->data_len, chan->buffer_size)) {
		entry->data_len = 0;
		error = 1;
	}

	/*
	 * Non-zero DMA status indicates a problem with the
	 * packet, but some of this status is informational.
	 * If the error bit is set, warn and drop by truncating
	 * to zero.
	 */
	if (dev_WARN_ONCE(&chan->dev->dev, failure,
			  "Received error packet from hardware: %d\n",
			  failure)) {
		entry->data_len = 0;
		error = 1;
	}

	/*
	 * Return the buffer to the complete list and signal any
	 * waiters that a new TX buffer is available.
	 */
	spin_lock_irqsave(&chan->lock, flags);
	if (error) {
		list_add(&entry->list, &chan->free.fifo);
		chan->free.depth++;
		pchan->stats.discards++;
	} else {
		list_add_tail(&entry->list, &complete->fifo);
		complete->depth++;
		pchan->stats.cdev_pkts++;
	}
	spin_unlock_irqrestore(&chan->lock, flags);

	wake_up_interruptible(wait_queue);
}

/**
 * linkid_cdev_handle_pkt_common() - Common handler for linkid channels.
 * @pchan:		The push channel which hardware sent the entry on.
 * @entry:		The software construct associated with the hardware message.
 * @link_id: 	Link id associated with this message.
 * @length:		The length of the payload in the entry.
 * @failure:	Whether hardware reported an error on the packet.
 */
static void linkid_cdev_handle_pkt_common(struct pp_push_channel *pchan,
				 struct pp_fifo_entry *entry,
				 u32 link_id,
				 int length,
				 int failure)
{
	struct pp_channel *chan = push_chan_to_chan(pchan);
	struct pp_linkid_push_channel *dchan = container_of(pchan,
						struct pp_linkid_push_channel,
						pchan);
	int i, idx = -1;
	unsigned long flags;
	for (i = 0; i < MAX_LINKID_DFS_COUNT; i++) {
		if (dchan->link_id[i] == link_id) {
			idx = i;
			break;
		}
	}

	if (dev_WARN_ONCE(&chan->dev->dev, idx == -1,
			  "Invalid linkid %d detected\n", link_id)) {
		spin_lock_irqsave(&chan->lock, flags);
		list_add(&entry->list, &chan->free.fifo);
		chan->free.depth++;
		spin_unlock_irqrestore(&chan->lock, flags);
		return;
	}

	cdev_handle_pkt_common(pchan,
			       entry,
			       &dchan->user_queue[idx],
			       &dchan->wait_queue[idx],
			       length,
			       failure);
	wake_up_interruptible(&dchan->wait_queue[idx]);
}

/**
 * c4sdu_push_handle_pkt() - Given an entry from hardware, write it to a user FIFO.
 * @pchan:	The push channel which hardware sent the entry on.
 * @entry:	The software construct associated with the hardware message.
 */
static void c4sdu_push_handle_pkt(struct pp_push_channel *pchan,
				 struct pp_fifo_entry *entry)
{
	struct c4sdu_push_descriptor *desc = (struct c4sdu_push_descriptor *)entry->desc;
	u32 link_id = desc->link_id << 24;

	linkid_cdev_handle_pkt_common(pchan,
			entry,
			link_id,
			desc->payload_length,
			desc->dma_status);
}

/**
 * cdec_push_handle_pkt() - Given an entry from hardware, write it to a user FIFO.
 * @pchan:	The push channel which hardware sent the entry on.
 * @entry:	The software construct associated with the hardware message.
 */
static void cdec_push_handle_pkt(struct pp_push_channel *pchan,
				 struct pp_fifo_entry *entry)
{
	struct cdec_push_descriptor *desc = (struct cdec_push_descriptor *)entry->desc;
	u32 link_id = desc->link_id;

	linkid_cdev_handle_pkt_common(pchan,
			entry,
			link_id,
			desc->payload_length,
			desc->dma_status & RX_DESC_DMA_STATUS_ERR_PACKET);
}

/**
 * aap_push_handle_pkt() - Given an entry from hardware, write to a user FIFO.
 * @pchan:	The push channel which hardware sent the entry on.
 * @entry:	The software construct associated with the hardware message.
 */
static void aap_push_handle_pkt(struct pp_push_channel *pchan,
				struct pp_fifo_entry *entry)
{
	struct aap_push_descriptor *desc = (struct aap_push_descriptor *)entry->desc;
	struct pp_channel *chan = push_chan_to_chan(pchan);

	cdev_handle_pkt_common(pchan,
			       entry,
			       &chan->complete,
			       &chan->wait_queue,
			       desc->payload_length,
			       desc->dma_status & RX_DESC_DMA_STATUS_ERR_PACKET);
}

/**
 * net_handle_pkt_common() - Common handler for network packets.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 * @length:	The length of the payload in the entry.
 * @failure:	Whether hardware reported an error on the packet.
 */
static void net_handle_pkt_common(struct pp_push_channel *pchan,
				  struct pp_fifo_entry *entry,
				  int length,
				  int failure)
{
	struct pp_channel *chan = push_chan_to_chan(pchan);
	struct pp_net_push_channel *dchan = container_of(pchan,
						struct pp_net_push_channel,
						pchan);
	unsigned long flags;

	/*
	 * This indicates that the buffer has not been properly freed or cleaned
	 * up.  Drop the buffer entirely, to prevent possible future corruption.
	 */
	if (dev_WARN_ONCE(&chan->dev->dev, entry->data_len,
			  "Entry has non-zero length %zu\n", entry->data_len)) {
		dchan->netdev->stats.rx_errors++;
		dchan->netdev->stats.rx_fifo_errors++;
		pchan->stats.discards++;
		return;
	}

	/*
	 * In the case where we have a descriptor with an
	 * unexpected length, truncate it to zero. This could
	 * indicate corrupted data from the FPGA.
	 */
	if (failure) {
		entry->data_len = 0;
		dchan->netdev->stats.rx_errors++;
	} else if (length > chan->buffer_size) {
		entry->data_len = 0;
		dchan->netdev->stats.rx_errors++;
		dchan->netdev->stats.rx_length_errors++;
	} else {
		/*
		 * Extract the data length and store it.
		 */
		entry->data_len = length;
	}

	/*
	 * Return the buffer to the complete list and signal any
	 * waiters that a new TX buffer is available.
	 */
	spin_lock_irqsave(&chan->lock, flags);
	list_add_tail(&entry->list, &chan->complete.fifo);
	chan->complete.depth++;
	pchan->stats.network_pkts++;
	spin_unlock_irqrestore(&chan->lock, flags);

	napi_schedule(&dchan->napi);
}

/**
 * l3net_push_handle_pkt() - Handler for network packets from l3.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 */
static void l3net_push_handle_pkt(struct pp_push_channel *pchan,
				  struct pp_fifo_entry *entry)
{
	struct l3_push_descriptor *desc = (struct l3_push_descriptor *)entry->desc;

	net_handle_pkt_common(pchan, entry, desc->length, 0);
}

/**
 * l3_push_handle_pkt() - Handler for Catson L3 chardev packets.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 */
static void l3_push_handle_pkt(struct pp_push_channel *pchan,
			       struct pp_fifo_entry *entry)
{
	struct l3_push_descriptor *desc =
		(struct l3_push_descriptor *)entry->desc;
	struct pp_channel *chan = push_chan_to_chan(pchan);

	cdev_handle_pkt_common(pchan,
			       entry,
			       &chan->complete,
			       &chan->wait_queue,
			       desc->length,
			       0);
}

/**
 * cdecnet_push_handle_pkt() - Handler for network packets from cfe.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 */
static void cdecnet_push_handle_pkt(struct pp_push_channel *pchan,
				    struct pp_fifo_entry *entry)
{
	struct cdec_push_descriptor *desc = (struct cdec_push_descriptor *)entry->desc;
	int length = desc->payload_length;

	net_handle_pkt_common(pchan, entry, length,
			      desc->dma_status & RX_DESC_DMA_STATUS_ERR_PACKET);
}

/**
 * c4ethnet_push_handle_pkt() - Handler for network packets from cut4 cfe.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 */
static void c4ethnet_push_handle_pkt(struct pp_push_channel *pchan,
				    struct pp_fifo_entry *entry)
{
	struct c4eth_push_descriptor *desc =
		(struct c4eth_push_descriptor *)entry->desc;
	u32 payload_length = desc->payload_length_upper << 4;
	payload_length |= desc->payload_length_lower;
	net_handle_pkt_common(pchan, entry, payload_length,
			      desc->status);
}

/**
 * snpu_push_handle_pkt() - Handler for incoming packets from Sat NPU.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 */
static void snpu_push_handle_pkt(struct pp_push_channel *pchan,
				 struct pp_fifo_entry *entry)
{
	struct npu_push_descriptor *desc =
		(struct npu_push_descriptor *)entry->desc;
	struct pp_channel *chan = push_chan_to_chan(pchan);

	cdev_handle_pkt_common(pchan,
			       entry,
			       &chan->complete,
			       &chan->wait_queue,
			       desc->length,
			       desc->dma_status & RX_DESC_DMA_STATUS_ERR_PACKET);
}

/**
 * snpunet_push_handle_pkt() - Handler for network packets from Sat NPU.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 */
static void snpunet_push_handle_pkt(struct pp_push_channel *pchan,
				  struct pp_fifo_entry *entry)
{
	struct npu_push_descriptor *desc =
		(struct npu_push_descriptor *)entry->desc;
	int length = desc->length;

	net_handle_pkt_common(pchan, entry, length,
			      desc->dma_status & RX_DESC_DMA_STATUS_ERR_PACKET);
}

/**
 * gnpu_push_handle_pkt() - Handler for incoming Ethernet packets from GNET
 *    NPU.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 */
static void gnpu_push_handle_pkt(struct pp_push_channel *pchan,
				    struct pp_fifo_entry *entry)
{
	struct pp_hybrid_push_channel *dchan = container_of(pchan,
						struct pp_hybrid_push_channel,
						net_dev.pchan);
	struct npu_push_descriptor *desc =
		(struct npu_push_descriptor *)entry->desc;

	if (WARN_ONCE(desc->pkt_type >= GPNU_PACKET_TYPE_MAX,
		      "Unknown packet type 0x%x\n", desc->pkt_type))
		desc->pkt_type = GNPU_PACKET_TYPE_NONE;

	if (desc->pkt_type == GNPU_PACKET_TYPE_SPACEX) {
		cdev_handle_pkt_common(pchan,
				       entry,
				       &dchan->cdev_queue,
				       &dchan->cdev_wait_queue,
				       desc->length,
			      desc->dma_status & RX_DESC_DMA_STATUS_ERR_PACKET);
	} else {
		net_handle_pkt_common(pchan, entry, desc->length,
			      desc->dma_status & RX_DESC_DMA_STATUS_ERR_PACKET);
	}
}

/**
 * pktproc_push_handle_pkt() - Handler for network packets from Sat NPU.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 */
static void pktproc_push_handle_pkt(struct pp_push_channel *pchan,
				    struct pp_fifo_entry *entry)
{
	struct pp_channel *chan = push_chan_to_chan(pchan);
	struct pktproc_push_descriptor *desc =
		(struct pktproc_push_descriptor *)entry->desc;

	cdev_handle_pkt_common(pchan,
			       entry,
			       &chan->complete,
			       &chan->wait_queue,
			       desc->length,
			       desc->dma_status);
}

/**
 * pktprocnet_push_handle_pkt() - Handler for network packets from TLM
 * node FPGA ingress pathway.
 * @pchan:	The push channel a packet was received on.
 * @entry:	The entry received from hardware.
 */
static void pktprocnet_push_handle_pkt(struct pp_push_channel *pchan,
				    struct pp_fifo_entry *entry)
{
	struct pktproc_push_descriptor *desc =
		(struct pktproc_push_descriptor *)entry->desc;
	int length = desc->length;

	net_handle_pkt_common(pchan,
			       entry,
				   length,
			       desc->dma_status & RX_DESC_DMA_STATUS_ERR_PACKET);
}

/**
 * linkid_push_chan_init() - Code for initializing a linkid based channel.
 * @pchan:	The push channel to init.
 * @dev:	The backing platform device.
 * @of_node:	The push channel of_node.
 *
 * @return	0 on success, <0 on failure.
 */
static int linkid_push_chan_init(struct pp_push_channel *pchan,
				 struct platform_device *dev,
				 struct device_node *of_node)
{
	int i;
	struct pp_linkid_push_channel *dchan = container_of(pchan,
						struct pp_linkid_push_channel,
						pchan);

	for (i = 0; i < ARRAY_SIZE(dchan->user_queue); i++) {
		INIT_LIST_HEAD(&dchan->user_queue[i].fifo);
		init_waitqueue_head(&dchan->wait_queue[i]);
	}

	return 0;
}

/**
 * pp_net_napi_poll() - NAPI poll code for RX network completion.
 * @napi:	The NAPI interface for polling packets.
 * @budget:	The maximum number of packets to process.
 *
 * @return	0 on success, <0 on failure.
 */
static int pp_net_napi_poll(struct napi_struct *napi, int budget)
{
	struct pp_net_push_channel *dchan = container_of(napi,
						struct pp_net_push_channel,
						napi);
	struct pp_priv *priv = netdev_priv(dchan->netdev);
	struct pp_push_channel *pchan = &dchan->pchan;
	struct pp_channel *chan = push_chan_to_chan(pchan);
	int packets_handled = 0;
	struct pp_fifo_entry *entry;
	unsigned long flags;
	unsigned short proto;

	spin_lock_irqsave(&chan->lock, flags);
	while (packets_handled < budget &&
	       (entry = list_first_entry_or_null(&chan->complete.fifo,
						 struct pp_fifo_entry,
						 list))) {
		struct sk_buff *skb = entry->priv;

		list_del(&entry->list);
		chan->complete.depth--;

		if (dev_WARN_ONCE(&chan->dev->dev, !skb,
				  "Entry has no SKB allocated\n")) {
			dchan->netdev->stats.rx_errors++;
			goto free_entry;
		}

		spin_unlock_irqrestore(&chan->lock, flags);

		dma_unmap_single(&chan->dev->dev, entry->buffer_phys,
				 chan->buffer_size, DMA_FROM_DEVICE);
		packets_handled++;

		skb_put(skb, entry->data_len);

		if (dchan->header_len > 0) {
			if (entry->data_len <= dchan->header_len) {
				dchan->netdev->stats.rx_dropped++;
				goto free_entry_and_clear_lock;
			}

			skb_pull(skb, dchan->header_len);
		}

		skb->dev = dchan->netdev;

		if (priv->ip_dev) {
			u8 ip_version = skb->len ? (skb->data[0] >> 4) : 0;

			switch (ip_version) {
			case 4:
				proto = htons(ETH_P_IP);
				break;
			case 6:
				proto = htons(ETH_P_IPV6);
				break;
			default:
				dchan->netdev->stats.rx_dropped++;
				goto free_entry_and_clear_lock;
			}

			skb_reset_mac_header(skb);
		} else {
			proto = eth_type_trans(skb, dchan->netdev);
		}

		skb->protocol = proto;

		skb->ip_summed = CHECKSUM_NONE;
		napi_gro_receive(napi, skb);

#ifdef PUSHPULL_ENABLE_TIMING
		pp_timing_update(&chan->timing, entry->timestamp);
#endif

		dchan->netdev->stats.rx_packets++;
		dchan->netdev->stats.rx_bytes += entry->data_len;

		entry->priv = NULL;
free_entry_and_clear_lock:
		spin_lock_irqsave(&chan->lock, flags);
free_entry:
		list_add(&entry->list, &chan->free.fifo);
		chan->free.depth++;
	}

	pchan->stats.usr_queued += pp_push_start_receive(pchan);

	spin_unlock_irqrestore(&chan->lock, flags);

	if (packets_handled < budget) {
		napi_complete(napi);
	}

	return packets_handled;
}

/**
 * net_push_chan_init() - Code for initializing a network based channel.
 * @pchan:	The push channel to init.
 * @dev:	The backing platform device.
 * @of_node:	The push channel of_node.
 *
 * @return	0 on success, <0 on failure.
 */
static int net_push_chan_init(struct pp_push_channel *pchan,
			      struct platform_device *dev,
			      struct device_node *of_node)
{
	struct pp_priv *priv = platform_get_drvdata(dev);
	struct pp_net_push_channel *dchan = container_of(pchan,
						struct pp_net_push_channel,
						pchan);

	dchan->netdev = priv->netdev;

	netif_napi_add(priv->netdev, &dchan->napi, pp_net_napi_poll,
		       NAPI_POLL_WEIGHT);

	napi_enable(&dchan->napi);

	return 0;
}

/**
 * hybrid_push_chan_init() - Code for initializing a network based channel.
 * @pchan:	The push channel to init.
 * @dev:	The backing platform device.
 * @of_node:	The push channel of_node.
 *
 * @return	0 on success, <0 on failure.
 */
static int hybrid_push_chan_init(struct pp_push_channel *pchan,
				 struct platform_device *dev,
				 struct device_node *of_node)
{
	int ret;
	struct pp_hybrid_push_channel *dchan = container_of(pchan,
						struct pp_hybrid_push_channel,
						net_dev.pchan);

	ret = net_push_chan_init(pchan, dev, of_node);
	if (ret < 0)
		return ret;

	INIT_LIST_HEAD(&dchan->cdev_queue.fifo);
	init_waitqueue_head(&dchan->cdev_wait_queue);

	return 0;
}

/**
 * phynet_push_chan_init() - Code for initializing the phynet push channel.
 * @pchan:	The push channel to init.
 * @dev:	The backing platform device.
 * @of_node:	The push channel of_node.
 *
 * @return	0 on success, <0 on failure.
 */
static int phynet_push_chan_init(struct pp_push_channel *pchan,
			      struct platform_device *dev,
			      struct device_node *of_node)
{
	struct pp_net_push_channel *dchan = container_of(pchan,
						struct pp_net_push_channel,
						pchan);

	dchan->header_len = SPACEX_HEADER_LEN;

	return net_push_chan_init(pchan, dev, of_node);
}

/**
 * pp_push_chan_init() - Code for allocating and initializing a push channel.
 * @dev		platform device.
 * @of_node	The device_node containing channel specific metadata.
 * @desc_info	The push information.
 * @resource_idx The resource index for registers and interrupts.
 *
 * @return	Freshly allocated push channel on success, NULL on failure.
 */
static struct pp_push_channel *pp_push_chan_init(struct platform_device *dev,
						 struct device_node *of_node,
						 const struct pp_desc_info *desc_info,
						 u32 resource_idx)
{
	int rc;
	struct pp_channel *chan;
	struct pp_push_channel *pchan;
	u32 fifo_depth;
	u32 fifo_reg;

	if (!desc_info->push_handle_pkt) {
		return NULL;
	}

	/*
	 * Allocate structure.
	 */
	pchan = devm_kzalloc(&dev->dev, PP_PUSH_CHANNEL_SIZE, GFP_KERNEL);
	if (!pchan) {
		return NULL;
	}

	chan = push_chan_to_chan(pchan);

	/*
	 * Perform generic initalization of the channel.
	 */
	rc = pp_chan_init_common(dev, of_node, chan, "rx", resource_idx,
				 desc_info->dynamic_push_buffers, pp_push_irq);
	if (rc < 0) {
		dev_err(&dev->dev, "Failed common init (%d)\n", rc);
		goto free_channel;
	}

	/*
	 * Perform descriptor type specific initialization.
	 */
	if (desc_info->push_extra_init) {
		rc = desc_info->push_extra_init(pchan, dev, of_node);
		if (rc) {
			dev_err(&dev->dev,
				"Failed block specific push init %d\n", rc);
			goto free_channel;
		}
	}
	pchan->push_handle_pkt = desc_info->push_handle_pkt;
	pchan->push_prepare_desc = desc_info->push_prepare_desc;

	/* Configure almost full to mean half full */
	fifo_depth = (push_reg_read(pchan, DMA_PUSH_FIFO_DEPTH_0_REG_OFFSET) &
				DMA_PUSH_FIFO_DEPTH_0__WORK_FIFO_DEPTH_bm) >>
			DMA_PUSH_FIFO_DEPTH_0__WORK_FIFO_DEPTH_bp;

	fifo_reg = push_reg_read(pchan, DMA_PUSH_WORK_FIFO_CTRLx_REG_OFFSET);
	fifo_reg = (fifo_reg & ~DMA_PUSH_WORK_FIFO_CTRLx__AF_THRESH_bm) |
			((fifo_depth / 2) << DMA_PUSH_WORK_FIFO_CTRLx__AE_THRESH_bp);
	push_reg_write(pchan, DMA_PUSH_WORK_FIFO_CTRLx_REG_OFFSET, fifo_reg);

	/*
	 * Set PUSHPULL_MAX_PACKET for receive.
	 */
	push_reg_write(pchan, DMA_PUSH_CTRL1_REG_OFFSET,
		       pchan->chan.buffer_size);

	/*
	 * Set the PUSH minimum transfer size.
	 */
	if (chan->hw_dma_min_transfer) {
		push_reg_write(pchan, DMA_PUSH_CTRL3_REG_OFFSET, chan->hw_dma_min_transfer);
	}
	/*
	 * Read hardware free FIFO depth.
	 */
	chan->hw_fifo_max_depth = (push_reg_read(pchan,
			DMA_PUSH_FIFO_DEPTH_0_REG_OFFSET) &
			DMA_PUSH_FIFO_DEPTH_0__FREE_FIFO_DEPTH_bm) >>
			DMA_PUSH_FIFO_DEPTH_0__FREE_FIFO_DEPTH_bp;

	/*
	 * Create sysfs entries related to the push channel.
	 */
	rc = sysfs_create_group(&dev->dev.kobj, &pp_push_sysfs_regs_group);
	if (rc) {
		dev_err(&dev->dev, "Failed creating reg sysfs (%d)\n", rc);
		goto deinit_channel;
	}

	rc = sysfs_create_group(&dev->dev.kobj, &pp_push_sysfs_stats_group);
	if (rc) {
		dev_err(&dev->dev, "Failed creating stat sysfs (%d)\n", rc);
		goto deinit_channel;
	}

	return pchan;

deinit_channel:
	if (desc_info->push_extra_deinit) {
		desc_info->push_extra_deinit(pchan);
	}
	pp_deinit_channel(dev, push_chan_to_chan(pchan));
free_channel:
	return NULL;
}

/**
 * pp_push_chan_start() - Enables push interface.
 * @pchan:		Push channel to start.
 * @drop_if_no_bufs	Whether we should configure the push to drop packets if
 *			there are no free buffers available.
 *
 * @return	Freshly allocated push channel on success, NULL on failure.
 */
static void pp_push_chan_start(struct pp_push_channel *pchan, bool drop_if_no_bufs)
{
	struct pp_channel *chan = push_chan_to_chan(pchan);
	u32 ctrl0_flags = DMA_PUSH_CTRL0__ENABLE_bm | DMA_PUSH_CTRL0__RESET_STATS_bm;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	pp_push_start_receive(pchan);
	spin_unlock_irqrestore(&chan->lock, flags);

	if (drop_if_no_bufs) {
		ctrl0_flags |= DMA_PUSH_CTRL0__DROP_IF_NO_BUFS_bm;
	}

	/*
	 * Clear any interrupt status then enable each block's interrupts.
	 */
	push_clear_interrupts(pchan);

	/*
	 * Finish clearing interrupts before enabling.
	 */
	wmb();
	push_reg_write(pchan, DMA_PUSH_INTR0_EN_REG_OFFSET,
		       BIT(PUSH_RX_INTC0_CHAN_BIT));

	push_reg_write(pchan, DMA_PUSH_INTR1_EN_REG_OFFSET, PUSH_RX_INTC1_MASK);

	wmb();
	/*
	 * Note that this clears the DISABLE_WORK_FIFO field if set.
	 */
	push_reg_write(pchan, DMA_PUSH_CTRL0_REG_OFFSET, ctrl0_flags);
}

/**
 * cdec_pull_set_descriptor() - Code for translating user request into EUT desc.
 * @pchan	Pull channel to set the descriptor for.
 * @filp:	The file the user made the request on.
 * @buf:	The user buffer to transfer
 * @count:	The size of buf.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet
 *
 * @return	0 on success, <0 on failure.
 */
static int cdec_pull_set_descriptor(struct pp_pull_channel *pchan,
				    void *ctx,
				    size_t count,
				    struct pp_fifo_entry *entry,
				    enum pp_pull_source source)
{
	struct file *filp = (struct file *)ctx;
	struct cdec_pull_descriptor *desc = (struct cdec_pull_descriptor *)entry->desc;
	struct pp_file_ctx *fctx = filp->private_data;

	desc->link_id = fctx->dev_ctx->link_id;
	desc->address = (u32)entry->buffer_phys;
	desc->sw_ext = 0;

	/*
	 * If a padding size was provided, and is greater then
	 * the transmit size, then use the padding. This is intended
	 * to help mitigate hardware issues with variation in packet
	 * size.
	 */
	if (pchan->pad_size > 0 && pchan->pad_size > count) {
		desc->length = pchan->pad_size;
		memset((u8 *)entry->buffer + count, 0, pchan->pad_size - count);
	} else {
		desc->length = count;
	}

	return 0;
}

/**
 * c4sdu_pull_set_descriptor() - Code for translating user request into C4SDU desc.
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 * @count:	The size of the data buffer.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet
 *
 * @return	0 on success, <0 on failure.
 */
static int c4sdu_pull_set_descriptor(struct pp_pull_channel *pchan,
				   void *ctx,
				   size_t count,
				   struct pp_fifo_entry *entry,
				   enum pp_pull_source source)
{
	struct file *filp = (struct file *)ctx;
	struct c4sdu_pull_descriptor *desc = (struct c4sdu_pull_descriptor *)entry->desc;
	struct pp_file_ctx *fctx = filp->private_data;

	desc->link_id = fctx->dev_ctx->link_id >> 24;
	desc->address = (u32)entry->buffer_phys;

	/*
	 * If a padding size was provided, and is greater then
	 * the transmit size, then use the padding. This is intended
	 * to help mitigate hardware issues with variation in packet
	 * size.
	 */
	if (pchan->pad_size > 0 && pchan->pad_size > count) {
		desc->length = pchan->pad_size;
		memset((u8 *)entry->buffer + count, 0, pchan->pad_size - count);
	} else {
		desc->length = count;
	}

	return 0;
}

/**
 * aap_pull_set_descriptor() - Code for translating user request into AAP desc.
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 * @count:	The size of the data buffer.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet.
 *
 * @return	0 on success, <0 on failure.
 */
static int aap_pull_set_descriptor(struct pp_pull_channel *pchan,
				   void *ctx,
				   size_t count,
				   struct pp_fifo_entry *entry,
				   enum pp_pull_source source)
{
	struct aap_pull_descriptor *desc = (struct aap_pull_descriptor *)entry->desc;

	/*
	 * Slicing is not currently supported.
	 */
	desc->slice_bytes = 0;
	desc->address = (u32)entry->buffer_phys;

	/*
	 * If a padding size was provided, and is greater then
	 * the transmit size, then use the padding. This is intended
	 * to help mitigate hardware issues with variation in packet
	 * size.
	 */
	if (pchan->pad_size > 0 && pchan->pad_size > count) {
		desc->length = pchan->pad_size;
		memset((u8 *)entry->buffer + count, 0, pchan->pad_size - count);
	} else {
		desc->length = count;
	}

	return 0;
}

/**
 * l3net_pull_set_descriptor() - Code for translating net request into L3 desc.
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 * @count:	The size of the data buffer.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet.
 *
 * @return	0 on success, <0 on failure.
 */
static int l3net_pull_set_descriptor(struct pp_pull_channel *pchan,
				     void *ctx,
				     size_t count,
				     struct pp_fifo_entry *entry,
				     enum pp_pull_source source)
{
	struct l3dl_pull_descriptor *desc = (struct l3dl_pull_descriptor *)entry->desc;

	desc->ether_type = 0;
	desc->address = (u32)entry->buffer_phys;
	desc->length = count;

	return 0;
}

/**
 * cl3ul_pull_set_descriptor() - Generate packet descriptor for Catson
 *	L3 uplink packets.  (For packets already containing a SpaceX header.)
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 * @count:	The size of the data buffer.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet.
 *
 * @return	0 on success, <0 on failure.
 */
static int cl3ul_pull_set_descriptor(struct pp_pull_channel *pchan,
				     void *ctx,
				     size_t count,
				     struct pp_fifo_entry *entry,
				     enum pp_pull_source source)
{
	struct file *filp = (struct file *)ctx;
	struct pp_file_ctx *fctx = filp->private_data;

	struct l3ul_pull_descriptor *desc =
		(struct l3ul_pull_descriptor *)entry->desc;
	memset(desc, 0, sizeof(struct l3ul_pull_descriptor));

	/*
	 * Set security tunnel and class of service specified.
	 */
	desc->security_tunnel_valid = fctx->security_tunnel_valid;
	desc->security_tunnel = fctx->security_tunnel;
	desc->cos = fctx->cos;

	/*
	 * The first beam on the user terminal is the only beam in use.
	 */
	desc->beam_id = 0;

	/*
	 * Nothing uses packet header suppression currently.
	 */
	desc->phs_low = 0;
	desc->phs_high = 0;
	desc->phs_valid = 0;

	/*
	 * Set the address and length of the entry.
	 */
	desc->address = (u32)entry->buffer_phys;
	desc->length = count;

	return 0;
}

/**
 * snpu_pull_set_descriptor() - Code for translating user request into SNET NPU
 *     desc.
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 * @count:	The size of the data buffer.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet.
 *
 * @return	0 on success, <0 on failure.
 */
static int snpu_pull_set_descriptor(struct pp_pull_channel *pchan,
				    void *ctx,
				    size_t count,
				    struct pp_fifo_entry *entry,
				    enum pp_pull_source source)
{
	struct snpu_pull_descriptor *desc =
		(struct snpu_pull_descriptor *)entry->desc;

	memset(desc, 0, sizeof(struct snpu_pull_descriptor));

	/*
	 * Set the address and length of the user packet, leaving the
	 * rest of the fields as 0.
	 *
	 * This marks both explicit destinations as invalid, which
	 * instructs the NPU to do a forwarding lookup for the packet.
	 */
	desc->address = (u32)entry->buffer_phys;
	desc->length = count;

	return 0;
}

/**
 * gnpu_pull_set_descriptor() - Code for translating net request into NPU
 *     pull desc.
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 * @count:	The size of the data buffer.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet.
 *
 * @return	0 on success, <0 on failure.
 */
static int gnpu_pull_set_descriptor(struct pp_pull_channel *pchan,
				    void *ctx,
				    size_t count,
				    struct pp_fifo_entry *entry,
				    enum pp_pull_source source)
{
	struct gnpu_pull_descriptor *desc =
		(struct gnpu_pull_descriptor *)entry->desc;
	memset(desc, 0, sizeof(struct gnpu_pull_descriptor));

	/*
	 * Explicitly route the packet out to the Ethernet port.
	 * Setting pkt_type = CPU bypasses the forwarding lookup.
	 */
	desc->address = (u32)entry->buffer_phys;
	desc->length = count;

	desc->op_index = 0;
	if (source == PULL_SOURCE_NETWORK) {
		desc->dest_port = GNPU_DEST_ETH_0;
		desc->pkt_type = GNPU_PACKET_TYPE_CPU;
	} else {
		desc->dest_port = GNPU_DEST_SX_LOAD_BALANCE;
		desc->pkt_type = GNPU_PACKET_TYPE_SPACEX;
	}

	return 0;
}

/**
 * pktproc_pull_set_descriptor() - Code for translating net request into NPU
 *     processor pull desc.
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 * @count:	The size of the data buffer.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet.
 *
 * @return	0 on success, <0 on failure.
 */
static int pktproc_pull_set_descriptor(struct pp_pull_channel *pchan,
				       void *ctx,
				       size_t count,
				       struct pp_fifo_entry *entry,
				       enum pp_pull_source source)
{
	struct pktproc_pull_descriptor *desc =
		(struct pktproc_pull_descriptor *)entry->desc;
	memset(desc, 0, sizeof(struct pktproc_pull_descriptor));

	desc->address = (u32)entry->buffer_phys;
	desc->length = count;

	return 0;
}

/**
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 *
 * @return	linkid to use for transmit.
 */
static uint16_t net_pull_get_target_linkid(struct pp_pull_channel *pchan,
					   void *ctx)
{
	struct pp_skb_ctx *skb_ctx = (struct pp_skb_ctx *)ctx;
	struct pp_net_pull_channel *dchan = container_of(pchan,
					      struct pp_net_pull_channel,
					      pchan);

	if (is_multicast_ether_addr(skb_ctx->skb->data) ||
	    is_broadcast_ether_addr((skb_ctx->skb->data))) {
		return dchan->link_id[skb_ctx->transmit_idx];
	} else {
		/*
		 * For routing purposes, the lowest 16 bits of the MAC address
		 * must match the endpoint linkid.
		 */
		return (((const u8 *)skb_ctx->skb->data)[4] << 8) |
				 ((const u8 *)skb_ctx->skb->data)[5];
	}
}

/**
 * cdecnet_pull_set_descriptor() - Code for translating net request into cdec
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 * @count:	The size of the data buffer.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet.
 *
 * @return	0 on success, <0 on failure.
 */
static int cdecnet_pull_set_descriptor(struct pp_pull_channel *pchan,
				       void *ctx,
				       size_t count,
				       struct pp_fifo_entry *entry,
				       enum pp_pull_source source)
{
	struct cdec_pull_descriptor *desc = (struct cdec_pull_descriptor *)entry->desc;

	desc->link_id = net_pull_get_target_linkid(pchan, ctx);
	desc->address = (u32)entry->buffer_phys;
	desc->length = count;
	desc->sw_ext = 0;

	return 0;
}

/**
 * phynet_pull_set_descriptor() - Code for translating net request into cdec
 * @pchan	Pull channel to set the descriptor for.
 * @ctx:	Context for initializing the descriptor.
 * @count:	The size of the data buffer.
 * @entry:	The fifo entry to use for setting up the descriptor.
 * @source:	Source of the packet.
 *
 * @return	0 on success, <0 on failure.
 */
static int phynet_pull_set_descriptor(struct pp_pull_channel *pchan,
				       void *ctx,
				       size_t count,
				       struct pp_fifo_entry *entry,
				       enum pp_pull_source source)
{
	struct cdec_pull_descriptor *desc =
		(struct cdec_pull_descriptor *)entry->desc;

	/* Link_id is ignored in upstream direction. */
	desc->link_id = 0;
	desc->address = (u32)entry->buffer_phys;
	desc->length = count;
	desc->sw_ext = 0;

	return 0;
}

static bool phynet_pull_encode_header(struct pp_pull_channel *pchan,
						void *buffer,
						const struct sk_buff *skb)
{
	const struct iphdr *iph;
	__be32 saddr;
	__be32 daddr;

	/*
	 * A fixed SpaceX header.
	 *
	 * Fixed parameters:
	 * last_header (bit 3) = true
	 * TTL (bits 16-21) = 0x3F
	 * CoS (bits 23-25) = mac accelerator (0x6)
	 *
	 * Labels:
	 * Bytes 9-12, 17-18: Source SpaceX label.
	 * Bytes 13-16, 19-20: Destination SpaceX label.
	 *
	 * The middle two bytes of each label are the 'component'and the
	 * remaining bytes are the routing prefix. The routing prefix for
	 * link-local packets is 0xFFFFFFFFF.
	 *
	 * The 'component' is the lower two bytes of the IPv4 address of
	 * the node.
	 */
	uint8_t header_bytes[SPACEX_HEADER_LEN] = {
		0x10, 0x00, 0xfd, 0x80,
		0x00, 0x00, 0x00, 0x00,
		0xff, 0xff, 0x00, 0x00,
		0xff, 0xff, 0x00, 0x00,
		0xff, 0xff, 0xff, 0xff
	};

	if (skb->protocol != htons(ETH_P_IP))
		return false;

	iph = ip_hdr(skb);
	saddr = ntohl(iph->saddr);
	daddr = ntohl(iph->daddr);

	header_bytes[10] = (daddr >> 8) & 0xFF;
	header_bytes[11] = daddr & 0xFF;
	header_bytes[14] = (saddr >> 8) & 0xFF;
	header_bytes[15] = saddr & 0xFF;

	memcpy(buffer, header_bytes, sizeof(header_bytes));

	return true;
}

/**
 * net_pull_chan_init_common() - Common code for initializing a network channel.
 * @pchan	The pull channel to init.
 * @dev		platform device.
 * @of_node	The device_node containing channel specific metadata.
 *
 * @return	0 on success, <0 on failure.
 */
static int net_pull_chan_init_common(struct pp_pull_channel *pchan,
				     struct platform_device *dev,
				     struct device_node *of_node)
{
	struct pp_net_pull_channel *dchan = container_of(pchan,
					     struct pp_net_pull_channel,
					     pchan);
	struct pp_priv *priv = platform_get_drvdata(dev);

	dchan->netdev = priv->netdev;

	/* If a more specific init has not specified multiple endpoints */
	if (!pchan->endpoints) {
		pchan->endpoints = 1;
	}

	return 0;
}

/**
 * linkid_net_pull_chan_init() - Code for initializing a linkid network channel.
 * @pchan	The pull channel to init.
 * @dev		platform device.
 * @of_node	The device_node containing channel specific metadata.
 *
 * @return	0 on success, <0 on failure.
 */
static int linkid_net_pull_chan_init(struct pp_pull_channel *pchan,
				     struct platform_device *dev,
				     struct device_node *of_node)
{
	struct pp_net_pull_channel *dchan = container_of(pchan,
					     struct pp_net_pull_channel,
					     pchan);
	if (!of_node->parent) {
		return -1;
	}

	/*
	 * Pad out packets to create standard ethernet frames.
	 */
	if (pchan->pad_size < ETH_ZLEN + ETH_FCS_LEN) {
		pchan->pad_size = ETH_ZLEN + ETH_FCS_LEN;
	}

	pchan->endpoints = of_property_read_variable_u32_array(
						of_node->parent, "link-id",
						dchan->link_id, 1,
						MAX_LINKID_DFS_COUNT);
	if (pchan->endpoints < 0)
		return pchan->endpoints;

	return net_pull_chan_init_common(pchan, dev, of_node);
}

/**
 * phynet_pull_chan_init() - Code for initializing a phynet linkid network
 * channel.
 * @pchan	The pull channel to init.
 * @dev		platform device.
 * @of_node	The device_node containing channel specific metadata.
 *
 * @return	0 on success, <0 on failure.
 */
static int phynet_pull_chan_init(struct pp_pull_channel *pchan,
				     struct platform_device *dev,
				     struct device_node *of_node)
{
	struct pp_net_pull_channel *dchan = container_of(pchan,
					     struct pp_net_pull_channel,
					     pchan);

	dchan->header_len = SPACEX_HEADER_LEN;

	return linkid_net_pull_chan_init(pchan, dev, of_node);
}

/**
 * pp_pull_chan_init() - Code for initializing a pull channel.
 * @dev		platform device.
 * @of_node	The device_node containing channel specific metadata.
 * @desc_info	The pull descriptor information to use.
 * @resource_idx The resource index for registers and interrupts.
 *
 * @return	Freshly allocated pull channel on success, NULL on failure.
 */
static struct pp_pull_channel *pp_pull_chan_init(struct platform_device *dev,
						 struct device_node *of_node,
						 const struct pp_desc_info *desc_info,
						 u32 resource_idx)
{
	int rc;
	struct pp_channel *chan;
	struct pp_pull_channel *pchan;
	u32 free_fifo_depth;
	u32 free_fifo_thresh;

	if (!desc_info->pull_encode_descriptor) {
		return NULL;
	}

	/*
	 * Allocate the pull channel.
	 */
	pchan = devm_kzalloc(&dev->dev, PP_PULL_CHANNEL_SIZE, GFP_KERNEL);
	if (!pchan) {
		return NULL;
	}

	chan = pull_chan_to_chan(pchan);

	/*
	 * Set up descriptor specific functionality.
	 */
	if (desc_info->pull_extra_init) {
		rc = desc_info->pull_extra_init(pchan, dev, of_node);
		if (rc) {
			dev_err(&dev->dev, "Failed extra init (%d)\n", rc);
			goto free_channel;
		}
	}
	pchan->encode_header = desc_info->net_pull_encode_header;
	pchan->encode_descriptor = desc_info->pull_encode_descriptor;
	pchan->extra_work = desc_info->pull_extra_work;

	/*
	 * Read the optional parameter on how much to pad out each transfer.
	 */
	of_property_read_u32(of_node, "pad-size", &pchan->pad_size);

	/*
	 * Perform common initalization.
	 */
	rc = pp_chan_init_common(dev, of_node, chan, "tx", resource_idx,
				 false, pp_pull_irq);
	if (rc) {
		dev_err(&dev->dev, "Failed common init (%d)\n", rc);
		goto free_channel;
	}

	if (pchan->pad_size > chan->buffer_size) {
		dev_err(&dev->dev, "pad-size' too large\n");
		goto deinit_channel;
	}

	/*
	 * Create pull specific sysfs debug files.
	 */
	rc = sysfs_create_group(&dev->dev.kobj, &pp_pull_sysfs_regs_group);
	if (rc) {
		dev_err(&dev->dev, "Failed creating reg sysfs (%d)\n", rc);
		goto deinit_channel;
	}

	rc = sysfs_create_group(&dev->dev.kobj, &pp_pull_sysfs_stats_group);
	if (rc) {
		dev_err(&dev->dev, "Failed creating stat sysfs (%d)\n", rc);
		goto deinit_channel;
	}

	/*
	 * Read hardware work FIFO depth.
	 */
	chan->hw_fifo_max_depth = (pull_reg_read(pchan,
			DMA_PULL_FIFO_DEPTH_REG_OFFSET) &
			DMA_PULL_FIFO_DEPTH__WORK_FIFO_DEPTH_bm) >>
			DMA_PULL_FIFO_DEPTH__WORK_FIFO_DEPTH_bp;

	/* Interrupt Coalecsing: Don't interrupt on every pull completion. */
	free_fifo_depth = (pull_reg_read(pchan, DMA_PULL_FIFO_DEPTH_REG_OFFSET) &
				DMA_PULL_FIFO_DEPTH__FREE_FIFO_DEPTH_bm) >>
			  DMA_PULL_FIFO_DEPTH__FREE_FIFO_DEPTH_bc;
	free_fifo_thresh = (pull_reg_read(pchan,
					  DMA_PULL_FREE_FIFO_CTRL_REG_OFFSET) &
			    ~DMA_PULL_FREE_FIFO_CTRL__AF_THRESH_bm) |
			   (min(32U, free_fifo_depth / 2) <<
				DMA_PULL_FREE_FIFO_CTRL__AF_THRESH_bc);
	pull_reg_write(pchan, DMA_PULL_FREE_FIFO_CTRL_REG_OFFSET,
		       free_fifo_thresh);

	return pchan;

deinit_channel:
	pp_deinit_channel(dev, chan);
free_channel:
	return NULL;
}

/**
 * l3ul_pull_ioctl() - Handle ioctl commands for the L3UL pull.
 * @flip:	The file to set the ioctl for.
 * @cmd:	The ioctl command sent from userspace.
 * @arg:	The argument sent from userspace.
 *
 * @return	0 on success, <0 on failure.
 */
static long l3ul_pull_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pp_file_ctx *fctx = filp->private_data;

	if (WARN_ONCE(!fctx, "Failed to lookup device\n")) {
		return -ENODEV;
	}

	switch (cmd) {
	case L3UL_PULL_SET_TUNNEL_VALID:
		if (arg > 1) {
			return -EINVAL;
		}

		fctx->security_tunnel_valid = arg;
		return 0;
	case L3UL_PULL_SET_TUNNEL_INDEX:
		fctx->security_tunnel = arg;
		return 0;
	case L3UL_PULL_SET_COS:
		fctx->cos = arg;
		return 0;
	}

	return -EINVAL;
}

/**
 * pp_pull_chan_start() - Enables pull interface.
 * @pchan:	Pull channel to start.
 */
static void pp_pull_chan_start(struct pp_pull_channel *pchan)
{
	/*
	 * Clear interrupts.
	 */
	pull_clear_interrupts(pchan);

	/*
	 * Finish clearing interrupts before enabling.
	 */
	wmb();
	pull_reg_write(pchan, DMA_PULL_INTR0_EN_REG_OFFSET,
		       PULL_TX_INTC0_MASK);
}

/**
 * pp_chrdev_register_file -Generic creation of a push pull character device.
 * @priv:	private context structure.
 * @fctx:	file context to initialize.
 * @name:	optional name to call the file.
 *
 * @return	0 on success, < 0 on failure.
 *
 * Context specific parameters must be filled out before this function is
 * called, as the character device may be used before this function returns.
 */
static int pp_chrdev_register_file(struct pp_priv *priv,
				   struct pp_dev_ctx *fctx,
				   const char *name)
{
	const struct file_operations *fops;
	int minor = 0;
	struct device *device;
	dev_t dev_id;
	int rc = 0;

	if (major_dev_id == 0) {
		dev_err(priv->dev, "Bad major_dev_id (%d)\n", rc);
		return -EBADF;
	}

	if (priv->push && !fctx->user_queue) {
		dev_err(priv->dev, "Push file requested without user interface\n");
		return -EINVAL;
	}

	/*
	 * Initialize the context.
	 */
	fctx->priv = priv;
	fctx->rd_lock = 0;

	if (priv->push && priv->pull) {
		fops = &pp_pushpull_fops;
	} else if (priv->push) {
		fops = &pp_push_fops;
	} else if (priv->pull) {
		fops = &pp_pull_fops;
	} else {
		/*
		 * This should never happen.  It would mean both a bad device
		 * tree and upper levels had not checked.
		 */
		dev_err(priv->dev, "Could not define fops, no push or pull\n");
		return -EINVAL;
	}

	if (mutex_lock_interruptible(&registered_minors_lock)) {
		return -ERESTARTSYS;
	}

	minor = find_first_zero_bit(registered_minors, MAX_PUSHPULL_MINORS);
	if (minor == MAX_PUSHPULL_MINORS) {
		dev_err(priv->dev, "No free slot for chrdev\n");
		rc = -EEXIST;
		goto out_mutex_unlock;
	}

	fctx->minor = minor;

	/*
	 * Make sure to initialize the cdev last, inc ase it immediately
	 * gets called.
	 */
	dev_id = MKDEV(MAJOR(major_dev_id), minor);
	cdev_init(&fctx->cdev, fops);
	rc = cdev_add(&fctx->cdev, dev_id, 1);
	if (rc) {
		fctx->priv = NULL;
		dev_err(priv->dev, "Failed to add char device (%d)\n", rc);
		goto out_mutex_unlock;
	}

	/*
	 * Create the device.
	 */
	if (name)
		device = device_create(pp_class, NULL, dev_id, NULL,
				       "pushpull_%s", name);
	else
		device = device_create(pp_class, NULL, dev_id, NULL,
				       "pushpull%d", minor);
	if (IS_ERR(device)) {
		cdev_del(&fctx->cdev);
		rc = PTR_ERR(device);
		dev_err(priv->dev, "Failed to create char device (%d)\n", rc);
	} else {
		set_bit(minor, registered_minors);
	}

out_mutex_unlock:
	mutex_unlock(&registered_minors_lock);

	return rc;
}

/**
 * pp_chrdev_register_simple - Single file create for no context pushpull.
 * @priv:	private context structure.
 *
 * @return	0 on success, < 0 on failure.
 */
static int pp_chrdev_register_simple(struct pp_priv *priv)
{
	int rc;
	const char *suffix_str = NULL;

	/* Read an optional suffix for /dev readability. */
	if (of_property_read_string(priv->dev->of_node, "suffix", &suffix_str)) {
		dev_dbg(priv->dev, "Missing optional parameter 'suffix'\n");
	}

	priv->dev_ctx_count = 1;
	priv->dev_ctx = devm_kzalloc(priv->dev, sizeof(struct pp_dev_ctx),
				      GFP_KERNEL);
	if (!priv->dev_ctx)
		return -ENOMEM;

	/*
	 * For the simple use case with a single user_queue corresponding to
	 * a single device file entry, reach into the push channel for a
	 * reference to the completion queue.  This breaks the abstraction layer
	 * between the top level and push layers, but saves lots of additional
	 * code and complexity involved in creating a push level registration
	 * callback.
	 */
	if (pp_priv_to_push(priv)) {
		priv->dev_ctx->user_queue = &push_chan_to_chan(pp_priv_to_push(priv))->complete;
		priv->dev_ctx->wait_queue = &push_chan_to_chan(pp_priv_to_push(priv))->wait_queue;
	}

	rc = pp_chrdev_register_file(priv, priv->dev_ctx, suffix_str);
	if (rc) {
		priv->dev_ctx_count = 0;
		priv->dev_ctx = NULL;
	}

	return rc;
}

/**
 * pp_chrdev_register_lid - Single file create for linkid defined pushpull.
 * @priv:		private context structure.
 *
 * @return	0 on success, < 0 on failure.
 */
static int pp_chrdev_register_lid(struct pp_priv *priv)
{
	int rc;
	u32 link_id[MAX_LINKID_DFS_COUNT];
	int i, j;
	const char *suffix_str = NULL;
	struct pp_push_channel *push_chan = pp_priv_to_push(priv);
	struct pp_linkid_push_channel *dchan = push_chan ?
						container_of(push_chan,
						struct pp_linkid_push_channel,
						pchan) :
						NULL;
	int linkid_count = 0;
	int modem_count = 0;

	/* Read an optional suffix for /dev readability. */
	if (of_property_read_string(priv->dev->of_node, "suffix",
				    &suffix_str)) {
		dev_dbg(priv->dev, "Missing required parameter 'suffix'\n");
		return -EINVAL;
	}

	/* Read the link ID to set when transmitting packets. */
	linkid_count = of_property_read_variable_u32_array(priv->dev->of_node,
							"link-id", link_id, 1,
							MAX_LINKID_DFS_COUNT);
	if (linkid_count <= 0) {
		dev_err(priv->dev, "Invalid required parameter 'link-id'\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(priv->dev->of_node, "modem-count",
				  &modem_count);
	if (rc) {
		modem_count = 1;
	} else if (modem_count < 1 || modem_count > 16) {
		dev_err(priv->dev, "Invalid modem-count of %d\n", modem_count);
		return -EINVAL;
	}

	priv->dev_ctx_count = linkid_count * modem_count;
	priv->dev_ctx = devm_kzalloc(priv->dev,
			      sizeof(struct pp_dev_ctx) * priv->dev_ctx_count,
			      GFP_KERNEL);
	if (!priv->dev_ctx)
		return -ENOMEM;

	for (i = 0; i < linkid_count; i++) {
		if (link_id[i] > 0xFFFF || link_id[i] & 0xF) {
			dev_err(priv->dev, "Invalid linkid %x\n", link_id[i]);
			priv->dev_ctx_count = i;
			priv->desc_info->if_unregister(priv);
			return -EINVAL;
		}

		for (j = 0; j < modem_count; j++) {
			int offset;
			int idx = i * modem_count + j;
			char suffix_name[32];
			char *name_ptr = suffix_name;
			int length = sizeof(suffix_name);

			priv->dev_ctx[idx].link_id = link_id[i] + j;

			if (push_chan) {
				priv->dev_ctx[idx].user_queue = &dchan->user_queue[idx];
				priv->dev_ctx[idx].wait_queue = &dchan->wait_queue[idx];
				dchan->link_id[idx] = priv->dev_ctx[idx].link_id;
			}

			offset = snprintf(name_ptr, length, "%s", suffix_str);
			name_ptr += offset;
			length -= offset;

			if (linkid_count > 1) {
				offset = snprintf(name_ptr, length, "_%03x",
						  priv->dev_ctx[idx].link_id >> 4);
				name_ptr += offset;
				length -= offset;
			}

			if (modem_count > 1) {
				offset = snprintf(name_ptr, length, "_%01x", j);
				name_ptr += offset;
				length -= offset;
			}

			rc = pp_chrdev_register_file(priv, &priv->dev_ctx[idx],
						     suffix_name);
			if (rc) {
				priv->dev_ctx_count = i;
				priv->desc_info->if_unregister(priv);
				return rc;
			}
		}
	}

	return 0;
}

/**
 * pp_chrdev_unregister_device() - removes a push-pull device.
 * @priv:	private device data.
 *
 * This removes the device from the minor-number lookup table.
 */
static void pp_chrdev_unregister_device(struct pp_priv *priv)
{
	int i;

	mutex_lock(&registered_minors_lock);
	for (i = 0; i < priv->dev_ctx_count; i++) {
		device_destroy(pp_class, priv->dev_ctx[i].cdev.dev);
		cdev_del(&priv->dev_ctx[i].cdev);
		clear_bit(priv->dev_ctx[i].minor, registered_minors);
	}
	mutex_unlock(&registered_minors_lock);

	priv->dev_ctx_count = 0;
	priv->dev_ctx = NULL;
}

/**
 * pp_register_net() - Register the pushpull net dev with the network subsystem.
 * @priv:       private device data.
 *
 * Returns using register_netdev return convention.
 */
static int pp_register_net(struct pp_priv *priv)
{
	return register_netdev(priv->netdev);
}

/**
 * pp_register_hybrid() - Register the pushpull net dev and char dev
 * @priv:       private device data.
 *
 * Returns using register_netdev return convention.
 */
static int pp_register_hybrid(struct pp_priv *priv)
{
	int ret;
	struct pp_push_channel *push_chan = pp_priv_to_push(priv);
	struct pp_hybrid_push_channel *dchan = push_chan ?
						container_of(push_chan,
						struct pp_hybrid_push_channel,
						net_dev.pchan) :
						NULL;

	ret = register_netdev(priv->netdev);
	if (ret < 0)
		return ret;

	ret = pp_chrdev_register_simple(priv);
	if (ret < 0) {
		unregister_netdev(priv->netdev);
		return ret;
	}

	if (dchan) {
		priv->dev_ctx->user_queue = &dchan->cdev_queue;
		priv->dev_ctx->wait_queue = &dchan->cdev_wait_queue;
	}

	return 0;
}

/**
 * pp_unregister_net() - Unegister the pushpull net dev
 * @priv:       private device data.
 */
static void pp_unregister_net(struct pp_priv *priv)
{
	unregister_netdev(priv->netdev);
}

/**
 * pp_unregister_hybrid() - Unegister the pushpull net dev and char dev
 * @priv:       private device data.
 */
static void pp_unregister_hybrid(struct pp_priv *priv)
{
	pp_chrdev_unregister_device(priv);

	pp_unregister_net(priv);
}

/**
 * pp_open() - opens a push-pull char device.
 * @inode:	the &struct inode the user wants to open.
 * @filp:	pointer to &struct file to be initialized.
 *
 * @return	zero on success, non-zero on failure.
 */
static int pp_open(struct inode *inode, struct file *filp)
{
	struct pp_file_ctx *fctx;
	struct pp_dev_ctx *dctx = container_of(inode->i_cdev,
						struct pp_dev_ctx, cdev);
	if (WARN_ONCE(!dctx->priv, "Failed to lookup file\n")) {
		return -ENODEV;
	}

	if ((filp->f_flags & O_ACCMODE) != O_WRONLY && test_and_set_bit_lock(0, &dctx->rd_lock)) {
		dev_err(dctx->priv->dev, "only one readable file is allowed\n");
		return -EPERM;
	}

	fctx = kzalloc(sizeof(struct pp_file_ctx),
				      GFP_KERNEL);
	if (!fctx)
		return -ENOMEM;

	fctx->dev_ctx = dctx;
	filp->private_data = fctx;

	return 0;
}

/**
 * pp_release() - closes a push-pull char device.
 * @inode:	the actual &struct inode the user is interacting with.
 * @filp:	pointer to the &struct file to be closed.
 *
 * @return	zero on success, non-zero on failure.
 *
 * This function is called when the last reference to an open file is closed.
 * Recall that fork()ing makes multiple references to the same file
 * descriptor in userspace (maps to a &struct file in the kernel).  So calling
 * close() won't necessarily call this function (until the last instance of
 * that particular file descriptor is closed).
 */
static int pp_release(struct inode *inode, struct file *filp)
{
	struct pp_file_ctx *fctx = filp->private_data;

	if ((filp->f_flags & O_ACCMODE) != O_WRONLY)
		clear_bit_unlock(0, &fctx->dev_ctx->rd_lock);

	kfree(fctx);
	filp->private_data = NULL;

	return 0;
}

/**
 * push_read() - reads from a push fifo.
 * @pchan:	The push channel to read from.
 * @filp:	&struct file pointer for the device file.
 * @fctx:	The file context this read is running from.
 * @buf:	userspace buffer.
 * @count:	maximum number of bytes to read.
 * @pos:	position in the file (ignored -- this is a stream).
 *
 * @return	number of bytes read on success, negative value on failure.
 */
static ssize_t push_read(struct pp_push_channel *pchan,
			 struct file *filp,
			 struct pp_dev_ctx *fctx,
			 char __user *buf,
			 size_t count,
			 loff_t *pos)
{
	struct pp_channel *chan = push_chan_to_chan(pchan);
	struct pp_fifo_entry *entry;
	ssize_t rc = 0;
	struct fifo_wait *read_fifo = fctx->user_queue;
	unsigned long flags;

	pchan->stats.usr_reads++;

	/*
	 * As we can potentially drop packets and wait multiple times before
	 * returning a buffer, loop on the return code being zero.
	 */
	while (rc == 0) {
		/* Wait for data. */
		while (list_empty(&read_fifo->fifo)) {
			if (filp->f_flags & O_NONBLOCK) {
				return -EAGAIN;
			}

			dev_dbg(&chan->dev->dev,
				"PID %d (%s) going to sleep on read\n",
				current->pid, current->comm);

			if (wait_event_interruptible((*fctx->wait_queue),
					(!list_empty(&read_fifo->fifo)))) {
				dev_dbg(&chan->dev->dev,
					"PID %d (%s) caught a signal while waiting on read()\n",
					current->pid, current->comm);
				/* pass it up to the FS layer */
				return -ERESTARTSYS;
			}
		}

		spin_lock_irqsave(&chan->lock, flags);

		entry = list_first_entry_or_null(&read_fifo->fifo,
						 struct pp_fifo_entry,
						 list);
		/* Someone else must have grabbed the entry. */
		if (!entry) {
			rc = 0;
			spin_unlock_irqrestore(&chan->lock, flags);
			continue;
		}

		list_del(&entry->list);
		read_fifo->depth--;
		spin_unlock_irqrestore(&chan->lock, flags);

		/* Copy data into caller buffer if size is reasonable,
		 * otherwise, drop the packet and continue.
		 */
		if ((entry->data_len > 0) && (entry->data_len <= count)) {
			rc = copy_to_user(buf, entry->buffer,
					  entry->data_len);

			/* Report and continue. */
			if (dev_WARN_ONCE(&chan->dev->dev, rc,
					  "Failed to copy data to user: %zd",
					  rc)) {
				rc = -EFAULT;
			} else {
				rc = entry->data_len;
			}
		} else {
			/* In this case, explicitly continue, as the
			 * caller is still expecting us to return a
			 * buffer, but we still need to handle returning
			 * the buffer to the free fifo, so continue is
			 * not appropriate.
			 */
			rc = 0;
		}

#ifdef PUSHPULL_ENABLE_TIMING
		pp_timing_update(&chan->timing, entry->timestamp);
#endif

		/* Return the buffer to the free_fifo */
		spin_lock_irqsave(&chan->lock, flags);
		list_add(&entry->list, &chan->free.fifo);
		chan->free.depth++;
		pchan->stats.usr_queued += pp_push_start_receive(pchan);
		spin_unlock_irqrestore(&chan->lock, flags);
	}

	if (rc > 0)
		pchan->stats.usr_pkts_read++;

	return rc;
}

/**
 * pp_read() - reads from a pull fifo.
 * @filp:	&struct file pointer for the device file.
 * @buf:	userspace buffer.
 * @count:	maximum number of bytes to read.
 * @pos:	position in the file (ignored -- this is a stream).
 *
 * @return	number of bytes read on success, negative value on failure.
 */
static ssize_t pp_read(struct file *filp, char __user *buf, size_t count,
		       loff_t *pos)
{
	struct pp_file_ctx *fctx = filp->private_data;

	if (WARN_ONCE(!fctx->dev_ctx || !fctx->dev_ctx->priv || !pp_priv_to_push(fctx->dev_ctx->priv),
		      "Failed to get file context\n")) {
		return -ENODEV;
	}

	return push_read(pp_priv_to_push(fctx->dev_ctx->priv), filp, fctx->dev_ctx, buf, count,
			 pos);
}

/**
 * pull_write() - writes to a pull fifo.
 * @pchan:	The pull channel to write to.
 * @filp:	&struct file pointer for the device file.
 * @buf:	userspace buffer.
 * @count:	maximum number of bytes to write.
 * @pos:	position in the file (ignored -- this is a stream).
 *
 * @return	number of bytes written on success, negative value on failure.
 */
static ssize_t pull_write(struct pp_pull_channel *pchan, struct file *filp,
			  const char __user *buf, size_t count, loff_t *pos)
{
	struct pp_channel *chan = pull_chan_to_chan(pchan);
	struct pp_fifo_entry *entry = NULL;
	ssize_t rc = 0;
	ssize_t usr_blocked = 0;
	unsigned long flags;

	pchan->stats.usr_writes++;

	if (count > chan->buffer_size) {
		dev_err(&chan->dev->dev, "write too large - %zu", count);
		return -EFBIG;
	}

	/* Get a free buffer. */
	while (!entry) {
		/*
		 * Only increment the stat once per pull_write.
		 */
		if (list_empty(&chan->free.fifo)) {
			if (!usr_blocked) {
				pchan->stats.usr_blocked++;
				usr_blocked = 1;
			}

			if (filp->f_flags & O_NONBLOCK) {
				rc = -EAGAIN;
				goto out;
			}
		}

		if (wait_event_interruptible(
			chan->wait_queue,
			(!list_empty(&chan->free.fifo)))) {
			dev_dbg(
			    &chan->dev->dev,
			    "PID %d (%s) caught a signal while waiting on write()\n",
			    current->pid, current->comm);
			/* pass it up to the FS layer */
			rc = -ERESTARTSYS;
			goto out;
		}

		spin_lock_irqsave(&chan->lock, flags);
		entry = list_first_entry_or_null(&chan->free.fifo,
						 struct pp_fifo_entry,
						 list);
		if (entry) {
			list_del(&entry->list);
			chan->free.depth--;
		}
		spin_unlock_irqrestore(&chan->lock, flags);
	}

	rc = pchan->encode_descriptor(pchan, filp, count, entry,
				      PULL_SOURCE_CHR_DEV);

	if (dev_WARN_ONCE(&chan->dev->dev,
			  copy_from_user(entry->buffer, buf, count),
			  "Failed to copy data from user\n")) {
		rc = -EFAULT;
	}

	spin_lock_irqsave(&chan->lock, flags);
	if (rc == 0) {
#ifdef PUSHPULL_ENABLE_TIMING
		entry->timestamp = ktime_get();
#endif
		list_add_tail(&entry->list, &chan->used.fifo);
		chan->used.depth++;

		pchan->stats.usr_queued += pp_pull_start_transmit(pchan);
		pchan->stats.cdev_pkts++;
	} else {
		list_add(&entry->list, &chan->free.fifo);
	}
	spin_unlock_irqrestore(&chan->lock, flags);

out:
	return rc ? rc : count;
}

/**
 * pp_write() - writes to a push fifo.
 * @filp:	&struct file pointer for the device file.
 * @buf:	userspace buffer to fill.
 * @count:	maximum number of bytes to write.
 * @pos:	position in the file (ignored -- this is a stream).
 *
 * @return	number of bytes read on success, negative value on failure.
 */
static ssize_t pp_write(struct file *filp,
			const char __user *buf,
			size_t count,
			loff_t *pos)
{
	struct pp_file_ctx *fctx = filp->private_data;

	if (WARN_ONCE(!fctx->dev_ctx || !fctx->dev_ctx->priv || !pp_priv_to_pull(fctx->dev_ctx->priv),
		      "Failed to get file context\n")) {
		return -ENODEV;
	}

	return pull_write(pp_priv_to_pull(fctx->dev_ctx->priv), filp, buf, count, pos);
}

/**
 * pull_poll() - queries whether the device appears to be writable.
 * @filp:	the &struct file we are querying.
 * @wait:	the wait-table; if device is not ready, the process can
 *		optionally be added to the table, sleeping to wait for
 *		readiness.  This parameter can be NULL (e.g. if no waiting
 *		is desired).
 *
 * @return	mask of flags as to ready state.
 */
static unsigned int pp_pull_poll(struct file *filp,
				 struct poll_table_struct *wait)
{
	struct pp_file_ctx *fctx = filp->private_data;
	struct pp_pull_channel *pchan = pp_priv_to_pull(fctx->dev_ctx->priv);
	struct pp_channel *chan = pull_chan_to_chan(pchan);
	unsigned int mask = 0;

	if (WARN_ONCE(!fctx->dev_ctx->priv || !pp_priv_to_pull(fctx->dev_ctx->priv),
		      "Failed to get file context\n")) {
		return -ENODEV;
	}

	/*
	 * Register for the revelant wait queues.
	 */
	poll_wait(filp, &chan->wait_queue, wait);

	/*
	 * If there's an available transmit buffer, then it is ok to call
	 * write().
	 */
	if (!list_empty(&chan->free.fifo)) {
		mask |= POLLOUT | POLLWRNORM;
	}

	return mask;
}

/**
 * push_poll() - queries whether the device appears to be writable.
 * @filp:	the &struct file we are querying.
 * @wait:	the wait-table; if device is not ready, the process can
 *		optionally be added to the table, sleeping to wait for
 *		readiness.  This parameter can be NULL (e.g. if no waiting
 *		is desired).
 *
 * @return	mask of flags as to ready state.
 */
static unsigned int pp_push_poll(struct file *filp,
				 struct poll_table_struct *wait)
{
	struct pp_file_ctx *fctx = filp->private_data;
	struct fifo_wait *read_fifo = fctx->dev_ctx->user_queue;
	unsigned int mask = 0;

	if (WARN_ONCE(!fctx->dev_ctx->priv, "Failed to get file context\n")) {
		return -ENODEV;
	}

	/*
	 * Register for the revelant wait queues.
	 */
	poll_wait(filp, fctx->dev_ctx->wait_queue, wait);

	/*
	 * If the receive complete fifo is not empty, then there is a packet
	 * to read().
	 */
	if (!list_empty(&read_fifo->fifo)) {
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

/**
 * pp_poll() - queries whether the device appears to be readable/writable
 * @filp:	the &struct file we are querying.
 * @wait:	the wait-table; if device is not ready, the process can
 *		optionally be added to the table, sleeping to wait for
 *		readiness.  This parameter can be NULL (e.g. if no waiting
 *		is desired).
 *
 * @return	mask of flags as to ready state.
 */
static unsigned int pp_poll(struct file *filp,
			    struct poll_table_struct *wait)
{
	return  pp_push_poll(filp, wait) | pp_pull_poll(filp, wait);
}

/**
 * pp_push_handle_interrupt() - handle interrupts from the push (rx) block
 * @pchan:	The channel to handle the interrupt on.
 *
 * @return	irq handled status.
 */
static irqreturn_t pp_push_handle_interrupt(struct pp_push_channel *pchan)
{
	u32 desc_phys;
	struct pp_channel *chan = push_chan_to_chan(pchan);
	int handled = 0;
	unsigned long flags;

	/* Loop while descriptors exist, either in software or hardware. */
	while (0 != (desc_phys = push_reg_read(
	      pchan, DMA_PUSH_WORK_FIFO_RD_DESCx_REG_OFFSET))) {
		struct pp_fifo_entry *entry;

		spin_lock_irqsave(&chan->lock, flags);
		entry = list_first_entry_or_null(&chan->used.fifo,
						 struct pp_fifo_entry,
						 list);
		/*
		 * This would indicate a serious desyncronization between the IP
		 * and buffer management, as this would be the result us
		 * getting a packet when we had not provided any to receive.
		 * This could indicate a hardware issue or a race condition.
		 */
		if (dev_WARN_ONCE(&chan->dev->dev, !entry,
				  "Received interrupt with no outstanding hardware descriptors\n")) {
			spin_unlock_irqrestore(&chan->lock, flags);
			break;
		}
		list_del(&entry->list);
		chan->used.depth--;
		spin_unlock_irqrestore(&chan->lock, flags);

		/*
		 * This tests for ordering in the FIFO - if it hits, it
		 * likely indicates that the hardware has reset and is
		 * now out of sync with software.
		 */
		if (dev_WARN_ONCE(&chan->dev->dev,
				  entry->desc_phys != desc_phys,
				  "Unable to find matching entry at head of workers\n")) {
			break;
		}

#ifdef PUSHPULL_ENABLE_TIMING
		entry->timestamp = ktime_get();
#endif

		dev_WARN_ONCE(&chan->dev->dev, entry->data_len,
			      "Entry has non-zero length %zu\n",
			      entry->data_len);

		handled = 1;

		pchan->push_handle_pkt(pchan, entry);

		pchan->stats.pkts_received++;

		/*
		 * Acknowledge any pending status. The status bits are level,
		 * not edge so if a new packet comes in between the above
		 * while() loop and these clears, the interrupt will correctly
		 * re-signal.
		 */
		push_clear_interrupts(pchan);
	}

	if (!chan->used.depth) {
		pchan->stats.hw_blocked++;
	}

	if (handled) {
		spin_lock_irqsave(&chan->lock, flags);
		pchan->stats.irq_queued += pp_push_start_receive(pchan);
		spin_unlock_irqrestore(&chan->lock, flags);
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

/**
 * pp_push_irq() - handle interrupts from the push (rx) block.
 * @irq:	interrupt.
 * @d:		device context for irq.
 *
 * @return	irq handled status.
 */
static irqreturn_t pp_push_irq(int irq, void *d)
{
	struct pp_channel *chan = d;
	struct pp_push_channel *pchan = chan_to_push_chan(chan);

	return pp_push_handle_interrupt(pchan);
}

/**
 * pp_pull_handle_interrupt() - handle interrupts from the pull (tx) block.
 * @pchan:	The channel to handle the interrupt on.
 *
 * @return	irq handled status.
 */
static irqreturn_t pp_pull_handle_interrupt(struct pp_pull_channel *pchan)
{
	u32 desc_phys;
	struct pp_channel *chan = pull_chan_to_chan(pchan);
	int handled = 0;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	/* Release all completed transmit buffers. */
	while (0 != (desc_phys = pull_reg_read(
			 pchan, DMA_PULL_FREE_FIFO_RD_DESC_REG_OFFSET))) {
		struct pp_fifo_entry *entry = list_first_entry_or_null(&chan->complete.fifo,
							struct pp_fifo_entry,
							list);

		handled = 1;

		if (dev_WARN_ONCE(&chan->dev->dev, !entry,
				  "Received interrupt without any queued entries\n")) {
			break;
		}

		/*
		 * This tests for ordering in the FIFO - if it hits, it
		 * likely indicates that the hardware has reset and is
		 * now out of sync with software.
		 */
		dev_WARN_ONCE(&chan->dev->dev, entry->desc_phys != desc_phys,
			      "Hardware descriptor 0x%llx does not match queued 0x%x\n",
			      entry->desc_phys, desc_phys);

		/*
		 * Return the buffer to the free list and signal any
		 * waiters that a new TX buffer is available.
		 */
		list_move(&entry->list, &chan->free.fifo);
		chan->complete.depth--;
		chan->free.depth++;
	}

	pchan->stats.irq_queued += pp_pull_start_transmit(pchan);
	spin_unlock_irqrestore(&chan->lock, flags);

	if (pchan->extra_work) {
		pchan->extra_work(pchan);
	}

	/*
	 * Give notice to the user that entries may be available.
	 */
	wake_up_interruptible(&chan->wait_queue);

	/*
	 * Acknowledge any pending status. The status bits are level, not edge
	 * so if a new packet comes in between the above while() loop and these
	 * clears, the interrupt will correctly re-signal.
	 */
	pull_clear_interrupts(pchan);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

/**
 * pp_pull_irq() - handle interrupts from the push (rx) block.
 * @irq:	interrupt.
 * @d:		device context for irq.
 *
 * @return	irq handled status.
 */
static irqreturn_t pp_pull_irq(int irq, void *d)
{
	struct pp_channel *chan = d;
	struct pp_pull_channel *pchan = chan_to_pull_chan(chan);

	return pp_pull_handle_interrupt(pchan);
}

/**
 * pp_ioctl() - Handle ioctl commands for a push/pull.
 * @flip:	The file to set the ioctl for.
 * @cmd:	The ioctl command sent from userspace.
 * @arg:	The argument sent from userspace.
 *
 * @return	0 on success, <0 on failure.
 */
static long pp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pp_priv *priv;
	struct pp_file_ctx *fctx = filp->private_data;

	if (_IOC_TYPE(cmd) != PP_APP_TYPE) {
		return -EINVAL;
	}

	if (WARN_ONCE(!fctx->dev_ctx->priv, "Failed to lookup file\n")) {
		return -ENODEV;
	}

	priv = fctx->dev_ctx->priv;
	if (priv->desc_info->if_ioctl) {
		return priv->desc_info->if_ioctl(filp, cmd, arg);
	}

	return -EINVAL;
}

/*
 * Push-pull character device file operations for when both a push and pull
 * block exist.
 */
static const struct file_operations pp_pushpull_fops = {
	.owner = THIS_MODULE,
	.open = pp_open,
	.release = pp_release,
	.read = pp_read,
	.write = pp_write,
	.flush = NULL,
	.poll = pp_poll,
	.unlocked_ioctl = pp_ioctl,

	/*
	 * From Linux Device Drivers:
	 * "If this function pointer is NULL, seek calls will
	 * modify the position counter in the file structure
	 * (described in the section "The File Structure")
	 * in potentially unpredictable ways.
	 */
	.llseek = noop_llseek,
};

/*
 * Push-pull character device file operations for when only a push block exists.
 */
static const struct file_operations pp_push_fops = {
	.owner = THIS_MODULE,
	.open = pp_open,
	.release = pp_release,
	.read = pp_read,
	.write = NULL,
	.flush = NULL,
	.poll = pp_push_poll,
	.unlocked_ioctl = pp_ioctl,
	.llseek = noop_llseek,
};

/*
 * Push-pull character device file operations for when only a pull block exists.
 */
static const struct file_operations pp_pull_fops = {
	.owner = THIS_MODULE,
	.open = pp_open,
	.release = pp_release,
	.read = NULL,
	.write = pp_write,
	.flush = NULL,
	.poll = pp_pull_poll,
	.unlocked_ioctl = pp_ioctl,
	.llseek = noop_llseek,
};

/**
 * pp_chrdev_init() - performs pre-driver-registration initialization.
 */
static void __init pp_chrdev_init(void)
{
	/* shouldn't be necessary, but for extra paranoia... */
	bitmap_zero(registered_minors, MAX_PUSHPULL_MINORS);

	pp_class = class_create(THIS_MODULE, DRIVER_NAME);
}

/**
 * pp_chrdev_exit() - cleans up after pp_chrdev_init.
 *
 * Note: we omit the __exit macro here so that this method can be safely
 * called from __init functions for rollback in error cases.
 */
static void pp_chrdev_exit(void) { class_destroy(pp_class); }

#ifdef PUSHPULL_ENABLE_TIMING
static ssize_t pushpull_show_timing(struct pp_channel *chan,
				    char *buf)
{
	unsigned long flags;
	struct pp_timing timing;
	ssize_t ret = 0;
	unsigned int bucket_top = PUSHPULL_TIMING_BUCKET_STRIDE;
	int i;

	spin_lock_irqsave(&chan->timing.lock, flags);
	timing = chan->timing;
	memcpy(&timing, &chan->timing, sizeof(timing));
	pp_timing_clear(&chan->timing);
	spin_unlock_irqrestore(&chan->timing.lock, flags);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			 "Interval Length:   %lldus\n"
			 "Packets:           %lld\n"
			 "Min Latency:       %lldus\n"
			 "Max Latency:       %lldus\n"
			 "Avg Latency:       %lldus\n"
			 "Total Latency:     %lldus\n",
			 ktime_us_delta(ktime_get(), timing.start_interval),
			 timing.packet_cnt,
			 timing.packet_cnt ? timing.min_latency : 0,
			 timing.max_latency,
			 timing.total_latency / timing.packet_cnt,
			 timing.total_latency);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			 "        < %5dus: %lld\n",
			 bucket_top,
			 timing.bucket[0]);

	for (i = 1; i < PUSHPULL_TIMING_BUCKET_CNT - 1; i++) {
		unsigned int  bucket_bottom = bucket_top;

		bucket_top *= PUSHPULL_TIMING_BUCKET_STRIDE;
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				 "%5dus - %5dus: %lld\n",
				 bucket_bottom,
				 bucket_top,
				 timing.bucket[i]);
	}
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			 "%5dus <        : %lld\n",
			 bucket_top,
			 timing.bucket[PUSHPULL_TIMING_BUCKET_CNT - 1]);

	return ret;
}

static ssize_t pushpull_push_show_timing(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct pp_priv *pp = dev_get_drvdata(dev);

	return pushpull_show_timing(&pp->push->chan, buf);
}
static DEVICE_ATTR(push_timing, 0444, pushpull_push_show_timing, NULL);

static ssize_t pushpull_pull_show_timing(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct pp_priv *pp = dev_get_drvdata(dev);

	return pushpull_show_timing(&pp->pull->chan, buf);
}
static DEVICE_ATTR(pull_timing, 0444, pushpull_pull_show_timing, NULL);
#endif /* PUSHPULL_ENABLE_TIMING */

/**
 * Declare all registers to map to sysfs entries, along with friendly names.
 */
#define PP_SHOW_REG_SYSFS(type, offset, name) \
static ssize_t pushpull_reg_show_##name(struct device *dev, \
					struct device_attribute *attr, \
					char *buf) \
{ \
	struct pp_priv *pp = dev_get_drvdata(dev); \
	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", \
			 ioread32(pp->type->chan.regs.regs + (offset))); \
} \
static DEVICE_ATTR(name, 0444, pushpull_reg_show_##name, NULL)

#define PP_SHOW_FIELD_SYSFS(type, name, field) \
static ssize_t pushpull_##type##_stat_show_##name(struct device *dev, \
					struct device_attribute *attr, \
					char *buf) \
{ \
	struct pp_priv *pp = dev_get_drvdata(dev); \
	return scnprintf(buf, PAGE_SIZE, "%llu\n", (u64)pp->type->field); \
} \
static DEVICE_ATTR(type##_##name, 0444, pushpull_##type##_stat_show_##name, NULL)

#define PP_SHOW_STAT_SYSFS(type, name) PP_SHOW_FIELD_SYSFS(type, name, stats.name)

PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS0_REG_OFFSET, push_pkts_pushed);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS1_REG_OFFSET, push_general_status);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS2_REG_OFFSET, push_err_pkts);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS3_REG_OFFSET, push_large_err_pkts);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS4_REG_OFFSET, push_no_buff_err_pkts);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS5_REG_OFFSET, push_drop_all_pkts);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS6_REG_OFFSET, push_drop_small_pkts);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS7_REG_OFFSET, push_data_bytes_out);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS8_REG_OFFSET, push_desc_bytes_out);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS9_REG_OFFSET, push_pkts_in_sop);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS10_REG_OFFSET, push_pkts_in_eop);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS11_REG_OFFSET, push_desc_in_count);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS12_REG_OFFSET, push_data_bytes_in);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_STATUS13_REG_OFFSET, push_total_bytes_out);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_FREE_FIFO_STATUS_REG_OFFSET, push_free_fifo_status);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_WORK_FIFO_STATUSx_REG_OFFSET, push_work_fifo_status);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_PRE_FIFO_ST_STATUS_REG_OFFSET, push_pre_fifo_st_status);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_CTRL_FIFO_ST_STATUS_REG_OFFSET, push_ctrl_fifo_st_status);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_DATA_FIFO_ST_STATUS_REG_OFFSET, push_data_fifo_st_status);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_FREE_FIFO_ST_STATUS_REG_OFFSET, push_free_fifo_st_status);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_WORK_FIFO_ST_STATUSx_REG_OFFSET, push_work_fifo_st_status0);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_ROTATE_DATA_FIFO_STATUS_REG_OFFSET, push_rotate_data_fifo_status);
PP_SHOW_REG_SYSFS(push, DMA_PUSH_PRE_CNT_FIFO_ST_STATUS_REG_OFFSET, push_pre_cnt_fifo_st_status);

static struct attribute *pp_push_sysfs_reg_attrs[] = {
	&dev_attr_push_pkts_pushed.attr,
	&dev_attr_push_general_status.attr,
	&dev_attr_push_err_pkts.attr,
	&dev_attr_push_large_err_pkts.attr,
	&dev_attr_push_no_buff_err_pkts.attr,
	&dev_attr_push_drop_all_pkts.attr,
	&dev_attr_push_drop_small_pkts.attr,
	&dev_attr_push_data_bytes_out.attr,
	&dev_attr_push_desc_bytes_out.attr,
	&dev_attr_push_pkts_in_sop.attr,
	&dev_attr_push_pkts_in_eop.attr,
	&dev_attr_push_desc_in_count.attr,
	&dev_attr_push_data_bytes_in.attr,
	&dev_attr_push_total_bytes_out.attr,
	&dev_attr_push_free_fifo_status.attr,
	&dev_attr_push_work_fifo_status.attr,
	&dev_attr_push_pre_fifo_st_status.attr,
	&dev_attr_push_ctrl_fifo_st_status.attr,
	&dev_attr_push_data_fifo_st_status.attr,
	&dev_attr_push_free_fifo_st_status.attr,
	&dev_attr_push_work_fifo_st_status0.attr,
	&dev_attr_push_rotate_data_fifo_status.attr,
	&dev_attr_push_pre_cnt_fifo_st_status.attr,
	NULL,
};

static const struct attribute_group pp_push_sysfs_regs_group = {
	.name = "push_registers", .attrs = pp_push_sysfs_reg_attrs,
};

PP_SHOW_STAT_SYSFS(push, usr_reads);
PP_SHOW_STAT_SYSFS(push, usr_pkts_read);
PP_SHOW_STAT_SYSFS(push, pkts_received);
PP_SHOW_STAT_SYSFS(push, hw_blocked);
PP_SHOW_STAT_SYSFS(push, irq_queued);
PP_SHOW_STAT_SYSFS(push, usr_queued);
PP_SHOW_STAT_SYSFS(push, network_pkts);
PP_SHOW_STAT_SYSFS(push, cdev_pkts);
PP_SHOW_STAT_SYSFS(push, discards);
PP_SHOW_FIELD_SYSFS(push, hw_outstanding, chan.used.depth); /* Legacy */
PP_SHOW_FIELD_SYSFS(push, free_entries, chan.free.depth);
PP_SHOW_FIELD_SYSFS(push, used_entries, chan.used.depth);
PP_SHOW_FIELD_SYSFS(push, complete_entries, chan.complete.depth);

static struct attribute *pp_push_sysfs_stat_attrs[] = {
	&dev_attr_push_usr_reads.attr,
	&dev_attr_push_usr_pkts_read.attr,
	&dev_attr_push_pkts_received.attr,
	&dev_attr_push_hw_blocked.attr,
	&dev_attr_push_irq_queued.attr,
	&dev_attr_push_usr_queued.attr,
	&dev_attr_push_network_pkts.attr,
	&dev_attr_push_cdev_pkts.attr,
	&dev_attr_push_discards.attr,
	&dev_attr_push_hw_outstanding.attr,
#ifdef PUSHPULL_ENABLE_TIMING
	&dev_attr_push_timing.attr,
#endif
	&dev_attr_push_free_entries.attr,
	&dev_attr_push_used_entries.attr,
	&dev_attr_push_complete_entries.attr,
	NULL,
};

static const struct attribute_group pp_push_sysfs_stats_group = {
	.name = "push_stats", .attrs = pp_push_sysfs_stat_attrs,
};

PP_SHOW_REG_SYSFS(pull, DMA_PULL_CTRL0_REG_OFFSET, pull_ctrl0);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_STATUS0_REG_OFFSET, pull_pkts_pulled);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_STATUS1_REG_OFFSET, pull_general_status);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_STATUS3_REG_OFFSET, pull_data_bytes_in);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_STATUS4_REG_OFFSET, pull_desc_bytes_in);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_STATUS5_REG_OFFSET, pull_pkts_out_sop);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_STATUS6_REG_OFFSET, pull_pkts_out_eop);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_STATUS7_REG_OFFSET, pull_desc_out_count);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_STATUS8_REG_OFFSET, pull_data_bytes_out);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_NUM_PKTS_DROPPED_REG_OFFSET, pull_pkts_dropped);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_FREE_FIFO_STATUS_REG_OFFSET, pull_free_fifo_status);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_WORK_FIFO_STATUS_REG_OFFSET, pull_work_fifo_status);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_PRE_FREE_FIFO_ST_STATUS_REG_OFFSET, pull_pre_free_fifo_st_status);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_DESC_FIFO_ST_STATUS_REG_OFFSET, pull_desc_fifo_st_status);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_KEEP_FIFO_ST_STATUS_REG_OFFSET, pull_keep_fifo_st_status);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_FREE_FIFO_ST_STATUS_REG_OFFSET, pull_free_fifo_st_status);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_WORK_FIFO_ST_STATUS_REG_OFFSET, pull_work_fifo_st_status);
PP_SHOW_REG_SYSFS(pull, DMA_PULL_PRE_CNT_FIFO_ST_STATUS_REG_OFFSET, pull_pre_cnt_fifo_st_status);

static struct attribute *pp_pull_sysfs_reg_attrs[] = {
	&dev_attr_pull_ctrl0.attr,
	&dev_attr_pull_pkts_pulled.attr,
	&dev_attr_pull_general_status.attr,
	&dev_attr_pull_data_bytes_in.attr,
	&dev_attr_pull_desc_bytes_in.attr,
	&dev_attr_pull_pkts_out_sop.attr,
	&dev_attr_pull_pkts_out_eop.attr,
	&dev_attr_pull_desc_out_count.attr,
	&dev_attr_pull_data_bytes_out.attr,
	&dev_attr_pull_pkts_dropped.attr,
	&dev_attr_pull_free_fifo_status.attr,
	&dev_attr_pull_work_fifo_status.attr,
	&dev_attr_pull_pre_free_fifo_st_status.attr,
	&dev_attr_pull_desc_fifo_st_status.attr,
	&dev_attr_pull_keep_fifo_st_status.attr,
	&dev_attr_pull_free_fifo_st_status.attr,
	&dev_attr_pull_work_fifo_st_status.attr,
	&dev_attr_pull_pre_cnt_fifo_st_status.attr,
	NULL
};

static const struct attribute_group pp_pull_sysfs_regs_group = {
	.name = "pull_registers", .attrs = pp_pull_sysfs_reg_attrs,
};

PP_SHOW_STAT_SYSFS(pull, usr_writes);
PP_SHOW_STAT_SYSFS(pull, pkts_sent);
PP_SHOW_STAT_SYSFS(pull, usr_blocked);
PP_SHOW_STAT_SYSFS(pull, usr_queued);
PP_SHOW_STAT_SYSFS(pull, irq_queued);
PP_SHOW_STAT_SYSFS(pull, network_pkts);
PP_SHOW_STAT_SYSFS(pull, cdev_pkts);
PP_SHOW_FIELD_SYSFS(pull, hw_outstanding, chan.complete.depth); /* Legacy */
PP_SHOW_STAT_SYSFS(pull, hw_full);
PP_SHOW_FIELD_SYSFS(pull, free_entries, chan.free.depth);
PP_SHOW_FIELD_SYSFS(pull, used_entries, chan.used.depth);
PP_SHOW_FIELD_SYSFS(pull, complete_entries, chan.complete.depth);

static struct attribute *pp_pull_sysfs_stat_attrs[] = {
	&dev_attr_pull_usr_writes.attr,
	&dev_attr_pull_pkts_sent.attr,
	&dev_attr_pull_usr_blocked.attr,
	&dev_attr_pull_usr_queued.attr,
	&dev_attr_pull_irq_queued.attr,
	&dev_attr_pull_network_pkts.attr,
	&dev_attr_pull_cdev_pkts.attr,
	&dev_attr_pull_hw_outstanding.attr,
	&dev_attr_pull_hw_full.attr,
#ifdef PUSHPULL_ENABLE_TIMING
	&dev_attr_pull_timing.attr,
#endif
	&dev_attr_pull_free_entries.attr,
	&dev_attr_pull_used_entries.attr,
	&dev_attr_pull_complete_entries.attr,
	NULL,
};

static const struct attribute_group pp_pull_sysfs_stats_group = {
	.name = "pull_stats", .attrs = pp_pull_sysfs_stat_attrs,
};

static ssize_t pushpull_cfe_show_catson(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct pp_priv *pp = dev_get_drvdata(dev);

	if (!pp->dev_ctx_count) {
		return -EINVAL;
	}

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			(pp->dev_ctx[0].link_id >> 4) & 0xf);
}

static ssize_t pushpull_cfe_set_catson(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int i;
	u32 catson;
	struct pp_priv *pp = dev_get_drvdata(dev);

	if (sscanf(buf, "%d\n", &catson) != 1 || catson > 16) {
		return -EINVAL;
	}

	for (i = 0; i < pp->dev_ctx_count; i++) {
		pp->dev_ctx[i].link_id =
			(pp->dev_ctx[i].link_id & ~0xf0) | (catson << 4);
	}

	if (pp->push) {
		struct pp_linkid_push_channel *dchan = container_of(pp->push,
						struct pp_linkid_push_channel,
						pchan);
		for (i = 0; i < MAX_LINKID_DFS_COUNT; i++) {
			dchan->link_id[i] =
				(dchan->link_id[i] & ~0xf0) |
				(catson << 4);
		}
	}

	dev_notice(dev, "Set Catson ID to %d\n", catson);

	return count;
}

static DEVICE_ATTR(cfe_catson, 0644, pushpull_cfe_show_catson,
		   pushpull_cfe_set_catson);

static struct attribute *pp_cfe_sysfs_attrs[] = {
	&dev_attr_cfe_catson.attr,
	NULL,
};

static const struct attribute_group pp_cfe_sysfs_group = {
	.name = "link_id", .attrs = pp_cfe_sysfs_attrs,
};

/**
 * pushpull_cfenet_show_catson() - Shows catson used in linkid.
 * @dev:	The linux backing device.
 * @attr:	This attribute.
 * @buf:	User data to show the catson ID.
 *
 * @return	Number of bytes written on success, <0 on failure.
 */
static ssize_t pushpull_cfenet_show_catson(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct pp_priv *pp = dev_get_drvdata(dev);
	struct pp_net_pull_channel *dchan;

	if (!pp->pull) {
		return -ENODEV;
	}

	dchan = container_of(pp->pull, struct pp_net_pull_channel, pchan);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 (dchan->link_id[0] >> 4) & 0xf);
}

/**
 * pushpull_cfenet_set_catson() - Sets catson used in linkid.
 * @dev:	The linux backing device.
 * @attr:	This attribute.
 * @buf:	User data to write the catson ID.
 * @count:	The length of buf.
 *
 * @return	count on success, <0 on failure.
 */
static ssize_t pushpull_cfenet_set_catson(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct pp_priv *pp = dev_get_drvdata(dev);
	struct pp_net_pull_channel *dchan;
	u32 catson;

	if (!pp->pull) {
		return -ENODEV;
	}

	if (sscanf(buf, "%d\n", &catson) != 1 || catson > 16) {
		return -EINVAL;
	}

	dchan = container_of(pp->pull, struct pp_net_pull_channel, pchan);

	dchan->link_id[0] = (dchan->link_id[0] & ~0xf0) | (catson << 4);

	dev_notice(dev, "Set Catson ID to %d\n", catson);

	return count;
}
static DEVICE_ATTR(cfenet_catson, 0644, pushpull_cfenet_show_catson,
		   pushpull_cfenet_set_catson);

static struct attribute *pp_cfenet_sysfs_attrs[] = {
	&dev_attr_cfenet_catson.attr,
	NULL,
};

static const struct attribute_group pp_cfenet_sysfs_group = {
	.name = "link_id", .attrs = pp_cfenet_sysfs_attrs,
};

/**
 * pp_netif_open() - Network hook for bringup up an interface
 * @dev:	The network device to open.
 *
 * @return	Zero on success, non-zero on failure.
 */
static int pp_netif_open(struct net_device *dev)
{
	struct pp_priv *priv = netdev_priv(dev);

	/* Disable queue and return if egress pathway is not defined */
	if (!priv->pull) {
		netif_stop_queue(dev);
		return 0;
	}

	netif_start_queue(dev);

	return 0;
}

/**
 * pp_netif_close() - Network hook for taking down an interface
 * @dev:	The network device to close.
 *
 * @return	Zero on success, non-zero on failure.
 */
static int pp_netif_close(struct net_device *dev)
{
	netif_stop_queue(dev);

	return 0;
}

/**
 * pp_netif_tx_packet() - Network hook for transmitting a packet.
 * @skb:	The network packet to send.
 * @dev:	The network device to send on.
 *
 * @return	Zero on success, non-zero on failure.
 */
static int pp_netif_tx_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct pp_priv *priv = netdev_priv(dev);
	struct pp_pull_channel *pchan = pp_priv_to_pull(priv);
	struct pp_channel *chan = pull_chan_to_chan(pchan);
	struct pp_net_pull_channel *dchan = container_of(pchan,
					      struct pp_net_pull_channel,
					      pchan);


	struct pp_fifo_entry *entry;
	int rc = 0;
	int msg_count = 1;
	int i;
	unsigned long flags;
	LIST_HEAD(msgs);
	struct pp_skb_ctx skb_ctx = {
		.skb = skb,
	};
#ifdef PUSHPULL_ENABLE_TIMING
	const ktime_t start_time = ktime_get();
#endif

	pchan->stats.usr_writes++;

	if (skb->len > chan->buffer_size + dchan->header_len) {
		dev_err(&chan->dev->dev, "write too large - %u", skb->len);
		dev->stats.tx_errors++;
		return NETDEV_TX_BUSY;
	}

	if (netif_queue_stopped(dev)) {
		dev->stats.tx_errors++;
		return NETDEV_TX_BUSY;
	}

	if (!priv->ip_dev &&
		pchan->endpoints > 1 &&
		(is_multicast_ether_addr(skb->data) ||
		 is_broadcast_ether_addr(skb->data))) {
		msg_count = pchan->endpoints;
	}

	/* Get the entries needed */
	spin_lock_irqsave(&chan->lock, flags);
	for (i = 0; i < msg_count; i++) {
		entry = list_first_entry_or_null(&chan->free.fifo,
						 struct pp_fifo_entry,
						 list);
		if (unlikely(!entry)) {
			list_splice(&msgs, &chan->free.fifo);
			netif_stop_queue(dev);
			spin_unlock_irqrestore(&chan->lock, flags);
			dev->stats.tx_errors++;
			return NETDEV_TX_BUSY;
		}

		list_move_tail(&entry->list, &msgs);
	}
	chan->free.depth -= msg_count;
	if (chan->free.depth < pchan->endpoints) {
		netif_stop_queue(dev);
		pchan->stats.usr_blocked++;
	}
	spin_unlock_irqrestore(&chan->lock, flags);

	/* Configure the entries */
	i = 0;
	list_for_each_entry(entry, &msgs, list) {
		skb_ctx.transmit_idx = i++;

#ifdef PUSHPULL_ENABLE_TIMING
		entry->timestamp = start_time;
#endif
		entry->data_len = dchan->header_len + skb_headlen(skb);

		if (pchan->encode_header &&
			!pchan->encode_header(pchan, entry->buffer, skb)) {
			dev->stats.tx_errors++;
			break;
		}

		memcpy(entry->buffer + dchan->header_len,
			   skb->data,
			   skb_headlen(skb));

		if (pchan->pad_size > 0 && pchan->pad_size > entry->data_len) {
			memset((u8 *)entry->buffer + entry->data_len,
			       0,
			       pchan->pad_size - entry->data_len);
		}

		rc = pchan->encode_descriptor(pchan, &skb_ctx,
					      max((size_t)pchan->pad_size,
							  entry->data_len),
					      entry,
					      PULL_SOURCE_NETWORK);
		if (rc != 0) {
			dev->stats.tx_errors++;
			break;
		}
	}

	/* Then queue them */
	spin_lock_irqsave(&chan->lock, flags);
	list_splice_tail(&msgs, &chan->used.fifo);
	chan->used.depth += msg_count;
	pchan->stats.usr_queued += pp_pull_start_transmit(pchan);
	spin_unlock_irqrestore(&chan->lock, flags);

	kfree_skb(skb);

	dev->stats.tx_packets += msg_count;
	pchan->stats.network_pkts += msg_count;
	dev->stats.tx_bytes += msg_count * skb->len;

	return rc ? NETDEV_TX_BUSY : 0;
}

static const struct pp_desc_info pp_desc_list[] = {
	{
		/* Common descriptor for EUT/UT/CFE blocks on RC Platforms. */
		.name = "cdec",
		.pull_encode_descriptor = cdec_pull_set_descriptor,
		.push_extra_init = linkid_push_chan_init,
		.push_handle_pkt = cdec_push_handle_pkt,
		.if_register = pp_chrdev_register_lid,
		.if_unregister = pp_chrdev_unregister_device,
		.attrs = &pp_cfe_sysfs_group,
	},
	{
		.name = "cdecnet",
		.pull_extra_init = linkid_net_pull_chan_init,
		.pull_extra_work = network_pull_extra_work,
		.pull_encode_descriptor = cdecnet_pull_set_descriptor,
		.push_extra_init = net_push_chan_init,
		.push_extra_deinit = net_push_chan_deinit,
		.push_prepare_desc = pp_net_push_prepare_desc,
		.push_handle_pkt = cdecnet_push_handle_pkt,
		.if_register = pp_register_net,
		.if_unregister = pp_unregister_net,
		.attrs = &pp_cfenet_sysfs_group,
		.dynamic_push_buffers = true,
	},
	{
		.name = "c4ethnet",
		.pull_extra_init = linkid_net_pull_chan_init,
		.pull_extra_work = network_pull_extra_work,
		.pull_encode_descriptor = cdecnet_pull_set_descriptor,
		.push_extra_init = net_push_chan_init,
		.push_extra_deinit = net_push_chan_deinit,
		.push_prepare_desc = pp_net_push_prepare_desc,
		.push_handle_pkt = c4ethnet_push_handle_pkt,
		.if_register = pp_register_net,
		.if_unregister = pp_unregister_net,
		.attrs = &pp_cfenet_sysfs_group,
		.dynamic_push_buffers = true,
	},
	{
		.name = "c4sdu",
		.pull_encode_descriptor = c4sdu_pull_set_descriptor,
		.push_extra_init = linkid_push_chan_init,
		.push_handle_pkt = c4sdu_push_handle_pkt,
		.if_register = pp_chrdev_register_lid,
		.if_unregister = pp_chrdev_unregister_device,
	},
	{
		.name = "aap",
		.pull_encode_descriptor = aap_pull_set_descriptor,
		.push_handle_pkt = aap_push_handle_pkt,
		.if_register = pp_chrdev_register_simple,
		.if_unregister = pp_chrdev_unregister_device,
	},
	{
		.name = "l3gmac_net",
		.pull_extra_init = net_pull_chan_init_common,
		.pull_extra_work = network_pull_extra_work,
		.pull_encode_descriptor = l3net_pull_set_descriptor,
		.push_extra_init = net_push_chan_init,
		.push_extra_deinit = net_push_chan_deinit,
		.push_prepare_desc = pp_net_push_prepare_desc,
		.push_handle_pkt = l3net_push_handle_pkt,
		.if_register = pp_register_net,
		.if_unregister = pp_unregister_net,
		.dynamic_push_buffers = true,
	},
	{
		.name = "cl3ul",
		.pull_encode_descriptor = cl3ul_pull_set_descriptor,
		.push_handle_pkt = l3_push_handle_pkt,
		.if_register = pp_chrdev_register_simple,
		.if_unregister = pp_chrdev_unregister_device,
		.if_ioctl = l3ul_pull_ioctl,
	},
	{
		.name = "snpu",
		.pull_encode_descriptor = snpu_pull_set_descriptor,
		.push_handle_pkt = snpu_push_handle_pkt,
		.if_register = pp_chrdev_register_simple,
		.if_unregister = pp_chrdev_unregister_device,
	},
	{
		.name = "snpunet",
		.pull_extra_init = net_pull_chan_init_common,
		.pull_extra_work = network_pull_extra_work,
		.pull_encode_descriptor = snpu_pull_set_descriptor,
		.push_extra_init = net_push_chan_init,
		.push_extra_deinit = net_push_chan_deinit,
		.push_prepare_desc = pp_net_push_prepare_desc,
		.push_handle_pkt = snpunet_push_handle_pkt,
		.if_register = pp_register_net,
		.if_unregister = pp_unregister_net,
		.dynamic_push_buffers = true,
	},
	{
		.name = "gnpu",
		.pull_extra_init = net_pull_chan_init_common,
		.pull_extra_work = network_pull_extra_work,
		.pull_encode_descriptor = gnpu_pull_set_descriptor,
		.push_extra_init = hybrid_push_chan_init,
		.push_extra_deinit = net_push_chan_deinit,
		.push_prepare_desc = gnpu_push_prepare_desc,
		.push_handle_pkt = gnpu_push_handle_pkt,
		.if_register = pp_register_hybrid,
		.if_unregister = pp_unregister_hybrid,
		.dynamic_push_buffers = true,
	},
	{
		.name = "phynet",
		.pull_extra_init = phynet_pull_chan_init,
		.pull_extra_work = network_pull_extra_work,
		.net_pull_encode_header = phynet_pull_encode_header,
		.pull_encode_descriptor = phynet_pull_set_descriptor,
		.push_extra_init = phynet_push_chan_init,
		.push_extra_deinit = net_push_chan_deinit,
		.push_prepare_desc = pp_net_push_prepare_desc,
		.push_handle_pkt = cdecnet_push_handle_pkt,
		.if_register = pp_register_net,
		.if_unregister = pp_unregister_net,
		.dynamic_push_buffers = true,
	},
	{
		.name = "pktproc",
		.pull_encode_descriptor = pktproc_pull_set_descriptor,
		.push_handle_pkt = pktproc_push_handle_pkt,
		.if_register = pp_chrdev_register_simple,
		.if_unregister = pp_chrdev_unregister_device,
	},
	{
		.name = "pktprocnet",
		.pull_extra_init = net_pull_chan_init_common,
		.pull_extra_work = network_pull_extra_work,
		.pull_encode_descriptor = pktproc_pull_set_descriptor,
		.push_extra_init = net_push_chan_init,
		.push_extra_deinit = net_push_chan_deinit,
		.push_prepare_desc = pp_net_push_prepare_desc,
		.push_handle_pkt = pktprocnet_push_handle_pkt,
		.if_register = pp_register_net,
		.if_unregister = pp_unregister_net,
		.dynamic_push_buffers = true,
	},
};

/**
 * pp_probe_common() - allocates and inits memory for push-pull
 * @priv:	The pushpull instance to probe.
 * @dev:	our platform_device
 *
 * @return	Zero on success, non-zero on failure.
 */
static int pp_probe_common(struct pp_priv *priv, struct platform_device *dev)
{
	int rc = 0;
	struct phy *phy;
	struct device_node *push_child = NULL;
	struct device_node *pull_child = NULL;
	int pull_idx = 0;
	const char *desc_type_str;
	int i;

	BUILD_BUG_ON(sizeof(struct cdec_push_descriptor) !=
		     CDEC_PUSH_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct cdec_pull_descriptor) !=
		     CDEC_PULL_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct c4eth_push_descriptor) !=
		     C4ETH_PUSH_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct c4sdu_push_descriptor) !=
		     C4SDU_PUSH_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct c4sdu_pull_descriptor) !=
		     C4SDU_PULL_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct cfe_pull_descriptor) !=
		     CFE_PULL_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct aap_push_descriptor) !=
		     AAP_PUSH_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct aap_pull_descriptor) !=
		     AAP_PULL_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct l3_push_descriptor) !=
		     L3_PUSH_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct l3dl_pull_descriptor) !=
		     L3DL_PULL_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct l3ul_pull_descriptor) !=
		     L3UL_PULL_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct npu_push_descriptor) !=
			 NPU_PUSH_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct snpu_pull_descriptor) !=
		     SNPU_PULL_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct gnpu_pull_descriptor) !=
		     GNPU_PULL_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct pktproc_push_descriptor) !=
		     PKTPROC_PUSH_DESCRIPTOR_SIZE);
	BUILD_BUG_ON(sizeof(struct pktproc_pull_descriptor) !=
		     PKTPROC_PULL_DESCRIPTOR_SIZE);

	/*
	 * Set private data *before* requesting IRQ, as that information would
	 * be needed immediately if an interrupt were to arrive right away.
	 */
	platform_set_drvdata(dev, priv);
	/* and set the back-references as well */
	priv->pdev = dev;
	priv->dev = &dev->dev;

	phy = devm_phy_get(&dev->dev, "catson-phy");
	if (!IS_ERR(phy)) {
		phy_init(phy);
	} else if (PTR_ERR(phy) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	}

	phy = devm_phy_get(&dev->dev, "fcnet-phy");
	if (!IS_ERR(phy)) {
		phy_init(phy);
	} else if (PTR_ERR(phy) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	}

	/*
	 * Determine the type of descriptors to use for this channel.
	 */
	if (of_property_read_string(dev->dev.of_node, "desc-type",
				    &desc_type_str)) {
		dev_err(&dev->dev, "Could not find descriptor type\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(pp_desc_list); i++) {
		if (!strcmp(desc_type_str, pp_desc_list[i].name)) {
			priv->desc_info = &pp_desc_list[i];
			break;
		}
	}

	if (!priv->desc_info || !priv->desc_info->if_register) {
		dev_err(&dev->dev, "Unsupported descriptor type '%s' requested\n",
			desc_type_str);
		return -EINVAL;
	}

	push_child = of_get_child_by_name(dev->dev.of_node, "push");
	pull_child = of_get_child_by_name(dev->dev.of_node, "pull");

	if (!push_child && !pull_child) {
		dev_err(&dev->dev, "Failed finding either push or pull child\n");
		rc = -ENODEV;
		goto out;
	}

	if (push_child) {
		priv->push = pp_push_chan_init(dev,
					       push_child,
					       priv->desc_info,
					       PUSHPULL_PUSH_PARAM_IDX);
		if (!priv->push) {
			dev_err(&dev->dev, "error allocating push channel\n");
			rc = -ENOMEM;
			goto out;
		}

		/*
		 * The push index is always 0.  However, the pull index is
		 * defined on whether a push block was defined.
		 */
		pull_idx++;
	}

	if (pull_child) {
		priv->pull = pp_pull_chan_init(dev,
					       pull_child,
					       priv->desc_info,
					       pull_idx);
		if (!priv->pull) {
			dev_err(&dev->dev, "error allocating pull channel\n");
			rc = -ENOMEM;
			goto out;
		}
	}

	rc = priv->desc_info->if_register(priv);
	if (rc) {
		goto out;
	}

	if (priv->desc_info->attrs) {
		rc = sysfs_create_group(&dev->dev.kobj, priv->desc_info->attrs);
		if (rc) {
			goto chrdev_cleanup;
		}
	}

	/*
	 * Push has a dependency where the linkids are not configured until the
	 * character device is configured.  Therefore, any traffic received
	 * before this is configured is detected as an error.
	 */
	if (priv->push) {
		bool drop_if_no_bufs = true;

		if (of_find_property(push_child, "no-drop-if-no-bufs", NULL)) {
			drop_if_no_bufs = false;
		}

		pp_push_chan_start(priv->push, drop_if_no_bufs);
	}

	if (priv->pull) {
		pp_pull_chan_start(priv->pull);
	}

	return 0;

chrdev_cleanup:
	if (priv->desc_info->if_unregister) {
		priv->desc_info->if_unregister(priv);
	}
out:
	if (priv->push) {
		pp_push_free_channel(dev, priv->push, priv->desc_info);
		priv->push = NULL;
	}

	if (priv->pull) {
		pp_pull_free_channel(dev, priv->pull, priv->desc_info);
		priv->pull = NULL;
	}

	return rc;
}

/**
 * pp_probe_cdev() - allocates and inits memory for cdev push-pull
 * @dev:	our platform_device
 *
 * @return	Zero on success, non-zero on failure.
 */
static int pp_probe_cdev(struct platform_device *dev)
{
	struct pp_priv *priv = NULL;

	priv = devm_kzalloc(&dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}

	return pp_probe_common(priv, dev);
}

static const struct net_device_ops pp_netdev_ops = {
	.ndo_open		 = pp_netif_open,
	.ndo_stop		 = pp_netif_close,
	.ndo_start_xmit		 = pp_netif_tx_packet,
	.ndo_set_mac_address	 = eth_mac_addr,
	.ndo_validate_addr	 = eth_validate_addr,
};

static const struct net_device_ops pp_ip_netdev_ops = {
	.ndo_open		 = pp_netif_open,
	.ndo_stop		 = pp_netif_close,
	.ndo_start_xmit		 = pp_netif_tx_packet,
};

/**
 * pp_net_init() - Network callback hook for initializing netdev device.
 * @netdev:	the network device to initialize
 */
static void pp_net_init(struct net_device *netdev)
{
	ether_setup(netdev);
	netdev->watchdog_timeo = msecs_to_jiffies(1000);
	netdev->netdev_ops = &pp_netdev_ops;
}

static void pp_ip_net_init(struct net_device *netdev)
{
	/* Point-to-Point TUN Device */
	netdev->hard_header_len = 0;
	netdev->addr_len = 0;
	netdev->mtu = 1500;

	/* Zero header length */
	netdev->type = ARPHRD_NONE;
	netdev->flags = IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;

	netdev->watchdog_timeo = msecs_to_jiffies(1000);
	netdev->netdev_ops = &pp_ip_netdev_ops;
}

/**
 * pp_probe_net() - allocates and registers a network pushpull.
 * @dev:	our platform_device
 *
 * @return	Zero on success, non-zero on failure.
 */
static int pp_probe_net(struct platform_device *dev)
{
	struct net_device *net_dev;
	struct pp_priv *priv = NULL;
	char ifname[32];
	int rc;

	bool ip_dev = false;
	const char *suffix_str = NULL;

	/* This suffix gets appended to the /dev/pushpull_ char device name. */
	if (of_property_read_string(dev->dev.of_node, "suffix", &suffix_str)) {
		dev_dbg(&dev->dev, "Missing required parameter 'suffix'\n");
		return -1;
	}

	snprintf(ifname, sizeof(ifname), "pp_%s", suffix_str);

	if (of_find_property(dev->dev.of_node, "ip_netdev", NULL))
		ip_dev = true;

	if (!ip_dev) {
		net_dev = alloc_netdev(sizeof(struct pp_priv), ifname,
					NET_NAME_PREDICTABLE, pp_net_init);
	} else {
		net_dev = alloc_netdev(sizeof(struct pp_priv), ifname,
					NET_NAME_PREDICTABLE, pp_ip_net_init);
	}

	if (!net_dev) {
		dev_err(&dev->dev, "Failed to allocate netdev");
		return -ENODEV;
	}

	priv = netdev_priv(net_dev);
	memset(priv, 0, sizeof(struct pp_priv));

	priv->netdev = net_dev;
	priv->ip_dev = ip_dev;

	rc = pp_probe_common(priv, dev);
	if (rc != 0) {
		goto out_probe_fail;
	}

	return 0;

out_probe_fail:
	free_netdev(net_dev);

	return rc;
}

/**
 * pp_remove() - called when kernel wants us to unbind from the device.
 * @dev:	the device to unbind from.
 *
 * @return	Zero on success, non-zero on failure.
 */
static int pp_remove(struct platform_device *dev)
{
	struct pp_priv *priv = NULL;

	priv = platform_get_drvdata(dev);

	if (priv->desc_info->attrs) {
		sysfs_remove_group(&dev->dev.kobj, priv->desc_info->attrs);
	}

	if (priv->desc_info->if_unregister) {
		priv->desc_info->if_unregister(priv);
	}

	/* De-initialize channels */
	if (priv->push) {
		pp_push_free_channel(dev, priv->push, priv->desc_info);
		priv->push = NULL;
	}

	if (priv->pull) {
		pp_pull_free_channel(dev, priv->pull, priv->desc_info);
		priv->pull = NULL;
	}

	return 0;
}

/**
 * pp_remove_net() - called when a network device is destroyed.
 * @dev:	the device to unbind from.
 *
 * @return	Zero on success, non-zero on failure.
 */
static int pp_remove_net(struct platform_device *dev)
{
	struct pp_priv *priv = platform_get_drvdata(dev);

	pp_remove(dev);

	free_netdev(priv->netdev);

	return 0;
}

/*
 * Here we list which devices we are compatible with.
 */
static const struct of_device_id pp_cdev_ids[] = {
	{
		.compatible = CDEV_COMPAT_ID,
	},
	{},
};

/*
 * Now we register the compatibility information with the kernel.
 */
MODULE_DEVICE_TABLE(of, pp_cdev_ids);

/*
 * Here we define the driver itself.
 */
static struct platform_driver pp_driver_cdev = {
	.driver = {
		.name = DRIVER_NAME "_cdev",
		.owner = THIS_MODULE,
		.of_match_table = pp_cdev_ids,
	},
	.probe = pp_probe_cdev,
	.remove = pp_remove,
};

/*
 * Here we list which devices we are compatible with.
 */
static const struct of_device_id pp_net_ids[] = {
	{
		.compatible = NET_COMPAT_ID,
	},
	{
		.compatible = HYBRID_COMPAT_ID,
	},
	{},
};

/*
 * Now we register the compatibility information with the kernel.
 */
MODULE_DEVICE_TABLE(of, pp_net_ids);

/*
 * Here we define the driver itself.
 */
static struct platform_driver pp_driver_net = {
	.driver = {
		.name = DRIVER_NAME "_net",
		.owner = THIS_MODULE,
		.of_match_table = pp_net_ids,
	},
	.probe = pp_probe_net,
	.remove = pp_remove_net,
};

/**
 * pp_init() - initializes the driver.
 *
 * @return	Zero on success, non-zero on failure.
 */
static int __init pp_init(void)
{
	int rc = 0;

	/* Get a major id from the kernel. */
	rc = alloc_chrdev_region(&major_dev_id, 0, 1, DRIVER_NAME);
	if (rc) {
		goto out_unregister;
	}

	/* Initialize the char device portion of the driver. */
	pp_chrdev_init();

	/* This makes the driver real and eligible for devices to attach. */
	rc = platform_driver_register(&pp_driver_cdev);
	if (rc) {
		goto out_free_chrdev;
	}

	rc = platform_driver_register(&pp_driver_net);
	if (rc) {
		goto out;
	}

	return 0;

out_free_chrdev:
	pp_chrdev_exit();
	unregister_chrdev_region(major_dev_id, 1);
out_unregister:
	platform_driver_unregister(&pp_driver_net);
out:
	return rc;
}
module_init(pp_init);

/**
 * pp_exit() - removes the driver
 */
static void __exit pp_exit(void)
{
	platform_driver_unregister(&pp_driver_cdev);
	pp_chrdev_exit();
	unregister_chrdev_region(major_dev_id, 1);

	platform_driver_unregister(&pp_driver_net);

	major_dev_id = 0;
}
module_exit(pp_exit);

MODULE_LICENSE("GPL");
