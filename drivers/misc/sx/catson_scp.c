/*
 * SpaceX Catson SCP Master driver.
 *
 * Serial Control Protocol (SCP) is a packet-based protocol for
 * communicating with antenna beam-formers in phased-array systems. This
 * driver implements a character device for sending and receiving packets
 * and a set of sysfs entries for statistics and reset control.
 *
 * This driver supports the RX, TXL, and TXH FIFOs.
 */

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/poll.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/workqueue.h>

/* Auto-generated register headers. */
#include "catson_scp_regs.h"

/* Additional register definitions for easier code reuse. */
#define FIFO_BA_RELATIVE_OFFSET	0x00000000u
#define FIFO_FLAGS_RELATIVE_OFFSET	0x00000004u
#define FIFO_STATUS_RELATIVE_OFFSET	0x00000008u
#define FIFO_WR_IDX_RELATIVE_OFFSET	0x0000000Cu
#define FIFO_RD_IDX_RELATIVE_OFFSET	0x00000010u
#define FIFO_RELEASE_RELATIVE_OFFSET	0x00000014u

#define DRIVER_NAME "catscp"
#define COMPATIBLE_ID "sx,catson-scp-1.0"

/**
 * SCP_SET_IRQ_MASK - IRQ enable mask for each buffer sent or received.
 */
#define SCP_SET_IRQ_MASK                       \
	(SCP_CMN_ITS__RX_BUF_COMPLETE_ITS_bm | \
	 SCP_CMN_ITS__TXL_BUF_COMPLETE_ITS_bm | \
	 SCP_CMN_ITS__TXH_BUF_COMPLETE_ITS_bm)

/**
 * SCP_CLEAR_IRQ_MASK - IRQ enable mask to disable all IRQs.
 */
#define SCP_CLEAR_IRQ_MASK 0x1FFF

/**
 * LINK_DELAY_US - Time to wait during link up sequence.
 *
 * This value was determined experimentally from a Sarong.
 */
#define LINK_DELAY_US 1000

/**
 * PLL_LOCK_US - Time to wait after selecting clock.
 *
 * This value is specified by ST.
 */
#define PLL_LOCK_US 200

/**
 * SCP_RESET_HOLD_TIME_US - Time to assert reset.
 *
 * This is anecdotally tested to improve reset behavior.  Without a delay,
 * the Catson gets into a state where it believe the interface is up when
 * it is not.
 */
#define SCP_RESET_HOLD_TIME_US 1000

/*
 * Valid values for SCP_MEM_*_FIFO_FLAGS__*_LENGTH.
 *
 * Must be a power of two, and the field value is calculated by
 * log2(value) - log2(8), so 8 -> 0x0, and 1024 -> 0x7.
 */
#define HW_FIFO_COUNT_MIN 8
#define HW_FIFO_COUNT_MAX 1024

/*
 * SCP gate message validation constants.
 */
#define SCP_LEN_BITS 8
#define SCP_TTL_BITS 24
#define SCP_GATE_SIZE 4
#define SCP_TTL_INVALID (1<<SCP_TTL_BITS)

/*
 * Valid values for SCP_LNK_CTRL__PORT_STATE_CTRL field.
 */
enum {
	STATE_NO_SUCH_PORT = 0,
	STATE_DEFAULT = 1,
	STATE_UP = 2,
	STATE_DISABLED = 3,
	STATE_SUSP_INIT = 4,
	STATE_SUSP = 5,
	STATE_SUSP_EXIT = 6,
	STATE_FAILED = 7
};

/*
 * Valid link speed modes.
 */
enum {
	LINK_SPEED_20MBIT = 0,
	LINK_SPEED_600MBIT,
	LINK_SPEED_MAX
};

/**
 * Static globals
 */
static dev_t major;

/**
 * struct scp_descriptor - hardware ring buffer descriptor
 * @address:		physical address of descriptor
 * @length:		length of data in bytes
 */
struct scp_descriptor {
	u32 address;
	u32 length;
} __packed;
#define SCP_DESCRIPTOR_SIZE 8

/**
 * struct catscp_fifo_entry - describes a single buffer
 * @buffer:		virtual address of buffer memory
 * @buffer_phys:	physical address of buffer memory
 * @desc:		virtual address of descriptor memory
 * @desc_phys:		physical address of descriptor memory
 * @data_len:		length of valid data present in buffer. May be zero.
 */
struct catscp_fifo_entry {
	void *buffer;
	dma_addr_t buffer_phys;

	void *desc;
	u32 desc_phys;

	size_t data_len;
};

/**
 * struct catscp_channel - channel tracking structure
 * @buffer_count:	Total number of buffers available for use.
 * @buffer_size:	Size of each buffer allocated.
 * @hw_desc_count:	Number of hardware descriptors.
 * @desc:		Descriptor buffer.
 * @desc_phys:		Descriptor buffer physical address.
 * @buffer_pool:	DMA pool of buffer memory.
 * @used_fifo:		kfifo to track buffers in used state.
 * @free_fifo:		kfifo to track buffers in free state.
 * @complete_fifo:	kfifo to track buffers in complete state.
 * @used_queue:		wait queue when a buffer is added to used_fifo.
 * @free_queue:		wait queue when a buffer is added to free_fifo.
 * @complete_queue:	wait queue when a buffer is added to complete_fifo.
 * @desc_index:		index in descriptor ring buffer.
 * @desc_size:		size of individual descriptors.
 * @last_complete_index	last descriptor processed by software.
 * @lock:		protect access to indicies and registers in struct.
 * @io_mutex:		protect access from more then one reader or writer.
 */
struct catscp_channel {
	u32 buffer_count;
	u32 buffer_size;

	u32 hw_desc_count;

	void *desc;
	dma_addr_t desc_phys;

	struct dma_pool *buffer_pool;

	DECLARE_KFIFO_PTR(free_fifo, struct catscp_fifo_entry);
	DECLARE_KFIFO_PTR(used_fifo, struct catscp_fifo_entry);
	DECLARE_KFIFO_PTR(complete_fifo, struct catscp_fifo_entry);

	wait_queue_head_t free_queue;
	wait_queue_head_t used_queue;
	wait_queue_head_t complete_queue;

	u32 desc_index;
	u32 desc_size;
	u32 last_complete_index;

	spinlock_t lock;

	struct mutex io_mutex;
};

/**
 * struct catscp_priv - Private driver data structure
 * @pdev:		back-reference to the owning &struct platform_device
 * @dev:		back-reference to the owning &struct device
 * @cdev:		character device structure
 * @cdev_txh:	character device structure for high-priority TX queue
 * @major:		major device
 * @regs:		I/O memory region
 * @irq:		interrupt handler number
 * @rx:			receive channel context
 * @tx:			transmit channel context
 * @txh:		high-priority transmit channel context
 * @rx_work_thread:	worker thread for receive
 * @tx_work_thread:	worker thread for transmit
 * @txh_work_thread:	worker thread for high-priority transmit
 * @rx_timeout:		hardware buffer consolidation timeout
 * @link_speed:		hardware link-speed
 * @evt_msg_received:	true if we've received an SCP event message
 * @scp_reset		reset line for the SCP hardware
 * @reg_mutex		controls register access via sysfs
 */
struct catscp_priv {
	struct platform_device *pdev;
	struct device *dev;
	struct cdev cdev;
	struct cdev cdev_txh;
	dev_t major;

	void __iomem *regs;
	int irq;

	struct catscp_channel rx;
	struct catscp_channel tx;
	struct catscp_channel txh;

	struct task_struct *rx_work_thread;
	struct task_struct *tx_work_thread;
	struct task_struct *txh_work_thread;

	u32 rx_timeout;
	u32 link_speed;

	struct {
		u32 min_ttl;
		u32 max_ttl;
		u32 time_between_ttl;
		u32 prev_ttl;
		unsigned long prev_jiffies;
	} gate;

	bool evt_msg_received;

	struct reset_control *scp_reset;
	struct mutex reg_mutex;

	struct {
		/* Totals as we pass packets through each phase. */
		atomic_t sw_read_count; /* read() completed */
		atomic_t hw_read_count; /* packet received from hardware */
		atomic_t sw_write_count; /* write() completed */
		atomic_t hw_write_count; /* packet placed on hardware FIFO */
		atomic_t hw_write_complete_count; /* hardware marked packet as complete */

		/* If software dropped packets, where did we drop them? */
		atomic_t sw_read_drop_count; /* at the read() call */
		atomic_t sw_write_drop_count; /* at the write() call */
		atomic_t hw_write_drop_count; /* before we put them on the hardware FIFO() */

		/* Gate message stats. */
		atomic_t gate_under_min_count; /* Gates with TTL below min margin */
		atomic_t gate_over_max_count; /* Gates with TTL above max margin */
		atomic_t gate_low_time_between_count; /* Gates where TTL since prev gate is below threshold */
		atomic_t gate_unordered_count; /* Gates received with out-of-order TTL */
		atomic_t gate_min_margin; /* Min (relative) gate time in TTL counts */
		atomic_t gate_max_margin; /* Max (relative) gate time in TTL counts */
	} stats;
};

/* driver class */
static struct class *catscp_class;

/* forward-declaration */
static const struct file_operations catscp_fops;
static const struct file_operations catscp_fops_txh;

/**
 * catscp_reg_read() - reads a value from a register
 * @priv:	catscp private data structure
 * @offset:	which register to read
 *
 * @return	contents of register
 */
static u32 catscp_reg_read(struct catscp_priv *priv, u32 offset)
{
	char __iomem *regs = priv->regs;

	return ioread32(regs + offset);
}

/**
 * catscp_reg_write() - writes a value to a register
 * @priv:	catscp private data structure
 * @offset:	which register to write
 * @value:	the value to write to the register
 */
static void catscp_reg_write(struct catscp_priv *priv, u32 offset, u32 value)
{
	char __iomem *regs = priv->regs;

	return iowrite32(value, regs + offset);
}

/**
 * allocate_fifo_entry() - allocate fifo buffer
 * @dev		platform device
 * @chan	channel owning the buffer pool
 * @entry	structure to contain results of allocations
 *
 * @return	zero on success, non-zero on failure.
 *
 * Allocate a descriptor/buffer pair from the dma pools associated with
 * a channel object.
 */
static int allocate_fifo_entry(struct platform_device *dev,
			       struct catscp_channel *chan,
			       struct catscp_fifo_entry *entry)
{
	int rc = 0;
	struct catscp_priv *priv = NULL;

	priv = platform_get_drvdata(dev);

	memset(entry, 0, sizeof(*entry));

	entry->buffer = dma_pool_zalloc(
	    chan->buffer_pool, GFP_KERNEL, &entry->buffer_phys);
	if (!entry->buffer) {
		rc = -ENOMEM;
		goto alloc_failure;
	}
	/* Sanity checking the linux dev should default to 32-bit */
	if (WARN_ONCE(entry->buffer_phys > DMA_BIT_MASK(32),
		      "Hardware only supports 32-bit DMA")) {
		dma_pool_free(chan->buffer_pool,
			      entry->buffer,
			      entry->buffer_phys);
		rc = -ENOMEM;
		goto alloc_failure;
	}

	return 0;

alloc_failure:
	/*
	 * Since the pools are attached to the device, no specific cleanup is
	 * required on allocation failure.
	 */
	return rc;
}

/**
 * link_is_up() - check if link is up.
 * @priv	private context structure
 *
 * @return	true if link is up, false otherwise.
 */
static bool link_is_up(struct catscp_priv *priv)
{
	bool link_up = ((catscp_reg_read(priv, SCP_LNK_STATUS_REG_OFFSET) &
			 SCP_LNK_STATUS__PORT_STATE_STATUS_bm) >>
			SCP_LNK_STATUS__PORT_STATE_STATUS_bp) == STATE_UP;

	return link_up;
}

/**
 * rx_reset() - Reset RX FIFO.
 * @context	private context structure
 */
static void rx_reset(struct catscp_priv *priv)
{
	u32 wr_idx;
	struct catscp_channel *chan = &priv->rx;
	unsigned long flags;

	/*
	 * This spinlock protects access to the channel
	 * FIFO indices both in hardware and software.
	 */
	spin_lock_irqsave(&chan->lock, flags);

	catscp_reg_write(priv, SCP_MEM_RX_FIFO_FLAGS_REG_OFFSET, 0);

	memset(chan->desc, 0, chan->desc_size);

	/* Drain writes before assigning descriptor ring buffer. */
	smp_mb();

	catscp_reg_write(priv, SCP_MEM_RX_FIFO_BA_REG_OFFSET,
			 (u32)chan->desc_phys);

	/* Reset all indexes to the WR_IDX location. */
	wr_idx = (catscp_reg_read(priv, SCP_MEM_RX_FIFO_WR_IDX_REG_OFFSET) &
		  SCP_MEM_RX_FIFO_WR_IDX__RX_WR_IDX_bm) >>
		 SCP_MEM_RX_FIFO_WR_IDX__RX_WR_IDX_bp;

	catscp_reg_write(priv, SCP_MEM_RX_FIFO_RD_IDX_REG_OFFSET, wr_idx);
	chan->desc_index = wr_idx;
	chan->last_complete_index = wr_idx;

	/* Assign timeout on reads for coalescing. */
	catscp_reg_write(priv, SCP_MEM_RX_FIFO_TIMEOUT_REG_OFFSET,
			 priv->rx_timeout);

	/* Enable RX flow. */
	catscp_reg_write(
	    priv, SCP_MEM_RX_FIFO_FLAGS_REG_OFFSET,
	    SCP_MEM_RX_FIFO_FLAGS__RX_ENABLE_bm |
		((ilog2(chan->hw_desc_count) - ilog2(HW_FIFO_COUNT_MIN))
		 << SCP_MEM_RX_FIFO_FLAGS__RX_LENGTH_bp));

	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * rx_can_fill_free_fifo() - test if the hardware receive fifo can be filled
 * @priv:	private context structure
 *
 * @return	true if ready to fill, else false
 *
 * Read the fifo status and the current kfifo of free buffers to determine
 * if a new buffer can be pushed onto the hardware free fifo to use for receive.
 */
static bool rx_can_fill_free_fifo(struct catscp_priv *priv)
{
	struct catscp_channel *chan = &priv->rx;
	u32 rd_idx = catscp_reg_read(priv, SCP_MEM_RX_FIFO_RD_IDX_REG_OFFSET) &
		     SCP_MEM_RX_FIFO_RD_IDX__RX_RD_IDX_bm;

	return ((rd_idx + 1) % chan->hw_desc_count) !=
		   chan->last_complete_index &&
	       !kfifo_is_empty(&chan->free_fifo);
}

/**
 * catscp_fill_rx_fifo_thread() - RX FIFO fill thread
 * @context	private context structure
 *
 * @return	zero on successful termination, non-zero on failure.
 *
 * This thread fills the RX free fifos with new buffers to receive. This helps
 * honor the single-reader/single-writer contracts of the kfifos.
 */
static int catscp_fill_rx_fifo_thread(void *context)
{
	struct catscp_priv *priv = (struct catscp_priv *)context;
	struct catscp_channel *chan = &priv->rx;
	u32 desc_phys = (u32)((uintptr_t)chan->desc_phys);
	unsigned long flags;

	/*
	 * Interruptable condition is that new free buffers are available,
	 * so we should check if we can put more onto the RX hardware free fifo.
	 */
	while (!kthread_should_stop()) {
		/* Wait for more free buffers. */
		if (wait_event_interruptible(chan->free_queue,
					     kthread_should_stop() ||
						 rx_can_fill_free_fifo(priv))) {
			/* interrupted, loop to see if we should stop. */
			continue;
		}

		/* Did we wake up because we should exit? */
		if (kthread_should_stop()) {
			break;
		}

		/*
		 * Until we're out of free buffers or the hardware is full,
		 * push new buffers onto the hardware free fifo.
		 */
		while (rx_can_fill_free_fifo(priv)) {
			struct catscp_fifo_entry entry;
			struct scp_descriptor *desc = chan->desc;

			/* Get a buffer from the free kfifo */
			if (WARN_ONCE(!kfifo_get(&chan->free_fifo, &entry),
				      "Failed to get free entry\n")) {
				break;
			}

			/*
			 * This spinlock protects access to the channel
			 * FIFO indices both in hardware and software.
			 */
			spin_lock_irqsave(&chan->lock, flags);

			/* Make sure entry.data_len is zero. */
			entry.data_len = 0;

			/* Write into the next free descriptor entry. */
			desc[chan->desc_index].address =
			    (u32)((uintptr_t)entry.buffer_phys);
			desc[chan->desc_index].length = chan->buffer_size;

			/* Store descriptor addresses for consistency tracking
			 */
			entry.desc = (void *)&desc[chan->desc_index];
			entry.desc_phys =
			    desc_phys +
			    (chan->desc_index * sizeof(struct scp_descriptor));

			/* Advance to next index, set read register. */
			chan->desc_index =
			    (chan->desc_index + 1) % chan->hw_desc_count;

			/* Mark as in-use. */
			kfifo_put(&chan->used_fifo, entry);

			/* Drain any writes then update the read pointer. */
			smp_mb();
			catscp_reg_write(priv,
					 SCP_MEM_RX_FIFO_RD_IDX_REG_OFFSET,
					 chan->desc_index);

			spin_unlock_irqrestore(&chan->lock, flags);

			/* Indicate that it is possible to receive */
			wake_up_interruptible(&chan->used_queue);
		}
	}

	return 0;
}

/**
 * tx_reset() - Reset TX FIFO.
 * @context	private context structure
 * @high_priority whether to use the TXH or TXL FIFO
 */
static void tx_reset(struct catscp_priv *priv, bool high_priority)
{
	struct catscp_channel *chan = high_priority ? &priv->txh : &priv->tx;
	u32 offset = high_priority ?
		SCP_MEM_TXH_FIFO_BA_REG_OFFSET : SCP_MEM_TXL_FIFO_BA_REG_OFFSET;
	u32 rd_idx;
	unsigned long flags;

	/*
	 * This spinlock protects access to the channel
	 * FIFO indices both in hardware and software.
	 */
	spin_lock_irqsave(&chan->lock, flags);

	catscp_reg_write(priv, offset + FIFO_FLAGS_RELATIVE_OFFSET, 0);

	memset(chan->desc, 0, chan->desc_size);

	/* Drain writes before assigning descriptor ring buffer. */
	smp_mb();

	catscp_reg_write(priv, offset + FIFO_BA_RELATIVE_OFFSET,
			 (u32)chan->desc_phys);

	/* Reset all indexes to the RD_IDX location. */
	rd_idx = catscp_reg_read(priv, offset + FIFO_RD_IDX_RELATIVE_OFFSET) &
				 SCP_MEM_TXL_FIFO_RD_IDX__TXL_RD_IDX_bm;

	catscp_reg_write(priv, offset + FIFO_WR_IDX_RELATIVE_OFFSET, rd_idx);
	chan->desc_index = rd_idx;
	chan->last_complete_index = rd_idx;

	/* Enable TX flow. */
	catscp_reg_write(
	    priv, offset + FIFO_FLAGS_RELATIVE_OFFSET,
	    SCP_MEM_TXL_FIFO_FLAGS__TXL_ENABLE_bm |
		((ilog2(chan->hw_desc_count) - ilog2(HW_FIFO_COUNT_MIN))
		 << SCP_MEM_TXL_FIFO_FLAGS__TXL_LENGTH_bp));

	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * tx_can_fill_work_fifo() - test if the hardware transmit fifo can be filled
 * @priv:	private context structure
 * @high_priority:	whether to use the TXH or TXL fifo
 *
 * @return	true if ready to fill, else false
 *
 * Read the fifo status and the current kfifo of used buffers to determine
 * if a new buffer can be pushed onto the hardware work fifo to transmit.
 */
static bool tx_can_fill_work_fifo(struct catscp_priv *priv, bool high_priority)
{
	struct catscp_channel *chan = high_priority ? &priv->txh : &priv->tx;
	u32 offset = high_priority ?
		SCP_MEM_TXH_FIFO_BA_REG_OFFSET : SCP_MEM_TXL_FIFO_BA_REG_OFFSET;
	u32 wr_idx = catscp_reg_read(priv,
					 offset + FIFO_WR_IDX_RELATIVE_OFFSET) &
				     SCP_MEM_TXL_FIFO_WR_IDX__TXL_WR_IDX_bm;

	return (((wr_idx + 1) % chan->hw_desc_count) !=
		chan->last_complete_index) &&
	       !kfifo_is_empty(&chan->used_fifo);
}

/**
 * check_tx_gate_msg() - Check that TX gate messages meet thresholds.
 * @priv:	private context structure
 * @entry:	fifo buffer entry
 *
 * @return	true if gate meets all checks, else false.
 *
 * SCP gate messages may be sent shortly (<1ms) before the TTL gate time. If
 * the message doesn't make into hardware before that time the SCP master will
 * block until the TTL counter wraps (roughly 280ms) which causes missed slots.
 *
 * In order to handle unexpected preemption we check the gate timing in the
 * kernel as close as possible to when we write the message to hardware.
 *
 * This function should be called with interrupts disabled immediately after
 * transmitting the gate message.
 */
static void check_tx_gate_msg(struct catscp_priv *priv,
						struct catscp_fifo_entry entry)
{
	/* Make sure we are just checking gate messages. */
	if (entry.data_len == SCP_GATE_SIZE) {
		u32 ttl_gate, ttl_now, ttl_prev;
		int margin, ms_delta, ttl_delta;

		/* Check gate time against current TTL counter value. */
		ttl_gate = (*(u32 *)entry.buffer) >> SCP_LEN_BITS;
		ttl_now =
			catscp_reg_read(priv, SCP_CMN_TTL_CNTR_REG_OFFSET) & 0xFFFFFFu;
		margin = sign_extend32(ttl_gate - ttl_now, SCP_TTL_BITS - 1);

		if (margin < priv->gate.min_ttl) {
			atomic_inc(&priv->stats.gate_under_min_count);
		} else if (margin > priv->gate.max_ttl) {
			atomic_inc(&priv->stats.gate_over_max_count);
		}

		/* Check gate time against previous gate time. */
		ttl_prev = priv->gate.prev_ttl;
		ttl_delta = sign_extend32(ttl_gate - ttl_prev, SCP_TTL_BITS - 1);
		ms_delta = jiffies_to_msecs(jiffies - priv->gate.prev_jiffies);

		if (ttl_delta < 0 && ms_delta < 100) {
			atomic_inc(&priv->stats.gate_unordered_count);
		} else if (ttl_delta < priv->gate.time_between_ttl && ms_delta < 100) {
			atomic_inc(&priv->stats.gate_low_time_between_count);
		}

		priv->gate.prev_ttl = ttl_gate;
		priv->gate.prev_jiffies = jiffies;

		/* Update timing margin stats. */
		if (margin < atomic_read(&priv->stats.gate_min_margin))
			atomic_set(&priv->stats.gate_min_margin, margin);
		else if (margin > atomic_read(&priv->stats.gate_max_margin))
			atomic_set(&priv->stats.gate_max_margin, margin);
	}
}

/**
 * catscp_fill_tx_fifo_thread() - TX FIFO fill thread
 * @context	context private context structure
 * @high_priority whether using TXH or TXL FIFO
 *
 * @return	zero on successful termination, non-zero on failure.
 *
 * This thread fills the TX work fifos with new buffers to send.
 */
static int catscp_fill_tx_fifo_thread(void *context, bool high_priority)
{
	struct catscp_priv *priv = (struct catscp_priv *)context;
	u32 offset = high_priority ?
		SCP_MEM_TXH_FIFO_BA_REG_OFFSET : SCP_MEM_TXL_FIFO_BA_REG_OFFSET;
	struct catscp_channel *chan = high_priority ? &priv->txh : &priv->tx;
	unsigned long flags;

	/*
	 * Interruptable condition that buffers are available to transmit, and
	 * should be added to the TX work fifo, if space is available.
	 */
	while (!kthread_should_stop()) {
		/* Wait for more used buffers. */
		if (wait_event_interruptible(chan->used_queue,
					     kthread_should_stop() ||
						 tx_can_fill_work_fifo(priv, high_priority))) {
			/* interrupted, loop to see if we should stop. */
			continue;
		}

		/* Did we wake up because we should exit? */
		if (kthread_should_stop()) {
			break;
		}

		/*
		 * Until we're out of transmit buffers, try and push them
		 * on to the work fifo.
		 */
		while (tx_can_fill_work_fifo(priv, high_priority)) {
			struct catscp_fifo_entry entry;
			struct scp_descriptor *desc;

			/* Get a buffer from the used kfifo, if available. */
			if (!kfifo_get(&chan->used_fifo, &entry)) {
				break;
			}

			/*
			 * To transmit a message we must have a valid link along
			 * with confirmation of a message from the peer.
			 * Otherwise drop the message and increment a counter.
			 */
			if (link_is_up(priv) && priv->evt_msg_received) {
				/*
				 * This spinlock protects access to the channel
				 * FIFO indices both in hardware and software.
				 */
				spin_lock_irqsave(&chan->lock, flags);

				/* Write into the next free descriptor entry. */
				desc = &(((struct scp_descriptor *)
					      chan->desc)[chan->desc_index]);
				desc->address = (u32)((uintptr_t)entry.buffer_phys);
				desc->length = entry.data_len;

				/*
				 * Update the descriptor addresses for consistency
				 * tracking.
				 */
				entry.desc = (void *)desc;
				entry.desc_phys =
				    (u32)((uintptr_t)chan->desc_phys) +
				    chan->desc_index * sizeof(struct scp_descriptor);

				/* Advance to next index, set read register. */
				chan->desc_index =
				    (chan->desc_index + 1) % chan->hw_desc_count;

				/* Move it to the completed FIFO. */
				kfifo_put(&chan->complete_fifo, entry);

				/* Drain any writes then update the write pointer. */
				smp_mb();
				catscp_reg_write(priv,
						 offset + FIFO_WR_IDX_RELATIVE_OFFSET,
						 chan->desc_index);

				atomic_inc(&priv->stats.hw_write_count);

				spin_unlock_irqrestore(&chan->lock, flags);

				/* Collect TTL stats from high-priority queue gate messages. */
				if (high_priority)
					check_tx_gate_msg(priv, entry);
			} else {
				/* Drop packets when the link is down. */
				kfifo_put(&chan->free_fifo, entry);
				atomic_inc(&priv->stats.hw_write_drop_count);
			}
		}
	}

	return 0;
}

/**
 * catscp_fill_txl_fifo_thread() - TXL FIFO fill thread
 * @context	context private context structure
 *
 * @return	zero on successful termination, non-zero on failure.
 *
 * This thread fills the TXL work fifos with new buffers to send.
 */
static int catscp_fill_txl_fifo_thread(void *context)
{
	return catscp_fill_tx_fifo_thread(context, false);
}

/**
 * catscp_fill_txh_fifo_thread() - TXH FIFO fill thread
 * @context	context private context structure
 *
 * @return	zero on successful termination, non-zero on failure.
 *
 * This thread fills the TXH work fifos with new buffers to send.
 */
static int catscp_fill_txh_fifo_thread(void *context)
{
	return catscp_fill_tx_fifo_thread(context, true);
}

/**
 * catscp_enable_set_clock() - configure clock settings
 * @priv:	private context structure
 */
static void catscp_enable_set_clock(struct catscp_priv *priv)
{
	u32 val;

	/*
	 * Enable core for configuration changes to take effect.
	 */
	if (priv->link_speed == LINK_SPEED_20MBIT) {
		val = SCP_CMN_CTRL__SCP_EN_bm |
		      ((1 << SCP_CMN_CTRL__SCP_DIVIDER_bp) &
		       SCP_CMN_CTRL__SCP_DIVIDER_bm) |
		      ((1 << SCP_CMN_CTRL__OUT_DIVIDER_bp) &
		       SCP_CMN_CTRL__OUT_DIVIDER_bm) |
		      SCP_CMN_CTRL__BYPASS_DIV3_PHI_bm |
		      SCP_CMN_CTRL__BYPASS_RESET_STRECH_bm |
		      SCP_CMN_CTRL__PLL_BYPASS_bm;
	} else if (priv->link_speed == LINK_SPEED_600MBIT) {
		val = SCP_CMN_CTRL__SCP_EN_bm |
		      ((2 << SCP_CMN_CTRL__SCP_DIVIDER_bp) &
		       SCP_CMN_CTRL__SCP_DIVIDER_bm) |
		      ((10 << SCP_CMN_CTRL__OUT_DIVIDER_bp) &
		       SCP_CMN_CTRL__OUT_DIVIDER_bm) |
		      SCP_CMN_CTRL__BYPASS_RESET_STRECH_bm;
	} else {
		dev_err(priv->dev, "Invalid link speed %d\n", priv->link_speed);
		return;
	}

	catscp_reg_write(priv, SCP_CMN_CTRL_REG_OFFSET, val);
}

/**
 * catscp_reset() - reset the controller
 * @priv:	private context structure
 *
 * @return	zero on success, non-zero on failure.
 */
static int catscp_reset(struct catscp_priv *priv)
{
	int ret = 0;

	if (mutex_lock_killable(&priv->reg_mutex)) {
		dev_err(priv->dev, "Caught signal in catscp_reset\n");
		ret = -EINTR;
		goto out;
	}

	ret = reset_control_assert(priv->scp_reset);
	if (ret) {
		dev_err(priv->dev, "can't assert reset\n");
		goto out_unlock;
	}

	udelay(SCP_RESET_HOLD_TIME_US);

	ret = reset_control_deassert(priv->scp_reset);
	if (ret) {
		dev_err(priv->dev, "can't deassert reset\n");
		goto out_unlock;
	}

out_unlock:
	mutex_unlock(&priv->reg_mutex);
out:
	return ret;
}

/**
 * catscp_setup() - configure the controller
 * @priv:	private context structure
 */
static void catscp_setup(struct catscp_priv *priv)
{
	u32 val;
	u32 irq_stat;

	catscp_enable_set_clock(priv);

	val = catscp_reg_read(priv, SCP_LNK_CTRL_REG_OFFSET);
	val |=
	    SCP_LNK_CTRL__TX_RELATIVE_TTL_bm | SCP_LNK_CTRL__RX_RELATIVE_TTL_bm;
	catscp_reg_write(priv, SCP_LNK_CTRL_REG_OFFSET, val);

	val =
	    ((90 << SCP_TURNAROUND_CONTROL__RX_PORT_INPUT_DISABLE_DURATION_bp) &
	     SCP_TURNAROUND_CONTROL__RX_PORT_INPUT_DISABLE_DURATION_bm) |
	    ((15 << SCP_TURNAROUND_CONTROL__TX_PORT_DUMMY_SYMBOL_DURATION_bp) &
	     SCP_TURNAROUND_CONTROL__TX_PORT_DUMMY_SYMBOL_DURATION_bm);
	catscp_reg_write(priv, SCP_TURNAROUND_CONTROL_REG_OFFSET, val);

	/* Then reset all FIFOs. */
	rx_reset(priv);
	tx_reset(priv, false);
	tx_reset(priv, true);

	/*
	 * Clear any interrupt status then enable each block's interrupts.
	 */
	irq_stat = catscp_reg_read(priv, SCP_CMN_ITS_REG_OFFSET);
	catscp_reg_write(priv, SCP_CMN_ITS_BCLR_REG_OFFSET, irq_stat);

	catscp_reg_write(priv, SCP_CMN_ITM_BCLR_REG_OFFSET, SCP_CLEAR_IRQ_MASK);
	catscp_reg_write(priv, SCP_CMN_ITM_BSET_REG_OFFSET, SCP_SET_IRQ_MASK);
}

/**
 * catscp_set_link() - set link state
 * @priv:	private context structure
 * @enable:	enable or disable link
 */
static void catscp_set_link(struct catscp_priv *priv, bool enable)
{
	u32 val;

	/* Reset event message state. */
	priv->evt_msg_received = false;

	if (enable) {
		/* Re-initialize clock settings in case speed has changed. */
		catscp_enable_set_clock(priv);

		/*
		 * Enable low power mode.
		 */
		val = catscp_reg_read(priv, SCP_PHY_CTRL_REG_OFFSET);
		val |= SCP_PHY_CTRL__SCP_DATA_LPE_bm;
		catscp_reg_write(priv, SCP_PHY_CTRL_REG_OFFSET, val);

		/*
		 * Select the clock source and wait for a lock. This must be done
		 * as a separate action before outputting a clock to allow time
		 * for the PLL to lock.
		 */
		val = catscp_reg_read(priv, SCP_PHY_CTRL_REG_OFFSET);
		if (priv->link_speed == LINK_SPEED_20MBIT) {
			val &= ~SCP_PHY_CTRL__RX_CLK_REF_SEL_bm;
		} else if (priv->link_speed == LINK_SPEED_600MBIT) {
			val |= SCP_PHY_CTRL__RX_CLK_REF_SEL_bm;
		}
		catscp_reg_write(priv, SCP_PHY_CTRL_REG_OFFSET, val);

		udelay(PLL_LOCK_US);

		/*
		 * Low power is set and the clock is ready, enable output.
		 */
		val = catscp_reg_read(priv, SCP_PHY_CTRL_REG_OFFSET);
		val |= SCP_PHY_CTRL__CLK_DIRECTION_bm;
		catscp_reg_write(priv, SCP_PHY_CTRL_REG_OFFSET, val);

		/*
		 * Give link partner some time to lock.
		 */
		udelay(LINK_DELAY_US);

		/*
		 * Take the link up.
		 */
		val = catscp_reg_read(priv, SCP_LNK_CTRL_REG_OFFSET);
		val &= ~SCP_LNK_CTRL__PORT_STATE_CTRL_bm;
		val |= STATE_UP << SCP_LNK_CTRL__PORT_STATE_CTRL_bp;
		catscp_reg_write(priv, SCP_LNK_CTRL_REG_OFFSET, val);
	} else {
		/*
		 * Disable the clock and the set the link down.
		 */
		val = catscp_reg_read(priv, SCP_PHY_CTRL_REG_OFFSET);
		val &= ~SCP_PHY_CTRL__CLK_DIRECTION_bm;
		catscp_reg_write(priv, SCP_PHY_CTRL_REG_OFFSET, val);

		val = catscp_reg_read(priv, SCP_LNK_CTRL_REG_OFFSET);
		val &= ~SCP_LNK_CTRL__PORT_STATE_CTRL_bm;
		val |= STATE_DEFAULT << SCP_LNK_CTRL__PORT_STATE_CTRL_bp;
		catscp_reg_write(priv, SCP_LNK_CTRL_REG_OFFSET, val);
	}
}


/**
 * catscp_reset_channel_fifos() - Resets the fifos.
 * @priv:	private context structure
 * @chan	channel owning the buffer pool
 *
 * @return	zero on success, non-zero on failure.
 */
static int catscp_reset_channel_fifos(struct catscp_priv *priv,
				      struct catscp_channel *chan)
{
	struct catscp_fifo_entry entry;
	unsigned long flags;

	/* Acquire channel I/O mutex. */
	if (mutex_lock_interruptible(&chan->io_mutex)) {
		dev_err(priv->dev, "Caught signal in catscp_reset_channel_fifos\n");
		return -ERESTARTSYS;
	}

	/* Acquire channel spinlock to protect KFIFOs. */
	spin_lock_irqsave(&chan->lock, flags);

	while (kfifo_get(&chan->used_fifo, &entry)) {
		kfifo_put(&chan->free_fifo, entry);
	}

	while (kfifo_get(&chan->complete_fifo, &entry)) {
		kfifo_put(&chan->free_fifo, entry);
	}

	spin_unlock_irqrestore(&chan->lock, flags);
	mutex_unlock(&chan->io_mutex);

	return 0;
}

/**
 * catscp_deinit_channel() - Free channel resources
 * @dev		platform device
 * @chan	channel owning the buffer pool
 *
 * Note the dma pools are managed, so no frees are required.
 */
static void catscp_deinit_channel(struct platform_device *dev,
				  struct catscp_channel *chan)
{
	struct catscp_priv *priv = NULL;

	priv = platform_get_drvdata(dev);

	kfifo_free(&chan->free_fifo);
	kfifo_free(&chan->used_fifo);
	kfifo_free(&chan->complete_fifo);

	if (chan->desc)
		dma_free_coherent(&dev->dev, chan->desc_size, chan->desc,
				  chan->desc_phys);

	mutex_destroy(&chan->io_mutex);
}

/**
 * catscp_init_channel() - Initialize a channel
 * @dev		platform device
 * @chan	channel owning the buffer pool
 * @pool_name	string identifying pool use
 *
 * @return	zero on successful allocation, non-zero on failure.
 */
static int catscp_init_channel(struct platform_device *dev,
			       struct catscp_channel *chan,
			       const char *pool_name)
{
	int rc = 0;
	int kfifo_size;
	u32 i;
	struct catscp_priv *priv = NULL;
	struct catscp_fifo_entry entry;

	priv = platform_get_drvdata(dev);

	/* Allocate DMA buffer pool. */
	chan->buffer_pool = dmam_pool_create(
	    pool_name, &dev->dev, chan->buffer_size, chan->buffer_size, 0);
	if (!chan->buffer_pool) {
		dev_err(&dev->dev, "dmam_pool_create failed\n");
		rc = -ENOMEM;
		goto buffer_create_failed;
	}

	/*
	 * Allocate the kfifo structures. Each one is large enough to support
	 * all allocated buffers, so they should be incapable of overflowing.
	 */
	kfifo_size = roundup_pow_of_two(chan->buffer_count *
					sizeof(struct catscp_fifo_entry));

	rc = kfifo_alloc(&chan->free_fifo, kfifo_size, GFP_KERNEL);
	if (rc) {
		goto free_alloc_failed;
	}

	rc = kfifo_alloc(&chan->used_fifo, kfifo_size, GFP_KERNEL);
	if (rc) {
		goto used_alloc_failed;
	}

	rc = kfifo_alloc(&chan->complete_fifo, kfifo_size, GFP_KERNEL);
	if (rc) {
		goto complete_alloc_failed;
	}

	chan->desc_size = round_up(
	    chan->hw_desc_count * sizeof(struct scp_descriptor), PAGE_SIZE);
	chan->desc =
	    dma_alloc_coherent(&dev->dev, chan->desc_size, &chan->desc_phys,
			       GFP_KERNEL);
	if (!chan->desc) {
		rc = -ENOMEM;
		goto desc_alloc_failed;
	}
	/* Sanity checking the linux dev should default to 32-bit */
	if (WARN_ONCE(chan->desc_phys > DMA_BIT_MASK(32),
		      "Hardware only supports 32-bit DMA")) {
		rc = -ENOMEM;
		goto desc_alloc_invalid;
	}

	memset(chan->desc, 0, chan->desc_size);

	/* Initialize the wait queues. */
	init_waitqueue_head(&chan->free_queue);
	init_waitqueue_head(&chan->used_queue);
	init_waitqueue_head(&chan->complete_queue);

	/* Init the channel io lock. */
	mutex_init(&chan->io_mutex);

	/* And initialize register protection */
	spin_lock_init(&chan->lock);

	/* Allocate and add all buffers to the free kfifo. */
	for (i = 0; i < chan->buffer_count; i++) {
		rc = allocate_fifo_entry(dev, chan, &entry);
		if (rc) {
			goto entry_alloc_failed;
		}

		/*
		 * Add to free kfifo. Since this is init,
		 * defer signalling the wait queue.
		 */
		kfifo_put(&chan->free_fifo, entry);
	}

	return 0;

entry_alloc_failed:
	mutex_destroy(&chan->io_mutex);
desc_alloc_invalid:
	dma_free_coherent(&dev->dev, chan->desc_size, chan->desc,
			  chan->desc_phys);
desc_alloc_failed:
	kfifo_free(&chan->complete_fifo);
complete_alloc_failed:
	kfifo_free(&chan->used_fifo);
used_alloc_failed:
	kfifo_free(&chan->free_fifo);
free_alloc_failed:
buffer_create_failed:
	return rc;
}

/**
 * catscp_register_device() - registers a scp device
 * @priv:	private device data
 *
 * @return	zero on success, non-zero on failure
 *
 * This adds the scp devices to the lookup table so that open() on
 * the character device works.
 */
int catscp_register_device(struct catscp_priv *priv)
{
	int minor = 1;
	int rc = 0;
	dev_t dev_id;
	struct device *device;

	/* Write and read device. */
	dev_id = MKDEV(MAJOR(priv->major), minor);
	cdev_init(&priv->cdev, &catscp_fops);
	rc = cdev_add(&priv->cdev, dev_id, minor);
	if (rc) {
		dev_err(priv->dev, "Failed to add char device catscp0 (%d)\n", rc);
		goto out;
	}

	device = device_create(catscp_class, NULL, dev_id, NULL, "catscp0");
	if (IS_ERR(device)) {
		rc = PTR_ERR(device);
		dev_err(priv->dev, "Failed to create char device catscp0 (%d)\n", rc);
		goto out_cdev_del;
	}

	/* Write only device. */
	minor = 2;
	dev_id = MKDEV(MAJOR(priv->major), minor);
	cdev_init(&priv->cdev_txh, &catscp_fops_txh);
	rc = cdev_add(&priv->cdev_txh, dev_id, minor);
	if (rc) {
		dev_err(priv->dev, "Failed to add char device catscp1 (%d)\n", rc);
		goto out_cdev_del;
	}

	device = device_create(catscp_class, NULL, dev_id, NULL, "catscp1");
	if (IS_ERR(device)) {
		rc = PTR_ERR(device);
		dev_err(priv->dev, "Failed to create char device catscp1 (%d)\n", rc);
		goto out_cdev_txh_del;
	}

	return 0;

out_cdev_txh_del:
	cdev_del(&priv->cdev_txh);
out_cdev_del:
	cdev_del(&priv->cdev);
out:
	return rc;
}

/**
 * catscp_unregister_device() - removes a scp device
 * @priv:	private device data
 *
 * This removes the device from the minor-number lookup table.
 */
void catscp_unregister_devices(struct catscp_priv *priv)
{
	/* Destroy push-pull device. */
	device_destroy(catscp_class, priv->cdev.dev);
	cdev_del(&priv->cdev);

	/* Destroy pull device. */
	device_destroy(catscp_class, priv->cdev_txh.dev);
	cdev_del(&priv->cdev_txh);
}

/**
 * catscp_open() - opens a scp char device
 * @inode:	the &struct inode the user wants to open
 * @filp:	pointer to &struct file to be initialized
 *
 * @return	zero on success, non-zero on failure
 */
static int catscp_open(struct inode *inode, struct file *filp)
{
	struct catscp_priv *priv = NULL;

	priv = container_of(inode->i_cdev, struct catscp_priv, cdev);
	filp->private_data = priv; /* for other methods */

	return 0;
}

/**
 * catscp_open() - opens a scp char device
 * @inode:	the &struct inode the user wants to open
 * @filp:	pointer to &struct file to be initialized
 *
 * @return	zero on success, non-zero on failure
 */
static int catscp_open_txh(struct inode *inode, struct file *filp)
{
	struct catscp_priv *priv = NULL;

	priv = container_of(inode->i_cdev, struct catscp_priv, cdev_txh);
	filp->private_data = priv; /* for other methods */

	return 0;
}

/**
 * catscp_read() - reads from a pull fifo
 * @filp:	&struct file pointer for the device file
 * @buf:	userspace buffer
 * @count:	maximum number of bytes to read
 * @pos:	position in the file (ignored -- this is a stream)
 *
 * @return	number of bytes read on success, negative value on failure
 */
static ssize_t catscp_read(struct file *filp, char __user *buf, size_t count,
			   loff_t *pos)
{
	struct catscp_priv *priv = filp->private_data;
	struct catscp_channel *chan = &priv->rx;
	struct catscp_fifo_entry entry;
	ssize_t rc = 0;

	/* Acquire for wait, first-come, first-serve. */
	if (mutex_lock_interruptible(&chan->io_mutex)) {
		dev_err(priv->dev, "Caught signal in catscp_read\n");
		return -ERESTARTSYS;
	}

	/*
	 * Check if the FIFO is currently empty, and then either wait for
	 * data or return EAGAIN based on flags.
	 */
	if (kfifo_is_empty(&chan->complete_fifo)) {
		/*
		 * To be able to aquire this mutex elsewhere in the driver
		 * when performing a reset, we must release it here while
		 * waiting on available data.
		 */
		mutex_unlock(&chan->io_mutex);
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(
			chan->complete_queue,
			(!kfifo_is_empty(&chan->complete_fifo)))) {
			/* pass it up to the FS layer */
			dev_err(priv->dev,
				"PID %d (%s) caught signal on read\n",
				current->pid, current->comm);
			return -ERESTARTSYS;
		}

		/* Re-acquire for wait, first-come, first-serve. */
		if (mutex_lock_interruptible(&chan->io_mutex)) {
			dev_err(priv->dev, "Caught signal in catscp_read\n");
			return -ERESTARTSYS;
		}
	}

	/* Peek first and see if it's too large for the caller buffer. */
	if (!kfifo_peek(&chan->complete_fifo, &entry)) {
		rc = -EIO;
		goto out;
	}

	if (entry.data_len > count) {
		rc = -EFBIG;
		goto out;
	}

	if (kfifo_get(&chan->complete_fifo, &entry)) {
		rc = copy_to_user(buf, entry.buffer, entry.data_len);

		/* If the copy fails don't return success. */
		if (WARN_ONCE(rc, "Copy to user failed: %zd\n", rc)) {
			/* The packet is lost in this case. */
			atomic_inc(&priv->stats.sw_read_drop_count);
			rc = -EIO;
		} else {
			atomic_inc(&priv->stats.sw_read_count);
			rc = entry.data_len;
		}

		/* Return the buffer to the free_fifo */
		kfifo_put(&chan->free_fifo, entry);

		/* Wake the rx fill thread. */
		wake_up_interruptible(&chan->free_queue);
	} else {
		rc = -EIO;
	}

out:
	mutex_unlock(&chan->io_mutex);
	return rc;
}

/**
 * catscp_write() - writes to a fifo
 * @filp:	&struct file pointer for the device file
 * @buf:	userspace buffer to fill
 * @count:	maximum number of bytes to write
 * @pos:	position in the file (ignored -- this is a stream)
 * @high_priority:	whether to use the TXH or TXL FIFO
 *
 * @return	number of bytes read on success, negative value on failure
 */
static ssize_t catscp_write(struct file *filp, const char __user *buf,
			    size_t count, loff_t *pos, bool high_priority)
{
	struct catscp_priv *priv = filp->private_data;
	struct catscp_channel *chan = high_priority ? &priv->txh : &priv->tx;
	struct catscp_fifo_entry entry;
	ssize_t rc = 0;

	if (count > chan->buffer_size) {
		return -EFBIG;
	}

	if (mutex_lock_interruptible(&chan->io_mutex)) {
		dev_err(priv->dev, "Caught signal in catscp_write\n");
		return -ERESTARTSYS;
	}

	/*
	 * Check if the FIFO is currently empty, and then either wait for
	 * data or return EAGAIN based on flags.
	 */
	if (kfifo_is_empty(&chan->free_fifo)) {
		/*
		 * To be able to aquire this mutex elsewhere in the driver
		 * when performing a reset, we must release it here while
		 * waiting on available data.
		 */
		mutex_unlock(&chan->io_mutex);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(
			chan->free_queue,
			(!kfifo_is_empty(&chan->free_fifo)))) {
			dev_err(priv->dev,
				"PID %d (%s) caught signal on read\n",
				current->pid, current->comm);
			/* pass it up to the FS layer */
			return -ERESTARTSYS;
		}

		/* Re-acquire for wait, first-come, first-serve. */
		if (mutex_lock_interruptible(&chan->io_mutex)) {
			dev_err(priv->dev, "Caught signal in catscp_write\n");
			return -ERESTARTSYS;
		}
	}

	if (kfifo_get(&chan->free_fifo, &entry)) {
		rc = copy_from_user(entry.buffer, buf, count);

		/* On error return the buffer to free and fail. */
		if (WARN_ONCE(rc, "Copy from user failed: %zd\n", rc)) {
			kfifo_put(&chan->free_fifo, entry);

			/* This counts as "dropping" a packet. */
			atomic_inc(&priv->stats.sw_write_drop_count);
			rc = -EIO;
			goto out;
		}

		entry.data_len = count;
		rc = count;
		atomic_inc(&priv->stats.sw_write_count);

		kfifo_put(&chan->used_fifo, entry);

		/* Indicate that used buffers are available for transmit. */
		wake_up_interruptible(&chan->used_queue);

	} else {
		rc = -EIO;
	}

out:
	mutex_unlock(&chan->io_mutex);

	return rc;
}

/**
 * catscp_write_txh() - writes to the TXH fifo
 * @filp:	&struct file pointer for the device file
 * @buf:	userspace buffer to fill
 * @count:	maximum number of bytes to write
 * @pos:	position in the file (ignored -- this is a stream)
 *
 * @return	number of bytes read on success, negative value on failure
 */
static ssize_t catscp_write_txh(struct file *filp, const char __user *buf,
			    size_t count, loff_t *pos)
{
	return catscp_write(filp, buf, count, pos, true);
}

/**
 * catscp_write_txl() - writes to the TXL fifo
 * @filp:	&struct file pointer for the device file
 * @buf:	userspace buffer to fill
 * @count:	maximum number of bytes to write
 * @pos:	position in the file (ignored -- this is a stream)
 *
 * @return	number of bytes read on success, negative value on failure
 */
static ssize_t catscp_write_txl(struct file *filp, const char __user *buf,
			    size_t count, loff_t *pos)
{
	return catscp_write(filp, buf, count, pos, false);
}

/**
 * catscp_poll() - queries whether the device appears to be readable/writable
 * @filp:	the &struct file we are querying
 * @wait:	the wait-table; if device is not ready, the process can
 *		optionally be added to the table, sleeping to wait for
 *		readiness.  This parameter can be NULL (e.g. if no waiting
 *		is desired).
 *
 * @return	mask of flags as to ready state
 */
static unsigned int catscp_poll(struct file *filp,
				struct poll_table_struct *wait)
{
	struct catscp_priv *priv = filp->private_data;
	unsigned int mask = 0;

	/*
	 * Register for the revelant wait queues.
	 */
	poll_wait(filp, &priv->tx.free_queue, wait);
	poll_wait(filp, &priv->rx.complete_queue, wait);

	/*
	 * If there's an available transmit buffer, then it is ok to call
	 * write().
	 */
	if (!kfifo_is_empty(&priv->tx.free_fifo)) {
		mask |= POLLOUT | POLLWRNORM;
	}

	/*
	 * If the receive complete fifo is not empty, then there is a packet
	 * to read().
	 */
	if (!kfifo_is_empty(&priv->rx.complete_fifo)) {
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

/**
 * catscp_poll_txh() - queries whether the high-priority device appears to be
 *		writable.
 * @filp:	the &struct file we are querying
 * @wait:	the wait-table; if device is not ready, the process can
 *		optionally be added to the table, sleeping to wait for
 *		readiness.  This parameter can be NULL (e.g. if no waiting
 *		is desired).
 *
 * @return	mask of flags as to ready state
 */
static unsigned int catscp_poll_txh(struct file *filp,
				struct poll_table_struct *wait)
{
	struct catscp_priv *priv = filp->private_data;
	unsigned int mask = 0;

	/*
	 * Register for the revelant wait queues.
	 */
	poll_wait(filp, &priv->txh.free_queue, wait);

	/*
	 * If there's an available transmit buffer, then it is ok to call
	 * write().
	 */
	if (!kfifo_is_empty(&priv->txh.free_fifo)) {
		mask |= POLLOUT | POLLWRNORM;
	}

	return mask;
}

/**
 * catscp_rx_irq() - handle rx interrupts
 * @priv:	device context
 *
 * @return	irq handled status
 */
static irqreturn_t catscp_rx_irq(struct catscp_priv *priv)
{
	struct catscp_channel *chan = &priv->rx;
	u32 desc_phys;
	u32 wr_idx;
	struct catscp_fifo_entry entry;
	struct scp_descriptor *desc;
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;

	wr_idx = (catscp_reg_read(priv, SCP_MEM_RX_FIFO_WR_IDX_REG_OFFSET) &
		  SCP_MEM_RX_FIFO_WR_IDX__RX_WR_IDX_bm) >>
		 SCP_MEM_RX_FIFO_WR_IDX__RX_WR_IDX_bp;

	while (chan->last_complete_index != wr_idx) {
		WARN_ONCE(kfifo_is_empty(&chan->used_fifo),
			  "Received interrupt without any outstanding descriptors\n");

		/*
		 * This spinlock protects access to the channel
		 * FIFO indices both in hardware and software.
		 */
		spin_lock_irqsave(&chan->lock, flags);

		if (kfifo_get(&chan->used_fifo, &entry)) {
			desc = (struct scp_descriptor *)entry.desc;
			desc_phys = (u32)chan->desc_phys +
				    (chan->last_complete_index *
				     sizeof(struct scp_descriptor));

			WARN_ONCE(entry.desc_phys != desc_phys,
				  "Hardware descriptor 0x%x does not match queued 0x%x\n",
				  entry.desc_phys, desc_phys);
			WARN_ONCE(desc->length == 0,
				  "Received zero sized packets\n");
			WARN_ONCE(desc->address != entry.buffer_phys,
				  "Descriptor address 0x%x does not match entry physical 0x%llx\n",
				  desc->address, entry.buffer_phys);

			/* Truncate buffers with invalid lengths to zero. */
			if (WARN_ONCE(desc->length > chan->buffer_size,
				      "Entry (%d) exceeds maximum buffer size (%d)\n",
				      desc->length, chan->buffer_size)) {
				entry.data_len = 0;
			} else {
				entry.data_len = desc->length;
			}

			/*
			 * Peek to see if it's the link up event message.
			 * This should never be attached with other messages, and
			 * the link must currently be up for it to count.
			 */
			if (!priv->evt_msg_received && link_is_up(priv)) {
				if ((entry.data_len == 12) && (((u8 *)entry.buffer)[8] == 131)) {
					dev_info(priv->dev, "link up message received\n");
					priv->evt_msg_received = true;
				}
			}

			/* Add buffer to complete fifo. */
			kfifo_put(&chan->complete_fifo, entry);
			atomic_inc(&priv->stats.hw_read_count);

			/* Indicate that work was actually performed. */
			ret = IRQ_HANDLED;
		}

		chan->last_complete_index =
		    (chan->last_complete_index + 1) % chan->hw_desc_count;

		spin_unlock_irqrestore(&chan->lock, flags);
	}

	/*
	 * Indicate to any listeners that there is data to read, and to
	 * any listeners for the free_queue, indicating that there should
	 * be space available now on the hardware fifo.
	 */
	wake_up_interruptible(&chan->complete_queue);
	wake_up_interruptible(&chan->free_queue);

	return ret;
}

/**
 * catscp_tx_irq() - handle tx interrupts
 * @priv:	device context
 # @high_priority: whether to use the TXH or TXL FIFO
 *
 * @return	irq handled status
 */
static irqreturn_t catscp_tx_irq(struct catscp_priv *priv, bool high_priority)
{
	struct catscp_channel *chan = high_priority ? &priv->txh : &priv->tx;
	u32 offset = high_priority ?
		SCP_MEM_TXH_FIFO_BA_REG_OFFSET : SCP_MEM_TXL_FIFO_BA_REG_OFFSET;
	u32 desc_phys;
	u32 wr_idx;
	struct catscp_fifo_entry entry;
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;

	wr_idx = catscp_reg_read(priv, offset + FIFO_WR_IDX_RELATIVE_OFFSET) &
				 SCP_MEM_TXL_FIFO_WR_IDX__TXL_WR_IDX_bm;
	while (chan->last_complete_index != wr_idx) {
		WARN_ONCE(kfifo_is_empty(&chan->complete_fifo),
			  "Received interrupt with no outstanding descriptors\n");

		/*
		 * This spinlock protects access to the channel
		 * FIFO indices both in hardware and software.
		 */
		spin_lock_irqsave(&chan->lock, flags);

		if (kfifo_get(&chan->complete_fifo, &entry)) {
			/*
			 * This tests for ordering in the FIFO - if it hits, it
			 * likely indicates that the hardware has reset
			 * and is now out of sync with software.
			 */
			desc_phys = (u32)chan->desc_phys +
				    (chan->last_complete_index *
				     sizeof(struct scp_descriptor));

			/* Leak the entry and warn if there's a mismatch. */
			if (WARN_ONCE(entry.desc_phys != desc_phys,
				      "Hardware descriptor 0x%x does not match queued 0x%x\n",
				      entry.desc_phys, desc_phys))
				break;

			/*
			 * Return the buffer to the free list and signal
			 * any waiters that a new TX buffer is available.
			 */
			kfifo_put(&chan->free_fifo, entry);
			atomic_inc(&priv->stats.hw_write_complete_count);

			/* Indicate that work was actually performed. */
			ret = IRQ_HANDLED;
		}

		chan->last_complete_index =
		    (chan->last_complete_index + 1) % chan->hw_desc_count;

		spin_unlock_irqrestore(&chan->lock, flags);
	}

	/*
	 * If the transmit work fifo was previously full, there now may
	 * be room to transmit as one or more buffers have completed
	 * transmission. Wake the tx used_queue to give the transmit
	 * worker a chance to fill back up the TX work fifo.
	 */
	wake_up_interruptible(&chan->free_queue);
	wake_up_interruptible(&chan->used_queue);

	return ret;
}

/**
 * catscp_irq() - handle interrupts
 * @irq:	interrupt
 * @d:		device context for irq
 *
 * @return	irq handled status
 */
static irqreturn_t catscp_irq(int irq, void *d)
{
	u32 irq_stat;
	struct catscp_priv *priv;
	irqreturn_t ret = IRQ_NONE;
	struct device *dev = d;

	priv = dev_get_drvdata(dev);

	irq_stat = catscp_reg_read(priv, SCP_CMN_ITS_REG_OFFSET);

	if (irq_stat & SCP_CMN_ITS__TXH_BUF_COMPLETE_ITS_bm) {
		if (catscp_tx_irq(priv, true) == IRQ_HANDLED)
			ret = IRQ_HANDLED;
	}

	if (irq_stat & SCP_CMN_ITS__RX_BUF_COMPLETE_ITS_bm) {
		if (catscp_rx_irq(priv) == IRQ_HANDLED)
			ret = IRQ_HANDLED;
	}

	if (irq_stat & SCP_CMN_ITS__TXL_BUF_COMPLETE_ITS_bm) {
		if (catscp_tx_irq(priv, false) == IRQ_HANDLED)
			ret = IRQ_HANDLED;
	}

	/* Clear all pending interrupts. */
	catscp_reg_write(priv, SCP_CMN_ITS_BCLR_REG_OFFSET, irq_stat);

	/* Return the logically OR'd result of each interrupt handler. */
	return ret;
}

/**
 * catscp_stop() - Stops the driver.
 *
 * @priv:	catscp private data structure
 *
 * @return	result status
 */
static int catscp_stop(struct catscp_priv *priv)
{
	/* Take down the link. */
	catscp_set_link(priv, false);

	/* Disable hardware interrupts. */
	catscp_reg_write(priv, SCP_CMN_ITS_BCLR_REG_OFFSET, SCP_CLEAR_IRQ_MASK);

	/* Explicitly disable FIFOs. */
	catscp_reg_write(priv, SCP_MEM_RX_FIFO_FLAGS_REG_OFFSET, 0);
	catscp_reg_write(priv, SCP_MEM_TXL_FIFO_FLAGS_REG_OFFSET, 0);
	catscp_reg_write(priv, SCP_MEM_TXH_FIFO_FLAGS_REG_OFFSET, 0);

	/* Disable IRQ handler. */
	if (priv->irq) {
		free_irq(priv->irq, priv->dev);
		priv->irq = 0;
	}

	/* Stop worker threads. */
	if (priv->txh_work_thread) {
		kthread_stop(priv->txh_work_thread);
		priv->txh_work_thread = NULL;
	}

	if (priv->tx_work_thread) {
		kthread_stop(priv->tx_work_thread);
		priv->tx_work_thread = NULL;
	}

	if (priv->rx_work_thread) {
		kthread_stop(priv->rx_work_thread);
		priv->rx_work_thread = NULL;
	}

	return 0;
}

/**
 * catscp_start() - Starts the driver.
 *
 * @priv:	catscp private data structure
 *
 * @return	result status
 */
static int catscp_start(struct catscp_priv *priv)
{
	int rc;

	/* Make sure the link is down and then reset the FIFOs. */
	rc = catscp_reset(priv);
	if (rc) {
		return rc;
	}

	/* Double-make sure the link is down, setup the controller again. */
	catscp_set_link(priv, false);

	catscp_setup(priv);

	/* Then re-start the worker threads. */
	priv->rx_work_thread = kthread_run(catscp_fill_rx_fifo_thread,
					   (void *)priv, "catscp_rx_work");
	if (IS_ERR(priv->rx_work_thread)) {
		return PTR_ERR(priv->rx_work_thread);
	}

	priv->tx_work_thread = kthread_run(catscp_fill_txl_fifo_thread,
					   (void *)priv, "catscp_tx_work");
	if (IS_ERR(priv->tx_work_thread)) {
		rc = PTR_ERR(priv->tx_work_thread);
		goto cleanup_rx_thread;
	}

	priv->txh_work_thread = kthread_run(catscp_fill_txh_fifo_thread,
					   (void *)priv, "catscp_txh_work");
	if (IS_ERR(priv->txh_work_thread)) {
		rc = PTR_ERR(priv->txh_work_thread);
		goto cleanup_tx_thread;
	}

	priv->irq = platform_get_irq(priv->pdev, 0);
	if (priv->irq < 0) {
		dev_err(priv->dev, "No irq specified\n");
		rc = priv->irq;
		priv->irq = 0;
		goto cleanup_threads;
	}

	rc = request_irq(priv->irq, catscp_irq, 0, DRIVER_NAME, priv->dev);
	if (rc) {
		dev_err(priv->dev, "Could not register irq %d\n", priv->irq);
		goto cleanup_threads;
	}

	/* Everything is ready, kick the rx free queue to begin operation. */
	wake_up_interruptible(&priv->rx.free_queue);

	/* Wait for a RX buffer before trying link up. */
	if (wait_event_interruptible(priv->rx.used_queue,
				     kfifo_len(&priv->rx.used_fifo) > 0)) {
		dev_err(priv->dev, "interrupted wait on used_queue\n");
	}

	/* Try to link-up */
	catscp_set_link(priv, true);

	return rc;

cleanup_threads:
	kthread_stop(priv->txh_work_thread);
	priv->txh_work_thread = NULL;
cleanup_tx_thread:
	kthread_stop(priv->tx_work_thread);
	priv->tx_work_thread = NULL;
cleanup_rx_thread:
	kthread_stop(priv->rx_work_thread);
	priv->rx_work_thread = NULL;

	return rc;
}

/*
 * Push-pull character device file operations.
 */
static const struct file_operations catscp_fops = {
	.owner = THIS_MODULE,
	.open = catscp_open,
	.read = catscp_read,
	.write = catscp_write_txl,
	.poll = catscp_poll,

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
 * Pull only character device file operations.
 */
static const struct file_operations catscp_fops_txh = {
	.owner = THIS_MODULE,
	.open = catscp_open_txh,
	.read = NULL,
	.write = catscp_write_txh,
	.poll = catscp_poll_txh,

	/*
	 * From Linux Device Drivers:
	 * "If this function pointer is NULL, seek calls will
	 * modify the position counter in the file structure
	 * (described in the section "The File Structure")
	 * in potentially unpredictable ways.
	 */
	.llseek = noop_llseek,
};

/**
 * catscp_chrdev_init() - performs pre-driver-registration initialization
 */
void __init catscp_chrdev_init(void)
{
	catscp_class = class_create(THIS_MODULE, DRIVER_NAME);
}

/**
 * catscp_chrdev_exit() - cleans up after catscp_chrdev_init
 *
 * Note: we omit the __exit macro here so that this method can be safely
 * called from __init functions for rollback in error cases.
 */
void catscp_chrdev_exit(void) { class_destroy(catscp_class); }

/**
 * Declare all statistics to map to sysfs entries.
 */
#define CATSCP_SHOW_STATS_SYSFS(variable)				       \
	static ssize_t catscp_stats_show_##variable(			       \
	    struct device *dev, struct device_attribute *attr, char *buf)      \
	{								       \
		struct catscp_priv *priv = dev_get_drvdata(dev);	       \
		return scnprintf(buf, PAGE_SIZE, "%u\n",		       \
				 atomic_read(&priv->stats.variable));	       \
	}								       \
	static DEVICE_ATTR(variable, 0444, catscp_stats_show_##variable, NULL)

#define CATSCP_SHOW_MARGIN_SYSFS(variable)				       \
	static ssize_t catscp_margin_show_##variable(			       \
	    struct device *dev, struct device_attribute *attr, char *buf)      \
	{								       \
		struct catscp_priv *priv = dev_get_drvdata(dev);	       \
		return scnprintf(buf, PAGE_SIZE, "%d\n",		       \
				 atomic_read(&priv->stats.variable));	       \
	}								       \
	static DEVICE_ATTR(variable, 0444, catscp_margin_show_##variable, NULL)

#define CATSCP_SHOW_FIFO_DEPTH_SYSFS(fifo, name)			       \
	static ssize_t catscp_fifolen_show_##name(			       \
	    struct device *dev, struct device_attribute *attr, char *buf)      \
	{								       \
		struct catscp_priv *priv = dev_get_drvdata(dev);	       \
		return scnprintf(buf, PAGE_SIZE, "%u\n",		       \
				 kfifo_len(&priv->fifo));		       \
	}								       \
	static DEVICE_ATTR(name, 0444, catscp_fifolen_show_##name, NULL)

CATSCP_SHOW_STATS_SYSFS(sw_read_count);
CATSCP_SHOW_STATS_SYSFS(hw_read_count);
CATSCP_SHOW_STATS_SYSFS(sw_write_count);
CATSCP_SHOW_STATS_SYSFS(hw_write_count);
CATSCP_SHOW_STATS_SYSFS(hw_write_complete_count);
CATSCP_SHOW_STATS_SYSFS(sw_read_drop_count);
CATSCP_SHOW_STATS_SYSFS(sw_write_drop_count);
CATSCP_SHOW_STATS_SYSFS(hw_write_drop_count);
CATSCP_SHOW_STATS_SYSFS(gate_under_min_count);
CATSCP_SHOW_STATS_SYSFS(gate_over_max_count);
CATSCP_SHOW_STATS_SYSFS(gate_low_time_between_count);
CATSCP_SHOW_STATS_SYSFS(gate_unordered_count);
CATSCP_SHOW_MARGIN_SYSFS(gate_min_margin);
CATSCP_SHOW_MARGIN_SYSFS(gate_max_margin);
CATSCP_SHOW_FIFO_DEPTH_SYSFS(rx.free_fifo, fifo_read_free_count);
CATSCP_SHOW_FIFO_DEPTH_SYSFS(rx.used_fifo, fifo_read_used_count);
CATSCP_SHOW_FIFO_DEPTH_SYSFS(rx.complete_fifo, fifo_read_complete_count);
CATSCP_SHOW_FIFO_DEPTH_SYSFS(tx.free_fifo, fifo_write_free_count);
CATSCP_SHOW_FIFO_DEPTH_SYSFS(tx.used_fifo, fifo_write_used_count);
CATSCP_SHOW_FIFO_DEPTH_SYSFS(tx.complete_fifo, fifo_write_complete_count);
CATSCP_SHOW_FIFO_DEPTH_SYSFS(txh.free_fifo, txh_fifo_write_free_count);
CATSCP_SHOW_FIFO_DEPTH_SYSFS(txh.used_fifo, txh_fifo_write_used_count);
CATSCP_SHOW_FIFO_DEPTH_SYSFS(txh.complete_fifo, txh_fifo_write_complete_count);

static struct attribute *catscp_sysfs_stats_attrs[] = {
	&dev_attr_sw_read_count.attr,
	&dev_attr_hw_read_count.attr,
	&dev_attr_sw_write_count.attr,
	&dev_attr_hw_write_count.attr,
	&dev_attr_hw_write_complete_count.attr,
	&dev_attr_sw_read_drop_count.attr,
	&dev_attr_sw_write_drop_count.attr,
	&dev_attr_hw_write_drop_count.attr,
	&dev_attr_gate_under_min_count.attr,
	&dev_attr_gate_over_max_count.attr,
	&dev_attr_gate_low_time_between_count.attr,
	&dev_attr_gate_unordered_count.attr,
	&dev_attr_gate_min_margin.attr,
	&dev_attr_gate_max_margin.attr,
	&dev_attr_fifo_read_free_count.attr,
	&dev_attr_fifo_read_used_count.attr,
	&dev_attr_fifo_read_complete_count.attr,
	&dev_attr_fifo_write_free_count.attr,
	&dev_attr_fifo_write_used_count.attr,
	&dev_attr_fifo_write_complete_count.attr,
	&dev_attr_txh_fifo_write_free_count.attr,
	&dev_attr_txh_fifo_write_used_count.attr,
	&dev_attr_txh_fifo_write_complete_count.attr,
	NULL,
};

static const struct attribute_group catscp_sysfs_stats_group = {
	.name = "stats", .attrs = catscp_sysfs_stats_attrs,
};

/**
 * Declare all registers to map to sysfs entries, along with friendly names.
 */
#define CATSCP_SHOW_REG_SYSFS(offset, name)                               \
	static ssize_t catscp_reg_show_##name(                            \
	    struct device *dev, struct device_attribute *attr, char *buf) \
	{                                                                 \
		int ret;                                                  \
		int reg_value;                                            \
		struct catscp_priv *priv = dev_get_drvdata(dev);          \
									  \
		if (mutex_lock_interruptible(&priv->reg_mutex)) {         \
			dev_err(priv->dev,                                \
				"Caught signal in catscp_reg_show\n");    \
			return -ERESTARTSYS;                              \
		}                                                         \
									  \
		reg_value = ioread32(priv->regs + (offset));		  \
									  \
		mutex_unlock(&priv->reg_mutex);                           \
									  \
		ret = scnprintf(buf, PAGE_SIZE, "0x%08x\n", reg_value);   \
									  \
		return ret;                                               \
	}                                                                 \
	static DEVICE_ATTR(name, 0444, catscp_reg_show_##name, NULL)

CATSCP_SHOW_REG_SYSFS(SCP_CMN_TTL_CNTR_REG_OFFSET, common_ttl_cntr);
CATSCP_SHOW_REG_SYSFS(SCP_CMN_TTL_CAPTURE_REG_OFFSET, common_ttl_capture);
CATSCP_SHOW_REG_SYSFS(SCP_LNK_STATUS_REG_OFFSET, link_stat);
CATSCP_SHOW_REG_SYSFS(SCP_LNK_ERROR_REG_OFFSET, link_error);
CATSCP_SHOW_REG_SYSFS(SCP_LNK_NACK_RCV_REG_OFFSET, link_nack_rcv);
CATSCP_SHOW_REG_SYSFS(SCP_LNK_CONSEC_NACK_RCV_REG_OFFSET, link_consec_nack_rcv);
CATSCP_SHOW_REG_SYSFS(SCP_LNK_MSG_RCV_REG_OFFSET, link_message_rcv);
CATSCP_SHOW_REG_SYSFS(SCP_LNK_MSG_SENT_REG_OFFSET, link_message_sent);
CATSCP_SHOW_REG_SYSFS(SCP_PHY_STATUS_REG_OFFSET, phy_stat);
CATSCP_SHOW_REG_SYSFS(SCP_MEM_RX_FIFO_WR_IDX_REG_OFFSET, rx_write_index);
CATSCP_SHOW_REG_SYSFS(SCP_MEM_RX_FIFO_RD_IDX_REG_OFFSET, rx_read_index);
CATSCP_SHOW_REG_SYSFS(SCP_MEM_TXL_FIFO_WR_IDX_REG_OFFSET, txl_write_index);
CATSCP_SHOW_REG_SYSFS(SCP_MEM_TXL_FIFO_RD_IDX_REG_OFFSET, txl_read_index);
CATSCP_SHOW_REG_SYSFS(SCP_MEM_TXL_FIFO_STATUS_REG_OFFSET, txl_status);
CATSCP_SHOW_REG_SYSFS(SCP_MEM_TXH_FIFO_WR_IDX_REG_OFFSET, txh_write_index);
CATSCP_SHOW_REG_SYSFS(SCP_MEM_TXH_FIFO_RD_IDX_REG_OFFSET, txh_read_index);
CATSCP_SHOW_REG_SYSFS(SCP_MEM_TXH_FIFO_STATUS_REG_OFFSET, txh_status);

static struct attribute *catscp_sysfs_reg_attrs[] = {
	&dev_attr_common_ttl_cntr.attr,
	&dev_attr_common_ttl_capture.attr,
	&dev_attr_link_stat.attr,
	&dev_attr_link_error.attr,
	&dev_attr_link_nack_rcv.attr,
	&dev_attr_link_consec_nack_rcv.attr,
	&dev_attr_link_message_rcv.attr,
	&dev_attr_link_message_sent.attr,
	&dev_attr_phy_stat.attr,
	&dev_attr_rx_write_index.attr,
	&dev_attr_rx_read_index.attr,
	&dev_attr_txl_write_index.attr,
	&dev_attr_txl_read_index.attr,
	&dev_attr_txl_status.attr,
	&dev_attr_txh_write_index.attr,
	&dev_attr_txh_read_index.attr,
	&dev_attr_txh_status.attr,
	NULL,
};

static const struct attribute_group catscp_sysfs_regs_group = {
	.name = "registers", .attrs = catscp_sysfs_reg_attrs,
};

/**
 * catscp_control_store_link_enable() - reset the link and controller
 * when any value is written to the sysfs file.
 *
 * @dev:	device context
 * @attrib:	device attribute
 * @buf:	input buffer
 * @count:	input buffer length
 *
 * @return	result status
 */
static ssize_t catscp_control_store_link_enable(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct catscp_priv *priv = dev_get_drvdata(dev);
	int rc;

	rc = catscp_stop(priv);
	if (rc)
		return rc;

	/* Reset KFIFO state */
	rc = catscp_reset_channel_fifos(priv, &priv->rx);
	if (rc)
		return rc;

	rc = catscp_reset_channel_fifos(priv, &priv->tx);
	if (rc)
		return rc;

	rc = catscp_reset_channel_fifos(priv, &priv->txh);
	if (rc)
		return rc;

	rc = catscp_start(priv);
	if (rc)
		return rc;

	return count;
}

static DEVICE_ATTR(link_enable, 0200, NULL,
		   catscp_control_store_link_enable);

/**
 * catscp_control_show_link_speed() - show the current link speed.
 * @dev:	device context
 * @attrib:	device attribute
 * @buf:	output buffer
 * @count:	output buffer length
 *
 * @return	length of string returned
 */
static ssize_t catscp_control_show_link_speed(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct catscp_priv *priv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", priv->link_speed);
}

/**
 * catscp_control_store_link_speed() - validate and set a new link speed.
 * @dev:	device context
 * @attrib:	device attribute
 * @buf:	input buffer
 * @count:	input buffer length
 *
 * @return	Positive non-zero for success.
 */
static ssize_t catscp_control_store_link_speed(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct catscp_priv *priv = dev_get_drvdata(dev);
	unsigned long new_link_speed = 0;
	int rc = 0;

	rc = kstrtoul(buf, 10, &new_link_speed);
	if (rc != 0)
		return rc;

	if (new_link_speed < LINK_SPEED_MAX) {
		priv->link_speed = new_link_speed;
		return count;
	} else {
		return -EINVAL;
	}
}

/**
 * catscp_control_show_gate_min_ttl() - show the min gate TTL threshold.
 * @dev:	device context
 * @attrib:	device attribute
 * @buf:	output buffer
 * @count:	output buffer length
 *
 * @return	length of string returned
 */
static ssize_t catscp_control_show_gate_min_ttl(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct catscp_priv *priv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", priv->gate.min_ttl);
}

/**
 * catscp_control_store_gate_min_ttl() - set the min gate TTL threshold and
 *                                       reset the min margin stats.
 * @dev:	device context
 * @attrib:	device attribute
 * @buf:	input buffer
 * @count:	input buffer length
 *
 * @return	Positive non-zero for success.
 */
static ssize_t catscp_control_store_gate_min_ttl(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	struct catscp_priv *priv = dev_get_drvdata(dev);
	unsigned long new_ttl = 0;
	int rc = 0;

	rc = kstrtoul(buf, 10, &new_ttl);
	if (rc != 0)
		return rc;

	priv->gate.min_ttl = new_ttl;
	atomic_set(&priv->stats.gate_min_margin, SCP_TTL_INVALID);

	return count;
}

/**
 * catscp_control_show_gate_max_ttl() - show the max gate TTL threshold.
 * @dev:	device context
 * @attrib:	device attribute
 * @buf:	output buffer
 * @count:	output buffer length
 *
 * @return	length of string returned
 */
static ssize_t catscp_control_show_gate_max_ttl(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct catscp_priv *priv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", priv->gate.max_ttl);
}

/**
 * catscp_control_store_gate_max_ttl() - set the max gate TTL threshold and
 *                                       reset the max margin stats.
 * @dev:	device context
 * @attrib:	device attribute
 * @buf:	input buffer
 * @count:	input buffer length
 *
 * @return	Positive non-zero for success.
 */
static ssize_t catscp_control_store_gate_max_ttl(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	struct catscp_priv *priv = dev_get_drvdata(dev);
	unsigned long new_ttl = 0;
	int rc = 0;

	rc = kstrtoul(buf, 10, &new_ttl);
	if (rc != 0)
		return rc;

	priv->gate.max_ttl = new_ttl;
	atomic_set(&priv->stats.gate_max_margin, -SCP_TTL_INVALID);

	return count;
}

/**
 * catscp_control_show_gate_time_between_ttl() - show the min TTL between gates
 *                                               threshold.
 * @dev:	device context
 * @attrib:	device attribute
 * @buf:	output buffer
 * @count:	output buffer length
 *
 * @return	length of string returned
 */
static ssize_t catscp_control_show_gate_time_between_ttl(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct catscp_priv *priv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", priv->gate.time_between_ttl);
}

/**
 * catscp_control_store_gate_time_between_ttl() - set the min TTL between gates
 *                                                threshold and reset the
 *                                                respective counter.
 * @dev:	device context
 * @attrib:	device attribute
 * @buf:	input buffer
 * @count:	input buffer length
 *
 * @return	Positive non-zero for success.
 */
static ssize_t catscp_control_store_gate_time_between_ttl(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	struct catscp_priv *priv = dev_get_drvdata(dev);
	unsigned long new_ttl = 0;
	int rc = 0;

	rc = kstrtoul(buf, 10, &new_ttl);
	if (rc != 0)
		return rc;

	priv->gate.time_between_ttl = new_ttl;
	atomic_set(&priv->stats.gate_low_time_between_count, 0);

	return count;
}

static DEVICE_ATTR(link_speed, 0600,
		   catscp_control_show_link_speed,
		   catscp_control_store_link_speed);

static DEVICE_ATTR(gate_min_ttl, 0600,
		   catscp_control_show_gate_min_ttl,
		   catscp_control_store_gate_min_ttl);

static DEVICE_ATTR(gate_max_ttl, 0600,
		   catscp_control_show_gate_max_ttl,
		   catscp_control_store_gate_max_ttl);

static DEVICE_ATTR(gate_time_between_ttl, 0600,
		   catscp_control_show_gate_time_between_ttl,
		   catscp_control_store_gate_time_between_ttl);

static struct attribute *catscp_sysfs_control_attrs[] = {
	&dev_attr_link_enable.attr,
	&dev_attr_link_speed.attr,
	&dev_attr_gate_min_ttl.attr,
	&dev_attr_gate_max_ttl.attr,
	&dev_attr_gate_time_between_ttl.attr,
	NULL,
};

static const struct attribute_group catscp_sysfs_control_group = {
	.name = "control", .attrs = catscp_sysfs_control_attrs,
};

/**
 * catscp_probe() - allocates and inits memory for scp
 * @dev:	our platform_device
 *
 * @return	Zero on success, non-zero on failure.
 */
static int catscp_probe(struct platform_device *dev)
{
	struct catscp_priv *priv = NULL;
	struct resource *mem;
	int rc = 0;

	BUILD_BUG_ON(sizeof(struct scp_descriptor) != SCP_DESCRIPTOR_SIZE);

	priv = devm_kzalloc(&dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		rc = -ENOMEM;
		goto out;
	}

	/*
	 * Set private data *before* requesting IRQ, as that information would
	 * be needed immediately if an interrupt were to arrive right away.
	 */
	platform_set_drvdata(dev, priv);
	/* and set the back-references as well */
	priv->pdev = dev;
	priv->dev = &dev->dev;
	priv->major = major;

	/* Default SCP gate TTL, set to half the rollover value. */
	priv->gate.min_ttl = 0x0;
	priv->gate.max_ttl = 0x7FFFFF;
	priv->gate.time_between_ttl = 0x0;

	atomic_set(&priv->stats.gate_min_margin, SCP_TTL_INVALID);
	atomic_set(&priv->stats.gate_max_margin, -SCP_TTL_INVALID);

	/* Request reset line control. */
	priv->scp_reset = devm_reset_control_get(priv->dev, "scp-master-rst");
	if (IS_ERR(priv->scp_reset)) {
		dev_err(priv->dev, "scp reset control not defined\n");
		goto out;
	}

	/* Read and validate the hardware descriptor counts for each FIFO. */
	rc = of_property_read_u32(dev->dev.of_node, "rx-desc-count",
				  &priv->rx.hw_desc_count);
	if (rc) {
		dev_err(&dev->dev,
			"Missing required parameter 'rx-desc-count'\n");
		goto out;
	}

	if (priv->rx.hw_desc_count < HW_FIFO_COUNT_MIN ||
	    priv->rx.hw_desc_count > HW_FIFO_COUNT_MAX ||
	    !is_power_of_2(priv->rx.hw_desc_count)) {
		dev_err(&dev->dev, "Invalid 'rx-desc-count' %d\n",
			priv->rx.hw_desc_count);
		goto out;
	}

	rc = of_property_read_u32(dev->dev.of_node, "tx-desc-count",
				  &priv->tx.hw_desc_count);
	if (rc) {
		dev_err(&dev->dev,
			"Missing required parameter 'tx-desc-count'\n");
		goto out;
	}

	if (priv->tx.hw_desc_count < HW_FIFO_COUNT_MIN ||
	    priv->tx.hw_desc_count > HW_FIFO_COUNT_MAX ||
	    !is_power_of_2(priv->tx.hw_desc_count)) {
		dev_err(&dev->dev, "Invalid 'tx-desc-count' %d\n",
			priv->tx.hw_desc_count);
		goto out;
	}
	/* Use same settings for TXH fifo. */
	priv->txh.hw_desc_count = priv->tx.hw_desc_count;

	rc = of_property_read_u32(dev->dev.of_node, "rx-buffer-size",
				  &priv->rx.buffer_size);
	if (rc) {
		dev_err(&dev->dev,
			"Missing required parameter 'rx-buffer-size'\n");
		goto out;
	}

	rc = of_property_read_u32(dev->dev.of_node, "tx-buffer-size",
				  &priv->tx.buffer_size);
	if (rc) {
		dev_err(&dev->dev,
			"Missing required parameter 'tx-buffer-size'\n");
		goto out;
	}
	/* Use same settings for TXH fifo. */
	priv->txh.buffer_size = priv->tx.buffer_size;

	rc = of_property_read_u32(dev->dev.of_node, "rx-buffer-count",
				  &priv->rx.buffer_count);
	if (rc) {
		dev_err(&dev->dev,
			"Missing required parameter 'rx-buffer-count'\n");
		goto out;
	}

	rc = of_property_read_u32(dev->dev.of_node, "tx-buffer-count",
				  &priv->tx.buffer_count);
	if (rc) {
		dev_err(&dev->dev,
			"Missing required parameter 'tx-buffer-count'\n");
		goto out;
	}
	/* Use same settings for TXH fifo. */
	priv->txh.buffer_count = priv->tx.buffer_count;

	rc = of_property_read_u32(dev->dev.of_node, "rx-timeout",
				  &priv->rx_timeout);
	if (rc) {
		dev_err(&dev->dev, "Missing required parameter 'rx-timeout'\n");
		goto out;
	}

	/* Default link is high speed, but allow it to be overwritten. */
	rc = of_property_read_u32(dev->dev.of_node, "link-speed",
				  &priv->link_speed);
	if (rc || priv->link_speed >= LINK_SPEED_MAX) {
		priv->link_speed = LINK_SPEED_600MBIT;
	}

	mem = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (resource_size(mem) < sizeof(SCP_t)) {
		dev_err(&dev->dev, "resource size too small for SCP block\n");
		rc = -ENOMEM;
		goto out;
	}

	priv->regs = ioremap(mem->start, resource_size(mem));
	if (!priv->regs) {
		dev_err(&dev->dev, "ioremap() failed\n");
		rc = -ENOMEM;
		goto out;
	}

	/* Initialize to RX, the first RX FIFO block. */
	rc = catscp_init_channel(dev, &priv->rx, "rx_scp");
	if (rc) {
		dev_err(&dev->dev, "error allocating rx channel\n");
		rc = -ENOMEM;
		goto out_free_mem_regions;
	}

	/* Initialize to TXL, the second "low-priority" TX FIFO block. */
	rc = catscp_init_channel(dev, &priv->tx, "txl_scp");
	if (rc) {
		dev_err(&dev->dev, "error allocating tx channel\n");
		rc = -ENOMEM;
		goto out_free_rx_channel;
	}

	/* Initialize to TXH, the "high-priority" TX FIFO block. */
	rc = catscp_init_channel(dev, &priv->txh, "txh_scp");
	if (rc) {
		dev_err(&dev->dev, "error allocating txh channel\n");
		rc = -ENOMEM;
		goto out_free_tx_channel;
	}

	/* Allocate char device */
	rc = catscp_register_device(priv);
	if (rc) {
		dev_err(&dev->dev, "Error registering char device\n");
		goto out_free_txh_channel;
	}

	/*
	 * Create sysfs debug files.
	 */
	mutex_init(&priv->reg_mutex);
	rc = sysfs_create_group(&dev->dev.kobj, &catscp_sysfs_stats_group);
	if (rc) {
		dev_err(&dev->dev, "Error registering stats sysfs group\n");
		goto out_unregister_device;
	}

	rc = sysfs_create_group(&dev->dev.kobj, &catscp_sysfs_regs_group);
	if (rc) {
		dev_err(&dev->dev, "Error registering regs sysfs group\n");
		goto out_remove_sysfs_stats;
	}

	rc = sysfs_create_group(&dev->dev.kobj, &catscp_sysfs_control_group);
	if (rc) {
		dev_err(&dev->dev, "Error registering control sysfs group\n");
		goto out_remove_sysfs_regs;
	}

	rc = catscp_start(priv);
	if (rc) {
		dev_err(&dev->dev, "Failed to start controller\n");
		goto out_remove_sysfs_groups;
	}

	return 0;

out_remove_sysfs_groups:
	sysfs_remove_group(&dev->dev.kobj, &catscp_sysfs_control_group);
out_remove_sysfs_regs:
	sysfs_remove_group(&dev->dev.kobj, &catscp_sysfs_regs_group);
out_remove_sysfs_stats:
	sysfs_remove_group(&dev->dev.kobj, &catscp_sysfs_stats_group);
out_unregister_device:
	mutex_destroy(&priv->reg_mutex);
	catscp_unregister_devices(priv);
out_free_txh_channel:
	catscp_deinit_channel(dev, &priv->txh);
out_free_tx_channel:
	catscp_deinit_channel(dev, &priv->tx);
out_free_rx_channel:
	catscp_deinit_channel(dev, &priv->rx);
out_free_mem_regions:
	iounmap(priv->regs);
out:
	return rc;
}

/**
 * catscp_remove() - called when kernel wants us to unbind from the device
 * @dev:	the device to unbind from
 *
 * @return	Zero on success, non-zero on failure.
 */
static int catscp_remove(struct platform_device *dev)
{
	struct catscp_priv *priv = NULL;

	priv = platform_get_drvdata(dev);

	catscp_stop(priv);

	/* Unregister sysfs groups. */
	sysfs_remove_group(&dev->dev.kobj, &catscp_sysfs_stats_group);
	sysfs_remove_group(&dev->dev.kobj, &catscp_sysfs_regs_group);
	sysfs_remove_group(&dev->dev.kobj, &catscp_sysfs_control_group);
	mutex_destroy(&priv->reg_mutex);

	/* Unregister devices */
	catscp_unregister_devices(priv);

	/* De-initialize channels */
	catscp_deinit_channel(dev, &priv->rx);
	catscp_deinit_channel(dev, &priv->tx);
	catscp_deinit_channel(dev, &priv->txh);

	/* Unmap memory regions. */
	iounmap(priv->regs);

	return 0;
}

/*
 * Here we list which devices we are compatible with.
 */
static const struct of_device_id catscp_ids[] = {
	{
		.compatible = COMPATIBLE_ID,
	},
	{},
};

/*
 * Now we register the compatibility information with the kernel.
 */
MODULE_DEVICE_TABLE(of, catscp_ids);

/*
 * Here we define the driver itself.
 */
static struct platform_driver catscp_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = catscp_ids,
	},
	.probe = catscp_probe,
	.remove = catscp_remove,
};

/**
 * catscp_init() - initializes the driver
 *
 * @return	Zero on success, non-zero on failure.
 */
static int __init catscp_init(void)
{
	int rc = 0;

	/* Get a major id from the kernel. */
	rc = alloc_chrdev_region(&major, 0, 2, DRIVER_NAME);
	if (rc) {
		goto out;
	}

	/* Initialize the char device portion of the driver. */
	catscp_chrdev_init();

	/* This makes the driver real and eligible for devices to attach. */
	rc = platform_driver_register(&catscp_driver);
	if (rc) {
		goto out_free_chrdev;
	}

	return 0;

out_free_chrdev:
	catscp_chrdev_exit();
	unregister_chrdev_region(major, 1);
out:
	return rc;
}
module_init(catscp_init);

/**
 * catscp_exit() - removes the driver
 */
static void __exit catscp_exit(void)
{
	platform_driver_unregister(&catscp_driver);
	catscp_chrdev_exit();
	unregister_chrdev_region(major, 1);

	major = 0;
}
module_exit(catscp_exit);

MODULE_LICENSE("GPL");
