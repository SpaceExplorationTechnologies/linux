/*
 * flash driver private data definitions.
 *
 * @author Matthieu Bucchianeri
 * @date 10/23/2015
 */

#ifndef _FLASH_H_
#define _FLASH_H_

#include "flash_chip.h"
#include "sx_irqs.h"
#include "sx_mem_regions.h"
#include "sx_sysfs_dev_vars.h"

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/compiler.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

/**
 * The driver name.
 */
#define FLASH_DRIVER_NAME	"sx-flash"

/*
 * The maximum number of pending commands for all tracks
 * combined. Here we make room for all tracks having a read scheduled
 * simultaneously. Because cancelled requests still use room in the
 * queue, double this number to handle all tracks seeking then reading
 * simultaneously.
 */
#define MAX_COMMANDS			(MAX_TRACK_COUNT * 2)

/*
 * The maximum size of the read buffer in flash pages. This is
 * arbitrarily chosen.
 */
#define MAX_READ_BUFFER_LENGTH		128

/* The default size of the read buffer in flash pages. */
#define DEFAULT_READ_BUFFER_LENGTH	2

struct flash_priv;
struct flash_desc;

#ifdef CONFIG_SX_FPGA_FLASH_CORE_ERRATA_43
/*
 * Enable workaround for the unsupported Track Full condition when bad
 * blocks are present at the end of a track (FPGA-43).
 */
#define ERRATA_43_TRACK_FULL_WITH_BAD_BLOCKS
#endif

/*
 * The error, statistics, and performance counters both available
 * per-track and for the whole device. For performance reasons, there
 * should not be any non-error fastpath counters in here, otherwise,
 * the performance with multiple tracks will dramatically go down.
 */
#define FLASH_AND_TRACK_COUNTERS()				\
	COUNTER_DECLARE(stat_num_command_processed)		\
	COUNTER_DECLARE(stat_num_ecc_uncorrectable_errors)	\
	COUNTER_DECLARE(error_erasing)				\
	COUNTER_DECLARE(error_write_page_timeout)		\
	COUNTER_DECLARE(error_fifo_low_watermark_timeout)	\
	COUNTER_DECLARE(error_fifo_lockup)			\
	COUNTER_DECLARE(error_read_page_timeout)		\
	COUNTER_DECLARE(error_drain_fault)			\
	COUNTER_DECLARE(error_buffer_overflow_fault)		\
	COUNTER_DECLARE(error_fill_write_buffer_timeout)	\
	COUNTER_DECLARE(error_fill_write_buffer_short)		\
	COUNTER_DECLARE(error_flush_writes_hung)		\
	COUNTER_DECLARE(error_cancel_reads_hung)		\
	COUNTER_DECLARE(error_dma_in_failure)			\
	COUNTER_DECLARE(error_dma_in_short)			\
	COUNTER_DECLARE(error_dma_out_failure)			\
	COUNTER_DECLARE(error_dma_out_short)			\
	COUNTER_DECLARE(error_read_timeout)			\
	COUNTER_DECLARE(error_read_page_violation)		\
	COUNTER_DECLARE(error_block_failure_end_of_track)	\
	COUNTER_DECLARE(stat_num_block_failure)			\
	COUNTER_DECLARE(stat_num_block_failure_erasing)		\
	COUNTER_DECLARE(stat_num_block_recovery_attempt)	\
	COUNTER_DECLARE(stat_num_track_full)			\
	COUNTER_DECLARE(stat_num_open_read)			\
	COUNTER_DECLARE(stat_max_readers)			\
	COUNTER_DECLARE(stat_num_open_write)

/* The error and statistics counters available only per-track. */
#define TRACK_COUNTERS()					\
	COUNTER_DECLARE(stat_num_dma_out)			\
	COUNTER_DECLARE(stat_dma_out_bytes)			\
	COUNTER_DECLARE(stat_num_write)				\
	COUNTER_DECLARE(stat_write_bytes)			\
	COUNTER_DECLARE(stat_num_write_workers)			\
	COUNTER_DECLARE(stat_num_fifo_low_watermark)		\
	COUNTER_DECLARE(stat_num_dma_in)			\
	COUNTER_DECLARE(stat_dma_in_bytes)			\
	COUNTER_DECLARE(stat_num_read)				\
	COUNTER_DECLARE(stat_read_bytes)			\
	COUNTER_DECLARE(stat_num_read_page_complete)		\
	COUNTER_DECLARE(stat_num_poll_read)			\
	COUNTER_DECLARE(stat_num_poll_write)			\
	COUNTER_DECLARE(stat_num_write_page_complete)		\
	COUNTER_DECLARE(stat_num_read_peek)			\
	COUNTER_DECLARE(stat_read_peek_bytes)			\
	COUNTER_DECLARE(stat_num_seek_read)			\
	COUNTER_DECLARE(stat_num_command_rejected)		\
	TRACK_PERF_COUNTERS()

/* The performance counters available only per-track. */
#ifdef CONFIG_SX_PERF_CTR
#define TRACK_PERF_COUNTERS()					\
	COUNTER_DECLARE(perf_write)				\
	COUNTER_DECLARE(perf_write_housekeeping)		\
	COUNTER_DECLARE(perf_write_copy_user)			\
	COUNTER_DECLARE(perf_write_drain_buffer)		\
	COUNTER_DECLARE(perf_write_drain_buffer_async)		\
	COUNTER_DECLARE(perf_write_drain_buffer_housekeeping)	\
	COUNTER_DECLARE(perf_write_drain_buffer_wait_fifo)	\
	COUNTER_DECLARE(perf_write_drain_buffer_copy_device)	\
	COUNTER_DECLARE(perf_write_drain_buffer_wait_complete)	\
	COUNTER_DECLARE(perf_write_error)			\
	COUNTER_DECLARE(perf_read)				\
	COUNTER_DECLARE(perf_read_housekeeping)			\
	COUNTER_DECLARE(perf_read_copy_user)			\
	COUNTER_DECLARE(perf_read_wait)				\
	COUNTER_DECLARE(perf_read_process_command_copy_device)	\
	COUNTER_DECLARE(perf_read_process_command_wait_page)	\
	COUNTER_DECLARE(perf_read_error)
#else
#define TRACK_PERF_COUNTERS()
#endif

/*
 * The error and statistics counters available only for the whole
 * device.
 */
#define FLASH_COUNTERS()					\
	COUNTER_DECLARE(stat_num_command_workers)		\
	COUNTER_DECLARE(error_user_bad_partitions)		\
	COUNTER_DECLARE(error_fram_bad_partitions)		\
	COUNTER_DECLARE(error_partitioning)			\
	COUNTER_DECLARE(error_formatting)			\
	COUNTER_DECLARE(error_parity_failure)			\
	COUNTER_DECLARE(stat_current_vmalloc_bytes)		\
	COUNTER_DECLARE(stat_peak_vmalloc_bytes)		\
	FLASH_PERF_COUNTERS()

/* The performance counters available only for the whole device. */
#ifdef CONFIG_SX_PERF_CTR
#define FLASH_PERF_COUNTERS()					\
	COUNTER_DECLARE(perf_command_worker)			\
	COUNTER_DECLARE(perf_command_worker_housekeeping)	\
	COUNTER_DECLARE(perf_command_worker_process)
#else
#define FLASH_PERF_COUNTERS()
#endif

/**
 * struct flash_track - State for a single track.
 *
 * @priv:			Back-reference to the owning &struct flash_priv.
 * @index:			The index of the track.
 * @cdev:			The character device structure.
 * @write_mutex:		The mutex to serialize calls to write().
 * @accept_data:		Whether the track can accept data or not.
 * @write_buffer:		The internal write buffer.
 * @write_buffer_producer:	The internal write buffer write offset in words.
 * @write_buffer_size:		The size of the internal write buffer in words.
 * @write_queue:		The workqueue to process writing asynchronously.
 * @write_worker:		The worker to process writing asynchronously.
 * @drain_mutex:		The mutex to serialize draining of the write
 *				buffer between synchronous calls to write()
 *				and the recovery write worker.
 * @write_buffer_consumer:	The internal write buffer read offset in words.
 * @writer_closing:		Whether the writer is closing or not.
 * @write_block:		The current block being written to. This value
 *				is a cache of get_write_block() and is updated
 *				only when moving to a new block (see
 *				sync_track()).
 * @low_watermark_comp:		The completion to signal low fifo watermark.
 * @write_comp:			The completion to signal write completion.
 * @dma_comp:			The completion to signal DMA ended.
 * @write_recovery:		Whether write recovery should be performed.
 * @written:			Whether some data was written to the FIFO.
 * @page_buffer:		The address of the page buffer (in hardware).
 * @page_buffer_phys:		The physical address of the page buffer.
 * @write_waitqueue:		The waitqueue for waiting on write space.
 * @write_buffer_lock:		The lock protecting the internal write buffer.
 * @write_error:		The error counter, increment after each failure
 *				in the write path, and reset upon success.
 * @write_address:		The current write pointer in flash pages.
 * @bad_block_count:		The number of bad blocks.
 * @block_count:		The number of blocks.
 * @wrap:			Whether the track wraps or not.
 * @full:			Whether the track is full or not.
 * [...]			Statistic counters.
 * @writer:			Whether there is an fd opened for writing.
 * @readers:			The number of fds opened for reading.
 * @block_length:		The number of flash pages in a block.
 * @chip_number:		The chip containing the track.
 * @page_length:		The number of words in a flash page.
 * @word_size:			The size in bytes of a word.
 */
struct flash_track {
	struct flash_priv *priv;
	unsigned int index;

	struct cdev cdev;

	struct mutex write_mutex;

	struct sx_sysfs_int_var accept_data;

	uint32_t *write_buffer;
	unsigned int write_buffer_producer;
	size_t write_buffer_size;

	struct workqueue_struct *write_queue;
	struct work_struct write_worker;

	struct mutex drain_mutex;

	unsigned int write_buffer_consumer;
	bool writer_closing;
	unsigned int write_block;

	struct completion low_watermark_comp;
	struct completion write_comp;

	struct completion dma_comp;

	bool write_recovery;
	bool written;

	__iomem uint32_t *page_buffer;
	resource_size_t page_buffer_phys;

	wait_queue_head_t write_waitqueue;
	rwlock_t write_buffer_lock;

	unsigned int write_error;

	struct sx_sysfs_int_var write_address;
	struct sx_sysfs_int_var bad_block_count;
	struct sx_sysfs_int_var block_count;
	struct sx_sysfs_int_var wrap;
	struct sx_sysfs_int_var full;

#define COUNTER_DECLARE(name)		\
	struct sx_sysfs_ctr_var name;

	TRACK_COUNTERS()
	FLASH_AND_TRACK_COUNTERS()

#undef COUNTER_DECLARE

	bool writer;
	atomic_t readers;

	struct sx_sysfs_int_var block_length;
	struct sx_sysfs_int_var chip_number;
	struct sx_sysfs_int_var page_length;
	struct sx_sysfs_int_var word_size;
};

/**
 * struct flash_command - A read command.
 *
 * @desc:		The track file descriptor to read to.
 * @processing:		Whether the command is currently being processed or not.
 *
 * The command queue only contains descriptors that have scheduled
 * operations. The page address and number of pages are in the
 * descriptor itself.
 */
struct flash_command {
	struct flash_desc *desc;
	bool processing;
};

/**
 * struct flash_command_queue - A queue of commands.
 *
 * @lock:	The lock protecting the queue.
 * @queue:	The queue itself.
 * @head:	The head pointer.
 * @tail:	The tail pointer.
 * @count:	The number of items in the queue.
 */
struct flash_command_queue {
	spinlock_t lock;

	struct flash_command queue[MAX_COMMANDS];
	unsigned int head;
	unsigned int tail;
	unsigned int count;
};

/**
 * struct flash_priv - Private data structure for this driver.
 *
 * @pdev:			Back-reference to the owning
 *				&struct platform_device.
 * @dev:			Back-reference to the owning &struct device.
 * @mem_regions:		The memory-mapped I/O regions.
 * @irqs:			The interrupt descriptors.
 * @tracks:			The tracks data structures.
 * @num_tracks:			The number of tracks supported by the
 *				controller.
 * @dma_write_threshold:	The minimum amount of data to transfer using
 *				DMAs when writing to the device (other
 *				transfers will use memcpy()).
 * @low_cpu:			Whether Low CPU mode is enabled or not.
 * @low_watermark_int_enable:	A bitmask of tracks waiting on low watermark
 *				event.
 * @write_complete_int_enable:	A bit mask of tracks waiting on write
 *				completion.
 * @command_mutex:		The mutex to serialize commands to the device.
 * @command_queue:		The workqueue to process commands
 *				asynchronously.
 * @command_worker:		The worker to process commands asynchronously.
 * @commands:			The queue of commands.
 * @command_comp:		The completion signal for a command.
 * @pending_command:		Whether there is a pending command or not.
 * @command_error:		The error code for a command.
 * @dma_comp:			The completion to signal DMA ended.
 * @command_waitqueue:		The waitqueue for waiting on room in the command
 *				queue.
 * @blocks_per_chip:		The number of blocks per chip.
 * [...]			Statistic counters.
 * @chip_count:			The number of flash chips.
 * @chip_size:			The size in bytes of flash chips.
 * @device_formatted:		Whether the device is formatted or not.
 * @partition_valid:		Whether the partition table is valid or not.
 * @readahead_page_count:	The number of flash pages to read ahead.
 * @track_count:		The number of tracks.
 * @forensic:			Whether forensic mode is enabled or not.
 * @format_mutex:		The mutex to serialize formatting of the device.
 * @initializing_sysfs:		Whether sysfs is initializing or not.
 */
struct flash_priv {
	struct platform_device *pdev;
	struct device *dev;
	struct sx_mem_region mem_regions[NUM_FLASH_MEM_REGIONS];
	struct sx_irq irqs[NUM_FLASH_INTERRUPTS];

	struct flash_track tracks[MAX_TRACK_COUNT];
	unsigned int num_tracks;

	struct sx_sysfs_int_var dma_write_threshold;
	struct sx_sysfs_int_var low_cpu;

	atomic_t low_watermark_int_enable;
	atomic_t write_complete_int_enable;

	struct mutex command_mutex;

	struct workqueue_struct *command_queue;
	struct work_struct command_worker;

	struct flash_command_queue commands;
	struct completion command_comp;
	bool pending_command;
	int command_error;

	struct completion dma_comp;

	wait_queue_head_t command_waitqueue;

#define COUNTER_DECLARE(name)		\
	struct sx_sysfs_ctr_var name;

	FLASH_AND_TRACK_COUNTERS()
	FLASH_COUNTERS()

#undef COUNTER_DECLARE

	unsigned int blocks_per_chip;
	struct sx_sysfs_int_var chip_count;
	struct sx_sysfs_ctr_var chip_size;
	struct sx_sysfs_int_var device_formatted;
	struct sx_sysfs_int_var partition_valid;
	struct sx_sysfs_int_var readahead_page_count;
	struct sx_sysfs_int_var track_count;

	struct sx_sysfs_int_var forensic;

	struct mutex format_mutex;

	bool initializing_sysfs;
};

/**
 * struct flash_desc - State of an opened track (file descriptor).
 *
 * @priv:			Back-reference to the owning &struct flash_priv.
 * @track:			The index of the track.
 * @read_mutex:			The mutex to serialize calls to read().
 * @seeking:			Whether we are seeking in the track or not.
 * @seek_offset:		The absolute offset in bytes to seek to.
 * @read_buffer:		The internal read buffer.
 * @read_buffer_consumer:	The internal read buffer's read offset in bytes.
 * @read_buffer_count:		The number of bytes in the internal read buffer.
 * @read_buffer_size:		The size of the internal read buffer in bytes.
 * @read_buffer_base_address:	The address of the flash page corresponding
 *				to the beginning (0) of the internal read
 *				buffer.
 * @read_buffer_page_pool:	The number of free flash pages waiting to be
 *				queued for read.
 * @read_address:		The address of the next page to read from flash.
 * @read_pending_count:		The number of flash pages to read from flash.
 * @read_done:			Whether the read operation is done. This is
 *				somewhat equivalent to read_pending_count != 0,
 *				but also handles the case of cancellation, where
 *				we clear read_pending_count during a read.
 * @read_waitqueue:		The waitqueue for waiting on data available.
 * @read_buffer_producer:	The internal read buffer write offset in bytes.
 * @read_error:			The error counter, increment after each failure
 *				to read a page, and reset upon success.
 * @buffer_size:		The size of the memory region (including this
 *				&struct flash_desc) allocated for the
 *				descriptor.
 *
 * A pointer to this structure is placed in the private data of
 * @struct file, and can be used to reference the @struct flash_priv
 * and identify the opened track. For file descriptors opened in read
 * mode, this structure also contains the read buffer and bookkeeping
 * data. For file descriptors opened in write mode, the write buffer
 * and bookkeeping data is placed in the @struct flash_track.
 */
struct flash_desc {
	struct flash_priv *priv;
	unsigned int track;

	struct mutex read_mutex;

	bool seeking;
	loff_t seek_offset;

	uint8_t *read_buffer;
	unsigned int read_buffer_consumer;
	atomic_t read_buffer_count;
	size_t read_buffer_size;

	unsigned int read_buffer_base_address;
	atomic_t read_buffer_page_pool;

	unsigned int read_address;
	unsigned int read_pending_count;
	bool read_done;

	wait_queue_head_t read_waitqueue;

	unsigned int read_buffer_producer;

	unsigned int read_error;

	size_t buffer_size;
};

#endif /* _FLASH_H_ */
