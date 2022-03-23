/**
 * The main header file for the SpaceX simple mmap driver. It has declarations
 * for everything shared across implementation files.
 */

#include "sx_common.h"

#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/dma-direction.h>
#include <linux/hrtimer.h>

/**
 * struct smmap_region - a device memory region
 *
 * @resource:		the struct resource for this region
 * @regs:		virtual address for this region
 * @label:		short name for the region
 * @name:		unique name for the region (includes host device name)
 * @bin_attr:		struct bin_attribute for the region (sysfs file)
 * @remap_start:	starting address of remapped region
 * @remap_size:		size of remapped region
 */
struct smmap_region {
	struct resource resource;
	__be32 __iomem *regs;
	const char *label;
	char name[SX_NAME_LEN];
	struct bin_attribute bin_attr;
	resource_size_t remap_start;
	resource_size_t remap_size;
};

/**
 * struct smmap_dma_entry - an entry in the device memory list
 *
 * @region:	device memory region, e.g. dac, gpio, etc.
 * @offset:	number of bytes from the start of the device memory region
 * @length:	number of bytes after the offset
 */
struct smmap_dma_entry {
	struct smmap_region *region;
	unsigned long offset;
	unsigned long length;
};

/**
 * struct smmap_dma_list - list of device memory chunks
 *
 * @entry_count:	number of items in entries
 * @entries:		list of device memory regions
 * @sg:			dmaengine list of the device memory regions
 * @nents:		the value returned from dma_map_sg() and passed to
 *			device_prep_dma_sg()
 */
struct smmap_dma_list {
	size_t entry_count;
	struct smmap_dma_entry *entries;
	struct scatterlist *sg;
	int nents;
};

/**
 * struct smmap_dma_buf - a temporary buffer that is mapped into user space and
 * copied to or from device memory through DMA.
 *
 * @size:	size of the buffer
 * @p:		the buffer.
 * @sg:		a single entry dmaengine list for the buffer
 * @nents:	the value returned from dma_map_sg() and passed to
 *		device_prep_dma_sg()
 */
struct smmap_dma_buf {
	size_t size;
	void *p;
	struct scatterlist sg;
	int nents;
};

/**
 * struct smmap_dma - sysfs handles and state variables for the sysfs input or
 * output DMA interfaces
 *
 * @kobj:	holds the sysfs directory
 * @clear_attr:	sysfs interface to clear the scatter or gather list
 * @list_attr:	interface to set and get the scatter or gather list
 * @start_attr:	interface to start a DMA transfer
 * @start_kn:	cached kernfs node for the start attribute
 * @buf_attr:	interface to map the temporary buffer to user space
 * @list:	list of device memory regions
 * @buf:	the temporary buffer that DMA transferrs to or from
 * @chan:	the DMA channel for the most recent transfer
 * @direction:	the direction for the next transfer
 * @timer:	a timer for delayed transfers
 * @flags:	an atomic bitfield for internal state such as "busy"
 */
struct smmap_dma {
	struct kobject *kobj;
	struct kobj_attribute clear_attr;
	struct kobj_attribute list_attr;
	struct kobj_attribute start_attr;
	struct kernfs_node *start_kn;
	struct bin_attribute buf_attr;
	struct smmap_dma_list list;
	struct smmap_dma_buf buf;
	struct dma_chan *chan;
	enum dma_data_direction direction;
	struct hrtimer timer;
	struct timespec64 transfer_start_time;
	unsigned long flags;
};

/**
 * struct smmap_priv - private data structure for this driver
 *
 * @pdev:		back-reference to the owning &struct platform_device
 * @dev:		back-reference to the owning &struct device
 * @num_regions:	number of I/O memory regions (size of @regions array)
 * @regions:		I/O memory regions (dynamically allocated)
 * @mmap_kobj:		kobject to create a directory for the mmapp()able
 *			@regions
 * @input:		state and sysfs attributes for the input/ interfaces
 * @output:		state and sysfs attributes for the output/ interfaces
 * @is_cached:		should the memory regions be mapped cached or not
 */
struct smmap_priv {
	struct platform_device *pdev;
	struct device *dev;
	unsigned num_regions;
	struct smmap_region *regions;
	struct kobject *mmap_kobj;
	struct smmap_dma input;
	struct smmap_dma output;
	bool is_cached;
	bool no_dma;
};

int smmap_create_device_files(struct smmap_priv *priv);
void smmap_remove_device_files(struct smmap_priv *priv);

int smmap_dma_set(
	struct smmap_dma *dma,
	struct smmap_dma_list *list,
	size_t size,
	bool sending);
int smmap_dma_start(struct smmap_dma *dma);
int smmap_dma_start_delay(struct smmap_dma *dma, const struct timespec64 *delay);

void smmap_dma_timer_init(struct smmap_dma *dma);
void smmap_dma_destroy(struct smmap_dma *dma);

ssize_t smmap_dma_parse_list(
		struct smmap_priv *priv,
		struct smmap_dma_list *dma_list,
		const char *buf,
		size_t count);
ssize_t smmap_dma_print_list(
		struct smmap_dma_list *dma_list,
		char *buf,
		size_t count);
void smmap_dma_free_list(struct smmap_dma_list *list);

int smmap_dma_create_buf(struct smmap_dma_buf *buf, size_t size);
void smmap_dma_free_buf(struct smmap_dma_buf *buf);
