/**
 * Configure and initiate DMA transfers for the simple mmap driver.
 *
 * This uses the dmaengine API and requires that you call dmaengine_get()
 * in your initialization (and you should call dmaengine_put() in your
 * teardown).
 */

#include "simple_mmap.h"

#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>

#define REG_ALIGN 4
#define BUSY_BIT 0

/**
 * dma_set() - clear the current scatter/gather list.
 *
 * @dma:	sysfs attributes and state
 */
static void dma_unset(struct smmap_dma *dma)
{
	const struct smmap_dma_list empty_list = {};

	if (dma->chan) {
		if (!WARN_ON((int)dma->list.entry_count <= 0 ||
			     (dma->direction != DMA_TO_DEVICE &&
			      dma->direction != DMA_FROM_DEVICE))) {

			dma_unmap_sg(dma->chan->device->dev, &dma->buf.sg, 1,
					dma->direction);

			/* See the comment about DMA_ATTR_SKIP_CPU_SYNC
			 * near the dma_map_sg_attrs() call. */
			dma_unmap_sg_attrs(dma->chan->device->dev, dma->list.sg,
					   dma->list.entry_count,
					   dma->direction,
					   DMA_ATTR_SKIP_CPU_SYNC);
		}
	} /* else no list mapped yet */

	/* Destroy the old list and save an empty one. */
	smmap_dma_free_list(&dma->list);

	dma->list = empty_list;
	dma->buf.nents = 0;
	dma->direction = DMA_NONE;
	dma->chan = NULL;
}

/**
 * dma_set() - set the scatter/gather list to use in subsequent
 * transfers.
 *
 * @dma:	sysfs attributes and state
 * @list:	a new device memory list, as created by smmap_dma_parse_list()
 * @size:	the total number of bytes in the transfer, as returned by
 *		smmap_dma_parse_list()
 * @sending:	true if the transfer is to the device or false otherwise.
 *
 * @return zero on success or a negative error code on a failure.
 */
static int dma_set(
	struct smmap_dma *dma,
	struct smmap_dma_list *list,
	size_t size,
	bool sending)
{
	struct dma_chan *chan = NULL;
	const enum dma_data_direction direction =
		sending ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	/* A temporary scatterlist so dma->buf.sg isn't changed in case of an
	 * error. */
	struct scatterlist buf_sg = {};
	int buf_nents = 0;

	if (WARN_ON(!dma->buf.size || !dma->buf.p ||
		    (int)list->entry_count <= 0))
		return -EINVAL;

	if (size > dma->buf.size)
		return -EFBIG;

	chan = dma_find_channel(DMA_SG);

	if (WARN_ON(!chan || !chan->device))
		return -ENXIO;

	/*
	 * Set the buffer's list to just the size needed for the transfer.
	 */
	sg_init_table(&buf_sg, 1);
	sg_set_buf(&buf_sg, dma->buf.p, size);

	/* Map for DMA. */
	buf_nents = dma_map_sg(chan->device->dev, &buf_sg, 1, direction);

	if (buf_nents <= 0) {
		dev_info(chan->device->dev, "failed to map buffer %s list",
				sending ? "scatter" : "gather");
		return -EIO;
	}

	/* On ARM64, the dma_map_sg_attrs() operation always performs
	 * a cache invalidation or cache flushing of the virtually
	 * mapped memory regions pointed by the scatter-gather
	 * list. But we don't use these virtual addresses, and they
	 * are not mapped in the kernel address space where
	 * expected. The DMA_ATTR_SKIP_CPU_SYNC attribute instructs
	 * the operation to skip the cache maintenance. */
	list->nents = dma_map_sg_attrs(chan->device->dev, list->sg,
				       list->entry_count, direction,
				       DMA_ATTR_SKIP_CPU_SYNC);

	if (list->nents <= 0) {
		dma_unmap_sg(chan->device->dev, &buf_sg, 1, direction);

		dev_info(chan->device->dev, "failed to map device %s list",
				sending ? "scatter" : "gather");
		return -EIO;
	}

	/* Unmap and free the old list. */
	dma_unset(dma);

	/* Save the new list. */
	dma->chan = chan;
	dma->direction = direction;
	dma->buf.sg = buf_sg;
	dma->buf.nents = buf_nents;
	dma->list = *list;

	return 0;
}

/**
 * smmap_dma_set() - set the scatter/gather list to use in subsequent
 * transfers. On success, any prior list in @dma is freed and the @list is
 * saved, so the caller is no longer responsible for freeing @list. On failure,
 * @dma is not changed and the caller remains responsible for freeing @list
 * (with smmap_dma_free_list()).
 *
 * @dma:	sysfs attributes and state
 * @list:	a new device memory list, as created by smmap_dma_parse_list()
 * @size:	the total number of bytes in the transfer, as returned by
 *		smmap_dma_parse_list()
 * @sending:	true if the transfer is to the device or false otherwise.
 *
 * @return zero on success or a negative error code on a failure
 */
int smmap_dma_set(
	struct smmap_dma *dma,
	struct smmap_dma_list *list,
	size_t size,
	bool sending)
{
	int rc = 0;

	if (WARN_ON(!dma || !list))
		return -EINVAL;

	if (test_and_set_bit(BUSY_BIT, &dma->flags)) {
		/* Don't touch anything during a transfer. */
		return -EBUSY;
	}

	mb();

	if (size == 0) {
		/* Unmap and free the old list. */
		dma_unset(dma);

		/* Clear the entire buffer when the list is cleared. */
		memset(dma->buf.p, 0, dma->buf.size);
	} else {
		/* Save and map the new list, this also unmaps and frees the
		 * old list. */
		rc = dma_set(dma, list, size, sending);
	}

	mb();

	/* Ready to go. */
	clear_bit(BUSY_BIT, &dma->flags);

	return rc;
}

/**
 * smmap_dma_complete() - dmaengine transfer complete callback
 *
 * @p:	our private device data
 */
static void dma_complete(void *p)
{
	struct smmap_dma *dma = p;

	if (WARN_ON(!dma || !dma->chan))
		return;

	/*
	 * Save the completion time.
	 */
	ktime_get_raw_ts64(&dma->transfer_start_time);

	if (dma->direction == DMA_FROM_DEVICE) {
		/*
		 * Ensure that the CPU sees the newly transfered data.
		 */
		dma_sync_sg_for_cpu(dma->chan->device->dev, &dma->buf.sg, 1,
				dma->direction);
	}

	/*
	 * Clear the busy bit to allow new transfers.
	 */
	WARN_ON(!test_and_clear_bit(BUSY_BIT, &dma->flags));

	/*
	 * Indicate to any waiting applications that the transfer is complete.
	 */
	if (!WARN_ON(!dma->start_kn))
		kernfs_notify_poll(dma->start_kn);
}

/**
 * dma_start() - start a dmaengine transfer. This is meant to be called only
 * from smmap_dma_start(), which wraps this with the busy bit test and set.
 *
 * @dma:	sysfs attributes and state
 *
 * @return zero on success or a negative error code on a failure
 */
static int dma_start(struct smmap_dma *dma)
{
	dma_cookie_t cookie = 0;
	struct dma_async_tx_descriptor *dma_desc = NULL;

	struct scatterlist *dst_sg = NULL;
	struct scatterlist *src_sg = NULL;
	int dst_nents = 0;
	int src_nents = 0;

	if (!dma->chan || WARN_ON(dma->buf.nents <= 0 ||
			dma->list.nents <= 0 || !dma->list.entry_count))
		return -EINVAL;

	if (dma->direction == DMA_FROM_DEVICE) {
		dst_sg = &dma->buf.sg;
		dst_nents = dma->buf.nents;

		src_sg = dma->list.sg;
		src_nents = dma->list.nents;

	} else if (WARN_ON(dma->direction != DMA_TO_DEVICE)) {
		/* Must be either to or from */
		return -EINVAL;

	} else /* dma->direction == DMA_TO_DEVICE */ {
		dst_sg = dma->list.sg;
		dst_nents = dma->list.nents;

		src_sg = &dma->buf.sg;
		src_nents = dma->buf.nents;

		dma_sync_sg_for_device(dma->chan->device->dev, &dma->buf.sg, 1,
				dma->direction);
	}

	/* Configure the transfer. */
	dma_desc = dma->chan->device->device_prep_dma_sg(dma->chan,
				dst_sg, dst_nents, src_sg, src_nents,
				DMA_CTRL_ACK);

	if (WARN_ON(!dma_desc))
		return -EIO;

	dma_desc->callback = dma_complete;
	dma_desc->callback_param = dma;

	/* Queue it. */
	cookie = dmaengine_submit(dma_desc);

	if (WARN_ON(dma_submit_error(cookie))) {
		return -EIO;
	}

	/* Flush to hardware. */
	dma_async_issue_pending(dma->chan);

	/* The callback handles cleanup. */
	return 0;
}

/**
 * smmap_dma_start() - start a dmaengine transfer. Set up the transfer with
 * smmap_dma_parse_list() and smmap_dma_create_buf() prior to calling this.
 *
 * @dma:	sysfs attributes and state
 *
 * @return zero on success or a negative error code on a failure
 */
int smmap_dma_start(struct smmap_dma *dma)
{
	int rc = 0;

	if (WARN_ON(!dma))
		return -EINVAL;

	if (test_and_set_bit(BUSY_BIT, &dma->flags)) {
		/* Only one transfer at a time. */
		return -EBUSY;
	}

	mb();

	rc = dma_start(dma);

	if (rc < 0) {
		/* Leave an invalid start time on an error. */
		dma->transfer_start_time.tv_sec = 0;
		dma->transfer_start_time.tv_nsec = -1;

		/* Clear busy if not started. */
		clear_bit(BUSY_BIT, &dma->flags);
	}

	return rc;
}

/**
 * smmap_dma_start_delay() - start a transfer after some delay
 *
 * @dma:	sysfs attributes and state
 * @delay:	the amount of time to wait before starting the transfer
 *
 * @return zero on success or a negative error code on a failure
 */
int smmap_dma_start_delay(struct smmap_dma *dma, const struct timespec64 *delay)
{
	if (!dma->timer.function)
		return -ENXIO;

	if (delay->tv_sec != 0 || !timespec64_valid(delay)) {
		/* Don't allow delays > 1 second. */
		return -EINVAL;
	}

	hrtimer_start(&dma->timer, timespec64_to_ktime(*delay),
			HRTIMER_MODE_REL);
	return 0;
}

/**
 * dma_start_callback() - an hrtimer callback to start a DMA transfer
 *
 * @timer:	the hrtimer
 *
 * @return HRTIMER_NORESTART
 */
static enum hrtimer_restart dma_start_callback(struct hrtimer *timer)
{
	struct smmap_dma *dma = container_of(timer, struct smmap_dma, timer);

	/*
	 * There's nowhere to return an error, but any error will be noted with
	 * an invalid 'start_transfer_time'.
	 */
	smmap_dma_start(dma);

	return HRTIMER_NORESTART;
}

/**
 * Initialize the hrtimer struct.
 *
 * @dma:	sysfs attributes and state
 */
void smmap_dma_timer_init(struct smmap_dma *dma)
{
	hrtimer_init(&dma->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	dma->timer.function = dma_start_callback;

	dma->transfer_start_time.tv_sec = 0;
	dma->transfer_start_time.tv_nsec = -1;
}

/**
 * Cancel and prevent any transfers.
 *
 * @dma:	sysfs attributes and state
 */
void smmap_dma_destroy(struct smmap_dma *dma)
{
	hrtimer_cancel(&dma->timer);

	dma->timer.function = NULL;

	/* Set the busy bit to prevent new transfers. */
	if (test_and_set_bit(BUSY_BIT, &dma->flags)) {
		/* Cancel any current transfer. */
		if (!WARN_ON(!dma->chan))
			dmaengine_terminate_all(dma->chan);
	}

	mb();

	/* Unmap and free the current list. */
	dma_unset(dma);
}

/**
 * next_word() - get the next word on the line, skipping any leading whitespace
 * except a newline character
 *
 * @cur:	the starting point
 * @end:	the stopping point
 * @word:	returns the start of the word
 *
 * @return the first character after the word on success or null on a failure.
 * If no word is found, the return == @word. If @end is reached before the end
 * of a word, the return == @end.
 */
static const char *next_word(
		const char *cur,
		const char *const end,
		const char **word)
{
	if (WARN_ON(!cur || !end || !word))
		return NULL;

	/* Seek to the beginning of the next word or the end of the line. */
	while (cur < end && *cur != '\n' && isspace(*cur))
		cur++;

	*word = cur;

	/* Seek to the end of the word. */
	while (cur < end && !isspace(*cur))
		cur++;

	return cur;
}

/**
 * next_number() - wraps next_word() and parses the word as a decimal integer
 *
 * @cur:	the starting point
 * @end:	the stopping point
 * @number:	returns the number
 *
 * @return the first character after the number on success or null on a failure
 */
static const char *next_number(
		const char *cur,
		const char *const end,
		unsigned long *number)
{
	const char *word = cur;
	char *tmp = NULL;

	/*
	 * Call next_word() to check if there are any non-space characters
	 * before the newline, otherwise simple_strtoul() could skip past a
	 * newline character and start parsing the next line.
	 */
	cur = next_word(cur, end, &word);

	if (!cur || cur == word) {
		/* Empty. */
		return NULL;
	}

	*number = simple_strtoul(word, &tmp, 10);

	if (tmp != cur) {
		/* An invalid number. */
		return NULL;
	}

	return cur;
}

/**
 * find_region() - returns the parameters for the specified region
 *
 * @priv:	our private device data
 * @name:	the name of the region. does not need to be null terminated.
 * @name_len:	the number of bytes in name.
 *
 * @return the region on success or null on a failure
 */
struct smmap_region *find_region(
		const struct smmap_priv *priv,
		const char *name,
		size_t name_len)
{
	unsigned i = 0;

	if (WARN_ON(!priv || !name))
		return NULL;

	for (i = 0; i < priv->num_regions; i++) {
		struct smmap_region *region = &priv->regions[i];

		/* Must compare the same and be the same length. */
		if (!strncmp(region->label, name, name_len)
			&& region->label[name_len] == '\0')
			return region;
	}

	return NULL;
}

/**
 * next_line() - parse a scatter/gather list entry
 *
 * @priv:	our private device data
 * @cur:	the starting point
 * @end:	the stopping point
 * @entry:	returns the parsed list entry
 *
 * @return the first character after the new line on success or null on a
 * failure
 */
static const char *next_line(
		const struct smmap_priv *priv,
		const char *cur,
		const char *const end,
		struct smmap_dma_entry *entry)
{
	const char *word = NULL;

	if (WARN_ON(!cur || !end || !entry))
		return NULL;

	/* Parse the region name. */
	cur = next_word(cur, end, &word);

	if (!cur || cur == word) {
		dev_info(priv->dev, "empty line\n");
		return NULL;
	}

	entry->region = find_region(priv, word, cur - word);

	if (!entry->region) {
		dev_info(priv->dev, "invalid region\n");
		return NULL;
	}

	/* Parse the offset. */
	cur = next_number(cur, end, &entry->offset);

	if (!cur) {
		dev_info(priv->dev, "missing or invalid offset\n");
		return NULL;
	}

	/* Parse the length. */
	cur = next_number(cur, end, &entry->length);

	if (!cur) {
		dev_info(priv->dev, "missing or invalid length\n");
		return NULL;
	}

	/* Seek to the end of the line. */
	cur = next_word(cur, end, &word);

	if (!cur || cur != word) {
		dev_info(priv->dev, "extra parameters\n");
		return NULL;
	}

	if (WARN_ON(cur >= end || *cur != '\n')) {
		/* There should be a newline before the end. */
		return NULL;
	}

	/* return the beginning of the next line */
	return cur + 1;
}

/**
 * invalid_entry() - check that the parameters for an entry in the device memory
 * list are valid
 *
 * @entry: the parameters to check
 *
 * @return true if the entry is invalid or false if not
 */
static bool invalid_entry(struct smmap_dma_entry *entry)
{
	return /* Empty. */
		entry->length == 0

		/* Overflow. */
		|| entry->offset + entry->length < entry->offset

		/* Past the end of the memory region. */
		|| entry->offset + entry->length
			> resource_size(&entry->region->resource)

		/* Unaligned register access. */
		|| entry->offset % REG_ALIGN != 0
		|| entry->length % REG_ALIGN != 0;
}

/**
 * set_entry() - copy parameters for a scatterlist entry
 *
 * @sg:		parameters are copied to this
 * @entry:	parameters are copied from this
 */
static void set_entry(struct scatterlist *sg, struct smmap_dma_entry *entry)
{
	resource_size_t start = entry->region->resource.start + entry->offset;

	sg_set_page(sg, pfn_to_page(start >> PAGE_SHIFT), entry->length,
			offset_in_page(start));
}

/**
 * smmap_dma_parse_list() - parse the scatter/gather list text format
 *
 * @priv:	our private device data
 * @dma_list:	a pointer to a valid struct that returns an allocated list of
 *		device memory chunks parsed from the buffer. The list should
 *		either be handed off with smmap_dma_set() or freed with
 *		smmap_dma_free_list().
 * @buf:	the buffer to parse
 * @count:	the number of bytes in the buffer
 *
 * @return the total number of bytes to transfer on success or a negative error
 * code on a failure
 */
ssize_t smmap_dma_parse_list(
		struct smmap_priv *priv,
		struct smmap_dma_list *dma_list,
		const char *buf,
		size_t count)
{
	const char *const end = buf + count;
	const char *cur = NULL;
	const char *next = NULL;
	size_t entry_count = 0;
	size_t i = 0;
	int total_size = 0;
	struct smmap_dma_entry *entries = NULL;
	struct scatterlist *sg = NULL;

	if (WARN_ON(!priv || !dma_list || !buf))
		return -EINVAL;

	if (end == buf) {
		dev_info(priv->dev, "empty write\n");
		return -EINVAL;
	}

	/* Every entry, including the last, must end with a newline. */
	if (*(end - 1) != '\n') {
		dev_info(priv->dev, "no trailing newline\n");
		return -EINVAL;
	}

	/* One pass to determine the number of entries by counting newlines. */
	for (cur = buf; cur < end; cur++) {
		if (*cur == '\0') {
			dev_info(priv->dev, "null characters\n");
			return -EINVAL;

		} else if (*cur == '\n')
			++entry_count;
	}

	/* A second pass to build the scatterlist. */
	entries = kzalloc(entry_count * sizeof(*entries), GFP_KERNEL);
	sg = kzalloc(entry_count * sizeof(*sg), GFP_KERNEL);

	if (!entries || !sg) {
		kfree(entries);
		kfree(sg);

		dev_info(priv->dev, "failed to allocate for the scatter/gather"
				" list\n");
		return -ENOMEM;
	}

	sg_init_table(sg, entry_count);

	for (cur = buf; cur < end && !WARN_ON(i >= entry_count); cur = next) {
		struct smmap_dma_entry *entry = &entries[i];

		next = next_line(priv, cur, end, entry);

		if (!next) {
			/* Break early to return an error. */
			break;
		}

		if (invalid_entry(entry)) {
			dev_info(priv->dev, "invalid parameters: %s %lu %lu\n",
					entry->region->label, entry->offset,
					entry->length);
			break;
		}

		/* Add the entry. */
		set_entry(&sg[i], entry);

		total_size += entry->length;

		if (total_size <= 0) {
			/* Overflow. */
			dev_info(priv->dev, "parsed scatter/gather list size is"
					" too large");
			break;
		}

		i++;
	}

	if (cur != end || WARN_ON(i < entry_count)) {
		/* Failed to parse the full list. */
		kfree(entries);
		kfree(sg);

		return -EINVAL;
	}

	/* The list is valid. Populate the return value. */
	dma_list->entry_count = entry_count;
	dma_list->entries = entries;
	dma_list->sg = sg;

	return total_size;
}

/**
 * smmap_dma_print_list() - print a scatter/gather list to return
 *
 * @dma_list:	a list of device memory chunks
 * @buf:	the buffer to copy to
 * @count:	the number of bytes avaiable in the buffer
 *
 * @return the total number of bytes copied or a negative error code on a
 * failure
 */
ssize_t smmap_dma_print_list(
		struct smmap_dma_list *dma_list,
		char *buf,
		size_t count)
{
	const char *const end = buf + count;
	char *cur = buf;
	size_t i = 0;

	if (WARN_ON(!dma_list || !buf))
		return -EINVAL;

	if (WARN_ON(dma_list->entry_count > 0 && !dma_list->entries))
		return -EINVAL;

	for (i = 0; i < dma_list->entry_count && !WARN_ON(cur >= end); i++) {
		struct smmap_dma_entry *entry = &dma_list->entries[i];

		cur += scnprintf(cur, end - cur, "%s %lu %lu\n",
				entry->region->label, entry->offset,
				entry->length);
	}

	return cur - buf;
}

/**
 * smmap_dma_free_list() - free memory allocated for a scatter/gather list
 *
 * @list:	holds the list to free
 */
void smmap_dma_free_list(struct smmap_dma_list *list)
{
	if (WARN_ON(!list))
		return;

	list->entry_count = 0;

	kfree(list->entries);
	list->entries = NULL;

	kfree(list->sg);
	list->sg = NULL;
}

/**
 * smmap_dma_create_buf() - allocate a temporary buffer
 *
 * @buf:	a pointer to a valid struct that returns the allocated buffer.
 *		The buffer should be freed with smmap_dma_free_buf() when no
 *		longer needed.
 * @size:	the number of bytes to allocate
 *
 * @return zero on success or a negative error code on a failure
 */
int smmap_dma_create_buf(struct smmap_dma_buf *buf, size_t size)
{
	void *p = NULL;

	if (WARN_ON(!buf))
		return -EINVAL;

	/* Contiguous memory that can be mapped to user space. */
	p = kzalloc(size, GFP_KERNEL | GFP_DMA);

	if (!p)
		return -ENOMEM;

	buf->size = size;
	buf->p = p;

	return 0;
}

/**
 * smmap_dma_free_buf() - free a temporary buffer
 *
 * @buf:	holds the buffer to free
 */
void smmap_dma_free_buf(struct smmap_dma_buf *buf)
{
	if (WARN_ON(!buf))
		return;

	buf->size = 0;

	kfree(buf->p);
	buf->p = NULL;
}

