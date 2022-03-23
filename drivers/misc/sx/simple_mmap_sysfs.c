/**
 * Sysfs interfaces for the simple mmap driver.
 */

#include "simple_mmap.h"

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stat.h>
#include <linux/sysfs.h>

/**
 * region_mmap() - maps a memory region into userspace memory
 *
 * @filp:	our file pointer
 * @kobj:	holds the parent directory
 * @bin_attr:	the sysfs attribute
 * @vma:	the virtual memory area that userspace is trying to map
 *
 * @return zero on success or a negative error code on a failure
 */
static int region_mmap(
		struct file *filp,
		struct kobject *kobj,
		struct bin_attribute *bin_attr,
		struct vm_area_struct *vma)
{
	/* initialized below, after the error check */
	unsigned long offset = 0;
	unsigned long desired_size = 0;
	struct smmap_priv *priv = NULL;
	struct smmap_region *region = NULL;
	unsigned region_num = 0;
	phys_addr_t physaddr = 0;
	unsigned long pageframe = 0;

	if (WARN_ON(!filp || !kobj || !bin_attr || !vma))
		return -EINVAL;

	offset = vma->vm_pgoff << PAGE_SHIFT;
	desired_size = vma->vm_end - vma->vm_start;
	priv = bin_attr->private;
	region = container_of(bin_attr, struct smmap_region, bin_attr);

	if (WARN_ON(!priv))
		return -EINVAL;

	/* Recover memory region number by subtracting the offset of the
	 * region in the array of regions. */
	region_num = region - priv->regions;

	/* extra super paranoia, though we have no reason to believe that the
	 * bin_attr pointer is wild */
	if (region_num >= priv->num_regions) {
		dev_err(priv->dev, "Got a bad bin_attr?! (%p)\n", bin_attr);
		return -EINVAL;
	}
	physaddr = region->remap_start + offset;
	pageframe = physaddr >> PAGE_SHIFT;

	if ((physaddr < region->remap_start) ||
	    (physaddr >= (region->remap_start + region->remap_size))) {
		dev_warn(priv->dev,
			 "mmap() start out of range for '%s' (0x%08llx not "
			 "within 0x%08llx-0x%08llx)\n",
			 bin_attr->attr.name,
			 (unsigned long long)physaddr,
			 (unsigned long long)region->remap_start,
			 (unsigned long long)(region->remap_start +
					      region->remap_size));
		return -EINVAL;
	}
	if ((physaddr + desired_size) >
	    (region->remap_start + region->remap_size)) {
		dev_warn(priv->dev, "mmap() would span beyond end of "
			 "memory region '%s' "
			 "((0x%08llx + 0x%08lx) > (0x%08llx + 0x%08llx))\n",
			 bin_attr->attr.name,
			 (unsigned long long)physaddr, desired_size,
			 (unsigned long long)region->remap_start,
			 (unsigned long long)region->remap_size);
		return -EINVAL;
	}

	if (!priv->is_cached) {
		/* Mark the memory mapping as uncacheable.  If we do not do
		 * this, any accesses will do a cached read-modify-write
		 * operation on the entire cacheline.  We don't want that
		 * for I/O memory.
		 * Note that for kernel virtual memory mappings, ioremap()
		 * does this automatically for us.
		 */
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	}

	dev_dbg(priv->dev, "'%s' mmap(%p, 0x%08lx, 0x%08lx, 0x%08lx, "
		"0x%08llx)\n",
		bin_attr->attr.name, vma,
		vma->vm_start, pageframe, desired_size,
		(u64)pgprot_val(vma->vm_page_prot));
	return io_remap_pfn_range(vma, vma->vm_start, pageframe, desired_size,
				  vma->vm_page_prot);
}

/**
 * remove_mmap_dir() - removes sysfs files for mmap/
 *
 * @priv:	our private device data
 * @num_files:	number of files to remove or SIZE_MAX to remove all files
 */
static void remove_mmap_dir(struct smmap_priv *priv, size_t num_files)
{
	size_t i = 0;

	if (WARN_ON(!priv))
		return;

	for (i = 0; i < num_files && i < priv->num_regions; i++) {
		sysfs_remove_bin_file(priv->mmap_kobj,
				&priv->regions[i].bin_attr);
	}

	kobject_put(priv->mmap_kobj);
	priv->mmap_kobj = NULL;
}

/**
 * create_mmap_dir() - creates sysfs files for mmap/
 *
 * @priv:		our private device data
 * @_total_size:	returns the total number of bytes in the device memory
 *			regions
 *
 * @return zero on success or a negative error code on a failure
 */
static int create_mmap_dir(
		struct smmap_priv *priv,
		size_t *_total_size)
{
	int rc = 0;
	size_t i = 0;
	size_t total_size = 0;

	if (WARN_ON(!priv || !_total_size))
		return -EINVAL;

	priv->mmap_kobj = kobject_create_and_add("mmap", &priv->dev->kobj);

	if (!priv->mmap_kobj) {
		dev_err(priv->dev, "Unable to create 'mmap' directory\n");
		return -ENOMEM;
	}

	for (i = 0; i < priv->num_regions; i++) {
		struct smmap_region *region = &priv->regions[i];
		struct bin_attribute *bin_attr = &region->bin_attr;

		sysfs_bin_attr_init(bin_attr);
		bin_attr->attr.name = region->label;
		bin_attr->attr.mode = S_IRUSR | S_IWUSR;
		bin_attr->private = priv;
		bin_attr->size = region->remap_size;
		bin_attr->mmap = region_mmap;

		rc = sysfs_create_bin_file(priv->mmap_kobj, bin_attr);

		if (rc != 0) {
			dev_err(priv->dev, "Unable to create sysfs bin file "
				"for '%s'\n", bin_attr->attr.name);
			break;
		}

		total_size += bin_attr->size;
	}

	if (rc != 0)
		remove_mmap_dir(priv, i);

	*_total_size = total_size;

	return rc;
}

/**
 * copy_length() - check the length to copy for a read or write call
 *
 * @poss:	the starting offset
 * @count:	the length from the start
 * @max:	the maximum offset + length
 *
 * @return the length to copy or a negative error code on a failure
 */
static ssize_t copy_length(loff_t pos, size_t count, size_t max)
{
	int rc = 0;

	if (pos > max)
		return -EINVAL;

	rc = max - pos;

	if (rc < count)
		return rc;

	return count;
}

/**
 * buf_read() - sysfs interface to read the temporary buffer
 *
 * @filp:	our file pointer
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 * @buf:	the buffer copied to
 * @pos:	the byte offset to start copying from
 * @count:	the number of bytes to copy
 *
 * @return the number of bytes copied on success or a negative error code on a
 * failure
 */
static ssize_t buf_read(
		struct file *filp,
		struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf,
		loff_t pos,
		size_t count)
{
	struct smmap_dma_buf *dma_buf = NULL;
	ssize_t rc = 0;

	if (WARN_ON(!filp || !kobj || !attr || !buf))
		return -EINVAL;

	dma_buf = attr->private;

	if (WARN_ON(!dma_buf))
		return -EINVAL;

	rc = copy_length(pos, count, attr->size);

	if (rc >= 0)
		memcpy(buf, dma_buf->p + pos, rc);

	return rc;
}

/**
 * buf_mmap() - sysfs interface to mmap the temporary buffer
 *
 * @filp:	our file pointer
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 * @vma:	the virtual memory area that userspace is trying to map
 *
 * @return zero on success or a negative error code on a failure
 */
static int buf_mmap(
		struct file *filp,
		struct kobject *kobj,
		struct bin_attribute *bin_attr,
		struct vm_area_struct *vma)
{
	struct smmap_dma_buf *buf = NULL;
	unsigned long size = 0;
	unsigned long pfn = 0;

	if (WARN_ON(!filp || !kobj || !bin_attr || !vma))
		return -EINVAL;

	buf = bin_attr->private;
	size = vma->vm_end - vma->vm_start;

	if (WARN_ON(!buf || buf->p != PTR_ALIGN(buf->p, PAGE_SIZE)
				|| size != ALIGN(size, PAGE_SIZE)))
		return -EINVAL;

	if (size > buf->size || vma->vm_pgoff
			> ((buf->size - size) >> PAGE_SHIFT)) {
		/* Past the end of the buffer. */
		return -EINVAL;
	}

	/* Get the page frame number of the buffer, then add the offset. */
	pfn = (__pa(buf->p) >> PAGE_SHIFT) + vma->vm_pgoff;

	return remap_pfn_range(vma, vma->vm_start, pfn, size,
			vma->vm_page_prot);
}

/**
 * clear_store() - sysfs interface to clear the scatter or gather list of device
 * memory regions.
 *
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 * @buf:	the buffer copied from
 * @count:	the number of bytes to copy
 *
 * @return count on success or a negative error code on a failure
 */
static ssize_t clear_store(
		struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf,
		size_t count)
{
	struct smmap_dma *dma = NULL;
	struct smmap_dma_list list = {};
	int rc = 0;

	if (WARN_ON(!kobj || !attr || !buf))
		return -EINVAL;

	dma = container_of(attr, struct smmap_dma, clear_attr);

	/* Clear by setting an empty list. */
	rc = smmap_dma_set(dma, &list, 0, false);

	if (rc < 0)
		return rc;

	return count;
}

/**
 * list_show() - sysfs interface to get the current scatter or gather list of
 * device memory regions
 *
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 * @buf:	the buffer copied to
 *
 * @return the number of bytes copied on success or a negative error code on a
 * failure
 */
static ssize_t list_show(
		struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	struct smmap_dma *dma = NULL;

	if (WARN_ON(!kobj || !attr || !buf))
		return -EINVAL;

	dma = container_of(attr, struct smmap_dma, list_attr);

	return smmap_dma_print_list(&dma->list, buf, PAGE_SIZE);
}

/**
 * list_store() - sysfs interface to set the scatter or gather list of device
 * memory regions.
 *
 * @priv:	our private device data
 * @dma:	attributes and state for the interface
 * @buf:	the buffer copied from
 * @count:	the number of bytes to copy
 * @sending:	true if the transfer is to the device or false otherwise.
 *
 * @return count on success or a negative error code error code on a failure
 */
static ssize_t list_store(
		struct smmap_priv *priv,
		struct smmap_dma *dma,
		const char *buf,
		size_t count,
		bool sending)
{
	struct smmap_dma_list list = {};
	int rc = 0;

	if (WARN_ON(!priv || !dma || !buf))
		return -EINVAL;

	rc = smmap_dma_parse_list(priv, &list, buf, count);

	if (rc < 0)
		return rc;

	rc = smmap_dma_set(dma, &list, rc, sending);

	if (rc < 0) {
		smmap_dma_free_list(&list);
		return rc;
	}

	return count;
}

/**
 * gather_list_store() - wraps list_store() for the input/gather-list interface
 *
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 * @buf:	the buffer copied from
 * @count:	the number of bytes to copy
 *
 * @return count on success or a negative error code on a failure
 */
static ssize_t gather_list_store(
		struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf,
		size_t count)
{
	struct smmap_priv *priv = NULL;

	if (WARN_ON(!kobj || !attr || !buf))
		return -EINVAL;

	priv = container_of(attr, struct smmap_priv, input.list_attr);

	return list_store(priv, &priv->input, buf, count, false);
}

/**
 * start_show() - placeholder for reads on the update and commit interfaces
 *
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 * @buf:	the buffer copied to
 *
 * @return zero bytes copied
 */
static ssize_t start_show(
		struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	struct smmap_dma *dma = NULL;

	if (WARN_ON(!kobj || !attr || !buf))
		return -EINVAL;

	dma = container_of(attr, struct smmap_dma, start_attr);

	BUILD_BUG_ON(sizeof(dma->transfer_start_time) !=
		     sizeof(struct timespec64));

	memcpy(buf, &dma->transfer_start_time, sizeof(struct timespec64));

	return sizeof(struct timespec64);
}

/**
 * start_store() - initiates a transfer for the input or output interfaces
 *
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 * @buf:	the buffer copied from
 * @count:	the number of bytes to copy
 *
 * @return count on success or a negative error code on a failure
 */
static ssize_t start_store(
		struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf,
		size_t count)
{
	struct smmap_dma *dma = NULL;
	int rc = 0;

	if (WARN_ON(!kobj || !attr || !buf))
		return -EINVAL;

	dma = container_of(attr, struct smmap_dma, start_attr);

	if (count == sizeof(struct timespec64)) {
		/* 'buf' contains a struct timespec */
		rc = smmap_dma_start_delay(dma, (struct timespec64 *)buf);

	} else if (count == 1 && buf[0] == 0) {
		/* start immediately */
		rc = smmap_dma_start(dma);
	} else
		rc = -EINVAL;

	if (rc < 0)
		return rc;

	return count;
}

/**
 * scatter_list_store() - wraps list_store() for the output/scatter-list
 * interface
 *
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 * @buf:	the buffer copied from
 * @count:	the number of bytes to copy
 *
 * @return count on success or a negative error code on a failure
 */
static ssize_t scatter_list_store(
		struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf,
		size_t count)
{
	struct smmap_priv *priv = NULL;

	if (WARN_ON(!kobj || !attr || !buf))
		return -EINVAL;

	priv = container_of(attr, struct smmap_priv, output.list_attr);

	return list_store(priv, &priv->output, buf, count, true);
}

/**
 * remove_buf_attr() - removes the sysfs file for the temporary buffer
 *
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 */
static void remove_buf_attr(
		struct kobject *kobj,
		struct bin_attribute *attr)
{
	if (WARN_ON(!kobj || !attr))
		return;

	sysfs_remove_bin_file(kobj, attr);

	smmap_dma_free_buf(attr->private);
}

/**
 * create_buf_attr() - creates the sysfs file for the temporary buffer
 *
 * @kobj:	holds the parent directory
 * @attr:	the sysfs attribute
 * @dma_buf:	the handle to hold the allocated temporary buffer
 * @bufsz:	the total number of bytes in the device memory regions
 * @writable:	indicates whether or not the buffer is writable
 *
 * @return zero on success or a negative error code on a failure
 */
static int create_buf_attr(
		struct kobject *kobj,
		struct bin_attribute *attr,
		struct smmap_dma_buf *dma_buf,
		size_t bufsz,
		bool writable)
{
	int rc = 0;

	if (WARN_ON(!kobj || !attr || !dma_buf))
		return -EINVAL;

	/* mmap() will require a multiple of page size. */
	bufsz = ALIGN(bufsz, PAGE_SIZE);

	/* Alloc the buffer. */
	rc = smmap_dma_create_buf(dma_buf, bufsz);

	if (rc < 0)
		return rc;

	/* Create the attribute. */
	sysfs_bin_attr_init(attr);
	attr->attr.name = "buf";
	attr->attr.mode = S_IRUSR | (writable ? S_IWUSR : 0);
	attr->private = dma_buf;
	attr->size = bufsz;
	attr->read = buf_read;
	attr->write = NULL;
	attr->mmap = buf_mmap;

	rc = sysfs_create_bin_file(kobj, attr);

	if (rc < 0) {
		smmap_dma_free_buf(dma_buf);
		attr->private = NULL;
	}

	return rc;
}

/**
 * remove_dma_dir() - removes sysfs files for the input and output directories
 *
 * @dma:	attributes and state for the interface
 */
static void remove_dma_dir(struct smmap_dma *dma)
{
	if (WARN_ON(!dma || !dma->kobj))
		return;

	sysfs_put(dma->start_kn);
	dma->start_kn = NULL;

	smmap_dma_destroy(dma);

	remove_buf_attr(dma->kobj, &dma->buf_attr);

	{
		const struct attribute *attrs[] = {
			&dma->clear_attr.attr,
			&dma->list_attr.attr,
			&dma->start_attr.attr,
			NULL
		};

		sysfs_remove_files(dma->kobj, attrs);
	}

	kobject_put(dma->kobj);
	dma->kobj = NULL;
}

/**
 * create_dma_dir() - creates sysfs files for the input and output directories
 *
 * @priv:	our private device data
 * @dma:	attributes and state for the interface
 * @name:	name of the directory
 * @bufsz:	the total number of bytes in the device memory regions
 * @writable:	indicates whether or not the temporary buffer is writable
 *
 * @return zero on success or a negative error code on a failure
 */
static int create_dma_dir(
		struct smmap_priv *priv,
		struct smmap_dma *dma,
		const char *name,
		size_t bufsz,
		bool writable)
{
	struct kobject *kobj = NULL;

	if (WARN_ON(!dma))
		return -EINVAL;

	/* Create the directory. */
	kobj = kobject_create_and_add(name, &priv->dev->kobj);

	if (!kobj) {
		dev_err(priv->dev, "Unable to create '%s' directory\n", name);
		return -ENOMEM;
	}

	/* Init the timer for delayed transfers. */
	smmap_dma_timer_init(dma);

	{
		/* Add the files. */
		const struct attribute *attrs[] = {
			&dma->clear_attr.attr,
			&dma->list_attr.attr,
			&dma->start_attr.attr,
			NULL
		};

		int rc = sysfs_create_files(kobj, attrs);

		if (rc < 0) {
			kobject_put(kobj);

			dev_err(priv->dev, "Unable to create '%s' files", name);
			return rc;
		}

		rc = create_buf_attr(kobj, &dma->buf_attr, &dma->buf, bufsz,
				writable);

		if (rc < 0) {
			sysfs_remove_files(kobj, attrs);
			kobject_put(kobj);

			dev_err(priv->dev, "Unable to create '%s/buf'", name);
			return rc;
		}
	}

	/* This dirent is cached and used to notify when DMA completes. */
	dma->start_kn = sysfs_get_dirent(kobj->sd, dma->start_attr.attr.name);

	dma->kobj = kobj;

	return 0;
}

/**
 * remove_input_dir() - removes sysfs files for input/
 *
 * @priv:	our private device data
 */
static void remove_input_dir(struct smmap_priv *priv)
{
	return remove_dma_dir(&priv->input);
}

/**
 * create_input_dir() - creates sysfs files for input/
 *
 * @priv:	our private device data
 * @bufsz:	the total number of bytes in the device memory regions
 *
 * @return zero on success or a negative error code on a failure
 */
static int create_input_dir(struct smmap_priv *priv, size_t bufsz)
{
	struct kobj_attribute *attr = NULL;

	if (WARN_ON(!priv))
		return -EINVAL;

	attr = &priv->input.clear_attr;
	sysfs_attr_init(&attr->attr);
	attr->attr.name = "clear";
	attr->attr.mode = S_IWUSR;
	attr->show = NULL;
	attr->store = clear_store;

	attr = &priv->input.list_attr;
	sysfs_attr_init(&attr->attr);
	attr->attr.name = "gather-list";
	attr->attr.mode = S_IRUGO | S_IWUSR;
	attr->show = list_show;
	attr->store = gather_list_store;

	attr = &priv->input.start_attr;
	sysfs_attr_init(&attr->attr);
	attr->attr.name = "update";
	attr->attr.mode = S_IRUGO | S_IWUSR;
	attr->show = start_show;
	attr->store = start_store;

	return create_dma_dir(priv, &priv->input, "input", bufsz, false);
}

/**
 * remove_output_dir() - removes sysfs files for output/
 *
 * @priv:	our private device data
 */
static void remove_output_dir(struct smmap_priv *priv)
{
	return remove_dma_dir(&priv->output);
}

/**
 * create_output_dir() - creates sysfs files for output/
 *
 * @priv:	our private device data
 * @bufsz:	the total number of bytes in the device memory regions
 *
 * @return zero on success or a negative error code on a failure
 */
static int create_output_dir(struct smmap_priv *priv, size_t bufsz)
{
	struct kobj_attribute *attr = NULL;

	if (WARN_ON(!priv))
		return -EINVAL;

	attr = &priv->output.clear_attr;
	sysfs_attr_init(&attr->attr);
	attr->attr.name = "clear";
	attr->attr.mode = S_IWUSR;
	attr->show = NULL;
	attr->store = clear_store;

	attr = &priv->output.list_attr;
	sysfs_attr_init(&attr->attr);
	attr->attr.name = "scatter-list";
	attr->attr.mode = S_IRUGO | S_IWUSR;
	attr->show = list_show;
	attr->store = scatter_list_store;

	attr = &priv->output.start_attr;
	sysfs_attr_init(&attr->attr);
	attr->attr.name = "commit";
	attr->attr.mode = S_IRUGO | S_IWUSR;
	attr->show = start_show;
	attr->store = start_store;

	return create_dma_dir(priv, &priv->output, "output", bufsz, true);
}

/**
 * smmap_create_device_files() - creates sysfs files
 *
 * @priv:	our private device data
 *
 * @return zero on success or a negative error code on a failure
 */
int smmap_create_device_files(struct smmap_priv *priv)
{
	int rc = 0;
	size_t bufsz = 0;

	if (WARN_ON(!priv))
		return -EINVAL;

	dev_dbg(priv->dev, "Creating sysfs files\n");

	rc = create_mmap_dir(priv, &bufsz);

	if (rc != 0)
		return rc;

	/* only create the input/output dma files if uncached */
	if (!priv->no_dma) {
		rc = create_input_dir(priv, bufsz);

		if (rc != 0) {
			remove_mmap_dir(priv, SIZE_MAX);
			return rc;
		}

		rc = create_output_dir(priv, bufsz);

		if (rc != 0) {
			remove_input_dir(priv);
			remove_mmap_dir(priv, SIZE_MAX);
			return rc;
		}
	}

	return 0;
}

/**
 * smmap_remove_device_files() - removes sysfs files for the mmaps
 *
 * @priv:	our private device data
 */
void smmap_remove_device_files(struct smmap_priv *priv)
{
	remove_output_dir(priv);
	remove_input_dir(priv);
	remove_mmap_dir(priv, SIZE_MAX);

	dev_dbg(priv->dev, "Removed sysfs files\n");
}
