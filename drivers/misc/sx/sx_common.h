/*
 * This file contains common definitions for SpaceX driver code.
 */

#ifndef SX_COMMON_H
#define SX_COMMON_H

/*
 * This is the number of bytes we want to use for a unique description field.
 *
 * Such a field might be used to uniquely identify the device in /proc/iomem
 *  or /proc/interrupts, for example.
 */
#define SX_NAME_LEN	32

#ifndef SIZE_MAX
/*
 * The maximum value of a size_t. Defined in linux/kernel.h in later versions.
 */
#define SIZE_MAX        (~(size_t)0)
#endif

/*
 * SpaceX's PCI Vendor ID.
 */
#define PCI_VENDOR_SPACEX	0x1bb8

#endif /* SX_COMMON_H */
