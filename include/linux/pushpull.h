#ifndef LINUX_PUSHPULL_KERNEL_H
#define LINUX_PUSHPULL_KERNEL_H

#include <linux/ioctl.h>

/*
 * Application type for the push/pull.
 */
#define PP_APP_TYPE 'k'

/*
 * IOCTL to set security_tunnel_valid. Only useful for L3UL push/pull.
 */
#define L3UL_PULL_SET_TUNNEL_VALID	_IOW(PP_APP_TYPE, 1, uint8_t)

/*
 * IOCTL to set security_tunnel_index. Only useful for L3UL push/pull.
 */
#define L3UL_PULL_SET_TUNNEL_INDEX	_IOW(PP_APP_TYPE, 2, uint8_t)

/*
 * IOCTL to set class of service. Only useful for L3UL push/pull.
 */
#define L3UL_PULL_SET_COS			_IOW(PP_APP_TYPE, 3, uint8_t)

#endif /* LINUX_PUSHPULL_KERNEL_H */
