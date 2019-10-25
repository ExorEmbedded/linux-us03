/*
 * xethm:vs.h
 *
 * In-kernel interface for accessing the plxx_manager driver command
 *
 * Copyright (C) Exor S.p.a.
 * Written by: Exor S.p.a.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef XETHM_VS_H
#define XETHM_VS_H

#include <linux/types.h>
#include <linux/memory.h>
#include <linux/err.h>

/* ioctl commands */
#define XETHM_VS_IOCTL_ADD_NIC			1100
#define XETHM_VS_IOCTL_REMOVE_NIC		1101

/* structure for ioctls */
struct nic_definition
{
	u8 master_instance;
	u8 slave_pos;
};

#endif
