/*
 * plxx_manager.h
 *
 * In-kernel interface for accessing the plxx_manager driver command
 *
 * Copyright (C) Exor S.p.a.
 * Written by: Giovanni Pavoni, Exor S.p.a.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef PLXX_MANAGER_H
#define PLXX_MANAGER_H

#include <linux/types.h>
#include <linux/memory.h>
#include <linux/err.h>

/* interface commands */
#define RS422_485_IF_SETFD 1
#define RS422_485_IF_SETHD 2

#endif
