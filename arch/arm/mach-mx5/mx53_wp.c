/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include "mx53_wp.h"

/*!
 * @file mach-mx5/mx53_wp.c
 *
 * @brief This file contains the information about MX53 CPU working points.
 *
 * @ingroup MSL_MX53
 */
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
extern struct dvfs_wp *(*get_dvfs_core_wp)(int *wp);

static int num_cpu_wp;
static struct cpu_wp *cpu_wp_table;

static struct dvfs_wp *dvfs_core_setpoint;
static int num_dvfs_core_setpoint;

/* Place holder for dvfs_core setpoints for AEC parts */
static struct dvfs_wp dvfs_core_setpoint_aec[] = {
				{33, 0, 33, 10, 10, 0x08} }; /*800MHz*/

/*
 * manfred.schlaegl@ginzinger.com
 * added industrial AP
 */
static struct dvfs_wp dvfs_core_setpoint_iec[] = {
			{30, 18, 33, 20, 10, 0x08}, /* 800MHz */
			{25, 8, 33, 20, 10, 0x08}, /* 400MHz */
			{28, 8, 33, 20, 30, 0x08}, /* 400MHZ, 133MHz */
			{29, 0, 33, 20, 10, 0x08},}; /* 400MHZ, 50MHz. */

/* Place holder for dvfs_core setpoints for 1.2GHz parts */
static struct dvfs_wp dvfs_core_setpoint_ces_1_2G[] = {
			{33, 25, 33, 10, 10, 0x08}, /*1_2GHz*/
			{30, 18, 33, 20, 10, 0x08}, /* 800MHz */
			{25, 8, 33, 20, 10, 0x08}, /* 400MHz */
			{28, 8, 33, 20, 30, 0x08}, /* 400MHZ, 133MHz */
			{29, 0, 33, 20, 10, 0x08},}; /* 400MHZ, 50MHz. */

/* Place holder for dvfs_core setpoints for 1 GHz parts */
static struct dvfs_wp dvfs_core_setpoint_ces[] = {
			{33, 25, 33, 10, 10, 0x08}, /*1GHz*/
			{30, 18, 33, 20, 10, 0x08}, /* 800MHz */
			{25, 8, 33, 20, 10, 0x08}, /* 400MHz */
			{28, 8, 33, 20, 30, 0x08}, /* 400MHz, 133MHz */
			{29, 0, 33, 20, 10, 0x08},}; /* 400MHz, 50MHz. */

/* working point for auto*/
static struct cpu_wp cpu_wp_aec[] = {
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
};

/*
 * manfred.schlaegl@ginzinger.com
 * added industrial AP
 * working point for industrial 
 */
static struct cpu_wp cpu_wp_iec[] = {
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
	 {
	  .pll_rate = 800000000,
	  .cpu_rate = 400000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 1,
	 .cpu_voltage = 950000,},
};

/* working point for consumer 1G*/
static struct cpu_wp cpu_wp_ces[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
/*
 * manfred.schlaegl@ginzinger.com
 * CEC 1.2V-1.3V nominal 1.25V
 * LTC delivers maximum of 1.219
 */
//	 .cpu_voltage = 1250000,},	/* original */
	 .cpu_voltage = 1219000,},	/* ginzinger */
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
	 {
	  .pll_rate = 800000000,
	  .cpu_rate = 400000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 1,
	 .cpu_voltage = 950000,},
};

/* working point for consumer 1.2G*/
static struct cpu_wp cpu_wp_ces_1_2g[] = {
	{
	 .pll_rate = 1200000000,
	 .cpu_rate = 1200000000,
	 .pdf = 0,
	 .mfi = 12,
	 .mfd = 1,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1300000,},
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1250000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
	 {
	  .pll_rate = 800000000,
	  .cpu_rate = 400000000,
	  .cpu_podf = 1,
	  .cpu_voltage = 950000,},
};

static struct dvfs_wp *mx53_get_dvfs_core_table(int *wp)
{
	/* Add 2 to num_cpu_wp to handle LPAPM mode transitions. */
	*wp = num_dvfs_core_setpoint;
	return dvfs_core_setpoint;
}

struct cpu_wp *mx53_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_table;
}

void mx53_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

void mx53_set_cpu_part_number(enum mx53_cpu_part_number part_num)
{
	get_cpu_wp = mx53_get_cpu_wp;
	set_num_cpu_wp = mx53_set_num_cpu_wp;
	get_dvfs_core_wp = mx53_get_dvfs_core_table;

	switch (part_num) {
	case IMX53_CEC_1_2G:
		cpu_wp_table = cpu_wp_ces_1_2g;
		num_cpu_wp = ARRAY_SIZE(cpu_wp_ces_1_2g);
		dvfs_core_setpoint = dvfs_core_setpoint_ces_1_2G;
		num_dvfs_core_setpoint =
				ARRAY_SIZE(dvfs_core_setpoint_ces_1_2G);
		break;
	case IMX53_CEC:
		cpu_wp_table = cpu_wp_ces;
		num_cpu_wp = ARRAY_SIZE(cpu_wp_ces);
		dvfs_core_setpoint = dvfs_core_setpoint_ces;
		num_dvfs_core_setpoint =
				ARRAY_SIZE(dvfs_core_setpoint_ces);
		break;
/*
 * manfred.schlaegl@ginzinger.com
 * added industrial AP
 * working point for industrial 
 */
	case IMX53_IEC:
		cpu_wp_table = cpu_wp_iec;
		num_cpu_wp = ARRAY_SIZE(cpu_wp_iec);
		dvfs_core_setpoint = dvfs_core_setpoint_iec;
		num_dvfs_core_setpoint =
				ARRAY_SIZE(dvfs_core_setpoint_iec);
		break;
	case IMX53_AEC:
	default:
		cpu_wp_table = cpu_wp_aec;
		num_cpu_wp = ARRAY_SIZE(cpu_wp_aec);
		dvfs_core_setpoint = dvfs_core_setpoint_aec;
		num_dvfs_core_setpoint =
				ARRAY_SIZE(dvfs_core_setpoint_aec);
		break;
	}
}


