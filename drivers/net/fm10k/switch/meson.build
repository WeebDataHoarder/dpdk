# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2017 Intel Corporation

sources = [
	'fm10k_i2c.c',
	'fm10k_sbus.c',
	'fm10k_ext_port.c',
	'fm10k_serdes.c',
	'fm10k_sm.c',
	'fm10k_spico_code.c',
	'fm10k_config.c',
	'fm10k_ffu.c',
	'fm10k_stats.c',
	'fm10k_flow.c',
	'fm10k_switch.c'
]

error_cflags = ['-Wno-unused-parameter', '-Wno-unused-value',
	'-Wno-strict-aliasing', '-Wno-format-extra-args',
	'-Wno-unused-variable',
	'-Wno-implicit-fallthrough'
]
c_args = cflags
foreach flag: error_cflags
	if cc.has_argument(flag)
		c_args += flag
	endif
endforeach

switch_lib = static_library('fm10k_switch', sources,
	dependencies: [static_rte_eal, static_rte_bus_pci, static_rte_hash, static_rte_ethdev],
	c_args: c_args)
switch_objs = switch_lib.extract_all_objects()
