/* spdx-license-identifier: bsd-3-clause
 * copyright 2019   silicom ltd. connectivity solutions
 */

#include <rte_time.h>
#include <rte_kvargs.h>
#include <rte_hash.h>
#include <rte_flow.h>
#include <rte_flow_driver.h>
#include <rte_tm_driver.h>

#include "../base/fm10k_type.h"
#include "../base/fm10k_osdep.h"
#include "../fm10k.h"
#include "../fm10k_logs.h"
#include "fm10k_debug.h"
#include "fm10k_regs.h"
#include "fm10k_ext_port.h"
#include "fm10k_stats.h"
#include "fm10k_switch.h"
#include "fm10k_config.h"
#include "fm10k_ffu.h"

#define FM10K_MAX_VLAN_COUNTER			63
#define FM10K_NB_RX_STATS_BANKS			6
#define FM10K_BINS_PER_RX_STATS_BANK		16
#define FM10K_WORDS_PER_RX_STATS_COUNTER	4

/* Max expected entries in read stats scatter gather list */
#define MAX_STATS_SGLIST 128
#define FM10K_RX_STATS_BASE			(0xE00000)
#define FM10K_MOD_BASE				(0xE80000)
#define FM10K_CM_APPLY_BASE			(0xD40000)
#define FM10K_EPL_BASE				(0x0E0000)

#define FM10K_RX_STATS_BANK(index1, index0, word)		\
			((0x001000) * ((index1) - 0) + \
			(0x000004) * ((index0) - 0) + \
			(word) + (0x000000) + \
			(FM10K_RX_STATS_BASE))
#define FM10K_MOD_STATS_BANK_FRAME(index1, index0, word)	\
			((0x000800) * ((index1) - 0) + \
			(0x000002) * ((index0) - 0) + \
			(word) + (0x025000) + \
			(FM10K_MOD_BASE))
#define FM10K_MOD_STATS_BANK_BYTE(index1, index0, word)		\
			((0x000800) * ((index1) - 0) + \
			(0x000002) * ((index0) - 0) + \
			(word) + (0x026000) + \
			(FM10K_MOD_BASE))
#define FM10K_CM_APPLY_DROP_COUNT(index, word)			\
			((0x000002) * ((index) - 0) + \
			(word) + (0x000880) + \
			(FM10K_CM_APPLY_BASE))
#define FM10K_MAC_OVERSIZE_COUNTER(index1, index0)		\
			((0x000400) * ((index1) - 0) + \
			(0x000080) * ((index0) - 0) + \
			(0x000021) + \
			(FM10K_EPL_BASE))
#define FM10K_MAC_JABBER_COUNTER(index1, index0)		\
			((0x000400) * ((index1) - 0) + \
			(0x000080) * ((index0) - 0) + \
			(0x000022) + \
			(FM10K_EPL_BASE))
#define FM10K_MAC_UNDERSIZE_COUNTER(index1, index0)		\
			((0x000400) * ((index1) - 0) + \
			(0x000080) * ((index0) - 0) + \
			(0x000023) + \
			(FM10K_EPL_BASE))
#define FM10K_MAC_RUNT_COUNTER(index1, index0)			\
			((0x000400) * ((index1) - 0) + \
			(0x000080) * ((index0) - 0) + \
			(0x000024) + \
			(FM10K_EPL_BASE))
#define FM10K_MAC_OVERRUN_COUNTER(index1, index0)		\
			((0x000400) * ((index1) - 0) + \
			(0x000080) * ((index0) - 0) + \
			(0x000025) + \
			(FM10K_EPL_BASE))
#define FM10K_MAC_UNDERRUN_COUNTER(index1, index0)		\
			((0x000400) * ((index1) - 0) + \
			(0x000080) * ((index0) - 0) + \
			(0x000026) + \
			(FM10K_EPL_BASE))
#define FM10K_MAC_CODE_ERROR_COUNTER(index1, index0)		\
			((0x000400) * ((index1) - 0) + \
			(0x000080) * ((index0) - 0) + \
			(0x000027) + \
			(FM10K_EPL_BASE))

/* Rx Bank Definitions */
#define FM10K_RX_STAT_BANK_TYPE                       0
#define FM10K_RX_STAT_BANK_SIZE                       1
#define FM10K_RX_STAT_BANK_PRI                        2
#define FM10K_RX_STAT_BANK_FWD_1                      3
#define FM10K_RX_STAT_BANK_FWD_2                      4
#define FM10K_RX_STAT_BANK_VLAN                       5

/* FM10K_RX_STAT_BANK_TYPE Bin Definitions */
#define FM10K_RX_STAT_NONIP_L2UCAST                   0
#define FM10K_RX_STAT_NONIP_L2MCAST                   1
#define FM10K_RX_STAT_NONIP_L2BCAST                   2
#define FM10K_RX_STAT_IPV4_L2UCAST                    3
#define FM10K_RX_STAT_IPV4_L2MCAST                    4
#define FM10K_RX_STAT_IPV4_L2BCAST                    5
#define FM10K_RX_STAT_IPV6_L2UCAST                    6
#define FM10K_RX_STAT_IPV6_L2MCAST                    7
#define FM10K_RX_STAT_IPV6_L2BCAST                    8
#define FM10K_RX_STAT_IEEE802_3_PAUSE                 9
#define FM10K_RX_STAT_CLASS_BASED_PAUSE               10
#define FM10K_RX_STAT_FRAMING_ERR                     11
#define FM10K_RX_STAT_FCS_ERR                         12

/* FM10K_RX_STAT_BANK_SIZE Bin Definitions */
#define FM10K_RX_STAT_LEN_LT_64                       0
#define FM10K_RX_STAT_LEN_EQ_64                       1
#define FM10K_RX_STAT_LEN_65_127                      2
#define FM10K_RX_STAT_LEN_128_255                     3
#define FM10K_RX_STAT_LEN_256_511                     4
#define FM10K_RX_STAT_LEN_512_1023                    5
#define FM10K_RX_STAT_LEN_1024_1522                   6
#define FM10K_RX_STAT_LEN_1523_2047                   7
#define FM10K_RX_STAT_LEN_2048_4095                   8
#define FM10K_RX_STAT_LEN_4096_8191                   9
#define FM10K_RX_STAT_LEN_8192_10239                  10
#define FM10K_RX_STAT_LEN_GE_10240                    11

/* FM10K_RX_STAT_BANK_PRI Bin Definitions */
#define FM10K_RX_STAT_PRI_0                           0
#define FM10K_RX_STAT_PRI_1                           1
#define FM10K_RX_STAT_PRI_2                           2
#define FM10K_RX_STAT_PRI_3                           3
#define FM10K_RX_STAT_PRI_4                           4
#define FM10K_RX_STAT_PRI_5                           5
#define FM10K_RX_STAT_PRI_6                           6
#define FM10K_RX_STAT_PRI_7                           7
#define FM10K_RX_STAT_PRI_8                           8
#define FM10K_RX_STAT_PRI_9                           9
#define FM10K_RX_STAT_PRI_10                          10
#define FM10K_RX_STAT_PRI_11                          11
#define FM10K_RX_STAT_PRI_12                          12
#define FM10K_RX_STAT_PRI_13                          13
#define FM10K_RX_STAT_PRI_14                          14
#define FM10K_RX_STAT_PRI_15                          15

/* FM10K_RX_STAT_BANK_FWD_1 Bin Definitions */
#define FM10K_RX_STAT_FID_FORWARDED                   0
#define FM10K_RX_STAT_FLOOD_FORWARDED                 1
#define FM10K_RX_STAT_SPECIALLY_HANDLED               2
#define FM10K_RX_STAT_PARSER_ERROR_DROP               3
#define FM10K_RX_STAT_ECC_ERROR_DROP                  4
#define FM10K_RX_STAT_TRAPPED                         5
#define FM10K_RX_STAT_PAUSE_DROPS                     6
#define FM10K_RX_STAT_STP_DROPS                       7
#define FM10K_RX_STAT_SECURITY_VIOLATIONS             8
#define FM10K_RX_STAT_VLAN_TAG_DROPS                  9
#define FM10K_RX_STAT_VLAN_INGRESS_DROPS              10
#define FM10K_RX_STAT_VLAN_EGRESS_DROPS               11
#define FM10K_RX_STAT_GLORT_MISS_DROPS                12
#define FM10K_RX_STAT_FFU_DROPS                       13
#define FM10K_RX_STAT_TRIGGER_DROPS                   14
#define FM10K_RX_STAT_OVERFLOW4                       15

/* FM10K_RX_STAT_BANK_FWD_2 Bin Definitions */
#define FM10K_RX_STAT_POLICER_DROPS                   0
#define FM10K_RX_STAT_TTL_DROPS                       1
#define FM10K_RX_STAT_CM_PRIV_DROPS                   2
#define FM10K_RX_STAT_CM_SMP0_DROPS                   3
#define FM10K_RX_STAT_CM_SMP1_DROPS                   4
#define FM10K_RX_STAT_CM_RX_HOG_0_DROPS               5
#define FM10K_RX_STAT_CM_RX_HOG_1_DROPS               6
#define FM10K_RX_STAT_CM_TX_HOG_0_DROPS               7
#define FM10K_RX_STAT_CM_TX_HOG_1_DROPS               8
/* Bin 9 reserved */
#define FM10K_RX_STAT_TRIGGER_REDIRECTS               10
#define FM10K_RX_STAT_FLOOD_CONTROL_DROPS             11
#define FM10K_RX_STAT_GLORT_FORWARDED                 12
#define FM10K_RX_STAT_LOOPBACK_SUPPRESS               13
#define FM10K_RX_STAT_OTHERS                          14
#define FM10K_RX_STAT_OVERFLOW5                       15

/* FM10K_RX_STAT_BANK_VLAN Bin definitions */
#define FM10K_RX_STAT_VLAN_UCAST                      0
#define FM10K_RX_STAT_VLAN_MCAST                      1
#define FM10K_RX_STAT_VLAN_BCAST                      2

/* Tx Bank Definitions */
#define FM10K_TX_STAT_BANK_TYPE                       0
#define FM10K_TX_STAT_BANK_SIZE                       1

/* FM10K_TX_STAT_BANK_TYPE Bin Definitions */
#define FM10K_TX_STAT_L2UCAST                         0
#define FM10K_TX_STAT_L2MCAST                         1
#define FM10K_TX_STAT_L2BCAST                         2
#define FM10K_TX_STAT_ERR_SENT                        3
#define FM10K_TX_STAT_TIMEOUT_DROP                    4
#define FM10K_TX_STAT_ERR_DROP                        5
#define FM10K_TX_STAT_ECC_DROP                        6
#define FM10K_TX_STAT_LOOPBACK_DROP                   7
#define FM10K_TX_STAT_TTL1_DROP                       8
#define FM10K_TX_STAT_IEEE802_3_PAUSE                 9
#define FM10K_TX_STAT_CLASS_BASED_PAUSE               10

/* FM10K_TX_STAT_BANK_SIZE Bin Definitions */
#define FM10K_TX_STAT_LEN_LT_64                       0
#define FM10K_TX_STAT_LEN_EQ_64                       1
#define FM10K_TX_STAT_LEN_65_127                      2
#define FM10K_TX_STAT_LEN_128_255                     3
#define FM10K_TX_STAT_LEN_256_511                     4
#define FM10K_TX_STAT_LEN_512_1023                    5
#define FM10K_TX_STAT_LEN_1024_1522                   6
#define FM10K_TX_STAT_LEN_1523_2047                   7
#define FM10K_TX_STAT_LEN_2048_4095                   8
#define FM10K_TX_STAT_LEN_4096_8191                   9
#define FM10K_TX_STAT_LEN_8192_10239                  10
#define FM10K_TX_STAT_LEN_GE_10240                    11

/* This structure is used to build a table of all rx / tx counters */
struct fm10k_port_count_mapping {
	/* Bank in which the counter is located */
	uint32_t bank;

	/* Bin  in which the counter is located */
	uint32_t bin;

	/*
	 * Offset (in bytes) of a the frame counter in
	 * the fm10k_counters structure
	 */
	uint32_t frame_offset;

	/*
	 * Offset (in bytes) of a the byte counter in
	 * the fm10k_counters structure
	 */
	uint32_t byte_offset;
};

/*
 * Table of all RX port counters in the RX banks
 * This table is used for retrieving counters as
 * well as resetting them
 */
static struct fm10k_port_count_mapping rx_port_cnt_map_table[] = {
    /* Bank                        Bin
     * frame_offset
     * byte_offset
     */
    /* Type Bank */
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_NONIP_L2UCAST,
	offsetof(struct fm10k_port_counters, cnt_rx_ucst_pkts_nonip),
	offsetof(struct fm10k_port_counters, cnt_rx_ucst_octets_nonip)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_NONIP_L2MCAST,
	offsetof(struct fm10k_port_counters, cnt_rx_mcst_pkts_nonip),
	offsetof(struct fm10k_port_counters, cnt_rx_mcst_octets_nonip)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_NONIP_L2BCAST,
	offsetof(struct fm10k_port_counters, cnt_rx_bcst_pkts_nonip),
	offsetof(struct fm10k_port_counters, cnt_rx_bcst_octets_nonip)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_IPV4_L2UCAST,
	offsetof(struct fm10k_port_counters, cnt_rx_ucst_pkts_ipv4),
	offsetof(struct fm10k_port_counters, cnt_rx_ucst_octets_ipv4)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_IPV4_L2MCAST,
	offsetof(struct fm10k_port_counters, cnt_rx_mcst_pkts_ipv4),
	offsetof(struct fm10k_port_counters, cnt_rx_mcst_octets_ipv4)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_IPV4_L2BCAST,
	offsetof(struct fm10k_port_counters, cnt_rx_bcst_pkts_ipv4),
	offsetof(struct fm10k_port_counters, cnt_rx_bcst_octets_ipv4)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_IPV6_L2UCAST,
	offsetof(struct fm10k_port_counters, cnt_rx_ucst_pkts_ipv6),
	offsetof(struct fm10k_port_counters, cnt_rx_ucst_octets_ipv6)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_IPV6_L2MCAST,
	offsetof(struct fm10k_port_counters, cnt_rx_mcst_pkts_ipv6),
	offsetof(struct fm10k_port_counters, cnt_rx_mcst_octets_ipv6)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_IPV6_L2BCAST,
	offsetof(struct fm10k_port_counters, cnt_rx_bcst_pkts_ipv6),
	offsetof(struct fm10k_port_counters, cnt_rx_bcst_octets_ipv6)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_IEEE802_3_PAUSE,
	offsetof(struct fm10k_port_counters, cnt_rx_pause_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_pause_octets)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_CLASS_BASED_PAUSE,
	offsetof(struct fm10k_port_counters, cnt_rx_cbpause_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_cbpause_octets)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_FRAMING_ERR,
	offsetof(struct fm10k_port_counters, cnt_rx_framing_error_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_framing_error_octets)},
{	FM10K_RX_STAT_BANK_TYPE,  FM10K_RX_STAT_FCS_ERR,
	offsetof(struct fm10k_port_counters, cnt_rx_fcs_errors),
	offsetof(struct fm10k_port_counters, cnt_rx_fcs_errors_octets)},

    /* Size Bank */
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_LT_64,
	offsetof(struct fm10k_port_counters, cnt_rx_minto63_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_minto63_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_EQ_64,
	offsetof(struct fm10k_port_counters, cnt_rx_64_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_64_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_65_127,
	offsetof(struct fm10k_port_counters, cnt_rx_65to127_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_65to127_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_128_255,
	offsetof(struct fm10k_port_counters, cnt_rx_128to255_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_128to255_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_256_511,
	offsetof(struct fm10k_port_counters, cnt_rx_256to511_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_256to511_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_512_1023,
	offsetof(struct fm10k_port_counters, cnt_rx_512to1023_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_512to1023_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_1024_1522,
	offsetof(struct fm10k_port_counters, cnt_rx_1024to1522_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_1024to1522_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_1523_2047,
	offsetof(struct fm10k_port_counters, cnt_rx_1523to2047_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_1523to2047_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_2048_4095,
	offsetof(struct fm10k_port_counters, cnt_rx_2048to4095_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_2048to4095_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_4096_8191,
	offsetof(struct fm10k_port_counters, cnt_rx_4096to8191_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_4096to8191_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_8192_10239,
	offsetof(struct fm10k_port_counters, cnt_rx_8192to10239_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_8192to10239_octets)},
{	FM10K_RX_STAT_BANK_SIZE,  FM10K_RX_STAT_LEN_GE_10240,
	offsetof(struct fm10k_port_counters, cnt_rx_10240tomax_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_10240tomax_octets)},

    /* priority Bank */
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_0,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[0]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[0])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_1,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[1]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[1])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_2,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[2]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[2])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_3,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[3]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[3])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_4,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[4]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[4])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_5,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[5]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[5])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_6,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[6]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[6])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_7,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[7]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[7])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_8,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[8]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[8])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_9,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[9]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[9])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_10,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[10]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[10])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_11,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[11]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[11])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_12,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[12]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[12])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_13,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[13]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[13])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_14,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[14]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[14])},
{	FM10K_RX_STAT_BANK_PRI,   FM10K_RX_STAT_PRI_15,
	offsetof(struct fm10k_port_counters, cnt_rx_priority_pkts[15]),
	offsetof(struct fm10k_port_counters, cnt_rx_priority_octets[15])},

    /* Forwarding Bank */
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_FID_FORWARDED,
	offsetof(struct fm10k_port_counters, cnt_fid_forwarded_pkts),
	offsetof(struct fm10k_port_counters, cnt_fid_forwarded_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_FLOOD_FORWARDED,
	offsetof(struct fm10k_port_counters, cnt_flood_forwarded_pkts),
	offsetof(struct fm10k_port_counters, cnt_flood_forwarded_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_SPECIALLY_HANDLED,
	offsetof(struct fm10k_port_counters, cnt_specially_handled_pkts),
	offsetof(struct fm10k_port_counters, cnt_specially_handled_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_PARSER_ERROR_DROP,
	offsetof(struct fm10k_port_counters, cnt_parse_err_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_parse_err_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_ECC_ERROR_DROP,
	offsetof(struct fm10k_port_counters, cnt_parity_error_pkts),
	offsetof(struct fm10k_port_counters, cnt_parity_error_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_TRAPPED,
	offsetof(struct fm10k_port_counters, cnt_trapped_pkts),
	offsetof(struct fm10k_port_counters, cnt_trapped_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_PAUSE_DROPS,
	offsetof(struct fm10k_port_counters, cnt_pause_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_pause_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_STP_DROPS,
	offsetof(struct fm10k_port_counters, cnt_stp_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_stp_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_SECURITY_VIOLATIONS,
	offsetof(struct fm10k_port_counters, cnt_security_violation_pkts),
	offsetof(struct fm10k_port_counters, cnt_security_violation_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_VLAN_TAG_DROPS,
	offsetof(struct fm10k_port_counters, cnt_vlan_tag_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_vlan_tag_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_VLAN_INGRESS_DROPS,
	offsetof(struct fm10k_port_counters, cnt_vlan_ingressbv_pkts),
	offsetof(struct fm10k_port_counters, cnt_vlan_ingressbv_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_VLAN_EGRESS_DROPS,
	offsetof(struct fm10k_port_counters, cnt_vlan_egressbv_pkts),
	offsetof(struct fm10k_port_counters, cnt_vlan_egressbv_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_GLORT_MISS_DROPS,
	offsetof(struct fm10k_port_counters, cnt_glort_miss_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_glort_miss_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_FFU_DROPS,
	offsetof(struct fm10k_port_counters, cnt_ffu_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_ffu_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_1, FM10K_RX_STAT_TRIGGER_DROPS,
	offsetof(struct fm10k_port_counters, cnt_trigger_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_trigger_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_POLICER_DROPS,
	offsetof(struct fm10k_port_counters, cnt_policer_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_policer_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_TTL_DROPS,
	offsetof(struct fm10k_port_counters, cnt_ttl_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_ttl_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_CM_PRIV_DROPS,
	offsetof(struct fm10k_port_counters, cnt_cmpriv_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_cmpriv_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_CM_SMP0_DROPS,
	offsetof(struct fm10k_port_counters, cnt_smp0_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_smp0_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_CM_SMP1_DROPS,
	offsetof(struct fm10k_port_counters, cnt_smp1_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_smp1_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_CM_RX_HOG_0_DROPS,
	offsetof(struct fm10k_port_counters, cnt_rx_hog0_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_hog0_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_CM_RX_HOG_1_DROPS,
	offsetof(struct fm10k_port_counters, cnt_rx_hog1_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_rx_hog1_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_CM_TX_HOG_0_DROPS,
	offsetof(struct fm10k_port_counters, cnt_tx_hog0_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_hog0_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_CM_TX_HOG_1_DROPS,
	offsetof(struct fm10k_port_counters, cnt_tx_hog1_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_hog1_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_TRIGGER_REDIRECTS,
	offsetof(struct fm10k_port_counters, cnt_trigger_redir_pkts),
	offsetof(struct fm10k_port_counters, cnt_trigger_redir_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_FLOOD_CONTROL_DROPS,
	offsetof(struct fm10k_port_counters, cnt_flood_control_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_flood_control_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_GLORT_FORWARDED,
	offsetof(struct fm10k_port_counters, cnt_glort_forwarded_pkts),
	offsetof(struct fm10k_port_counters, cnt_glort_forwarded_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_LOOPBACK_SUPPRESS,
	offsetof(struct fm10k_port_counters, cnt_loopback_drops_pkts),
	offsetof(struct fm10k_port_counters, cnt_loopback_drop_octets)},
{	FM10K_RX_STAT_BANK_FWD_2, FM10K_RX_STAT_OTHERS,
	offsetof(struct fm10k_port_counters, cnt_other_pkts),
	offsetof(struct fm10k_port_counters, cnt_other_octets)},

}; /* end rxPortCntMapTable */


/*
 * Table of all TX port counters in the TX banks
 * This table is used for retrieving counters as
 * well as resetting them
 */
static struct fm10k_port_count_mapping tx_port_cnt_map_table[] = {
    /* Bank                        Bin
     * frame_offset
     * byte_offset
     */
    /* Type Bank */
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_L2UCAST,
	offsetof(struct fm10k_port_counters, cnt_tx_ucst_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_ucst_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_L2MCAST,
	offsetof(struct fm10k_port_counters, cnt_tx_mcst_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_mcst_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_L2BCAST,
	offsetof(struct fm10k_port_counters, cnt_tx_bcst_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_bcst_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_ERR_SENT,
	offsetof(struct fm10k_port_counters, cnt_tx_error_sent_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_error_sent_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_TIMEOUT_DROP,
	offsetof(struct fm10k_port_counters, cnt_tx_timeout_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_timeout_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_ERR_DROP,
	offsetof(struct fm10k_port_counters, cnt_tx_error_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_error_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_ECC_DROP,
	offsetof(struct fm10k_port_counters, cnt_tx_unrepair_ecc_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_unrepair_ecc_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_LOOPBACK_DROP,
	offsetof(struct fm10k_port_counters, cnt_tx_loopback_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_loopback_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_TTL1_DROP,
	offsetof(struct fm10k_port_counters, cnt_tx_ttl_drop_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_ttl_drop_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_IEEE802_3_PAUSE,
	offsetof(struct fm10k_port_counters, cnt_tx_pause_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_pause_octets)},
{	FM10K_TX_STAT_BANK_TYPE, FM10K_TX_STAT_CLASS_BASED_PAUSE,
	offsetof(struct fm10k_port_counters, cnt_tx_cbpause_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_cbpause_octets)},

    /* Size Bank */
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_LT_64,
	offsetof(struct fm10k_port_counters, cnt_tx_minto63_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_minto63_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_EQ_64,
	offsetof(struct fm10k_port_counters, cnt_tx_64_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_64_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_65_127,
	offsetof(struct fm10k_port_counters, cnt_tx_65to127_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_65to127_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_128_255,
	offsetof(struct fm10k_port_counters, cnt_tx_128to255_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_128to255_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_256_511,
	offsetof(struct fm10k_port_counters, cnt_tx_256to511_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_256to511_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_512_1023,
	offsetof(struct fm10k_port_counters, cnt_tx_512to1023_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_512to1023_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_1024_1522,
	offsetof(struct fm10k_port_counters, cnt_tx_1024to1522_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_1024to1522_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_1523_2047,
	offsetof(struct fm10k_port_counters, cnt_tx_1523to2047_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_1523to2047_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_2048_4095,
	offsetof(struct fm10k_port_counters, cnt_tx_2048to4095_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_2048to4095_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_4096_8191,
	offsetof(struct fm10k_port_counters, cnt_tx_4096to8191_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_4096to8191_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_8192_10239,
	offsetof(struct fm10k_port_counters, cnt_tx_8192to10239_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_8192to10239_octets)},
{	FM10K_TX_STAT_BANK_SIZE, FM10K_TX_STAT_LEN_GE_10240,
	offsetof(struct fm10k_port_counters, cnt_tx_10240tomax_pkts),
	offsetof(struct fm10k_port_counters, cnt_tx_10240tomax_octets)},

};  /* end tx_port_cnt_map_table */


/*
 * Get a 32bit EPL counter by adding it to the scatter gather
 * list.
 * Depends on the existence of the variables:
 * sgList [out] - scatter gather array to hold the reads
 * sgListCnt [out] - number of entry in the sgList
 */
static inline void
fm10k_get_epl_port_stat32(struct fm10k_scatter_gather_entry *sg_list,
		struct fm10k_port_counters *counters,
		int *sg_list_cnt, int epl, int lane)
{
	sg_list[*sg_list_cnt].addr = FM10K_MAC_OVERSIZE_COUNTER(epl, lane);
	sg_list[*sg_list_cnt].data =
			(uint32_t *)&counters->cnt_rx_oversized_pkts;
	sg_list[*sg_list_cnt].count = 1;
	(*sg_list_cnt)++;

	sg_list[*sg_list_cnt].addr = FM10K_MAC_JABBER_COUNTER(epl, lane);
	sg_list[*sg_list_cnt].data =
			(uint32_t *)&counters->cnt_rx_jabber_pkts;
	sg_list[*sg_list_cnt].count = 1;
	(*sg_list_cnt)++;

	sg_list[*sg_list_cnt].addr = FM10K_MAC_UNDERSIZE_COUNTER(epl, lane);
	sg_list[*sg_list_cnt].data =
			(uint32_t *)&counters->cnt_rx_undersized_pkts;
	sg_list[*sg_list_cnt].count = 1;
	(*sg_list_cnt)++;

	sg_list[*sg_list_cnt].addr = FM10K_MAC_RUNT_COUNTER(epl, lane);
	sg_list[*sg_list_cnt].data =
			(uint32_t *)&counters->cnt_rx_fragment_pkts;
	sg_list[*sg_list_cnt].count = 1;
	(*sg_list_cnt)++;

	sg_list[*sg_list_cnt].addr = FM10K_MAC_OVERRUN_COUNTER(epl, lane);
	sg_list[*sg_list_cnt].data =
			(uint32_t *)&counters->cnt_over_run_pkts;
	sg_list[*sg_list_cnt].count = 1;
	(*sg_list_cnt)++;

	sg_list[*sg_list_cnt].addr = FM10K_MAC_CODE_ERROR_COUNTER(epl, lane);
	sg_list[*sg_list_cnt].data =
			(uint32_t *)&counters->cnt_code_errors;
	sg_list[*sg_list_cnt].count = 1;
	(*sg_list_cnt)++;
}


static void
fm10k_read_scatter_gather(struct fm10k_switch *sw,
		int n_entries,
		struct fm10k_scatter_gather_entry *sg_list)
{
	int i;

	for (i = 0; i < n_entries; i++) {
		uint32_t addr  = sg_list[i].addr;
		uint32_t count = sg_list[i].count;
		uint32_t *data = sg_list[i].data;

		fm10k_read_switch_array(sw, addr, data, count);
	}
}

static int
fm10k_get_port_counters(struct fm10k_switch *sw,
		struct fm10k_ext_port *ext_port,
		struct fm10k_port_counters *counters)
{
	int phys_port;
	int epl;
	int lane;
	struct timeval ts;
	bool valid_ip_stats = true;
	uint32_t i;
	struct fm10k_scatter_gather_entry sg_list[MAX_STATS_SGLIST];
	int sg_list_cnt = 0;
	/* Temporary bin array to retrieve 128bit
	 * port counters (frame + bytes).
	 */
	uint32_t cnt_rx_port_stats_bank[FM10K_NB_RX_STATS_BANKS]
				       [FM10K_BINS_PER_RX_STATS_BANK]
				       [FM10K_WORDS_PER_RX_STATS_COUNTER];
	/*
	 * Fill the counters structure with 0 to assure
	 * all fields not explicitly set below, which will
	 * be the fields not supported by the FM10000,
	 * will be 0 on return.
	 */
	memset(counters, 0, sizeof(*counters));

	phys_port = ext_port->portno;
	epl = ext_port->eplno;
	lane = ext_port->first_lane;

	counters->cnt_version = FM10K_STATS_VERSION;

	/*
	 * Reading counters for each RX bank
	 *
	 * Because the RX_STATS_BANK register contains
	 * both, frame count and byte count in a 128bit
	 * register, we first store the data in a temporary
	 * array.
	 * 1. Fill Scatter Gather to retrieve each bin
	 *    counter from the rx bank and store in a temp
	 *    array.
	 */
	for (i = 0;
			i < sizeof(rx_port_cnt_map_table) /
				sizeof(rx_port_cnt_map_table[0]);
			i++) {
		/*
		 * Add a 128bit read of an rx counter to the scatter gather
		 * list.
		 * NOTE: Read values will be stored in the temporary
		 *       array switchExt->cnt_rx_port_stats_bank;
		 * Depends on the existence of the variables:
		 * switchExt->cnt_rx_PortStatsBank [out] - Array to store the
		 * 128bit values sgList
		 * [out] - scatter gather array to hold the reads sgListCnt
		 * [out] - number of entry in the sgList
		 */
		uint32_t bank = rx_port_cnt_map_table[i].bank;
		uint32_t bin = rx_port_cnt_map_table[i].bin;

		sg_list[sg_list_cnt].addr =
			FM10K_RX_STATS_BANK(bank, (phys_port << 4 | (bin)), 0);
		sg_list[sg_list_cnt].data = cnt_rx_port_stats_bank[bank][bin];
		sg_list[sg_list_cnt].count = 4;
		sg_list_cnt++;
}

	/*
	 * 2. Fill in the scatter gather for tx bank
	 *    counters. They will directly be stored
	 *    in the fm10k_counters structure upon
	 *    scatter gather read.
	 */
	for (i = 0;
			i < sizeof(tx_port_cnt_map_table) /
				sizeof(tx_port_cnt_map_table[0]);
			i++) {
		/*
		 * Add a 64bit read of a tx counter to the scatter gather list.
		 * Depends on the existence of the variables:
		 * sgList [out] - scatter gather array to hold the reads
		 * sgListCnt [out] - number of entry in the sgList
		 */
		uint32_t bank = tx_port_cnt_map_table[i].bank;
		uint32_t bin = tx_port_cnt_map_table[i].bin;
		uint32_t frame_offset = tx_port_cnt_map_table[i].frame_offset;

		sg_list[sg_list_cnt].addr =
			FM10K_MOD_STATS_BANK_FRAME(bank,
					(phys_port << 4 | (bin)), 0);
		sg_list[sg_list_cnt].data =
			(uint32_t *)(((uint8_t *)counters) + (frame_offset));
		sg_list[sg_list_cnt].count = 2;
		sg_list_cnt++;
	}

	/*
	 * 3. Fill in the scatter gather for TX CM DROP
	 *    and for EPL counters. They will directly be
	 *    stored in the fm10k_counters structure upon
	 *    scatter gather read.
	 */
	sg_list[sg_list_cnt].addr =
			FM10K_CM_APPLY_DROP_COUNT(phys_port, 0);
	sg_list[sg_list_cnt].data =
			(uint32_t *)&counters->cnt_rx_cm_drop_pkts;
	sg_list[sg_list_cnt].count = 2;
	sg_list_cnt++;
	/* EPL Counters */
	fm10k_get_epl_port_stat32(sg_list, counters, &sg_list_cnt, epl, lane);

	/*
	 * 4. Execute scatter gather read.
	 */
	if (sg_list_cnt >= MAX_STATS_SGLIST) {
		/*
		 * Pretty static. Mainly to warn if something new added,
		 * but the array size is not adjust accordingly
		 */
		FM10K_SW_ERR("Scatter list array %d for port %d overflow.\n",
				sg_list_cnt, phys_port);
		return -1;
	}

	/*
	 * Taking lock to protect temporary structures used to
	 * store 128b counters
	 */
	FM10K_SW_SWITCH_LOCK(sw);

	/* now get the stats in one shot, optimized for fibm */
	fm10k_read_scatter_gather(sw, sg_list_cnt, sg_list);
	FM10K_SW_SWITCH_UNLOCK(sw);

	counters->timestamp = ts.tv_sec * 1000000 + ts.tv_usec;

	/*
	 * 5. Retrieve the frame/byte counts from 128bit
	 *    registers stored in temporary array and set
	 *    the proper fm_portCounter structure members.
	 */
	for (i = 0;
			i < sizeof(rx_port_cnt_map_table) /
				sizeof(rx_port_cnt_map_table[0]);
			i++) {
		/*
		 * Update the counter variable related to a 128bit HW counter
		 * NOTE: Read values will be retrieved from the temporary
		 *       array switchExt->cnt_rx_PortStatsBank;
		 * Depends on the existence of the variables:
		 * switchExt->cnt_rx_PortStatsBank [in] - Array to read the
		 * 128bit values from
		 */
		uint32_t bank = rx_port_cnt_map_table[i].bank;
		uint32_t bin = rx_port_cnt_map_table[i].bin;
		uint32_t frame_offset = rx_port_cnt_map_table[i].frame_offset;
		uint32_t byte_offset = rx_port_cnt_map_table[i].byte_offset;

		*((uint64_t *)(((uint8_t *)counters) + frame_offset)) =
		(((uint64_t)(cnt_rx_port_stats_bank[bank][bin][1]) << 32) |
		((uint64_t)(cnt_rx_port_stats_bank[bank][bin][0])));

		*((uint64_t *)(((uint8_t *)counters) + byte_offset)) =
		(((uint64_t)(cnt_rx_port_stats_bank[bank][bin][3]) << 32) |
		((uint64_t)(cnt_rx_port_stats_bank[bank][bin][2])));
	}

	/*
	 * 6. Set some counters that are not available
	 *    in HW but can be computed from two or more
	 *    HW counters.
	 */
	/*
	 * If IP parsing is enabled then the IP stats will be read and will be
	 * valid. Else all traffic, whether IP or not, is counted as _nonip.
	 */
	if (valid_ip_stats) {
		/*
		 * RX counters.
		 */
		counters->cnt_rx_ucst_pkts =
				counters->cnt_rx_ucst_pkts_nonip +
				counters->cnt_rx_ucst_pkts_ipv4 +
				counters->cnt_rx_ucst_pkts_ipv6;

		counters->cnt_rx_mcst_pkts =
				counters->cnt_rx_mcst_pkts_nonip +
				counters->cnt_rx_mcst_pkts_ipv4 +
				counters->cnt_rx_mcst_pkts_ipv6;

		counters->cnt_rx_bcst_pkts =
				counters->cnt_rx_bcst_pkts_nonip +
				counters->cnt_rx_bcst_pkts_ipv4 +
				counters->cnt_rx_bcst_pkts_ipv6;

		/*
		 * Misc. counters.
		 */
		counters->cnt_rx_octets_nonip  =
				counters->cnt_rx_ucst_octets_nonip +
				counters->cnt_rx_mcst_octets_nonip +
				counters->cnt_rx_bcst_octets_nonip;

		counters->cnt_rx_octets_ipv4  =
				counters->cnt_rx_ucst_octets_ipv4 +
				counters->cnt_rx_mcst_octets_ipv4 +
				counters->cnt_rx_bcst_octets_ipv4;

		counters->cnt_rx_octets_ipv6  =
				counters->cnt_rx_ucst_octets_ipv6 +
				counters->cnt_rx_mcst_octets_ipv6 +
				counters->cnt_rx_bcst_octets_ipv6;

		counters->cnt_rx_good_octets =
				counters->cnt_rx_octets_nonip +
				counters->cnt_rx_octets_ipv4 +
				counters->cnt_rx_octets_ipv6;
	} else {
		/*
		 * RX counters.
		 */
		counters->cnt_rx_ucst_pkts =
				counters->cnt_rx_ucst_pkts_nonip;
		counters->cnt_rx_mcst_pkts =
				counters->cnt_rx_mcst_pkts_nonip;
		counters->cnt_rx_bcst_pkts =
				counters->cnt_rx_bcst_pkts_nonip;

		/*
		 * Misc. counters.
		 */
		counters->cnt_rx_octets_nonip  =
				counters->cnt_rx_ucst_octets_nonip +
				counters->cnt_rx_mcst_octets_nonip +
				counters->cnt_rx_bcst_octets_nonip;

		counters->cnt_rx_good_octets =
				counters->cnt_rx_octets_nonip;
	}

	counters->cnt_trigger_drop_redir_pkts =
			counters->cnt_trigger_redir_pkts +
			counters->cnt_trigger_drop_pkts;

	/* Emulate Tx _octets counter using the sum of all size bins. */
	counters->cnt_tx_octets += counters->cnt_tx_minto63_octets;
	counters->cnt_tx_octets += counters->cnt_tx_64_octets;
	counters->cnt_tx_octets += counters->cnt_tx_65to127_octets;
	counters->cnt_tx_octets += counters->cnt_tx_128to255_octets;
	counters->cnt_tx_octets += counters->cnt_tx_256to511_octets;
	counters->cnt_tx_octets += counters->cnt_tx_512to1023_octets;
	counters->cnt_tx_octets += counters->cnt_tx_1024to1522_octets;
	counters->cnt_tx_octets += counters->cnt_tx_1523to2047_octets;
	counters->cnt_tx_octets += counters->cnt_tx_2048to4095_octets;
	counters->cnt_tx_octets += counters->cnt_tx_4096to8191_octets;
	counters->cnt_tx_octets += counters->cnt_tx_8192to10239_octets;
	counters->cnt_tx_octets += counters->cnt_tx_10240tomax_octets;

	return 0;
}   /* end fm10k_get_port_counters */

struct fm10k_stats_data {
	uint64_t rx_pkts;
	uint64_t tx_pkts;
	uint64_t rx_drop;
};

static uint64_t
fm10k_stats_ffu_count_get(struct fm10k_switch *sw, int table_id)
{
	uint64_t data;

	if (FM10K_SW_FFU_CNT_BANK < 2) {
		data = fm10k_read_switch_reg64(sw,
				0xE40000 + 0x4000 * FM10K_SW_FFU_CNT_BANK +
				0x4 * (table_id + FM10K_SW_FFU_CNT_START) +
				0x8000);
	} else {
		data = fm10k_read_switch_reg64(sw,
				0xE40000 + 0x800 * (FM10K_SW_FFU_CNT_BANK - 2) +
				0x4 * (table_id + FM10K_SW_FFU_CNT_START) +
				0x10000);
	}
	return data;
}

static void
fm10k_stats_epl_ffu_print(struct fm10k_switch *sw, int epl)
{
	int table_id;
	uint64_t data;

	table_id = FM10K_FFU_EXT_PORT_RULE_INGRESS(epl);
	data = fm10k_stats_ffu_count_get(sw, table_id);
	FM10K_SW_INFO("             FFU[%d] ingress        %-12llu\n",
			table_id, (unsigned long long)(data));

	table_id = FM10K_FFU_EXT_PORT_RULE_EGRESS(epl);
	data = fm10k_stats_ffu_count_get(sw, table_id);
	FM10K_SW_INFO("             FFU[%d]  egress        %-12llu\n",
			table_id, (unsigned long long)(data));
}

#define FM10K_STATS_RULE_NUM_MAX	100
static uint16_t fm10k_stats_rule_list[FM10K_STATS_RULE_NUM_MAX];
void
fm10k_stats_rule_count_reg(uint16_t rule_id)
{
	int i;

	for (i = 0; i < FM10K_STATS_RULE_NUM_MAX; i++) {
		if (fm10k_stats_rule_list[i] == 0) {
			fm10k_stats_rule_list[i] = rule_id;
			break;
		}
	}
}

void
fm10k_stats_ffu_count_print(struct fm10k_switch *sw)
{
	int i, rule_id;
	uint64_t data;
	static uint64_t ffu_count[FM10K_STATS_RULE_NUM_MAX];

	for (i = 0; i < FM10K_STATS_RULE_NUM_MAX; i++) {
		if (fm10k_stats_rule_list[i] == 0)
			continue;

		rule_id = fm10k_stats_rule_list[i];
		data = fm10k_stats_ffu_count_get(sw, rule_id);
		FM10K_SW_INFO("FFU[%d] count         %-12llu\n",
				rule_id,
				(unsigned long long)
				(data - ffu_count[rule_id]));
		ffu_count[rule_id] = data;
	}
}

void
fm10k_stats_epl_port_print(struct fm10k_switch *sw)
{
	int i, lport;
	struct fm10k_port_counters counters;
	struct fm10k_ext_port ext_port;
	static struct fm10k_stats_data last_data[FM10K_SW_EXT_PORTS_MAX];
	struct fm10k_stats_data data;

	for (i = 0; i < FM10K_SW_EPLS_SUPPORTED; i++) {
		lport = sw->epl_map[i].logical_port;
		ext_port.portno = lport;
		fm10k_get_port_counters(sw, &ext_port, &counters);

		data.rx_pkts =
			counters.cnt_rx_bcst_pkts +
			counters.cnt_rx_ucst_pkts;
		data.tx_pkts =
			counters.cnt_tx_bcst_pkts +
			counters.cnt_tx_ucst_pkts;
		data.rx_drop = counters.cnt_cmpriv_drop_pkts;
		FM10K_SW_INFO("EPL  port %-2d lport %-5d tx_pkt %-12llu "
				"rx_pkt %-12llu drop_pkt %-12llu\n",
				i, lport,
				(unsigned long long)
				(data.tx_pkts - last_data[i].tx_pkts),
				(unsigned long long)
				(data.rx_pkts - last_data[i].rx_pkts),
				(unsigned long long)
				(data.rx_drop - last_data[i].rx_drop));
		last_data[i] = data;

		if (fm10k_config_check_debug(sw->dpdk_cfg,
				FM10K_CONFIG_DEBUG_STATS_FFU))
			fm10k_stats_epl_ffu_print(sw, i);
	}
}

static void
fm10k_stats_port_counter_print(struct fm10k_switch *sw, int lport)
{
	unsigned int i;
	struct fm10k_port_counters counters;
	struct fm10k_ext_port ext_port;
	uint64_t *pdata;

	memset(&ext_port, 0, sizeof(ext_port));
	ext_port.portno = lport;
	fm10k_get_port_counters(sw, &ext_port, &counters);

	for (i = 0;
			i < sizeof(rx_port_cnt_map_table) /
				sizeof(rx_port_cnt_map_table[0]);
			i++) {
		pdata =
			(uint64_t *)((uint8_t *)(&counters) +
			rx_port_cnt_map_table[i].frame_offset);
		if (*pdata != 0) {
			FM10K_SW_INFO("port %d rx bank %d idx %d pkt %llu\n",
					lport,
					rx_port_cnt_map_table[i].bank,
					rx_port_cnt_map_table[i].bin,
					(unsigned long long)*pdata);
		}
	}
	for (i = 0;
			i < sizeof(tx_port_cnt_map_table) /
				sizeof(tx_port_cnt_map_table[0]);
			i++) {
		pdata =
			(uint64_t *)((uint8_t *)(&counters) +
			tx_port_cnt_map_table[i].frame_offset);
		if (*pdata != 0) {
			FM10K_SW_INFO("port %d tx bank %d idx %d pkt %llu\n",
					lport,
					tx_port_cnt_map_table[i].bank,
					tx_port_cnt_map_table[i].bin,
					(unsigned long long)*pdata);
		}
	}
}

void
fm10k_stats_port_bank_print(struct fm10k_switch *sw)
{
	int i;

	for (i = 0;
		 i < FM10K_SW_EPLS_SUPPORTED;
		 i++) {
		fm10k_stats_port_counter_print(sw,
				sw->epl_map[i].logical_port);
	}
	for (i = 0;
		 i < FM10K_SW_PEPS_SUPPORTED;
		 i++) {
		fm10k_stats_port_counter_print(sw,
				sw->pep_map[i].logical_port);
	}
}

void *
fm10k_switch_process_stats(void *ctx)
{
	struct fm10k_switch *sw = ctx;

	usec_delay(5000000);

	if (!fm10k_config_check_debug(sw->dpdk_cfg,
			FM10K_CONFIG_DEBUG_ENABLE))
		return NULL;

	if (!sw->dpdk_cfg->stats_interval)
		return NULL;

	while (sw->detaching == 0) {
		if (fm10k_config_check_debug(sw->dpdk_cfg,
				FM10K_CONFIG_DEBUG_STATS_PORT)) {
			FM10K_SW_INFO("--- port statistic ---\n");
			fm10k_stats_epl_port_print(sw);
		}
		if (fm10k_config_check_debug(sw->dpdk_cfg,
				FM10K_CONFIG_DEBUG_STATS_FFU)) {
			FM10K_SW_INFO("--- ffu statistic ---\n");
			fm10k_stats_ffu_count_print(sw);
		}
		if (fm10k_config_check_debug(sw->dpdk_cfg,
				FM10K_CONFIG_DEBUG_STATS_MORE)) {
			FM10K_SW_INFO("--- detail statistic ---\n");
			fm10k_stats_port_bank_print(sw);
		}
		usec_delay(1000000 * sw->dpdk_cfg->stats_interval);
	}
	return NULL;
}
