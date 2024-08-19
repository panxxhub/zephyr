/*
 * Copyright (c) 2024 iCNC Precision Metal Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __ZEPHYR_DRIVERS_USB_UDC_ZYNQ_H__
#define __ZEPHYR_DRIVERS_USB_UDC_ZYNQ_H__

#include <zephyr/device.h>
#include <zephyr/drivers/usb/udc.h>

/************************** Constant Definitions *****************************/

#define XUSBPS_REG_SPACING 4

/** @name Timer 0 Register offsets
 *
 * @{
 */
#define XUSBPS_TIMER0_LD_OFFSET  0x00000080
#define XUSBPS_TIMER0_CTL_OFFSET 0x00000084
/* @} */

/** @name Timer Control Register bit mask
 *
 * @{
 */
#define XUSBPS_TIMER_RUN_MASK    0x80000000
#define XUSBPS_TIMER_STOP_MASK   0x80000000
#define XUSBPS_TIMER_RESET_MASK  0x40000000
#define XUSBPS_TIMER_REPEAT_MASK 0x01000000
/* @} */

/** @name Timer Control Register bit mask
 *
 * @{
 */
#define XUSBPS_TIMER_COUNTER_MASK 0x00FFFFFF
/* @} */

/** @name Device Hardware Parameters
 *
 * @{
 */
#define XUSBPS_HWDEVICE_OFFSET 0x0000000C

#define XUSBPS_EP_NUM_MASK  0x3E
#define XUSBPS_EP_NUM_SHIFT 1
/* @} */

/** @name Capability Register offsets
 */
#define XUSBPS_HCSPARAMS_OFFSET 0x00000104

/** @name Operational Register offsets.
 * Register comments are tagged with "H:" and "D:" for Host and Device modes,
 * respectively.
 * Tags are only present for registers that have a different meaning DEVICE and
 * HOST modes. Most registers are only valid for either DEVICE or HOST mode.
 * Those registers don't have tags.
 * @{
 */
#define XUSBPS_CMD_OFFSET           0x00000140 /**< Configuration */
#define XUSBPS_ISR_OFFSET           0x00000144 /**< Interrupt Status */
#define XUSBPS_IER_OFFSET           0x00000148 /**< Interrupt Enable */
#define XUSBPS_FRAME_OFFSET         0x0000014C /**< USB Frame Index */
#define XUSBPS_LISTBASE_OFFSET      0x00000154 /**< H: Periodic List Base Address */
#define XUSBPS_DEVICEADDR_OFFSET    0x00000154 /**< D: Device Address */
#define XUSBPS_ASYNCLISTADDR_OFFSET 0x00000158 /**< H: Async List Address */
#define XUSBPS_EPLISTADDR_OFFSET    0x00000158 /**< D: Endpoint List Addr */
#define XUSBPS_TTCTRL_OFFSET        0x0000015C /**< TT Control */
#define XUSBPS_BURSTSIZE_OFFSET     0x00000160 /**< Burst Size */
#define XUSBPS_TXFILL_OFFSET        0x00000164 /**< Tx Fill Tuning */
#define XUSBPS_ULPIVIEW_OFFSET      0x00000170 /**< ULPI Viewport */
#define XUSBPS_EPNAKISR_OFFSET      0x00000178 /**< Endpoint NAK IRQ Status */
#define XUSBPS_EPNAKIER_OFFSET      0x0000017C /**< Endpoint NAK IRQ Enable */
#define XUSBPS_PORTSCR1_OFFSET      0x00000184 /**< Port Control/Status 1 */

/* NOTE: The Port Control / Status Register index is 1-based. */
#define XUSBPS_PORTSCRn_OFFSET(n) (XUSBPS_PORTSCR1_OFFSET + (((n) - 1) * XUSBPS_REG_SPACING))

#define XUSBPS_OTGCSR_OFFSET  0x000001A4 /**< OTG Status and Control */
#define XUSBPS_MODE_OFFSET    0x000001A8 /**< USB Mode */
#define XUSBPS_EPSTAT_OFFSET  0x000001AC /**< Endpoint Setup Status */
#define XUSBPS_EPPRIME_OFFSET 0x000001B0 /**< Endpoint Prime */
#define XUSBPS_EPFLUSH_OFFSET 0x000001B4 /**< Endpoint Flush */
#define XUSBPS_EPRDY_OFFSET   0x000001B8 /**< Endpoint Ready */
#define XUSBPS_EPCOMPL_OFFSET 0x000001BC /**< Endpoint Complete */
#define XUSBPS_EPCR0_OFFSET   0x000001C0 /**< Endpoint Control 0 */
#define XUSBPS_EPCR1_OFFSET   0x000001C4 /**< Endpoint Control 1 */
#define XUSBPS_EPCR2_OFFSET   0x000001C8 /**< Endpoint Control 2 */
#define XUSBPS_EPCR3_OFFSET   0x000001CC /**< Endpoint Control 3 */
#define XUSBPS_EPCR4_OFFSET   0x000001D0 /**< Endpoint Control 4 */

#define XUSBPS_MAX_ENDPOINTS   12         /**< Number of supported Endpoints in this core. */
#define XUSBPS_EP_OUT_MASK     0x00000FFF /**< OUR (RX) endpoint mask */
#define XUSBPS_EP_IN_MASK      0x0FFF0000 /**< IN (TX) endpoint mask */
#define XUSBPS_EP_ALL_MASK     0x0FFF0FFF /**< Mask used for endpoint control registers */
#define XUSBPS_EPCRn_OFFSET(n) (XUSBPS_EPCR0_OFFSET + ((n) * XUSBPS_REG_SPACING))

#define XUSBPS_EPFLUSH_RX_SHIFT 0
#define XUSBPS_EPFLUSH_TX_SHIFT 16

/* @} */

/** @name Endpoint Control Register (EPCR) bit positions.
 *  @{
 */

/* Definitions for TX Endpoint bits */
#define XUSBPS_EPCR_TXT_TYPE_SHIFT   18         /* < Endpoint Type - TX bit shift*/
#define XUSBPS_EPCR_TXT_TYPE_MASK    0x000C0000 /* < Endpoint Type - TX read only*/
#define XUSBPS_EPCR_TXT_CONTROL_MASK 0x00000000 /**< Control Endpoint - TX */
#define XUSBPS_EPCR_TXT_ISO_MASK     0x00040000 /**< Isochronous. Endpoint */
#define XUSBPS_EPCR_TXT_BULK_MASK    0x00080000 /**< Bulk Endpoint - TX */
#define XUSBPS_EPCR_TXT_INTR_MASK    0x000C0000 /**< Interrupt Endpoint */
#define XUSBPS_EPCR_TXS_MASK         0x00010000 /**< Stall TX endpoint */
#define XUSBPS_EPCR_TXE_MASK         0x00800000 /**< Transmit enable  - TX */
#define XUSBPS_EPCR_TXR_MASK         0x00400000 /**< Data Toggle Reset Bit */

/* Definitions for RX Endpoint bits */
#define XUSBPS_EPCR_RXT_TYPE_SHIFT   2          /* < Endpoint Type - RX bit shift*/
#define XUSBPS_EPCR_RXT_TYPE_MASK    0x0000000C /**< Endpoint Type - RX read only*/
#define XUSBPS_EPCR_RXT_CONTROL_MASK 0x00000000 /**< Control Endpoint - RX */
#define XUSBPS_EPCR_RXT_ISO_MASK     0x00000004 /**< Isochronous Endpoint */
#define XUSBPS_EPCR_RXT_BULK_MASK    0x00000008 /**< Bulk Endpoint - RX */
#define XUSBPS_EPCR_RXT_INTR_MASK    0x0000000C /**< Interrupt Endpoint */
#define XUSBPS_EPCR_RXS_MASK         0x00000001 /**< Stall RX endpoint. */
#define XUSBPS_EPCR_RXE_MASK         0x00000080 /**< Transmit enable. - RX */
#define XUSBPS_EPCR_RXR_MASK         0x00000040 /**< Data Toggle Reset Bit */
/* @} */

/** @name USB Command Register (CR) bit positions.
 *  @{
 */
#define XUSBPS_CMD_RS_MASK    0x00000001 /**< Run/Stop */
#define XUSBPS_CMD_RST_MASK   0x00000002 /**< Controller RESET */
#define XUSBPS_CMD_FS01_MASK  0x0000000C /**< Frame List Size bit 0,1 */
#define XUSBPS_CMD_PSE_MASK   0x00000010 /**< Periodic Sched Enable */
#define XUSBPS_CMD_ASE_MASK   0x00000020 /**< Async Sched Enable */
#define XUSBPS_CMD_IAA_MASK   0x00000040 /**< IRQ Async Advance Doorbell */
#define XUSBPS_CMD_ASP_MASK   0x00000300 /**< Async Sched Park Mode Cnt */
#define XUSBPS_CMD_ASPE_MASK  0x00000800 /**< Async Sched Park Mode Enbl */
#define XUSBPS_CMD_SUTW_MASK  0x00002000 /**< Setup TripWire */
#define XUSBPS_CMD_ATDTW_MASK 0x00004000 /**< Add dTD TripWire */
#define XUSBPS_CMD_FS2_MASK   0x00008000 /**< Frame List Size bit 2 */
#define XUSBPS_CMD_ITC_MASK   0x00FF0000 /**< IRQ Threshold Control */
/* @} */

/**
 * Endpoint Device Transfer Descriptor
 *
 * The dTD describes to the device controller the location and quantity of data
 * to be sent/received for given transfer. The driver does not attempt to
 * modify any field in an active dTD except the Next Link Pointer.
 */
#define XUSBPS_dTDNLP     0x00 /**< Pointer to the next descriptor */
#define XUSBPS_dTDTOKEN   0x04 /**< Descriptor Token */
#define XUSBPS_dTDBPTR0   0x08 /**< Buffer Pointer 0 */
#define XUSBPS_dTDBPTR1   0x0C /**< Buffer Pointer 1 */
#define XUSBPS_dTDBPTR2   0x10 /**< Buffer Pointer 2 */
#define XUSBPS_dTDBPTR3   0x14 /**< Buffer Pointer 3 */
#define XUSBPS_dTDBPTR4   0x18 /**< Buffer Pointer 4 */
#define XUSBPS_dTDBPTR(n) (XUSBPS_dTDBPTR0 + (n) * 0x04)
#define XUSBPS_dTDRSRVD   0x1C /**< Reserved field */

/* We use the reserved field in the dTD to store user data. */
#define XUSBPS_dTDUSERDATA XUSBPS_dTDRSRVD /**< Reserved field */

/** @name dTD Next Link Pointer (dTDNLP) bit positions.
 *  @{
 */
#define XUSBPS_dTDNLP_T_MASK    0x00000001
/**< USB dTD Next Link Pointer Terminate Bit */
#define XUSBPS_dTDNLP_ADDR_MASK 0xFFFFFFE0
/**< USB dTD Next Link Pointer Address [31:5] */
/* @} */

/** @name dTD Token (dTDTOKEN) bit positions.
 *  @{
 */
#define XUSBPS_dTDTOKEN_XERR_MASK   0x00000008 /**< dTD Transaction Error */
#define XUSBPS_dTDTOKEN_BUFERR_MASK 0x00000020 /**< dTD Data Buffer Error */
#define XUSBPS_dTDTOKEN_HALT_MASK   0x00000040 /**< dTD Halted Flag */
#define XUSBPS_dTDTOKEN_ACTIVE_MASK 0x00000080 /**< dTD Active Bit */
#define XUSBPS_dTDTOKEN_MULTO_MASK  0x00000C00 /**< Multiplier Override Field [1:0] */
#define XUSBPS_dTDTOKEN_IOC_MASK    0x00008000 /**< Interrupt on Complete Bit */
#define XUSBPS_dTDTOKEN_LEN_MASK    0x7FFF0000 /**< Transfer Length Field */
/* @} */

/** @name USB Mode Register (MODE) bit positions.
 *  @{
 */
#define XUSBPS_MODE_CM_MASK		0x00000003 /**< Controller Mode Select */
#define XUSBPS_MODE_CM_IDLE_MASK	0x00000000
#define XUSBPS_MODE_CM_DEVICE_MASK	0x00000002
#define XUSBPS_MODE_CM_HOST_MASK	0x00000003
#define XUSBPS_MODE_ES_MASK		0x00000004 /**< USB Endian Select */
#define XUSBPS_MODE_SLOM_MASK		0x00000008 /**< USB Setup Lockout Mode Disable */
#define XUSBPS_MODE_SDIS_MASK		0x00000010
#define XUSBPS_MODE_VALID_MASK		0x0000001F

/* @} */

/** @name USB Device Address Register (DEVICEADDR) bit positions.
 *  @{
 */
#define XUSBPS_DEVICEADDR_DEVICEAADV_MASK	0x01000000
/**< Device Addr Auto Advance */
#define XUSBPS_DEVICEADDR_ADDR_MASK		0xFE000000
/**< Device Address */
#define XUSBPS_DEVICEADDR_ADDR_SHIFT		25
/**< Address shift */
#define XUSBPS_DEVICEADDR_MAX			127
/**< Biggest allowed address */
/* @} */

/** @name USB TT Control Register (TTCTRL) bit positions.
 *  @{
 */
#define XUSBPS_TTCTRL_HUBADDR_MASK	0x7F000000 /**< TT Hub Address */
/* @} */


/** @name USB Burst Size Register (BURSTSIZE) bit posisions.
 *  @{
 */
#define XUSBPS_BURSTSIZE_RX_MASK	0x000000FF /**< RX Burst Length */
#define XUSBPS_BURSTSIZE_TX_MASK	0x0000FF00 /**< TX Burst Length */
/* @} */


/** @name USB Tx Fill Tuning Register (TXFILL) bit positions.
 *  @{
 */
#define XUSBPS_TXFILL_OVERHEAD_MASK	0x000000FF
/**< Scheduler Overhead */
#define XUSBPS_TXFILL_HEALTH_MASK	0x00001F00
/**< Scheduler Health Cntr */
#define XUSBPS_TXFILL_BURST_MASK	0x003F0000
/**< FIFO Burst Threshold */
/* @} */

/** @name USB ULPI Viewport Register (ULPIVIEW) bit positions.
 *  @{
 */
#define XUSBPS_ULPIVIEW_DATWR_MASK 0x000000FF /**< ULPI Data Write */
#define XUSBPS_ULPIVIEW_DATRD_MASK 0x0000FF00 /**< ULPI Data Read */
#define XUSBPS_ULPIVIEW_ADDR_MASK  0x00FF0000 /**< ULPI Data Address */
#define XUSBPS_ULPIVIEW_PORT_MASK  0x07000000 /**< ULPI Port Number */
#define XUSBPS_ULPIVIEW_SS_MASK    0x08000000 /**< ULPI Synchronous State */
#define XUSBPS_ULPIVIEW_RW_MASK    0x20000000 /**< ULPI Read/Write Control */
#define XUSBPS_ULPIVIEW_RUN_MASK   0x40000000 /**< ULPI Run */
#define XUSBPS_ULPIVIEW_WU_MASK    0x80000000 /**< ULPI Wakeup */
/* @} */

/** @name Port Status Control Register bit positions.
 *  @{
 */
#define XUSBPS_PORTSCR_CCS_MASK  0x00000001 /**< Current Connect Status */
#define XUSBPS_PORTSCR_CSC_MASK  0x00000002 /**< Connect Status Change */
#define XUSBPS_PORTSCR_PE_MASK   0x00000004 /**< Port Enable/Disable */
#define XUSBPS_PORTSCR_PEC_MASK  0x00000008 /**< Port Enable/Disable Change */
#define XUSBPS_PORTSCR_OCA_MASK  0x00000010 /**< Over-current Active */
#define XUSBPS_PORTSCR_OCC_MASK  0x00000020 /**< Over-current Change */
#define XUSBPS_PORTSCR_FPR_MASK  0x00000040 /**< Force Port Resume */
#define XUSBPS_PORTSCR_SUSP_MASK 0x00000080 /**< Suspend */
#define XUSBPS_PORTSCR_PR_MASK   0x00000100 /**< Port Reset */
#define XUSBPS_PORTSCR_HSP_MASK  0x00000200 /**< High Speed Port */
#define XUSBPS_PORTSCR_LS_MASK   0x00000C00 /**< Line Status */
#define XUSBPS_PORTSCR_PP_MASK   0x00001000 /**< Port Power */
#define XUSBPS_PORTSCR_PO_MASK   0x00002000 /**< Port Owner */
#define XUSBPS_PORTSCR_PIC_MASK  0x0000C000 /**< Port Indicator Control */
#define XUSBPS_PORTSCR_PTC_MASK  0x000F0000 /**< Port Test Control */
#define XUSBPS_PORTSCR_WKCN_MASK 0x00100000 /**< Wake on Connect Enable */
#define XUSBPS_PORTSCR_WKDS_MASK 0x00200000 /**< Wake on Disconnect Enable */
#define XUSBPS_PORTSCR_WKOC_MASK 0x00400000 /**< Wake on Over-current Enable */
#define XUSBPS_PORTSCR_PHCD_MASK 0x00800000 /**< PHY Low Power Suspend - Clock Disable */
#define XUSBPS_PORTSCR_PFSC_MASK 0x01000000 /**< Port Force Full Speed Connect */
#define XUSBPS_PORTSCR_PSPD_MASK 0x0C000000 /**< Port Speed */
/* @} */

/** @name On-The-Go Status Control Register (OTGCSR) bit positions.
 *  @{
 */
#define XUSBPS_OTGSC_VD_MASK    0x00000001 /**< VBus Discharge Bit */
#define XUSBPS_OTGSC_VC_MASK    0x00000002 /**< VBus Charge Bit */
#define XUSBPS_OTGSC_HAAR_MASK  0x00000004 /**< HW Assist Auto Reset Enable Bit */
#define XUSBPS_OTGSC_OT_MASK    0x00000008 /**< OTG Termination Bit */
#define XUSBPS_OTGSC_DP_MASK    0x00000010 /**< Data Pulsing Pull-up Enable Bit */
#define XUSBPS_OTGSC_IDPU_MASK  0x00000020 /**< ID Pull-up Enable Bit */
#define XUSBPS_OTGSC_HADP_MASK  0x00000040 /**< HW Assist Data Pulse Enable Bit */
#define XUSBPS_OTGSC_HABA_MASK  0x00000080 /**< USB Hardware Assist B Disconnect to A Connect Enable Bit */
#define XUSBPS_OTGSC_ID_MASK    0x00000100 /**< ID Status Flag */
#define XUSBPS_OTGSC_AVV_MASK   0x00000200 /**< USB A VBus Valid Interrupt Status Flag */
#define XUSBPS_OTGSC_ASV_MASK   0x00000400 /**< USB A Session Valid Interrupt Status Flag */
#define XUSBPS_OTGSC_BSV_MASK   0x00000800 /**< USB B Session Valid Status Flag */
#define XUSBPS_OTGSC_BSE_MASK   0x00001000 /**< USB B Session End Status Flag */
#define XUSBPS_OTGSC_1MST_MASK  0x00002000 /**< USB 1 Millisecond Timer Status Flag */
#define XUSBPS_OTGSC_DPS_MASK   0x00004000 /**< Data Pulse Status Flag */
#define XUSBPS_OTGSC_IDIS_MASK  0x00010000 /**< USB ID Interrupt Status Flag */
#define XUSBPS_OTGSC_AVVIS_MASK 0x00020000 /**< USB A VBus Valid Interrupt Status Flag */
#define XUSBPS_OTGSC_ASVIS_MASK 0x00040000 /**< USB A Session Valid Interrupt Status Flag */
#define XUSBPS_OTGSC_BSVIS_MASK 0x00080000 /**< USB B Session Valid Interrupt Status Flag */
#define XUSBPS_OTGSC_BSEIS_MASK 0x00100000 /**< USB B Session End Interrupt Status Flag */
#define XUSBPS_OTGSC_1MSS_MASK  0x00200000 /**< 1 Millisecond Timer Interrupt Status Flag */
#define XUSBPS_OTGSC_DPIS_MASK  0x00400000 /**< Data Pulse Interrupt Status Flag */
#define XUSBPS_OTGSC_IDIE_MASK  0x01000000 /**< ID Interrupt Enable Bit */
#define XUSBPS_OTGSC_AVVIE_MASK 0x02000000 /**< USB A VBus Valid Interrupt Enable Bit */
#define XUSBPS_OTGSC_ASVIE_MASK 0x04000000 /**< USB A Session Valid Interrupt Enable Bit */
#define XUSBPS_OTGSC_BSVIE_MASK 0x08000000 /**< USB B Session Valid Interrupt Enable Bit */
#define XUSBPS_OTGSC_BSEE_MASK  0x10000000 /**< USB B Session End Interrupt Enable Bit */
#define XUSBPS_OTGSC_1MSE_MASK  0x20000000 /**< 1 Millisecond Timer Interrupt Enable Bit */
#define XUSBPS_OTGSC_DPIE_MASK  0x40000000 /**< Data Pulse Interrupt Enable Bit */

#define XUSBPS_OTG_ISB_ALL	(XUSBPS_OTGSC_IDIS_MASK |\
				 XUSBPS_OTGSC_AVVIS_MASK | \
				 XUSBPS_OTGSC_ASVIS_MASK | \
				 XUSBPS_OTGSC_BSVIS_MASK | \
				 XUSBPS_OTGSC_BSEIS_MASK | \
				 XUSBPS_OTGSC_1MSS_MASK | \
				 XUSBPS_OTGSC_DPIS_MASK)
/** Mask for All IRQ status masks */

#define XUSBPS_OTG_IEB_ALL	(XUSBPS_OTGSC_IDIE_MASK |\
				 XUSBPS_OTGSC_AVVIE_MASK | \
				 XUSBPS_OTGSC_ASVIE_MASK | \
				 XUSBPS_OTGSC_BSVIE_MASK | \
				 XUSBPS_OTGSC_BSEE_IEB_MASK | \
				 XUSBPS_OTGSC_1MSE_MASK | \
				 XUSBPS_OTGSC_DPIE_MASK)
/** Mask for All IRQ Enable masks */
/* @} */

/**< Alignment of the Device Queue Head List BASE. */
#define XUSBPS_dQH_BASE_ALIGN 2048

/**< Alignment of a Device Queue Head structure. */
#define XUSBPS_dQH_ALIGN 64

/**< Alignment of a Device Transfer Descriptor structure. */
#define XUSBPS_dTD_ALIGN 32

/**< Size of one RX buffer for a OUT Transfer Descriptor. */
#define XUSBPS_dTD_BUF_SIZE 4096

/**< Maximum size of one RX/TX buffer. */
#define XUSBPS_dTD_BUF_MAX_SIZE 16 * 1024

/**< Alignment requirement for Transfer Descriptor buffers. */
#define XUSBPS_dTD_BUF_ALIGN 4096

#endif
