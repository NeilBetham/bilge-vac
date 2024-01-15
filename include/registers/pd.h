/**
 * @brief PD PHY Registers
 */

#include "registers/helpers.h"

#pragma once

#define PD1_BASE 0x4000A000ul
#define PD2_BASE 0x4000A400ul

#define PD_CFG1_OFFSET       0x00000000ul
#define PD_CFG2_OFFSET       0x00000004ul
#define PD_CFG3_OFFSET       0x00000008ul
#define PD_CR_OFFSET         0x0000000Cul
#define PD_IMR_OFFSET        0x00000010ul
#define PD_SR_OFFSET         0x00000014ul
#define PD_ICR_OFFSET        0x00000018ul
#define PD_TX_ORDSET_OFFSET  0x0000001Cul
#define PD_TX_PAYSZ_OFFSET   0x00000020ul
#define PD_TXDR_OFFSET       0x00000024ul
#define PD_RX_ORDSET_OFFSET  0x00000028ul
#define PD_RX_PAYSZ_OFFSET   0x0000002Cul
#define PD_RXDR_OFFSET       0x00000030ul
#define PD_RX_ORDEXT1_OFFSET 0x00000034ul
#define PD_RX_ORDEXT2_OFFSET 0x00000038ul


#define PD1_CFG1       REGISTER(PD1_BASE + PD_CFG1_OFFSET)
#define PD1_CFG2       REGISTER(PD1_BASE + PD_CFG2_OFFSET)
#define PD1_CFG3       REGISTER(PD1_BASE + PD_CFG3_OFFSET)
#define PD1_CR         REGISTER(PD1_BASE + PD_CR_OFFSET)
#define PD1_IMR        REGISTER(PD1_BASE + PD_IMR_OFFSET)
#define PD1_SR         REGISTER(PD1_BASE + PD_SR_OFFSET)
#define PD1_ICR        REGISTER(PD1_BASE + PD_ICR_OFFSET)
#define PD1_TX_ORDSET  REGISTER(PD1_BASE + PD_TX_ORDSET_OFFSET)
#define PD1_TX_PAYSZ   REGISTER(PD1_BASE + PD_TX_PAYSZ_OFFSET)
#define PD1_TXDR       REGISTER(PD1_BASE + PD_TXDR_OFFSET)
#define PD1_RX_ORDSET  REGISTER(PD1_BASE + PD_RX_ORDSET_OFFSET)
#define PD1_RX_PAYSZ   REGISTER(PD1_BASE + PD_RX_PAYSZ_OFFSET)
#define PD1_RXDR       REGISTER(PD1_BASE + PD_RXDR_OFFSET)
#define PD1_RX_ORDEXT1 REGISTER(PD1_BASE + PD_RX_ORDEXT1_OFFSET)
#define PD1_RX_ORDEXT2 REGISTER(PD1_BASE + PD_RX_ORDEXT2_OFFSET)


#define PD2_CFG1       REGISTER(PD2_BASE + PD_CFG1_OFFSET)
#define PD2_CFG2       REGISTER(PD2_BASE + PD_CFG2_OFFSET)
#define PD2_CFG3       REGISTER(PD2_BASE + PD_CFG3_OFFSET)
#define PD2_CR         REGISTER(PD2_BASE + PD_CR_OFFSET)
#define PD2_IMR        REGISTER(PD2_BASE + PD_IMR_OFFSET)
#define PD2_SR         REGISTER(PD2_BASE + PD_SR_OFFSET)
#define PD2_ICR        REGISTER(PD2_BASE + PD_ICR_OFFSET)
#define PD2_TX_ORDSET  REGISTER(PD2_BASE + PD_TX_ORDSET_OFFSET)
#define PD2_TX_PAYSZ   REGISTER(PD2_BASE + PD_TX_PAYSZ_OFFSET)
#define PD2_TXDR       REGISTER(PD2_BASE + PD_TXDR_OFFSET)
#define PD2_RX_ORDSET  REGISTER(PD2_BASE + PD_RX_ORDSET_OFFSET)
#define PD2_RX_PAYSZ   REGISTER(PD2_BASE + PD_RX_PAYSZ_OFFSET)
#define PD2_RXDR       REGISTER(PD2_BASE + PD_RXDR_OFFSET)
#define PD2_RX_ORDEXT1 REGISTER(PD2_BASE + PD_RX_ORDEXT1_OFFSET)
#define PD2_RX_ORDEXT2 REGISTER(PD2_BASE + PD_RX_ORDEXT2_OFFSET)
