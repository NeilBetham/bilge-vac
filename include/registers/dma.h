/**
 * @brief DMA registers
 */

#include "registers/helpers.h"

#pragma once


// Base addresses
#define DMA_1_BASE   0x40020000
#define DMA_2_BASE   0x40020400
#define DMA_MUX_BASE 0x40020800


// Register offsets
#define DMA_ISR_OFFSET    0x00000000
#define DMA_IFCR_OFFSET   0x00000004
#define DMA_CCR1_OFFSET   0x00000008
#define DMA_CNDTR1_OFFSET 0x0000000C
#define DMA_CPAR1_OFFSET  0x00000010
#define DMA_CMAR1_OFFSET  0x00000014
#define DMA_CCR2_OFFSET   0x0000001C
#define DMA_CNDTR2_OFFSET 0x00000020
#define DMA_CPAR2_OFFSET  0x00000024
#define DMA_CMAR2_OFFSET  0x00000028
#define DMA_CCR3_OFFSET   0x00000030
#define DMA_CNDTR3_OFFSET 0x00000034
#define DMA_CPAR3_OFFSET  0x00000038
#define DMA_CMAR3_OFFSET  0x0000003C
#define DMA_CCR4_OFFSET   0x00000044
#define DMA_CNDTR4_OFFSET 0x00000048
#define DMA_CPAR4_OFFSET  0x0000004C
#define DMA_CMAR4_OFFSET  0x00000050
#define DMA_CCR5_OFFSET   0x00000058
#define DMA_CNDTR5_OFFSET 0x0000005C
#define DMA_CPAR5_OFFSET  0x00000060
#define DMA_CMAR5_OFFSET  0x00000064
#define DMA_CCR6_OFFSET   0x0000006C
#define DMA_CNDTR6_OFFSET 0x00000070
#define DMA_CPAR6_OFFSET  0x00000074
#define DMA_CMAR6_OFFSET  0x00000078
#define DMA_CCR7_OFFSET   0x00000080
#define DMA_CNDTR7_OFFSET 0x00000084
#define DMA_CPAR7_OFFSET  0x00000088
#define DMA_CMAR7_OFFSET  0x0000008C

#define DMA_MUX_C0CR_OFFSET  0x00000000
#define DMA_MUX_C1CR_OFFSET  0x00000004
#define DMA_MUX_C2CR_OFFSET  0x00000008
#define DMA_MUX_C3CR_OFFSET  0x0000000C
#define DMA_MUX_C4CR_OFFSET  0x00000010
#define DMA_MUX_C5CR_OFFSET  0x00000014
#define DMA_MUX_C6CR_OFFSET  0x00000018
#define DMA_MUX_C7CR_OFFSET  0x0000001C
#define DMA_MUX_C8CR_OFFSET  0x00000020
#define DMA_MUX_C9CR_OFFSET  0x00000024
#define DMA_MUX_C10CR_OFFSET 0x00000028
#define DMA_MUX_C11CR_OFFSET 0x0000002C
#define DMA_MUX_CSR_OFFSET   0x00000080
#define DMA_MUX_CFR_OFFSET   0x00000084
#define DMA_MUX_RG0CR_OFFSET 0x00000100
#define DMA_MUX_RG1CR_OFFSET 0x00000104
#define DMA_MUX_RG2CR_OFFSET 0x00000108
#define DMA_MUX_RG3CR_OFFSET 0x0000010C
#define DMA_MUX_RGSR_OFFSET  0x00000140
#define DMA_MUX_RGCFR_OFFSET 0x00000144


// DMA 1
#define DMA_1_ISR    REGISTER(DMA_1_BASE + DMA_ISR_OFFSET)
#define DMA_1_IFCR   REGISTER(DMA_1_BASE + DMA_IFCR_OFFSET)
#define DMA_1_CCR1   REGISTER(DMA_1_BASE + DMA_CCR1_OFFSET)
#define DMA_1_CNDTR1 REGISTER(DMA_1_BASE + DMA_CNDTR1_OFFSET)
#define DMA_1_CPAR1  REGISTER(DMA_1_BASE + DMA_CPAR1_OFFSET)
#define DMA_1_CMAR1  REGISTER(DMA_1_BASE + DMA_CMAR1_OFFSET)
#define DMA_1_CCR2   REGISTER(DMA_1_BASE + DMA_CCR2_OFFSET)
#define DMA_1_CNDTR2 REGISTER(DMA_1_BASE + DMA_CNDTR2_OFFSET)
#define DMA_1_CPAR2  REGISTER(DMA_1_BASE + DMA_CPAR2_OFFSET)
#define DMA_1_CMAR2  REGISTER(DMA_1_BASE + DMA_CMAR2_OFFSET)
#define DMA_1_CCR3   REGISTER(DMA_1_BASE + DMA_CCR3_OFFSET)
#define DMA_1_CNDTR3 REGISTER(DMA_1_BASE + DMA_CNDTR3_OFFSET)
#define DMA_1_CPAR3  REGISTER(DMA_1_BASE + DMA_CPAR3_OFFSET)
#define DMA_1_CMAR3  REGISTER(DMA_1_BASE + DMA_CMAR3_OFFSET)
#define DMA_1_CCR4   REGISTER(DMA_1_BASE + DMA_CCR4_OFFSET)
#define DMA_1_CNDTR4 REGISTER(DMA_1_BASE + DMA_CNDTR4_OFFSET)
#define DMA_1_CPAR4  REGISTER(DMA_1_BASE + DMA_CPAR4_OFFSET)
#define DMA_1_CMAR4  REGISTER(DMA_1_BASE + DMA_CMAR4_OFFSET)
#define DMA_1_CCR5   REGISTER(DMA_1_BASE + DMA_CCR5_OFFSET)
#define DMA_1_CNDTR5 REGISTER(DMA_1_BASE + DMA_CNDTR5_OFFSET)
#define DMA_1_CPAR5  REGISTER(DMA_1_BASE + DMA_CPAR5_OFFSET)
#define DMA_1_CMAR5  REGISTER(DMA_1_BASE + DMA_CMAR5_OFFSET)
#define DMA_1_CCR6   REGISTER(DMA_1_BASE + DMA_CCR6_OFFSET)
#define DMA_1_CNDTR6 REGISTER(DMA_1_BASE + DMA_CNDTR6_OFFSET)
#define DMA_1_CPAR6  REGISTER(DMA_1_BASE + DMA_CPAR6_OFFSET)
#define DMA_1_CMAR6  REGISTER(DMA_1_BASE + DMA_CMAR6_OFFSET)
#define DMA_1_CCR7   REGISTER(DMA_1_BASE + DMA_CCR7_OFFSET)
#define DMA_1_CNDTR7 REGISTER(DMA_1_BASE + DMA_CNDTR7_OFFSET)
#define DMA_1_CPAR7  REGISTER(DMA_1_BASE + DMA_CPAR7_OFFSET)
#define DMA_1_CMAR7  REGISTER(DMA_1_BASE + DMA_CMAR7_OFFSET)


// DMA 2
#define DMA_2_ISR    REGISTER(DMA_2_BASE + DMA_ISR_OFFSET)
#define DMA_2_IFCR   REGISTER(DMA_2_BASE + DMA_IFCR_OFFSET)
#define DMA_2_CCR1   REGISTER(DMA_2_BASE + DMA_CCR1_OFFSET)
#define DMA_2_CNDTR1 REGISTER(DMA_2_BASE + DMA_CNDTR1_OFFSET)
#define DMA_2_CPAR1  REGISTER(DMA_2_BASE + DMA_CPAR1_OFFSET)
#define DMA_2_CMAR1  REGISTER(DMA_2_BASE + DMA_CMAR1_OFFSET)
#define DMA_2_CCR2   REGISTER(DMA_2_BASE + DMA_CCR2_OFFSET)
#define DMA_2_CNDTR2 REGISTER(DMA_2_BASE + DMA_CNDTR2_OFFSET)
#define DMA_2_CPAR2  REGISTER(DMA_2_BASE + DMA_CPAR2_OFFSET)
#define DMA_2_CMAR2  REGISTER(DMA_2_BASE + DMA_CMAR2_OFFSET)
#define DMA_2_CCR3   REGISTER(DMA_2_BASE + DMA_CCR3_OFFSET)
#define DMA_2_CNDTR3 REGISTER(DMA_2_BASE + DMA_CNDTR3_OFFSET)
#define DMA_2_CPAR3  REGISTER(DMA_2_BASE + DMA_CPAR3_OFFSET)
#define DMA_2_CMAR3  REGISTER(DMA_2_BASE + DMA_CMAR3_OFFSET)
#define DMA_2_CCR4   REGISTER(DMA_2_BASE + DMA_CCR4_OFFSET)
#define DMA_2_CNDTR4 REGISTER(DMA_2_BASE + DMA_CNDTR4_OFFSET)
#define DMA_2_CPAR4  REGISTER(DMA_2_BASE + DMA_CPAR4_OFFSET)
#define DMA_2_CMAR4  REGISTER(DMA_2_BASE + DMA_CMAR4_OFFSET)
#define DMA_2_CCR5   REGISTER(DMA_2_BASE + DMA_CCR5_OFFSET)
#define DMA_2_CNDTR5 REGISTER(DMA_2_BASE + DMA_CNDTR5_OFFSET)
#define DMA_2_CPAR5  REGISTER(DMA_2_BASE + DMA_CPAR5_OFFSET)
#define DMA_2_CMAR5  REGISTER(DMA_2_BASE + DMA_CMAR5_OFFSET)
#define DMA_2_CCR6   REGISTER(DMA_2_BASE + DMA_CCR6_OFFSET)
#define DMA_2_CNDTR6 REGISTER(DMA_2_BASE + DMA_CNDTR6_OFFSET)
#define DMA_2_CPAR6  REGISTER(DMA_2_BASE + DMA_CPAR6_OFFSET)
#define DMA_2_CMAR6  REGISTER(DMA_2_BASE + DMA_CMAR6_OFFSET)
#define DMA_2_CCR7   REGISTER(DMA_2_BASE + DMA_CCR7_OFFSET)
#define DMA_2_CNDTR7 REGISTER(DMA_2_BASE + DMA_CNDTR7_OFFSET)
#define DMA_2_CPAR7  REGISTER(DMA_2_BASE + DMA_CPAR7_OFFSET)
#define DMA_2_CMAR7  REGISTER(DMA_2_BASE + DMA_CMAR7_OFFSET)

// DMA MUX
#define DMA_MUX_C0CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C0CR_OFFSET)
#define DMA_MUX_C1CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C1CR_OFFSET)
#define DMA_MUX_C2CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C2CR_OFFSET)
#define DMA_MUX_C3CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C3CR_OFFSET)
#define DMA_MUX_C4CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C4CR_OFFSET)
#define DMA_MUX_C5CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C5CR_OFFSET)
#define DMA_MUX_C6CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C6CR_OFFSET)
#define DMA_MUX_C7CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C7CR_OFFSET)
#define DMA_MUX_C8CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C8CR_OFFSET)
#define DMA_MUX_C9CR  REGISTER(DMA_MUX_BASE + DMA_MUX_C9CR_OFFSET)
#define DMA_MUX_C10CR REGISTER(DMA_MUX_BASE + DMA_MUX_C10CR_OFFSET)
#define DMA_MUX_C11CR REGISTER(DMA_MUX_BASE + DMA_MUX_C11CR_OFFSET)
#define DMA_MUX_CSR   REGISTER(DMA_MUX_BASE + DMA_MUX_CSR_OFFSET)
#define DMA_MUX_CFR   REGISTER(DMA_MUX_BASE + DMA_MUX_CFR_OFFSET)
#define DMA_MUX_RG0CR REGISTER(DMA_MUX_BASE + DMA_MUX_RG0CR_OFFSET)
#define DMA_MUX_RG1CR REGISTER(DMA_MUX_BASE + DMA_MUX_RG1CR_OFFSET)
#define DMA_MUX_RG2CR REGISTER(DMA_MUX_BASE + DMA_MUX_RG2CR_OFFSET)
#define DMA_MUX_RG3CR REGISTER(DMA_MUX_BASE + DMA_MUX_RG3CR_OFFSET)
#define DMA_MUX_RGSR  REGISTER(DMA_MUX_BASE + DMA_MUX_RGSR_OFFSET)
#define DMA_MUX_RGCFR REGISTER(DMA_MUX_BASE + DMA_MUX_RGCFR_OFFSET)


