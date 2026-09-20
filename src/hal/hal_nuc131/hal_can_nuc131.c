#include "hal/hal_can.h"
#include "core/ring_buffer.h"
#include "NUC131.h"
#include <string.h>

#define NUC_CAN_RX_RING_SIZE 32
#define CAN_TX_OBJ           1
#define CAN_RX_OBJ_START     2
#define CAN_RX_OBJ_END       32

static can_frame_t s_rx_storage[NUC_CAN_RX_RING_SIZE];
static ring_buffer_t s_can_rx_rb;

// Helper: Busy-wait until Interface register request clears
static inline void can_wait_if1(void) {
    while (CAN->IF1_CREQ & CAN_IF_CREQ_BUSY_Msk);
}

static inline void can_wait_if2(void) {
    while (CAN->IF2_CREQ & CAN_IF_CREQ_BUSY_Msk);
}

hal_status_t hal_can_init(can_baudrate_t baudrate) {
    ring_buffer_init(&s_can_rx_rb, s_rx_storage, sizeof(can_frame_t), NUC_CAN_RX_RING_SIZE);

    SYS_UnlockReg();

    // Enable CAN peripheral clock
    CLK->APBCLK |= CLK_APBCLK_CAN0_EN_Msk;

    // Set Multi-Function Pins: PD.6 -> CAN_RX, PD.7 -> CAN_TX
    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
    SYS->GPD_MFP |= (SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD);

    SYS_LockReg();

    // Enter Initialization and Configuration Change Enable
    CAN->CON |= (CAN_CON_INIT_Msk | CAN_CON_CCE_Msk);

    // Timing calculation (Based on 50 MHz APB clock, 10 Tq: Sync=1, Prop+Phase1=6, Phase2=3)
    // Nominal bit time = 10 Tq. Prescaler = 50 MHz / (10 * Baud)
    uint16_t brp = 0;
    switch (baudrate) {
        case CAN_BAUD_1M:   brp = 5;  break;
        case CAN_BAUD_500K: brp = 10; break;
        case CAN_BAUD_250K: brp = 20; break;
        case CAN_BAUD_125K: brp = 40; break;
        default:            brp = 10; break;
    }

    uint8_t tseg1 = 6;
    uint8_t tseg2 = 3;
    uint8_t sjw   = 1;

    CAN->BTIME = ((sjw - 1) << 6) |
                 ((tseg2 - 1) << 12) |
                 ((tseg1 - 1) << 8) |
                 ((brp - 1) & 0x3F);

    CAN->BRPE = ((brp - 1) >> 6) & 0x0F;

    // Clear all 32 message objects using IF1
    can_wait_if1();
    CAN->IF1_CMASK = CAN_IF_CMASK_WR_Msk | CAN_IF_CMASK_MASK_Msk | 
                     CAN_IF_CMASK_ARB_Msk | CAN_IF_CMASK_CONTROL_Msk;
    CAN->IF1_MASK1 = 0;
    CAN->IF1_MASK2 = 0;
    CAN->IF1_ARB1  = 0;
    CAN->IF1_ARB2  = 0; // MsgVal = 0 (invalid)
    CAN->IF1_MCON  = 0;

    for (int i = 1; i <= 32; i++) {
        CAN->IF1_CREQ = i;
        can_wait_if1();
    }

    // Configure Message Object 1 for Transmission
    CAN->IF1_CMASK = CAN_IF_CMASK_WR_Msk | CAN_IF_CMASK_ARB_Msk | CAN_IF_CMASK_CONTROL_Msk;
    CAN->IF1_ARB2  = CAN_IF_ARB2_DIR_Msk; // Transmit direction, initially invalid
    CAN->IF1_MCON  = CAN_IF_MCON_TXIE_Msk | CAN_IF_MCON_EOB_Msk;
    CAN->IF1_CREQ  = CAN_TX_OBJ;
    can_wait_if1();

    // Configure Message Objects 2..32 for Reception (FIFO match-all)
    CAN->IF1_CMASK = CAN_IF_CMASK_WR_Msk | CAN_IF_CMASK_MASK_Msk | 
                     CAN_IF_CMASK_ARB_Msk | CAN_IF_CMASK_CONTROL_Msk;
    CAN->IF1_MASK1 = 0;
    CAN->IF1_MASK2 = 0; // Accept all masks
    CAN->IF1_ARB1  = 0;
    CAN->IF1_ARB2  = CAN_IF_ARB2_MSGVAL_Msk; // Valid RX, DIR = 0

    for (int i = CAN_RX_OBJ_START; i <= CAN_RX_OBJ_END; i++) {
        CAN->IF1_MCON = CAN_IF_MCON_RXIE_Msk | CAN_IF_MCON_UMASK_Msk | 
                        ((i == CAN_RX_OBJ_END) ? CAN_IF_MCON_EOB_Msk : 0);
        CAN->IF1_CREQ = i;
        can_wait_if1();
    }

    // Leave Init mode and enable global interrupt
    CAN->CON &= ~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk);
    CAN->CON |= (CAN_CON_IE_Msk | CAN_CON_EIE_Msk);

    NVIC_EnableIRQ(CAN0_IRQn);
    return HAL_OK;
}

hal_status_t hal_can_set_filters(const can_filter_t *filters, uint8_t count) {
    (void)filters;
    (void)count;
    return HAL_OK;
}

hal_status_t hal_can_send(const can_frame_t *frame) {
    if (!frame) return HAL_ERROR;

    // Check if Object 1 is busy transmitting
    if (CAN->TXREQ1 & (1U << (CAN_TX_OBJ - 1))) {
        return HAL_BUSY;
    }

    can_wait_if1();

    CAN->IF1_CMASK = CAN_IF_CMASK_WR_Msk | CAN_IF_CMASK_ARB_Msk | 
                     CAN_IF_CMASK_CONTROL_Msk | CAN_IF_CMASK_DAT_A_Msk | 
                     CAN_IF_CMASK_DAT_B_Msk;

    if (frame->is_extended) {
        CAN->IF1_ARB1 = (uint16_t)(frame->id & 0xFFFF);
        CAN->IF1_ARB2 = (uint16_t)(((frame->id >> 16) & 0x1FFF) | 
                                   CAN_IF_ARB2_XTD_Msk | 
                                   CAN_IF_ARB2_DIR_Msk | 
                                   CAN_IF_ARB2_MSGVAL_Msk);
    } else {
        CAN->IF1_ARB1 = 0;
        CAN->IF1_ARB2 = (uint16_t)(((frame->id & 0x7FF) << 2) | 
                                   CAN_IF_ARB2_DIR_Msk | 
                                   CAN_IF_ARB2_MSGVAL_Msk);
    }

    CAN->IF1_MCON = CAN_IF_MCON_TXRQST_Msk | CAN_IF_MCON_EOB_Msk | (frame->dlc & 0x0F);

    // Pack 16-bit register slices
    CAN->IF1_DAT_A1 = ((uint16_t)frame->data[1] << 8) | frame->data[0];
    CAN->IF1_DAT_A2 = ((uint16_t)frame->data[3] << 8) | frame->data[2];
    CAN->IF1_DAT_B1 = ((uint16_t)frame->data[5] << 8) | frame->data[4];
    CAN->IF1_DAT_B2 = ((uint16_t)frame->data[7] << 8) | frame->data[6];

    // Transfer shadow registers to Object 1
    CAN->IF1_CREQ = CAN_TX_OBJ;
    return HAL_OK;
}

hal_status_t hal_can_receive(can_frame_t *frame) {
    if (!frame) return HAL_ERROR;
    return ring_buffer_pop(&s_can_rx_rb, frame) ? HAL_OK : HAL_TIMEOUT;
}

void CAN0_IRQHandler(void) {
    uint32_t status = CAN->IIDR;

    if (status >= CAN_RX_OBJ_START && status <= CAN_RX_OBJ_END) {
        uint8_t obj_num = (uint8_t)status;

        // Command read via IF2
        can_wait_if2();
        CAN->IF2_CMASK = CAN_IF_CMASK_ARB_Msk | CAN_IF_CMASK_CONTROL_Msk | 
                         CAN_IF_CMASK_CLRINTPND_Msk | CAN_IF_CMASK_DAT_A_Msk | 
                         CAN_IF_CMASK_DAT_B_Msk;
        CAN->IF2_CREQ = obj_num;
        can_wait_if2();

        if (CAN->IF2_MCON & CAN_IF_MCON_NEWDAT_Msk) {
            can_frame_t rx;
            rx.is_extended = (CAN->IF2_ARB2 & CAN_IF_ARB2_XTD_Msk) != 0;
            rx.is_remote   = (CAN->IF2_ARB2 & CAN_IF_ARB2_DIR_Msk) != 0;

            if (rx.is_extended) {
                rx.id = (((uint32_t)(CAN->IF2_ARB2 & 0x1FFF)) << 16) | CAN->IF2_ARB1;
            } else {
                rx.id = (CAN->IF2_ARB2 >> 2) & 0x7FF;
            }

            rx.dlc = CAN->IF2_MCON & 0x0F;

            uint16_t da1 = CAN->IF2_DAT_A1;
            uint16_t da2 = CAN->IF2_DAT_A2;
            uint16_t db1 = CAN->IF2_DAT_B1;
            uint16_t db2 = CAN->IF2_DAT_B2;

            rx.data[0] = (uint8_t)(da1);
            rx.data[1] = (uint8_t)(da1 >> 8);
            rx.data[2] = (uint8_t)(da2);
            rx.data[3] = (uint8_t)(da2 >> 8);
            rx.data[4] = (uint8_t)(db1);
            rx.data[5] = (uint8_t)(db1 >> 8);
            rx.data[6] = (uint8_t)(db2);
            rx.data[7] = (uint8_t)(db2 >> 8);
            rx.timestamp_ms = hal_get_tick_ms();

            ring_buffer_push(&s_can_rx_rb, &rx);
        }
    } else if (status == CAN_TX_OBJ) {
        // Clear TX complete interrupt flag on Object 1
        can_wait_if1();
        CAN->IF1_CMASK = CAN_IF_CMASK_CLRINTPND_Msk;
        CAN->IF1_CREQ = CAN_TX_OBJ;
    }
}
