#include "hal/hal_can.h"
#include "core/ring_buffer.h"
#include "core/can_router.h"
#include "NUC131.h"
#include "can.h"
#include <string.h>

/* Increased from 32 to 64 to absorb full vehicle CAN bursts during blocking UART TX */
#define NUC_CAN_RX_RING_SIZE 64
#define CAN_TX_OBJ_START     0u
#define CAN_TX_OBJ_END       5u
#define CAN_RX_OBJ_START     6u
#define CAN_RX_OBJ_END       31u
#define CAN_WAIT_TIMEOUT     10000u

static can_frame_t s_rx_storage[NUC_CAN_RX_RING_SIZE];
static ring_buffer_t s_can_rx_rb;

/* Shared with hal_system_nuc131.c so ISR flag clearing before WFI return never loses CAN wake state */
volatile bool g_nuc131_can_woke_flag = false;

/* Throttled Bus-Off recovery state */
static volatile bool s_bus_off_pending = false;
static uint32_t s_last_boff_recovery_ms = 0;

// Function implemented in Nuvoton CAN BSP driver
int32_t CAN_SetRxMsgObjAndMsk(CAN_T *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint32_t u32idmask, uint8_t u8singleOrFifoLast);
void CAN_EnterInitMode(CAN_T *tCAN);
void CAN_LeaveInitMode(CAN_T *tCAN);

// Helper: Bounded wait until Interface register request clears
static inline void can_wait_if(CAN_T *can, uint8_t iface) {
    uint32_t timeout = CAN_WAIT_TIMEOUT;
    while ((can->IF[iface & 1].CREQ & CAN_IF_CREQ_BUSY_Msk) && --timeout);
}

hal_status_t hal_can_init(can_baudrate_t baudrate) {
    ring_buffer_init(&s_can_rx_rb, s_rx_storage, sizeof(can_frame_t), NUC_CAN_RX_RING_SIZE);
    s_bus_off_pending = false;
    s_last_boff_recovery_ms = 0;

    SYS_UnlockReg();

    // Enable CAN peripheral clock
    CLK_EnableModuleClock(CAN0_MODULE);

    // Set Multi-Function Pins: PD.6 -> CAN_RX, PD.7 -> CAN_TX
    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
    SYS->GPD_MFP |= (SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD);

    SYS_LockReg();

    // Reset the CAN module
    SYS_ResetModule(CAN0_RST);

    // Directly configure CAN bit timing (bypassing dynamic CAN_Open loop)
    CAN_EnterInitMode(CAN0);
    if (baudrate == CAN_BAUD_125K) {
        // 48 MHz PCLK: 125 kbps, 81.25% sample point
        CAN0->BTIME = 0x2B17;
        CAN0->BRPE  = 0;
    } else if (baudrate == CAN_BAUD_500K) {
        // 48 MHz PCLK: 500 kbps, 81.25% sample point: tsegall=16, BRP=6, TSEG1=12, TSEG2=3
        CAN0->BTIME = 0x2B05;
        CAN0->BRPE  = 0;
    } else {
        // Default / 125 kbps
        CAN0->BTIME = 0x2B17;
        CAN0->BRPE  = 0;
    }
    CAN_LeaveInitMode(CAN0);

    // Set RX message objects (6..31) as a hardware FIFO to accept standard frames
    for (uint8_t i = CAN_RX_OBJ_START; i <= CAN_RX_OBJ_END; i++) {
        uint8_t is_last = (i == CAN_RX_OBJ_END) ? TRUE : FALSE;
        CAN_SetRxMsgObjAndMsk(CAN0, i, CAN_STD_ID, 0x0, 0x0, is_last);
        can_wait_if(CAN0, 0);
        can_wait_if(CAN0, 1);
    }

    // Enable interrupts: Message object interrupt + Error Warning/Bus-Off status interrupt
    CAN_EnableInt(CAN0, CAN_CON_IE_Msk | CAN_CON_EIE_Msk);
    NVIC_SetPriority(CAN0_IRQn, 0);
    NVIC_EnableIRQ(CAN0_IRQn);

    return HAL_STATUS_OK;
}

hal_status_t hal_can_set_filters(const can_filter_t *filters, uint8_t count) {
    (void)filters;
    (void)count;
    return HAL_STATUS_OK;
}

hal_status_t hal_can_send(const can_frame_t *frame) {
    if (!frame) return HAL_STATUS_ERROR;

    // Rate-limited recovery from Bus-Off if controller entered INIT state due to error counter overflow
    if ((CAN0->CON & CAN_CON_INIT_Msk) || s_bus_off_pending) {
        uint32_t now = hal_get_tick_ms();
        if ((now - s_last_boff_recovery_ms) >= 200u) {
            s_last_boff_recovery_ms = now;
            s_bus_off_pending = false;
            CAN_LeaveInitMode(CAN0);
        } else {
            return HAL_STATUS_BUSY;
        }
    }

    uint8_t dlc = frame->dlc & 0x0Fu;
    if (dlc > 8u) dlc = 8u;

    /* Protect C-CAN IF[0] access against preemption by CAN0_IRQHandler */
    NVIC_DisableIRQ(CAN0_IRQn);

    uint32_t txreq = CAN0->TXREQ1;
    for (uint8_t obj = CAN_TX_OBJ_START; obj <= CAN_TX_OBJ_END; obj++) {
        if ((txreq & (1u << obj)) == 0u) {
            can_wait_if(CAN0, 0);

            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;

            /* Write Mask, Arb, Control, TxRqst/NewDat, Data A, Data B simultaneously (0xF7) */
            CAN0->IF[0].CMASK = CAN_IF_CMASK_WRRD_Msk | CAN_IF_CMASK_MASK_Msk |
                                CAN_IF_CMASK_ARB_Msk | CAN_IF_CMASK_CONTROL_Msk |
                                CAN_IF_CMASK_TXRQSTNEWDAT_Msk |
                                CAN_IF_CMASK_DATAA_Msk | CAN_IF_CMASK_DATAB_Msk;

            if (frame->is_extended) {
                CAN0->IF[0].ARB1 = frame->id & 0xFFFFu;
                CAN0->IF[0].ARB2 = ((frame->id & 0x1FFF0000u) >> 16) |
                                   CAN_IF_ARB2_XTD_Msk | CAN_IF_ARB2_MSGVAL_Msk |
                                   (frame->is_remote ? 0u : CAN_IF_ARB2_DIR_Msk);
            } else {
                CAN0->IF[0].ARB1 = 0u;
                CAN0->IF[0].ARB2 = ((frame->id & 0x7FFu) << 2) |
                                   CAN_IF_ARB2_MSGVAL_Msk |
                                   (frame->is_remote ? 0u : CAN_IF_ARB2_DIR_Msk);
            }

            CAN0->IF[0].DAT_A1 = ((uint16_t)frame->data[1] << 8) | frame->data[0];
            CAN0->IF[0].DAT_A2 = ((uint16_t)frame->data[3] << 8) | frame->data[2];
            CAN0->IF[0].DAT_B1 = ((uint16_t)frame->data[5] << 8) | frame->data[4];
            CAN0->IF[0].DAT_B2 = ((uint16_t)frame->data[7] << 8) | frame->data[6];

            /* Configure TX object WITHOUT TXIE (polled via TXREQ1, avoids TX interrupt storms) */
            CAN0->IF[0].MCON = CAN_IF_MCON_NEWDAT_Msk | CAN_IF_MCON_TXRQST_Msk |
                               CAN_IF_MCON_EOB_Msk | dlc;
            CAN0->IF[0].CREQ = 1u + obj;

            can_wait_if(CAN0, 0);
            NVIC_EnableIRQ(CAN0_IRQn);
            return HAL_STATUS_OK;
        }
    }

    NVIC_EnableIRQ(CAN0_IRQn);
    return HAL_STATUS_BUSY;
}

hal_status_t hal_can_receive(can_frame_t *frame) {
    if (!frame) return HAL_STATUS_ERROR;

    /* Also service pending Bus-Off recovery in RX polling path if no TX is occurring */
    if (s_bus_off_pending || (CAN0->CON & CAN_CON_INIT_Msk)) {
        uint32_t now = hal_get_tick_ms();
        if ((now - s_last_boff_recovery_ms) >= 200u) {
            s_last_boff_recovery_ms = now;
            s_bus_off_pending = false;
            CAN_LeaveInitMode(CAN0);
        }
    }

    return ring_buffer_pop(&s_can_rx_rb, frame) ? HAL_STATUS_OK : HAL_STATUS_TIMEOUT;
}

void CAN0_IRQHandler(void) {
    // Latch and clear hardware CAN wake-up status flag if woke from sleep
    if (CAN0->WU_STATUS & CAN_WUSTATUS_WAKUP_STS_Msk) {
        CAN0->WU_STATUS = 0;
        g_nuc131_can_woke_flag = true;
    }

    /* Drain all pending interrupt sources in a loop so back-to-back FIFO objects are never stranded */
    uint32_t u32IIDRstatus;
    uint8_t max_iter = 36u;
    while (((u32IIDRstatus = CAN0->IIDR) != 0u) && max_iter--) {
        if (u32IIDRstatus == 0x8000u) {
            uint32_t sts = CAN0->STATUS;
            if (sts & CAN_STATUS_BOFF_Msk) {
                /* Defer Bus-Off recovery to rate-limited main-loop check to prevent ISR storm on shorted bus */
                s_bus_off_pending = true;
            }
        } else if (u32IIDRstatus <= 32u) {
            uint8_t obj_idx = (uint8_t)(u32IIDRstatus - 1u);
            if (obj_idx >= CAN_RX_OBJ_START) {
                STR_CANMSG_T rx_msg;
                CAN_Receive(CAN0, obj_idx, &rx_msg);

                can_frame_t frame;
                frame.id = rx_msg.Id;
                frame.dlc = rx_msg.DLC;
                frame.is_extended = (rx_msg.IdType == CAN_EXT_ID);
                frame.is_remote = (rx_msg.FrameType == CAN_REMOTE_FRAME);
                frame.timestamp_ms = hal_get_tick_ms();
                memcpy(frame.data, rx_msg.Data, 8);

                ring_buffer_push(&s_can_rx_rb, &frame);
            } else {
                /* Clear any unexpected pending bit on TX objects using IF[1] */
                can_wait_if(CAN0, 1);
                CAN0->IF[1].CMASK = CAN_IF_CMASK_CLRINTPND_Msk | CAN_IF_CMASK_TXRQSTNEWDAT_Msk;
                CAN0->IF[1].CREQ = 1u + obj_idx;
                can_wait_if(CAN0, 1);
            }
        }
    }
}
