#include "hal/hal_can.h"
#include "core/ring_buffer.h"
#include "NUC131.h"
#include <string.h>

#define NUC_CAN_RX_RING_SIZE 32
#define CAN_TX_OBJ           0u
#define CAN_RX_OBJ_START     1u
#define CAN_RX_OBJ_END       31u

static can_frame_t s_rx_storage[NUC_CAN_RX_RING_SIZE];
static ring_buffer_t s_can_rx_rb;

// Function implemented in Nuvoton CAN BSP driver but omitted from can.h
int32_t CAN_SetRxMsgObjAndMsk(CAN_T *tCAN, uint8_t u8MsgObj, uint8_t u8idType, uint32_t u32id, uint32_t u32idmask, uint8_t u8singleOrFifoLast);

// Helper: Busy-wait until Interface register request clears
static inline void can_wait_if(CAN_T *can, uint8_t iface) {
    while (can->IF[iface & 1].CREQ & CAN_IF_CREQ_BUSY_Msk);
}

hal_status_t hal_can_init(can_baudrate_t baudrate) {
    ring_buffer_init(&s_can_rx_rb, s_rx_storage, sizeof(can_frame_t), NUC_CAN_RX_RING_SIZE);

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

    // Set all RX message objects to accept all frames
    for (uint8_t i = CAN_RX_OBJ_START; i <= CAN_RX_OBJ_END; i++) {
        // Standard CAN ID, accept all (mask = 0)
        CAN_SetRxMsgObjAndMsk(CAN0, i, CAN_STD_ID, 0x0, 0x0, FALSE);
    }

    // Enable interrupts: Message object interrupt only
    CAN_EnableInt(CAN0, CAN_CON_IE_Msk);
    NVIC_SetPriority(CAN0_IRQn, 0);
    NVIC_EnableIRQ(CAN0_IRQn);

    return HAL_STATUS_OK;
}

hal_status_t hal_can_set_filters(const can_filter_t *filters, uint8_t count) {
    // NUC131 reference implementation does not implement per-filter masking.
    // All message objects are set to accept all frames at init time.
    // TODO: Implement proper filter configuration if needed for production use.
    (void)filters;
    (void)count;
    return HAL_STATUS_OK;
}

hal_status_t hal_can_send(const can_frame_t *frame) {
    if (!frame) return HAL_STATUS_ERROR;

    // Use message object 0 for transmission
    STR_CANMSG_T msg;
    msg.Id = frame->id;
    msg.DLC = frame->dlc & 0x0Fu;
    msg.IdType = frame->is_extended ? CAN_EXT_ID : CAN_STD_ID;
    memcpy(msg.Data, frame->data, 8);

    // Check if transmit object is busy
    if (CAN0->TXREQ1 & (1u << CAN_TX_OBJ)) {
        return HAL_STATUS_BUSY;
    }

    // Transmit the frame
    CAN_Transmit(CAN0, CAN_TX_OBJ, &msg);
    return HAL_STATUS_OK;
}

hal_status_t hal_can_receive(can_frame_t *frame) {
    if (!frame) return HAL_STATUS_ERROR;
    return ring_buffer_pop(&s_can_rx_rb, frame) ? HAL_STATUS_OK : HAL_STATUS_TIMEOUT;
}

void CAN0_IRQHandler(void) {
    // Clear hardware CAN wake-up status flag if woke from sleep
    if (CAN0->WU_STATUS & CAN_WUSTATUS_WAKUP_STS_Msk) {
        CAN0->WU_STATUS = 0;
    }

    uint32_t u32IIDRstatus = CAN0->IIDR;

    // Check if it's a status/error interrupt (reading STATUS clears it)
    if (u32IIDRstatus == 0x8000u) {
        volatile uint32_t sts = CAN0->STATUS;
        (void)sts;
    }
    // Check if it's a message object interrupt (1-31)
    else if ((u32IIDRstatus >= 1u) && (u32IIDRstatus <= 31u)) {
        uint8_t obj_num = (uint8_t)u32IIDRstatus;
        STR_CANMSG_T rx_msg;
        CAN_Receive(CAN0, obj_num - 1, &rx_msg);

        // Convert to canbox-core format and queue
        can_frame_t frame;
        frame.id = rx_msg.Id;
        frame.dlc = rx_msg.DLC;
        frame.is_extended = (rx_msg.IdType == CAN_EXT_ID);
        frame.is_remote = FALSE; // NUC131 BSP doesn't expose RTR flag easily
        frame.timestamp_ms = hal_get_tick_ms();
        memcpy(frame.data, rx_msg.Data, 8);

        ring_buffer_push(&s_can_rx_rb, &frame);

        // Clear the interrupt
        CAN_CLR_INT_PENDING_BIT(CAN0, obj_num - 1);
    }
}
