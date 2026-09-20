#include "hal/hal_can.h"
#include "core/ring_buffer.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include <string.h>

#define CAN_RX_RING_SIZE 32

static can_frame_t s_rx_storage[CAN_RX_RING_SIZE];
static ring_buffer_t s_can_rx_rb;

hal_status_t hal_can_init(can_baudrate_t baudrate) {
    ring_buffer_init(&s_can_rx_rb, s_rx_storage, sizeof(can_frame_t), CAN_RX_RING_SIZE);

    // Peripheral Clocks
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);

    // PA11 (CAN_RX) Input Pull-Up, PA12 (CAN_TX) Alternate Function Push-Pull
    LL_GPIO_InitTypeDef gpio_init;
    LL_GPIO_StructInit(&gpio_init);
    
    gpio_init.Pin = LL_GPIO_PIN_11;
    gpio_init.Mode = LL_GPIO_MODE_INPUT;
    gpio_init.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOA, &gpio_init);

    gpio_init.Pin = LL_GPIO_PIN_12;
    gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &gpio_init);

    // Request Initialization Mode
    CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM; // Auto-bus-off recovery
    while ((CAN1->MSR & CAN_MSR_INAK) == 0);

    // Timing Configuration (APB1 = 36 MHz)
    uint32_t prescaler = 4;
    switch (baudrate) {
        case CAN_BAUD_1M:   prescaler = 2;  break;
        case CAN_BAUD_500K: prescaler = 4;  break;
        case CAN_BAUD_250K: prescaler = 8;  break;
        case CAN_BAUD_125K: prescaler = 16; break;
        default:            prescaler = 4;  break;
    }

    CAN1->BTR = ((1 - 1) << CAN_BTR_SJW_Pos) |
                ((15 - 1) << CAN_BTR_TS1_Pos) |
                ((2 - 1) << CAN_BTR_TS2_Pos) |
                ((prescaler - 1) & CAN_BTR_BRP_Msk);

    // Filter Init: Accept all frames into FIFO 0
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~CAN_FA1R_FACT0;
    CAN1->FS1R |= CAN_FS1R_FSC0;     // 32-bit single filter scale
    CAN1->FM1R &= ~CAN_FM1R_FBM0;    // Mask mode
    CAN1->sFilterRegister[0].FR1 = 0; // ID: 0
    CAN1->sFilterRegister[0].FR2 = 0; // Mask: 0 (match all)
    CAN1->FFA1R &= ~CAN_FFA1R_FFA0;  // Assign to FIFO 0
    CAN1->FA1R |= CAN_FA1R_FACT0;    // Activate filter 0
    CAN1->FMR &= ~CAN_FMR_FINIT;

    // Leave Initialization Mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while ((CAN1->MSR & CAN_MSR_INAK) != 0);

    // Enable FIFO 0 message pending interrupt
    CAN1->IER |= CAN_IER_FMPIE0;
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    return HAL_STATUS_OK;
}

hal_status_t hal_can_send(const can_frame_t *frame) {
    if (!frame) return HAL_STATUS_ERROR;

    uint32_t tsr = CAN1->TSR;
    uint8_t mailbox = 0xFF;

    if (tsr & CAN_TSR_TME0) mailbox = 0;
    else if (tsr & CAN_TSR_TME1) mailbox = 1;
    else if (tsr & CAN_TSR_TME2) mailbox = 2;
    else return HAL_STATUS_BUSY;

    // Clear identifier and type
    CAN1->sTxMailBox[mailbox].TIR &= CAN_TI0R_TXRQ;

    if (frame->is_extended) {
        CAN1->sTxMailBox[mailbox].TIR |= (frame->id << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE;
    } else {
        CAN1->sTxMailBox[mailbox].TIR |= (frame->id << CAN_TI0R_STID_Pos);
    }

    if (frame->is_remote) {
        CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_RTR;
    }

    CAN1->sTxMailBox[mailbox].TDTR = (frame->dlc & CAN_TDT0R_DLC_Msk);

    // Pack data registers (Little-Endian byte layout in STM32 registers)
    CAN1->sTxMailBox[mailbox].TDLR = ((uint32_t)frame->data[3] << 24) |
                                    ((uint32_t)frame->data[2] << 16) |
                                    ((uint32_t)frame->data[1] << 8)  |
                                    ((uint32_t)frame->data[0]);

    CAN1->sTxMailBox[mailbox].TDHR = ((uint32_t)frame->data[7] << 24) |
                                    ((uint32_t)frame->data[6] << 16) |
                                    ((uint32_t)frame->data[5] << 8)  |
                                    ((uint32_t)frame->data[4]);

    CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ; // Request transmission
    return HAL_STATUS_OK;
}

hal_status_t hal_can_receive(can_frame_t *frame) {
    if (!frame) return HAL_STATUS_ERROR;
    return ring_buffer_pop(&s_can_rx_rb, frame) ? HAL_STATUS_OK : HAL_STATUS_TIMEOUT;
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
    while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
        can_frame_t rx;
        uint32_t rir = CAN1->sFIFOMailBox[0].RIR;

        rx.is_extended = (rir & CAN_RI0R_IDE) != 0;
        rx.is_remote   = (rir & CAN_RI0R_RTR) != 0;
        rx.id = rx.is_extended ? (rir >> CAN_RI0R_EXID_Pos) : (rir >> CAN_RI0R_STID_Pos);
        rx.dlc = CAN1->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk;

        uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
        uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;

        rx.data[0] = (uint8_t)(rdlr);
        rx.data[1] = (uint8_t)(rdlr >> 8);
        rx.data[2] = (uint8_t)(rdlr >> 16);
        rx.data[3] = (uint8_t)(rdlr >> 24);
        rx.data[4] = (uint8_t)(rdhr);
        rx.data[5] = (uint8_t)(rdhr >> 8);
        rx.data[6] = (uint8_t)(rdhr >> 16);
        rx.data[7] = (uint8_t)(rdhr >> 24);
        rx.timestamp_ms = hal_get_tick_ms();

        ring_buffer_push(&s_can_rx_rb, &rx);

        // Release FIFO 0 output mailbox
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
}
