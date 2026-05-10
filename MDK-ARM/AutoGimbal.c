#include "AutoGimbal.h"

#if VISION_USE_USB_CDC
#include "usb_device.h"
#include "usbd_cdc_if.h"
#else
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
#endif

volatile uint32_t vision_last_target_time = 0;

typedef union
{
    vision_rx_frame_t frame;
    uint8_t raw[VISION_RX_FRAME_LENGTH];
} vision_rx_buffer_t;

static vision_rx_buffer_t vision_rx_data;
static BUF RresPi;
static uint8_t vision_tx_buffer[VISION_TX_FRAME_LENGTH];
#if !VISION_USE_USB_CDC
static uint8_t sbus_rx_double_buf[2][BUFLENGTH];
static uint8_t rx_buf_idx = 0;
#endif

vision_tx_buffer_t auto_to_nuc_data;
#if VISION_USE_USB_CDC
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif

void vision_try_transmit(void)
{
#if VISION_USE_USB_CDC
    USBD_CDC_HandleTypeDef *hcdc;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hUsbDeviceFS.pClassData == NULL)
    {
        return;
    }

    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0U)
    {
        return;
    }

    memcpy(vision_tx_buffer, auto_to_nuc_data.raw, sizeof(vision_tx_buffer));
    (void)CDC_Transmit_FS(vision_tx_buffer, sizeof(vision_tx_buffer));
#else
    if (huart1.gState == HAL_UART_STATE_READY)
    {
        memcpy(vision_tx_buffer, auto_to_nuc_data.raw, sizeof(vision_tx_buffer));
        HAL_UART_Transmit_DMA(&huart1, vision_tx_buffer, sizeof(vision_tx_buffer));
    }
#endif
}

void AUTO_control_init(void)
{
#if VISION_USE_USB_CDC
    static uint8_t usb_inited = 0;
    if (usb_inited == 0U)
    {
        MX_USB_DEVICE_Init();
        usb_inited = 1U;
    }
#else
    static uint8_t uart_inited = 0;
    if (uart_inited == 0U)
    {
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        rx_buf_idx = 0;
        HAL_UART_Receive_DMA(&huart1, sbus_rx_double_buf[rx_buf_idx], BUFLENGTH);
        uart_inited = 1U;
    }
#endif
    vision_last_target_time = 0;
    memset(&RresPi, 0, sizeof(RresPi));
    memset(&vision_rx_data, 0, sizeof(vision_rx_data));
    RresPi.Rec.mode = 0;
    RresPi.Rec.x = 0.0f;
    RresPi.Rec.y = 0.0f;
    RresPi.Rec.yaw = 0.0f;
    RresPi.Rec.yaw_vel = 0.0f;
    RresPi.Rec.yaw_acc = 0.0f;
    RresPi.Rec.pitch = 0.0f;
    RresPi.Rec.pitch_vel = 0.0f;
    RresPi.Rec.pitch_acc = 0.0f;
    RresPi.Rec.distance = -1.0f;
}

static void vision_memory_from_buffer(const uint8_t *buffer, vision_rx_frame_t *ctrl)
{
    memcpy(ctrl, buffer, sizeof(*ctrl));
}

static void vision_update_ctrl_from_frame(const vision_rx_frame_t *frame, CTRL *ctrl);

void memory_from_buffer(uint8_t *buffer, CTRL *ctrl)
{
    if (buffer == NULL || ctrl == NULL)
    {
        return;
    }

    vision_memory_from_buffer(buffer, &vision_rx_data.frame);
    vision_update_ctrl_from_frame(&vision_rx_data.frame, ctrl);
}

static void vision_update_ctrl_from_frame(const vision_rx_frame_t *frame, CTRL *ctrl)
{
    if (frame == NULL || ctrl == NULL)
    {
        return;
    }

    ctrl->FRAME_HEADER = frame->head[0];
    ctrl->FRAME_HEADER_2 = frame->head[1];
    ctrl->mode = frame->mode;
    ctrl->x = frame->yaw;
    ctrl->y = -frame->pitch;
    ctrl->yaw = frame->yaw;
    ctrl->yaw_vel = frame->yaw_vel;
    ctrl->yaw_acc = frame->yaw_acc;
    ctrl->pitch = frame->pitch;
    ctrl->pitch_vel = frame->pitch_vel;
    ctrl->pitch_acc = frame->pitch_acc;
    ctrl->distance = (frame->mode == 2U) ? 0.0f : -1.0f;
    ctrl->blank = 0;
    ctrl->FRAME_TAIL = frame->tail[0];
    ctrl->FRAME_TAIL_2 = frame->tail[1];
}

static void vision_process_received(uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint8_t frame_received = 0;

    if (buf == NULL || len < VISION_RX_FRAME_LENGTH)
    {
        return;
    }

    for (i = 0; i + VISION_RX_FRAME_LENGTH <= len; i++)
    {
        if (buf[i] == 0x5A &&
            buf[i + 1] == 0xA5 &&
            buf[i + VISION_RX_FRAME_LENGTH - 2] == 0x7F &&
            buf[i + VISION_RX_FRAME_LENGTH - 1] == 0xFE)
        {
            vision_memory_from_buffer(&buf[i], &vision_rx_data.frame);
            vision_update_ctrl_from_frame(&vision_rx_data.frame, &RresPi.Rec);
            vision_last_target_time = HAL_GetTick();
            frame_received = 1;
            i += VISION_RX_FRAME_LENGTH - 1;
        }
    }

    if (frame_received)
    {
        vision_try_transmit();
    }
}

void USB_CDC_ProcessReceived(uint8_t *buf, uint32_t len)
{
#if VISION_USE_USB_CDC
    vision_process_received(buf, len);
#else
    (void)buf;
    (void)len;
#endif
}

void USART1_IDLE_Handler(void)
{
#if VISION_USE_USB_CDC
    return;
#else
    uint16_t data_length;
    uint8_t *process_buf;

    if (RESET == __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UART_AbortReceive(&huart1);

    data_length = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    process_buf = sbus_rx_double_buf[rx_buf_idx];
    rx_buf_idx ^= 1;

    HAL_UART_Receive_DMA(&huart1, sbus_rx_double_buf[rx_buf_idx], BUFLENGTH);
    vision_process_received(process_buf, data_length);
#endif
}

CTRL *get_AUTO_control_point(void)
{
    return &RresPi.Rec;
}
