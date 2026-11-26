/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

/**
  *	维护建议
  * 顺序：protocol.h -> referee_usart_task.c -> referee.c
  *		protocol.h:通信的基础约定 映射裁判系统串口协议的 帧结构、命令码、解包流程
  *				   统一底层解包和上层处理的标准
  *		referee_usart_task.c:完成 串口硬件接收 协议帧完整解析 ** DMA-FIFO **
  *		referee.c:完成协议帧到结构体映射、将数据分发、提供上层可调用的API
  */

#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "usart.h"
#include <stdint.h>

/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);
 
extern UART_HandleTypeDef huart6;

uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];		//// DMA双接收缓冲区，避免溢出

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
uint8_t usart1_dma_rx_idx = 0;
static void usart1_dma_doublebuf_start(void)
{
    // F1 用 ReceiveToIdle_DMA，IDLE空闲事件回调就是 HAL_UARTEx_RxEventCallback
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[usart1_dma_rx_idx], USART_RX_BUF_LENGHT);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void referee_usart_task(void *argument)
{
    // 原有初始化
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    
    // 启动接收
    usart1_dma_doublebuf_start();

    while(1)
    {
        referee_unpack_fifo_data();
        osDelay(10); 
    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
/**
  *flow
  *unpack_step_e 记录解包步骤 状态机
  *匹配 0xA5，匹配成功进入下一步
  *缓存 低8位 ，进入 高8位 解析
  *拼接总长度，校验长度合法性
  *缓存帧序号，进入帧头 CRC8
  *校验帧头，通过则进入数据段
  *缓存数据段，校验 CRC16 后分发
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;
		  
		//校验过程
        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        // usart1_dma_rx_idx 是刚才DMA接收填满的那块
        fifo_s_puts(&referee_fifo, (char*)usart1_buf[usart1_dma_rx_idx], Size);
        detect_hook(REFEREE_TOE);

        // 切换到另外一块buf，重启DMA接收
        usart1_dma_rx_idx = 1 - usart1_dma_rx_idx;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[usart1_dma_rx_idx], USART_RX_BUF_LENGHT);
    }
}
