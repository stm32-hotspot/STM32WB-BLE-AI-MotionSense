/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 * @file    app_ism_motion.c
 * @author  Minku Yeo
 * @brief   Motion application
 *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_ism_motion.h"
#include "ism330dhcx.h"
#include "main.h"

// Ref. https://wiki.stmicroelectronics.cn/stm32mcu/index.php?title=AI:How_to_perform_motion_sensing_on_STM32L4_IoTnode&oldid=19724
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"

#include "dbg_trace.h"
#include "stm32_seq.h"

#include "common_blesvc.h"
#include "custom_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct accel_data2float_data_type{
  float xf;
  float yf;
  float zf;
}accel_f_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_7B_I2C_ADDRESS 0x6A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static ISM330DHCX_Object_t m_ism_obj;
static int32_t m_scale_acc;

static ai_handle network;
static float aiInData[AI_NETWORK_IN_1_SIZE];
static float aiOutData[AI_NETWORK_OUT_1_SIZE];
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
const char* activities[AI_NETWORK_OUT_1_SIZE] = {
  "stationary", "walking", "running"
};
static ai_buffer * ai_input;
static ai_buffer * ai_output;

static uint32_t m_data_index;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
static int32_t motion_i2c_init(uint8_t sa0_pin);
static int32_t motion_config(void);

static void motion_data2float(ISM330DHCX_Axes_t * p_axes, accel_f_t * p_f);
static uint32_t motion_ai_argmax(const float * values, uint32_t len);
static void motion_run_ai_task(void);

static void motion_ai_init(void);
static void motion_ai_run(float *pIn, float *pOut);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int32_t app_ism_motion_init(uint8_t sa0_pin){
  int32_t ret = motion_i2c_init(sa0_pin);
  
  if(ret == ISM330DHCX_OK){    
    UTIL_SEQ_RegTask( 1<< CFG_TASK_RUN_AI_TASK_ID, UTIL_SEQ_RFU, motion_run_ai_task);
    
    motion_ai_init();
  }
  
  return ret;
}

int32_t app_ism_motion_start(void){
  
  int32_t ret;
  
  m_data_index = 0;
  
  ret  = ISM330DHCX_ACC_Enable(&m_ism_obj);
  ret |= ISM330DHCX_Set_INT1_Drdy(&m_ism_obj, 0x01);    /* INT1_DRDY_XL (Enables accel data-ready interrupt) */
  //ret |= ISM330DHCX_ACC_GetAxesRaw(&m_ism_obj, &dummy); /* Clear DRDY */
  
  return ret;
}

int32_t app_ism_motion_stop(void){
  
  int32_t               ret;
  ISM330DHCX_AxesRaw_t  dummy;
  
  m_data_index = 0;
  
  ret = ISM330DHCX_ACC_Disable(&m_ism_obj);
  ret |= ISM330DHCX_ACC_GetAxesRaw(&m_ism_obj, &dummy); /* Clear DRDY */
  //ISM330DHCX_ACC_Disable_DRDY_On_INT1(&m_ism_obj);
  return ret;
}

void app_ism_motion_int1(void){
  
  UTIL_SEQ_SetTask(1 << CFG_TASK_RUN_AI_TASK_ID, CFG_SCH_PRIO_0);
}

static int32_t motion_i2c_init(uint8_t sa0_pin){
  assert_param(sa0_pin <= 1);
  
  ISM330DHCX_IO_t               bsp_io_ctx = 
  {
    .BusType = ISM330DHCX_I2C_BUS,
    .Address = ((DEFAULT_7B_I2C_ADDRESS | sa0_pin) << 1) + 1,
    
    .Init = BSP_I2C3_Init,
    .DeInit = BSP_I2C3_DeInit,
    
    .ReadReg = BSP_I2C3_ReadReg,
    .WriteReg = BSP_I2C3_WriteReg,
    .GetTick = BSP_GetTick
  };
  
  uint8_t                       id;
  int32_t                       ret = BSP_ERROR_NONE;
  
  memset(&m_ism_obj, 0x00, sizeof(ISM330DHCX_Object_t));
    
  if(ISM330DHCX_RegisterBusIO(&m_ism_obj, &bsp_io_ctx) != ISM330DHCX_OK){
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else if(ISM330DHCX_ReadID(&m_ism_obj, &id) != ISM330DHCX_OK){
    ret = BSP_ERROR_BUS_FAILURE;
  }
  else if(id != ISM330DHCX_ID){
    ret = BSP_ERROR_BUS_FAILURE;
  }
  else{
    ret = motion_config();
  }
  return ret;
}

static int32_t motion_config(void){
  ISM330DHCX_Capabilities_t     cap;  
  int32_t                       ret;
  
  m_scale_acc = 4; /* Accelerometer scale [-4000mg; +4000mg] */
  
  (void) ISM330DHCX_GetCapabilities(&m_ism_obj, &cap);
  
  ret = ISM330DHCX_Init(&m_ism_obj);
  
  ret |= ISM330DHCX_ACC_SetOutputDataRate(&m_ism_obj, 26.0f);           /* 26 Hz */
  ret |= ISM330DHCX_ACC_SetFullScale(&m_ism_obj, m_scale_acc);                    
  ret |= ISM330DHCX_Set_Drdy_Mode(&m_ism_obj, 0x80);                    /* Data-ready pulsed mode - 75 �s long pulse */
  //ret |= ISM330DHCX_GYRO_Enable(&m_ism_obj);
  
  return ret;
}

static void motion_data2float(ISM330DHCX_Axes_t * p_axes, accel_f_t * p_f){
  float ratio = (float)(m_scale_acc) * 1000.0f;
  
  p_f->xf = (float)(p_axes->x) / ratio;
  p_f->yf = (float)(p_axes->y) / ratio;
  p_f->zf = (float)(p_axes->z) / ratio;
}

static uint32_t motion_ai_argmax(const float * values, uint32_t len)
{
  float max_value = values[0];
  uint32_t max_index = 0;
  for (uint32_t i = 1; i < len; i++) {
    if (values[i] > max_value) {
      max_value = values[i];
      max_index = i;
    }
  }
  return max_index;
}

static void motion_run_ai_task(void){
    
  ISM330DHCX_Axes_t             axes;
  accel_f_t                     accel_f;
  
  if(ISM330DHCX_ACC_GetAxes(&m_ism_obj, &axes) != ISM330DHCX_OK){
    Error_Handler();
  }
  motion_data2float(&axes, &accel_f);
  
  /* Normalize data to [-1; 1] and accumulate into input buffer */
  /* Note: window overlapping can be managed here */
  aiInData[m_data_index + 0] = accel_f.xf;
  aiInData[m_data_index + 1] = accel_f.yf;
  aiInData[m_data_index + 2] = accel_f.zf;
  m_data_index += 3;
  //APP_DBG_MSG("X = %02.4f\tY = %02.4f\tZ = %02.4f\t\r\n\r", accel_f.xf, accel_f.yf, accel_f.zf);
  
  if (m_data_index == AI_NETWORK_IN_1_SIZE) {
    m_data_index = 0;
    
    motion_ai_run(aiInData, aiOutData);
    
    uint32_t result = motion_ai_argmax(aiOutData, AI_NETWORK_OUT_1_SIZE);
    uint8_t data[] = {0x00, 0x00, 1 + (uint8_t) result};
    //P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, data);
    if(result == 2){
      data[2] = 4; // BlueST data format
    }
    memset(data, HAL_GetTick()>>3, sizeof(uint16_t));
    Custom_APP_Notify_activity_rec(data, sizeof(data));
    
    APP_DBG_MSG("Motion: %d (BLE data = %d) - %s / %s = %.3f\t%s = %.3f\t%s = %.3f\r\n",
                result, data[2], activities[result],
                activities[0], aiOutData[0],
                activities[1], aiOutData[1],
                activities[2], aiOutData[2]);
  }
}

static void motion_ai_init(void)
{
  ai_error err;

  /* Create a local array with the addresses of the activations buffers */
  const ai_handle act_addr[] = { activations };
  /* Create an instance of the model */
  err = ai_network_create_and_init(&network, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    APP_DBG_MSG("ai_network_create error - type=%d code=%d\r\n\r", err.type, err.code);
    Error_Handler();
  }
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);
}

static void motion_ai_run(float *pIn, float *pOut)
{
  ai_i32 batch;
  volatile ai_error err;

  /* Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(pIn);
  ai_output[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1) {
    err = ai_network_get_error(network);
    APP_DBG_MSG("AI ai_network_run error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
}
