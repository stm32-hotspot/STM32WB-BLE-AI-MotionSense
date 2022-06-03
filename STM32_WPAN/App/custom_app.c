/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_ism_motion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* HardwareService */
  uint8_t               Hw_motion_char_Notification_Status;
  uint8_t               Hw_env_char_Notification_Status;
  uint8_t               Hw_acc_event_char_Notification_Status;
  /* SoftwareService */
  uint8_t               Sw_quaternions_Notification_Status;
  uint8_t               Sw_ecompass_Notification_Status;
  uint8_t               Sw_activity_rec_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

/* USER CODE BEGIN PV */
uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

uint8_t SecureReadData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
  /* HardwareService */
static void Custom_Hw_motion_char_Update_Char(void);
static void Custom_Hw_motion_char_Send_Notification(void);
static void Custom_Hw_env_char_Update_Char(void);
static void Custom_Hw_env_char_Send_Notification(void);
static void Custom_Hw_acc_event_char_Update_Char(void);
static void Custom_Hw_acc_event_char_Send_Notification(void);
  /* SoftwareService */
static void Custom_Sw_quaternions_Update_Char(void);
static void Custom_Sw_quaternions_Send_Notification(void);
static void Custom_Sw_ecompass_Update_Char(void);
static void Custom_Sw_ecompass_Send_Notification(void);
static void Custom_Sw_activity_rec_Update_Char(void);
static void Custom_Sw_activity_rec_Send_Notification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch(pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

  /* HardwareService */
    case CUSTOM_STM_HW_MOTION_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HW_MOTION_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_HW_MOTION_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_HW_MOTION_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HW_MOTION_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_HW_MOTION_CHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_HW_ENV_CHAR_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HW_ENV_CHAR_READ_EVT */

      /* USER CODE END CUSTOM_STM_HW_ENV_CHAR_READ_EVT */
      break;

    case CUSTOM_STM_HW_ENV_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HW_ENV_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_HW_ENV_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_HW_ENV_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HW_ENV_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_HW_ENV_CHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_HW_ACC_EVENT_CHAR_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HW_ACC_EVENT_CHAR_READ_EVT */

      /* USER CODE END CUSTOM_STM_HW_ACC_EVENT_CHAR_READ_EVT */
      break;

    case CUSTOM_STM_HW_ACC_EVENT_CHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HW_ACC_EVENT_CHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_HW_ACC_EVENT_CHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_HW_ACC_EVENT_CHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HW_ACC_EVENT_CHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_HW_ACC_EVENT_CHAR_NOTIFY_DISABLED_EVT */
      break;

  /* SoftwareService */
    case CUSTOM_STM_SW_QUATERNIONS_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SW_QUATERNIONS_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_SW_QUATERNIONS_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SW_QUATERNIONS_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SW_QUATERNIONS_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_SW_QUATERNIONS_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SW_ECOMPASS_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SW_ECOMPASS_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_SW_ECOMPASS_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SW_ECOMPASS_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SW_ECOMPASS_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_SW_ECOMPASS_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SW_ACTIVITY_REC_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SW_ACTIVITY_REC_READ_EVT */

      /* USER CODE END CUSTOM_STM_SW_ACTIVITY_REC_READ_EVT */
      break;

    case CUSTOM_STM_SW_ACTIVITY_REC_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SW_ACTIVITY_REC_NOTIFY_ENABLED_EVT */
      if(app_ism_motion_start() != BSP_ERROR_NONE){
        Error_Handler();
      }
      Custom_App_Context.Sw_activity_rec_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_SW_ACTIVITY_REC_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SW_ACTIVITY_REC_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SW_ACTIVITY_REC_NOTIFY_DISABLED_EVT */
      if(app_ism_motion_stop() != BSP_ERROR_NONE){
        Error_Handler();
      }
      Custom_App_Context.Sw_activity_rec_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_SW_ACTIVITY_REC_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch(pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void Custom_APP_Notify_activity_rec(uint8_t * p_data, uint16_t size){
  
  memcpy(NotifyCharData, p_data, size);
  Custom_Sw_activity_rec_Send_Notification();
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

  /* HardwareService */
void Custom_Hw_motion_char_Update_Char(void) /* Property Read */
{
  Custom_STM_App_Update_Char(CUSTOM_STM_HW_MOTION_CHAR, (uint8_t *)UpdateCharData);
  /* USER CODE BEGIN Hw_motion_char_UC*/

  /* USER CODE END Hw_motion_char_UC*/
  return;
}

void Custom_Hw_motion_char_Send_Notification(void) /* Property Notification */
 {
  if(Custom_App_Context.Hw_motion_char_Notification_Status)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_HW_MOTION_CHAR, (uint8_t *)NotifyCharData);
    /* USER CODE BEGIN Hw_motion_char_NS*/

    /* USER CODE END Hw_motion_char_NS*/
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }
  return;
}

void Custom_Hw_env_char_Update_Char(void) /* Property Read */
{
  Custom_STM_App_Update_Char(CUSTOM_STM_HW_ENV_CHAR, (uint8_t *)UpdateCharData);
  /* USER CODE BEGIN Hw_env_char_UC*/

  /* USER CODE END Hw_env_char_UC*/
  return;
}

void Custom_Hw_env_char_Send_Notification(void) /* Property Notification */
 {
  if(Custom_App_Context.Hw_env_char_Notification_Status)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_HW_ENV_CHAR, (uint8_t *)NotifyCharData);
    /* USER CODE BEGIN Hw_env_char_NS*/

    /* USER CODE END Hw_env_char_NS*/
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }
  return;
}

void Custom_Hw_acc_event_char_Update_Char(void) /* Property Read */
{
  Custom_STM_App_Update_Char(CUSTOM_STM_HW_ACC_EVENT_CHAR, (uint8_t *)UpdateCharData);
  /* USER CODE BEGIN Hw_acc_event_char_UC*/

  /* USER CODE END Hw_acc_event_char_UC*/
  return;
}

void Custom_Hw_acc_event_char_Send_Notification(void) /* Property Notification */
 {
  if(Custom_App_Context.Hw_acc_event_char_Notification_Status)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_HW_ACC_EVENT_CHAR, (uint8_t *)NotifyCharData);
    /* USER CODE BEGIN Hw_acc_event_char_NS*/

    /* USER CODE END Hw_acc_event_char_NS*/
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }
  return;
}

  /* SoftwareService */
void Custom_Sw_quaternions_Update_Char(void) /* Property Read */
{
  Custom_STM_App_Update_Char(CUSTOM_STM_SW_QUATERNIONS, (uint8_t *)UpdateCharData);
  /* USER CODE BEGIN Sw_quaternions_UC*/

  /* USER CODE END Sw_quaternions_UC*/
  return;
}

void Custom_Sw_quaternions_Send_Notification(void) /* Property Notification */
 {
  if(Custom_App_Context.Sw_quaternions_Notification_Status)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SW_QUATERNIONS, (uint8_t *)NotifyCharData);
    /* USER CODE BEGIN Sw_quaternions_NS*/

    /* USER CODE END Sw_quaternions_NS*/
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }
  return;
}

void Custom_Sw_ecompass_Update_Char(void) /* Property Read */
{
  Custom_STM_App_Update_Char(CUSTOM_STM_SW_ECOMPASS, (uint8_t *)UpdateCharData);
  /* USER CODE BEGIN Sw_ecompass_UC*/

  /* USER CODE END Sw_ecompass_UC*/
  return;
}

void Custom_Sw_ecompass_Send_Notification(void) /* Property Notification */
 {
  if(Custom_App_Context.Sw_ecompass_Notification_Status)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SW_ECOMPASS, (uint8_t *)NotifyCharData);
    /* USER CODE BEGIN Sw_ecompass_NS*/

    /* USER CODE END Sw_ecompass_NS*/
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }
  return;
}

void Custom_Sw_activity_rec_Update_Char(void) /* Property Read */
{
  Custom_STM_App_Update_Char(CUSTOM_STM_SW_ACTIVITY_REC, (uint8_t *)UpdateCharData);
  /* USER CODE BEGIN Sw_activity_rec_UC*/

  /* USER CODE END Sw_activity_rec_UC*/
  return;
}

void Custom_Sw_activity_rec_Send_Notification(void) /* Property Notification */
 {
  if(Custom_App_Context.Sw_activity_rec_Notification_Status)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SW_ACTIVITY_REC, (uint8_t *)NotifyCharData);
    /* USER CODE BEGIN Sw_activity_rec_NS*/

    /* USER CODE END Sw_activity_rec_NS*/
  }
  else
  {
    APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
  }
  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
