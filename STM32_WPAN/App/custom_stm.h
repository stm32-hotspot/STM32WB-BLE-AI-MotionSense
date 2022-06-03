/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_STM_H
#define __CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* HardwareService */
  CUSTOM_STM_HW_MOTION_CHAR,
  CUSTOM_STM_HW_ENV_CHAR,
  CUSTOM_STM_HW_ACC_EVENT_CHAR,
  /* SoftwareService */
  CUSTOM_STM_SW_QUATERNIONS,
  CUSTOM_STM_SW_ECOMPASS,
  CUSTOM_STM_SW_ACTIVITY_REC,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* HWMotionChar */
  CUSTOM_STM_HW_MOTION_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_HW_MOTION_CHAR_NOTIFY_DISABLED_EVT,
  /* HWEnvChar */
  CUSTOM_STM_HW_ENV_CHAR_READ_EVT,
  CUSTOM_STM_HW_ENV_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_HW_ENV_CHAR_NOTIFY_DISABLED_EVT,
  /* HwAccEventChar */
  CUSTOM_STM_HW_ACC_EVENT_CHAR_READ_EVT,
  CUSTOM_STM_HW_ACC_EVENT_CHAR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_HW_ACC_EVENT_CHAR_NOTIFY_DISABLED_EVT,
  /* SWQuaternions */
  CUSTOM_STM_SW_QUATERNIONS_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_SW_QUATERNIONS_NOTIFY_DISABLED_EVT,
  /* SWEcompass */
  CUSTOM_STM_SW_ECOMPASS_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_SW_ECOMPASS_NOTIFY_DISABLED_EVT,
  /* SwActivityRecognition */
  CUSTOM_STM_SW_ACTIVITY_REC_READ_EVT,
  CUSTOM_STM_SW_ACTIVITY_REC_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_SW_ACTIVITY_REC_NOTIFY_DISABLED_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t   Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc( void );
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*__CUSTOM_STM_H */
