/**
  ******************************************************************************
  * @file          : app_tof.c
  * @author        : IMG SW Application Team
  * @brief         : This file provides code for the configuration
  *                  of the STMicroelectronics.X-CUBE-TOF1.3.4.3 instances.
  ******************************************************************************
  *
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include <stdio.h>
#include <stdint.h>

#include "custom_ranging_sensor.h"
#include "stm32f401xe.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_nucleo.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* uncomment following to use directly the bare driver instead of the BSP */
/* #define USE_BARE_DRIVER */
#define TIMING_BUDGET (20U) /* 5 ms < TimingBudget < 100 ms */
#define RANGING_FREQUENCY (20U) /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
#define POLLING_PERIOD (1)
#define TOF_SENSOR_COUNT (CUSTOM_RANGING_INSTANCES_NBR)

#define TOF_FRAME_SOF_0 (0xAAU)
#define TOF_FRAME_SOF_1 (0x55U)
#define TOF_FRAME_VERSION (0x01U)
#define TOF_FRAME_TYPE_RANGING (0x01U)
#define TOF_FRAME_TYPE_ERROR (0x02U)
#define TOF_FRAME_STATUS_NO_TARGET (255U)
#define TOF_FRAME_DIST_NO_TARGET (0xFFFFU)
#define TOF_ERROR_SENSOR_OFFLINE (0x01U)
#define TOF_ERROR_UART_TX_FAIL (0x02U)
#define TOF_UART_TX_TIMEOUT_MS (20U)
#define TOF_ZONE_RECORD_SIZE (3U) /* dist_u16 + status_u8 */
#define TOF_ERROR_PAYLOAD_SIZE (5U) /* error_code_u8 + error_value_u32 */
#define TOF_FRAME_MAX_SIZE (2U + 1U + 1U + 2U + 4U + 1U + 1U + 1U + 1U + 2U + \
                            (RANGING_SENSOR_MAX_NB_ZONES * TOF_ZONE_RECORD_SIZE) + 2U)

/* Private variables ---------------------------------------------------------*/
#ifndef USE_BARE_DRIVER
static RANGING_SENSOR_Capabilities_t Cap[TOF_SENSOR_COUNT];
#endif
static RANGING_SENSOR_ProfileConfig_t Profile;

static int32_t status = 0;
volatile uint8_t ToF_EventDetected = 0;
static uint32_t CurrentSensorIdx = 0U;
static uint8_t SensorActive[TOF_SENSOR_COUNT] = {0};
static int32_t SensorInitError[TOF_SENSOR_COUNT] = {0};
static uint32_t ActiveSensorCount = 0U;
static uint16_t TofFrameSequence = 0U;
static uint8_t TofBinaryOutputEnabled = 1U;
static uint32_t TofUartTxFailCount = 0U;

/* Private function prototypes -----------------------------------------------*/
#ifdef USE_BARE_DRIVER
static uint8_t map_target_status(uint8_t status);
static int32_t convert_data_format(VL53L8CX_Object_t *pObj,
                                   VL53L8CX_ResultsData *data, RANGING_SENSOR_Result_t *pResult);
#endif
static void MX_VL53L8CX_SimpleRanging_Init(void);
static void MX_VL53L8CX_SimpleRanging_Process(void);
static void print_result(uint32_t Instance, RANGING_SENSOR_Result_t *Result);
static void toggle_resolution(void);
static void toggle_signal_and_ambient(void);
static void display_commands_banner(uint32_t Instance);
static void handle_cmd(uint8_t cmd);
static uint8_t get_key(void);
static uint32_t com_has_data(void);
static uint32_t get_first_active_sensor(void);
static uint32_t get_next_active_sensor(uint32_t current_idx);
static void deactivate_sensor(uint32_t instance);
static void reset_current_sensor_index(void);
static void tof_fatal_loop(const char *msg);
static uint16_t tof_crc16_ccitt_false(const uint8_t *data, uint32_t len);
static uint8_t tof_resolution_code_from_zones(uint32_t zone_count);
static void tof_send_error_frame(uint8_t sensor_id, uint8_t error_code, uint32_t error_value);

void MX_TOF_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN TOF_Init_PreTreatment */

  /* USER CODE END TOF_Init_PreTreatment */

  /* Initialize the peripherals and the TOF components */

  MX_VL53L8CX_SimpleRanging_Init();

  /* USER CODE BEGIN TOF_Init_PostTreatment */

  /* USER CODE END TOF_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_TOF_Process(void)
{
  /* USER CODE BEGIN TOF_Process_PreTreatment */

  /* USER CODE END TOF_Process_PreTreatment */

  MX_VL53L8CX_SimpleRanging_Process();

  /* USER CODE BEGIN TOF_Process_PostTreatment */

  /* USER CODE END TOF_Process_PostTreatment */
}

static void MX_VL53L8CX_SimpleRanging_Init(void)
{
  /* Initialize Virtual COM Port */
  int32_t com_status = BSP_COM_Init(COM1);
  (void)BSP_COM_SelectLogPort(COM1);
  /* Disable buffering so VCP prints appear immediately */
  setvbuf(stdout, NULL, _IONBF, 0);

  printf("\r\n[TOF] MX_VL53L8CX_SimpleRanging_Init start (COM status=%ld, instances=%u)\r\n",
         (long)com_status, (unsigned int)TOF_SENSOR_COUNT);
  printf("\033[2H\033[2J");
  printf("Sensor initialization...\n");

  if (com_status != BSP_ERROR_NONE)
  {
    tof_fatal_loop("BSP_COM_Init(COM1) failed");
  }

  ActiveSensorCount = 0U;
  for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
  {
    SensorActive[instance] = 0U;
    SensorInitError[instance] = BSP_ERROR_NO_INIT;
  }

  for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
  {
    status = CUSTOM_RANGING_SENSOR_Init(instance);

    if (status == BSP_ERROR_NONE)
    {
      SensorActive[instance] = 1U;
      SensorInitError[instance] = BSP_ERROR_NONE;
      ActiveSensorCount++;
      printf("[TOF] Sensor %lu initialized successfully\r\n", (unsigned long)instance);
    }
    else
    {
      SensorActive[instance] = 0U;
      SensorInitError[instance] = status;
      printf("CUSTOM_RANGING_SENSOR_Init skipped instance %lu (error %ld)\n",
             (unsigned long)instance, (long)status);
    }
  }

  printf("[TOF] Active sensor count after init: %lu\r\n", (unsigned long)ActiveSensorCount);

  if (ActiveSensorCount == 0U)
  {
    for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
    {
      printf("[TOF][DIAG] instance %lu init_error=%ld\r\n",
             (unsigned long)instance,
             (long)SensorInitError[instance]);
    }
    tof_fatal_loop("No active VL53L8CX sensors detected. Check PWR_EN/LPn wiring and I2C bus.");
  }
}

#ifdef USE_BARE_DRIVER
static void MX_VL53L8CX_SimpleRanging_Process(void)
{
  static RANGING_SENSOR_Result_t Result;
  VL53L8CX_Object_t *pL5obj = CUSTOM_RANGING_CompObj[CUSTOM_VL53L8CX];
  static VL53L8CX_ResultsData data;
  uint8_t NewDataReady = 0;

  /* Start in 8x8 continuous mode so no manual toggle is required after reset */
  Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
  Profile.TimingBudget = TIMING_BUDGET;
  Profile.Frequency = RANGING_FREQUENCY; /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
  Profile.EnableAmbient = 0; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 0; /* Enable: 1, Disable: 0 */

  pL5obj->IsAmbientEnabled = Profile.EnableAmbient;
  pL5obj->IsSignalEnabled = Profile.EnableSignal;

  /*
     use case VL53L8CX_PROFILE_4x4_CONTINUOUS:
  */
  status = vl53l8cx_set_resolution(&(pL5obj->Dev), VL53L8CX_RESOLUTION_4X4);
  status |= vl53l8cx_set_ranging_mode(&(pL5obj->Dev), VL53L8CX_RANGING_MODE_CONTINUOUS);
  status |= vl53l8cx_set_integration_time_ms(&(pL5obj->Dev), TIMING_BUDGET);
  status |= vl53l8cx_set_ranging_frequency_hz(&(pL5obj->Dev), RANGING_FREQUENCY);

  if (status != VL53L8CX_STATUS_OK)
  {
    printf("ERROR : Configuration programming error!\n\n");
    while (1);
  }

  status = vl53l8cx_start_ranging(&(pL5obj->Dev));
  if (status != VL53L8CX_STATUS_OK)
  {
    printf("vl53l8cx_start_ranging failed\n");
    while (1);
  }

  while (1)
  {
    /* polling mode */
    (void)vl53l8cx_check_data_ready(&(pL5obj->Dev), &NewDataReady);

    if (NewDataReady != 0)
    {
      status = vl53l8cx_get_ranging_data(&(pL5obj->Dev), &data);

      if (status == VL53L8CX_STATUS_OK)
      {
        /*
         Convert the data format to Result format.
         Note that you can print directly from data format
        */
        if (convert_data_format(pL5obj, &data, &Result) < 0)
        {
          printf("convert_data_format failed\n");
          while (1);
        }
        print_result(CUSTOM_VL53L8CX_0, &Result);
      }
    }

    if (com_has_data())
    {
      handle_cmd(get_key());
    }

    HAL_Delay(POLLING_PERIOD);
  }
}
#else
static void MX_VL53L8CX_SimpleRanging_Process(void)
{
  static RANGING_SENSOR_Result_t Result;
  uint32_t Id;

  for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
  {
    if (SensorActive[instance] == 0U)
    {
      continue;
    }

    if (CUSTOM_RANGING_SENSOR_ReadID(instance, &Id) != BSP_ERROR_NONE)
    {
      printf("CUSTOM_RANGING_SENSOR_ReadID failed on instance %lu\n", (unsigned long)instance);
      tof_send_error_frame((uint8_t)instance, TOF_ERROR_SENSOR_OFFLINE, (uint32_t)BSP_ERROR_COMPONENT_FAILURE);
      deactivate_sensor(instance);
      continue;
    }

    if (CUSTOM_RANGING_SENSOR_GetCapabilities(instance, &Cap[instance]) != BSP_ERROR_NONE)
    {
      printf("CUSTOM_RANGING_SENSOR_GetCapabilities failed on instance %lu\n", (unsigned long)instance);
      tof_send_error_frame((uint8_t)instance, TOF_ERROR_SENSOR_OFFLINE, (uint32_t)BSP_ERROR_COMPONENT_FAILURE);
      deactivate_sensor(instance);
      continue;
    }

    printf("[TOF] Instance %lu capabilities: zones=%lu, targets=%lu\r\n",
           (unsigned long)instance,
           (unsigned long)Cap[instance].NumberOfZones,
           (unsigned long)Cap[instance].MaxNumberOfTargetsPerZone);
  }

  (void)Id;

  /* Default to 8x8 continuous so the wider FoV is active immediately */
  Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
  Profile.TimingBudget = TIMING_BUDGET;
  Profile.Frequency = RANGING_FREQUENCY; /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
  Profile.EnableAmbient = 0; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 0; /* Enable: 1, Disable: 0 */

  /* set the profile if different from default one */
  for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
  {
    if (SensorActive[instance] == 0U)
    {
      continue;
    }

    printf("[TOF] Configuring instance %lu with profile %lu\r\n",
           (unsigned long)instance,
           (unsigned long)Profile.RangingProfile);
    status = CUSTOM_RANGING_SENSOR_ConfigProfile(instance, &Profile);

    if (status != BSP_ERROR_NONE)
    {
      printf("CUSTOM_RANGING_SENSOR_ConfigProfile failed on instance %lu (error %ld)\n",
             (unsigned long)instance, (long)status);
      tof_send_error_frame((uint8_t)instance, TOF_ERROR_SENSOR_OFFLINE, (uint32_t)status);
      deactivate_sensor(instance);
      continue;
    }

    status = CUSTOM_RANGING_SENSOR_Start(instance, RS_MODE_ASYNC_CONTINUOUS);

    if (status != BSP_ERROR_NONE)
    {
      printf("CUSTOM_RANGING_SENSOR_Start failed on instance %lu\n", (unsigned long)instance);
      tof_send_error_frame((uint8_t)instance, TOF_ERROR_SENSOR_OFFLINE, (uint32_t)status);
      deactivate_sensor(instance);
      continue;
    }
  }

  if (ActiveSensorCount == 0U)
  {
    tof_fatal_loop("All VL53L8CX sensors failed to start");
  }

  reset_current_sensor_index();
  if (CurrentSensorIdx >= TOF_SENSOR_COUNT)
  {
    tof_fatal_loop("No active VL53L8CX sensors available after start");
  }

  display_commands_banner(CurrentSensorIdx);

  printf("[TOF] Entering ranging loop with %lu active sensor(s)\r\n", (unsigned long)ActiveSensorCount);

  while (1)
  {
    if (SensorActive[CurrentSensorIdx] == 0U)
    {
      CurrentSensorIdx = get_next_active_sensor(CurrentSensorIdx);
      HAL_Delay(POLLING_PERIOD);
      continue;
    }

    /* polling mode */
    status = CUSTOM_RANGING_SENSOR_GetDistance(CurrentSensorIdx, &Result);

    if (status == BSP_ERROR_NONE)
    {
      print_result(CurrentSensorIdx, &Result);
      CurrentSensorIdx = get_next_active_sensor(CurrentSensorIdx);
    }
    else if (status == BSP_ERROR_BUSY)
    {
      /* Async mode: no fresh sample for this sensor yet, move on quickly. */
      CurrentSensorIdx = get_next_active_sensor(CurrentSensorIdx);
    }
    else
    {
      printf("CUSTOM_RANGING_SENSOR_GetDistance failed on instance %lu (error %ld)\n",
             (unsigned long)CurrentSensorIdx, (long)status);
      tof_send_error_frame((uint8_t)CurrentSensorIdx, TOF_ERROR_SENSOR_OFFLINE, (uint32_t)status);
      deactivate_sensor(CurrentSensorIdx);

      if (ActiveSensorCount == 0U)
      {
        tof_fatal_loop("All VL53L8CX sensors offline");
      }

      CurrentSensorIdx = get_next_active_sensor(CurrentSensorIdx);
    }

    if (com_has_data())
    {
      handle_cmd(get_key());
    }

    HAL_Delay(POLLING_PERIOD);
  }
}
#endif /* USE_BARE_DRIVER */

static void print_result(uint32_t Instance, RANGING_SENSOR_Result_t *Result)
{
  if (TofBinaryOutputEnabled == 0U)
  {
    printf("[TOF][TXT] sensor=%lu zones=%lu\r\n",
           (unsigned long)Instance,
           (unsigned long)Result->NumberOfZones);

    for (uint32_t zone = 0U; zone < Result->NumberOfZones; zone++)
    {
      if (Result->ZoneResult[zone].NumberOfTargets > 0U)
      {
        printf("%lu,%lu,%lu\r\n",
               (unsigned long)zone,
               (unsigned long)Result->ZoneResult[zone].Distance[0],
               (unsigned long)Result->ZoneResult[zone].Status[0]);
      }
      else
      {
        printf("%lu,%u,%u\r\n",
               (unsigned long)zone,
               (unsigned int)TOF_FRAME_DIST_NO_TARGET,
               (unsigned int)TOF_FRAME_STATUS_NO_TARGET);
      }
    }

    return;
  }

  uint8_t frame_buf[TOF_FRAME_MAX_SIZE];
  uint32_t zone_count = Result->NumberOfZones;
  uint32_t payload_len;
  uint32_t idx = 0U;
  uint16_t crc;

  if (zone_count > RANGING_SENSOR_MAX_NB_ZONES)
  {
    zone_count = RANGING_SENSOR_MAX_NB_ZONES;
  }

  payload_len = zone_count * TOF_ZONE_RECORD_SIZE;

  frame_buf[idx++] = TOF_FRAME_SOF_0;
  frame_buf[idx++] = TOF_FRAME_SOF_1;
  frame_buf[idx++] = TOF_FRAME_VERSION;
  frame_buf[idx++] = TOF_FRAME_TYPE_RANGING;

  frame_buf[idx++] = (uint8_t)(TofFrameSequence & 0xFFU);
  frame_buf[idx++] = (uint8_t)((TofFrameSequence >> 8) & 0xFFU);

  uint32_t timestamp_ms = HAL_GetTick();
  frame_buf[idx++] = (uint8_t)(timestamp_ms & 0xFFU);
  frame_buf[idx++] = (uint8_t)((timestamp_ms >> 8) & 0xFFU);
  frame_buf[idx++] = (uint8_t)((timestamp_ms >> 16) & 0xFFU);
  frame_buf[idx++] = (uint8_t)((timestamp_ms >> 24) & 0xFFU);

  frame_buf[idx++] = (uint8_t)Instance;
  frame_buf[idx++] = tof_resolution_code_from_zones(zone_count);
  frame_buf[idx++] = (uint8_t)zone_count;
  frame_buf[idx++] = 0U; /* target index: send target #0 per zone */
  frame_buf[idx++] = (uint8_t)(payload_len & 0xFFU);
  frame_buf[idx++] = (uint8_t)((payload_len >> 8) & 0xFFU);

  for (uint32_t zone = 0U; zone < zone_count; zone++)
  {
    uint16_t dist_u16;
    uint8_t status_u8;

    if (Result->ZoneResult[zone].NumberOfTargets > 0U)
    {
      uint32_t raw_dist = Result->ZoneResult[zone].Distance[0];
      if (raw_dist > 0xFFFFU)
      {
        raw_dist = 0xFFFFU;
      }
      dist_u16 = (uint16_t)raw_dist;

      uint32_t raw_status = Result->ZoneResult[zone].Status[0];
      status_u8 = (raw_status > 0xFFU) ? 0xFFU : (uint8_t)raw_status;
    }
    else
    {
      dist_u16 = TOF_FRAME_DIST_NO_TARGET;
      status_u8 = TOF_FRAME_STATUS_NO_TARGET;
    }

    frame_buf[idx++] = (uint8_t)(dist_u16 & 0xFFU);
    frame_buf[idx++] = (uint8_t)((dist_u16 >> 8) & 0xFFU);
    frame_buf[idx++] = status_u8;
  }

  crc = tof_crc16_ccitt_false(&frame_buf[2], idx - 2U);
  frame_buf[idx++] = (uint8_t)(crc & 0xFFU);
  frame_buf[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

  if (HAL_UART_Transmit(&hcom_uart[COM1], frame_buf, (uint16_t)idx, TOF_UART_TX_TIMEOUT_MS) == HAL_OK)
  {
    TofFrameSequence++;
  }
  else
  {
    TofUartTxFailCount++;
    tof_send_error_frame((uint8_t)Instance, TOF_ERROR_UART_TX_FAIL, TofUartTxFailCount);
  }
}

static void tof_send_error_frame(uint8_t sensor_id, uint8_t error_code, uint32_t error_value)
{
  if (TofBinaryOutputEnabled == 0U)
  {
    printf("[TOF][ERR] sensor=%u code=%u value=%lu\r\n",
           (unsigned int)sensor_id,
           (unsigned int)error_code,
           (unsigned long)error_value);
    return;
  }

  uint8_t frame_buf[2U + 1U + 1U + 2U + 4U + 1U + 1U + 1U + 1U + 2U + TOF_ERROR_PAYLOAD_SIZE + 2U];
  uint32_t idx = 0U;

  frame_buf[idx++] = TOF_FRAME_SOF_0;
  frame_buf[idx++] = TOF_FRAME_SOF_1;
  frame_buf[idx++] = TOF_FRAME_VERSION;
  frame_buf[idx++] = TOF_FRAME_TYPE_ERROR;

  frame_buf[idx++] = (uint8_t)(TofFrameSequence & 0xFFU);
  frame_buf[idx++] = (uint8_t)((TofFrameSequence >> 8) & 0xFFU);

  uint32_t timestamp_ms = HAL_GetTick();
  frame_buf[idx++] = (uint8_t)(timestamp_ms & 0xFFU);
  frame_buf[idx++] = (uint8_t)((timestamp_ms >> 8) & 0xFFU);
  frame_buf[idx++] = (uint8_t)((timestamp_ms >> 16) & 0xFFU);
  frame_buf[idx++] = (uint8_t)((timestamp_ms >> 24) & 0xFFU);

  frame_buf[idx++] = sensor_id;
  frame_buf[idx++] = 0U; /* resolution_code not used in error frame */
  frame_buf[idx++] = 0U; /* zone_count not used in error frame */
  frame_buf[idx++] = 0U; /* target_index not used in error frame */
  frame_buf[idx++] = (uint8_t)(TOF_ERROR_PAYLOAD_SIZE & 0xFFU);
  frame_buf[idx++] = (uint8_t)((TOF_ERROR_PAYLOAD_SIZE >> 8) & 0xFFU);

  frame_buf[idx++] = error_code;
  frame_buf[idx++] = (uint8_t)(error_value & 0xFFU);
  frame_buf[idx++] = (uint8_t)((error_value >> 8) & 0xFFU);
  frame_buf[idx++] = (uint8_t)((error_value >> 16) & 0xFFU);
  frame_buf[idx++] = (uint8_t)((error_value >> 24) & 0xFFU);

  uint16_t crc = tof_crc16_ccitt_false(&frame_buf[2], idx - 2U);
  frame_buf[idx++] = (uint8_t)(crc & 0xFFU);
  frame_buf[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

  if (HAL_UART_Transmit(&hcom_uart[COM1], frame_buf, (uint16_t)idx, TOF_UART_TX_TIMEOUT_MS) == HAL_OK)
  {
    TofFrameSequence++;
  }
}

static uint16_t tof_crc16_ccitt_false(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFU;

  for (uint32_t i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static uint8_t tof_resolution_code_from_zones(uint32_t zone_count)
{
  if (zone_count == 16U)
  {
    return 1U; /* 4x4 */
  }

  if (zone_count == 64U)
  {
    return 2U; /* 8x8 */
  }

  return 0U; /* unknown/custom */
}

static void toggle_resolution(void)
{
  for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
  {
    if (SensorActive[instance] == 0U)
    {
      continue;
    }

    CUSTOM_RANGING_SENSOR_Stop(instance);
  }

  switch (Profile.RangingProfile)
  {
    case RS_PROFILE_4x4_AUTONOMOUS:
      Profile.RangingProfile = RS_PROFILE_8x8_AUTONOMOUS;
      break;

    case RS_PROFILE_4x4_CONTINUOUS:
      Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
      break;

    case RS_PROFILE_8x8_AUTONOMOUS:
      Profile.RangingProfile = RS_PROFILE_4x4_AUTONOMOUS;
      break;

    case RS_PROFILE_8x8_CONTINUOUS:
      Profile.RangingProfile = RS_PROFILE_4x4_CONTINUOUS;
      break;

    default:
      break;
  }

  for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
  {
    if (SensorActive[instance] == 0U)
    {
      continue;
    }

    CUSTOM_RANGING_SENSOR_ConfigProfile(instance, &Profile);
    status = CUSTOM_RANGING_SENSOR_Start(instance, RS_MODE_ASYNC_CONTINUOUS);

    if (status != BSP_ERROR_NONE)
    {
      printf("CUSTOM_RANGING_SENSOR_Start failed during toggle on instance %lu\n",
             (unsigned long)instance);
      tof_send_error_frame((uint8_t)instance, TOF_ERROR_SENSOR_OFFLINE, (uint32_t)status);
      deactivate_sensor(instance);
    }
  }

  if (ActiveSensorCount == 0U)
  {
    tof_fatal_loop("All VL53L8CX sensors offline after resolution toggle");
  }

  reset_current_sensor_index();
}

static void toggle_signal_and_ambient(void)
{
  for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
  {
    if (SensorActive[instance] == 0U)
    {
      continue;
    }

    CUSTOM_RANGING_SENSOR_Stop(instance);
  }

  Profile.EnableAmbient = (Profile.EnableAmbient) ? 0U : 1U;
  Profile.EnableSignal = (Profile.EnableSignal) ? 0U : 1U;

  for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
  {
    if (SensorActive[instance] == 0U)
    {
      continue;
    }

    CUSTOM_RANGING_SENSOR_ConfigProfile(instance, &Profile);
    status = CUSTOM_RANGING_SENSOR_Start(instance, RS_MODE_ASYNC_CONTINUOUS);

    if (status != BSP_ERROR_NONE)
    {
      printf("CUSTOM_RANGING_SENSOR_Start failed during toggle on instance %lu\n",
             (unsigned long)instance);
      tof_send_error_frame((uint8_t)instance, TOF_ERROR_SENSOR_OFFLINE, (uint32_t)status);
      deactivate_sensor(instance);
    }
  }

  if (ActiveSensorCount == 0U)
  {
    tof_fatal_loop("All VL53L8CX sensors offline after signal/ambient toggle");
  }

  reset_current_sensor_index();
}

static void display_commands_banner(uint32_t Instance)
{
  /* Avoid terminal escape sequences while debugging */
  printf("\r\n--- VL53L8CX Simple Ranging (sensor %lu/%u) ---\r\n",
         (unsigned long)(Instance + 1U), (unsigned int)TOF_SENSOR_COUNT);

  if (SensorActive[Instance] != 0U)
  {
    uint16_t address = 0U;
    if (CUSTOM_RANGING_SENSOR_GetAddress(Instance, &address) == BSP_ERROR_NONE)
    {
      printf("I2C address  : 0x%02X\r\n", (unsigned int)address);
    }
  }

  printf("Output mode  : %s\r\n", (TofBinaryOutputEnabled != 0U) ? "binary frame" : "text debug");
  printf("Commands: r=resolution, s=signal/ambient, o=toggle output, c=clear\r\n\r\n");
}

static void handle_cmd(uint8_t cmd)
{
  switch (cmd)
  {
    case 'r':
      toggle_resolution();
      break;

    case 's':
      toggle_signal_and_ambient();
      break;

    case 'c':
      printf("\r\n[TOF] Manual clear requested\r\n");
      break;

    case 'o':
      TofBinaryOutputEnabled = (TofBinaryOutputEnabled != 0U) ? 0U : 1U;
      printf("\r\n[TOF] Output mode switched to: %s (tx_fail=%lu)\r\n",
             (TofBinaryOutputEnabled != 0U) ? "binary frame" : "text debug",
             (unsigned long)TofUartTxFailCount);
      display_commands_banner(CurrentSensorIdx);
      break;

    default:
      break;
  }
}

static uint8_t get_key(void)
{
  uint8_t cmd = 0;

  HAL_UART_Receive(&hcom_uart[COM1], &cmd, 1, HAL_MAX_DELAY);

  return cmd;
}

static uint32_t com_has_data(void)
{
  return __HAL_UART_GET_FLAG(&hcom_uart[COM1], UART_FLAG_RXNE);
}

static uint32_t get_first_active_sensor(void)
{
  for (uint32_t instance = 0U; instance < TOF_SENSOR_COUNT; instance++)
  {
    if (SensorActive[instance] != 0U)
    {
      return instance;
    }
  }

  return TOF_SENSOR_COUNT;
}

static uint32_t get_next_active_sensor(uint32_t current_idx)
{
  if (ActiveSensorCount == 0U)
  {
    return TOF_SENSOR_COUNT;
  }

  uint32_t next = current_idx;

  for (uint32_t attempt = 0U; attempt < TOF_SENSOR_COUNT; attempt++)
  {
    next = (next + 1U) % TOF_SENSOR_COUNT;

    if (SensorActive[next] != 0U)
    {
      return next;
    }
  }

  return current_idx;
}

static void deactivate_sensor(uint32_t instance)
{
  if (instance >= TOF_SENSOR_COUNT)
  {
    return;
  }

  if (SensorActive[instance] != 0U)
  {
    SensorActive[instance] = 0U;

    if (ActiveSensorCount > 0U)
    {
      ActiveSensorCount--;
    }
  }
}

static void reset_current_sensor_index(void)
{
  uint32_t first = get_first_active_sensor();

  if (first < TOF_SENSOR_COUNT)
  {
    CurrentSensorIdx = first;
  }
}

static void tof_fatal_loop(const char *msg)
{
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    if (msg != NULL)
    {
      printf("[TOF][FATAL] %s\r\n", msg);
    }
    HAL_Delay(500);
  }
}

#ifdef USE_BARE_DRIVER
static uint8_t map_target_status(uint8_t status)
{
  uint8_t ret;

  if ((status == 5U) || (status == 9U))
  {
    ret = 0U; /* ranging is OK */
  }
  else if (status == 0U)
  {
    ret = 255U; /* no update */
  }
  else
  {
    ret = status; /* return device status otherwise */
  }

  return ret;
}

static int32_t convert_data_format(VL53L8CX_Object_t *pObj,
                                   VL53L8CX_ResultsData *data, RANGING_SENSOR_Result_t *pResult)
{
  int32_t ret;
  uint8_t i, j;
  uint8_t resolution;
  uint8_t target_status;

  if ((pObj == NULL) || (pResult == NULL))
  {
    ret = VL53L8CX_INVALID_PARAM;
  }
  else if (vl53l8cx_get_resolution(&pObj->Dev, &resolution) != VL53L8CX_STATUS_OK)
  {
    ret = VL53L8CX_ERROR;
  }
  else
  {
    pResult->NumberOfZones = resolution;

    for (i = 0; i < resolution; i++)
    {
      pResult->ZoneResult[i].NumberOfTargets = data->nb_target_detected[i];

      for (j = 0; j < data->nb_target_detected[i]; j++)
      {
        pResult->ZoneResult[i].Distance[j] = (uint32_t)data->distance_mm[(VL53L8CX_NB_TARGET_PER_ZONE * i) + j];

        /* return Ambient value if ambient rate output is enabled */
        if (pObj->IsAmbientEnabled == 1U)
        {
          /* apply ambient value to all targets in a given zone */
          pResult->ZoneResult[i].Ambient[j] = (float_t)data->ambient_per_spad[i];
        }
        else
        {
          pResult->ZoneResult[i].Ambient[j] = 0.0f;
        }

        /* return Signal value if signal rate output is enabled */
        if (pObj->IsSignalEnabled == 1U)
        {
          pResult->ZoneResult[i].Signal[j] =
            (float_t)data->signal_per_spad[(VL53L8CX_NB_TARGET_PER_ZONE * i) + j];
        }
        else
        {
          pResult->ZoneResult[i].Signal[j] = 0.0f;
        }

        target_status = data->target_status[(VL53L8CX_NB_TARGET_PER_ZONE * i) + j];
        pResult->ZoneResult[i].Status[j] = map_target_status(target_status);
      }
    }

    ret = VL53L8CX_OK;
  }

  return ret;
}
#endif

#ifdef __cplusplus
}
#endif
