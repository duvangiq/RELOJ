/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "uart.h"
#include "MPU6050.h"
#include "dsp_effects.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STREAM_PACKET_SIZE 529U
#define STREAM_ADC_OFFSET 5U
#define STREAM_SENSOR_OFFSET 517U
#define STREAM_CRC_OFFSET 527U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// Buffer circular de 512 muestras de guitarra para DMA (256 mitad 1 + 256 mitad 2)
uint16_t adc_buf[512];
static uint8_t stream_packet[2][STREAM_PACKET_SIZE];
static uint16_t stream_sequence = 0;
volatile uint32_t adc_half_events = 0;
volatile uint32_t adc_full_events = 0;
volatile uint32_t uart2_dma_start_errors = 0;

// Inclinación medida con la MPU6050 (grados)
float inclinacion_roll;   // Inclinación lateral (eje X)
float inclinacion_pitch;  // Inclinación frontal (eje Y)

// 1 si la MPU6050 respondió y quedó inicializada
uint8_t mpu_ok = 0;
// Dirección I2C detectada (0x68 si AD0 = GND, 0x69 si AD0 = VCC); 0 = no encontrada
uint8_t mpu_addr = 0;
char buf[64];
uint8_t rx_cmd_byte = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

static uint8_t last_scanned_addr = 0;
static uint16_t last_i2c_status = 0;

static void MPU6050_ResetBus(void)
{
  __HAL_RCC_I2C1_FORCE_RESET();
  for (volatile int d = 0; d < 1000; d++);
  __HAL_RCC_I2C1_RELEASE_RESET();

  // Configurar PB6 (SCL) como salida OD con pullup y PB7 (SDA) como entrada con pullup
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Generar 16 pulsos en SCL para destrabar el sensor si quedó colgado enviando un bit 0
  for (int i = 0; i < 16; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 500; d++);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    for (volatile int d = 0; d < 500; d++);
  }

  // Restaurar pines a modo alterno I2C1
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_I2C_Init(&hi2c1);
}

// Busca la MPU6050 en el bus I2C (0x68 o 0x69). Devuelve la dirección de 7 bits o 0.
static uint8_t MPU6050_Buscar(void)
{
  HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, 0x68 << 1, 1, 2);
  last_i2c_status = (uint16_t)st;
  if (st == HAL_OK) return 0x68;

  st = HAL_I2C_IsDeviceReady(&hi2c1, 0x69 << 1, 1, 2);
  last_i2c_status = (uint16_t)st;
  if (st == HAL_OK) return 0x69;

  return 0;
}

static void Stream_PutInt16(uint8_t *dest, float value, float scale)
{
  int32_t scaled = 0;
  if (isfinite(value)) {
    float scaled_value = value * scale;
    if (scaled_value > 32767.0f) scaled_value = 32767.0f;
    if (scaled_value < -32768.0f) scaled_value = -32768.0f;
    scaled = (int32_t)lroundf(scaled_value);
  }

  dest[0] = (uint8_t)(scaled & 0xFF);
  dest[1] = (uint8_t)((uint32_t)scaled >> 8);
}

static uint16_t Stream_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void Stream_PreparePacket(uint8_t *packet, const uint16_t *samples)
{
  packet[0] = 0xA5U;
  packet[1] = 0x5AU;
  packet[2] = (uint8_t)(stream_sequence & 0xFFU);
  packet[3] = (uint8_t)(stream_sequence >> 8);
  stream_sequence++;
  // Byte 4: Bit 0 = MPU OK, Bits 4..7 = Efecto activo (0: Clean, 1: Overdrive, 2: Distortion, 3: Tremolo)
  packet[4] = (mpu_ok ? 1U : 0U) | ((uint8_t)(dsp_engine.active_effect & 0x0FU) << 4);
  memcpy(&packet[STREAM_ADC_OFFSET], samples, 512U);

  if (mpu_ok) {
    Stream_PutInt16(&packet[STREAM_SENSOR_OFFSET], inclinacion_roll, 100.0f);
    Stream_PutInt16(&packet[STREAM_SENSOR_OFFSET + 2U], inclinacion_pitch, 100.0f);
    Stream_PutInt16(&packet[STREAM_SENSOR_OFFSET + 4U], MPU6050.ace_x_f, 1000.0f);
    Stream_PutInt16(&packet[STREAM_SENSOR_OFFSET + 6U], MPU6050.ace_y_f, 1000.0f);
    Stream_PutInt16(&packet[STREAM_SENSOR_OFFSET + 8U], MPU6050.ace_z_f, 1000.0f);
  } else {
    packet[STREAM_SENSOR_OFFSET]     = (uint8_t)(last_i2c_status & 0xFF);
    packet[STREAM_SENSOR_OFFSET + 1] = (uint8_t)(last_i2c_status >> 8);
    packet[STREAM_SENSOR_OFFSET + 2] = (uint8_t)(hi2c1.ErrorCode & 0xFF);
    packet[STREAM_SENSOR_OFFSET + 3] = (uint8_t)(hi2c1.ErrorCode >> 8);
    packet[STREAM_SENSOR_OFFSET + 4] = (uint8_t)hi2c1.State;
    packet[STREAM_SENSOR_OFFSET + 5] = (uint8_t)last_scanned_addr;
    packet[STREAM_SENSOR_OFFSET + 6] = (uint8_t)((GPIOB->IDR >> 6) & 0x03);
    packet[STREAM_SENSOR_OFFSET + 7] = 0;
    packet[STREAM_SENSOR_OFFSET + 8] = 0;
    packet[STREAM_SENSOR_OFFSET + 9] = 0;
  }

  uint16_t crc = Stream_Crc16(&packet[2], STREAM_CRC_OFFSET - 2U);
  packet[STREAM_CRC_OFFSET] = (uint8_t)(crc & 0xFFU);
  packet[STREAM_CRC_OFFSET + 1U] = (uint8_t)(crc >> 8);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  char buffer[128];
  uint32_t ultimo_diag = HAL_GetTick();

  // La MPU6050 necesita ~100 ms tras energizarse antes de aceptar comandos
  HAL_Delay(100);

  // Diagnóstico: ¿responde algo en el bus I2C?
  mpu_addr = MPU6050_Buscar();
  if (mpu_addr == 0) {
    MPU6050_ResetBus();
    mpu_addr = MPU6050_Buscar();
  }
  if (mpu_addr == 0) {
    uartx_write_text(&huart1, "ERROR: MPU6050 no responde en I2C (0x68/0x69). Revisar SDA/SCL, VCC, GND y pull-ups.\r\n");
  } else {
    sprintf(buffer, "MPU6050 detectada en 0x%02X\r\n", mpu_addr);
    uartx_write_text(&huart1, buffer);
  }

  // Inicializar MPU6050
  if (mpu_addr != 0 && MPU6050_init() == 1) {
    MPU6050_Calibra();            // Calibración inicial en reposo (no mover el sensor)
    Kalman_Init(&kalman_roll_);   // Inicializar filtro de Roll
    Kalman_Init(&kalman_pitch_);  // Inicializar filtro de Pitch
    mpu_ok = 1;
  } else {
    uartx_write_text(&huart1, "ERROR: MPU6050_init() fallo. Roll/Pitch se mostraran en 0.\r\n");
  }

  // Inicializar motor DSP de efectos
  DSP_Init();

  // Habilitar recepcion no bloqueante por interrupcion en UART2 para comandos Wi-Fi
  HAL_UART_Receive_IT(&huart2, &rx_cmd_byte, 1);

  // Armar el ADC y DMA antes de habilitar el disparo periódico de TIM3.
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 512) != HAL_OK) {
    uartx_write_text(&huart1, "ERROR: no se pudo iniciar ADC1 con DMA.\r\n");
  } else if (HAL_TIM_Base_Start(&htim3) != HAL_OK) {
    uartx_write_text(&huart1, "ERROR: no se pudo iniciar TIM3 para el ADC.\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (mpu_ok) {
      TIMER_PROCEDIMIENTOS(); // Lee registros I2C y calcula inclinación

      // Si el filtro se corrompe (NaN/inf), se reinicia en vez de arrastrar el NaN para siempre
      if (!isfinite(kalman_roll) || !isfinite(kalman_pitch)) {
        Kalman_Init(&kalman_roll_);
        Kalman_Init(&kalman_pitch_);
      }

      // Toma el ángulo filtrado si es válido; si no, el directo del acelerómetro
      inclinacion_roll  = isfinite(kalman_roll)  ? kalman_roll  : acel_roll;
      inclinacion_pitch = isfinite(kalman_pitch) ? kalman_pitch : acel_pitch;

      if (!isfinite(inclinacion_roll))  inclinacion_roll  = 0.0f;
      if (!isfinite(inclinacion_pitch)) inclinacion_pitch = 0.0f;

      // Modula los parámetros del efecto activo según la inclinación de la guitarra
      DSP_UpdateFromMPU(inclinacion_roll, inclinacion_pitch);
    } else {
      inclinacion_roll  = 0.0f;
      inclinacion_pitch = 0.0f;
      DSP_UpdateFromMPU(0.0f, 0.0f);

      // Reintento automático para detectar la MPU si se energizó o conectó después
      static uint32_t ultimo_mpu_retry = 0;
      if (HAL_GetTick() - ultimo_mpu_retry >= 1000) {
        ultimo_mpu_retry = HAL_GetTick();
        MPU6050_ResetBus();
        mpu_addr = MPU6050_Buscar();
        if (mpu_addr != 0 && MPU6050_init() == 1) {
          MPU6050_Calibra();
          Kalman_Init(&kalman_roll_);
          Kalman_Init(&kalman_pitch_);
          mpu_ok = 1;
        }
      }
    }

    // Lectura DIRECTA por registros hardware de USART2 
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
      uint8_t c = (uint8_t)(USART2->DR & 0xFF);
      if (c == 'C' || c == '0') DSP_SetEffect(EFFECT_CLEAN);
      else if (c == 'O' || c == '1') DSP_SetEffect(EFFECT_OVERDRIVE);
      else if (c == 'D' || c == '2') DSP_SetEffect(EFFECT_DISTORTION);
      else if (c == 'T' || c == '3') DSP_SetEffect(EFFECT_TREMOLO);
    }
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG(&huart2);
    }

    // Fila de estado para la consola (solo 1 vez por segundo en diagnostico para no frenar la CPU)
    if (HAL_GetTick() - ultimo_diag >= 1000) {
      int len = snprintf(buffer, sizeof(buffer),
          "ADC H:%lu F:%lu TXerr:%lu U2:%u\r\n",
          (unsigned long)adc_half_events,
          (unsigned long)adc_full_events,
          (unsigned long)uart2_dma_start_errors,
          (unsigned int)HAL_UART_GetState(&huart2));
      HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);
      ultimo_diag = HAL_GetTick();
    }

    HAL_Delay(10); // 100 Hz de refresco para el MPU-6050 (reacción instantánea en tiempo real)

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Callbacks para streaming continuo de guitarra por DMA a USART2
// Se transmite 1 de cada 4 paquetes (~10 paquetes/segundo) para que el módulo Wi-Fi DT-06
// no sature su buffer TCP y la señal llegue 100% fluida y estable.
static uint8_t wifi_decimate = 0;
static uint16_t dsp_out_buf[256];

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_half_events++;
    wifi_decimate++;
    if (wifi_decimate >= 4)
    {
      wifi_decimate = 0;
      // Aplica el efecto DSP en tiempo real sobre las 256 muestras
      DSP_ProcessBlock(&adc_buf[0], dsp_out_buf, 256);
      Stream_PreparePacket(stream_packet[0], dsp_out_buf);
      if (HAL_UART_Transmit_DMA(&huart2, stream_packet[0], STREAM_PACKET_SIZE) != HAL_OK) {
        uart2_dma_start_errors++;
      }
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_full_events++;
  }
}

// Callback de interrupción de UART2: se ejecuta al instante cuando llega un comando Wi-Fi
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (rx_cmd_byte == 'C' || rx_cmd_byte == '0') DSP_SetEffect(EFFECT_CLEAN);
    else if (rx_cmd_byte == 'O' || rx_cmd_byte == '1') DSP_SetEffect(EFFECT_OVERDRIVE);
    else if (rx_cmd_byte == 'D' || rx_cmd_byte == '2') DSP_SetEffect(EFFECT_DISTORTION);
    else if (rx_cmd_byte == 'T' || rx_cmd_byte == '3') DSP_SetEffect(EFFECT_TREMOLO);

    // Re-armar para el siguiente byte
    HAL_UART_Receive_IT(&huart2, &rx_cmd_byte, 1);
  }
}

// Recuperar el receptor si ocurre un error de Overrun (ORE) o ruido
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    HAL_UART_Receive_IT(&huart2, &rx_cmd_byte, 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
