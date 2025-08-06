/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MH-Z19C CO2 Sensor (PWM 방식) 측정 코드
  * @details        : TIM2 채널1 (PA0) 입력 캡처로 PWM 측정, UART 출력
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f4xx_hal_pwr_ex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define AVG_CNT     10
#define PCF8591_ADDR (0x48 << 1)

#define CAL_A       1.25f
#define CAL_B       0.015f
#define DEDT        0.005f
#define THER_INIT   25.0f
#define DHT11_PORT GPIOA
#define DHT11_PIN  GPIO_PIN_1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t rx_byte;                // UART 수신 바이트 저장용
static uint8_t rx_state = 0;
/* Buffers for moving average */
static float emf_buf[AVG_CNT];
static float ther_buf[AVG_CNT];
static uint8_t avg_idx = 0;
static bool avg_filled = false;

/*디지털 온도센서 */
static void DHT11_Start(void);
static uint8_t DHT11_CheckResponse(void);
static uint8_t DHT11_ReadByte(void);
static float DHT11_ReadTemp(void);

/* PWM CO2 (MH-Z19C) */
volatile uint32_t ic_rising = 0;
volatile uint32_t ic_falling = 0;
volatile uint8_t  edge_flag = 0; // 0: rising 대기, 1: falling 대기
volatile uint32_t high_time = 0;
volatile uint32_t period = 1004; // 기본 주기(ms)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static uint8_t PCF8591_ReadChannel(I2C_HandleTypeDef *hi2c, uint8_t ch);
static float adc8_to_mv(uint8_t v);
static float emf_mv_to_emf(float mv);
static float ntc_calc_celsius(uint8_t ther_raw);
static void push_avg(float emf_mv, float temp_c, float *emf_avg, float *temp_avg);
static float calc_ppm(float emf_avg, float temp_avg);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t PCF8591_ReadChannel(I2C_HandleTypeDef *hi2c, uint8_t ch) {
    uint8_t cmd = 0x40 | (ch & 0x03);
    uint8_t dummy, val;
    HAL_I2C_Master_Transmit(hi2c, PCF8591_ADDR, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(2);
    HAL_I2C_Master_Receive(hi2c, PCF8591_ADDR, &dummy, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, PCF8591_ADDR, &val,   1, HAL_MAX_DELAY);
    return val;
}

/* UART 수신 인터럽트 콜백 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    	 printf("받은 명령: %c\n", rx_byte);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // LED OFF

        if (rx_state == 0 && rx_byte == 'L') {
            rx_state = 1;
        } else if (rx_state == 1) {
            if (rx_byte == '1') {
                printf("LED_ON\n");
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // LED ON
            } else if (rx_byte == '0') {
                printf("LED_OFF\n");
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // LED OFF
            }
            rx_state = 0;
        } else {
            rx_state = 0;
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  // 다음 수신 예약
        printf("Receive_IT called!");
    }

}

static inline float adc8_to_mv(uint8_t v) {
    return (v / 255.0f) * 5000.0f;
}
static inline float emf_mv_to_emf(float mv) {
    return mv / 6.0f;
}
static float ntc_calc_celsius(uint8_t ther_raw) {
    const float C1 = 0.00230088f, C2 = 0.000224f, C3 = 0.000000021133f;
    const float R0 = 15.0f;
    float v = (float)ther_raw;
    float r = (R0 * v) / (255.0f - v);
    float t = 1.0f / (C1 + C2 * log(r) + C3 * pow(log(r), 3)) - 273.15f;
    return t;
}
static void push_avg(float emf_mv, float temp_c, float *emf_avg, float *temp_avg) {
    emf_buf[avg_idx] = emf_mv;
    ther_buf[avg_idx] = temp_c;
    avg_idx = (avg_idx + 1) % AVG_CNT;
    if (avg_idx == 0) avg_filled = true;
    uint8_t n = avg_filled ? AVG_CNT : avg_idx;
    float e_sum = 0, t_sum = 0;
    for (uint8_t i = 0; i < n; ++i) {
        e_sum += emf_buf[i];
        t_sum += ther_buf[i];
    }
    *emf_avg = e_sum / n;
    *temp_avg = t_sum / n;
}
float calc_ppm(float emf_avg, float temp_avg) {
    float cal = CAL_A - (emf_avg + DEDT * (THER_INIT - temp_avg));
    return powf(10.0f, cal / CAL_B);
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



  /* USER CODE BEGIN 2 */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /*디지털 온도 센서*/
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  char buf[100];
  // PA5 LED 수동 초기화
   printf("USART1 State: %d\r\n", huart1.gState);
       __HAL_RCC_GPIOA_CLK_ENABLE();
       GPIO_InitTypeDef GPIO_InitStruct = {0};
       GPIO_InitStruct.Pin = GPIO_PIN_6;
       GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
       HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

       // UART 인터럽트 수신 시작
       HAL_StatusTypeDef result = HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
       if (result != HAL_OK) {
           printf("❌ Receive_IT 등록 실패! result = %d\r\n", result);  // 1 = BUSY, 2 = ERROR
       } else {
           printf("✅ Receive_IT 등록 성공!\r\n");
       }

       // NVIC 설정 (중요!)
       HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
       HAL_NVIC_EnableIRQ(USART1_IRQn);
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
       HAL_Delay(1000);
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_I2C_IsDeviceReady(&hi2c1, PCF8591_ADDR, 3, 100) != HAL_OK) {
			  HAL_UART_Transmit(&huart2, (uint8_t *)"I2C FAIL\r\n", 10, HAL_MAX_DELAY);
			  HAL_Delay(500);
			  continue;
	 	  }
	  // --- I2C CO₂, 온도 ---
	  float co2_pwm = 0;
	  if (high_time > 0 && period > 4000) {
	              co2_pwm = 2000.0f * ((float)high_time - 2000.0f) / ((float)period - 4000.0f);
	              high_time = 0;
	          }

	          uint8_t ther_raw = PCF8591_ReadChannel(&hi2c1, 1);
	          uint8_t emf_raw = PCF8591_ReadChannel(&hi2c1, 2);

	          float emf_mv = emf_mv_to_emf(adc8_to_mv(emf_raw));
	          float temp_c = ntc_calc_celsius(ther_raw);
	          float emf_avg, temp_avg;
	          push_avg(emf_mv, temp_c, &emf_avg, &temp_avg);
	          float co2_analog = calc_ppm(emf_avg, temp_avg);
	          float temp_dht = DHT11_ReadTemp();

	          float moist_percent = (750 - emf_avg) * 100 / 350;
	          if (moist_percent > 100) moist_percent = 100;
	          if (moist_percent < 0)   moist_percent = 0;

	          int len = snprintf(buf, sizeof(buf), "%.0f|%.1f|%.1f\r\n", co2_pwm, moist_percent, temp_dht);
	          HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
	          HAL_Delay(1000);
  }

}
	/* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if (edge_flag == 0) // rising edge
    {
      ic_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
      edge_flag = 1;
    }
    else // falling edge
    {
      ic_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      high_time = (ic_falling >= ic_rising) ? (ic_falling - ic_rising) : ((htim->Instance->ARR - ic_rising) + ic_falling);

      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      edge_flag = 0;

      static uint32_t last_rising = 0;
      period = (ic_rising >= last_rising) ? (ic_rising - last_rising) : ((htim->Instance->ARR - last_rising) + ic_rising);
      last_rising = ic_rising;
      //printf("rise: %lu, fall: %lu, high_time: %lu, period: %lu\r\n", ic_rising, ic_falling, high_time, period);

    }
  }
}

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/*디지털 온도센서*/
void delay_us(uint16_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < cycles);
}

void DHT11_Start(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Output
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);  // at least 18ms
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);   // 20~40us

    // Input
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

uint8_t DHT11_CheckResponse(void) {
    delay_us(40);
    if (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
        delay_us(80);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) {
            delay_us(80);
            return 1;
        }
    }
    return 0;
}

uint8_t DHT11_ReadByte(void) {
    uint8_t value = 0;
    for (int i = 0; i < 8; i++) {
        while (!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)); // LOW wait
        delay_us(40);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
            value |= (1 << (7 - i));
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));  // HIGH wait
    }
    return value;
}

float DHT11_ReadTemp(void) {
    DHT11_Start();
    if (DHT11_CheckResponse()) {
        DHT11_ReadByte(); // 습도 정수
        DHT11_ReadByte(); // 습도 소수
        uint8_t temp_int = DHT11_ReadByte();
        uint8_t temp_dec = DHT11_ReadByte();  // ← 여기서 소수점 읽기
        DHT11_ReadByte(); // checksum
        return (float)temp_int + (float)temp_dec * 0.1f;
    }
    return -100.0f;
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

#ifdef  USE_FULL_ASSERT
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
