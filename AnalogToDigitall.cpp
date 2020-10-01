#include <stm32f1xx_hal.h>
#include <stm32_hal_legacy.h>
#include <arm_math.h>

#define MAX_FFT_SAMPLES			64

#define FFT_FORWARD			  0x01

#define twoPi 6.28318531
#define fourPi 12.56637061
#define sixPi 18.84955593

#ifdef __cplusplus
extern "C"
#endif
	
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
	
void SystemClock_Config(void);
static void MX_ADC1_Init(void);

void FFT_Windowing(double *vData, uint16_t samples, uint8_t dir);
void FFT_Compute(double *vReal, double *vImag, uint16_t samples, uint16_t dir);
uint8_t FFT_Exponent(uint16_t value);
void FFT_Swap(double *x, double *y);

ADC_HandleTypeDef hadc1;

/*-------------------------------------- FFT Variabiles --------------------------------------*/
double vReal[MAX_FFT_SAMPLES];
double vImg[MAX_FFT_SAMPLES];

/*-------------------------------------- FFT Variabiles --------------------------------------*/

void Error_Handler()
{
	
}

uint32_t ADC_ReadConversion()
{
	uint32_t val;
	
	HAL_ADC_Start(&hadc1);
	while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) == RESET);
	val = HAL_ADC_GetValue(&hadc1);
	__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
	HAL_ADC_Stop(&hadc1);
	
	return val;
}


void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;    // 12 MHZ ADC
	
	
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = { 0 };
	GPIO_InitTypeDef gpio = { 0 };
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Pin = GPIO_PIN_0;
	gpio.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &gpio);
	
	__HAL_RCC_ADC1_CLK_ENABLE();
	
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel 
	*/
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

int main(void)
{
	HAL_Init();

	SystemClock_Config();
	MX_ADC1_Init();
	
	
	__GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_13;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	
	for (;;)
	{
		for (uint8_t i = 0; i < MAX_FFT_SAMPLES; i++)
		{
			vImg[i] = 0;
			vReal[i] = ADC_ReadConversion();
			HAL_Delay(2);
		}
		
		
		
		
	}
}

void FFT_Windowing(double *vData, uint16_t samples, uint8_t dir)
{
	double samplesMinusOne = (double)samples - 1.0;
	
	for (uint16_t i = 0; i < (samples >> 1); i++)
	{
		double indexMinusOne = (double)i;
		double ratio = (indexMinusOne / samplesMinusOne);
		double weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
		
		if (dir == FFT_FORWARD)
		{
			vData[i] *= weighingFactor;
			vData[samples - (i + 1)] *= weighingFactor;
		}
	}
}

void FFT_Compute(double *vReal, double *vImag, uint16_t samples, uint16_t dir)
{
	uint8_t power = FFT_Exponent(samples);
	
	uint16_t j = 0;
	
	for (uint16_t i = 0; i < (samples - 1); i++) 
	{
		if (i < j) 
		{
			FFT_Swap(&vReal[i], &vReal[j]);
			//if (dir == FFT_REVERSE)
				//Swap(&vImag[i], &vImag[j]);
		}
		
		uint16_t k = (samples >> 1);
		while (k <= j) 
		{
			j -= k;
			k >>= 1;
		}
		
		j += k;
	}
	
	double c1 = -1.0;
	double c2 = 0.0;
	uint16_t l2 = 1;
	
	for (uint8_t l = 0; (l < power); l++)
	{
		uint16_t l1 = l2;
		l2 <<= 1;
		double u1 = 1.0;
		double u2 = 0.0;
		for (j = 0; j < l1; j++) 
		{
			for (uint16_t i = j; i < samples; i += l2) 
			{
				uint16_t i1 = i + l1;
				double t1 = u1 * vReal[i1] - u2 * vImag[i1];
				double t2 = u1 * vImag[i1] + u2 * vReal[i1];
				vReal[i1] = vReal[i] - t1;
				vImag[i1] = vImag[i] - t2;
				vReal[i] += t1;
				vImag[i] += t2;
			}
			
			double z = ((u1 * c1) - (u2 * c2));
			u2 = ((u1 * c2) + (u2 * c1));
			u1 = z;
		}
		
		c2 = sqrt((1.0 - c1) / 2.0);
		if (dir == FFT_FORWARD) 
		{
			c2 = -c2;
		}
		c1 = sqrt((1.0 + c1) / 2.0);
	}
	// Scaling for reverse transform
	if(dir != FFT_FORWARD) 
	{
		for (uint16_t i = 0; i < samples; i++)
		{
			vReal[i] /= samples;
			vImag[i] /= samples;
		}
	}
}

uint8_t FFT_Exponent(uint16_t value)
{
	uint8_t result = 0;
	while (((value >> result) & 1) != 1) result++;
	return (result);
}

void FFT_Swap(double *x, double *y)
{
	double temp = *x;
	*x = *y;
	*y = temp;
}
