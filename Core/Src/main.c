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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ff.h"

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint16_t channels;
    uint32_t sampleRate;
    uint16_t bitsPerSample;
    DWORD dataSize;
} WavInfo;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_FRAMES     4096
#define AUDIO_CHANNELS   2
#define AUDIO_SAMPLES    (AUDIO_FRAMES * AUDIO_CHANNELS)
#define AUDIO_HALF_FRAMES  (AUDIO_FRAMES / 2)
#define AUDIO_HALF_SAMPLES (AUDIO_SAMPLES / 2)
#define WAV_FILE_NAME    "collectathon.wav"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
__attribute__((section(".RAM_D2"), aligned(32)))
static int16_t g_audioBuffer[AUDIO_SAMPLES];

static FATFS g_fatfs;

static FIL g_audioFile;
static WavInfo g_wavInfo;
static DWORD g_wavDataRemaining;
static uint8_t g_wavReadBuf[1024];
static uint16_t g_volumeQ15 = 983U;
static volatile uint8_t g_fillFirstHalf;
static volatile uint8_t g_fillSecondHalf;
static uint8_t g_audioFileOpen;
static uint8_t g_playbackStarted;
static uint8_t g_playbackEofPrinted;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SAI1_Init(void);
static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UartPrint(const char* text)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void UartPrintFResult(const char* prefix, FRESULT fr)
{
    char msg[64];
    snprintf(msg, sizeof(msg), "%s: %d\r\n", prefix, (int)fr);
    UartPrint(msg);
}

static void UartPrintSdStatus(const char* prefix)
{
    char msg[160];
    HAL_SD_CardStateTypeDef cardState = HAL_SD_GetCardState(&hsd1);

    snprintf(msg, sizeof(msg),
             "%s: retSD=%u, SDPath=\"%s\", HAL_State=%u, CardState=%u, ErrorCode=0x%08lX\r\n",
             prefix,
             (unsigned)retSD,
             SDPath,
             (unsigned)HAL_SD_GetState(&hsd1),
             (unsigned)cardState,
             (unsigned long)hsd1.ErrorCode);
    UartPrint(msg);
}

static void ListRootDirectory(void)
{
    DIR dir;
    FILINFO fileInfo;
    FRESULT fr;
    char msg[320];

    UartPrint("Root directory:\r\n");

    fr = f_opendir(&dir, SDPath);
    if (fr != FR_OK)
    {
        UartPrintFResult("f_opendir failed", fr);
        return;
    }

    for (;;)
    {
        fr = f_readdir(&dir, &fileInfo);
        if (fr != FR_OK)
        {
            UartPrintFResult("f_readdir failed", fr);
            break;
        }

        if (fileInfo.fname[0] == '\0')
        {
            break;
        }

        snprintf(msg, sizeof(msg),
                 "%c %10lu  %s\r\n",
                 (fileInfo.fattrib & AM_DIR) ? 'D' : 'F',
                 (unsigned long)fileInfo.fsize,
                 fileInfo.fname);
        UartPrint(msg);
    }

    fr = f_closedir(&dir);
    if (fr != FR_OK)
    {
        UartPrintFResult("f_closedir failed", fr);
    }
}

static uint16_t ReadLe16(const uint8_t* data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t ReadLe32(const uint8_t* data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static int16_t ApplyVolume(int16_t sample)
{
    return (int16_t)(((int32_t)sample * (int32_t)g_volumeQ15) >> 15);
}

static FRESULT ReadExact(FIL* file, void* buffer, UINT bytes)
{
    UINT bytesRead = 0;
    FRESULT fr = f_read(file, buffer, bytes, &bytesRead);
    if (fr != FR_OK)
    {
        return fr;
    }
    return (bytesRead == bytes) ? FR_OK : FR_DISK_ERR;
}

static FRESULT SkipChunkPayload(FIL* file, DWORD chunkSize)
{
    DWORD skipSize = chunkSize + (chunkSize & 1U);
    return f_lseek(file, f_tell(file) + skipSize);
}

static uint32_t WavSampleRateToSaiFrequency(uint32_t sampleRate)
{
    switch (sampleRate)
    {
    case 8000U:
        return SAI_AUDIO_FREQUENCY_8K;
    case 11025U:
        return SAI_AUDIO_FREQUENCY_11K;
    case 16000U:
        return SAI_AUDIO_FREQUENCY_16K;
    case 22050U:
        return SAI_AUDIO_FREQUENCY_22K;
    case 32000U:
        return SAI_AUDIO_FREQUENCY_32K;
    case 44100U:
        return SAI_AUDIO_FREQUENCY_44K;
    case 48000U:
        return SAI_AUDIO_FREQUENCY_48K;
    case 96000U:
        return SAI_AUDIO_FREQUENCY_96K;
    case 192000U:
        return SAI_AUDIO_FREQUENCY_192K;
    default:
        return 0U;
    }
}

static FRESULT ParseWavHeader(FIL* file, WavInfo* info)
{
    uint8_t header[16];
    uint8_t chunkHeader[8];
    uint8_t fmt[16];
    uint8_t fmtFound = 0;
    uint8_t dataFound = 0;

    memset(info, 0, sizeof(*info));

    FRESULT fr = ReadExact(file, header, 12);
    if (fr != FR_OK)
    {
        return fr;
    }

    if (memcmp(&header[0], "RIFF", 4) != 0 || memcmp(&header[8], "WAVE", 4) != 0)
    {
        return FR_INVALID_OBJECT;
    }

    while (!dataFound)
    {
        fr = ReadExact(file, chunkHeader, sizeof(chunkHeader));
        if (fr != FR_OK)
        {
            return fr;
        }

        DWORD chunkSize = ReadLe32(&chunkHeader[4]);

        if (memcmp(chunkHeader, "fmt ", 4) == 0)
        {
            if (chunkSize < sizeof(fmt))
            {
                return FR_INVALID_OBJECT;
            }

            fr = ReadExact(file, fmt, sizeof(fmt));
            if (fr != FR_OK)
            {
                return fr;
            }

            uint16_t audioFormat = ReadLe16(&fmt[0]);
            info->channels = ReadLe16(&fmt[2]);
            info->sampleRate = ReadLe32(&fmt[4]);
            info->bitsPerSample = ReadLe16(&fmt[14]);

            if (audioFormat != 1U ||
                (info->channels != 1U && info->channels != 2U) ||
                info->bitsPerSample != 16U)
            {
                return FR_INVALID_OBJECT;
            }

            fmtFound = 1;

            if (chunkSize > sizeof(fmt))
            {
                fr = f_lseek(file, f_tell(file) + (chunkSize - sizeof(fmt)) + (chunkSize & 1U));
                if (fr != FR_OK)
                {
                    return fr;
                }
            }
            else if (chunkSize & 1U)
            {
                fr = f_lseek(file, f_tell(file) + 1U);
                if (fr != FR_OK)
                {
                    return fr;
                }
            }
        }
        else if (memcmp(chunkHeader, "data", 4) == 0)
        {
            if (!fmtFound)
            {
                return FR_INVALID_OBJECT;
            }

            info->dataSize = chunkSize;
            dataFound = 1;
        }
        else
        {
            fr = SkipChunkPayload(file, chunkSize);
            if (fr != FR_OK)
            {
                return fr;
            }
        }
    }

    return FR_OK;
}

static HAL_StatusTypeDef ConfigureSaiForWav(const WavInfo* info)
{
    uint32_t saiFrequency = WavSampleRateToSaiFrequency(info->sampleRate);
    if (saiFrequency == 0U)
    {
        return HAL_ERROR;
    }

    hsai_BlockA1.Init.AudioFrequency = saiFrequency;
    hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
    return HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, AUDIO_CHANNELS);
}

static void FillAudioBufferHalf(uint32_t halfIndex)
{
    int16_t* dst = &g_audioBuffer[halfIndex * AUDIO_HALF_SAMPLES];
    uint32_t framesToFill = AUDIO_HALF_FRAMES;
    uint32_t frameIndex = 0;
    UINT bytesRead = 0;
    const uint32_t wavFrameBytes = (uint32_t)g_wavInfo.channels * sizeof(int16_t);

    while (framesToFill > 0U && g_audioFileOpen && g_wavDataRemaining >= wavFrameBytes)
    {
        UINT maxFrames = (UINT)(sizeof(g_wavReadBuf) / wavFrameBytes);
        UINT framesThisRead = (framesToFill < maxFrames) ? (UINT)framesToFill : maxFrames;
        DWORD remainingFrames = g_wavDataRemaining / wavFrameBytes;
        if ((DWORD)framesThisRead > remainingFrames)
        {
            framesThisRead = (UINT)remainingFrames;
        }

        UINT bytesToRead = framesThisRead * (UINT)wavFrameBytes;
        FRESULT fr = f_read(&g_audioFile, g_wavReadBuf, bytesToRead, &bytesRead);
        if (fr != FR_OK || bytesRead == 0U)
        {
            g_wavDataRemaining = 0;
            break;
        }

        UINT framesRead = bytesRead / (UINT)wavFrameBytes;
        for (UINT i = 0; i < framesRead; ++i)
        {
            int16_t left = (int16_t)ReadLe16(&g_wavReadBuf[i * wavFrameBytes]);
            int16_t right = left;

            if (g_wavInfo.channels == 2U)
            {
                right = (int16_t)ReadLe16(&g_wavReadBuf[i * wavFrameBytes + 2U]);
            }

            left = ApplyVolume(left);
            right = ApplyVolume(right);

            dst[(frameIndex * AUDIO_CHANNELS) + 0U] = left;
            dst[(frameIndex * AUDIO_CHANNELS) + 1U] = right;
            ++frameIndex;
        }

        g_wavDataRemaining -= bytesRead;
        framesToFill -= framesRead;

        if (framesRead < framesThisRead)
        {
            break;
        }
    }

    if (framesToFill > 0U)
    {
        memset(&dst[frameIndex * AUDIO_CHANNELS], 0, framesToFill * AUDIO_CHANNELS * sizeof(int16_t));
    }

    if (g_audioFileOpen && g_wavDataRemaining < wavFrameBytes)
    {
        f_close(&g_audioFile);
        g_audioFileOpen = 0;
    }
}

static void ProcessAudioPlayback(void)
{
    if (g_fillFirstHalf)
    {
        g_fillFirstHalf = 0;
        FillAudioBufferHalf(0);
    }

    if (g_fillSecondHalf)
    {
        g_fillSecondHalf = 0;
        FillAudioBufferHalf(1);
    }

    if (g_playbackStarted && !g_audioFileOpen && !g_playbackEofPrinted)
    {
        UartPrint("WAV playback reached EOF, outputting silence\r\n");
        g_playbackEofPrinted = 1;
    }
}

static void StartWavPlayback(void)
{
    FRESULT fr;
    char msg[160];
    char filePath[32];

    UartPrint("==== WAV playback start ====\r\n");
    UartPrintSdStatus("Before f_mount");

    fr = f_mount(&g_fatfs, SDPath, 1);
    if (fr != FR_OK)
    {
        UartPrintFResult("f_mount failed", fr);
        UartPrintSdStatus("After f_mount failed");
        return;
    }

    UartPrint("f_mount OK\r\n");
    UartPrintSdStatus("After f_mount OK");
    ListRootDirectory();

    snprintf(filePath, sizeof(filePath), "%s%s", SDPath, WAV_FILE_NAME);
    fr = f_open(&g_audioFile, filePath, FA_READ);
    if (fr != FR_OK)
    {
        UartPrintFResult("f_open WAV failed", fr);
        return;
    }
    g_audioFileOpen = 1;

    fr = ParseWavHeader(&g_audioFile, &g_wavInfo);
    if (fr != FR_OK)
    {
        UartPrintFResult("ParseWavHeader failed", fr);
        f_close(&g_audioFile);
        g_audioFileOpen = 0;
        return;
    }

    snprintf(msg, sizeof(msg),
             "WAV: %lu Hz, %u ch, %u bits, data=%lu bytes\r\n",
             (unsigned long)g_wavInfo.sampleRate,
             (unsigned)g_wavInfo.channels,
             (unsigned)g_wavInfo.bitsPerSample,
             (unsigned long)g_wavInfo.dataSize);
    UartPrint(msg);

    if (ConfigureSaiForWav(&g_wavInfo) != HAL_OK)
    {
        UartPrint("Unsupported WAV sample rate for SAI\r\n");
        f_close(&g_audioFile);
        g_audioFileOpen = 0;
        return;
    }

    g_wavDataRemaining = g_wavInfo.dataSize;
    g_playbackEofPrinted = 0;
    g_fillFirstHalf = 0;
    g_fillSecondHalf = 0;

    FillAudioBufferHalf(0);
    FillAudioBufferHalf(1);

    if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)g_audioBuffer, AUDIO_SAMPLES) != HAL_OK)
    {
        UartPrint("HAL_SAI_Transmit_DMA failed\r\n");
        if (g_audioFileOpen)
        {
            f_close(&g_audioFile);
            g_audioFileOpen = 0;
        }
        return;
    }

    g_playbackStarted = 1;
    UartPrint("WAV playback DMA started\r\n");
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SAI1_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart1, (uint8_t *)"Hello World!\r\n", 14, HAL_MAX_DELAY);

  StartWavPlayback();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ProcessAudioPlayback();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC|RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 393;
  PeriphClkInitStruct.PLL2.PLL2P = 32;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */
  // FIX:
  // hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  // hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 8;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A)
  {
    g_fillFirstHalf = 1;
  }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A)
  {
    g_fillSecondHalf = 1;
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
