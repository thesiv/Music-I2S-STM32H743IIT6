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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ff.h"
#include "lcd.h"
#include "touch.h"

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

enum
{
    LOG_QUEUE_DEPTH = 24U,
    LOG_MESSAGE_MAX_LEN = 192U
};

typedef struct
{
    char text[LOG_MESSAGE_MAX_LEN];
} LogMessage;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_FRAMES     4096
#define AUDIO_CHANNELS   2
#define AUDIO_SAMPLES    (AUDIO_FRAMES * AUDIO_CHANNELS)
#define AUDIO_HALF_FRAMES  (AUDIO_FRAMES / 2)
#define AUDIO_HALF_SAMPLES (AUDIO_SAMPLES / 2)
#define WAV_FILE_NAME    "collectathon_48k.wav"
#define AUDIO_EVT_HALF     (1UL << 0)
#define AUDIO_EVT_COMPLETE (1UL << 1)
#define AUDIO_DMA_ACCESSIBLE __attribute__((section(".RAM_D2"), aligned(32)))
#define SD_DMA_ACCESSIBLE __attribute__((section(".RAM_D1"), aligned(32)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for audioTask */
osThreadId_t audioTaskHandle;
const osThreadAttr_t audioTask_attributes = {
  .name = "audioTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for storageTask */
osThreadId_t storageTaskHandle;
const osThreadAttr_t storageTask_attributes = {
  .name = "storageTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uiTask */
osThreadId_t uiTaskHandle;
const osThreadAttr_t uiTask_attributes = {
  .name = "uiTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for inputTask */
osThreadId_t inputTaskHandle;
const osThreadAttr_t inputTask_attributes = {
  .name = "inputTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for logTask */
osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = {
  .name = "logTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
AUDIO_DMA_ACCESSIBLE
static int16_t g_audioBuffer[AUDIO_SAMPLES];

SD_DMA_ACCESSIBLE
static FATFS g_fatfs;

SD_DMA_ACCESSIBLE
static FIL g_audioFile;
static WavInfo g_wavInfo;
static DWORD g_wavDataRemaining;
SD_DMA_ACCESSIBLE
static uint8_t g_wavReadBuf[1024];
static uint16_t g_volumeQ15 = 5983U;
static uint8_t g_audioFileOpen;
static uint8_t g_playbackStarted;
static uint8_t g_playbackEofPrinted;
static osMessageQueueId_t g_logQueueHandle;
static uint8_t g_lcdReady;
static uint8_t g_touchReady;
static uint8_t g_lcdBrightnessPercent = 100U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FMC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2S3_Init(void);
static void MX_SDMMC1_SD_Init(void);
void StartAudioTask(void *argument);
void StartStorageTask(void *argument);
void StartUiTask(void *argument);
void StartInputTask(void *argument);
void StartLogTask(void *argument);

/* USER CODE BEGIN PFP */
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram);
static void DrawBrightnessBar(uint8_t percent);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UserLedSet(uint8_t enabled)
{
    HAL_GPIO_WritePin(USER_LED_GPIO_Port,
                      USER_LED_Pin,
                      enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void UartWriteBlocking(const char* text)
{
    if (text == NULL)
    {
        return;
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void UartPrint(const char* text)
{
    if (text == NULL)
    {
        return;
    }

    if (g_logQueueHandle != NULL && osKernelGetState() == osKernelRunning)
    {
        LogMessage msg;
        snprintf(msg.text, sizeof(msg.text), "%s", text);
        (void)osMessageQueuePut(g_logQueueHandle, &msg, 0U, 0U);
        return;
    }

    UartWriteBlocking(text);
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

static void DrawBrightnessBar(uint8_t percent)
{
    const uint16_t barX = 40U;
    const uint16_t barY = LCD_HEIGHT - 42U;
    const uint16_t barW = LCD_WIDTH - 80U;
    const uint16_t barH = 18U;
    uint16_t fillW;

    if (!g_lcdReady)
    {
        return;
    }

    if (percent > 100U)
    {
        percent = 100U;
    }

    fillW = (uint16_t)(((uint32_t)barW * percent) / 100U);

    LCD_DrawFilledRect(barX - 2U, barY - 2U, barW + 4U, barH + 4U, LCD_COLOR_WHITE);
    LCD_DrawFilledRect(barX, barY, barW, barH, 0x7BEFU);
    if (fillW > 0U)
    {
        LCD_DrawFilledRect(barX, barY, fillW, barH, LCD_COLOR_YELLOW);
    }
}

static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram)
{
    FMC_SDRAM_CommandTypeDef command = {0};
    const uint32_t modeReg = 0x0001U | 0x0000U | 0x0030U | 0x0000U | 0x0200U;

    command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    command.AutoRefreshNumber = 1;
    command.ModeRegisterDefinition = 0;

    command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
    if (HAL_SDRAM_SendCommand(hsdram, &command, 0xFFFFU) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_Delay(1);

    command.CommandMode = FMC_SDRAM_CMD_PALL;
    if (HAL_SDRAM_SendCommand(hsdram, &command, 0xFFFFU) != HAL_OK)
    {
        Error_Handler();
    }

    command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    command.AutoRefreshNumber = 4;
    if (HAL_SDRAM_SendCommand(hsdram, &command, 0xFFFFU) != HAL_OK)
    {
        Error_Handler();
    }

    command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
    command.AutoRefreshNumber = 1;
    command.ModeRegisterDefinition = modeReg;
    if (HAL_SDRAM_SendCommand(hsdram, &command, 0xFFFFU) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_SDRAM_ProgramRefreshRate(hsdram, 824U) != HAL_OK)
    {
        Error_Handler();
    }
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

static uint32_t WavSampleRateToI2sFrequency(uint32_t sampleRate)
{
    switch (sampleRate)
    {
    case 8000U:
        return I2S_AUDIOFREQ_8K;
    case 11025U:
        return I2S_AUDIOFREQ_11K;
    case 16000U:
        return I2S_AUDIOFREQ_16K;
    case 22050U:
        return I2S_AUDIOFREQ_22K;
    case 32000U:
        return I2S_AUDIOFREQ_32K;
    case 44100U:
        return I2S_AUDIOFREQ_44K;
    case 48000U:
        return I2S_AUDIOFREQ_48K;
    case 96000U:
        return I2S_AUDIOFREQ_96K;
    case 192000U:
        return I2S_AUDIOFREQ_192K;
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

static HAL_StatusTypeDef ConfigureI2sForWav(const WavInfo* info)
{
    uint32_t i2sFrequency = WavSampleRateToI2sFrequency(info->sampleRate);
    if (i2sFrequency == 0U || i2sFrequency != hi2s3.Init.AudioFreq)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
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

static void ProcessAudioPlayback(uint32_t events)
{
    if ((events & AUDIO_EVT_HALF) != 0U)
    {
        FillAudioBufferHalf(0);
    }

    if ((events & AUDIO_EVT_COMPLETE) != 0U)
    {
        FillAudioBufferHalf(1);
    }

    if (g_playbackStarted && !g_audioFileOpen && !g_playbackEofPrinted)
    {
        UartPrint("WAV playback reached EOF, outputting silence\r\n");
        UserLedSet(0U);
        g_playbackEofPrinted = 1;
    }
}

static void StartWavPlayback(void)
{
    FRESULT fr;
    char msg[160];
    char filePath[32];

    UserLedSet(0U);
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

    if (ConfigureI2sForWav(&g_wavInfo) != HAL_OK)
    {
        UartPrint("Unsupported WAV sample rate for I2S3\r\n");
        f_close(&g_audioFile);
        g_audioFileOpen = 0;
        return;
    }

    g_wavDataRemaining = g_wavInfo.dataSize;
    g_playbackEofPrinted = 0;

    FillAudioBufferHalf(0);
    FillAudioBufferHalf(1);

    if (HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)g_audioBuffer, AUDIO_SAMPLES) != HAL_OK)
    {
        UartPrint("HAL_I2S_Transmit_DMA failed\r\n");
        if (g_audioFileOpen)
        {
            f_close(&g_audioFile);
            g_audioFileOpen = 0;
        }
        return;
    }

    g_playbackStarted = 1;
    UserLedSet(1U);
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
  MX_FMC_Init();
  MX_USART1_UART_Init();
  MX_I2S3_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  if (LCD_Init() == HAL_OK)
  {
    char lcdMsg[64];

    g_lcdReady = 1U;
    LCD_Fill(LCD_COLOR_WHITE);

    LCD_DrawFilledRect(240U, 80U, 320U, 320U, LCD_COLOR_WHITE);
    LCD_DrawFilledRect(300U, 140U, 200U, 200U, LCD_COLOR_BLUE);
    snprintf(lcdMsg, sizeof(lcdMsg), "LCD center pixel=0x%04X\r\n", LCD_ReadPixel(400U, 240U));
    UartPrint(lcdMsg);
    UartPrint("LCD BL test: PH6/LCD0BL high\r\n");
    LCD_BacklightSelfTest();
    LCD_BacklightSetDuty(g_lcdBrightnessPercent);
    DrawBrightnessBar(g_lcdBrightnessPercent);
    UartPrint(LCD_BacklightRead() == GPIO_PIN_RESET ? "LCD BL PH6/LCD0BL=LOW\r\n" : "LCD BL PH6/LCD0BL=HIGH\r\n");
    UartPrint("LCD 800x480 initialized\r\n");
  }
  else
  {
    UartPrint("LCD init failed\r\n");
  }

  if (Touch_Init() == HAL_OK)
  {
    g_touchReady = 1U;
    UartPrint("Touch controller initialized\r\n");
  }
  else
  {
    UartPrint("Touch controller not found\r\n");
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  g_logQueueHandle = osMessageQueueNew(LOG_QUEUE_DEPTH, sizeof(LogMessage), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of audioTask */
  audioTaskHandle = osThreadNew(StartAudioTask, NULL, &audioTask_attributes);

  /* creation of storageTask */
  storageTaskHandle = osThreadNew(StartStorageTask, NULL, &storageTask_attributes);

  /* creation of uiTask */
  uiTaskHandle = osThreadNew(StartUiTask, NULL, &uiTask_attributes);

  /* creation of inputTask */
  inputTaskHandle = osThreadNew(StartInputTask, NULL, &inputTask_attributes);

  /* creation of logTask */
  logTaskHandle = osThreadNew(StartLogTask, NULL, &logTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC|RCC_PERIPHCLK_SPI123;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 393;
  PeriphClkInitStruct.PLL2.PLL2P = 32;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s3.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s3.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s3.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
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

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  SDRAM_Initialization_Sequence(&hsdram1);

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A)
  {
    if (audioTaskHandle != NULL)
    {
      (void)osThreadFlagsSet(audioTaskHandle, AUDIO_EVT_HALF);
    }
  }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A)
  {
    if (audioTaskHandle != NULL)
    {
      (void)osThreadFlagsSet(audioTaskHandle, AUDIO_EVT_COMPLETE);
    }
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAudioTask */
/**
  * @brief  Function implementing the audioTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAudioTask */
void StartAudioTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  StartWavPlayback();

  for(;;)
  {
    uint32_t events = osThreadFlagsWait(AUDIO_EVT_HALF | AUDIO_EVT_COMPLETE,
                                        osFlagsWaitAny,
                                        osWaitForever);
    if ((events & osFlagsError) == 0U)
    {
      ProcessAudioPlayback(events);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartStorageTask */
/**
* @brief Function implementing the storageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStorageTask */
void StartStorageTask(void *argument)
{
  /* USER CODE BEGIN StartStorageTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartStorageTask */
}

/* USER CODE BEGIN Header_StartUiTask */
/**
* @brief Function implementing the uiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUiTask */
void StartUiTask(void *argument)
{
  /* USER CODE BEGIN StartUiTask */
  if (g_lcdReady)
  {
    DrawBrightnessBar(g_lcdBrightnessPercent);
  }

  for(;;)
  {
    osDelay(20);
  }
  /* USER CODE END StartUiTask */
}

/* USER CODE BEGIN Header_StartInputTask */
/**
* @brief Function implementing the inputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputTask */
void StartInputTask(void *argument)
{
  /* USER CODE BEGIN StartInputTask */
  const uint16_t brightnessHitY = LCD_HEIGHT - 80U;
  TouchState touch;
  uint8_t lastBrightness = g_lcdBrightnessPercent;
  uint8_t brightnessDragActive = 0U;
  uint16_t filteredX = 0U;

  for(;;)
  {
    if (g_lcdReady && g_touchReady && Touch_Read(&touch) == HAL_OK && touch.count > 0U)
    {
      uint16_t x = (touch.points[0].x < LCD_WIDTH) ? touch.points[0].x : (LCD_WIDTH - 1U);
      uint16_t y = (touch.points[0].y < LCD_HEIGHT) ? touch.points[0].y : (LCD_HEIGHT - 1U);

      if (!brightnessDragActive && y >= brightnessHitY)
      {
        brightnessDragActive = 1U;
        filteredX = x;
      }

      if (brightnessDragActive)
      {
        int16_t dx = (int16_t)x - (int16_t)filteredX;

        if (dx > 3 || dx < -3)
        {
          filteredX = (uint16_t)(((uint32_t)filteredX * 3U + (uint32_t)x) / 4U);
        }

        {
          uint8_t brightness = (uint8_t)(((uint32_t)filteredX * 100U) / (LCD_WIDTH - 1U));
          uint8_t delta = (brightness > lastBrightness) ?
                          (uint8_t)(brightness - lastBrightness) :
                          (uint8_t)(lastBrightness - brightness);

          if (delta >= 2U || brightness == 0U || brightness == 100U)
          {
            g_lcdBrightnessPercent = brightness;
            lastBrightness = brightness;
            LCD_BacklightSetDuty(brightness);
            DrawBrightnessBar(brightness);
          }
        }
      }
    }
    else
    {
      brightnessDragActive = 0U;
    }

    osDelay(20);
  }
  /* USER CODE END StartInputTask */
}

/* USER CODE BEGIN Header_StartLogTask */
/**
* @brief Function implementing the logTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogTask */
void StartLogTask(void *argument)
{
  /* USER CODE BEGIN StartLogTask */
  LogMessage msg;

  UartWriteBlocking("Log task started\r\n");

  for(;;)
  {
    if (g_logQueueHandle == NULL)
    {
      osDelay(100);
      continue;
    }

    if (osMessageQueueGet(g_logQueueHandle, &msg, NULL, osWaitForever) == osOK)
    {
      UartWriteBlocking(msg.text);
    }
  }
  /* USER CODE END StartLogTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
