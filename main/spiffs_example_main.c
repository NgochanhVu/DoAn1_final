#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "driver/i2s.h"
#include "driver/adc.h"
//#include "audio_example_file.h"
#include "esp_adc_cal.h"
#include "esp_rom_sys.h"
//library for spiffs
#include <esp_spiffs.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#if CONFIG_IDF_TARGET_ESP32

static const char* TAG = "ad/da";
#define V_REF   1100
#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_7)

/*---------------------------------------------------------------
                            EXAMPLE CONFIG
---------------------------------------------------------------*/
//enable record sound and save in flash
#define RECORD_IN_FLASH_EN        (1)
//enable replay recorded sound in flash
#define REPLAY_FROM_FLASH_EN      (1)

//i2s number
#define EXAMPLE_I2S_NUM           (0)
//i2s sample rate
#define EXAMPLE_I2S_SAMPLE_RATE   (16000)
//i2s data bits
#define EXAMPLE_I2S_SAMPLE_BITS   (16)
//enable display buffer for debug
#define EXAMPLE_I2S_BUF_DEBUG     (0)
//I2S read buffer length
#define EXAMPLE_I2S_READ_LEN      (16 * 1024)
//I2S data format
#define EXAMPLE_I2S_FORMAT        (I2S_CHANNEL_FMT_ONLY_LEFT)
//I2S channel number
#define EXAMPLE_I2S_CHANNEL_NUM   ((EXAMPLE_I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0

//flash record size, for recording 5 seconds' data
#define FLASH_RECORD_SIZE         (EXAMPLE_I2S_CHANNEL_NUM * EXAMPLE_I2S_SAMPLE_RATE * EXAMPLE_I2S_SAMPLE_BITS / 8 * 10)
#define FLASH_ERASE_SIZE          (FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE == 0) ? FLASH_RECORD_SIZE : FLASH_RECORD_SIZE + (FLASH_SECTOR_SIZE - FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE)
//sector size of flash
#define FLASH_SECTOR_SIZE         (0x1000)
//flash read / write address
#define FLASH_ADDR                (0x200000)

#define I2S_SCK                   32
#define I2S_SD                    33
#define I2S_WS                    25
/**
 * @brief I2S ADC/DAC mode init.
 */
void example_i2s_init(void)
{
     int i2s_num = EXAMPLE_I2S_NUM;
     i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate =  EXAMPLE_I2S_SAMPLE_RATE,
        .bits_per_sample = EXAMPLE_I2S_SAMPLE_BITS,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .channel_format = EXAMPLE_I2S_FORMAT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 64,
        .dma_buf_len = 1024,
        .use_apll = 1,
     };
     //install and start i2s driver
     i2s_driver_install(i2s_num, &i2s_config, 0, NULL);

     const  i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
     };

    i2s_set_pin(EXAMPLE_I2S_NUM, &pin_config);
}

FILE* f;

const char filename[] = "/recording.wav";

const int headerSize = 44;
void wavHeader(char* header, int wavSize) {
    header[0] = 'R';
    header[1] = 'I';
    header[2] = 'F';
    header[3] = 'F';
    unsigned int fileSize = wavSize + headerSize - 8;
    header[4] = fileSize & 0xFF;
    header[5] = (fileSize >> 8) & 0xFF;
    header[6] = (fileSize >> 16) & 0xFF;
    header[7] = (fileSize >> 24) & 0xFF;
    header[8] = 'W';
    header[9] = 'A';
    header[10] = 'V';
    header[11] = 'E';
    header[12] = 'f';
    header[13] = 'm';
    header[14] = 't';
    header[15] = ' ';
    header[16] = 0x10;
    header[17] = 0x00;
    header[18] = 0x00;
    header[19] = 0x00;
    header[20] = 0x01;
    header[21] = 0x00;
    header[22] = 0x01;
    header[23] = 0x00;
    header[24] = 0x80;
    header[25] = 0x3E;
    header[26] = 0x00;
    header[27] = 0x00;
    header[28] = 0x00;
    header[29] = 0x7D;
    header[30] = 0x01;
    header[31] = 0x00;
    header[32] = 0x02;    
    header[33] = 0x00;    
    header[34] = 0x10;    
    header[35] = 0x00;    
    header[36] = 'd';    
    header[37] = 'a';    
    header[38] = 't';    
    header[39] = 'a';    
    header[40] = wavSize & 0xff;
    header[41] = wavSize >> 8 & 0xff;
    header[42] = wavSize >> 16 & 0xff;
    header[43] = wavSize >> 24 & 0xff;
}

void spiffs_task() {
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/storage",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } 
        else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
        esp_spiffs_format(conf.partition_label);
        return;
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Check consistency of reported partiton size info.
    if (used > total) {
        ESP_LOGW(TAG, "Number of used bytes cannot be larger than total. Performing SPIFFS_check().");
        ret = esp_spiffs_check(conf.partition_label);
         if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
            return;
        } else {
            ESP_LOGI(TAG, "SPIFFS_check() successful");
        }
    }
    
    ESP_LOGI(TAG, "Opening file");
    f = fopen("/storage/recording.wav", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    char* header[headerSize];
    wavHeader(header, FLASH_RECORD_SIZE);
    fprintf(f, header, headerSize);
    //ham listSPIFFS khong biet chuyen sang idf
}
/**
 * @brief debug buffer data
 */
void example_disp_buf(uint8_t* buf, int length)
{
    printf("======\n");
    for (int i = 0; i < length; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 8 == 0) {
            printf("\n");
        }
    }
    printf("======\n");
}


void example_i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
#if (EXAMPLE_I2S_SAMPLE_BITS == 16)
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 4096;
    }
#else
    for (int i = 0; i < len; i += 4) {
        dac_value = ((((uint16_t)(s_buff[i + 3] & 0xf) << 8) | ((s_buff[i + 2]))));
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 4096;
    }
#endif
}

/**
 * @brief I2S ADC/DAC example
 *        1. Erase flash
 *        2. Record audio from ADC and save in flash
 *        3. Read flash and replay the sound via DAC
 *        4. Play an example audio file(file format: 8bit/8khz/single channel)
 *        5. Loop back to step 3
 */
void example_i2s_adc_dac(void*arg)
{
    int i2s_read_len = EXAMPLE_I2S_READ_LEN;
    int flash_wr_size = 0;
    size_t bytes_read;

    char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
    uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    i2s_adc_enable(EXAMPLE_I2S_NUM);

    printf("*****Start recording*****\n");

    while (flash_wr_size < FLASH_RECORD_SIZE) {
        //read data from I2S bus, in this case, from ADC.
        i2s_read(EXAMPLE_I2S_NUM, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
        example_disp_buf((uint8_t*) i2s_read_buff, 64);
        //save original data from I2S(ADC) into flash.
        example_i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
        fprintf(f, i2s_read_buff, i2s_read_len);
        fwrite(i2s_read_buff,1 , i2s_read_len, f);
        flash_wr_size += i2s_read_len;
        esp_rom_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
        esp_rom_printf("Never Used Stack Size: %u\n", uxTaskGetStackHighWaterMark(NULL));
    }
    //fprintf(f, i2s_read_buff, i2s_read_len);
    fclose(f);
    ESP_LOGI(TAG, "File written");
    ESP_LOGI(TAG, "SPIFFS unmounted");
    //i2s_adc_disable(EXAMPLE_I2S_NUM);
    free(i2s_read_buff);
    i2s_read_buff = NULL;

    //ham listSPIFFS khong biet chuyen sang idf

    vTaskDelete(NULL);
}

esp_err_t app_main(void)
{
    example_i2s_init();
    esp_log_level_set("I2S", ESP_LOG_INFO);
    spiffs_task();
    xTaskCreate(example_i2s_adc_dac, "example_i2s_adc_dac", 1024 * 2, NULL, 5, NULL);
    return ESP_OK;
}
#endif