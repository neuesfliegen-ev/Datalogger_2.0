#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"

#include "pins.h"
#include "modules/telemetry.h"
#include "hal/sd_card.h"

static const char *TAG = "SD";
static const char *SD_MOUNT_POINT = "/sdcard";

esp_err_t SDCard::begin() {
    if (mounted) {
        return ESP_OK;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
        mount_config.format_if_mount_failed = false;
        mount_config.max_files = 5;
        mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;   // VSPI beim klassischen ESP32
    host.max_freq_khz = 10000; //idk but it works

    spi_host_device_t spi_host = SPI3_HOST;

    spi_bus_config_t bus_cfg = {};
        bus_cfg.mosi_io_num = SD_MOSI_PIN;
        bus_cfg.miso_io_num = SD_MISO_PIN;
        bus_cfg.sclk_io_num = SD_SCK_PIN;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        bus_cfg.max_transfer_sz = 16 * 1024;

    esp_err_t ret = spi_bus_initialize(spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot_config.gpio_cs = SD_CS_PIN;
        slot_config.host_id = spi_host;

    ret = esp_vfs_fat_sdspi_mount(
        SD_MOUNT_POINT,
        &host,
        &slot_config,
        &mount_config,
        &card
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        spi_bus_free(spi_host);
        card = nullptr;
        mounted = false;
        return ret;
    }

    mounted = true;
    sdmmc_card_print_info(stdout, card);

    return ESP_OK;
}

//Creates and opens binary file for logging
esp_err_t SDCard::openLogFile(const char *path){
    if (!mounted) {
        return ESP_ERR_INVALID_STATE;
    }

    if (log_file != nullptr) {
        fclose(log_file);
        log_file = nullptr;
    }

    log_file = fopen(path, "wb");   // new binary file

    if (log_file == nullptr) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t SDCard::writeDatasets(const SDataset *data, size_t count){
    if (!mounted || log_file == nullptr || data == nullptr || count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t written = fwrite(data, sizeof(SDataset), count, log_file);

    if (written != count) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t SDCard::flush(){
    if (!mounted || log_file == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    int ret = fflush(log_file);

    if (ret != 0) {
        ESP_LOGE(TAG, "fflush failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t SDCard::end()
{
    esp_err_t ret = ESP_OK;

    if (log_file != nullptr) {
        if (fflush(log_file) != 0) {
            ESP_LOGE(TAG, "fflush failed");
            ret = ESP_FAIL;
        }

        if (fclose(log_file) != 0) {
            ESP_LOGE(TAG, "fclose failed");
            ret = ESP_FAIL;
        }

        log_file = nullptr;
    }

    if (mounted && card != nullptr) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
        card = nullptr;
        mounted = false;
    }

    esp_err_t spi_ret = spi_bus_free(SPI3_HOST);
    if (spi_ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_free failed: %s", esp_err_to_name(spi_ret));
        ret = spi_ret;
    }

    return ret;
}

/*
esp_err_t SDCard::readFile(const char *path, char *buffer, size_t buffer_size){
    if (!mounted) {
        return ESP_ERR_INVALID_STATE;
    }

    if (buffer == nullptr || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *file = fopen(path, "r");
    if (file == nullptr) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", path);
        return ESP_FAIL;
    }

    size_t bytes_read = fread(buffer, 1, buffer_size - 1, file);
    buffer[bytes_read] = '\0';

    fclose(file);

    return ESP_OK;
}

esp_err_t SDCard::writeFile(const char *path, const char *data){
    if (!mounted) {
        return ESP_ERR_INVALID_STATE;
    }

    FILE *file = fopen(path, "w");
    if (file == nullptr) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", path);
        return ESP_FAIL;
    }

    int written = fprintf(file, "%s", data);
    fclose(file);

    if (written < 0) {
        ESP_LOGE(TAG, "Failed to write to file: %s", path);
        return ESP_FAIL;
    }

    return ESP_OK;
}
*/