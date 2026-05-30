#pragma once

#include "esp_err.h"
#include "sdmmc_cmd.h"
#include "telemetry.h"


class SDCard {
public:
    esp_err_t begin();
    void end();

    esp_err_t openLogFile(const char *path);

    esp_err_t writeDatasets(const SDataset *data, size_t count);

    esp_err_t flush();

    esp_err_t writeFile(const char *path, const char *data);
    esp_err_t readFile(const char *path, char *buffer, size_t buffer_size);

private:
    sdmmc_card_t *card = nullptr;
    bool mounted = false;
    FILE *log_file = nullptr;
};