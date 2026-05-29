#pragma once

#include "esp_err.h"
#include "sdmmc_cmd.h"
#include "telemetry.h"


class SDCard {
public:
    esp_err_t begin();
    void end();

    esp_err_t writeFile(const char *path, const char *data);
    esp_err_t readFile(const char *path, char *buffer, size_t buffer_size);
    esp_err_t writeDataset(const SDataset &data);


private:
    sdmmc_card_t *card = nullptr;
    bool mounted = false;
    FILE *log_file = nullptr;
};