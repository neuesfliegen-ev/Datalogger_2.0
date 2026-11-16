#include "unity.h"

#include "hal/sd_card.h"

TEST_CASE("[SD] Full storage diagnostic", "[sd]")
{
    /* Step 1: Does the SD card mount successfully? */
    // Create the SD card driver.
    SDCard sd;

    // Test whether the SD card can be mounted.
    esp_err_t result = sd.begin();

    if (result != ESP_OK)
    {
        TEST_FAIL_MESSAGE(
            "Failed to mount the SD card. "
            "Check power, GND, MISO GPIO19, MOSI GPIO23, "
            "SCK GPIO18, CS GPIO5, and the card format.");
    }

    TEST_MESSAGE("[SD] Mount passed.");

    // Step 2: Create and open the diagnostic file.
    result = sd.openLogFile("/sdcard/diagnostic.bin");

    if (result != ESP_OK)
    {
        // Release resources before failing.
        sd.end();

        TEST_FAIL_MESSAGE(
            "SD file open failed. "
            "Check the filesystem and card write protection.");
    }

    TEST_MESSAGE("[SD] File creation/open passed.");

    // Step 3: Write one binary dataset to the diagnostic file.
    SDataset sample{};
    sample.t = 1234;

    result = sd.writeDatasets(&sample, 1);

    if (result != ESP_OK)
    {
        sd.end();

        TEST_FAIL_MESSAGE(
            "SD write failed. "
            "Check whether the file is open and the card has free space.");
    }

    TEST_MESSAGE("[SD] Binary write passed.");

    // Step 4: Flush buffered data to the SD card.
    result = sd.flush();

    if (result != ESP_OK)
    {
        sd.end();

        TEST_FAIL_MESSAGE(
            "SD flush failed. "
            "The data may not have been committed to the card.");
    }

    TEST_MESSAGE("[SD] Flush passed.");

    // Close the file and unmount the SD card.
    result = sd.end();

    if (result != ESP_OK)
    {
        TEST_FAIL_MESSAGE(
            "Failed to close the file or unmount the SD card.");
    }

    TEST_MESSAGE("[SD] Close and unmount passed.");

    // Step 5: Mount the SD card again for read-back verification.
    result = sd.begin();

    if (result != ESP_OK)
    {
        TEST_FAIL_MESSAGE(
            "Failed to remount the SD card for read-back verification.");
    }

    // Open the diagnostic file in binary read mode.
    FILE *file = fopen("/sdcard/diagnostic.bin", "rb");

    if (file == nullptr)
    {
        sd.end();

        TEST_FAIL_MESSAGE(
            "Failed to reopen diagnostic.bin for reading.");
    }

    // Read one dataset from the diagnostic file.
    SDataset read_back{};

    size_t count = fread(
        &read_back,
        sizeof(SDataset),
        1,
        file);

    fclose(file);
    sd.end();

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(
        1,
        count,
        "Failed to read one dataset from diagnostic.bin.");

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(
        sample.t,
        read_back.t,
        "Read-back data does not match the written data.");

    TEST_MESSAGE("[SD] Read-back verification passed.");
}