#include "unity.h"

TEST_CASE("hardware test application starts", "[smoke]")
{
    TEST_ASSERT_TRUE(true);
}

extern "C" void app_main()
{
    unity_run_menu();
}
