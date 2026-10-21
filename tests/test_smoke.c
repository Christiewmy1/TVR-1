/*
 * Smoke test: proves the build and Unity work, and checks the type sizes
 * the firmware relies on, so a laptop build behaves like the STM32F405
 * (single-precision float, exact-width integers).
 */
#include <stdint.h>

#include "unity.h"
#include "config.h"

void setUp(void) {}
void tearDown(void) {}

void test_float_is_single_precision(void)
{
    TEST_ASSERT_EQUAL_size_t(4, sizeof(float));
}

void test_fixed_width_ints_have_expected_size(void)
{
    TEST_ASSERT_EQUAL_size_t(1, sizeof(uint8_t));
    TEST_ASSERT_EQUAL_size_t(2, sizeof(uint16_t));
    TEST_ASSERT_EQUAL_size_t(4, sizeof(int32_t));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_float_is_single_precision);
    RUN_TEST(test_fixed_width_ints_have_expected_size);
    return UNITY_END();
}
