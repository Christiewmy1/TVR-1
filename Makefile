# TVR-1 host build: compiles the firmware modules together with each Unity
# test file and runs them on the laptop or in CI. Uses the system compiler
# (clang on macOS, gcc on Ubuntu CI). No STM32/HAL code is built here.

CFLAGS   := -std=c99 -Wall -Wextra -Werror
INCLUDES := -Ifirmware/include -Itests/unity
LDLIBS   := -lm

BUILD_DIR := build
FW_SRCS   := $(wildcard firmware/src/*.c)
FW_HDRS   := $(wildcard firmware/include/*.h)
UNITY_SRC := tests/unity/unity.c
TEST_SRCS := $(wildcard tests/test_*.c)
TEST_BINS := $(patsubst tests/%.c,$(BUILD_DIR)/%,$(TEST_SRCS))

SAMPLE_LOG   := ground/tests/data/sample_flight_raw.csv
SAMPLE_INPUT := tests/data/Sensor_Fusion_OR_Data_updated.csv

.PHONY: test clean sample-log

# Run every test binary, then fail if any of them failed.
test: $(TEST_BINS)
	@fail=0; for t in $(TEST_BINS); do echo "== $$t"; ./$$t || fail=1; done; exit $$fail

# One binary per test file: the test + all firmware sources + Unity.
$(BUILD_DIR)/test_%: tests/test_%.c $(FW_SRCS) $(FW_HDRS) $(UNITY_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $(filter %.c,$^) -o $@ $(LDLIBS)

# Sample dongle log for the ground station, made by the real state machine
# and encoder from an OpenRocket flight. Commit the result.
sample-log: $(BUILD_DIR)/make_sample_log
	./$< $(SAMPLE_INPUT) > $(SAMPLE_LOG)

$(BUILD_DIR)/make_sample_log: tests/tools/make_sample_log.c $(FW_SRCS) $(FW_HDRS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $(filter %.c,$^) -o $@ $(LDLIBS)

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)
