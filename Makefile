CC := gcc
CFLAGS := -std=c99 -Wall -Wextra -Wpedantic -Werror -Iinclude -O2
LDFLAGS := -lm

SRC := src/scurve.c
BUILD_DIR := build

TEST_SCURVE := $(BUILD_DIR)/test_scurve

all: test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TEST_SCURVE): $(SRC) tests/test_scurve.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

test: $(TEST_SCURVE)

clean:
	rm -rf $(BUILD_DIR)
