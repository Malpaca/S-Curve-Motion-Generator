CC := gcc
CFLAGS := -std=c99 -Wall -Wextra -Wpedantic -Werror -Iinclude -O2
LDFLAGS := -lm

SRC := src/scurve.c src/trajectory.c
BUILD_DIR := build

TEST_SCURVE := $(BUILD_DIR)/test_scurve
TEST_TRAJ := $(BUILD_DIR)/test_trajectory

all: test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TEST_SCURVE): $(SRC) tests/test_scurve.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(TEST_TRAJ): $(SRC) tests/test_trajectory.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

test: $(TEST_SCURVE) $(TEST_TRAJ)

clean:
	rm -rf $(BUILD_DIR)
