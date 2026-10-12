# USB-CDC 驱动测试 Makefile
#
# 编译 test/driver_test.cpp 与父目录的 usb_cdc.cpp, 产物全部放进本目录的 build/ 下.
# 测试使用伪终端(pty)充当假设备, 不需要真实硬件, 也不依赖任何第三方库.
#
# 常用目标:
#   make            编译测试程序
#   make run        编译并运行
#   make tsan       用 ThreadSanitizer 重新编译并运行(检查数据竞争)
#   make clean      清掉 build/ 目录

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
LDLIBS   ?= -pthread

# 父目录(usb_cdc.cpp 与 usb_cdc.hpp 所在目录)
SRC_DIR  := ..

BUILD_DIR := build
TARGET    := $(BUILD_DIR)/usb_cdc_test
TSAN_BIN  := $(BUILD_DIR)/usb_cdc_test_tsan

GIT_REV := $(shell git rev-parse --short HEAD 2>/dev/null || echo unknown)

# 部分内核(如 6.8)配合旧版 TSan 时会报 "unexpected memory mapping", 需要先关掉 ASLR.
# 这里探测一下 setarch 是否可用, 可用就用它跑.
SETARCH := $(shell setarch $$(uname -m) -R true >/dev/null 2>&1 && echo "setarch $$(uname -m) -R")

# -I 让 driver_test.cpp 能直接 #include 父目录的头文件
CXXFLAGS += -I$(SRC_DIR) -DUSB_CDC_TEST_GIT_REV=\"$(GIT_REV)\"

SRCS := driver_test.cpp $(SRC_DIR)/usb_cdc.cpp

.PHONY: all run test tsan clean

all: $(TARGET)

$(TARGET): $(SRCS) $(SRC_DIR)/usb_cdc.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ $(LDLIBS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

run test: $(TARGET)
	./$(TARGET)

# 线程检查器: 会大幅拖慢执行, 计时类用例在该模式下自动跳过
tsan: | $(BUILD_DIR)
	$(CXX) -std=c++17 -O1 -g -fsanitize=thread -Wall -Wextra -Wpedantic \
	    -I$(SRC_DIR) -DUSB_CDC_TEST_GIT_REV=\"$(GIT_REV)\" \
	    $(SRCS) -o $(TSAN_BIN) -fsanitize=thread -pthread
	$(SETARCH) ./$(TSAN_BIN)

clean:
	rm -rf $(BUILD_DIR)
