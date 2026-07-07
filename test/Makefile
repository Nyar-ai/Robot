# ============================================================
#  PC 端单元测试 Makefile (MinGW gcc)
#
#  用法:
#    mingw32-make            # 编译并链接
#    mingw32-make run        # 编译 + 运行
#    mingw32-make clean      # 清理
#
#  说明: 仅编译纯算法/逻辑层(mecanum/scurve/chassis),
#        不编译 stepper.c/main.c 等依赖 HAL 的硬件代码.
# ============================================================

# ---- 工具链 ----
CC      = gcc
CFLAGS  = -std=c99 -Wall -Wextra -O2 -g
CFLAGS += -I. -I../Core/Inc
LDFLAGS = -lm

# ---- 目标 ----
TARGET_DIR = bin
TARGET     = $(TARGET_DIR)/run_tests.exe

# ---- 源文件 ----
# 被测业务源文件(仅纯算法层, 不含依赖 HAL 的 stepper/main)
SRC_BUSINESS = ../Core/Src/mecanum.c \
               ../Core/Src/scurve.c  \
               ../Core/Src/chassis.c

# 测试源文件
SRC_TEST = test_main.c      \
           test_mecanum.c   \
           test_scurve.c    \
           test_chassis.c

OBJS = $(patsubst %.c,$(TARGET_DIR)/%.o,$(notdir $(SRC_BUSINESS) $(SRC_TEST)))

# ---- 规则 ----
.PHONY: all run clean

all: $(TARGET)

# 链接
$(TARGET): $(OBJS) | $(TARGET_DIR)
	@echo "[LINK] $@"
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

# 编译业务源文件(来自 ../Core/Src)
$(TARGET_DIR)/%.o: ../Core/Src/%.c | $(TARGET_DIR)
	@echo "[CC]  $<"
	$(CC) $(CFLAGS) -c $< -o $@

# 编译测试源文件(来自当前目录)
$(TARGET_DIR)/%.o: %.c | $(TARGET_DIR)
	@echo "[CC]  $<"
	$(CC) $(CFLAGS) -c $< -o $@

# 创建输出目录(POSIX 语法, 兼容 MinGW sh)
$(TARGET_DIR):
	@mkdir -p $(TARGET_DIR)

# 运行测试
run: $(TARGET)
	$(TARGET)

# 清理
clean:
	@rm -rf $(TARGET_DIR)