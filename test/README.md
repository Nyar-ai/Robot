# 单元测试（PC 端 / 脱离硬件）

本目录对项目中**纯算法/逻辑层**模块进行 PC 端单元测试，无需 STM32 硬件、仿真器或 HAL 库，直接用 MinGW GCC 编译运行。

## 测试范围

| 模块 | 被测源文件 | 说明 |
|------|-----------|------|
| 麦克纳姆轮运动学 | `Core/Src/mecanum.c` | 逆解/正解/几何参数/NULL 容错 |
| S 形曲线规划器 | `Core/Src/scurve.c` | 到位精度/速度加速度上限/双向/边界 |
| 底盘控制层 | `Core/Src/chassis.c` | 移动/转向/停止/陀螺仪/位姿一致性 |

> **不在范围内**：`stepper.c`（依赖 HAL 定时器/GPIO）、`main.c`、`main.py`（OpenART 视觉）。这些需通过硬件在环（HIL）或实机测试。

## 环境要求

- **MinGW GCC**（已在 `C:\MinGW\bin\gcc.exe`，版本 6.3.0+）
- **mingw32-make**

## 快速开始

```bash
# 在项目任意目录执行（推荐用 -C 指定 test 目录）
mingw32-make -C test clean
mingw32-make -C test          # 仅编译
test/bin/run_tests.exe        # 运行

# 或一步到位：编译 + 运行
mingw32-make -C test run
```

预期输出：
```
##############################################################
# 快递分拣机器人 - PC 端单元测试 (脱离硬件, 验证纯算法/逻辑层) #
##############################################################

========== 麦克纳姆轮运动学 ==========
  [纯前进 vx>0                  ] OK
  ...
========== 汇总 ==========
  通过: 22
  失败: 0
==========================
```

退出码 = 失败用例数（`0` 表示全部通过），便于接入 CI。

## 目录结构

```
test/
├── Makefile                    # 构建脚本（MinGW gcc）
├── test_framework.h            # 极简断言宏（零依赖）
├── test_main.c                 # 测试入口（全局统计 + 汇总）
├── test_mecanum.c              # 麦轮运动学用例
├── test_scurve.c               # S 曲线规划器用例
├── test_chassis.c              # 底盘控制层用例
├── .vscode/c_cpp_properties.json  # VSCode IntelliSense include 路径
└── bin/                        # 编译产物（已 gitignore）
```

## 设计要点

1. **不修改业务代码**：被测 `.c` 文件直接从 `../Core/Src/` 引入编译。
2. **排除硬件依赖**：`Makefile` 只编译 `mecanum/scurve/chassis` 三个纯算法文件，不碰 `stepper/main` 等 HAL 强相关代码。
3. **零外部依赖**：测试框架是自写的断言宏（`test_framework.h`），不引入 Unity/Ceedling 等第三方库。
4. **全局单例处理**：`chassis.c` 内部用全局 `g_chassis`，每个用例开头调 `chassis_init()` 重置状态，避免用例间污染。

## 编写新用例

1. 在对应 `test_xxx.c` 中新增 `static void test_yyy(void) { ... }` 函数
2. 在文件末尾 `test_xxx_run()` 里调用它
3. 重新 `mingw32-make -C test run`

可用断言：
- `TEST_ASSERT_FLOAT_NEAR(期望, 实际, 容差, "说明")` — 浮点近似（核心）
- `TEST_ASSERT_INT_EQ(期望, 实际, "说明")` — 整数相等
- `TEST_ASSERT_TRUE(条件, "说明")` / `TEST_ASSERT_FALSE(...)`
- `TEST_ASSERT(条件, "说明")` — 通用

## 后续扩展方向

- **stepper.c 的 mock 测试**：为 HAL 函数（`__HAL_TIM_SET_AUTORELOAD`、`HAL_GPIO_WritePin`）写桩函数，捕获写入值，验证 `mm/s → ARR` 换算。
- **HIL 测试**：在 `main.c` 加串口调试 shell（如 `move 1000 0` / `turn 90`），真机验证闭环。
- **CI 集成**：把 `mingw32-make -C test && test/bin/run_tests.exe` 接入 GitHub Actions。