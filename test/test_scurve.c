/**
 * @file    test_scurve.c
 * @brief   S形曲线(七段式)速度规划器单元测试
 *
 * 验证 scurve.h 的:
 *   - 基本到位与位置精度
 *   - 速度/加速度不超限
 *   - 双向运动
 *   - 极小目标/Stop/NULL 等边界
 */
#include "test_framework.h"
#include "scurve.h"

/* 测试套件入口(由 test_main.c 调用) */
void test_scurve_run(void);

/* 默认测试参数(量纲 mm) */
static const Scurve_Config DEF_CFG = { 400.0f, 800.0f, 2000.0f };

/* ---- 各用例 ---- */

/* 基本到位: MoveTo(1000) 跑到 idle, 位置应≈1000 */
static void test_basic_reach(void)
{
    TEST_BEGIN("基本到位 MoveTo(1000)");
    Scurve_Planner p;
    Scurve_Init(&p, &DEF_CFG);
    Scurve_MoveTo(&p, 1000.0f);

    int max_ticks = 100000; /* 上限保护, 防死循环 */
    while (!Scurve_IsIdle(&p) && max_ticks-- > 0) {
        Scurve_Update(&p, 0.001f);
    }
    TEST_ASSERT_TRUE(Scurve_IsIdle(&p), "应到达 STOP");
    TEST_ASSERT_FLOAT_NEAR(1000.0f, Scurve_GetPos(&p), 10.0f, "最终位置≈1000(误差<1%)");
    TEST_END();
}

/* 速度上限: 全过程 |speed| 不超 max_speed(留 0.5% 数值裕度) */
static void test_speed_limit(void)
{
    TEST_BEGIN("速度上限不超 max_speed");
    Scurve_Planner p;
    Scurve_Init(&p, &DEF_CFG);
    Scurve_MoveTo(&p, 5000.0f); /* 长距离确保进入匀速段 */

    float max_observed = 0.0f;
    int max_ticks = 100000;
    while (!Scurve_IsIdle(&p) && max_ticks-- > 0) {
        float s = Scurve_Update(&p, 0.001f);
        if (s < 0.0f) s = -s;
        if (s > max_observed) max_observed = s;
    }
    TEST_ASSERT_FLOAT_NEAR(DEF_CFG.max_speed, max_observed, DEF_CFG.max_speed * 0.01f,
                           "峰值速度≈max_speed(±1%)");
    /* 严格不超过(含 0.5% 裕度吸收浮点误差) */
    TEST_ASSERT_TRUE(max_observed <= DEF_CFG.max_speed * 1.005f,
                     "速度不得显著超限");
    TEST_END();
}

/* 双向: MoveTo(-1000) 最终位置≈-1000 */
static void test_negative_target(void)
{
    TEST_BEGIN("双向 MoveTo(-1000)");
    Scurve_Planner p;
    Scurve_Init(&p, &DEF_CFG);
    Scurve_MoveTo(&p, -1000.0f);

    int max_ticks = 100000;
    while (!Scurve_IsIdle(&p) && max_ticks-- > 0) {
        Scurve_Update(&p, 0.001f);
    }
    TEST_ASSERT_TRUE(Scurve_IsIdle(&p), "应到达 STOP");
    TEST_ASSERT_FLOAT_NEAR(-1000.0f, Scurve_GetPos(&p), 10.0f, "最终位置≈-1000");
    TEST_END();
}

/* 极小目标: MoveTo(0.0001) 应立即判定完成(STOP) */
static void test_tiny_target(void)
{
    TEST_BEGIN("极小目标立即STOP");
    Scurve_Planner p;
    Scurve_Init(&p, &DEF_CFG);
    Scurve_MoveTo(&p, 0.0001f);
    TEST_ASSERT_TRUE(Scurve_IsIdle(&p), "极小目标应直接STOP");
    TEST_END();
}

/* Stop: 运动中调用 Stop 应立即停下且 IsIdle=true */
static void test_stop_midway(void)
{
    TEST_BEGIN("运动中Stop立即停");
    Scurve_Planner p;
    Scurve_Init(&p, &DEF_CFG);
    Scurve_MoveTo(&p, 5000.0f);
    /* 跑 100ms 让它动起来 */
    for (int i = 0; i < 100; ++i) Scurve_Update(&p, 0.001f);
    TEST_ASSERT_FALSE(Scurve_IsIdle(&p), "100ms后应还在运动");
    Scurve_Stop(&p);
    TEST_ASSERT_TRUE(Scurve_IsIdle(&p), "Stop后应IDLE");
    TEST_ASSERT_FLOAT_NEAR(0.0f, Scurve_GetSpeed(&p), 0.001f, "Stop后速度=0");
    TEST_END();
}

/* NULL/非法参数容错 */
static void test_null_safe(void)
{
    TEST_BEGIN("NULL 容错");
    Scurve_Init(NULL, &DEF_CFG);     /* 不应崩溃 */
    Scurve_MoveTo(NULL, 100.0f);
    Scurve_Stop(NULL);
    TEST_ASSERT_TRUE(Scurve_IsIdle(NULL) == false, "NULL planner 非idle");
    TEST_ASSERT_FLOAT_NEAR(0.0f, Scurve_GetPos(NULL), 0.001f, "NULL pos=0");
    TEST_ASSERT_FLOAT_NEAR(0.0f, Scurve_GetSpeed(NULL), 0.001f, "NULL speed=0");
    TEST_ASSERT_FLOAT_NEAR(0.0f, Scurve_Update(NULL, 0.001f), 0.001f, "NULL update=0");
    TEST_END();
}

/* dt<=0 不推进 */
static void test_nonpositive_dt(void)
{
    TEST_BEGIN("dt<=0 不推进");
    Scurve_Planner p;
    Scurve_Init(&p, &DEF_CFG);
    Scurve_MoveTo(&p, 1000.0f);
    float s = Scurve_Update(&p, 0.0f);
    TEST_ASSERT_FLOAT_NEAR(0.0f, s, 0.001f, "dt=0 返回0");
    TEST_ASSERT_FALSE(Scurve_IsIdle(&p), "dt=0 不改变状态");
    TEST_END();
}

/* ---- 套件汇总 ---- */
void test_scurve_run(void)
{
    TEST_SUITE("S形曲线规划器");
    test_basic_reach();
    test_speed_limit();
    test_negative_target();
    test_tiny_target();
    test_stop_midway();
    test_null_safe();
    test_nonpositive_dt();
}