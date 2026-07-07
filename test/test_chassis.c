/**
 * @file    test_chassis.c
 * @brief   底盘控制层单元测试
 *
 * 验证 chassis.h 的:
 *   - 初始化状态
 *   - 移动到坐标后位姿收敛
 *   - 转向到角度后航向收敛
 *   - 即时停止
 *   - 陀螺仪注入
 *   - pose get/set 一致性
 *
 * 注: chassis 内部使用全局单例 g_chassis, 每个用例开头调 chassis_init() 重置状态.
 */
#include "test_framework.h"
#include "chassis.h"

/* 测试套件入口(由 test_main.c 调用) */
void test_chassis_run(void);

/* 跑足够多 tick 直到 idle 或超时, 返回是否 idle */
static int run_until_idle(int max_ticks)
{
    for (int i = 0; i < max_ticks; ++i) {
        chassis_tick();
        if (chassis_is_idle()) return 1;
    }
    return 0;
}

/* ---- 各用例 ---- */

/* 初始化: is_idle, pose=(0,0,0) */
static void test_init_state(void)
{
    TEST_BEGIN("初始化状态");
    chassis_init();
    TEST_ASSERT_TRUE(chassis_is_idle(), "init后应IDLE");
    float x = 999, y = 999, th = 999;
    chassis_get_pose(&x, &y, &th);
    TEST_ASSERT_FLOAT_NEAR(0.0f, x, 0.001f, "init x=0");
    TEST_ASSERT_FLOAT_NEAR(0.0f, y, 0.001f, "init y=0");
    TEST_ASSERT_FLOAT_NEAR(0.0f, th, 0.001f, "init theta=0");
    /* 轮速应全 0 */
    float w[4] = {1, 1, 1, 1};
    chassis_get_wheel_speed(w);
    for (int i = 0; i < 4; ++i)
        TEST_ASSERT_FLOAT_NEAR(0.0f, w[i], 0.001f, "init 轮速=0");
    TEST_END();
}

/* 移动到 (1000, 0): 最终 x≈1000 */
static void test_move_forward(void)
{
    TEST_BEGIN("前进到 (1000,0)");
    chassis_init();
    move_to_coordinate(1000.0f, 0.0f);
    TEST_ASSERT_FALSE(chassis_is_idle(), "发起后非IDLE");
    int ok = run_until_idle(5000);
    TEST_ASSERT_TRUE(ok, "应在限定tick内到达");
    float x, y, th;
    chassis_get_pose(&x, &y, &th);
    TEST_ASSERT_FLOAT_NEAR(1000.0f, x, 20.0f, "最终x≈1000(±20mm)");
    TEST_ASSERT_FLOAT_NEAR(0.0f, y, 20.0f, "y应≈0");
    TEST_END();
}

/* 移动到斜向 (600, 800): 距离=1000 */
static void test_move_diagonal(void)
{
    TEST_BEGIN("斜移到 (600,800)");
    chassis_init();
    move_to_coordinate(600.0f, 800.0f);
    int ok = run_until_idle(5000);
    TEST_ASSERT_TRUE(ok, "应到达");
    float x, y, th;
    chassis_get_pose(&x, &y, &th);
    TEST_ASSERT_FLOAT_NEAR(600.0f, x, 30.0f, "最终x≈600");
    TEST_ASSERT_FLOAT_NEAR(800.0f, y, 30.0f, "最终y≈800");
    TEST_END();
}

/* 转向 90°: 最终 theta≈90 */
static void test_turn_90(void)
{
    TEST_BEGIN("转向 90°");
    chassis_init();
    headturn(90);
    TEST_ASSERT_FALSE(chassis_is_idle(), "发起转向后非IDLE");
    int ok = run_until_idle(5000);
    TEST_ASSERT_TRUE(ok, "转向应到达");
    float x, y, th;
    chassis_get_pose(&x, &y, &th);
    TEST_ASSERT_FLOAT_NEAR(90.0f, th, 3.0f, "最终theta≈90(±3°)");
    TEST_END();
}

/* 即时停止: 运动中 chassis_stop 应立即 IDLE 且轮速=0 */
static void test_stop_midway(void)
{
    TEST_BEGIN("运动中即时停止");
    chassis_init();
    move_to_coordinate(2000.0f, 0.0f);
    /* 跑一段让它动起来 */
    for (int i = 0; i < 200; ++i) chassis_tick();
    TEST_ASSERT_FALSE(chassis_is_idle(), "200tick后仍在动");
    chassis_stop();
    TEST_ASSERT_TRUE(chassis_is_idle(), "stop后应IDLE");
    float w[4] = {9, 9, 9, 9};
    chassis_get_wheel_speed(w);
    for (int i = 0; i < 4; ++i)
        TEST_ASSERT_FLOAT_NEAR(0.0f, w[i], 0.001f, "stop后轮速=0");
    TEST_END();
}

/* 陀螺仪注入: feed_gyro 后航向采用陀螺仪值 */
static void test_gyro_feed(void)
{
    TEST_BEGIN("陀螺仪注入");
    chassis_init();
    chassis_feed_gyro(45.0f);
    float x, y, th;
    chassis_get_pose(&x, &y, &th);
    TEST_ASSERT_FLOAT_NEAR(45.0f, th, 0.1f, "注入后theta=陀螺仪值");
    chassis_clear_gyro();
    TEST_END();
}

/* set/get pose 一致性 */
static void test_set_get_pose(void)
{
    TEST_BEGIN("set/get pose 一致");
    chassis_init();
    chassis_set_pose(100.0f, 200.0f, 30.0f);
    float x, y, th;
    chassis_get_pose(&x, &y, &th);
    TEST_ASSERT_FLOAT_NEAR(100.0f, x, 0.001f, "set x=100");
    TEST_ASSERT_FLOAT_NEAR(200.0f, y, 0.001f, "set y=200");
    TEST_ASSERT_FLOAT_NEAR(30.0f, th, 0.001f, "set theta=30");
    TEST_END();
}

/* 目标已在范围内: move 到原点附近应立即到达 */
static void test_already_at_target(void)
{
    TEST_BEGIN("已在目标处立即到达");
    chassis_init();
    /* 起点在 (0,0), 目标 (0.5, 0.5) 在 POS_TOL 内 */
    bool arrived = move_to_coordinate(0.5f, 0.5f);
    TEST_ASSERT_TRUE(arrived, "目标在容差内应返回已到达");
    TEST_ASSERT_TRUE(chassis_is_idle(), "应处于IDLE");
    TEST_END();
}

/* ---- 套件汇总 ---- */
void test_chassis_run(void)
{
    TEST_SUITE("底盘控制层");
    test_init_state();
    test_move_forward();
    test_move_diagonal();
    test_turn_90();
    test_stop_midway();
    test_gyro_feed();
    test_set_get_pose();
    test_already_at_target();
}