/**
 * @file    test_mecanum.c
 * @brief   麦克纳姆轮运动学单元测试
 *
 * 验证 mecanum.h 中 Inverse/Forward/SetGeometry 的:
 *   - 基本运动分解(前进/平移/旋转)
 *   - 正逆解闭环一致性
 *   - 几何参数运行时覆盖
 *   - NULL 容错
 */
#include "test_framework.h"
#include "mecanum.h"
#include <math.h>

/* 测试套件入口(由 test_main.c 调用) */
void test_mecanum_run(void);

/* ---- 各用例 ---- */

/* 纯前进: vx>0, 4轮都等于 vx */
static void test_pure_forward(void)
{
    TEST_BEGIN("纯前进 vx>0");
    float w[MECANUM_NUM];
    Mecanum_Inverse(100.0f, 0.0f, 0.0f, w);
    for (int i = 0; i < MECANUM_NUM; ++i) {
        TEST_ASSERT_FLOAT_NEAR(100.0f, w[i], 0.001f, "前进时4轮应=vx");
    }
    TEST_END();
}

/* 纯后退: vx<0, 4轮都等于 vx */
static void test_pure_backward(void)
{
    TEST_BEGIN("纯后退 vx<0");
    float w[MECANUM_NUM];
    Mecanum_Inverse(-100.0f, 0.0f, 0.0f, w);
    for (int i = 0; i < MECANUM_NUM; ++i) {
        TEST_ASSERT_FLOAT_NEAR(-100.0f, w[i], 0.001f, "后退时4轮应=vx");
    }
    TEST_END();
}

/* 纯左移(vy>0): X布局下 LF/RB 与 RF/LB 符号相反 */
static void test_pure_strafe_left(void)
{
    TEST_BEGIN("纯左移 vy>0");
    float w[MECANUM_NUM];
    Mecanum_Inverse(0.0f, 100.0f, 0.0f, w);
    /* X布局(FLIP_Y=1): vLF=-vy, vRF=+vy, vLB=+vy, vRB=-vy */
    TEST_ASSERT_FLOAT_NEAR(-100.0f, w[MECANUM_LF], 0.001f, "左前轮速");
    TEST_ASSERT_FLOAT_NEAR( 100.0f, w[MECANUM_RF], 0.001f, "右前轮速");
    TEST_ASSERT_FLOAT_NEAR( 100.0f, w[MECANUM_LB], 0.001f, "左后轮速");
    TEST_ASSERT_FLOAT_NEAR(-100.0f, w[MECANUM_RB], 0.001f, "右后轮速");
    TEST_END();
}

/* 纯逆时针旋转(ω>0): 左轮负, 右轮正(前进方向驱动) */
static void test_pure_ccw_rotation(void)
{
    TEST_BEGIN("纯逆时针旋转 ω>0");
    float w[MECANUM_NUM];
    /* k = (Lx+Ly)*ω = (180+180)*1 = 360 mm/s */
    Mecanum_Inverse(0.0f, 0.0f, 1.0f, w);
    /* vLF = -k, vRF = +k, vLB = -k, vRB = +k */
    TEST_ASSERT_FLOAT_NEAR(-360.0f, w[MECANUM_LF], 0.001f, "左前(ω)");
    TEST_ASSERT_FLOAT_NEAR( 360.0f, w[MECANUM_RF], 0.001f, "右前(ω)");
    TEST_ASSERT_FLOAT_NEAR(-360.0f, w[MECANUM_LB], 0.001f, "左后(ω)");
    TEST_ASSERT_FLOAT_NEAR( 360.0f, w[MECANUM_RB], 0.001f, "右后(ω)");
    TEST_END();
}

/* 正逆解闭环: Inverse(v) 再 Forward 应还原 v */
static void test_forward_inverse_roundtrip(void)
{
    TEST_BEGIN("正逆解闭环还原");
    float w[MECANUM_NUM];
    float vx = 150.0f, vy = -80.0f, omega = 0.5f;
    Mecanum_Inverse(vx, vy, omega, w);
    float r_vx, r_vy, r_omega;
    Mecanum_Forward(w, &r_vx, &r_vy, &r_omega);
    TEST_ASSERT_FLOAT_NEAR(vx,    r_vx,    0.001f, "还原 vx");
    TEST_ASSERT_FLOAT_NEAR(vy,    r_vy,    0.001f, "还原 vy");
    TEST_ASSERT_FLOAT_NEAR(omega, r_omega, 0.0001f, "还原 omega");
    TEST_END();
}

/* 修改几何参数后 k=(Lx+Ly)*ω 应相应变化 */
static void test_set_geometry(void)
{
    TEST_BEGIN("SetGeometry 改参数");
    Mecanum_SetGeometry(100.0f, 100.0f, 30.0f); /* Lx+Ly=200 */
    float w[MECANUM_NUM];
    Mecanum_Inverse(0.0f, 0.0f, 1.0f, w);
    /* k = 200*1 = 200 */
    TEST_ASSERT_FLOAT_NEAR(-200.0f, w[MECANUM_LF], 0.001f, "改参数后左前");
    TEST_ASSERT_FLOAT_NEAR( 200.0f, w[MECANUM_RF], 0.001f, "改参数后右前");
    /* 恢复默认, 避免污染后续测试 */
    Mecanum_SetGeometry(MECANUM_LX_MM, MECANUM_LY_MM, MECANUM_WHEEL_RADIUS_MM);
    TEST_END();
}

/* NULL 容错: Forward(NULL,...) 不应崩溃, 且把输出置 0 */
static void test_forward_null_safe(void)
{
    TEST_BEGIN("Forward(NULL) 容错");
    float vx = 999.0f, vy = 999.0f, om = 999.0f;
    Mecanum_Forward(NULL, &vx, &vy, &om);
    TEST_ASSERT_FLOAT_NEAR(0.0f, vx, 0.001f, "NULL时vx=0");
    TEST_ASSERT_FLOAT_NEAR(0.0f, vy, 0.001f, "NULL时vy=0");
    TEST_ASSERT_FLOAT_NEAR(0.0f, om, 0.001f, "NULL时omega=0");
    TEST_END();
}

/* ---- 套件汇总 ---- */
void test_mecanum_run(void)
{
    TEST_SUITE("麦克纳姆轮运动学");
    test_pure_forward();
    test_pure_backward();
    test_pure_strafe_left();
    test_pure_ccw_rotation();
    test_forward_inverse_roundtrip();
    test_set_geometry();
    test_forward_null_safe();
}