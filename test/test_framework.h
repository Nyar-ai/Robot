/**
 * @file    test_framework.h
 * @brief   极简 C 单元测试框架(零依赖, 仅 stdio)
 *
 * 设计:
 *   - 每个测试函数为 void f(void), 用 TEST_BEGIN/TEST_END 包裹
 *   - 断言失败时打印位置并 return(跳过本用例剩余部分)
 *   - 统计计数定义在 test_main.c, 头文件 extern 声明, 跨文件共享
 *
 * 用法:
 *   void test_xxx(void) {
 *       TEST_BEGIN("用例名");
 *       TEST_ASSERT_FLOAT_NEAR(1.0f, foo(), 0.01f, "foo应≈1");
 *       TEST_END();
 *   }
 */
#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>

/* 全局统计(定义在 test_main.c, 跨文件共享) */
extern int g_test_pass;
extern int g_test_fail;

/* 套件标记 */
#define TEST_SUITE(name) \
    do { printf("\n========== %s ==========\n", name); } while (0)

/* 用例开始: 打印名字(不换行, 等结果补齐) */
#define TEST_BEGIN(name) \
    do { printf("  [%-32s] ", name); fflush(stdout); } while (0)

/* 用例通过 */
#define TEST_END() \
    do { printf("OK\n"); g_test_pass++; } while (0)

/* 用例失败: 打印位置后 return 退出当前测试函数 */
#define TEST_FAIL(msg) \
    do { \
        printf("FAIL\n    >> %s (%s:%d)\n", msg, __FILE__, __LINE__); \
        g_test_fail++; \
        return; \
    } while (0)

/* 通用断言 */
#define TEST_ASSERT(cond, msg) \
    do { if (!(cond)) TEST_FAIL(msg); } while (0)

#define TEST_ASSERT_TRUE(cond, msg)  TEST_ASSERT((cond), msg)
#define TEST_ASSERT_FALSE(cond, msg) TEST_ASSERT(!(cond), msg)

/* 整数相等 */
#define TEST_ASSERT_INT_EQ(exp, act, msg) \
    do { \
        int _e = (int)(exp); int _a = (int)(act); \
        if (_e != _a) { \
            printf("FAIL\n    >> %s (期望=%d 实际=%d) (%s:%d)\n", \
                   msg, _e, _a, __FILE__, __LINE__); \
            g_test_fail++; return; \
        } \
    } while (0)

/* 浮点近似(核心断言, 处理浮点误差) */
#define TEST_ASSERT_FLOAT_NEAR(exp, act, tol, msg) \
    do { \
        float _e = (float)(exp); float _a = (float)(act); float _t = (float)(tol); \
        float _d = _a - _e; if (_d < 0.0f) _d = -_d; \
        if (_d > _t) { \
            printf("FAIL\n    >> %s (期望=%g 实际=%g 容差=%g 差=%g) (%s:%d)\n", \
                   msg, _e, _a, _t, _d, __FILE__, __LINE__); \
            g_test_fail++; return; \
        } \
    } while (0)

/* 汇总(放在 main 末尾, 返回失败数作为进程退出码) */
#define TEST_SUMMARY() \
    do { \
        printf("\n========== 汇总 ==========\n"); \
        printf("  通过: %d\n  失败: %d\n", g_test_pass, g_test_fail); \
        printf("==========================\n"); \
        return g_test_fail; \
    } while (0)

#endif /* TEST_FRAMEWORK_H */