/**
 * @file    test_main.c
 * @brief   单元测试入口: 定义全局统计, 依次运行各套件
 *
 * 编译: 见 Makefile. 运行后返回失败数(0=全通过).
 */
#include "test_framework.h"
#include <stdio.h>

/* 全局统计(头文件 extern 声明, 此处定义) */
int g_test_pass = 0;
int g_test_fail = 0;

/* 各套件入口 */
void test_mecanum_run(void);
void test_scurve_run(void);
void test_chassis_run(void);

int main(int argc, char **argv)
{
    (void)argc; (void)argv;

    printf("##############################################################\n");
    printf("# 快递分拣机器人 - PC 端单元测试 (脱离硬件, 验证纯算法/逻辑层) #\n");
    printf("##############################################################\n");

    test_mecanum_run();
    test_scurve_run();
    test_chassis_run();

    printf("\n");
    TEST_SUMMARY();
}