/**
 * @file    chassis.c
 * @brief   底盘控制层实现
 *
 * 数据流:
 *   move_to_coordinate(tx,ty) ──┐
 *   headturn(angle)          ──┤
 *                              ├─► [状态机] ─► Scurve_Planner(平移/旋转)
 *   chassis_tick (1ms)       ──┘                │
 *                                                ▼
 *                                          速度分解 (vx_b, vy_b, ω)
 *                                                │
 *                            里程计积分 ◄────────┤
 *                                                ▼
 *                                        Mecanum_Inverse → 4 轮速
 *
 * 角度来源: chassis_feed_gyro 注入则用陀螺仪, 否则用 ω 积分 θ
 *
 * 平移航向修正(对齐 1064tiaozheng Omega_attitude_solution):
 *   平移模式下, 对 (target_heading - current_heading) 做位置式 PID,
 *   输出 ω 叠加到麦轮逆解, 在平移过程中持续纠偏.
 */
#include "chassis.h"
#include "scurve.h"
#include "mecanum.h"
#include <math.h>

/* ---- 偏航 PID 状态 ------------------------------------------------------ */
typedef struct {
    float kp, ki, kd;
    float imax;         /* 输出限幅 (deg/s) */
    float integral;     /* 积分累加 */
    float prev_error;   /* error[n-1] */
} YawPID_t;

static YawPID_t g_yaw_pid = {
    CHASSIS_YAW_KP, CHASSIS_YAW_KI, CHASSIS_YAW_KD,
    CHASSIS_YAW_IMAX,
    0.0f, 0.0f
};

/* ---- 内部状态 ----------------------------------------------------------- */
typedef enum {
    CH_MODE_IDLE = 0,
    CH_MODE_MOVING,
    CH_MODE_TURNING
} Chassis_Mode;

typedef struct {
    /* 位姿(世界系) */
    float x;          /* mm   */
    float y;          /* mm   */
    float theta_deg;  /* deg  (世界系航向) */

    /* 陀螺仪注入 */
    bool  gyro_used;
    float gyro_theta_deg;

    /* 运动模式 */
    Chassis_Mode mode;
    bool arrived;          /* 当前运动是否已完成 */

    /* 航向锁定 (平移时自动保持) */
    float target_heading_deg;  /* deg, 平移时要保持的绝对角度 */

    /* 平移规划 */
    Scurve_Planner trans_planner;
    float trans_phi_rad;   /* 本次平移在车体系下的方位角(发起时锁定) */
    float move_tx, move_ty;/* 本次平移目标(用于判定目标是否变化) */

    /* 旋转规划 */
    Scurve_Planner rot_planner;
    float rot_dir;         /* +1 / -1 */
    float turn_target_deg; /* 本次转向目标绝对角度 */

    /* 输出 */
    float wheel_speed[MECANUM_NUM]; /* 4 轮目标线速度 mm/s */
} Chassis_t;

static Chassis_t g_chassis;

/* ---- 内部辅助 ----------------------------------------------------------- */

/* 当前使用的航向角(deg): 优先陀螺仪, 否则里程计 */
static float chassis_theta_deg(void)
{
    return g_chassis.gyro_used ? g_chassis.gyro_theta_deg : g_chassis.theta_deg;
}

static float deg2rad(float d) { return d * (float)(3.14159265358979323846 / 180.0); }
static float rad2deg(float r) { return r * (float)(180.0 / 3.14159265358979323846); }

/* 把角度归一到 (-180, 180] */
static float normalize_angle_deg(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static float fabsf_local(float v) { return v < 0.0f ? -v : v; }

/* ---- 偏航 PID (位置式) ---- */
static float yaw_pid_compute(float error)
{
    YawPID_t *p = &g_yaw_pid;

    /* 积分限幅 + 抗饱和 */
    p->integral += error;
    if (p->integral >  p->imax) p->integral =  p->imax;
    if (p->integral < -p->imax) p->integral = -p->imax;

    float output = p->kp * error
                 + p->ki * p->integral
                 + p->kd * (error - p->prev_error);

    p->prev_error = error;

    /* 输出限幅 */
    if (output >  p->imax) output =  p->imax;
    if (output < -p->imax) output = -p->imax;

    return output;
}

static void yaw_pid_reset(void)
{
    g_yaw_pid.integral   = 0.0f;
    g_yaw_pid.prev_error = 0.0f;
}

/* ---- 对外接口 ----------------------------------------------------------- */

void chassis_init(void)
{
    Scurve_Config trans_cfg = {
        .max_speed = CHASSIS_TRANS_MAX_SPEED,
        .max_accel = CHASSIS_TRANS_MAX_ACCEL,
        .min_speed = CHASSIS_TRANS_MIN_SPEED
    };
    Scurve_Config rot_cfg = {
        .max_speed = CHASSIS_ROT_MAX_SPEED,
        .max_accel = CHASSIS_ROT_MAX_ACCEL,
        .min_speed = CHASSIS_ROT_MIN_SPEED
    };
    Scurve_Init(&g_chassis.trans_planner, &trans_cfg);
    Scurve_Init(&g_chassis.rot_planner,   &rot_cfg);

    g_chassis.x = 0.0f;
    g_chassis.y = 0.0f;
    g_chassis.theta_deg = 0.0f;

    g_chassis.gyro_used = false;
    g_chassis.gyro_theta_deg = 0.0f;

    g_chassis.mode = CH_MODE_IDLE;
    g_chassis.arrived = true;

    g_chassis.target_heading_deg = 0.0f;

    g_chassis.trans_phi_rad = 0.0f;
    g_chassis.move_tx = 0.0f;
    g_chassis.move_ty = 0.0f;

    g_chassis.rot_dir = 1.0f;
    g_chassis.turn_target_deg = 0.0f;

    for (int i = 0; i < MECANUM_NUM; ++i) g_chassis.wheel_speed[i] = 0.0f;

    /* 重置偏航 PID 状态 */
    yaw_pid_reset();
}

void chassis_feed_gyro(float gyro_deg)
{
    g_chassis.gyro_theta_deg = normalize_angle_deg(gyro_deg);
    g_chassis.gyro_used = true;
    /* 用陀螺仪角同步里程计 θ, 保证 chassis_theta_deg 一致 */
    g_chassis.theta_deg = g_chassis.gyro_theta_deg;
}

void chassis_clear_gyro(void)
{
    g_chassis.gyro_used = false;
}

void chassis_set_target_heading(float heading_deg)
{
    g_chassis.target_heading_deg = normalize_angle_deg(heading_deg);
    yaw_pid_reset();  /* 切换目标时清 PID 历史 */
}

void chassis_set_yaw_pid(float kp, float ki, float kd, float imax)
{
    if (kp  > 0.0f) g_yaw_pid.kp   = kp;
    if (ki  > 0.0f) g_yaw_pid.ki   = ki;
    if (kd  > 0.0f) g_yaw_pid.kd   = kd;
    if (imax> 0.0f) g_yaw_pid.imax = imax;
}

void chassis_set_pose(float x, float y, float theta_deg)
{
    g_chassis.x = x;
    g_chassis.y = y;
    g_chassis.theta_deg = normalize_angle_deg(theta_deg);
    if (g_chassis.gyro_used) g_chassis.gyro_theta_deg = g_chassis.theta_deg;
}

void chassis_get_pose(float *x, float *y, float *theta_deg)
{
    if (x)         *x = g_chassis.x;
    if (y)         *y = g_chassis.y;
    if (theta_deg) *theta_deg = chassis_theta_deg();
}

bool chassis_is_idle(void)
{
    return g_chassis.mode == CH_MODE_IDLE;
}

void chassis_stop(void)
{
    Scurve_Stop(&g_chassis.trans_planner);
    Scurve_Stop(&g_chassis.rot_planner);
    g_chassis.mode = CH_MODE_IDLE;
    g_chassis.arrived = true;
    for (int i = 0; i < MECANUM_NUM; ++i) g_chassis.wheel_speed[i] = 0.0f;
    yaw_pid_reset();
}

void chassis_get_wheel_speed(float w[4])
{
    for (int i = 0; i < MECANUM_NUM; ++i) w[i] = g_chassis.wheel_speed[i];
}

bool move_to_coordinate(float tx, float ty)
{
    /* 目标变化 → 重新规划 */
    float dx = tx - g_chassis.move_tx;
    float dy = ty - g_chassis.move_ty;
    bool target_changed = (g_chassis.mode != CH_MODE_MOVING) ||
                          (fabsf_local(dx) > 0.5f) || (fabsf_local(dy) > 0.5f);

    if (target_changed) {
        /* 切换到平移模式前, 先停掉旋转 */
        if (g_chassis.mode == CH_MODE_TURNING) {
            Scurve_Stop(&g_chassis.rot_planner);
        }

        float theta_rad = deg2rad(chassis_theta_deg());
        /* 世界系误差 */
        float ewx = tx - g_chassis.x;
        float ewy = ty - g_chassis.y;
        /* 世界 → 车体 */
        float ebx =  ewx * cosf(theta_rad) + ewy * sinf(theta_rad);
        float eby = -ewx * sinf(theta_rad) + ewy * cosf(theta_rad);
        float d   = sqrtf(ebx * ebx + eby * eby);

        g_chassis.move_tx = tx;
        g_chassis.move_ty = ty;
        g_chassis.trans_phi_rad = atan2f(eby, ebx); /* 车体系方位角 */

        /* 锁定当前航向作为平移保持目标 */
        g_chassis.target_heading_deg = chassis_theta_deg();
        yaw_pid_reset();   /* 清 PID 历史 */

        if (d < CHASSIS_POS_TOL_MM) {
            /* 已在目标范围内 */
            g_chassis.mode = CH_MODE_IDLE;
            g_chassis.arrived = true;
        } else {
            Scurve_MoveTo(&g_chassis.trans_planner, d);
            g_chassis.mode = CH_MODE_MOVING;
            g_chassis.arrived = false;
        }
    }

    return g_chassis.arrived;
}

bool headturn(int16_t angle)
{
    float target = (float)angle;

    /* 目标变化 → 重新规划 */
    bool target_changed = (g_chassis.mode != CH_MODE_TURNING) ||
                          (fabsf_local(normalize_angle_deg(target - g_chassis.turn_target_deg)) > 0.5f);

    if (target_changed) {
        /* 切换到旋转模式前, 先停掉平移 */
        if (g_chassis.mode == CH_MODE_MOVING) {
            Scurve_Stop(&g_chassis.trans_planner);
        }

        float dtheta = normalize_angle_deg(target - chassis_theta_deg());
        g_chassis.turn_target_deg = target;
        g_chassis.rot_dir = (dtheta >= 0.0f) ? 1.0f : -1.0f;

        yaw_pid_reset();   /* 清 PID 历史 */

        if (fabsf_local(dtheta) < CHASSIS_ANG_TOL_DEG) {
            g_chassis.mode = CH_MODE_IDLE;
            g_chassis.arrived = true;
        } else {
            /* 规划器按"距离(角度差绝对值)"规划, 方向由 rot_dir 体现 */
            Scurve_MoveTo(&g_chassis.rot_planner, fabsf_local(dtheta));
            g_chassis.mode = CH_MODE_TURNING;
            g_chassis.arrived = false;
        }
    }

    return g_chassis.arrived;
}

void chassis_tick(void)
{
    float dt = CHASSIS_TICK_DT_S;
    float vx_b = 0.0f, vy_b = 0.0f, omega = 0.0f; /* 车体系目标速度 */

    switch (g_chassis.mode) {
    case CH_MODE_MOVING: {
        /* S 曲线给出沿 phi 方向的速度(标量, 始终 >= 0) */
        float v = Scurve_Update(&g_chassis.trans_planner, dt);
        vx_b = v * cosf(g_chassis.trans_phi_rad);
        vy_b = v * sinf(g_chassis.trans_phi_rad);

        /* ---- 航向锁定: 陀螺仪偏航 PID 闭环修正 (对齐 1064tiaozheng) ----
         * 在平移过程中, 对 (target_heading - current_heading) 做位置式 PID,
         * 输出 ω 叠加到麦轮逆解, 持续纠正车轮打滑/地面不平导致的偏航漂移.
         * 有陀螺仪时用 gyro_theta, 否则用里程计 theta. */
        {
            float heading_err = normalize_angle_deg(
                g_chassis.target_heading_deg - chassis_theta_deg());
            omega = deg2rad(yaw_pid_compute(heading_err));
        }

        /* 到达判定: 规划器空闲 且 实际位置误差足够小 */
        if (Scurve_IsIdle(&g_chassis.trans_planner)) {
            float ewx = g_chassis.move_tx - g_chassis.x;
            float ewy = g_chassis.move_ty - g_chassis.y;
            float d = sqrtf(ewx * ewx + ewy * ewy);
            if (d < CHASSIS_POS_TOL_MM * 3.0f) {
                g_chassis.mode = CH_MODE_IDLE;
                g_chassis.arrived = true;
                vx_b = vy_b = 0.0f;
                omega = 0.0f;
                yaw_pid_reset();
            } else {
                /* 规划器已停但没到(积分漂移), 重新规划剩余距离 */
                float theta_rad = deg2rad(chassis_theta_deg());
                float ebx =  ewx * cosf(theta_rad) + ewy * sinf(theta_rad);
                float eby = -ewx * sinf(theta_rad) + ewy * cosf(theta_rad);
                g_chassis.trans_phi_rad = atan2f(eby, ebx);
                Scurve_MoveTo(&g_chassis.trans_planner, d);
            }
        }
        break;
    }
    case CH_MODE_TURNING: {
        /* S 曲线给出角速度幅值(deg/s, >=0), 乘方向 */
        float w_deg = Scurve_Update(&g_chassis.rot_planner, dt);
        omega = deg2rad(w_deg * g_chassis.rot_dir);
        vx_b = 0.0f;
        vy_b = 0.0f;

        if (Scurve_IsIdle(&g_chassis.rot_planner)) {
            float err = fabsf_local(normalize_angle_deg(g_chassis.turn_target_deg - chassis_theta_deg()));
            if (err < CHASSIS_ANG_TOL_DEG * 2.0f) {
                g_chassis.mode = CH_MODE_IDLE;
                g_chassis.arrived = true;
                omega = 0.0f;
                yaw_pid_reset();
            } else {
                /* 没转到位, 重新规划剩余角度 */
                float dtheta = normalize_angle_deg(g_chassis.turn_target_deg - chassis_theta_deg());
                g_chassis.rot_dir = (dtheta >= 0.0f) ? 1.0f : -1.0f;
                Scurve_MoveTo(&g_chassis.rot_planner, fabsf_local(dtheta));
            }
        }
        break;
    }
    case CH_MODE_IDLE:
    default:
        vx_b = vy_b = omega = 0.0f;
        break;
    }

    /* 麦轮逆解 → 4 轮目标线速度 */
    Mecanum_Inverse(vx_b, vy_b, omega, g_chassis.wheel_speed);

    /* ---- 里程计积分 ---- */
    /* 用"当前 θ"做车体→世界旋转(θ 优先陀螺仪) */
    float theta_rad = deg2rad(chassis_theta_deg());
    float vx_w = vx_b * cosf(theta_rad) - vy_b * sinf(theta_rad);
    float vy_w = vx_b * sinf(theta_rad) + vy_b * cosf(theta_rad);
    g_chassis.x += vx_w * dt;
    g_chassis.y += vy_w * dt;

    /* θ: 有陀螺仪时不积分(由 chassis_feed_gyro 提供); 无陀螺仪时用 ω 积分 */
    if (!g_chassis.gyro_used) {
        g_chassis.theta_deg = normalize_angle_deg(g_chassis.theta_deg + rad2deg(omega) * dt);
    }
}