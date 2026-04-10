/*******************************************************************************
 * main.cpp — 走马观碑赛道巡线主程序
 *
 * 改动说明（相比原版）：
 *   1. JSON 参数读取：启动时从 ../param/config.json 读取所有参数
 *   2. 分段 PID：直道/弯道自动切换 kp/kd，速度也随之调整
 *   3. 双线程解耦：
 *        - 采集线程（capture_thread）：10ms 一帧，持续抓图
 *        - 控制线程（control_thread）：20ms 一次，取最新帧处理+控制
 *      这样摄像头不会因为图像处理耗时而卡顿
 *   4. Floodfill 去背景：preprocess() 内部已处理，seed_hint 由控制线程维护
 *   5. 场景检测防抖：SceneState 成员变量管理，无 static 局部计数器
 *
 * ★ 调参指南 ★
 *   只需修改 param/config.json，无需重新编译
 *   主要调节项：
 *     - IPM逆透视/src       ← 每次换赛道/摄像头安装角度必须重新标定
 *     - PID_直道/弯道        ← 速度越高 kp 越小、kd 越大
 *     - 速度配置/base_speed  ← 从30起步，稳定后逐步提高
 ******************************************************************************/

/*ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIN6BvFeR66XS2LgNY19SYI1F0/1T47q5To3QaBy5eQiZ fy051213@gmail.com*/
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "headfile.h"
#include "param/param.hpp"

/* ═══════════════════════════ 全局配置 ══════════════════════════════════ */

static CarConfig g_cfg;             // JSON 读取的全局配置
static const char* CONFIG_PATH = "../param/config.json";

/* ═══════════════════════════ 双线程共享数据 ════════════════════════════ */

static std::mutex  g_frame_mutex;
static cv::Mat     g_latest_frame;       // 采集线程写，控制线程读
static bool        g_frame_ready = false;
static std::atomic<bool> g_running{true};

/* 用于 Floodfill seed 反馈（控制线程写，采集线程透明） */
static std::atomic<int> g_seed_hint_x{-1};

/* ═══════════════════════════ 采集线程回调 ══════════════════════════════ */
/*
 * 通过 TimerThread 每 10ms 触发一次（约100fps）
 * 只负责抓图，不做任何处理
 */

struct CaptureContext {
    cv::VideoCapture* cap;
};

static void capture_callback(void* arg)
{
    auto* ctx = static_cast<CaptureContext*>(arg);
    cv::Mat img;

    if (!ctx->cap->read(img) || img.empty()) return;

    std::lock_guard<std::mutex> lk(g_frame_mutex);
    g_latest_frame = img;   // 浅拷贝头，数据由 OpenCV 管理
    g_frame_ready  = true;
}

/* ═══════════════════════════ 控制线程回调 ══════════════════════════════ */
/*
 * 通过 TimerThread 每 20ms 触发一次（约50fps控制）
 * 负责：图像处理 → 巡线 → 场景检测 → PID → 控制输出 → 图传
 */

struct ControlContext {
    IPMConfig*          ipm_cfg;
    cv::Mat*            M;              // IPM 变换矩阵
    PIDController*      steer_pid;
    SceneState*         scene_state;
    TransmissionStreamServer* server;

    /* 状态变量（跨帧保持） */
    int  lost_frame_cnt;
    int  steer_out_last;    // 丢线时保持上次转向值

    /* 帧率统计 */
    int64_t t_last_tick;
    int     fps_cnt;

    ControlContext()
        : lost_frame_cnt(0)
        , steer_out_last(0)
        , t_last_tick(0)
        , fps_cnt(0)
    {}
};

static void control_callback(void* arg)
{
    auto* ctx = static_cast<ControlContext*>(arg);

    /* ── 取最新帧（非阻塞） ── */
    cv::Mat img;
    {
        std::lock_guard<std::mutex> lk(g_frame_mutex);
        if (!g_frame_ready || g_latest_frame.empty()) return;
        img = g_latest_frame.clone();  // 深拷贝，避免采集线程覆盖
    }

    /* ── 图像预处理（含 Floodfill） ── */
    cv::Mat img_bin, img_ipm;
    preprocess(img, img_bin, g_seed_hint_x.load());
    ipm_warp(img_bin, img_ipm, *(ctx->M));

    /* ── 迷宫法巡线 ── */
    LineInfo line_info;
    line_info_reset(line_info);
    maze_detect(img_ipm, line_info, g_cfg.line);
    detect_straight(line_info, g_cfg.line);

    /* ── 场景检测 ── */
    SceneType scene = detect_scene(img_ipm, img, line_info,
                                   *(ctx->scene_state), g_cfg.scene);

    /* ── 根据直道/弯道切换 PID 参数 ── */
    if (line_info.is_straight) {
        pid_tune(ctx->steer_pid,
                 g_cfg.pid_straight.kp,
                 g_cfg.pid_straight.ki,
                 g_cfg.pid_straight.kd);
    } else {
        pid_tune(ctx->steer_pid,
                 g_cfg.pid_curve.kp,
                 g_cfg.pid_curve.ki,
                 g_cfg.pid_curve.kd);
    }

    /* ── 控制决策 ── */
    int  drive_speed = g_cfg.speed.base_speed;
    int  steer_out   = 0;
    bool do_drive    = true;

    if (line_info.found) {
        ctx->lost_frame_cnt = 0;

        /* 更新 Floodfill 种子（用底部中心） */
        int bot_r = g_cfg.line.row_scan_start;
        if (line_info.road_width[bot_r] > 0) {
            g_seed_hint_x.store(line_info.center_x[bot_r]);
        }

        switch (scene) {

        case SCENE_INTERSECTION:
            /* 十字路口：强制直行，PID清积分 */
            pid_reset(ctx->steer_pid);
            steer_out   = 0;
            drive_speed = g_cfg.speed.speed_inter;
            break;

        case SCENE_OBSTACLE: {
            /* 红色路障：向对侧偏转，减速 */
            /* obstacle_side: false=障碍在左(向右偏), true=障碍在右(向左偏) */
            steer_out   = ctx->scene_state->obstacle_side ? -35 : 35;
            drive_speed = g_cfg.speed.speed_obstacle;
            break;
        }

        case SCENE_ZEBRA:
            /* 斑马线：正常巡线速度（如需停车在此取消注释） */
            steer_out   = (int)pid_compute(ctx->steer_pid, line_info.error);
            drive_speed = g_cfg.speed.base_speed;
            // car_stop(); do_drive = false; g_running = false; // 终点停车
            break;

        case SCENE_ROUNDABOUT_IN:
            steer_out   = (int)pid_compute(ctx->steer_pid, line_info.error);
            drive_speed = g_cfg.speed.speed_curve;
            break;

        default: /* SCENE_NORMAL */
            steer_out = (int)pid_compute(ctx->steer_pid, line_info.error);

            /* 动态调速：直道快，弯道慢 */
            if (line_info.is_straight) {
                drive_speed = g_cfg.speed.base_speed;
            } else {
                /* 偏差越大越减速 */
                float abs_err = std::abs(line_info.error);
                if      (abs_err > 40.0f) drive_speed = g_cfg.speed.speed_curve;
                else if (abs_err > 20.0f) drive_speed = (g_cfg.speed.base_speed +
                                                          g_cfg.speed.speed_curve) / 2;
                else                      drive_speed = g_cfg.speed.base_speed;
            }
            break;
        }

        ctx->steer_out_last = steer_out;

    } else {
        /* ── 丢线处理 ── */
        ctx->lost_frame_cnt++;
        pid_reset(ctx->steer_pid);
        g_seed_hint_x.store(-1);  // 丢线时重置seed，下帧重新搜索

        if (ctx->lost_frame_cnt <= 10) {
            /* 短暂丢线：保持上次转向，减速 */
            steer_out   = ctx->steer_out_last;
            drive_speed = g_cfg.speed.speed_lost;
        } else if (ctx->lost_frame_cnt <= 30) {
            /* 中度丢线：回中缓行 */
            steer_out   = 0;
            drive_speed = g_cfg.speed.speed_lost / 2;
        } else {
            /* 长时间丢线：停车保护 */
            car_stop();
            do_drive = false;
            static bool warned = false;
            if (!warned) {
                std::cerr << "[WARN] 丢线超过30帧，停车保护\n";
                warned = true;
            }
        }
    }

    /* ── 执行控制输出 ── */
    if (do_drive) {
        car_drive(drive_speed, steer_out);
    }

    /* ── 图传调试 ── */
    if (g_cfg.debug.debug_stream && ctx->server->is_running()) {
        cv::Mat img_debug;
        cv::cvtColor(img_ipm, img_debug, cv::COLOR_GRAY2BGR);
        draw_line_result(img_debug, line_info, g_cfg.line);

        /* 叠加场景/速度/转向信息 */
        const char* scene_str[] = {
            "NORMAL", "INTER", "ROUND",
            "ROUND_IN", "OBSTACLE", "SLOPE", "ZEBRA"
        };
        char buf[80];
        snprintf(buf, sizeof(buf), "S:%s spd:%d st:%d %s",
                 scene_str[(int)scene],
                 drive_speed, steer_out,
                 line_info.is_straight ? "STRAIGHT" : "CURVE");
        cv::putText(img_debug, buf,
                    cv::Point(2, img_debug.rows - 4),
                    cv::FONT_HERSHEY_PLAIN, 0.7,
                    cv::Scalar(255, 200, 0), 1);

        ctx->server->update_frame_mat(img_debug);
    }

    /* ── 帧率统计 ── */
    if (g_cfg.debug.debug_print_fps) {
        ctx->fps_cnt++;
        int64_t t_now = cv::getTickCount();
        if (ctx->t_last_tick > 0) {
            double dt = (double)(t_now - ctx->t_last_tick) / cv::getTickFrequency();
            if (dt >= 1.0) {
                std::cout << "[FPS] " << ctx->fps_cnt << " fps"
                          << "  err=" << line_info.error
                          << "  " << (line_info.is_straight ? "直道" : "弯道")
                          << "\n";
                ctx->fps_cnt  = 0;
                ctx->t_last_tick = t_now;
            }
        } else {
            ctx->t_last_tick = t_now;
        }
    }
}

/* ═══════════════════════════ main ══════════════════════════════════════ */

int main()
{
    /* ─── 1. 读取 JSON 配置 ─── */
    bool cfg_ok = load_config(g_cfg, CONFIG_PATH);
    if (!cfg_ok) {
        std::cout << "[WARN] 使用所有默认参数\n";
    }

    int img_w = g_cfg.camera.img_width;
    int img_h = g_cfg.camera.img_height;

    /* ─── 2. 启动图传服务器 ─── */
    TransmissionStreamServer server;
    if (server.start_server(g_cfg.debug.transmission_port) < 0) {
        std::cerr << "[ERROR] 图传服务器启动失败\n";
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    /* ─── 3. 摄像头初始化 ─── */
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FOURCC,
            cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  img_w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, img_h);
    cap.set(cv::CAP_PROP_FPS, g_cfg.camera.fps);

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] 摄像头打开失败\n";
        return -1;
    }
    std::cout << "[INFO] 摄像头已打开 "
              << img_w<< "x" << img_h
              << " @ " << g_cfg.camera.fps << "fps\n";

    /* ─── 4. 电机/舵机初始化 ─── */
    if (car_init() < 0) {
        std::cerr << "[WARN] 电机初始化失败，进入纯图像调试模式\n";
        /* 不退出，方便调试图像处理 */
    }

    /* ─── 5. IPM 变换矩阵（一次计算） ─── */
    IPMConfig ipm_cfg;
    ipm_init(ipm_cfg, g_cfg);
    cv::Mat M = get_ipm_matrix(ipm_cfg);

    /* ─── 6. PID 初始化（以直道参数为起点） ─── */
    PIDController steer_pid;
    pid_init(&steer_pid,
             g_cfg.pid_straight.kp,
             g_cfg.pid_straight.ki,
             g_cfg.pid_straight.kd,
             g_cfg.pid_straight.i_limit,
             g_cfg.pid_straight.out_limit,
             false);  // 位置式 PID

    /* ─── 7. 场景状态机 ─── */
    SceneState scene_state;
    scene_state_init(scene_state);

    /* ─── 8. 线程上下文 ─── */
    CaptureContext cap_ctx;
    cap_ctx.cap = &cap;

    ControlContext ctl_ctx;
    ctl_ctx.ipm_cfg     = &ipm_cfg;
    ctl_ctx.M           = &M;
    ctl_ctx.steer_pid   = &steer_pid;
    ctl_ctx.scene_state = &scene_state;
    ctl_ctx.server      = &server;

    /* ─── 9. 启动双线程 ─── */
    /*
     * 采集线程：10ms = 100fps 抓图（只抓，不处理）
     * 控制线程：20ms = 50fps  处理+控制
     *
     * 这样控制帧率独立于采集帧率，互不阻塞
     */
    TimerThread cap_thread(capture_callback, &cap_ctx,  10); // 10ms
    TimerThread ctl_thread(control_callback, &ctl_ctx,  20); // 20ms

    cap_thread.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 让采集先跑一会
    ctl_thread.start();

    std::cout << "[INFO] 巡线程序启动完成\n";
    std::cout << "[INFO] 图传地址：http://<板子IP>:"
              << g_cfg.debug.transmission_port << "\n";
    std::cout << "[INFO] 按 Ctrl+C 退出\n";

    /* ─── 10. 主线程等待（线程为 detach 模式，主线程挂起即可） ─── */
    while (g_running.load() && server.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    /* ─── 清理 ─── */
    car_stop();
    cap.release();
    server.stop_server();
    std::cout << "[INFO] 程序退出\n";
    return 0;
}