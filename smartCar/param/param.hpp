#pragma once
/*******************************************************************************
 * param.hpp — JSON参数读取头文件
 *
 * 用法：
 *   #include "param/param.hpp"
 *   CarConfig cfg;
 *   load_config(cfg, "../param/config.json");
 *
 * 现场调参：只需修改 param/config.json，无需重新编译
 ******************************************************************************/

#include <string>
#include <fstream>
#include <iostream>
#include "../include/json.hpp"   // nlohmann JSON 单头文件库

using json = nlohmann::json;

/* ─────────────────────── 子结构体 ─────────────────────── */

struct PIDParam {
    float kp       = 0.9f;
    float ki       = 0.005f;
    float kd       = 0.6f;
    float i_limit  = 40.0f;
    float out_limit = 100.0f;
};

struct IPMParam {
    // src/dst 各4个点，顺序: [左下, 右下, 右上, 左上]
    float src[4][2] = {
        {15.0f, 115.0f}, {145.0f, 115.0f},
        {105.0f, 66.0f}, { 55.0f,  66.0f}
    };
    float dst[4][2] = {
        { 20.0f, 119.0f}, {140.0f, 119.0f},
        {140.0f,   0.0f}, { 20.0f,   0.0f}
    };
};

struct LineParam {
    int   row_scan_start    = 110;
    int   row_scan_end      =  20;
    int   road_width_min    =  15;
    int   search_radius     =  40;
    float straight_var_thresh = 5.0f;
};

struct SceneParam {
    int   inter_width_thresh   = 100;
    int   inter_confirm_frames =   3;
    int   inter_pass_frames    =  25;
    float obstacle_red_ratio   = 0.05f;
    int   zebra_stripe_min     =   4;
};

struct SpeedParam {
    int base_speed     = 50;
    int speed_curve    = 38;
    int speed_inter    = 35;
    int speed_obstacle = 25;
    int speed_lost     = 20;
};

struct CameraParam {
    int img_width  = 160;
    int img_height = 120;
    int fps        =  60;
};

struct DebugParam {
    bool debug_stream    = true;
    bool debug_print_fps = false;
    int  transmission_port = 8080;
};

/* ─────────────────────── 总配置结构体 ─────────────────────── */

struct CarConfig {
    CameraParam camera;
    SpeedParam  speed;
    PIDParam    pid_straight;
    PIDParam    pid_curve;
    IPMParam    ipm;
    LineParam   line;
    SceneParam  scene;
    DebugParam  debug;
};

/* ─────────────────────── 辅助：安全读取 ─────────────────────── */

// 尝试从 js[key] 读取 T 类型，失败时保留默认值 def 并打印警告
template<typename T>
static T js_get(const json& js, const std::string& key, T def)
{
    if (js.contains(key)) {
        try { return js.at(key).get<T>(); }
        catch (...) {
            std::cerr << "[param] 警告: 解析 \"" << key << "\" 失败，使用默认值\n";
        }
    }
    return def;
}

/* ─────────────────────── 读取PID ─────────────────────── */
static void parse_pid(PIDParam& p, const json& js)
{
    p.kp        = js_get(js, "kp",        p.kp);
    p.ki        = js_get(js, "ki",        p.ki);
    p.kd        = js_get(js, "kd",        p.kd);
    p.i_limit   = js_get(js, "i_limit",   p.i_limit);
    p.out_limit = js_get(js, "out_limit", p.out_limit);
}

/* ─────────────────────── 主加载函数 ─────────────────────── */

inline bool load_config(CarConfig& cfg, const std::string& json_path)
{
    std::ifstream ifs(json_path);
    if (!ifs.good()) {
        std::cerr << "[param] 错误: 找不到配置文件 " << json_path << "\n";
        std::cerr << "[param] 将使用所有默认参数继续运行\n";
        return false;
    }

    json js;
    try {
        ifs >> js;
    } catch (const std::exception& e) {
        std::cerr << "[param] JSON解析失败: " << e.what() << "\n";
        std::cerr << "[param] 将使用所有默认参数继续运行\n";
        return false;
    }

    /* ── 摄像头 ── */
    if (js.contains("摄像头配置")) {
        auto& jc = js["摄像头配置"];
        cfg.camera.img_width  = js_get(jc, "img_width",  cfg.camera.img_width);
        cfg.camera.img_height = js_get(jc, "img_height", cfg.camera.img_height);
        cfg.camera.fps        = js_get(jc, "fps",        cfg.camera.fps);
    }

    /* ── 速度 ── */
    if (js.contains("速度配置")) {
        auto& jv = js["速度配置"];
        cfg.speed.base_speed     = js_get(jv, "base_speed",     cfg.speed.base_speed);
        cfg.speed.speed_curve    = js_get(jv, "speed_curve",    cfg.speed.speed_curve);
        cfg.speed.speed_inter    = js_get(jv, "speed_inter",    cfg.speed.speed_inter);
        cfg.speed.speed_obstacle = js_get(jv, "speed_obstacle", cfg.speed.speed_obstacle);
        cfg.speed.speed_lost     = js_get(jv, "speed_lost",     cfg.speed.speed_lost);
    }

    /* ── PID直道 ── */
    if (js.contains("PID_直道")) parse_pid(cfg.pid_straight, js["PID_直道"]);

    /* ── PID弯道 ── */
    if (js.contains("PID_弯道")) parse_pid(cfg.pid_curve, js["PID_弯道"]);

    /* ── IPM ── */
    if (js.contains("IPM逆透视")) {
        auto& ji = js["IPM逆透视"];
        if (ji.contains("src") && ji["src"].size() == 4) {
            for (int k = 0; k < 4; k++) {
                cfg.ipm.src[k][0] = ji["src"][k][0].get<float>();
                cfg.ipm.src[k][1] = ji["src"][k][1].get<float>();
            }
        }
        if (ji.contains("dst") && ji["dst"].size() == 4) {
            for (int k = 0; k < 4; k++) {
                cfg.ipm.dst[k][0] = ji["dst"][k][0].get<float>();
                cfg.ipm.dst[k][1] = ji["dst"][k][1].get<float>();
            }
        }
    }

    /* ── 巡线 ── */
    if (js.contains("巡线参数")) {
        auto& jl = js["巡线参数"];
        cfg.line.row_scan_start      = js_get(jl, "row_scan_start",      cfg.line.row_scan_start);
        cfg.line.row_scan_end        = js_get(jl, "row_scan_end",        cfg.line.row_scan_end);
        cfg.line.road_width_min      = js_get(jl, "road_width_min",      cfg.line.road_width_min);
        cfg.line.search_radius       = js_get(jl, "search_radius",       cfg.line.search_radius);
        cfg.line.straight_var_thresh = js_get(jl, "straight_var_thresh", cfg.line.straight_var_thresh);
    }

    /* ── 场景检测 ── */
    if (js.contains("场景检测")) {
        auto& js2 = js["场景检测"];
        cfg.scene.inter_width_thresh   = js_get(js2, "inter_width_thresh",   cfg.scene.inter_width_thresh);
        cfg.scene.inter_confirm_frames = js_get(js2, "inter_confirm_frames", cfg.scene.inter_confirm_frames);
        cfg.scene.inter_pass_frames    = js_get(js2, "inter_pass_frames",    cfg.scene.inter_pass_frames);
        cfg.scene.obstacle_red_ratio   = js_get(js2, "obstacle_red_ratio",   cfg.scene.obstacle_red_ratio);
        cfg.scene.zebra_stripe_min     = js_get(js2, "zebra_stripe_min",     cfg.scene.zebra_stripe_min);
    }

    /* ── 调试 ── */
    if (js.contains("调试开关")) {
        auto& jd = js["调试开关"];
        cfg.debug.debug_stream     = js_get(jd, "debug_stream",     cfg.debug.debug_stream);
        cfg.debug.debug_print_fps  = js_get(jd, "debug_print_fps",  cfg.debug.debug_print_fps);
        cfg.debug.transmission_port = js_get(jd, "transmission_port", cfg.debug.transmission_port);
    }

    std::cout << "[param] 配置加载成功: " << json_path << "\n";
    std::cout << "[param] base_speed=" << cfg.speed.base_speed
              << " kp_straight=" << cfg.pid_straight.kp
              << " kp_curve=" << cfg.pid_curve.kp << "\n";
    return true;
}