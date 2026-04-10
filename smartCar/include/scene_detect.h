#pragma once
/*******************************************************************************
 * scene_detect.h — 特殊场景检测模块
 *
 * 改动：
 *   - 防抖计数器从函数内 static 变量改为 SceneState 成员变量
 *     （static局部变量在多次调用间共享，改为成员更清晰、可重置）
 *   - detect_scene() 接收 SceneParam，所有阈值由 JSON 配置
 ******************************************************************************/

#include <opencv2/opencv.hpp>
#include "line_detect.h"
#include "param/param.hpp"

/* ─────────────────────── 场景类型枚举 ─────────────────────────────── */

enum SceneType {
    SCENE_NORMAL = 0,       // 正常巡线
    SCENE_INTERSECTION,     // 十字路口
    SCENE_ROUNDABOUT_IN,    // 环岛
    SCENE_OBSTACLE,         // 红色路障
    SCENE_SLOPE,            // 坡道（预留）
    SCENE_ZEBRA             // 斑马线
};

/* ─────────────────────── 场景状态机 ─────────────────────────────────── */

struct SceneState {
    SceneType current;

    /* 十字路口 */
    int inter_confirm_cnt;  // 连续检测到路口的帧数（防抖用）
    int inter_pass_frames;  // 进入路口后已过帧数

    /* 环岛 */
    int roundabout_frames;
    bool roundabout_in;

    /* 障碍物 */
    bool obstacle_side;     // false=障碍在左，true=障碍在右

    SceneState()
        : current(SCENE_NORMAL)
        , inter_confirm_cnt(0)
        , inter_pass_frames(0)
        , roundabout_frames(0)
        , roundabout_in(false)
        , obstacle_side(false)
    {}
};

/* ─────────────────────── 接口声明 ──────────────────────────────────── */

/**
 * @brief 初始化场景状态机
 */
void scene_state_init(SceneState& state);

/**
 * @brief 综合场景检测（每帧调用一次）
 *
 * @param bin_ipm     IPM鸟瞰二值图
 * @param color_img   原始彩色图（用于红色障碍物检测）
 * @param line_info   巡线结果
 * @param state       场景状态机（会被修改）
 * @param sp          场景参数（来自 CarConfig.scene）
 * @return            当前场景类型
 */
SceneType detect_scene(const cv::Mat&  bin_ipm,
                       const cv::Mat&  color_img,
                       const LineInfo& line_info,
                       SceneState&     state,
                       const SceneParam& sp);

/* ─────────────────────── 内部检测函数（可单独调用调试） ────────────── */

/**
 * @brief 十字路口检测
 * 原理：底部若干行平均道路宽度超过阈值
 */
bool detect_intersection(const LineInfo& line_info, int width_thresh,
                         int row_scan_start);

/**
 * @brief 红色障碍物检测
 * 原理：HSV提取红色区域，计算在图像下半部分的占比
 */
bool detect_red_obstacle(const cv::Mat& color_img, float ratio_thresh);

/**
 * @brief 斑马线检测
 * 原理：沿中心列垂直扫描，统计黑白交替次数
 */
bool detect_zebra_line(const cv::Mat& bin_ipm, int stripe_min,
                       int row_scan_start);