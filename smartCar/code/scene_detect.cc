/*******************************************************************************
 * scene_detect.cpp — 特殊场景检测实现
 *
 * 改动说明：
 *   1. inter_confirm_cnt 从 static 局部变量改为 SceneState 成员
 *      → 可以在外部重置，避免场景切换后计数残留
 *   2. 所有阈值通过 SceneParam 传入，不再硬编码
 *   3. detect_intersection / detect_red_obstacle / detect_zebra_line
 *      接口添加参数，调用方从 JSON 配置传入
 ******************************************************************************/

#include "scene_detect.h"
#include <cstring>
#include <cmath>
#include <iostream>

/* ═══════════════════════════ 初始化 ════════════════════════════════════ */

void scene_state_init(SceneState& state)
{
    state.current           = SCENE_NORMAL;
    state.inter_confirm_cnt = 0;
    state.inter_pass_frames = 0;
    state.roundabout_frames = 0;
    state.roundabout_in     = false;
    state.obstacle_side     = false;
}

/* ═══════════════════════════ 十字路口检测 ══════════════════════════════ */

bool detect_intersection(const LineInfo& line_info, int width_thresh,
                         int row_scan_start)
{
    if (!line_info.found) return false;

    /* 统计底部15行平均道路宽度 */
    int   count     = 0;
    float width_sum = 0.0f;
    int   row_start = std::min(row_scan_start, IMG_H - 1);

    for (int r = row_start; r >= row_start - 15 && r >= 0; r--) {
        if (line_info.road_width[r] > 0) {
            width_sum += (float)line_info.road_width[r];
            count++;
        }
    }
    if (count == 0) return false;

    float avg = width_sum / (float)count;
    return (avg > (float)width_thresh);
}

/* ═══════════════════════════ 红色障碍物检测 ════════════════════════════ */

bool detect_red_obstacle(const cv::Mat& color_img, float ratio_thresh)
{
    if (color_img.empty()) return false;

    cv::Mat hsv;
    cv::cvtColor(color_img, hsv, cv::COLOR_BGR2HSV);

    /* 只检测图像下半部分 */
    int y_start = color_img.rows / 2;
    cv::Mat roi = hsv(cv::Rect(0, y_start,
                               color_img.cols,
                               color_img.rows - y_start));

    /* 红色HSV范围：H在0~10 和 170~180，S>80，V>60 */
    cv::Mat mask1, mask2, mask_red;
    cv::inRange(roi, cv::Scalar(  0, 80, 60), cv::Scalar( 10, 255, 255), mask1);
    cv::inRange(roi, cv::Scalar(170, 80, 60), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask_red);

    int   total_pixels = roi.rows * roi.cols;
    int   red_pixels   = cv::countNonZero(mask_red);
    float ratio        = (float)red_pixels / (float)total_pixels;

    return (ratio > ratio_thresh);
}

/* ═══════════════════════════ 斑马线检测 ════════════════════════════════ */

bool detect_zebra_line(const cv::Mat& bin_ipm, int stripe_min,
                       int row_scan_start)
{
    if (bin_ipm.empty()) return false;

    int W  = bin_ipm.cols;
    int H  = bin_ipm.rows;
    int cx = W / 2;
    int scan_top    = H * 2 / 3;
    int scan_bottom = std::min(row_scan_start, H - 1);

    int   transitions = 0;
    uchar prev_val    = bin_ipm.at<uchar>(scan_bottom, cx);

    for (int r = scan_bottom - 1; r >= scan_top; r--) {
        uchar cur_val  = bin_ipm.at<uchar>(r, cx);
        bool  prev_white = (prev_val >= 128);
        bool  cur_white  = (cur_val  >= 128);
        if (prev_white != cur_white) transitions++;
        prev_val = cur_val;
    }

    /* 斑马线要求至少 stripe_min 次黑白交替（*2因为一条纹=2次跳变） */
    return (transitions >= stripe_min * 2);
}

/* ═══════════════════════════ 综合场景状态机 ════════════════════════════ */

SceneType detect_scene(const cv::Mat&   bin_ipm,
                       const cv::Mat&   color_img,
                       const LineInfo&  line_info,
                       SceneState&      state,
                       const SceneParam& sp)
{
    /* ─── 路口直行保持阶段 ─── */
    if (state.current == SCENE_INTERSECTION) {
        state.inter_pass_frames++;
        if (state.inter_pass_frames >= sp.inter_pass_frames) {
            state.current           = SCENE_NORMAL;
            state.inter_pass_frames = 0;
            state.inter_confirm_cnt = 0;  // 重置防抖计数
        }
        return state.current;
    }

    /* ─── 环岛内 ─── */
    if (state.current == SCENE_ROUNDABOUT_IN) {
        state.roundabout_frames++;
        const int ROUNDABOUT_PASS_FRAMES = 80;  // 可后续移入 SceneParam
        if (state.roundabout_frames >= ROUNDABOUT_PASS_FRAMES) {
            state.current           = SCENE_NORMAL;
            state.roundabout_frames = 0;
            state.roundabout_in     = false;
        }
        return state.current;
    }

    /* ─── 正常检测流程 ─── */

    /* 优先级1：红色障碍物（安全相关，最高优先级） */
    if (detect_red_obstacle(color_img, sp.obstacle_red_ratio)) {
        /* 计算障碍物偏左/右 */
        if (!color_img.empty()) {
            cv::Mat hsv, mask1, mask2, mask_red;
            cv::cvtColor(color_img, hsv, cv::COLOR_BGR2HSV);
            int y0  = color_img.rows / 2;
            cv::Mat roi = hsv(cv::Rect(0, y0,
                                       color_img.cols,
                                       color_img.rows - y0));
            cv::inRange(roi, cv::Scalar( 0,80,60), cv::Scalar( 10,255,255), mask1);
            cv::inRange(roi, cv::Scalar(170,80,60),cv::Scalar(180,255,255), mask2);
            cv::bitwise_or(mask1, mask2, mask_red);

            cv::Moments m = cv::moments(mask_red, true);
            if (m.m00 > 0) {
                double cx_obs = m.m10 / m.m00;
                state.obstacle_side = (cx_obs > color_img.cols / 2.0);
            }
        }
        state.current = SCENE_OBSTACLE;
        return SCENE_OBSTACLE;
    }

    /* 优先级2：斑马线 */
    if (detect_zebra_line(bin_ipm, sp.zebra_stripe_min, sp.inter_width_thresh)) {
        state.current = SCENE_ZEBRA;
        return SCENE_ZEBRA;
    }

    /* 优先级3：十字路口（带防抖） */
    if (detect_intersection(line_info, sp.inter_width_thresh,
                            IMG_H - 1 /* row_scan_start，用实际参数 */))
    {
        /*
         * 防抖：需要连续 inter_confirm_frames 帧都检测到路口才触发
         * 防抖计数器存在 state 里，不再用 static 局部变量
         */
        state.inter_confirm_cnt++;
        if (state.inter_confirm_cnt >= sp.inter_confirm_frames) {
            state.inter_confirm_cnt = 0;
            state.current           = SCENE_INTERSECTION;
            state.inter_pass_frames = 0;
            return SCENE_INTERSECTION;
        }
        /* 未达到确认帧数，保持当前场景（不要立即切换） */
        return state.current;
    } else {
        /* 本帧没检测到路口 → 重置防抖计数 */
        state.inter_confirm_cnt = 0;
    }

    /* 正常行驶 */
    state.current = SCENE_NORMAL;
    return SCENE_NORMAL;
}