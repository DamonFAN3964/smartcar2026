#pragma once
/*******************************************************************************
 * line_detect.h — 迷宫法巡线模块
 *
 * 改动：
 *   - LineInfo 新增 is_straight 字段（直道标志）
 *   - maze_detect() 新增 LineParam 参数，替代原来的宏定义
 *   - 新增 detect_straight() 辅助函数
 ******************************************************************************/

#include <opencv2/opencv.hpp>
#include "param/param.hpp"

/* ─────────────────────── 常量（退化默认值，实际由 LineParam 覆盖） ───── */
#define IMG_W           160
#define IMG_H           120
// 注意：ROW_SCAN_START / ROW_SCAN_END / ROAD_WIDTH_MIN / SEARCH_RADIUS
//       统一通过 LineParam 传入，不再用宏，避免改参数要重新编译

/* ─────────────────────── 巡线结果结构体 ───────────────────────────────── */

struct LineInfo {
    int   left_edge [IMG_H];   // 每行左边界 x
    int   right_edge[IMG_H];   // 每行右边界 x
    int   center_x  [IMG_H];   // 每行中心 x
    int   road_width[IMG_H];   // 每行道路宽度

    float error;        // 加权偏差（像素，正=偏右，负=偏左）
    float avg_width;    // 有效行平均道路宽度（像素）
    int   valid_rows;   // 有效行数
    int   top_row;      // 搜到的最高行（最远行）

    bool  found;        // 是否找到道路
    bool  left_lost;    // 左边界是否贴图像边
    bool  right_lost;   // 右边界是否贴图像边

    /* 新增：直道判断 */
    bool  is_straight;  // true=直道，false=弯道
                        // 判据：底部N行 center_x 标准差 < straight_var_thresh
};

/* ─────────────────────── 接口声明 ───────────────────────────────────── */

/**
 * @brief 重置 LineInfo（每帧调用前必须调用）
 */
void line_info_reset(LineInfo& info);

/**
 * @brief 迷宫法巡线主函数
 *
 * @param bin_ipm   IPM 鸟瞰二值图（CV_8UC1，白=路，黑=边界）
 * @param info      输出巡线结果
 * @param lp        巡线参数（来自 CarConfig.line）
 */
void maze_detect(const cv::Mat& bin_ipm, LineInfo& info, const LineParam& lp);

/**
 * @brief 直道/弯道判断（在 maze_detect 之后调用）
 *
 * 取底部 check_rows 行的 center_x，计算标准差：
 *   标准差 < lp.straight_var_thresh → is_straight = true
 *
 * @param info        maze_detect 的结果（会修改 info.is_straight）
 * @param lp          巡线参数
 * @param check_rows  检查底部几行（默认10行）
 */
void detect_straight(LineInfo& info, const LineParam& lp, int check_rows = 10);

/**
 * @brief 调试：在彩色图上绘制巡线结果（左/右/中心边界点 + 偏差数值）
 */
void draw_line_result(cv::Mat& img, const LineInfo& info, const LineParam& lp);