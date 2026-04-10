#pragma once
/*******************************************************************************
 * image_proc.h — 图像预处理模块
 *
 * 改动：
 *   - ipm_init() 改为从 IPMParam 读取坐标（不再硬编码）
 *   - preprocess() 新增 seed_point 参数，支持Floodfill去背景
 ******************************************************************************/

#include <opencv2/opencv.hpp>
#include "param/param.hpp"

/* ───────────────────── IPM 配置 ───────────────────── */

struct IPMConfig {
    cv::Point2f src[4];
    cv::Point2f dst[4];
};

/**
 * @brief 从 CarConfig 初始化 IPM 配置
 * @param cfg   JSON读取的全局配置
 * @param out   输出 IPMConfig
 */
void ipm_init(IPMConfig& out, const CarConfig& cfg);

/**
 * @brief 计算透视变换矩阵
 */
cv::Mat get_ipm_matrix(const IPMConfig& cfg);

/* ───────────────────── 图像预处理 ───────────────────── */

/**
 * @brief BGR图像 → 二值化（灰度 + 高斯 + Otsu + 形态学）
 *        新增：Floodfill 去除背景噪点
 *
 * @param src_bgr     输入彩色图
 * @param dst_bin     输出二值图（255=路面白色，0=黑色边界/背景）
 * @param seed_hint_x 洪水填充种子点的x初始猜测（通常用上一帧中心）
 *                    传入 -1 则自动从图像底部中心搜索
 */
void preprocess(const cv::Mat& src_bgr, cv::Mat& dst_bin, int seed_hint_x = -1);

/* ───────────────────── 逆透视变换 ───────────────────── */

/**
 * @brief 将二值图做 IPM 鸟瞰变换
 */
void ipm_warp(const cv::Mat& src, cv::Mat& dst, const cv::Mat& M);

/* ───────────────────── 调试可视化 ───────────────────── */

/**
 * @brief 在原图上绘制 IPM 取样区域（绿色四边形）
 */
void draw_ipm_region(cv::Mat& img, const IPMConfig& cfg);