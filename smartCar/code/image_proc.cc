/*******************************************************************************
 * image_proc.cpp — 图像预处理模块实现
 *
 * 改动说明：
 *   1. ipm_init() 改为从 CarConfig 读坐标，不再硬编码
 *   2. preprocess() 新增 Floodfill 去背景噪点
 *      原理：Otsu 二值化后，从赛道底部找一个白色种子点，
 *            floodFill 把赛道区域标记出来，扣掉所有背景白噪声
 ******************************************************************************/

#include "image_proc.h"
#include <iostream>

/* ═══════════════════════════ IPM 初始化 ═══════════════════════════════ */

void ipm_init(IPMConfig& out, const CarConfig& cfg)
{
    for (int i = 0; i < 4; i++) {
        out.src[i] = cv::Point2f(cfg.ipm.src[i][0], cfg.ipm.src[i][1]);
        out.dst[i] = cv::Point2f(cfg.ipm.dst[i][0], cfg.ipm.dst[i][1]);
    }
}

cv::Mat get_ipm_matrix(const IPMConfig& cfg)
{
    return cv::getPerspectiveTransform(cfg.src, cfg.dst);
}

/* ═══════════════════════════ 图像预处理 ═══════════════════════════════ */

void preprocess(const cv::Mat& src_bgr, cv::Mat& dst_bin, int seed_hint_x)
{
    cv::Mat gray;

    /* 步骤1：BGR → 灰度 */
    if (src_bgr.channels() == 3) {
        cv::cvtColor(src_bgr, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = src_bgr.clone();
    }

    /* 步骤2：高斯滤波去噪（轻度，保留边缘） */
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);

    /* 步骤3：Otsu 自适应全局阈值 */
    cv::threshold(gray, dst_bin, 0, 255,
                  cv::THRESH_BINARY | cv::THRESH_OTSU);

    /* 步骤4：形态学操作（填孔 + 去噪） */
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(dst_bin, dst_bin, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(dst_bin, dst_bin, cv::MORPH_OPEN,  kernel);

    /* ────────────────────────────────────────────────────────────────
     * 步骤5：Floodfill 去背景噪点（新增）
     *
     * 目的：走马观碑赛道是白色PVC，赛场地面可能也有白色反光、
     *       杂物等干扰，Otsu会把它们全部二值化为255。
     *       Floodfill从赛道底部种子点出发，把赛道连通区域标记为127，
     *       最后只保留127（赛道），其余全部清零。
     * ────────────────────────────────────────────────────────────────*/

    int W = dst_bin.cols;
    int H = dst_bin.rows;

    // ── 寻找种子点 ──
    // 优先从上一帧中心 seed_hint_x 附近搜，其次从图像中心搜
    cv::Point seed(-1, -1);
    int bottom_row = H - 1;

    // 以 seed_hint_x 为中心向两侧扩展搜索
    int cx_start = (seed_hint_x >= 0 && seed_hint_x < W) ? seed_hint_x : W / 2;

    for (int d = 0; d < W / 2 && seed.x < 0; d++) {
        int xl = cx_start - d;
        int xr = cx_start + d;
        if (xl >= 0 && dst_bin.at<uchar>(bottom_row, xl) == 255) {
            seed = cv::Point(xl, bottom_row);
        } else if (xr < W && dst_bin.at<uchar>(bottom_row, xr) == 255) {
            seed = cv::Point(xr, bottom_row);
        }
    }

    // 如果底部找不到种子（极端情况），向上逐行搜索
    if (seed.x < 0) {
        for (int y = H - 1; y >= H * 3 / 4 && seed.x < 0; y--) {
            for (int x = W / 4; x < W * 3 / 4; x++) {
                if (dst_bin.at<uchar>(y, x) == 255) {
                    seed = cv::Point(x, y);
                    break;
                }
            }
        }
    }

    // ── 执行 Floodfill ──
    if (seed.x >= 0) {
        // mask 需要比图像大 2 像素（OpenCV 要求）
        cv::Mat mask = cv::Mat::zeros(H + 2, W + 2, CV_8UC1);

        // 将与种子连通的白色区域标记为 127（newVal，避免与0/255混淆）
        cv::floodFill(dst_bin, mask, seed,
                      cv::Scalar(127),   // 填充颜色
                      nullptr,           // boundingRect（不需要）
                      cv::Scalar(0),     // loDiff
                      cv::Scalar(0),     // upDiff
                      4 | cv::FLOODFILL_MASK_ONLY); // 只写mask，不改原图

        // 重建二值图：mask中被填充的像素(值=1) → 255，其余 → 0
        // OpenCV FLOODFILL_MASK_ONLY 模式：填充的像素在 mask 中设为 newVal(=1的部分)
        // 注意：newVal 在 MASK_ONLY 模式下写的是 flags >> 8 的低8位，默认是1
        for (int y = 0; y < H; y++) {
            uchar* bin_row  = dst_bin.ptr<uchar>(y);
            uchar* mask_row = mask.ptr<uchar>(y + 1) + 1; // mask偏移(1,1)
            for (int x = 0; x < W; x++) {
                bin_row[x] = (mask_row[x] != 0) ? 255 : 0;
            }
        }
    }
    // 如果找不到种子，保留原始 Otsu 结果（降级处理，不崩溃）
}

/* ═══════════════════════════ 逆透视变换 ════════════════════════════════ */

void ipm_warp(const cv::Mat& src, cv::Mat& dst, const cv::Mat& M)
{
    cv::warpPerspective(src, dst, M,
                        cv::Size(src.cols, src.rows),
                        cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar(0));
}

/* ═══════════════════════════ 调试可视化 ════════════════════════════════ */

void draw_ipm_region(cv::Mat& img, const IPMConfig& cfg)
{
    std::vector<cv::Point> pts;
    for (int i = 0; i < 4; i++) {
        pts.push_back(cv::Point((int)cfg.src[i].x, (int)cfg.src[i].y));
    }
    std::vector<std::vector<cv::Point>> contours = {pts};
    cv::polylines(img, contours, true, cv::Scalar(0, 255, 0), 1);
    for (int i = 0; i < 4; i++) {
        cv::circle(img, cv::Point((int)cfg.src[i].x, (int)cfg.src[i].y),
                   3, cv::Scalar(0, 0, 255), -1);
    }
}