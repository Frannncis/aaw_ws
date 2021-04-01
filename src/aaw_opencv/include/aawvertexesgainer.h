#ifndef AAWVERTEXESGAINER_H
#define AAWVERTEXESGAINER_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <iostream>

/**
 * @brief The AawVertexesGainer class
 * @note 输入的图像应该是经过畸变校正、降噪的灰度图。
 */
class AAWVertexesGainer
{
public:

    AAWVertexesGainer();
    AAWVertexesGainer(cv::Mat & image);
    ~AAWVertexesGainer();
    std::vector<cv::Point2f> get4Vertexes() const;

private:

    //存储精确拟合的边界线参数
    cv::Vec4f leftBoundary_;
    cv::Vec4f rightBoundary_;
    cv::Vec4f upBoundary_;
    cv::Vec4f downBoundary_;

    static const int windowSize4Fitting_LeftBoundary_;
    static const int windowSize4Fitting_RightBoundary_;
    static const int windowSize4Fitting_UpBoundary_;
    static const int windowSize4Fitting_DownBoundary_;
    static const float antiReflectThreshold_Left_;
    static const float antiReflectThreshold_Right_;
    static const float antiReflectThreshold_Up_;
    static const float antiReflectThreshold_Down_;
    static const int exceptionStartPos_Down_;

    /**
     * @brief vertexes_ 从图像左上角开始，逆时针旋转一圈为顶点编号。
     * 0  3
     * 1  2
     */
    std::vector<cv::Point2f> vertexes_;

    int getApproachingLine(cv::Mat & image,
                           bool upLine,
                           int startColROI,
                           int endColROI,
                           bool posExcept,
                           int exceptionStartPos);
    cv::Vec4f myLineFitting(cv::Mat & image,
                            int originalLineCol,
                            bool leftLine,
                            int startRow,
                            int endRow,
                            int windowSize,
                            bool blockLeft,
                            float antiReflectThreshold);
    std::vector<cv::Point2f> calcPointsThroughLine(cv::Vec4f inputLine,
                                                   unsigned int yCount);
    cv::Vec4f getTransposedLine(cv::Vec4f inputLine);
    cv::Point2f calcIntersectionPointOf2Lines(cv::Vec4f line1,
                                              cv::Vec4f line2);
    void showInformatIon(cv::Mat & image);
};

#endif // AAWVERTEXESGAINER_H
