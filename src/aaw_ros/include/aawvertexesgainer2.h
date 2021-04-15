#ifndef AAWVERTEXESGAINER2_H
#define AAWVERTEXESGAINER2_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

/**
 * @brief The AawVertexesGainer class
 * @note 输入的图像应该是经过畸变校正、降噪的灰度图。
 */
class AAWVertexesGainer2
{
public:

    AAWVertexesGainer2();
    AAWVertexesGainer2(cv::Mat & image);
    ~AAWVertexesGainer2();
    std::vector<cv::Point2f> get4Vertexes() const;

private:

    //存储精确拟合的边界线参数
    cv::Vec4f leftBoundary_;
    cv::Vec4f rightBoundary_;
    cv::Vec4f upBoundary_;
    cv::Vec4f downBoundary_;

    static const int grayScaleThresh4EstimateMainBody_;

    static const float antiMessThreshold_Left_;
    static const float antiMessThreshold_Right_;
    static const float antiMessThreshold_Up_;
    static const float antiMessThreshold_Down_;

    static const int upBoxWidth_;
    static const int leftBoxWidth_;
    static const int rightBoxWidth_;
    static const int downBoxWidth_;

    static const float upTakesInRatio_;
    static const float leftTakesInRatio_;
    static const float rightTakesInRatio_;
    static const float downTakesInRatio_;

    static const int sparceRatio_;  //对图片进行稀疏处理的倍率

    int mainBodyApproxiRows_;    //主体在视野中大约占了多少行像素
    int mainBodyApproxiCols_;    //主体在视野中大约占了多少列像素

    /**
     * @brief vertexes_ 从图像左上角开始，逆时针旋转一圈为顶点编号。
     * 0  3
     * 1  2
     */
    std::vector<cv::Point2f> vertexes_;

    void estimateMainBodySize(cv::Mat & inputImage,
                              unsigned char grayScaleThreshold);
    Eigen::MatrixXi calcColumnSumMatrix(cv::Mat & inputImage);
    std::vector<cv::Point2i> findMainBodyPosition(cv::Mat & inputImage);
    std::vector<cv::Vec4i> getImageROI(cv::Mat & inputImage,
                                       std::vector<cv::Point2i> corners);
    cv::Vec4f myLineFitting2(cv::Mat &image,
                             int startRow,
                             int endRow,
                             int colRangeLeft,
                             int colRangeRight,
                             bool blockRight,
                             float antiMessThreshold);
    std::vector<cv::Point2f> calcPointsThroughLine(cv::Vec4f inputLine,
                                                   unsigned int yCount);
    cv::Vec4f getTransposedLine(cv::Vec4f inputLine);
    cv::Point2f calcIntersectionPointOf2Lines(cv::Vec4f line1,
                                              cv::Vec4f line2);
    void showInformatIon(cv::Mat & image);

//    void drawBoxes(cv::Mat & inputImage, std::vector<cv::Point2i> corners);
};

#endif // AAWVERTEXESGAINER2_H
