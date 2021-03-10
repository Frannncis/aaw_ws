#include "aawvertexesgainer.h"

//静态成员变量，所有类对象共享

const int AawVertexesGainer::windowSize4Fitting_LeftBoundary_ = 80;
const int AawVertexesGainer::windowSize4Fitting_RightBoundary_ = 80;
const int AawVertexesGainer::windowSize4Fitting_UpBoundary_ = 100;
const int AawVertexesGainer::windowSize4Fitting_DownBoundary_ = 100;
const float AawVertexesGainer::antiReflectThreshold_Left_ = 8.0;
const float AawVertexesGainer::antiReflectThreshold_Right_ = 8.0;
const float AawVertexesGainer::antiReflectThreshold_Up_ = 5.0;
const float AawVertexesGainer::antiReflectThreshold_Down_ = 5.0;
const int AawVertexesGainer::exceptionStartPos_Down_ = 1100;


//公有成员函数

AawVertexesGainer::AawVertexesGainer()
{

}

AawVertexesGainer::AawVertexesGainer(cv::Mat & image)
{
    int iRows;
    int cRows;
    int approachingLine_Left;
    int approachingLine_Right;
    int approachingLine_Up;
    int approachingLine_Down;

    //限定ROI窗口长度方向的端点坐标，粗略和精确拟合直线都要用到。
    int ROIBoundaries4Up[2];
    int ROIBoundaries4Down[2];
    int ROIBoundaries4Left[2];
    int ROIBoundaries4Right[2];

    iRows = image.rows;
    cRows = iRows/2;

    ROIBoundaries4Left[0] = cRows/2;
    ROIBoundaries4Left[1] = (cRows + iRows)/2;
    ROIBoundaries4Right[0] = cRows/2;
    ROIBoundaries4Right[1] = (cRows + iRows)/2;

    //先找两条竖直边界线，干扰较小
    cv::Mat imageTransposed = image.t();
    approachingLine_Left = getApproachingLine(imageTransposed, 1, ROIBoundaries4Left[0], ROIBoundaries4Left[1], 0, 0);
    approachingLine_Right = getApproachingLine(imageTransposed, 0, ROIBoundaries4Right[0], ROIBoundaries4Right[1], 0, 0);

    ROIBoundaries4Up[0] = (3*approachingLine_Left + approachingLine_Right)/4;
    ROIBoundaries4Up[1] = (approachingLine_Left + 3*approachingLine_Right)/4;
    ROIBoundaries4Down[0] = approachingLine_Left;
    ROIBoundaries4Down[1] = approachingLine_Right;

    approachingLine_Up = getApproachingLine(image, 1, ROIBoundaries4Up[0], ROIBoundaries4Up[1], 0, 0);
    approachingLine_Down = getApproachingLine(image, 0, ROIBoundaries4Down[0], ROIBoundaries4Down[1], 1, exceptionStartPos_Down_);

    leftBoundary_ = myLineFitting(image, approachingLine_Left, 1, ROIBoundaries4Left[0], ROIBoundaries4Left[1], windowSize4Fitting_LeftBoundary_, 0, antiReflectThreshold_Left_);
    rightBoundary_ = myLineFitting(image, approachingLine_Right, 0, ROIBoundaries4Right[0], ROIBoundaries4Right[1], windowSize4Fitting_RightBoundary_, 1, antiReflectThreshold_Right_);
    upBoundary_ = getTransposedLine(myLineFitting(imageTransposed, approachingLine_Up, 1, ROIBoundaries4Up[0], ROIBoundaries4Up[1], windowSize4Fitting_UpBoundary_, 0, antiReflectThreshold_Up_));
    downBoundary_ = getTransposedLine(myLineFitting(imageTransposed, approachingLine_Down, 0, ROIBoundaries4Down[0], ROIBoundaries4Down[1], windowSize4Fitting_DownBoundary_, 1, antiReflectThreshold_Down_));

    vertexes_.push_back(calcIntersectionPointOf2Lines(leftBoundary_, upBoundary_));
    vertexes_.push_back(calcIntersectionPointOf2Lines(leftBoundary_, downBoundary_));
    vertexes_.push_back(calcIntersectionPointOf2Lines(rightBoundary_, downBoundary_));
    vertexes_.push_back(calcIntersectionPointOf2Lines(rightBoundary_, upBoundary_));

    showInformatIon(image);
}

AawVertexesGainer::~AawVertexesGainer()
{

}

std::vector<cv::Point2f> AawVertexesGainer::get4Vertexes() const
{
    return vertexes_;
}



//私有成员函数

/**
 * @brief AawVertexesGainer::getApproachingLine 获取粗略的边界线，每次只找一条，平行于图像边界，且在输入图像内为横线。
 * @param image 输入的图像，若要找原始图像中的竖直边界，则转置之后输入。
 * @param upLine 指定本次要找的线在输入图像的上方还是下方，true则为找上方的线，用于划分ROI的边界。
 * @param startColROI 限定ROI的左边界线。
 * @param endColROI 限定ROI的右边界线。
 * @param posExcept 是否需要在划定ROI时排除某些区域，true为是。
 * @param exceptionStartPos 该行号以外的区域都不包含在ROI内。
 * @return 返回找到的直线在输入图像中的行号。
 */
int AawVertexesGainer::getApproachingLine(cv::Mat &image, bool upLine, int startColROI, int endColROI, bool posExcept, int exceptionStartPos)
{
    int iRows = image.rows;
    int cRows = iRows/2;
    int ROIUpLimit, ROIDownLimit;
    if (posExcept) {
        ROIUpLimit = ROIDownLimit = exceptionStartPos;
    }
    else {
        ROIUpLimit = 0;
        ROIDownLimit = iRows;
    }

    //创建图像ROI，之后直接用Mat::Row(int)提取感兴趣的行即可，均为O(1)复杂度(最小)
    cv::Mat imageROI = image.colRange(startColROI, endColROI+1).clone();
    cv::Mat rowsDiffer;
    unsigned long long sumPixelDiffer = 0, sumMax = 0;

    int roughLine = cRows;

    if (upLine) {
        for (int i=cRows-1; i>ROIUpLimit; --i) {
            cv::absdiff(imageROI.row(i), imageROI.row(i-1), rowsDiffer);
            sumPixelDiffer = cv::sum(rowsDiffer).val[0];
            if (sumMax < sumPixelDiffer) {
                sumMax = sumPixelDiffer;
                roughLine = i; //行、列的下标都是从0开始的
            }
        }
    }
    else {
        for (int i=cRows; i<ROIDownLimit; ++i) {
            cv::absdiff(imageROI.row(i-1), imageROI.row(i), rowsDiffer);
            sumPixelDiffer = cv::sum(rowsDiffer).val[0];
            if (sumMax < sumPixelDiffer) {
                sumMax = sumPixelDiffer;
                roughLine = i;
            }
        }
    }

    return  roughLine;
}

/**
 * @brief AawVertexesGainer::myLineFitting 拟合精确的边界线，每次只找一条，且在输入图像内大致为竖直方向的线。
 * @note 需要将4条粗略的边界都提取之后再进行拟合，否则ROI的上下边界不准确，影响直线拟合的精度。
 * @param image 输入的图像，若需拟合水平方向的边界线，将图像转置之后输入。
 * @param originalLineCol 由getApproachingLine()获取的粗略边界线的坐标，在输入图像内为列号。
 * @param leftLine 需要拟合的直线是否为图像左侧的线，用于辅助确定ROI的边界。
 * @param startRow ROI的上边界。
 * @param endRow ROI的下边界。
 * @param windowSize ROI的宽度值。
 * @param blockLeft 指明物体是否在拟合直线的左侧，用于消除物体表面反光的影响。物体完全在相机视野内和不完全在视野内时，与leftLine值有差异，需单独设置。
 * @param antiReflectThreshold 为消除反光设置的阈值，即认为偏离几个像素时提取的点是受反光影响的误差点。
 * @return 返回在输入图像坐标系下定义的cv::Vec4f类型直线参数。
 */
cv::Vec4f AawVertexesGainer::myLineFitting(cv::Mat &image, int originalLineCol, bool leftLine, int startRow, int endRow, int windowSize, bool blockLeft, float antiReflectThreshold)
{
    cv::Vec4f fittedLineInitial, fittedLineUltimate;
    std::vector<cv::Point2f> selectedPointsInitial, selectedPointsUltimate, initFittedLinePoints;
    int colRangeLeft, colRangeRight, compareWindowSize;

    //The line should be vertical
    cv::Mat imageROIOriginal = image.rowRange(startRow, endRow + 1).clone();

    //确保选择ROI时不会因窗口过大而索引到图像以外的区域
    if (leftLine) {
        if (originalLineCol-windowSize/2 < 0)
            colRangeLeft = 0;
        else
            colRangeLeft = originalLineCol-windowSize/2;
        colRangeRight = originalLineCol+windowSize-windowSize/2 + 1;
    }
    else {
        if ((originalLineCol+windowSize-windowSize/2 + 1) > image.cols)
            colRangeRight = image.cols;
        else
            colRangeRight = originalLineCol+windowSize-windowSize/2 + 1;
        colRangeLeft = originalLineCol-windowSize/2;
    }
    compareWindowSize = colRangeRight - colRangeLeft - 1;

    cv::Mat imageROI = imageROIOriginal.colRange(colRangeLeft, colRangeRight);
    int pixelDiffer = 0, pixelDifferMax = 0;
    uchar * pixel;
    cv::Point2f MaxGrayGradientPoint;

    for (int row = 0; row <= endRow - startRow; ++row) {
        MaxGrayGradientPoint = cv::Point2f(1, row);
        pixel = imageROI.ptr<uchar>(row);
        for (int compareCount = 0; compareCount < compareWindowSize; ++compareCount) {
            pixelDiffer = abs(pixel[compareCount] - pixel[compareCount+1]);
            if (pixelDifferMax < pixelDiffer) {
                pixelDifferMax = pixelDiffer;
                MaxGrayGradientPoint = cv::Point2f(compareCount + 1, row);
            }
        }
        selectedPointsInitial.push_back(MaxGrayGradientPoint);
        pixelDifferMax = 0;
    }

    //直线拟合获得的cv::Vec4f类型直线参数，前两位为正则化的方向向量，后两位为直线上一点的坐标。
    cv::fitLine(selectedPointsInitial, fittedLineInitial, cv::DIST_HUBER, 0, 0.01, 0.01);
    initFittedLinePoints = calcPointsThroughLine(fittedLineInitial, selectedPointsInitial.size());
    if (blockLeft) {
        for (int row = 0; row <= endRow - startRow; ++row) {
            if (!((initFittedLinePoints[row].x - selectedPointsInitial[row].x) > antiReflectThreshold)) {
                selectedPointsUltimate.push_back(selectedPointsInitial[row]);
            }
        }
    }
    else {
        for (int row = 0; row <= endRow - startRow; ++row) {
            if (!((selectedPointsInitial[row].x - initFittedLinePoints[row].x) > antiReflectThreshold)) {
                selectedPointsUltimate.push_back(selectedPointsInitial[row]);
            }
        }
    }
    cv::fitLine(selectedPointsUltimate, fittedLineUltimate, cv::DIST_HUBER, 0, 0.01, 0.01);

    //补偿因选定ROI造成的坐标偏移量
    fittedLineUltimate[2] += (colRangeLeft);
    fittedLineUltimate[3] += startRow;

    return fittedLineUltimate;
}

/**
 * @brief calcPointsThroughLine 计算直线上的点的坐标。
 * @param inputLine 输入类型为cv::Vec4f的直线参数。
 * @param yCount 总共需要计算的点数量，纵坐标从0开始，步长1。
 * @return 返回std::vector<cv::Point2f>的直线上的点的集合。
 */
std::vector<cv::Point2f> AawVertexesGainer::calcPointsThroughLine(cv::Vec4f inputLine, unsigned int yCount)
{
    std::vector<cv::Point2f> pointsThroughLine;
    float k = inputLine[1]/inputLine[0];
    float b = inputLine[3] - k*inputLine[2];
    float x = 0;

    for (unsigned int y = 0; y < yCount; ++y) {
        x = (float(y) - b) / k;
        pointsThroughLine.push_back(cv::Point2f(x, float(y)));
    }

    return pointsThroughLine;
}

/**
 * @brief getTransposedLine 对图像转置后拟合出的直线参数进行转置变换，获得在原始图像中正确的直线参数。
 * @param inputLine cv::Vec4f的直线参数。
 * @return 返回经转置后的cv::Vec4f的直线参数。
 */
cv::Vec4f AawVertexesGainer::getTransposedLine(cv::Vec4f inputLine)
{
    cv::Vec4f lineTransposed;

    lineTransposed[0] = inputLine[1];
    lineTransposed[1] = inputLine[0];
    lineTransposed[2] = inputLine[3];
    lineTransposed[3] = inputLine[2];

    return lineTransposed;
}

/**
 * @brief AawVertexesGainer::calcIntersectionPointOf2Lines 计算两条直线的交点坐标。
 * @param line1 输入类型为cv::Vec4f的直线1参数。
 * @param line2 输入类型为cv::Vec4f的直线2参数。
 * @return 返回cv::Point2f类型的交点坐标。
 */
cv::Point2f AawVertexesGainer::calcIntersectionPointOf2Lines(cv::Vec4f line1, cv::Vec4f line2)
{
    float k1, k2, b1, b2, xIntersection, yIntersection;

    k1 = line1[1]/line1[0];
    k2 = line2[1]/line2[0];
    b1 = line1[3] - line1[1]*line1[2]/line1[0];
    b2 = line2[3] - line2[1]*line2[2]/line2[0];

    xIntersection = (b2 - b1)/(k1 - k2);
    yIntersection = k1*xIntersection + b1;

    cv::Point2f intersectionPoint(xIntersection, yIntersection);

    return intersectionPoint;
}

/**
 * @brief AawVertexesGainer::showInformatIon 将获取的边界、顶点和坐标实时显示在图片上。
 * @param image
 */
void AawVertexesGainer::showInformatIon(cv::Mat &image)
{
    std::string textCoordinate;
    textCoordinate += "P0 : (";
    textCoordinate += std::to_string(vertexes_[0].x);
    textCoordinate += ", ";
    textCoordinate += std::to_string(vertexes_[0].y);
    textCoordinate += ")";
    cv::putText(image, textCoordinate, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar::all(255), 2);
    textCoordinate.clear();
    textCoordinate += "P1 : (";
    textCoordinate += std::to_string(vertexes_[1].x);
    textCoordinate += ", ";
    textCoordinate += std::to_string(vertexes_[1].y);
    textCoordinate += ")";
    cv::putText(image, textCoordinate, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar::all(255), 2);
    textCoordinate.clear();
    textCoordinate += "P2 : (";
    textCoordinate += std::to_string(vertexes_[2].x);
    textCoordinate += ", ";
    textCoordinate += std::to_string(vertexes_[2].y);
    textCoordinate += ")";
    cv::putText(image, textCoordinate, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar::all(255), 2);
    textCoordinate.clear();
    textCoordinate += "P3 : (";
    textCoordinate += std::to_string(vertexes_[3].x);
    textCoordinate += ", ";
    textCoordinate += std::to_string(vertexes_[3].y);
    textCoordinate += ")";
    cv::putText(image, textCoordinate, cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar::all(255), 2);

    cv::line(image, vertexes_[0], vertexes_[1], 255, 3);
    cv::line(image, vertexes_[1], vertexes_[2], 255, 3);
    cv::line(image, vertexes_[2], vertexes_[3], 255, 3);
    cv::line(image, vertexes_[3], vertexes_[0], 255, 3);

    cv::circle(image, vertexes_[0], 5, 0, 3);
    cv::circle(image, vertexes_[1], 5, 0, 3);
    cv::circle(image, vertexes_[2], 5, 0, 3);
    cv::circle(image, vertexes_[3], 5, 0, 3);
}
