#include "aawvertexesgainer2.h"

//静态成员变量，所有类对象共享

//估计主体大小用的灰度值阈值，非常重要！！！环境变化之后需要更改！！！
const int AAWVertexesGainer2::grayScaleThresh4EstimateMainBody_ = 50;

//由AAWVertexesGainer类保留的参数，用来消除主体之外的灰度梯度较大位置的干扰
const float AAWVertexesGainer2::antiMessThreshold_Left_ = 8.0;
const float AAWVertexesGainer2::antiMessThreshold_Right_ = 8.0;
const float AAWVertexesGainer2::antiMessThreshold_Up_ = 5.0;
const float AAWVertexesGainer2::antiMessThreshold_Down_ = 5.0;

//imgROI的窗口宽度（垂直于边界方向）
const int AAWVertexesGainer2::upBoxWidth_ = 300;
const int AAWVertexesGainer2::leftBoxWidth_ = 600;
const int AAWVertexesGainer2::rightBoxWidth_ = 600;
const int AAWVertexesGainer2::downBoxWidth_ = 300;

//imgROI的窗口长度占主体边界尺寸的比例
const float AAWVertexesGainer2::upTakesInRatio_ = 0.5;
const float AAWVertexesGainer2::leftTakesInRatio_ = 0.75;
const float AAWVertexesGainer2::rightTakesInRatio_ = 0.75;
const float AAWVertexesGainer2::downTakesInRatio_ = 0.5;

//将图像稀疏处理的倍数，用来提高找主体的速度。越大越快，越小越精确。
const int AAWVertexesGainer2::sparceRatio_ = 10;

//公有成员函数

AAWVertexesGainer2::AAWVertexesGainer2()
{

}

AAWVertexesGainer2::AAWVertexesGainer2(cv::Mat & image)
{
    cv::Mat imageTransposed = image.t();

    std::vector<cv::Point2i> corners = findMainBodyPosition(image);
    std::vector<cv::Vec4i> imgROI = getImageROI(image, corners);

    upBoundary_ = getTransposedLine(myLineFitting2(imageTransposed, imgROI[0][0], imgROI[0][1], imgROI[0][2], imgROI[0][3], true, antiMessThreshold_Up_));
    downBoundary_ = getTransposedLine(myLineFitting2(imageTransposed, imgROI[1][0], imgROI[1][1], imgROI[1][2], imgROI[1][3], false, antiMessThreshold_Down_));
    leftBoundary_ = myLineFitting2(image, imgROI[2][0], imgROI[2][1], imgROI[2][2], imgROI[2][3], true, antiMessThreshold_Left_);
    rightBoundary_ = myLineFitting2(image, imgROI[3][0], imgROI[3][1], imgROI[3][2], imgROI[3][3], false, antiMessThreshold_Right_);

    vertexes_.push_back(calcIntersectionPointOf2Lines(leftBoundary_, upBoundary_));
    vertexes_.push_back(calcIntersectionPointOf2Lines(leftBoundary_, downBoundary_));
    vertexes_.push_back(calcIntersectionPointOf2Lines(rightBoundary_, downBoundary_));
    vertexes_.push_back(calcIntersectionPointOf2Lines(rightBoundary_, upBoundary_));

    showInformatIon(image);
}

AAWVertexesGainer2::~AAWVertexesGainer2()
{

}

std::vector<cv::Point2f> AAWVertexesGainer2::get4Vertexes() const
{
    return vertexes_;
}



//私有成员函数

/**
 * @brief AAWVertexesGainer2::estimateMainBodySize 估计主体的大小，用来确定findMainBodyPosition()所需要的滑动窗口尺寸。
 * @param inputImage
 * @param grayScaleThreshold 主体的灰度值上上限，这个参数的选择很重要，小于这个阈值的像素点会被计数。
 */
void AAWVertexesGainer2::estimateMainBodySize(cv::Mat & inputImage, unsigned char grayScaleThreshold)
{
    int iRows = inputImage.rows;
    int iCols = inputImage.cols;
    int pixelsCount = 0;
    uchar * p;
    for (int row = 0; row < iRows; ++row) {
        p = inputImage.ptr<uchar>(row);
        for (int col = 0; col < iCols; ++col) {
            if (p[col] <= grayScaleThreshold)
                ++pixelsCount;
        }
    }
    mainBodyApproxiRows_ = (int)sqrt(pixelsCount / 2);
    mainBodyApproxiCols_ = 2 * mainBodyApproxiRows_;

}

/**
 * @brief AAWVertexesGainer2::calcColumnSumMatrix 为了提高寻找主体位置时滑动窗口内像素点灰度值求和效率而建立的中间矩阵，具体定义见手稿。
 * @param inputImage
 * @return 返回该中间矩阵colSumMat
 */
Eigen::MatrixXi AAWVertexesGainer2::calcColumnSumMatrix(cv::Mat & inputImage)
{
    using namespace std;
    int iRows = inputImage.rows;
    int iCols = inputImage.cols;
    int sr = sparceRatio_;
//    int winCols = mainBodyApproxiCols_;
    int winRows = mainBodyApproxiRows_;

    //稀疏像素阵窗口的行列数
    int sparceWinRows = winRows/sr; //55
//    int sparceWinCols = winCols/sr; //110

    //括号里的-1是因为像素是从0行0列开始算的，而iCols和iRows代表实际像素有几行几列
    //如1100列像素实际下标为0-1099，可以得稀疏点阵110列，而1101个像素则可以得稀疏点阵111列（每10个元素中是取左上侧的那一个）
    int csCols = (iCols-1)/sr + 1;
    int csRows = (iRows-1)/sr -  sparceWinRows + 2;

    //colSumMat的大小为csRows * csCols
    Eigen::MatrixXi colSumMat = Eigen::MatrixXi::Zero(csRows, csCols);

    //colSumMat只有第0行元素需要逐个累加，第1 ~ csRows-1行都只需要增量计算
    //计算colSumMat第0行元素
    uchar * p;
    for (int row = 0; row < sparceWinRows; ++row) {
        p = inputImage.ptr<uchar>(row * sr);
        for (int col = 0; col < csCols; ++col) {
            colSumMat(0, col) += p[col * sr];
        }
    }

    uchar * pu; //减去上方的一个像素
    uchar * pd; //加上下方的一个像素
    //计算colSumMat第1 ~ csRows-1行元素
    for (int row = 1; row < csRows; ++row) {
        pu = inputImage.ptr<uchar>((row-1) * sr);
        pd = inputImage.ptr<uchar>((row+sparceWinRows-1) * sr);
        for (int col = 0; col < csCols; ++col) {
            colSumMat(row, col) = colSumMat(row-1, col) - pu[col*sr] + pd[col*sr];
        }
    }

    return colSumMat;
}

/**
 * @brief findMainBodyPosition 用稀疏点阵像素灰度值求和的方法找到主体所在位置
 * @return 返回主体所在位置的左上角和右下角像素点坐标
 */
std::vector<cv::Point2i> AAWVertexesGainer2::findMainBodyPosition(cv::Mat & inputImage)
{
    estimateMainBodySize(inputImage, grayScaleThresh4EstimateMainBody_);
    std::vector<cv::Point2i> boundPoints;

    int iRows = inputImage.rows;
    int iCols = inputImage.cols;
    int sr = sparceRatio_;
    int winCols = mainBodyApproxiCols_;
    int winRows = mainBodyApproxiRows_;

    //稀疏像素阵窗口的行列数
//    int sparceWinRows = winRows/sr; //55
    int sparceWinCols = winCols/sr; //110

    Eigen::MatrixXi colSumMat = calcColumnSumMatrix(inputImage);
    Eigen::VectorXi winSumCol = Eigen::VectorXi::Zero(colSumMat.rows());

    //老规矩，先让窗口贴着左侧边缘滑移一次
    for (int row = 0; row < colSumMat.rows(); ++row) {
        for (int col = 0; col < sparceWinCols; ++col) {
            winSumCol(row) += colSumMat(row, col);
        }
    }

    //滑动一下窗口，找到灰度值最小的地方，更新左上角的像素坐标
    int slideWinSum = winSumCol(0);
    cv::Point2i leftUpCorner, rightDownCorner;
    leftUpCorner.x = 0;
    leftUpCorner.y = 0;
    rightDownCorner.x = winCols - 1;
    rightDownCorner.y = winRows - 1;
    for (int row = 0; row < colSumMat.rows(); ++row) {
        if (slideWinSum > winSumCol(row)) {
            slideWinSum = winSumCol(row);
            leftUpCorner.y = row * sr;
        }
    }

    //然后每次向右移动一步，从上往下滑动，这样就只需要增量计算了
    for (int col = 1; col <= colSumMat.cols() - sparceWinCols; ++col) {
        for (int row = 0; row < colSumMat.rows(); ++row) {
            winSumCol(row) -= colSumMat(row, col - 1);
            winSumCol(row) += colSumMat(row, col+sparceWinCols-1);
            if (slideWinSum > winSumCol(row)) {
                slideWinSum = winSumCol(row);
                leftUpCorner.x = col * sr;
                leftUpCorner.y = row * sr;
            }
        }
    }

    rightDownCorner.x = leftUpCorner.x + winCols - 1;
    rightDownCorner.y = leftUpCorner.y + winRows - 1;

    if (rightDownCorner.x >= iCols - 1)
        rightDownCorner.x = iCols - 1;
    if (rightDownCorner.y >= iRows - 1)
        rightDownCorner.y = iRows - 1;

    boundPoints.push_back(leftUpCorner);
    boundPoints.push_back(rightDownCorner);

    return boundPoints;
}

/**
 * @brief getImageROI 由找到的主体位置划分出4个ROI用来拟合直线
 * @param cornerPoints 主体的左上角和右下角像素坐标
 * @return 上下左右4个ROI的信息，vector的每个元素首先存储边界直线方向的ROI像素上下限，然后存储窗口宽度方向的像素上下限（先存较小值）
 */
std::vector<cv::Vec4i> AAWVertexesGainer2::getImageROI(cv::Mat & inputImage, std::vector<cv::Point2i> corners)
{
    int iRows = inputImage.rows;
    int iCols = inputImage.cols;
    int lux = corners[0].x;
    int luy = corners[0].y;
    int rdx = corners[1].x;
    int rdy = corners[1].y;

    //上下界限左右肯定不会出界，由输入的角点保证的，同理，左右界限上下肯定不会出界
    std::vector<cv::Vec4i> imgROI;

    cv::Vec4i upLine;
    upLine[0] = int((rdx-lux)*(1-upTakesInRatio_)*0.5+lux);
    upLine[1] = int(rdx-(rdx-lux)*(1-upTakesInRatio_)*0.5);
    upLine[2] = luy-upBoxWidth_/2;
    upLine[3] = luy+upBoxWidth_/2;
    if (upLine[2] < 0)
        upLine[2] = 0;
    imgROI.push_back(upLine);

    cv::Vec4i downLine;
    downLine[0] = int((rdx-lux)*(1-downTakesInRatio_)*0.5+lux);
    downLine[1] = int(rdx-(rdx-lux)*(1-downTakesInRatio_)*0.5);;
    downLine[2] = rdy-downBoxWidth_/2;
    downLine[3] = rdy+downBoxWidth_/2;
    if (downLine[3] >= iRows)
        downLine[3] = iRows-1;
    imgROI.push_back(downLine);

    cv::Vec4i leftLine;
    leftLine[0] = int((rdy-luy)*(1-leftTakesInRatio_)*0.5+luy);
    leftLine[1] = int(rdy-(rdy-luy)*(1-leftTakesInRatio_)*0.5);
    leftLine[2] = lux-leftBoxWidth_/2;
    leftLine[3] = lux+leftBoxWidth_/2;
    if (leftLine[2] < 0)
        leftLine[2] = 0;
    imgROI.push_back(leftLine);

    cv::Vec4i rightLine;
    rightLine[0] = int((rdy-luy)*(1-rightTakesInRatio_)*0.5+luy);
    rightLine[1] = int(rdy-(rdy-luy)*(1-rightTakesInRatio_)*0.5);
    rightLine[2] = rdx-rightBoxWidth_/2;
    rightLine[3] = rdx+rightBoxWidth_/2;
    if (rightLine[3] >= iCols)
        rightLine[3] = iCols-1;
    imgROI.push_back(rightLine);

    return imgROI;
}

/**
 * @brief AAWVertexesGainer2::myLineFitting2 第2版拟合直线函数，整体和第一版差不多，只是有些关于ROI的参数提前确定了，一次拟合一条直线
 * @note 拟合的直线必须在输入图像中为竖直方向的直线，如果是水平方向的则将图像转置之后输入（这么做是为了提高计算效率）
 * @param image 输入灰度图像
 * @param startRow ROI的开始行号
 * @param endRow ROI的结束行号
 * @param colRangeLeft ROI的开始列号
 * @param colRangeRight ROI的技术列号
 * @param blockRight 主体是否在要拟合的直线的右侧？主要是用来消除主体之外的大灰度梯度物体干扰的
 * @param antiMessThreshold 消除主体之外干扰的阈值，偏移初始拟合直线超过这个阈值数量像素点的点会被剔除
 * @return 返回拟合的直线参数，前两位为方向向量，后两位为直线上一点
 */
cv::Vec4f AAWVertexesGainer2::myLineFitting2(cv::Mat &image,
                                             int startRow,
                                             int endRow,
                                             int colRangeLeft,
                                             int colRangeRight,
                                             bool blockRight,
                                             float antiMessThreshold)
{
    cv::Vec4f fittedLineInitial, fittedLineUltimate;
    std::vector<cv::Point2f> selectedPointsInitial, selectedPointsUltimate, initFittedLinePoints;
    int compareWindowSize;

    //The line should be vertical
    cv::Mat imageROIOriginal = image.rowRange(startRow, endRow + 1).clone();
    cv::Mat imageROI = imageROIOriginal.colRange(colRangeLeft, colRangeRight);

    compareWindowSize = colRangeRight - colRangeLeft - 1;

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
    if (blockRight) {
        for (int row = 0; row <= endRow - startRow; ++row) {
            if (!((initFittedLinePoints[row].x - selectedPointsInitial[row].x) > antiMessThreshold)) {
                selectedPointsUltimate.push_back(selectedPointsInitial[row]);
            }
        }
    }
    else {
        for (int row = 0; row <= endRow - startRow; ++row) {
            if (!((selectedPointsInitial[row].x - initFittedLinePoints[row].x) > antiMessThreshold)) {
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
std::vector<cv::Point2f> AAWVertexesGainer2::calcPointsThroughLine(cv::Vec4f inputLine, unsigned int yCount)
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
cv::Vec4f AAWVertexesGainer2::getTransposedLine(cv::Vec4f inputLine)
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
 * @note 必须先输入竖直方向的直线，然后输入水平方向的直线（主要用来判断这条直线的斜率是不是无穷大，即竖直直线）。
 * @param line1 输入类型为cv::Vec4f的直线1参数，为竖直方向的直线。
 * @param line2 输入类型为cv::Vec4f的直线2参数，为水平方向的直线。
 * @return 返回cv::Point2f类型的交点坐标。
 */
cv::Point2f AAWVertexesGainer2::calcIntersectionPointOf2Lines(cv::Vec4f line1, cv::Vec4f line2)
{
    float k1, k2, b1, b2, xIntersection, yIntersection;

    //line2为水平方向的线，斜率肯定不会出现无穷大的情况，所以可以计算
    k2 = line2[1]/line2[0];
    b2 = line2[3] - line2[1]*line2[2]/line2[0];

    //当line1为竖直线的时候，斜率无穷大
    if (line1[0] < 1e-6) {
        xIntersection = line1[2];
        yIntersection = k2*xIntersection + b2;
    }
    //当line1斜率不是无穷大的时候，可以计算交点
    else {
        k1 = line1[1]/line1[0];
        b1 = line1[3] - line1[1]*line1[2]/line1[0];
        xIntersection = (b2 - b1)/(k1 - k2);
        yIntersection = k1*xIntersection + b1;
    }

    cv::Point2f intersectionPoint(xIntersection, yIntersection);

    return intersectionPoint;
}

/**
 * @brief AawVertexesGainer::showInformatIon 将获取的边界、顶点和坐标实时显示在图片上。
 * @param image
 */
void AAWVertexesGainer2::showInformatIon(cv::Mat &image)
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

/**
 * @brief AAWVertexesGainer2::drawBoxes 绘出imgROI的位置，调试确定参数用。
 * @param inputImage
 * @param corners
 */
/*
void AAWVertexesGainer2::drawBoxes(cv::Mat & inputImage, std::vector<cv::Point2i> corners)
{
    int lux = corners[0].x;
    int luy = corners[0].y;
    int rdx = corners[1].x;
    int rdy = corners[1].y;

    int upBoxWidth = 150;
    int leftBoxWidth = 300;
    int rightBoxWidth = 300;
    int downBoxWidth = 150;

    float upTakesInRatio = 0.5;
    float leftTakesInRatio = 0.75;
    float rightTakesInRatio = 0.75;
    float downTakesInRatio = 0.5;

    float x, y;
    std::vector<cv::Point2f> points4Up;
    x = (rdx-lux)*(1-upTakesInRatio)*0.5+lux;
    y = luy - upBoxWidth*0.5;
    points4Up.push_back(cv::Point2f(x, y));
    y = luy + upBoxWidth*0.5;
    points4Up.push_back(cv::Point2f(x, y));
    x = rdx - (rdx-lux)*(1-upTakesInRatio)*0.5;
    points4Up.push_back(cv::Point2f(x, y));
    y = luy - upBoxWidth*0.5;
    points4Up.push_back(cv::Point2f(x, y));
    cv::line(inputImage, points4Up[0], points4Up[1], 255, 3);
    cv::line(inputImage, points4Up[1], points4Up[2], 255, 3);
    cv::line(inputImage, points4Up[2], points4Up[3], 255, 3);
    cv::line(inputImage, points4Up[3], points4Up[0], 255, 3);

    std::vector<cv::Point2f> points4Down;
    x = (rdx-lux)*(1-downTakesInRatio)*0.5+lux;
    y = rdy - downBoxWidth*0.5;
    points4Down.push_back(cv::Point2f(x, y));
    y = rdy + downBoxWidth*0.5;
    points4Down.push_back(cv::Point2f(x, y));
    x = rdx - (rdx-lux)*(1-downTakesInRatio)*0.5;
    points4Down.push_back(cv::Point2f(x, y));
    y = rdy - downBoxWidth*0.5;
    points4Down.push_back(cv::Point2f(x, y));
    cv::line(inputImage, points4Down[0], points4Down[1], 255, 3);
    cv::line(inputImage, points4Down[1], points4Down[2], 255, 3);
    cv::line(inputImage, points4Down[2], points4Down[3], 255, 3);
    cv::line(inputImage, points4Down[3], points4Down[0], 255, 3);

    std::vector<cv::Point2f> points4Left;
    x = lux - leftBoxWidth*0.5;
    y = (rdy-luy)*(1-leftTakesInRatio)*0.5+luy;
    points4Left.push_back(cv::Point2f(x, y));
    y = rdy - (rdy-luy)*(1-leftTakesInRatio)*0.5;
    points4Left.push_back(cv::Point2f(x, y));
    x = lux + leftBoxWidth*0.5;
    points4Left.push_back(cv::Point2f(x, y));
    y = (rdy-luy)*(1-leftTakesInRatio)*0.5+luy;
    points4Left.push_back(cv::Point2f(x, y));
    cv::line(inputImage, points4Left[0], points4Left[1], 255, 3);
    cv::line(inputImage, points4Left[1], points4Left[2], 255, 3);
    cv::line(inputImage, points4Left[2], points4Left[3], 255, 3);
    cv::line(inputImage, points4Left[3], points4Left[0], 255, 3);

    std::vector<cv::Point2f> points4Right;
    x = rdx - rightBoxWidth*0.5;
    y = (rdy-luy)*(1-rightTakesInRatio)*0.5+luy;
    points4Right.push_back(cv::Point2f(x, y));
    y = rdy - (rdy-luy)*(1-rightTakesInRatio)*0.5;
    points4Right.push_back(cv::Point2f(x, y));
    x = rdx + rightBoxWidth*0.5;
    points4Right.push_back(cv::Point2f(x, y));
    y = (rdy-luy)*(1-rightTakesInRatio)*0.5+luy;
    points4Right.push_back(cv::Point2f(x, y));
    cv::line(inputImage, points4Right[0], points4Right[1], 255, 3);
    cv::line(inputImage, points4Right[1], points4Right[2], 255, 3);
    cv::line(inputImage, points4Right[2], points4Right[3], 255, 3);
    cv::line(inputImage, points4Right[3], points4Right[0], 255, 3);
}
*/
