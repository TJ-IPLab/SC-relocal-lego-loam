#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
class DrawAxes{
public:
    DrawAxes();
    ~DrawAxes();
    //给输入图片加入一个外框，并输出加框之后的大图
    void InputFigure(cv::Mat insidefigure, cv::Mat outsidefigure);
    //画X轴 参数<坐标名，最小值，最大值，个数，颜色>
    void DrawLabel_X(std::string label_name, double min_value, double max_value, const int number, cv::Scalar label_color);
    //画Y轴 参数<坐标名，最小值，最大值，个数，颜色>
    void DrawLabel_Y(std::string label_name, double min_value, double max_value, const int number, cv::Scalar label_color);
    //画标题
    void DrawTitle(std::string title_name, cv::Scalar title_color = cv::Scalar(0, 0, 0));//添加标题

    cv::Mat image_noAxes;  //无坐标轴
    cv::Mat image_Axes;    //有坐标轴
    int InsidePic_cols;//内图的宽的开始像素
    int InsidePic_rows;//内图的高的开始像素
};

