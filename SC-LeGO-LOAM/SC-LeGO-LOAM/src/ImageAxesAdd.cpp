#include "ImageAxesAdd.h"
DrawAxes::DrawAxes(){
}

DrawAxes::~DrawAxes(){
}

void DrawAxes::InputFigure(cv::Mat insidefigure, cv::Mat outsidefigure){
    image_noAxes = insidefigure;
    image_Axes = outsidefigure;
    InsidePic_cols = image_Axes.cols - image_noAxes.cols - 10;
    InsidePic_rows = image_Axes.rows - image_noAxes.rows - 50;
    for (int i = 0; i < image_noAxes.rows; i++){
        for (int j = 0; j < image_noAxes.cols; j++){
            image_Axes.at<cv::Vec3b>(i+ InsidePic_rows,j+InsidePic_cols)= image_noAxes.at<cv::Vec3b>(i,j);
        }
    }
}

void DrawAxes::DrawLabel_Y(std::string label_name, double min_value, double max_value, const int number, cv::Scalar label_color){
    cv::line(image_Axes, cv::Point(InsidePic_cols - 1, InsidePic_rows - 1),
                    cv::Point(InsidePic_cols- 1, InsidePic_rows + image_noAxes.rows),
                    label_color, 1, 8, 0);
    cv::line(image_Axes, cv::Point(InsidePic_cols + image_noAxes.cols, InsidePic_rows - 1),
                    cv::Point(InsidePic_cols + image_noAxes.cols, InsidePic_rows + image_noAxes.rows),
                    cv::Scalar(0, 0, 0), 1, 8, 0);
    double label_step = image_noAxes.rows / number;

    for (int i = 0; i <= number; i++){
        double value = double(max_value - min_value)/double(image_noAxes.rows) * label_step * i + min_value;
        char str[20];
        std::sprintf(str,"%.0lf", value);
        std::string result = str;
        cv::line(image_Axes, cv::Point(InsidePic_cols - 10, InsidePic_rows + image_noAxes.rows - label_step*i),
                        cv::Point(InsidePic_cols - 1, InsidePic_rows+ image_noAxes.rows - label_step*i),
                        label_color, 1, 8, 0);
        putText(image_Axes, result,
                        cv::Point(InsidePic_cols - 30, InsidePic_rows + image_noAxes.rows - label_step*i +3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.3, label_color, 1, 8);
    }
    int baseline;
    cv::Size text_size = getTextSize(label_name, cv::FONT_HERSHEY_SIMPLEX, 1, 8, &baseline);

    cv::Mat TextSizeframe = cv::Mat(text_size.height+5, text_size.width/2, CV_8UC3, cv::Scalar(255, 255, 255));
    putText(TextSizeframe, label_name,
            cv::Point(0, text_size.height-1),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, label_color, 1, 8);

    cv::Mat TextSizeframe1, TextSizeframe2;
    cv::transpose(TextSizeframe, TextSizeframe2);
    flip(TextSizeframe2, TextSizeframe1, 0);
    for (int i = 0; i < TextSizeframe1.rows; i++){
        for (int j = 0; j < TextSizeframe1.cols; j++){
            image_Axes.at<cv::Vec3b>(i + InsidePic_rows + image_noAxes.rows/2 - 80, j + InsidePic_cols -60) = TextSizeframe1.at<cv::Vec3b>(i, j);
        }
    }
}

void DrawAxes::DrawLabel_X(std::string label_name, double min_value, double max_value, const int number, cv::Scalar label_color){
    cv::line(image_Axes, cv::Point(InsidePic_cols - 1, InsidePic_rows - 1),
                    cv::Point(InsidePic_cols + image_noAxes.cols , InsidePic_rows - 1),
                    cv::Scalar(0, 0, 0), 1, 8, 0);
    cv::line(image_Axes, cv::Point(InsidePic_cols - 1, InsidePic_rows + image_noAxes.rows ),
                    cv::Point(InsidePic_cols + image_noAxes.cols, InsidePic_rows + image_noAxes.rows ),
                    label_color, 1, 8, 0);
    double label_step = image_noAxes.cols / number;

    for (int i = 0; i <= number; i++){
        double value = double(max_value - min_value)/double(image_noAxes.cols)* label_step * i + min_value;
        char str[20];
        sprintf(str, "%.0lf", value);//, 6);
        std::string result = str;
        cv::line(image_Axes, cv::Point(InsidePic_cols + label_step*i, InsidePic_rows + image_noAxes.rows),
                        cv::Point(InsidePic_cols + label_step*i, InsidePic_rows + image_noAxes.rows +10),
                        label_color, 1, 8, 0);
        putText(image_Axes, result,
                        cv::Point(InsidePic_cols + label_step*i-10, InsidePic_rows + image_noAxes.rows +20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.3, label_color, 1, 8);
    }
    putText(image_Axes, label_name,
            cv::Point(InsidePic_cols + image_noAxes.cols/2-100, InsidePic_rows + image_noAxes.rows + 40),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, label_color, 1, 8);
}

void DrawAxes::DrawTitle(std::string title_name, cv::Scalar title_color){
    putText(image_Axes, title_name,
            cv::Point(InsidePic_cols + image_noAxes.cols / 2 -200, InsidePic_rows - 20),
            cv::FONT_HERSHEY_SIMPLEX, 1, title_color, 2, 8);
}

