#pragma once

#include "FallDetector.h"

int counter = 0;

//返回vec集合的标准差
double FallDetector::getStddev(vector<double>* vec) {
    Scalar mean, stddev;
    if(!vec->empty()) {
        meanStdDev(*vec, mean, stddev);
    }
    return stddev[0];
}

//根据当前帧和历史帧，从中提取出人的运动系数
double FallDetector::getMovementCoefficient(Mat* foreground,Mat* history) {
    double sumForeground = sum(*foreground)[0];
    double sumHistory = sum(*history)[0];
    return (sumHistory / sumForeground) * 100.0;
}

//判断人摔倒后是否不再运动，若是则判断成功，没有起身
void FallDetector::checkIfStaysInPlace(time_t start, bool* isChecking, bool* isFall, vector<double> xPos, vector<double> yPos) {
    double secondsSinceStart = difftime( time(0), start);
    double xDevValue;
    double yDevValue;


    xDevValue = this->getStddev(&xPos);
    yDevValue = this->getStddev(&yPos);
    //获取x,y方向点集的标准差，标准差小则意味着位置保持稳定

    if(xDevValue < 2 && yDevValue < 2) {
        *isFall = true;
        counter = counter + 1;
        if (counter == 6){
        system("sudo python3 /home/pi/Real-time-embedded-project/external/emailsender.py");
        counter = 0;
        }
        
        //连续6帧保持稳定则确认跌倒

    }

    if (!isFall && secondsSinceStart > 2) {
        *isChecking = false;
        //帧与帧之间间隔大说明人静止的状态不连续，因此人起身了
    }
}

void FallDetector::analyzePosition(Mat* frame, vector<double>* thetaRatio, vector<double>* aRatio, vector<double>* bRatio, vector<double>* xPos, vector<double>* yPos, vector<Point> largestContour) {
    Rect boundingRectangle = boundingRect(largestContour);//确定矩形的形状
    rectangle(*frame, boundingRectangle, Scalar(0, 255, 0), 2);//在图中画出矩形

    if(largestContour.size() > 5) {

        RotatedRect e = fitEllipse(largestContour);// 确定椭圆的形状
        ellipse( *frame, e, Scalar(255, 0, 0), 2 );// 在图中画出椭圆

        thetaRatio->push_back(e.angle);

        double a = (double)e.size.width / 2.0;
        double b = (double)e.size.height / 2.0;
        aRatio->push_back(a);
        bRatio->push_back(b);

        double x = e.center.x;
        double y = e.center.y;
        xPos->push_back(x);
        yPos->push_back(y);
        //从帧frame以及人的框架点集largestContour中提取出椭圆相关信息，长轴短轴中心点位置等等. 存到入参的vector中。
        
        //最多存放10帧的信息，超出则从vector头部开始删除。
        if(thetaRatio->size() > 10) {
            thetaRatio->erase(thetaRatio->begin());
        }

        if(aRatio->size() > 10) {
            aRatio->erase(aRatio->begin());
        }
        if(bRatio->size() > 10) {
            bRatio->erase(bRatio->begin());
        }
        if(xPos->size() > 10) {
            xPos->erase(xPos->begin());
        }
        if(yPos->size() > 10) {
            yPos->erase(yPos->begin());
        }
    }
}

void FallDetector::checkMovementAfterFall(bool* toBeChecked, bool *isFall, vector<double> xPos, vector<double> yPos) {
    //人跌倒判断后续 tobechecked标志位的复位。
    double xDevValue = this->getStddev(&xPos);
    double yDevValue = this->getStddev(&yPos);

    cout << "X pos: " << xDevValue;
    cout << ", Y pos: " << yDevValue << "\n";

    if(xDevValue > 2 && yDevValue > 2) {
        *isFall = false;
        *toBeChecked = false;
    }
}
