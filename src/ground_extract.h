#ifndef GROUND_EXTRACT_H
#define GROUND_EXTRACT_H

#include<iostream>
#include <vector>
#include "vpoint.h"

struct MyGrid
{
  float min_height;
  float max_height;
  int numPoints;
  int label;

  float slope;            //栅格斜率
  float roughness;        //栅格粗糙度
  float stepheight;       //栅格台阶高度
  float traversability;   //栅格可行性

  MyGrid();
  ~MyGrid();
};


class VGroudExtraction
{
public:
    VGroudExtraction();
    ~VGroudExtraction();

public:

    int extract(std::vector<VPointIL> &pointCloud);
    int extract(std::vector<VPointIL> &pointCloud, const float gridSize);
    int extract_trraversability(std::vector<VPointIL> &pointCloud);
    int extract_trraversability(std::vector<VPointIL> &pointCloud, const float gridSize);

private:
    float m_height_diff_threshold;      //两个栅格高度差，小于此阈值则认为连续
    int m_minNumInGrid;                 //每个栅格中最少要有几个点
    float m_seedGridRation;             //选择种子栅格时的比例
    float m_gridSize;                   //栅格大小
    float m_ground_diff_threshold;      //同一个栅格中，所允许的最大 地面的高度差

    int m_intAreaSize;                  //计算每个栅格时感兴趣区域长度（比如：3x3）
    float m_s_crit;                     //斜率临界值
    float m_r_crit;                     //粗糙度临界值
    float m_h_crit;                     //台阶高度临界值
    float m_s_weight;                   //斜率权重
    float m_r_weight;                   //粗糙度权重
    float m_h_weight;                   //台阶高度权重


private:
    //方差求解函数
    float GetVariance(std::vector<float> &inputVec);

    //获得最大高度差
    float GetStepHeight(float &cell1max, float &cell1min, float &cell2max, float &cell2min);

    //求实对称矩阵的特征值及特征向量的雅格比法
    int eejcb(double a[], int n, double v[], double eps, int jt);

    //计算法向量，返回与z夹角（注意方向）
    float Get_ps_fea(std::vector<VPointI> &_points);

};


#endif // GROUND_EXTRACT_H
