#include "ground_extract.h"
#include <algorithm>
#include <stack>
#include <cmath>
#include <numeric>
#include <QDebug>

MyGrid::MyGrid()
{
    min_height = 99999.9;
    max_height = -99999.9;
    numPoints = 0;
    label = 0;

    slope = 0.0;
    roughness = 0.0;
    stepheight = 0.0;
    traversability = 10.0;
}
MyGrid::~MyGrid()
{}

VGroudExtraction::VGroudExtraction()
{
    m_gridSize = 0.2;                           //栅格大小
    m_height_diff_threshold = 0.8;       //两个栅格高度差，小于此阈值则认为连续
    m_minNumInGrid = 1;                         //每个栅格中最少要有几个点
    m_seedGridRation = 0.1;                     //选择种子栅格时的比例
    m_ground_diff_threshold = 0.4;       //同一个栅格中，所允许的最大 地面的高度差

    m_intAreaSize = 5;                  //计算每个栅格时感兴趣区域长度（比如：3x3）
    m_s_crit = 30.0;                     //斜率临界值（°）
    m_r_crit = 0.02;                      //粗糙度临界值（方差）
    m_h_crit = 0.2;                      //台阶高度临界值
    m_s_weight = 0.4;                   //斜率权重
    m_r_weight = 0.3;                   //粗糙度权重
    m_h_weight = 0.3;                   //台阶高度权重
}


VGroudExtraction::~VGroudExtraction()
{

}

int VGroudExtraction::extract(std::vector<VPointIL> &pointCloud)
{
    return extract(pointCloud, m_gridSize);
}

//点云的占地面积不能太大，最好1000*1000以内，要不然，内存爆
int VGroudExtraction::extract(std::vector<VPointIL> &pointCloud, const float gridSize)
{
    //寻找边框：xmin、ymin、xmax、ymax
    float xmin=999999.0, ymin=999999.0, xmax=-999999.0, ymax=-999999.0;
    for(int i=0; i<pointCloud.size(); i++)
    {
        if(pointCloud[i].x==0.0 && pointCloud[i].y==0.0 && pointCloud[i].z==0.0) continue;
        if(pointCloud[i].x < xmin) xmin = pointCloud[i].x;
        if(pointCloud[i].y < ymin) ymin = pointCloud[i].y;
        if(pointCloud[i].x > xmax) xmax = pointCloud[i].x;
        if(pointCloud[i].y > ymax) ymax = pointCloud[i].y;
    }

    //确定栅格数组 长宽：_width、_height
    int _width = (xmax - xmin) / gridSize + 1;
    int _height = (ymax - ymin) / gridSize + 1;
    if(_width<10 || _height<10) return -1;

    std::vector<std::vector<MyGrid>> grids;
    grids.resize(_width);
    for(int i=0; i<_width; i++)
    {
        grids[i].resize(_height);
        for(int j=0; j<_height; j++)
        {
            grids[i][j].numPoints = 0;
            grids[i][j].min_height = 99999.9;
            grids[i][j].max_height = -99999.9;
            grids[i][j].label = 0;
        }
    }
    std::cout<<"grid size: "<<_width<<"  "<<_height<<std::endl;


    //将点云投至栅格
    for(int i=0; i<pointCloud.size(); i++)
    {
        if(pointCloud[i].x==0.0 && pointCloud[i].y==0.0 && pointCloud[i].z==0.0) continue;
        int row = (pointCloud[i].x - xmin) / gridSize;
        int col = (pointCloud[i].y - ymin) / gridSize;
        grids[row][col].numPoints++;
        if(pointCloud[i].z < grids[row][col].min_height)  //记录最低高度
            grids[row][col].min_height = pointCloud[i].z;
        if(pointCloud[i].z > grids[row][col].max_height)  //记录最高高度
            grids[row][col].max_height = pointCloud[i].z;
    }


    //寻找最低的若干栅格，作为种子地面栅格
    std::vector<float> grid_heights;
    for(int i=0; i<_width; i++)
    {
        for(int j=0; j<_height; j++)
        {
            if(grids[i][j].numPoints < m_minNumInGrid)  //点太少的，不要
                continue;
            float difference0 = fabs(grids[i][j].max_height -grids[i][j].min_height);
            if(difference0 > m_ground_diff_threshold)  //同一栅格内高度差太大的，不要
                continue;
            grid_heights.push_back(grids[i][j].min_height);
        }
    }

    sort(grid_heights.begin(), grid_heights.end()); //由小到大排序
    float threshold_height = grid_heights[(int)(grid_heights.size()*m_seedGridRation)];  //取高度最低的若干栅格，作为种子地面栅格

    for(int i=0; i<_width; i++)
        for(int j=0; j<_height; j++)
        {
            if(grids[i][j].numPoints < m_minNumInGrid)  //点太少的，不要
                continue;

            float difference0 = fabs(grids[i][j].max_height -grids[i][j].min_height);  //同一栅格内高度差太大的，不要
            if(difference0 > m_ground_diff_threshold)
                continue;

            if(grids[i][j].min_height <= threshold_height)
            {
               grids[i][j].label = 1; //标记为地面栅格
               std::cout << 'h';
            }
        }


    //邻域扩张，区域生长其它地面栅格
    struct index2D{int i; int j;};
    index2D tmpindex2d;
    std::stack<index2D> index_stack;
    int directions[8][2] = {-1,0, -1,1, 0,1, 1,1, 1,0, 1,-1, 0,-1, -1,-1};

    for(int i=0; i<_width; i++)
    {
        for(int j=0; j<_height; j++)
        {
            if(grids[i][j].label != 1)
                continue;

            tmpindex2d.i = i; tmpindex2d.j = j;
            index_stack.push(tmpindex2d);

            //利用栈 扩张
            while(!index_stack.empty())
            {
                //从栈中取出一个，标记为地面点
                tmpindex2d = index_stack.top();
                int current_i = tmpindex2d.i;
                int current_j = tmpindex2d.j;
                grids[current_i][current_j].label = 1;
                index_stack.pop();

                //遍历刚刚取出栅格 的八邻域，符合条件的入栈
                for(int k=0; k<8; k++)
                {
                    tmpindex2d.i = current_i + directions[k][0];
                    tmpindex2d.j = current_j + directions[k][1];

                    if(tmpindex2d.i<0 || tmpindex2d.i>=_width) continue;
                    if(tmpindex2d.j<0 || tmpindex2d.j>=_height) continue;

                    //已经被标记过了，不要
                    if(grids[tmpindex2d.i][tmpindex2d.j].label == 1)
                        continue;

                    //等于0，空格，直接当做地面点
                    if(grids[tmpindex2d.i][tmpindex2d.j].numPoints == 0)
                    {
                        grids[tmpindex2d.i][tmpindex2d.j].min_height = grids[current_i][current_j].min_height;
                        grids[tmpindex2d.i][tmpindex2d.j].max_height = grids[current_i][current_j].max_height;
                        index_stack.push(tmpindex2d);  //符合条件的入栈
                        continue;
                    }

                    //点太少了，不要
                    if(grids[tmpindex2d.i][tmpindex2d.j].numPoints < m_minNumInGrid)
                        continue;

                    //同一栅格内高度差太大的，不要
                    float difference0 = fabs(grids[tmpindex2d.i][tmpindex2d.j].max_height -grids[tmpindex2d.i][tmpindex2d.j].min_height);
                    if(difference0 > m_ground_diff_threshold)
                        continue;

                    //很重要：高度是否连续，不连续则不要
                    float difference = fabs(grids[tmpindex2d.i][tmpindex2d.j].max_height - grids[current_i][current_j].max_height);
                    if(difference > m_height_diff_threshold)
                        continue;

                    index_stack.push(tmpindex2d);  //符合条件的入栈
                }
            }
        }
    }


    //最后，重新将点云投至栅格，根据栅格属性确定此点标签
    for(int i=0; i<pointCloud.size(); i++)
    {
        if(pointCloud[i].x==0.0 && pointCloud[i].y==0.0 && pointCloud[i].z==0.0)
        {
            pointCloud[i].label = -1;  //无效点
            continue;
        }

        int row = (pointCloud[i].x - xmin) / gridSize;
        int col = (pointCloud[i].y - ymin) / gridSize;
        if(grids[row][col].numPoints >= m_minNumInGrid){
            if(grids[row][col].label == 1)
                pointCloud[i].label = 1;
            else pointCloud[i].label = 0;
        }else{
            pointCloud[i].label = -1;
        }
    }

    return 0;
}



int VGroudExtraction::extract_trraversability(std::vector<VPointIL> &pointCloud)
{
    return extract_trraversability(pointCloud, m_gridSize);
}


/*******************************  改进版  *******************************/
//点云的占地面积不能太大，最好1000*1000以内，要不然，内存爆
int VGroudExtraction::extract_trraversability(std::vector<VPointIL> &pointCloud, const float gridSize)
{
    //寻找边框：xmin、ymin、xmax、ymax
    float xmin=999999.0, ymin=999999.0, xmax=-999999.0, ymax=-999999.0;
    for(int i=0; i<pointCloud.size(); i++)
    {
        if(pointCloud[i].x==0.0 && pointCloud[i].y==0.0 && pointCloud[i].z==0.0) continue;
        if(pointCloud[i].x < xmin) xmin = pointCloud[i].x;
        if(pointCloud[i].y < ymin) ymin = pointCloud[i].y;
        if(pointCloud[i].x > xmax) xmax = pointCloud[i].x;
        if(pointCloud[i].y > ymax) ymax = pointCloud[i].y;
    }

    //确定栅格数组 长宽：_width、_height
    int _width = (xmax - xmin) / gridSize + 1;
    int _height = (ymax - ymin) / gridSize + 1;
    if(_width<10 || _height<10) return -1;

    //初始化栅格
    std::vector<std::vector<MyGrid>> grids;
    grids.resize(_width);
    for(int i=0; i<_width; i++)
    {
        grids[i].resize(_height);
        for(int j=0; j<_height; j++)
        {
            grids[i][j].min_height = 99999.9;
            grids[i][j].max_height = -99999.9;
            grids[i][j].slope = 0.0;            //栅格斜率
            grids[i][j].roughness = 0.0;        //栅格粗糙度
            grids[i][j].stepheight = 0.0;       //栅格台阶高度
            grids[i][j].traversability = 10.0;   //栅格可行性
            grids[i][j].label = 0;
        }
    }
    //std::cout<<"grid size: "<<_width<<"  "<<_height<<std::endl;

    //将点云投至栅格（1）：先记录最低高度
    for(int i=0; i<pointCloud.size(); i++)
    {
        if(pointCloud[i].x==0.0 && pointCloud[i].y==0.0 && pointCloud[i].z==0.0) continue;
        int row = (pointCloud[i].x - xmin) / gridSize;
        int col = (pointCloud[i].y - ymin) / gridSize;
        grids[row][col].numPoints++;
        if(pointCloud[i].z < grids[row][col].min_height)  //记录最低高度
            grids[row][col].min_height = pointCloud[i].z;
    }

    //将点云投至栅格（2）：然后记录最高度，为了处理悬空问题
    for(int i=0; i<pointCloud.size(); i++)
    {
        if(pointCloud[i].x==0.0 && pointCloud[i].y==0.0 && pointCloud[i].z==0.0) continue;
        int row = (pointCloud[i].x - xmin) / gridSize;
        int col = (pointCloud[i].y - ymin) / gridSize;
        if(pointCloud[i].z > grids[row][col].max_height)  //记录最高高度
            if((pointCloud[i].z-grids[row][col].min_height) < 1.5)  //最低高度1.5m以上的点不考虑
                grids[row][col].max_height = pointCloud[i].z;
    }

    //计算每个栅格中的相关指标
    VPointI tempVP;
    for(int i=0; i<_width; i++)
    {
        for(int j=0; j<_height; j++)
        {
            if(grids[i][j].numPoints <= 0) //没有点直接赋值、返回
            {
                grids[i][j].traversability = 1.0f;
                continue;
            }

            if((grids[i][j].max_height-grids[i][j].min_height) > 1.4)
            {
                grids[i][j].traversability = 2.0f;
                continue;
            }

            //收集邻域内的点
            std::vector<VPointI> tempVec;
            std::vector<float> temp_heights;
            float max_stepheight = 0.0;
            int stepheight_cnt = 0;
            for(int n=i-2; n<i+2; n++)
            {
                if(n<0 || n>=_width) continue;
                for(int m=j-2; m<j+2; m++)
                {
                    if(m<0 || m>=_height) continue;
                    if(grids[n][m].numPoints <= 0) continue;  //没有点

                    //保存相关信息
                    tempVP.x = (n + 0.5) * gridSize;
                    tempVP.y = (m + 0.5) * gridSize;
                    tempVP.z = grids[n][m].min_height;
                    tempVP.intensity = grids[n][m].max_height;  //借用intensity存最高高度
                    tempVec.push_back(tempVP);
                    temp_heights.push_back(grids[n][m].max_height);

                    //计算tmp step height
                    if(n==i && m==j) continue;
                    float tmp_stepheight = GetStepHeight(grids[n][m].max_height, grids[n][m].min_height,
                                                         grids[i][j].max_height, grids[i][j].min_height);
                    //cout<<"hgj==tmp_stepheight="<<tmp_stepheight;
                    if(tmp_stepheight > m_h_crit)
                    {
                        float angle_z = atan(gridSize * sqrt((i-n)*(i-n) + (j-m)*(j-m)) / tmp_stepheight) * 180.0 / 3.1415926;
                        //cout<<"hgj==angle_z="<<angle_z<<endl;
                        if(angle_z > m_s_crit)  //若斜率也大于临界值(考虑斜坡地带)
                        {
                            stepheight_cnt++;
                            if(tmp_stepheight > max_stepheight)
                            {
                                max_stepheight = tmp_stepheight;
                            }
                        }
                    }
                }
            }

            if(tempVec.size() < 10)  //邻域信息太少了
            {
                grids[i][j].traversability = 1.0f;
                continue;
            }

            //法向量与z角度差 斜率
            grids[i][j].slope = Get_ps_fea(tempVec);

            //粗糙度
            grids[i][j].roughness = GetVariance(temp_heights);

            //step height
            grids[i][j].stepheight = max_stepheight < (max_stepheight*stepheight_cnt/1.0)
                                    ? max_stepheight : (max_stepheight*stepheight_cnt/1.0);

            //融合
            grids[i][j].traversability = m_s_weight * grids[i][j].slope / m_s_crit
                                       + m_r_weight * grids[i][j].roughness / m_r_crit
                                       + m_h_weight * grids[i][j].stepheight / m_h_crit;


        //cout<<"hgj=="<<stepheight_cnt<<"  "<<max_stepheight<<"  "<<grids[i][j].stepheight<<endl;
        }
    }



    //traversability算法对于飘在空中的平面无法区别（例如房顶），因此要将其找出来直接将traversability置为大数，
    //方法：从所有“traversability大于阈值，且栅格内高度差大于1.4m的栅格”种子开始，邻域扩张（最高高度和这个差不多的）；
    //本质上：“traversability大于阈值，且栅格内高度差大于1.4m的栅格” 就是“墙”的栅格，即由墙栅格开始扩张






    //最后，重新将点云投至栅格，根据栅格属性确定此点标签
    for(int i=0; i<pointCloud.size(); i++)
    {
        if(pointCloud[i].x==0.0 && pointCloud[i].y==0.0 && pointCloud[i].z==0.0)
        {
            pointCloud[i].label = -1;  //无效点
            continue;
        }

        int row = (pointCloud[i].x - xmin) / gridSize;
        int col = (pointCloud[i].y - ymin) / gridSize;
        if(grids[row][col].numPoints >= m_minNumInGrid)
        {
            if((pointCloud[i].z-grids[row][col].min_height) >= 1.5 )
                pointCloud[i].label = 0;
            else if(grids[row][col].traversability <= 1.0)
                pointCloud[i].label = 1;
            else pointCloud[i].label = 0;
        }
        else
        {
            pointCloud[i].label = -1;
        }
    }

    return 0;
}







//方差求解函数
float VGroudExtraction::GetVariance(std::vector<float> &inputVec)     //一个vector的方差
{
    if(inputVec.size() == 0)
        return 0;
    float sum = std::accumulate(std::begin(inputVec), std::end(inputVec), 0.0);
    float mean =  sum / inputVec.size(); //均值

    float accum  = 0.0;
    std::for_each (std::begin(inputVec), std::end(inputVec), [&](const float d) {
        accum  += (d-mean)*(d-mean);
    });

    float stdev = accum/inputVec.size(); //方差
    return stdev;
}

//获得最大高度差
float VGroudExtraction::GetStepHeight(float &cell1max, float &cell1min, float &cell2max, float &cell2min)
{
    //float max = cell1max > cell2max ? cell1max : cell2max;
    //float min = cell1min < cell2min ? cell1min : cell2min;
    //return fabs(max - min);
    return fabs(cell2max - cell1max);
}

//求实对称矩阵的特征值及特征向量的雅格比法
//利用雅格比(Jacobi)方法求实对称矩阵的全部特征值及特征向量
//返回值小于0表示超过迭代jt次仍未达到精度要求
//返回值大于0表示正常返回
//a-长度为n*n的数组，存放实对称矩阵，返回时对角线存放n个特征值
//n-矩阵的阶数
//u-长度为n*n的数组，返回特征向量(按列存储)
//eps-控制精度要求
//jt-整型变量，控制最大迭代次数
int VGroudExtraction::eejcb(double a[], int n, double v[], double eps, int jt)
{
    int i,j,p,q,u,w,t,s,l;
    double fm,cn,sn,omega,x,y,d;
    l=1;
    for (i=0; i<=n-1; i++)
    {
        v[i*n+i]=1.0;
        for (j=0; j<=n-1; j++)
        {
            if (i!=j)
            {
                v[i*n+j]=0.0;
            }
        }
    }
    while (1==1)
    {
        fm=0.0;
        for (i=0; i<=n-1; i++)
        {
            for (j=0; j<=n-1; j++)
            {
                d=fabs(a[i*n+j]);
                if ((i!=j)&&(d>fm))
                {
                    fm=d;
                    p=i;
                    q=j;
                }
            }
        }
        if (fm<eps)
        {
            return(1);
        }
        if (l>jt)
        {
            return(-1);
        }
        l=l+1;
        u=p*n+q;
        w=p*n+p;
        t=q*n+p;
        s=q*n+q;
        x=-a[u];
        y=(a[s]-a[w])/2.0;
        omega=x/sqrt(x*x+y*y);
        if (y<0.0)
        {
            omega=-omega;
        }
        sn=1.0+sqrt(1.0-omega*omega);
        sn=omega/sqrt(2.0*sn);
        cn=sqrt(1.0-sn*sn);
        fm=a[w];
        a[w]=fm*cn*cn+a[s]*sn*sn+a[u]*omega;
        a[s]=fm*sn*sn+a[s]*cn*cn-a[u]*omega;
        a[u]=0.0;
        a[t]=0.0;
        for (j=0; j<=n-1; j++)
        {
            if ((j!=p)&&(j!=q))
            {
                u=p*n+j;
                w=q*n+j;
                fm=a[u];
                a[u]=fm*cn+a[w]*sn;
                a[w]=-fm*sn+a[w]*cn;
            }
        }
        for (i=0; i<=n-1; i++)
        {
            if ((i!=p)&&(i!=q))
            {
                u=i*n+p;
                w=i*n+q;
                fm=a[u];
                a[u]=fm*cn+a[w]*sn;
                a[w]=-fm*sn+a[w]*cn;
            }
        }
        for (i=0; i<=n-1; i++)
        {
            u=i*n+p;
            w=i*n+q;
            fm=v[u];
            v[u]=fm*cn+v[w]*sn;
            v[w]=-fm*sn+v[w]*cn;
        }
    }
    return(1);
}


float VGroudExtraction::Get_ps_fea(std::vector<VPointI> &_points)
{
    //点云特征，先需计算这堆点云的协方差矩阵，计算出3个特征值、特征向量

    //1)中心点center
    VPointI center;
    center.x = center.y = center.z = 0;
    int cnt_ = 0;
    for (int i=0; i<_points.size(); i++)
    {
        if(_points[i].x==0 && _points[i].y==0)
            continue;
        center.x += _points[i].x;
        center.y += _points[i].y;
        center.z += _points[i].z;
        cnt_++;
    }
    center.x /= (float)cnt_;
    center.y /= (float)cnt_;
    center.z /= (float)cnt_;

    //2) 协方差矩阵cov
    double cov[9] = {0.0}; //存放协方差矩阵
    int local_num = _points.size();
    for(int i=0; i<local_num; i++)
    {
        if(_points[i].x==0 && _points[i].y==0)
            continue;

        float buf[3];
        buf[0] = _points[i].x - center.x;
        buf[1] = _points[i].y - center.y;
        buf[2] = _points[i].z - center.z;
        for(int pp=0; pp<3; pp++)
        {
            for(int l=0; l<3; l++)
            {
                cov[pp*3+l] += buf[pp]*buf[l];
            }
        }
    }
    ////除以个数
    //for (int i=0; i<9; i++)
    //{
    //	cov[i] /= (double)cnt_;
    //}

    //3)计算cov的特征值、特征向量
    //---- 计算出的特征值为cov[0]、cov[4]、cov[8]；对应的
    //     特征向量在v中
    double v[9] = {0.0}; //存放特征向量
    eejcb(cov, 3, v, 0.001, 100);

    //4)整理结果，将三个特征值从小到大排列，并与特征向量对应好
    double eigenvalues[3] = {cov[0], cov[4], cov[8]};
    eigenvalues[0] = sqrt(eigenvalues[0] / cnt_);
    eigenvalues[1] = sqrt(eigenvalues[1] / cnt_);
    eigenvalues[2] = sqrt(eigenvalues[2] / cnt_);
    VPointI temp1, temp2, temp3;
    temp1.x = v[0];
    temp1.y = v[3];
    temp1.z = v[6];
    temp2.x = v[1];
    temp2.y = v[4];
    temp2.z = v[7];
    temp3.x = v[2];
    temp3.y = v[5];
    temp3.z = v[8];
    VPointI eigenvectors[3] = {temp1, temp2,temp3};
    for (int i=2; i>0; i--)  //冒泡排序
    {
        for (int j=0; j<i; j++)
        {
            if (eigenvalues[j] > eigenvalues[j+1])
            {
                double tmpVa = eigenvalues[j];
                VPointI tmpVe;
                tmpVe.x = eigenvectors[j].x;
                tmpVe.y = eigenvectors[j].y;
                tmpVe.z = eigenvectors[j].z;
                eigenvalues[j] = eigenvalues[j+1];
                eigenvectors[j] = eigenvectors[j+1];
                eigenvalues[j+1] = tmpVa;
                eigenvectors[j+1] = tmpVe;
            }
        }
    }

    //6) directional features
    VPointI vt; //切向量，最大特征值所对应的特征向量
    vt.x = eigenvectors[2].x;
    vt.y = eigenvectors[2].y;
    vt.z = eigenvectors[2].z;
    VPointI vn; //法向量，最小特征值所对应的特征向量
    vn.x = eigenvectors[0].x;
    vn.y = eigenvectors[0].y;
    vn.z = eigenvectors[0].z;

    //cout<<"++++++"<<vn.x<<"  "<<vn.y<<"   "<<vn.z<<endl;

    float angle1 = acos(fabs(vn.z)) * 180.0 / 3.1415926;
    return angle1;  //范围0--pi/2
}

