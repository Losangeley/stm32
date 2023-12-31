#include "underpan.h"
//此函数用于将底盘转过相应的角度，使得上方的机械臂能够正对所需到达的点（球）
//初始状态约定为右手坐标系下与x轴重合

//####

void underpan_roll(float x,float y)
{
    float angle;
    if(x!=0&&y!=0)
    {//用于存放底盘需转动的角度，注意，此处是从底盘电机total_angle为0时开始转动
        angle=atan(y/x);//得到转角的基准值
        angle=angle/(2*pi)*8192;
        //以下通过转角的基准值得到最终需转动的角度
        if(x<0&&y>0)///cxxx
        {
            angle=angle;
        }
        if(x>0&&y<0)
        {
            angle=-1*angle;
        }
        if(x<0&&y>0)
        {
            angle=angle+8192/4;
        }
        if(x<0&&y<0)
        {
            angle=angle+8192/2;
        }
    }
    //不可能出现x,y同时为0的情况
    if(x==0)
    {
        if(y>0)
        {
            angle=8192/4;
        }
        else
        {
            angle=-1*8192/4;
        }
    }
    if(y==0)
    {
        if(x>0)
        {
            angle=0;
        }
        else
        {
            angle=8192/2;
        }
    }
    //开始转动底盘电机
    Change_dji_loc(motor_id1,angle);
}