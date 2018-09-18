#include "ros/ros.h"
#include "ros/ros.h"
#include <stdio.h>
#include <time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Inertia.h>
#include <sstream>
#include <math.h>
#include <termios.h>
#include <signal.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <i2r_baxter_motion/Human.h>
#define num_n 4
#define num_m 4
#define freq 10
using namespace std;
i2r_baxter_motion::Human hdata;
geometry_msgs::Twist debug_states;
int time_itr;
double dir_sgn;
geometry_msgs::Vector3 human_pos[num_m+1]; //m+1[0]: current pos, [1:n-1]: previous poses
geometry_msgs::Vector3 human_vel[num_n+2]; //n+1[0:m-1]: previous velocities
geometry_msgs::Vector3 cur_vel;
geometry_msgs::Vector3 pos_i, pos_f;
double dir_h[2], len_h, xh0[2];
ros::Publisher human_pub, debug_pub;
double d_i, d_f;
int human_region[2];
int human_path;
double sh_0;
double sh, sh_max, sh_pre;
ros::Time previous_time;
vector<double> theta_x(num_n+num_m+1,0);
vector<double> theta_y(num_n+num_m+1,0);
double l_max = 0;
double l_min = 0;
const double l_sen = 0.25;  // sensivity of the l from initial point to final point

double duration()
{
  ros::Duration dt;
  dt = ros::Time::now()-previous_time;
  previous_time = ros::Time::now();
  return dt.toSec();
}

double dis(geometry_msgs::Vector3 data1,geometry_msgs::Vector3 data2)
{
  return sqrt(pow(data1.x-data2.x,2)+pow(data1.y-data2.y,2)+pow(data1.z-data2.z,2));
}

void vel(geometry_msgs::Vector3 data1, geometry_msgs::Vector3 data2, geometry_msgs::Vector3 & out, double &dts)
{
  dts = duration();
  out.x=(data2.x-data1.x)/dts;
  out.y=(data2.y-data1.y)/dts;
  out.z=(data2.z-data1.z)/dts;
}

vector<double> estimate(vector<double> &u, double y)
{
    int i,j;
    int n, m;
    n = num_n;
    m = num_m;
    double norm=10;
    double kt_denum=0;
    double utheta=0;
    const double k=100;
    //vector<vector <double> > u(N,vector <double> (n+m+1,0));
//    vector<double> u(n+m+1,0);
    vector<double> kt(n+m+1,0);
    vector<double> P_t_1_u(n+m+1,0);
    vector<double> theta(n+m+1,0);
    vector<double> theta_t_1(n+m+1,0);
    vector<vector <double> > P_t_1(n+m+1,vector <double> (n+m+1,0));
    vector<vector <double> > P_t(n+m+1,vector <double> (n+m+1,0));
    vector<vector <double> > I_ktu(n+m+1,vector <double> (n+m+1,0));
    double eps=.01;
    while (norm>=eps)
    {
        theta_t_1=theta;
        for (i=0; i<n+m+1 ; i++)
        {
            P_t_1[i][i] = k;
        }
        for(i=0; i<n+m+1; ++i)
        {
            for(j=0; j<m+n+1; ++j)
                {
                    P_t_1_u[i]+=P_t_1[i][j]*u[j];
                }
        }
//        kt_denum=0;
        kt_denum=1;
        for (i=0; i<n+m+1 ; i++)
        {
//        1 + uFu  ?
               kt_denum += P_t_1_u[i]*u[i];
               kt_denum += P_t_1_u[i]*u[i];
        }
        for(j=0; j<m+n+1; ++j)
        {
            kt[j]=P_t_1_u[j]/kt_denum;
        }
        utheta=0;
        for(j=0; j<m+n+1; ++j)
        {
                utheta+=u[j]*theta[j];
        }
        for(j=0; j<m+n+1; ++j)
        {
                theta[j] += kt[j]*(y-utheta);
        }
        for(i=0; i<m+n+1; ++i)
        {
            for (j=0; j<m+n+1; ++j)
            {
                    if (i==j)
                        {
                           I_ktu[i][j]=(1-kt[i]*u[j]);  
                        }
                    else
                    {
                        I_ktu[i][j]=(0-kt[i]*u[j]);
                    }
             }
        }

        for(i=0; i<n+m+1; ++i)
        {
            for(j=0; j<m+n+1; ++j)
                for(int k=0; k<m+n+1; ++k)
                {
                    {
                    P_t[i][j]+=I_ktu[i][k]*P_t_1[k][j];
                    }
                }
        }
        P_t_1=P_t;
        norm=0;
        for(i=0; i<n+m+1; ++i)
        {
            norm+=pow(theta[i]-theta_t_1[i],2);
        }
     }
    return theta;
}

int get_region()
{
  if (dis(human_pos[0], pos_i)<d_i){
    return 1;
  }else if(dis(human_pos[0], pos_f)<d_f){
    return 2;
  }else{
    return 3;
  }
}

int get_path()
{
  int diff=human_region[0]-human_region[1];
  switch ( diff ) {
  case 0:
    if (human_region[0]==1){
      if (sh>0.95){
        xh0[0]=pos_i.x;
        xh0[1]=pos_i.y;
       } else if (sh<0.0){
        xh0[0]=pos_i.x;
        xh0[1]=pos_i.y;
       } 
    }
    if (human_region[0]==2){
      if (sh>0.45){
        xh0[0]=pos_f.x;
        xh0[1]=pos_f.y;
       } else if (sh_0-sh>0.0){
        xh0[0]=pos_f.x;
        xh0[1]=pos_f.y;
       } 
    }
    return human_path;
    break;
  case 2:   //from r1 > r3 (fwd motion)
    xh0[0]=pos_i.x;
    xh0[1]=pos_i.y;
    dir_sgn = 1.0;
    l_max = 0;
    l_min = 0;
    return 4;
    break;
  case -1:  //from r3 > r2 (fwd end)
    if (fmod(sh_0, 1)<0.01)
    {
      sh_0 += 0.5;
    }
    xh0[0]=pos_f.x;
    xh0[1]=pos_f.y;
    dir_sgn = 0.0;
    return 2;
    break;
  case 1:   //from r2 > r3 (bkwd motion)
    dir_sgn = -1.0;
    l_max = 0;
    return 5;
    break;
  case -2:  //from r3 > r1 (bkwd end)
    if (fmod(sh_0, 1)>0.49)
    {
      sh_0 += 0.5;
    }
    xh0[0]=pos_i.x;
    xh0[1]=pos_i.y;
    dir_sgn = 0.0;
    return 1;
    break;
  default:
    return human_path;
    break;
}

  if (human_region[0]==human_region[1]){
    return human_path;
  }else if(dis(human_pos[0], pos_f)<d_f){
    return 2;
  }else{
    return 3;
  }
}


void set_path(int path)
{
    if (path ==5){  // from r2 to r3
        xh0[0]=pos_f.x;
        xh0[1]=pos_f.y;
        dir_sgn = -1.0;
        human_region[1] = 2;
        sh_0 += 0.5;

     } else {          // from r1 to r3
        xh0[0]=pos_i.x;
        xh0[1]=pos_i.y;
        dir_sgn = 1.0;
        human_region[1] = 1;
        sh_0 += 0.5;

     }
//    l_max = 0;
//    l_min = 0;
//     sh_0 += 0.5;
}




void ledACallback(geometry_msgs::Vector3 data)
{
  if (data.x!=0.0 && data.y!=0.0 && data.z!=0.0)
  {
    int n, m;
    n = num_n;
    m = num_m;
    // pos
    for (int j = m+1; j>0; j--)
    {
      human_pos[j].x = human_pos[j-1].x;
      human_pos[j].y = human_pos[j-1].y;
      human_pos[j].z = human_pos[j-1].z;
    }
    human_pos[0].x = data.x;
    human_pos[0].y = data.y;
    human_pos[0].z = data.z;
    human_region[1]=human_region[0];
    human_region[0]=get_region();
    // S_H
    double l = (dir_h[0]*(human_pos[0].x-xh0[0])+dir_h[1]*(human_pos[0].y-xh0[1]))/len_h;

    debug_states.linear.x = l;
    debug_states.linear.y = float(dir_sgn);
    debug_states.linear.z = sh_0;

    if (dir_sgn > 0){  // from 1 to 2
        if (l>.5){l = .5;}
        else if (l<0){l=0;}
        if (l>l_max){l_max=l;}
        if (l_max-l-l_sen>0 || l==.5){set_path(5);}
    }
    else if (dir_sgn < 0){             //from 2 to 1
        if(l<-.5){l=-.5;}
        else if (l>0){l=0;}
        if (l>l_max){l_max=l;}
        if (l<l_min){l_min=l;}
        if (l_min-l+l_sen<0  || l==-.5){set_path(4);}
    }
    // change S_H
    human_path = get_path();
    sh = dir_sgn*l+sh_0;

    debug_states.angular.x = human_path;
    debug_states.angular.y = sh;
    debug_states.angular.z = l_max-l-l_sen;
    debug_pub.publish(debug_states);

    sh = (sh < sh_pre) ? sh_pre : sh;
    sh_max = (sh > sh_max) ? sh : sh_max;

//    std::cout << sh << endl;
//    std::cout << "sh: " << std::setw(5) << sh << "region: " << human_region[0] <<" dir_h: " << dir_h[0]<< ", " << dir_h[1];
//    std::cout << "d_i: " << dis(human_pos[0], pos_i) << " d_f: " << dis(human_pos[0], pos_f) << " \n";
//    cout << "region: " << human_region[0] << "\t path:" << human_path << endl;

    // vel
    double vhx, vhy, dt;
    dt = duration();
    vhx=(human_pos[0].x-human_pos[1].x)/dt;
    vhy=(human_pos[0].y-human_pos[1].y)/dt;
    for (int j = n+2; j>1; j--)
    {
      human_vel[j].x = human_vel[j-1].x;
      human_vel[j].y = human_vel[j-1].y;
    }
    human_vel[0].x = vhx;
    human_vel[0].y = vhy;
    human_vel[1].x = human_vel[0].x;
    human_vel[1].y = human_vel[0].y;

    // estimator
    vector<double> u_x(n+m+1,0);
    vector<double> u_y(n+m+1,0);
    for (int j = 0; j<n; j++)
    {
      u_x[j] = -human_vel[j+2].x;
      u_y[j] = -human_vel[j+2].y;
    }   
    for (int j = 0; j<m+1; j++)
    {
      u_x[j+n] = human_pos[j].x;
      u_y[j+n] = human_pos[j].y;
    }   
    theta_x = estimate(u_x, human_vel[1].x);
    theta_y = estimate(u_y, human_vel[1].y);
    
    // publisher data
    hdata.x0 = human_pos[0].x;
    hdata.x1 = human_pos[1].x;
    hdata.x2 = human_pos[2].x;
    hdata.x3 = human_pos[3].x;
    hdata.x4 = human_pos[4].x;
    hdata.xd0 = human_vel[1].x; 
    hdata.xd1 = human_vel[2].x; 
    hdata.xd2 = human_vel[3].x; 
    hdata.xd3 = human_vel[4].x; 
    hdata.xd4 = human_vel[5].x; 
    hdata.y0 = human_pos[0].y; 
    hdata.y1 = human_pos[1].y;
    hdata.y2 = human_pos[2].y;
    hdata.y3 = human_pos[3].y;
    hdata.y4 = human_pos[4].y;
    hdata.yd0 = human_vel[1].y; 
    hdata.yd1 = human_vel[2].y; 
    hdata.yd2 = human_vel[3].y; 
    hdata.yd3 = human_vel[4].y; 
    hdata.yd4 = human_vel[5].y; 

    hdata.sh = sh;
    sh_pre = sh;
    hdata.sh0 = sh_0;
    hdata.dt = dt;
    hdata.dir_sgn = dir_sgn;
    hdata.path = human_path;
    hdata.tx0 = theta_x[0];
    hdata.tx1 = theta_x[1];
    hdata.tx2 = theta_x[2];
    hdata.tx3 = theta_x[3];
    hdata.tx4 = theta_x[4];
    hdata.tx5 = theta_x[5];
    hdata.tx6 = theta_x[6];
    hdata.tx7 = theta_x[7];
    hdata.tx8 = theta_x[8];
    hdata.ty0 = theta_y[0];
    hdata.ty1 = theta_y[1];
    hdata.ty2 = theta_y[2];
    hdata.ty3 = theta_y[3];
    hdata.ty4 = theta_y[4];
    hdata.ty5 = theta_y[5];
    hdata.ty6 = theta_y[6];
    hdata.ty7 = theta_y[7];
    hdata.ty8 = theta_y[8];
    hdata.xi = xh0[0];
    hdata.yi = xh0[1];
    hdata.z0 = human_pos[0].z;
    human_pub.publish(hdata);
  }
}

int main(int argc, char **argv)
{
  time_itr = 1;
  std::ifstream config_file("/home/baxter/ros_ws/src/i2r_baxter_motion/config.txt");
//  std::ifstream config_file("../config.txt");
  std::string str;
  std::getline(config_file, str);
  config_file >> pos_i.x >> pos_i.y >> pos_i.z;
  config_file >> pos_f.x >> pos_f.y >> pos_f.z;
  config_file.close();
  std::cout << str << endl;
  std::cout << pos_i.x << " " << pos_i.y << " " << pos_i.z << " \n";
  std::cout << pos_f.x << " " << pos_f.y << " " << pos_f.z << " \n";

  d_i=200;		  d_f=200;
  human_region[0] = 1; //current region
  human_region[1] = 1; //previous region
  human_path = 1; //current path
  sh_0 = 0.0;
  sh = 0.0;
  sh_pre = 0.0;
  sh_max = 0.0;
  xh0[0]=pos_i.x;
  xh0[1]=pos_i.y;
  dir_h[0]=pos_f.x-pos_i.x;	dir_h[1]=pos_f.y-pos_i.y;
  len_h=sqrt(dir_h[0]*dir_h[0] + dir_h[1]*dir_h[1]);
  dir_h[0]=dir_h[0]/len_h;	dir_h[1]=dir_h[1]/len_h;
  len_h=len_h*2;
//allocate the initial human_pos
  for (int j = num_m; j>=0; j--)
  {
    human_pos[j].x = pos_i.x;
    human_pos[j].y = pos_i.y;
    human_pos[j].z = pos_i.z;
  }
  for (int j = num_n+1; j>=0; j--)
  {
    human_vel[j].x = 0.0;
    human_vel[j].y = 0.0;
    human_vel[j].z = 0.0;
  }
  cur_vel.x = 0.01;
  cur_vel.y = 0.02;
  cur_vel.z = 0.03;
  geometry_msgs::Inertia t_x;
  geometry_msgs::Inertia t_y;
  geometry_msgs::Inertia hs_x;
  geometry_msgs::Inertia hs_y;
  ros::init(argc, argv, "arm_tracker_glove");
  ros::NodeHandle n("~");
  std::string led_name;
  if (not n.getParam("led_name", led_name))
  {
    led_name = "/LED8";
  }
  previous_time = ros::Time::now();
  ros::Rate loop_rate(freq);
  ros::Subscriber ledA_sub = n.subscribe(led_name, 1, ledACallback);
  human_pub = n.advertise<i2r_baxter_motion::Human>("/trust/human_states", 10);
  debug_pub = n.advertise<geometry_msgs::Twist>("/trust/tracker_debug", 10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

