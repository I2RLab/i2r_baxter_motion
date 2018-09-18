/***************************************************************************
 *                                                                         *
 * Human Trust Model                                                       *
 *                                                                         *
 * Authors:                                                                *
 *   Behzad Sadrfaridpour <bsadrfa@g.clemson.edu>                          *
 *                                                                         *
 ***************************************************************************/

/**
 @file
 @author Behzad Sadrfaridpour <bsadrfa@g.clemson.edu>
 */

#include "model.h"

#include <yane/odesolve.h>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include "spline.h"
#include <fstream>
#include <string>
#include <iomanip>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <i2r_baxter_motion/Human.h>
#include <i2r_baxter_motion/TrustData.h>
#include <i2r_baxter_motion/RobotData.h>
#include <i2r_baxter_motion/TrustMsg.h>



using namespace std;

#define PI 3.141592654
#define INF 1.0E19
#define epsilon 1E-6

#define a_t 0.9
#define b_t 0.05
#define c_t 0.05
#define w1 0.85
#define w2 0.85
#define v_h_max 1500  // [mm/s]
#define Ts .10
#define num_n 4
#define num_m 4
#define N_p 30
#define N_s 15

const yane::Utils::Uuid yane::I2R::TrustModel::ID((const unsigned char *)("507df855-d1fd-4470-831d-92eaef4f71b6"));
// 	Model (yane::OdeSolve::OdeSolveFirst *ode, yane::OdeSolve::OdeConfig *odeconf, 
//	int dimension, int ctrl_dimension, int restrsize, int paramsize=0, int iparamsize=0, int networkrestrsize=0)
//	setRestrictionSize (int restrsize)
//	Function to set the number of restrictions. 
yane::I2R::TrustModel::TrustModel() : yane::Model::Model(new yane::OdeSolve::RecurseSequence(), new yane::OdeSolve::OdeConfig(), 7, 1, 3, 7)
{
	setDoubleParameter(0, 0.0);
	setDoubleParameter(1, 0.0);
	setDoubleParameter(2, 0.5);
	setDoubleParameter(3, 0.5);
	setDoubleParameter(4, 0.5);
	setDoubleParameter(5, 0.5);
	setDoubleParameter(6, 0.5);

	odeConfig()->setTolerance(1E-4, 1E-4);

	_state_lb[0] = 0.0;
	_state_ub[0] = 1E3;
	_state_lb[1] = 0.0;
	_state_ub[1] = 1E3;
	_state_lb[2] = -10.0;
	_state_ub[2] = 10.0;
	_state_lb[3] = -10.0;
	_state_ub[3] = 10.0;
	_state_lb[4] = -10.0;
	_state_ub[4] = 10.0;
	_state_lb[5] = -1.0;
	_state_ub[5] = 1.0;
	_state_lb[6] = -1.0;
	_state_ub[6] = 1.0;

	_control_lb[0] = 0.0;
	_control_ub[0] = 1.0;
	
	std::ifstream config_file("/home/baxter/ros_ws/src/i2r_baxter_motion/config.txt");
	std::string str;
	std::getline(config_file, str);
	config_file >> pos_i.x >> pos_i.y >> pos_i.z;
	config_file >> pos_f.x >> pos_f.y >> pos_f.z;
	config_file.close();

	_xm = (pos_i.x+pos_f.x)/2;
	_ym = (pos_i.y+pos_f.y)/2;
	_range = (pow(pos_i.x-pos_f.x,2) + pow(pos_i.y-pos_f.y,2))*2;
	dir_h[0]=pos_f.x-pos_i.x;	dir_h[1]=pos_f.y-pos_i.y;
	len_h=sqrt(dir_h[0]*dir_h[0] + dir_h[1]*dir_h[1]);
	dir_h[0]=dir_h[0]/len_h;	dir_h[1]=dir_h[1]/len_h;
	len_h=len_h*2;
	r_status_coef[0]=1.0;
	r_status_coef[1]=0.0;
	_grad = 1;
	_lr = 1;
	_epsilon = 0;
	_sr = 0.0;	
	_ph = 0.5;
	_pr = 0.5;
	_trust = 1.0;
	_xh.resize(6);
	_xh_dot.resize(6);
	_yh.resize(6);
	_yh_dot.resize(6);

	_xh_org.resize(6);
	_xh_dot_org.resize(6);
	_yh_org.resize(6);
	_yh_dot_org.resize(6);
	for (int j=1; j<6; j++)
	{
	  _xh[j]=pos_i.x;
	  _xh_dot[j]=0.0;
	  _yh[j]=pos_i.y;
	  _yh_dot[j]=0.0;
	}
	_sh = 0.0;
	_xi=pos_i.x;
	_yi=pos_i.y;
	_sh0 = 0;
	_dir_sgn = 1;
	_tx.resize(10);
	_ty.resize(10);
	for (int j=1; j<10; j++)
	{
	  _tx[j]=0.0;
	  _ty[j]=0.0;
	}

	_pr_data.resize(N_p);
	_pr_data_org.resize(N_p);
	_ph_data.resize(N_p);
	_ph_data_org.resize(N_p);
	_pr_future.resize(6);
	_ph_future.resize(6);

	for (int j=0; j<N_p; j++)
	{
	  _pr_data[j]=_pr;
	  _pr_data_org[j]=_pr;
	  _ph_data[j]=_ph;
	  _ph_data_org[j]=_ph;
	}
	for (int j=0; j<6; j++)
	{
	  _pr_future[j]=_pr;
	  _ph_future[j]=_ph;
	}

	std::vector<double> Xref(13), Yref(13);
	Xref[0]=0.0; Xref[1]=0.16*.5; Xref[2]=0.37*.5; Xref[3]=0.5*.5; Xref[4]=0.66*.5; Xref[5]=0.87*.5; Xref[6]=1.0*.5;
	             Xref[7]=0.5+0.16*.5; Xref[8]=0.5+0.37*.5; Xref[9]=0.5+0.5*.5; Xref[10]=0.5+0.66*.5; Xref[11]=0.5+0.87*.5; Xref[12]=1.0;
	Yref[0]=0.0; Yref[1]=0.80; Yref[2]=0.45; Yref[3]=0.0; Yref[4]=0.80; Yref[5]=0.45; Yref[6]=0.0;
	             Yref[7]=0.80; Yref[8]=0.45; Yref[9]=0.0; Yref[10]=0.80; Yref[11]=0.45; Yref[12]=0.0;
	s_ref.set_points(Xref,Yref);    // it is required that Xref is already sorted

	// ROSparameters
	q = 100; r=100; w=50; z=10; du_max = 0.2;
    	_debug = false;
	if (_debug){
		cout << "----t0 , -u_pre , ----xh , ----yh , ----vh , ----vy , --x[0] , --x[1] , --x[2] , --x[3] , --x[4] , ---eps" << endl;
	}

}

yane::I2R::TrustModel::~TrustModel()
{
	delete odeSolver();
	delete odeConfig();
}
void yane::I2R::TrustModel::odeFunction(double t, double *x, double *u, double *dx)
{
    int idx; // time step index
    idx = (int)((t-_t0)/Ts);
    if (t<=_t0){
        _pr_data = _pr_data_org;
        _ph_data = _ph_data_org;
    }

	double s_diff, u_s_diff;
	double v[2] = {0, 0};
	double xn[2];
	double pr, ph;

    // reset human kinematics
    for (int j=0; j<7; j++)
    {
      _xh[j] = _xh_org[j];	  _xh_dot[j] = _xh_dot_org[j];
      _yh[j] = _yh_org[j];	  _yh_dot[j] = _yh_dot_org[j];
    }
    xn[0] = _xh[1];
    xn[1] = _yh[1];
    for (int k=0; k<=idx; k++)
    {
        // estimate the human motion
        for (int j=1; j<num_n+1; j++)
        {
          v[0] -= _tx[j]*_xh_dot[j+1];
          v[1] -= _ty[j]*_yh_dot[j+1];
        }
        for (int j=1; j<num_m+2; j++)
        {
          v[0] += _tx[j+num_n]*_xh[j];  //vx
          v[1] += _ty[j+num_n]*_yh[j];  //vy
        }
        for (int j=0; j<=2; j++)
        {
            if (v[j]>3000){v[j]=3000;}
            else if(v[j]<-3000){v[j]=-3000;}
        }
       	xn[0]=_xh_dot[1]*Ts+_xh[1];
       	xn[1]=_yh_dot[1]*Ts+_yh[1];
        // update the human motion for next estimate
        for (int j = num_n+1; j>0; j--)
        {
            _xh[j] =_xh[j-1];
            _yh[j] =_yh[j-1];
        }
        _xh[1] = xn[0];
        _yh[1] = xn[1];
        for (int j = num_m+1; j>0; j--)
        {
            _xh_dot[j] =_xh_dot[j-1];
            _yh_dot[j] =_yh_dot[j-1];
        }
        _xh_dot[1] = v[0];
        _yh_dot[1] = v[1];
    }
    v[0] = _xh_dot[2];   // velocity at current time step
    v[1] = _yh_dot[2];   // velocity at current time step
    xn[0] = _xh[2];     // position at current time step    
    xn[1] = _yh[2];     // position at current time step    

//	S_H
    dx[0] = round((_dir_sgn*(dir_h[0]*(xn[0]-_xi) + dir_h[1]*(xn[1]-_yi))/len_h+_sh0)*100.0)/100.0;
    if (dx[0]>_sh0+2){dx[0]=_sh0+2;}
    else if(dx[0]<x[0]){dx[0]=x[0];}
//	S_R
	dx[1] = x[1]+u[0]*Ts/(2*2*_lr*_grad);  //note

//	p_H
	s_diff = x[1]-x[0];
	if (s_diff > 1.0) {
	    s_diff = 1;
	    u_s_diff = 1; 
	} else if (s_diff > 0.0) {
	    u_s_diff = 1;
	} else {
	    u_s_diff = 0;
	    s_diff = s_diff*-1;
	}
	ph = 1-(1-w1)*abs(abs(dir_h[0]*v[0]+dir_h[1]*v[1])/v_h_max-v_h_ref(x[0]))-w1*s_diff*u_s_diff;
	ph = (ph > 1.0) ? 1.0 : ph;
//	p_R
	pr = 1-(1-w2)*(u[0]-v_r_ref(x[1]))-w2*abs(s_diff);
	pr = (pr > 1.0) ? 1.0 : pr;

//	P_H and P_R average
	_ph_future[idx]=_ph;
	_pr_future[idx]=_pr;

	double sum_ph, sum_pr;
	sum_ph = 0;
	sum_pr = 0;
	for (int j=0; j<N_s-idx-1; j++)
	{
	    sum_ph += _ph_data[j];
	    sum_pr += _pr_data[j];
//	    cout << j << "/";
	}
	for (int j=0; j<=idx; j++)
	{
        sum_ph += _ph_future[j];
        sum_pr += _pr_future[j];
//        cout << j << " : " << _pr_future[j] << " , ";
    }
	dx[2] = (sum_ph)/(N_s+1);
	dx[3] = (sum_pr)/(N_s+1);

//	Trust
	dx[4] = a_t*x[4]+b_t*dx[3]+c_t*dx[2];
	dx[4] = (dx[4] > 1.0) ? 1.0 : dx[4];
	counter++;
//	additional states for tracking u
	dx[5]=x[6];
	dx[6]=u[0];

//    cout << "model-" << setw(10) <<" : "<< counter <<setw(10) << x[6] <<setw(10) <<u[0] <<endl;
//    cout << "grad and l" << setw(10) <<" : " << _grad <<setw(10) <<_lr <<endl;
    if (_debug)
    {
        i2r_baxter_motion::TrustMsg msg;
        msg.x     = dx[0];
        msg.xdot  = dx[1];
        msg.y     = dx[2];
        msg.ydot  = dx[3];
        msg.sh    = dx[0];
        msg.sr    = dx[1];
        msg.ph    = dx[2];
        msg.pr    = dx[3];
        msg.trust = dx[4];
        msg.u     = u[0];
        _debug_pub.publish(msg);
        cout <<endl<<_t0 << setw(6) << " , " << t << setw(6) << " , " << idx << setw(6) << " , " << u[0] << setw(6) << " , ";
        cout << dx[0]  << setw(6) << " , " << dx[1] << setw(6) << " , "<< dx[2] << " , " << setw(6) << dx[3] << setw(6) << " , " << dx[0] << setw(6) << " , ";
        cout << dx[1]  << setw(6) << " , " << dx[2] << setw(6) << " , "<< dx[3] << " , " << setw(6) << dx[4] << setw(6) ;
    }

}

double yane::I2R::TrustModel::continuousCostfunction(double t, double *x, double *u)
{
	double cost;
	cost = (q*(x[1]-x[0])*(x[1]-x[0]) + w*(x[4]-1)*(x[4]-1) + r*(u[0]-1)*(u[0]-1) + z*(u[0]-x[5])*(u[0]-x[5]));
	return cost;
}

void yane::I2R::TrustModel::restrictionFunction(double t, double *x, double *u, double *fx)
{
    if (du_max+_epsilon >= 1){
        fx[0] = 1;
        fx[1] = 1;
    }else{
        fx[0] = du_max+_epsilon - x[5] + u[0];
        fx[1] = du_max+_epsilon + x[5] - u[0];
    }
    if (du_max+_epsilon >= 0.6){
        fx[2] = 1;
    }else{
        fx[2] = x[4] - 0.5;
    }
}

double yane::I2R::TrustModel::discreteCostfunction(int length, int horizont, double *t, double *x, double *u)
{
    double cost;
    cost = ( q*(x[1]-x[0])*(x[1]-x[0]) + w*(x[4]-1)*(x[4]-1) + r*(u[0]-1)*(u[0]-1) + z*(u[0]-x[5])*(u[0]-x[5]));
    return 0;
//    return cost;
}

void yane::I2R::TrustModel::getObjectiveWeight(double &weight_continuousCostfunction, double &weight_discreteCostfunction)
{
	weight_continuousCostfunction = 0.9;
	weight_discreteCostfunction = 0.0;
}

void yane::I2R::TrustModel::getDefaultState(double *x)
{
	counter = 0;

	for (int j=1; j<6; j++)
	{
	  _xh[j]=pos_i.x;
	  _xh_dot[j]=pos_i.y;
	  _yh[j]=0.0;
	  _yh_dot[j]=0.0;
	}
	_sh = 0.0;
	_xi=pos_i.x;
	_yi=pos_i.y;
	_sh0 = 0;
	_dir_sgn = 1;
	for (int j=1; j<10; j++)
	{
	  _tx[j]=0.0;
	  _ty[j]=0.0;
	}
	x[0] = pos_i.x;
	x[1] = 0.0;
	x[2] = pos_i.y;
	x[3] = 0.0;
	x[0] = 0.0;
	x[1] = 0.0;
	x[2] = _ph;
	x[3] = _pr;
	x[4] = _trust;
	x[5] =  0.5;
	x[6] = 0.5;

}

void yane::I2R::TrustModel::getDefaultControl(double *u)
{
	u[0] = 0.25;
}


void yane::I2R::TrustModel::UpdateX(double *x)
{
//_xh[0] original data
//_xh[1] updated data in ode_fuction
	for (int j=0; j<7; j++)
	{
	  _xh[j] = _xh_org[j];	  _xh_dot[j] = _xh_dot_org[j];
	  _yh[j] = _yh_org[j];	  _yh_dot[j] = _yh_dot_org[j];
	}
	_pr_data = _pr_data_org;
	_ph_data = _ph_data_org;
	x[0] = _xh[0];
	x[1] = _xh_dot[0];
	x[2] = _yh[0];
	x[3] = _yh_dot[0];
	x[0] = _sh;
	x[1] = _sr;	//ok
	x[2] = _ph;
	x[3] = _pr;
	x[4] = _trust;	//ok
	x[5] = _u_pre;
	x[6] = _u_pre;
//	 print the states
	cout <<  endl <<"  Update X " << endl;
//	cout <<  "(";
//	for ( int i = 4; i < 9; i++ )
//	{
//		cout << setprecision ( 2 ) << setw ( 6 ) << x[i] << " ";
//	}
//	cout <<  ") ";
//	cout << endl;
}


void yane::I2R::TrustModel::eventBeforeMPC(int horizont, double *t, double *x, double *sdatavalues)
{
//_xh[0] original data
//_xh[1] updated data in ode_fuction
	for (int j=0; j<7; j++)
	{
	  _xh[j] = _xh_org[j];	  _xh_dot[j] = _xh_dot_org[j];
	  _yh[j] = _yh_org[j];	  _yh_dot[j] = _yh_dot_org[j];
	}
	_pr_data = _pr_data_org;
	_ph_data = _ph_data_org;
	x[0] = _sh;
	x[1] = _sr;	//ok
	x[2] = _ph;
	x[3] = _pr;
	x[4] = _trust;	//ok
	x[5] = _u_pre;
	x[6] = _u_pre;
	counter = 0;
	_t0 = *t;

//	print the data
//    cout << "----t0 , -u_pre , ----xh , ----yh , ----vh , ----vy , --x[0] , --x[1] , --x[2] , --x[3] , --x[4] , ---eps" << endl;
    cout <<endl<<_t0 << setw(6) << " , " << _u_pre << setw(6) << " , ";
    cout << _xh[1]  << setw(6) << " , " << _yh[1] << setw(6) << " , "<< _xh_dot[1] << " , " << setw(6) << _yh_dot[1] << setw(6) << " , " << x[0] << setw(6) << " , ";
    cout << x[1]  << setw(6) << " , " << x[2] << setw(6) << " , "<< x[3] << " , " << setw(6) << x[4] << " , " << setw(6) << _epsilon << setw(6); //<< endl;
//    cout << "------ , ------ , ------ , ------ , ------ , ------ , ------ , ------ , ------ , ------ , ------ , ------" << endl;
//    cout << "-----t , -----u , --cost" << endl;

}


void yane::I2R::TrustModel::updateTrustStates(double trust, vector<double> & pr_data, vector<double> & ph_data, double pr, double ph)
{
	_trust = trust;	
	_pr = pr;
	_ph = ph;
	_pr_data_org = pr_data;
	_ph_data_org = ph_data;
}

void yane::I2R::TrustModel::updateRobotStates(double sr, double status_r, double grad, double lr)
{
	_sr = sr;
	if (abs(status_r-2)<epsilon) { 	//pick up
	  r_status_coef[0]=0.0;
	  r_status_coef[1]=0.5;
	} else {			//motion
	  r_status_coef[0]=1.0;
	  r_status_coef[1]=0.0;
	}
	_grad = grad;
	_lr = lr;
}


void yane::I2R::TrustModel::updateHumanStates(double x0, double x1, double x2, double x3, double x4, 
						double xd0, double xd1, double xd2, double xd3, double xd4,
						double y0, double y1, double y2, double y3,  double y4,
						double yd0, double yd1, double yd2, double yd3, double yd4,
						double sh0, double dir_sgn, 
						double tx0, double tx1, double tx2, double tx3, double tx4, 
						double tx5, double tx6, double tx7, double tx8, 
						double ty0, double ty1, double ty2, double ty3, double ty4, 
						double ty5, double ty6, double ty7, double ty8,
						double sh, double xi, double yi)
{
	_xh_org[0]=x0;	_xh_org[1]=x0;	_xh_org[2]=x1;
	_xh_org[3]=x2;	_xh_org[4]=x3;	_xh_org[5]=x4;
	_xh_dot_org[0]=xd0;	_xh_dot_org[1]=xd0;	_xh_dot_org[2]=xd1;
	_xh_dot_org[3]=xd2;	_xh_dot_org[4]=xd3;	_xh_dot_org[5]=xd4;
	_yh_org[0]=y0;	_yh_org[1]=y0;	_yh_org[2]=y1;
	_yh_org[3]=y2;	_yh_org[4]=y3;	_yh_org[5]=y4;
	_yh_dot_org[0]=yd0;	_yh_dot_org[1]=yd0;	_yh_dot_org[2]=yd1;
	_yh_dot_org[3]=yd2;	_yh_dot_org[4]=yd3;	_yh_dot_org[5]=yd4;
	_sh = sh;
	_xi = xi;
	_yi = yi;
	_sh0 = sh0;
	_dir_sgn = dir_sgn;
	_tx[1]=tx0;	_tx[2]=tx1;	_tx[3]=tx2;
	_tx[4]=tx3;	_tx[5]=tx4;	_tx[6]=tx5;
	_tx[7]=tx6;	_tx[8]=tx7;	_tx[9]=tx8;
	_ty[1]=ty0;	_ty[2]=ty1;	_ty[3]=ty2;
	_ty[4]=ty3;	_ty[5]=ty4;	_ty[6]=ty5;
	_ty[7]=ty6;	_ty[8]=ty7;	_ty[9]=ty8;
}

void yane::I2R::TrustModel::updateControlStates(double &u)
{
    _u_pre = u;
}

void yane::I2R::TrustModel::getStates(i2r_baxter_motion::TrustMsg &msg)
{
    msg.x     = _xh[0];
    msg.xdot  = _xh_dot[0];
    msg.y     = _yh[0];
    msg.ydot  = _yh_dot[0];
    msg.sh    = _sh;
    msg.sr    = _sr;
    msg.ph    = _ph;
    msg.pr    = _pr;
    msg.trust = _trust;
    msg.u     = _u_pre;

    if (_debug){
        cout << endl;
        cout << "       t0,         t,   idx,        u, x[0]       , x[1]    , x[2]     , x[3]      ,";
        cout << fixed <<setw(6)<<setprecision(2)<< msg.sh    <<",";
        cout << fixed <<setw(6)<<setprecision(2)<< msg.sr    <<",";
        cout << fixed <<setw(6)<<setprecision(2)<< msg.ph    <<",";
        cout << fixed <<setw(6)<<setprecision(2)<< msg.pr    <<",";
        cout << fixed <<setw(6)<<setprecision(2)<< msg.trust <<",";
        cout << fixed <<setw(6)<<setprecision(2)<< msg.u     << endl;
    }
}

void yane::I2R::TrustModel::setParams(double &qn, double &rn, double &wn, double &zn, double &dun, const int &horizon)
{
    q = qn;
    r = rn;
    w = wn;
    z = zn;
    du_max = dun;
    cout << "ROS parameters\n";
    cout << q << " " << r << " " << w << " " << z << " " << du_max << endl;
    _horizon = horizon;
}
void yane::I2R::TrustModel::setEpsilon(double eps)
{
    _epsilon = eps;
}
void yane::I2R::TrustModel::setNodeHandler(ros::NodeHandle &node)
{
    _node = node;
    _debug_pub = _node.advertise<i2r_baxter_motion::TrustMsg>("/trust/debug_model",1);
    _debug = true;

}

const yane::Utils::Uuid & yane::I2R::TrustModel::id()
{
	return ID;
}

double yane::I2R::TrustModel::v_r_ref(double s_r)
{
	return s_ref(fmod(s_r, 1));
}

double yane::I2R::TrustModel::v_h_ref(double s_h)
{
	return s_ref(fmod(s_h, 1));
}
