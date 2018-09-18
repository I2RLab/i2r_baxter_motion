/***************************************************************************
 *                                                                         *
 * Human Trust Model                                                       *
 *                                                                         *
 * Authors:                                                                *
 *   Behzad Sadr <bsadrfa@g.clemson.edu>                                   *
 *                                                                         *
 ***************************************************************************/
/**
 @file
 @author Behzad Sadr <bsadrfa@g.clemson.edu>
 */
#ifndef TrustModel_H
#define TrustModel_H
#include <yane/model.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <iostream>
#include "spline.h"
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <i2r_baxter_motion/TrustMsg.h>


namespace yane
{
	namespace I2R
	{

        class TrustModel : public yane::Model::Model
		{
			public:
                TrustModel ( );
                ~TrustModel ( );
				virtual void odeFunction ( double t, double * x, double * u,
				        double * dx );
				virtual double continuousCostfunction ( double t, double * x,
				        double * u );
				virtual void restrictionFunction ( double t, double *x,
				        double * u, double * fx );
				virtual double discreteCostfunction ( int length, int horizont,
				        double * t, double * x, double * u );
				virtual void getObjectiveWeight (
				        double & weight_continuousCostfunction,
				        double & weight_discreteCostfunction );
				virtual void getDefaultState ( double * x );
				virtual void getDefaultControl ( double * u );
				virtual void eventBeforeMPC ( int horizont, double * timesteps,
				        double * x, double * sdatavalues );
				virtual const yane::Utils::Uuid & id ( );
				static const yane::Utils::Uuid ID;

				void updateRobotStates(double sr, double status_r, double grad, double lr);
				void updateHumanStates(double x0, double x1, double x2, double x3,  double x4,
						double xd0, double xd1, double xd2, double xd3, double xd4,
						double y0, double y1, double y2, double y3,  double y4,
						double yd0, double yd1, double yd2, double yd3, double yd4,
						double sh0, double dir_sgn, 
						double tx0, double tx1, double tx2, double tx3, double tx4, 
						double tx5, double tx6, double tx7, double tx8, 
						double ty0, double ty1, double ty2, double ty3, double ty4, 
						double ty5, double ty6, double ty7, double ty8,
						double sh, double xi, double yi);
//				void updateTrustStates(double trust, double pr, double ph);
                void updateTrustStates(double trust, std::vector<double> & pr_data, std::vector<double> & ph_data, double pr, double ph);
                void updateControlStates(double &u);
                void getStates(i2r_baxter_motion::TrustMsg &msg);
                void setParams(double &q, double &r, double &w, double &z, double &du, const int &horizon);
                void setEpsilon(double eps);
                void UpdateX(double *x);
                void setNodeHandler(ros::NodeHandle &node);
			private:
				double v_r_ref(double s_r);
				double v_h_ref(double s_h);
				double dir_h[2], len_h;
				double _sh0;
				double r_status_coef[2]; 
				std::vector<double> _xh, _xh_dot, _yh, _yh_dot;
				std::vector<double> _xh_org, _xh_dot_org, _yh_org, _yh_dot_org;
				std::vector<double> _tx, _ty;
				std::vector<double> _pr_future, _ph_future;
				std::vector<double> _pr_data, _ph_data;
				std::vector<double> _pr_data_org, _ph_data_org;
				double _sh, _sr, _ph, _pr, _trust, _status_h;
				double _grad, _lr;
				double _xi, _yi;
				double _xm, _ym, _range;
				double _dir_sgn;
				int counter;
				geometry_msgs::Vector3 pos_i,pos_f;
				tk::spline s_ref;
				double _u_pre;
                double q, r, w, z, du_max;
                double _epsilon;
              	ros::NodeHandle _node;
            	ros::Publisher _debug_pub;
            	bool _debug = false;
            	int _horizon;
            	double _t0;

		};
	}
}

#endif