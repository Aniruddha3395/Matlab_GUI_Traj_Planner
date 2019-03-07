#include <iostream>
#include </usr/local/include/eigen3/Eigen/Eigen>
#include <vector>
#include <string>
#include <cmath>
#include "nlopt.hpp"
#include "stdlib.h"
#include "opt_obj.hpp"
#include "iiwa_utilities.hpp"
#include <algorithm>

namespace opt_obj
{
    // defualt nlopt fuction
    double customminfunc(const std::vector<double>& x, std::vector<double>& grad, void* data) 
    {
        // Because we wanted a Class
        // without static members, but NLOpt library does not support
        // passing methods of Classes, we use these auxilary functions.
        opt_obj *c = (opt_obj *) data;
        return c->ObjFun(x,grad);
    }

    double customconfunc(const std::vector<double>& x, std::vector<double>& grad, void* data) 
    {
        // Because we wanted a Class
        // without static members, but NLOpt library does not support
        // passing methods of Classes, we use these auxilary functions.
        opt_obj *d = (opt_obj *) data;
        return d->ConFun(x,grad);
    }

    // class constructor
    opt_obj::opt_obj(Eigen::MatrixXd Theta, Eigen::MatrixXd Point, Eigen::MatrixXd Robot_ree_T_tee, std::vector<double> _X_init, double OptH, double OptXtolRel, int Rob_version) 
    {
        //choose optimizer
        // alg_type = nlopt::LN_NEWUOA;
        // alg_type = nlopt::LN_NEWUOA_BOUND;
        // alg_type = nlopt::LD_MMA;
        // alg_type = nlopt::LD_TNEWTON_PRECOND_RESTART;
        // alg_type = nlopt::LN_BOBYQA;
        // alg_type = nlopt::LN_COBYLA;
        alg_type = nlopt::LD_SLSQP;
        // alg_type = nlopt::LD_LBFGS;
        // alg_type = nlopt::GN_ISRES;
        
        theta = Theta;
        point = Point;
        robot_ree_T_tee = Robot_ree_T_tee;
        rob_version = Rob_version;
        tolerance[0] = 0.003;
        tolerance[1] = 0.0524;
        tolerance[2] = 0.0524;
        tolerance[3] = 1;

        X_init = _X_init;
        optH = OptH;
        optXtolRel = OptXtolRel;

        // Optimization Parameters
        OptVarDim = _X_init.size();
        opt = nlopt::opt(alg_type, OptVarDim);
        OptVarlb.resize(OptVarDim);
        OptVarub.resize(OptVarDim);

        OptVarlb[0] = -168*3.14159/180; OptVarub[0] = -OptVarlb[0];
        OptVarlb[1] = -118*3.14159/180; OptVarub[1] = -OptVarlb[1];
        OptVarlb[2] = -168*3.14159/180; OptVarub[2] = -OptVarlb[2];
        OptVarlb[3] = -118*3.14159/180; OptVarub[3] = -OptVarlb[3];
        OptVarlb[4] = -168*3.14159/180; OptVarub[4] = -OptVarlb[4];
        OptVarlb[5] = -118*3.14159/180; OptVarub[5] = -OptVarlb[5];
        OptVarlb[6] = -173*3.14159/180; OptVarub[6] = -OptVarlb[6];

        opt.set_xtol_rel(optXtolRel);
        opt.set_min_objective(customminfunc, this);
        opt.add_inequality_constraint(customconfunc, this, 1e-8);
        
        opt.set_lower_bounds(OptVarlb);
        opt.set_upper_bounds(OptVarub);
        opt.set_ftol_rel(1e-8);
        opt.set_maxeval(1000);
        
        optx.resize(OptVarDim);
    }

    // class destructor
    opt_obj::~opt_obj()
    {
    }

    double opt_obj::NLConFun(const std::vector<double> &x)
    {
        // ROBOT end-effector
        Eigen::MatrixXd rob_base_T = Eigen::MatrixXd::Identity(4,4);
        Eigen::Matrix4d transf_mat;
        if (rob_version==7)
        {
            Eigen::MatrixXd FK_all = iiwa::compute_iiwa7_FK_all(x,rob_base_T);
            transf_mat = FK_all.block(32,0,4,4)*robot_ree_T_tee;
        }
        else if (rob_version==14)
        {
            Eigen::MatrixXd FK_all = iiwa::compute_iiwa14_FK_all(x,rob_base_T);
            transf_mat = FK_all.block(32,0,4,4)*robot_ree_T_tee;
        }   

        // Error Function. Do not Change
        return cos(30*3.14159/180) - transf_mat(0,2)*point(0,9) - transf_mat(1,2)*point(0,10) - transf_mat(2,2)*point(0,11);
    };

    double opt_obj::ErrFun(const std::vector<double> &x)
    {
        // ROBOT end-effector
        Eigen::MatrixXd rob_base_T = Eigen::MatrixXd::Identity(4,4);
        Eigen::Matrix4d transf_mat;
        if (rob_version==7)
        {
            Eigen::MatrixXd ee_base_all = iiwa::compute_iiwa7_FK_all(x,rob_base_T);
            transf_mat = ee_base_all.block(32,0,4,4)*robot_ree_T_tee;
        }
        else if (rob_version==14)
        {
            Eigen::MatrixXd ee_base_all = iiwa::compute_iiwa14_FK_all(x,rob_base_T);
            transf_mat = ee_base_all.block(32,0,4,4)*robot_ree_T_tee;
        }
        double tool_xyz[3] = {transf_mat(0,3),transf_mat(1,3),transf_mat(2,3)};

        // Error Function. Donot Change
        double err_orient =  1 - (point(0,6)*transf_mat(0,1) + point(0,7)*transf_mat(1,1) + point(0,8)*transf_mat(2,1));
        double err_pose[3] = {point(0,0)-tool_xyz[0],point(0,1)-tool_xyz[1],point(0,2)-tool_xyz[2]};
        return 0.5*( 0.25*err_orient + err_pose[0]*err_pose[0] + err_pose[1]*err_pose[1] + err_pose[2]*err_pose[2]); 
		// return 0.5*err_orient + 0.5*sqrt(err_pose[0]*err_pose[0] + err_pose[1]*err_pose[1] + err_pose[2]*err_pose[2]);
		// return std::max(std::fabs(err_orient), std::fabs(sqrt(err_pose[0]*err_pose[0] + err_pose[1]*err_pose[1] + err_pose[2]*err_pose[2])));
    };

    bool opt_obj::solveOPT()
    {
        solx = X_init;
        bool successFlag = false;
        try{
            
            nlopt::result result = opt.optimize(solx, solminf);
            successFlag = true;
        }
        catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
        return successFlag;
    };

    std::vector<double> opt_obj::get_solx()
    {
        return solx;
    };

    double opt_obj::get_solminf()
    {
        return solminf;
    };

    Eigen::MatrixXd opt_obj::ascent_IK()
    {
        // Attempt IK with init_guess
        bool stat = solveOPT();
        Eigen::MatrixXd ascent_IK_ret(8,1);
        if (stat)
        {
            // Verify the solution
            theta(0,0) = solx[0]; 
            theta(1,0) = solx[1];
            theta(2,0) = solx[2];
            theta(3,0) = solx[3];
            theta(4,0) = solx[4];
            theta(5,0) = solx[5];
            theta(6,0) = solx[6];
            Eigen::MatrixXd rob_base_T = Eigen::MatrixXd::Identity(4,4);
            Eigen::MatrixXd transf_mat;
            if (rob_version==7)
            {
                Eigen::MatrixXd ee_base_all = iiwa::compute_iiwa7_FK_all(theta,rob_base_T);
                transf_mat = ee_base_all.block(32,0,4,4)*robot_ree_T_tee;
            }
            else if (rob_version==14)
            {
                Eigen::MatrixXd ee_base_all = iiwa::compute_iiwa14_FK_all(theta,rob_base_T);   
                transf_mat = ee_base_all.block(32,0,4,4)*robot_ree_T_tee;
            }
            
            // Error Position
            double err_xyz = sqrt((point(0,0)-transf_mat(0,3))*(point(0,0)-transf_mat(0,3)) + (point(0,1)-transf_mat(1,3))*(point(0,1)-transf_mat(1,3)) + (point(0,2)-transf_mat(2,3))*(point(0,2)-transf_mat(2,3)));

            // Error Orientation
            double err_by = acos(point(0,6)*transf_mat(0,1) + point(0,7)*transf_mat(1,1) + point(0,8)*transf_mat(2,1));
            double err_bz = acos(point(0,9)*transf_mat(0,2) + point(0,10)*transf_mat(1,2) + point(0,11)*transf_mat(2,2));
            
            if (err_xyz > tolerance[0] || err_bz > tolerance[3] || err_by > tolerance[2])
            {
                status = 0;
            }
            else
            {
                status = 1;
            }
            ascent_IK_ret << solx[0],solx[1],solx[2],solx[3],solx[4],solx[5],solx[6],status;
        }
        else
        {
            status = 0;
            ascent_IK_ret << theta(0,0),theta(1,0),theta(2,0),theta(3,0),theta(4,0),theta(5,0),theta(6,0),status;
            std::cout << status << std::endl;
        }
        return ascent_IK_ret;
    }

    // gradient computation:
    // Forward Difference Method
    double opt_obj::ObjFun(const std::vector<double> &x, std::vector<double> &grad)
    {
        double err = ErrFun(x);
        if (!grad.empty()) {
            std::vector<double> xph = x;
            for (uint i=0; i < x.size(); ++i)
            {
                xph[i] += optH;
                grad[i] = (ErrFun(xph)-err)/optH;
                xph[i] -= optH;
            }
        }    
        return err;
    };

    double opt_obj::ConFun(const std::vector<double> &x, std::vector<double> &grad)
    {
        double err = NLConFun(x);
        if (!grad.empty()) {
            std::vector<double> xph = x;
            for (uint i=0; i < x.size(); ++i)
            {
                xph[i] += optH;
                grad[i] = (NLConFun(xph)-err)/optH;
                xph[i] -= optH;
            }
        }    
        return err;
    };
}