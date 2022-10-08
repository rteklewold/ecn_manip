#include <robot_turret.h>
#include <trig_solvers.h>

// Model of Turret robot

// Any constant transform at base or end-effector
void ecn::RobotTurret::init_wMe()
{
    // Generated end-effector code
      wMe[0][0] = 1.;
      wMe[0][1] = 0;
      wMe[0][2] = 0;
      wMe[0][3] = 0;
      wMe[1][0] = 0;
      wMe[1][1] = 1.;
      wMe[1][2] = 0;
      wMe[1][3] = 0;
      wMe[2][0] = 0;
      wMe[2][1] = 0;
      wMe[2][2] = 1.;
      wMe[2][3] = 0;
      wMe[3][0] = 0;
      wMe[3][1] = 0;
      wMe[3][2] = 0;
      wMe[3][3] = 1.;
      // End of end-effector code

}

// Direct Geometry
vpHomogeneousMatrix ecn::RobotTurret::fMw(const vpColVector &q) const
{
    vpHomogeneousMatrix M;
    // Generated pose code
     const double c1 = cos(q[0]);
     const double c2 = cos(q[1]);
     const double s1 = sin(q[0]);
     const double s2 = sin(q[1]);
     M[0][0] = c1*c2;
     M[0][1] = -s1;
     M[0][2] = -s2*c1;
     M[0][3] = (-q[2] - 0.1)*s2*c1;
     M[1][0] = s1*c2;
     M[1][1] = c1;
     M[1][2] = -s1*s2;
     M[1][3] = (-q[2] - 0.1)*s1*s2;
     M[2][0] = s2;
     M[2][1] = 0;
     M[2][2] = c2;
     M[2][3] = (q[2] + 0.1)*c2 + 0.5;
     M[3][0] = 0;
     M[3][1] = 0;
     M[3][2] = 0;
     M[3][3] = 1.;
     // End of pose code


    return M;
}


// Inverse Geometry
vpColVector ecn::RobotTurret::inverseGeometry(const vpHomogeneousMatrix &fMe_des, const vpColVector &q0) const
{
    for (double q1:solveType3(-1,0,fMe_des[0][1],0,1,fMe_des[1][1])){
        for (double q2:solveType3(1,0,fMe_des[2][0],0,1,fMe_des[2][2])){
            double a=fMe_des[2][0];
            double b=fMe_des[1][1];
            double q3=-(fMe_des[0][3]/(a*b))-0.1;
            addCandidate({q1,q2,q3});
        }
    }


    return bestCandidate(q0);
}

// Wrist Jacobian
vpMatrix ecn::RobotTurret::fJw(const vpColVector &q) const
{
    vpMatrix J(6, dofs);

    // Generated Jacobian code
        const double c1 = cos(q[0]);
        const double c2 = cos(q[1]);
        const double s1 = sin(q[0]);
        const double s2 = sin(q[1]);
        J[0][0] = (q[2] + 0.1)*s1*s2;
        J[0][1] = (-q[2] - 0.1)*c1*c2;
        J[0][2] = -s2*c1;
        J[1][0] = (-q[2] - 0.1)*s2*c1;
        J[1][1] = (-q[2] - 0.1)*s1*c2;
        J[1][2] = -s1*s2;
        //J[2][0] = 0;
        J[2][1] = -(1.0*q[2] + 0.1)*s2;
        J[2][2] = c2;
        //J[3][0] = 0;
        J[3][1] = s1;
        //J[3][2] = 0;
        //J[4][0] = 0;
        J[4][1] = -c1;
        //J[4][2] = 0;
        J[5][0] = 1.;
        //J[5][1] = 0;
        //J[5][2] = 0;
        // End of Jacobian code


    return J;
}
