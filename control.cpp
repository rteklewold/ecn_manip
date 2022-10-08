#include <robot_init.h>

using namespace std;

int main(int argc, char ** argv)
{
    // initialize robot class and get DOF's
    auto robot = ecn::initRobot(argc, argv, 100);
    const unsigned n = robot->getDofs();

    // robot properties
    const vpColVector vMax = robot->vMax();
    const vpColVector aMax = robot->aMax();

    // main variables
    vpColVector q(n);               // joint position
    vpPoseVector p;                 // operational pose
    vpColVector qCommand(n);        // joint position setpoint
    vpColVector vCommand(n);        // joint velocity setpoint

    vpMatrix J;
    vpHomogeneousMatrix M;          // current pose
    vpHomogeneousMatrix M0, Md, Mi; // previous, final and current desired poses
    vpPoseVector pd;                // desired pose
    vpColVector v;                  // desired operational velocity

    // TODO declare other variables if needed
    vpColVector q0(n), qf(n);        // joint position setpoint for initial and final poses
    double t, t0, tf;

    // main control loop
    while(robot->ok())
    {
        // current time
        t = robot->time();

        // update desired pose if has changed
        if(robot->newRef())
        {
            Md = robot->Md();
            M0 = robot->M0();
            pd.buildFrom(Md);
            t0 = t;
        }


        // get current joint positions
        q = robot->jointPosition();
        cout << "Current joint position : " << q.t() << endl;

        // Direct Geometry for end-effector
        M = robot->fMe(q);  // matrix form
        p.buildFrom(M);     // translation + angle-axis form

        if(robot->mode() == ecn::Robot::MODE_POSITION_MANUAL)
        {
            // just check the Direct Geometric Model
            // TODO: fill the fMw function
            robot->checkPose(M);
        }


        else if(robot->mode() == ecn::Robot::MODE_VELOCITY_MANUAL)
        {
            // follow a given operational velocity
            v = robot->vw();

            // TODO: fill the fJw function
            vpMatrix R(6,6);
            vpColVector fVe;
            double r1=0;
            double c1=0;
            double r2=3;
            double c2=3;

            ecn::putAt(R,M.getRotationMatrix(),r1,c1);
            ecn::putAt(R,M.getRotationMatrix(),r2,c2);
            fVe=R*v;
           // TODO: compute vCommand
            vCommand=robot->fJe(q).pseudoInverse()*fVe;
            robot->setJointVelocity(vCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_DIRECT_P2P)
        {
            // find the Inverse Geometry to reach Md
            // TODO: fill the inverseGeometry function
            qf = robot->inverseGeometry(Md, q);
            robot->setJointPosition(qf);
        }




        else if(robot->mode() == ecn::Robot::MODE_INTERP_P2P)
        {
            // reach Md with interpolated joint trajectory
            // use q0 (initial position), qf (final), aMax and vMax

            // if reference has changed, compute new tf
            if(robot->newRef())
            {
                q0 = robot->inverseGeometry(M0, q);
                qf = robot->inverseGeometry(Md, q);
                tf=0;
                for (unsigned int i=0; i<n;i++){
                    double tfa=3*std::abs(qf[i]-q0[i])/2*vMax[i];
                    double tfb=sqrt(6*std::abs(qf[i]-q0[i])/aMax[i]);
                    double tfmax;
                    if (tfa>tfb){
                         tfmax=tfa;
                    }
                    else {
                        tfmax=tfb;
                    }
                    if (tfmax>tf)
                    {
                        tf = tfmax;
                    }
                }



            }

            // TODO: compute qCommand from q0, qf, t, t0 and tf
            auto P=3*pow((t-t0)/tf,2)-2*pow((t-t0)/tf,3);
            qCommand=q0+P*(qf-q0);

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_STRAIGHT_LINE_P2P)
        {
            // go from M0 to Md in 1 sec
            tf = 1;
            double alpha;

            // TODO: compute qCommand from M0, Md, t, t0 and tf
            if (t-t0<tf) {
                  alpha=(t-t0)/tf;
            }
            else {
                alpha=1;
            }
                  Mi=robot->intermediaryPose(M0,Md,alpha);


            qCommand=robot->inverseGeometry(Mi, q);
            // use robot->intermediaryPose to build poses between M0 and Md

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_VELOCITY_P2P)
        {
            // go to Md using operational velocity
           vpHomogeneousMatrix e_Me;
           vpRotationMatrix fRe_des=Md.getRotationMatrix();
           vpColVector vf(6);
           e_Me=Md.inverse()*M;
           vpPoseVector Pe;
           Pe.buildFrom(e_Me);
           auto theta_u=Pe.getThetaUVector();
           auto translation=Pe.getTranslationVector();
           auto v=-robot->lambda()*fRe_des*translation;
           auto w=-robot->lambda()*fRe_des*theta_u.getU()*theta_u.getTheta();

           for (int i=0;i<6;i++)
           {
               if (i<3)
                   vf[i]=v[i];
           else {
               vf[i]=w[i-3];
               }
           }

            // TODO: compute joint velocity command

           vCommand=robot->fJe(q).pseudoInverse()*vf;

            robot->setJointVelocity(vCommand);
        }


    }
}
