#include "lab_auve/uavsim.h"
#include <eigen3/Eigen/Dense>
#include "matplotlibcpp.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// Plot stuff
namespace plt = matplotlibcpp;
std::vector<double> plot_data;
std::vector<double> plot_data2;
std::vector<double> plot_data3;
std::vector<double> plot_data4;

//Usefull variables
double la = 0.17;
double kt = 5.5e-6;
double kd = 3.299e-7;
double drone_mass = 1.0;
double gravity = 9.81;



Matrix4d mixer;
Matrix4d inv_mixer;

// Function generates a circular trajectory in 3D space at a fixed height.
void generateCircularTrajectory(double time, Vector3d& desired_position, Vector3d& desired_velocity)
{

    double radius = 1.0;  // Adjust the radius of the circle as needed
    double omega = 0.5;   // Adjust the angular velocity as needed

    desired_position[0] = radius * cos(omega * time);
    desired_position[1] = radius * sin(omega * time);
    desired_position[2] = 2.0;  // Fixed height 

    desired_velocity[0] = -radius * omega * sin(omega * time);
    desired_velocity[1] = radius * omega * cos(omega * time);
    desired_velocity[2] = 0.0;
}

// Function to test mixer
void test_mixer(Vector4d& wrench, const double& value) {
    // Reset all values in the wrench vector to 0
    wrench.setZero();

    // Set value as Force thrust
    wrench[0] = value;
}



// Function calculates the control torque for orientation control using a PD controller.
Vector3d orientationControl(const Quaterniond& desired_orientation, const Quaterniond& current_orientation, const Vector3d current_velocity)
{
    Quaterniond qd = desired_orientation.normalized();
    Quaterniond q = current_orientation.normalized();
    Quaterniond qe = q.inverse()*qd;
    qe = qe.normalized();
    double qr = qe.w();
    Vector3d qi = qe.vec();
    double kpx = 2.0;
    double kpy = 2.0;
    double kpz = 0.1;
    double kdx = 0.4;
    double kdy = 0.4;
    double kdz = 0.01;

    Matrix3d KP;
    KP << kpx, 0, 0,
                   0, kpy ,0,
                    0,0, kpz;
    Matrix3d KD;
    KD << kdx, 0, 0,
                   0, kdy ,0,
                    0,0, kdz;
    Vector3d tau = -KD*current_velocity + KP * sgn(qr)*qi;
    return tau;
}

// Function calculates the control quaternion and scalar force for position control using a PD controller.
Quaterniond positionControl(const Vector3d& desired_position, const Vector3d& desired_velocity,
                             const Vector3d& current_position, const Vector3d& current_velocity,
                             double& scalar_force)
{
    double kpx = 2.0;
    double kpy = 2.0;
    double kpz = 2.0;
    double kdx = 3.0;
    double kdy = 3.0;
    double kdz = 3.0;

    Matrix3d KP;
    KP << kpx, 0, 0,
                   0, kpy ,0,
                    0,0, kpz;
    Matrix3d KD;
    KD << kdx, 0, 0,
                   0, kdy ,0,
                    0,0, kdz;

    Vector3d fg {0,0,drone_mass*gravity};
    Vector3d Force = KD*(desired_velocity-current_velocity) + KP* (desired_position-current_position) + fg;
    //Vector3d Force = KD*(desired_velocity) + KP* (desired_position) + fg;             //Forward Controller

    scalar_force = Force.norm();
    Vector3d x,y,z;
    z = Force.normalized();
    y = {0,1,0};
    x = y.cross(z);
    x = x.normalized();
    y = z.cross(x);
    y = y.normalized();
    Matrix3d R;
    R << x[0],y[0],z[0],
            x[1],y[1],z[1],
            x[2],y[2],z[2];

    Quaterniond q = Quaterniond(R);
    return q;
}


int main()
{
    plt::figure();
    int loop_count=0;

    UAVSim uav = UAVSim();
    double prev_clock = uav.getClock();

    sleep(1); // necessary fr proper init before start control

    mixer << kt, kt, kt, kt,
            -la*kt, la*kt, 0, 0,
            0, 0, -la*kt, la*kt,
            -kd, -kd, kd, kd;

    inv_mixer = mixer.inverse();

    Vector3d desired_position = {0,0,2};
    Vector3d desired_velocity = {0,0,0};
    Quaterniond desired_orientation;

    Vector3d current_position;
    //bool method = false;            //false = Desired position
    bool method = true;              //true = Circular Trajectory
    Vector3d current_velocity;
    Vector3d angular_velocity;
    Quaterniond current_orientation;
    double scalar_force;
    Vector3d tau;
    Vector4d w2;
    Vector4d wrench;
    while (uav.isReadyToGo() and loop_count<5000) {
        if ((uav.getClock() - prev_clock) > 0.005) // Loop at 200Hz
        {
            prev_clock = uav.getClock();
            Vector4d desired_motors;
            Vector4d unsaturated_motors;
            // Motor speed should be between 0 and 1466 rpm
            current_position = uav.getPosition();
            current_velocity = uav.getLinearVelocity();
            angular_velocity = uav.getAngularVelocity();
            current_orientation = uav.getOrientation();
            if (method == true){
                generateCircularTrajectory(prev_clock, desired_position, desired_velocity);
            }
            desired_orientation = positionControl(desired_position,desired_velocity,current_position, current_velocity,scalar_force);

            tau = orientationControl(desired_orientation,current_orientation,angular_velocity);
            wrench = {scalar_force, tau[0], tau[1], tau[2]};

//            double test_thrust = 12;              // For mixer_matrix test
//            test_mixer(wrench,test_thrust);
            w2 = inv_mixer*wrench;


            desired_motors = w2.cwiseSqrt();
            unsaturated_motors = desired_motors;
            for(int i = 0;i<4;i++){
                if(desired_motors[i]<0){
                    desired_motors[i] = 0;
                }
                else if (desired_motors[i]>1466) {
                    desired_motors[i] = 1466;
                }
            }
            uav.setMotors(desired_motors);
            loop_count = 0;
            // A basic example of a scope plot
            // Indeed you need to adapt this to your needs
            // It uses a C++ library based on python matplotlib with similar calls
//            plt::clf();
//            // You can create as many plots as you want
//            plot_data.push_back(desired_motors[0]);
//            plot_data2.push_back(desired_motors[1]);
//            plot_data3.push_back(desired_motors[2]);
//            plot_data4.push_back(desired_motors[3]);
//            plt::plot(plot_data);
//            plt::plot(plot_data2);
//            plt::plot(plot_data3);
//            plt::plot(plot_data4);

//            // Add title and labels
//            plt::title("Motors");
//            plt::xlabel("Time (iterations)");
//            plt::ylabel("Position");


//            plt::pause(0.0001);
        }
        usleep(1);
        loop_count ++;
    }

}


