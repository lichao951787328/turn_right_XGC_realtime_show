#ifndef _TYPE_H_
#define _TYPE_H_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
using namespace std;
struct foot_data
{
    double foot_front_length;
    double foot_end_length;
    double threld;
    double fit_length;
    double min_length;
    double max_length;
};

struct footstep
{
    bool is_left;
    double x;
    double y;
    double z;
    double theta;
    footstep() {}
    footstep(bool is_left_, double x_, double y_, double theta_)
    {
        is_left = is_left_; x = x_; y = y_; theta = theta_; z = 0.0;
    }
    footstep& operator =(const footstep &a)
	{
		is_left = a.is_left;
        x = a.x;
        y = a.y;
        z = a.z;
        theta = a.theta;
		return *this;
	}
};

struct taijie
{
    double start = 0.0;
    double end = 0.0;
    double height = 0.0;
    friend bool operator < (const struct taijie & n1, const struct taijie & n2)
    {
        return n1.start > n2.start;//小顶堆
    }
};

struct rectPlane
{
    Eigen::Vector3d corners[4];
    Eigen::Vector3d center;
    Eigen::Vector3d direct;
    double width;
    double width_2d;
    friend bool operator < (const struct rectPlane & n1, const struct rectPlane & n2)
    {
        return n1.center.norm() > n2.center.norm();
    }
    rectPlane(vector<Eigen::Vector3d> & edgepoints)
    {
        if (edgepoints.size() != 4)
        {
            cout<<"construct rect plane failed."<<endl;
        }
        else
        {
            center = Eigen::Vector3d::Zero();
            for (size_t i = 0; i < 4; i++)
            {
                corners[i] = edgepoints.at(i);
                center += edgepoints.at(i);
            }
            center /= 4;
            double minl = (corners[1] - corners[0]).norm();
            Eigen::Vector3d minP = corners[1];
            for (size_t i = 2; i < 4; i++)
            {
                if ((corners[i] - corners[0]).norm() < minl)
                {
                    minl = (corners[i] - corners[0]).norm();
                    minP = corners[i];
                }
            }
            // 这个是带倾斜角度的真实宽度
            width = (minP - corners[0]).norm();
            width_2d = (minP - corners[0]).head(2).norm();
            direct = (minP - corners[0]).normalized();
            if (direct(0) < 0)
            {
                direct = -direct;
            }
        }
        
    }
};

struct foot_cor
{
    bool is_left;
    double cor[3];
};
/*
class time_self
{
public:
    uint32_t secs;// 到当前时刻的秒数
    uint32_t usecs;//微秒
    time_self(){}
    time_self(uint32_t secs_, uint32_t usecs_)
    {
        secs = secs_;
        usecs = usecs_;
    }
    time_self(ros::Time time_)
    {
        secs = time_.sec;
        usecs = time_.nsec/1e3;
    }
    void refresh(ros::Time time_)
    {
        setSecs(time_.sec);
        setUsecs(time_.nsec/1e3);
    }
    void setSecs(uint32_t secs_)
    {
        secs = secs_;
    }
    void setUsecs(uint32_t usecs_)
    {
        usecs = usecs_;
    }
    double operator-(const time_self& other)
    {
        // cout<<"this: secs "<<secs<<" nsecs "<<usecs<<endl;
        // cout<<"other: secs "<<other.secs<<" nsecs "<<other.usecs<<endl;
        double s_r;
        if (usecs - other.usecs > 0)
        {
            int ns = usecs - other.usecs;
            s_r = (double)(((double)(ns))/1e6);
        }
        else
        {
            secs --;
            int ns = (usecs + 1e6 - other.usecs);
            s_r = (double)(((double)(ns))/1e6);
        }
        // cout<<"s_r = "<<s_r<<endl;
        // cout<<"secs - other.secs = "<<secs - other.secs<<endl;
        // cout<<"(int)(secs - other.secs) = "<<(int)(secs - other.secs)<<endl;
        if (secs >= other.secs)
        {
            s_r += (int)(secs - other.secs);
        }
        else
        {
            s_r += (int)(secs - other.secs);
        }
        // cout<<"s_r = "<<s_r<<endl;
        return s_r;
    }

    time_self& operator =(const time_self& other)//赋值运算符
    {
        if (this != &other)
        {
            secs = other.secs;
            usecs = other.usecs;
        }
        return *this;
    }
};

class base_state
{
private:
    time_self time_stamp;
    // Eigen::VectorXd x(10);
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    std::vector<sensor_msgs::Imu> imuData;
    Eigen::Matrix<double,3,3> Matrix_C2B;
    Eigen::Vector3d g_;
public:
    base_state(){}
    base_state(time_self time, std::vector<sensor_msgs::Imu> & imuData_, Eigen::Matrix<double,3,3> Matrix_C2B_)
    {
        time_stamp = time;
        imuData = imuData_;
        Matrix_C2B = Matrix_C2B_;
        g_<<0.0, 0.0, -9.8;
    }
    void inline initial(double x, double y, double z, 
                        double v_x, double v_y, double v_z, 
                        double roll, double pitch, double yaw)
    {
        Eigen::Quaterniond quaternion;
        quaternion = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) * 
                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
                        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitX());
        setXq(quaternion);
        Eigen::Vector3d v(v_x, v_y, v_z);
        setXv(v);
        Eigen::Vector3d p(x, y, z);
        setXp(p);
    }
    void inline setXq(Eigen::Quaterniond & q_)
    {
        q = q_;
    }
    void inline setXv(Eigen::Vector3d & v_)
    {
        v = v_;
    }
    void inline setXp(Eigen::Vector3d & p_)
    {
        p = p_;
    }
    auto inline getQuaternion()
    {
        return q;
    }
    auto inline getVelocity()
    {
        return v;
    }
    auto inline getPosition()
    {
        return p;
    }
    void update(u_int32_t seq)
    {
        std::ofstream data;
        std::string path = "/home/humanoid/catkin_bhr_ws/src/mutiThread_perception/" + std::to_string(seq) + ".data";
        data.open(path);
        for(auto & iter_imudata : imuData)
        {
            data<<p(0)<<" "<<p(1)<<" "<<p(2)<<endl;
            // sensor_msgs::Imu msg = imuData.front();
            // imuData.erase(imuData.begin());
            // imuData.pop();
            Eigen::Matrix3d R = q.toRotationMatrix();
            Eigen::Matrix<double,4,4> S = Eigen::Matrix<double,4,4>::Zero();
            Eigen::Vector3d w_C(iter_imudata.angular_velocity.x, iter_imudata.angular_velocity.y, iter_imudata.angular_velocity.z);
            Eigen::Vector3d w_W = R * Matrix_C2B * w_C;
            S << 0   ,-w_W[0] ,-w_W[1] ,-w_W[2],
            w_W[0], 0    , w_W[2] ,-w_W[1],
            w_W[1],-w_W[2] , 0    , w_W[0],
            w_W[2], w_W[1] ,-w_W[0] , 0   ;
            time_self imu_time(iter_imudata.header.stamp);
            double dt = imu_time - time_stamp;
            time_stamp = imu_time;
            Eigen::Matrix<double,10,10> A = Eigen::Matrix<double,10,10>::Zero();
            A.block<4,4>(0,0) = 0.5 * dt * S + Eigen::Matrix<double,4,4>::Identity();
            A.block<3,3>(4,4) = Eigen::Matrix<double,3,3>::Identity();
            A.block<3,3>(7,4) = dt*Eigen::Matrix<double,3,3>::Identity();
            A.block<3,3>(7,7) = Eigen::Matrix<double,3,3>::Identity();

            Eigen::Vector3d a(iter_imudata.linear_acceleration.x, iter_imudata.linear_acceleration.y, iter_imudata.linear_acceleration.z);
            Eigen::Matrix<double,10,10> B = Eigen::Matrix<double,10,10>::Zero();
            B.block<3,3>(4,4) = dt*Eigen::Matrix<double,3,3>::Identity();
            B.block<3,3>(7,7) = 0.5*dt*dt*Eigen::Matrix<double,3,3>::Identity();
            Eigen::Matrix<double,10,1> U = Eigen::Matrix<double,10,1>::Zero();
            U.block<3,1>(4,0) = R * Matrix_C2B * a + g_;
            U.block<3,1>(7,0) = R * Matrix_C2B * a + g_;

            Eigen::Matrix<double,10,1> X = Eigen::Matrix<double,10,1>::Zero();
            // Eigen::Vector4d q_q(q.w, q.x, q.y, q.z);
            // X.block<4,1>(0,0) = q_q;
            X(0,0) = q.w(); X(1,0) = q.x(); X(2,0) = q.y(); X(3,0) = q.z();
            X.block<3,1>(4,0) = v;
            X.block<3,1>(7,0) = p;
            Eigen::VectorXd X_new = A*X+B*U;
            Eigen::Quaterniond q_(X_new(0,0), X_new(1,0), X_new(2,0), X_new(3,0));
            setXq(q_);
            Eigen::Vector3d v_(X_new(4,0), X_new(5,0), X_new(6,0));
            setXv(v_);
            Eigen::Vector3d p_(X_new(7,0), X_new(8,0), X_new(9,0));
            setXp(p_);
        }
    }
    auto inline getMatrix4d()
    {
        Eigen::Matrix4d res;
        res.setIdentity();
        res.block<3,3>(0,0) = q.toRotationMatrix();
        res.block<3,1>(0,3) = p;
        return res;
    }

};
*/

#endif