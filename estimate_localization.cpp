#include <ros/ros.h>
#include <iostream>
#include <localization/estimate_localization.h>
#include <sensor_msgs/NavSatStatus.h>

#include "proj.cpp"

namespace localization
{

namespace gnss_solution_status
{
enum status
{
    FIX = 1,
    FLOAT = 2,
    SINGLE = 5,
};
};

EstimateLocalization::EstimateLocalization()
{
    setRosParam();
    gnss_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>(gnss_topic_, 1, &EstimateLocalization::gnssCallback, this);
    doppler_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/tcpvel", 1, &EstimateLocalization::dopplerCallback, this);
    encoder_sub_ = nh_.subscribe<nav_msgs::Odometry>(encoder_topic_, 1, &EstimateLocalization::encoderCallback, this);
    imu_sub_ = nh_.subscribe<nav_msgs::Odometry>(imu_topic_, 1, &EstimateLocalization::imuCallback, this);
    position_pub_ = nh_.advertise<nav_msgs::Odometry>(publish_topic_, 1);
    odom_.header.frame_id = parent_frame_;
    odom_.child_frame_id = child_frame_;
    is_update_ = false;
    setInitCovariance();
    pre_gnss_status_ = 0;
    gnss_status_ = 0;
    start_time_ = std::chrono::system_clock::now();
}

void EstimateLocalization::setInitCovariance()
{
    Q_(0, 0) = 0.1 * 0.1; // 入力vの分散 [m/s]^2
    Q_(1, 1) = 0.1 * 0.1; // 入力wの分散 [rad/s]^2

    R_fix_ = Eigen::Matrix2d::Zero(2, 2);
    R_fix_(0, 0) = 0.1 * 0.1; // 観測fix解xの分散
    R_fix_(1, 1) = 0.1 * 0.1; // 観測fix解yの分散

    R_float_ = Eigen::Matrix2d::Zero(2, 2);
    R_float_(0, 0) = 2.0 * 2.0; // 観測float解xの分散
    R_float_(1, 1) = 2.0 * 2.0; // 観測float解yの分散

    z_(0, 0) = 0.0; // 観測値の初期値
    z_(1, 0) = 0.0;

    u_(0, 0) = 0.0; // 入力の初期値
    u_(1, 0) = 0.0;

    X_ = Eigen::MatrixXd::Zero(3, 1); // 状態量初期値
    X_(0, 0) = 0.0;                   //-10148.8;
    X_(1, 0) = 0.0;                   //-32730;

    P_ = Eigen::MatrixXd::Zero(3, 3); // 状態量分散初期値
    P_(0, 0) = 600.0;
    P_(1, 1) = 600.0;
    P_(2, 2) = M_PI_2 * M_PI_2;

    F_ = Eigen::Matrix3d::Zero(3, 3);
    G_ = Eigen::MatrixXd::Zero(3, 2);
    H_ = Eigen::MatrixXd::Zero(2, 3);
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;

    Q_bias_(0, 0) = 0.05 * 0.05; // 入力vbiasの分散 [m/s]^2
    Q_bias_(1, 1) = 0.05 * 0.05; // 入力wbiasの分散 [rad/s]^2

    R_bias_(0, 0) = 0.001 * 0.001; // 観測vbiasの分散
    R_bias_(1, 1) = 0.1 * 0.1;     // 観測wbiasの分散

    z_bias_(0, 0) = 0.0; // 観測値の初期値
    z_bias_(1, 0) = 0.0;

    X_bias_ = Eigen::MatrixXd::Zero(2, 1); // 状態量初期値

    P_bias_ = Eigen::MatrixXd::Zero(2, 2); // 状態量分散初期値
    P_bias_(0, 0) = 60.0;
    P_bias_(1, 1) = 600.0;

    F_bias_ = Eigen::Matrix2d::Identity(2, 2);
    H_bias_ = Eigen::Matrix2d::Identity(2, 2);
    ROS_INFO("init covariance");
}

void EstimateLocalization::setRosParam()
{
    nh_.param<double>("/estimate_localization/inspection_distance", inspection_distance_, 1.0);
    nh_.param<std::string>("/estimate_localization/gnss_topic", gnss_topic_, "/tcpfix");
    nh_.param<std::string>("/estimate_localization/imu_topic", imu_topic_, "/fog");
    nh_.param<std::string>("/estimate_localization/encoder_topic", encoder_topic_, "/ypspur_ros/odom");
    nh_.param<std::string>("/estimate_localization/publish_topic", publish_topic_, "/odom");
    nh_.param<std::string>("/estimate_localization/parent_frame", parent_frame_, "/world");
    nh_.param<std::string>("/estimate_localization/child_frame", child_frame_, "/base_link");
    nh_.param<std::string>("/estimate_localization/gps_frame", gnss_frame_, "/gps_link");
    nh_.param<double>("/estimate_localization/rate", rate_, 60.0);
}

tf::StampedTransform EstimateLocalization::getTransform(const std::string parent, const std::string child)
{
    tf::StampedTransform transform;
    try
    {
        listener_.lookupTransform(parent, child, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return transform;
}

void EstimateLocalization::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr &position)
{
    std::tuple<double, double> xy = latlon_to_xy::proj(position->latitude, position->longitude);
    tf::StampedTransform world_to_base = getTransform(parent_frame_, child_frame_);
    tf::StampedTransform world_to_gnss = getTransform(parent_frame_, gnss_frame_);
    // std::cout << "-------------" << std::endl;
    // std::cout << world_to_base.getOrigin().x() - world_to_gnss.getOrigin().x() << std::endl;
    // std::cout << world_to_base.getOrigin().y() - world_to_gnss.getOrigin().y() << std::endl;
    double gnss_to_base_x = world_to_base.getOrigin().x() - world_to_gnss.getOrigin().x();
    double gnss_to_base_y = world_to_base.getOrigin().y() - world_to_gnss.getOrigin().y();
    z_(0, 0) = std::get<0>(xy) + gnss_to_base_x;
    z_(1, 0) = std::get<1>(xy) + gnss_to_base_y;
    // z_(0, 0) = position->pose.pose.position.x;
    // z_(1, 0) = position->pose.pose.position.y;

    pre_gnss_status_ = gnss_status_;
    //    gnss_status_ = gnss_solution_status::status::FIX;
    if (position->status.status == gnss_solution_status::status::FIX)
    {
        gnss_status_ = gnss_solution_status::status::FIX;
    }
    else if (position->status.status == gnss_solution_status::status::FLOAT)
    {
        gnss_status_ = gnss_solution_status::status::FLOAT;
    }
    else
    {
        gnss_status_ = gnss_solution_status::status::SINGLE;
    }
    is_update_ = true;
}

void EstimateLocalization::dopplerCallback(const geometry_msgs::TwistStamped::ConstPtr &velocity)
{
	
}

void EstimateLocalization::encoderCallback(const nav_msgs::Odometry::ConstPtr &enc)
{
    u_(0, 0) = enc->twist.twist.linear.x;
}

void EstimateLocalization::imuCallback(const nav_msgs::Odometry::ConstPtr &imu)
{
    u_(1, 0) = imu->twist.twist.angular.z;
}

void EstimateLocalization::extentedKalmanFilter()
{
    std::cout << "-------------" << std::endl;
    std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_).count() * std::pow(10, -6);
    start_time_ = end_time;
    Eigen::Matrix<double, 3, 1> X_past = X_;

    setJacobian(dt, X_past);
    prediction(dt, X_past);
    bool is_use_solution = mesurementInspection(dt,X_past);
    if (is_use_solution == true)
    {
        update();
    }
    std::cout << "z:" << std::endl;
    std::cout << std::setprecision(10) << z_ << std::endl;
    std::cout << "status: " << gnss_status_ << std::endl;
    std::cout << "X:" << std::endl;
    std::cout << std::setprecision(10) << X_ << std::endl;
    std::cout << "P" << std::endl;
    std::cout << P_ << std::endl;
}

void EstimateLocalization::setJacobian(const double dt, const Eigen::Matrix<double, 3, 1> X_past)
{
    F_(0, 0) = 1.0;
    F_(0, 2) = u_(0, 0) * (-std::sin(X_past(2, 0) + u_(1, 0) * dt)) * dt;
    F_(1, 1) = 1.0;
    F_(1, 2) = u_(0, 0) * std::cos(X_past(2, 0) + u_(1, 0) * dt) * dt;
    F_(2, 2) = 1.0;

    G_(0, 0) = std::cos(X_past(2, 0)) * dt;
    G_(1, 0) = std::sin(X_past(2, 0)) * dt;
    G_(2, 1) = dt;
}

void EstimateLocalization::prediction(const double dt, const Eigen::Matrix<double, 3, 1> X_past)
{
    // X_(0, 0) = X_past(0, 0) + (u_(0, 0) - X_bias_(0, 0)) * std::cos(X_past(2, 0) + u_(1, 0) * dt) * dt;
    // X_(1, 0) = X_past(1, 0) + (u_(0, 0) - X_bias_(0, 0)) * std::sin(X_past(2, 0) + u_(1, 0) * dt) * dt;
    //X_(2, 0) = X_past(2, 0) + (u_(1, 0) - X_bias_(1, 0)) * dt;

    X_(0, 0) = X_past(0, 0) + u_(0, 0) * std::cos(X_past(2, 0) + u_(1, 0) * dt) * dt;
    X_(1, 0) = X_past(1, 0) + u_(0, 0) * std::sin(X_past(2, 0) + u_(1, 0) * dt) * dt;
    X_(2, 0) = X_past(2, 0) + u_(1, 0) * dt;
    X_(2, 0) = pi2pi(X_(2, 0));
    P_ = F_ * P_ * F_.transpose() + G_ * Q_ * G_.transpose();
}

bool EstimateLocalization::mesurementInspection(const double dt,const Eigen::Matrix<double, 3, 1> X_past)
{
    bool is_use = false;
    if (is_update_ == true)
    {
        if (gnss_status_ == gnss_solution_status::status::FIX && pre_gnss_status_ == gnss_solution_status::status::FIX)
        {
            is_use = true;
        }
        else
        {
            double dx = X_past(0, 0) - z_(0, 0);
            double dy = X_past(1, 0) - z_(1, 0);
            double distance = std::sqrt(dx * dx + dy * dy);
            inspection_distance_=u_(0,0)*dt*1.1;
            is_use = (distance < inspection_distance_);
        }
    }
    return is_use;
}

void EstimateLocalization::update()
{
    Eigen::MatrixXd v = z_ - H_ * X_;
    Eigen::MatrixXd St;
    if (gnss_status_ == gnss_solution_status::FIX)
    {
        St = H_ * P_ * H_.transpose() + R_fix_;
    }
    else if (gnss_status_ == gnss_solution_status::FLOAT)
    {
        St = H_ * P_ * H_.transpose() + R_float_;
    }
    Eigen::MatrixXd Kt = P_ * H_.transpose() * St.inverse();
    X_ = X_ + Kt * v;
    X_(2, 0) = pi2pi(X_(2, 0));
    // P_ = P_ - Kt * St * Kt.transpose();
    P_ = (Eigen::Matrix3d::Identity(3, 3) - Kt * H_) * P_;
    is_update_ = false;
}

void EstimateLocalization::estimateBias(const double dt, const Eigen::Matrix<double, 3, 1> X, const Eigen::Matrix<double, 3, 1> X_past)
{
    X_bias_ = F_bias_ * X_bias_;
    P_bias_ = P_bias_ + Q_bias_;

    double zx = X(0, 0);           // * std::cos(X(2, 0));
    double zy = X(1, 0);           // * std::sin(X(2, 0));
    double zx_past = X_past(0, 0); // * std::cos(X_past(2, 0));
    double zy_past = X_past(1, 0); // * std::sin(X_past(2, 0));
    double dpos = std::sqrt(std::pow(zx - zx_past, 2) + std::pow(zy - zy_past, 2));
    z_bias_(0, 0) = u_(0, 0) - dpos / dt;
    z_bias_(1, 0) = u_(1, 0) - pi2pi(X(2, 0) - X_past(2, 0)) / dt;

    std::cout << z_bias_(1, 0) << std::endl;
    std::cout << u_(1, 0) << ", " << pi2pi(X(2, 0) - X_past(2, 0)) / dt << std::endl;

    Eigen::MatrixXd v_bias = z_bias_ - H_bias_ * X_bias_;
    Eigen::MatrixXd St_bias = H_bias_ * P_bias_ * H_bias_.transpose() + R_bias_;
    Eigen::MatrixXd Kt_bias = P_bias_ * H_bias_.transpose() * St_bias.inverse();
    X_bias_ = X_bias_ + Kt_bias * v_bias;
    P_bias_ = P_bias_ - Kt_bias * St_bias * Kt_bias.transpose();
}

double EstimateLocalization::pi2pi(const double radian)
{
    double value, quotient;
    if (radian > M_PI)
    {
        quotient = std::floor(radian / (M_PI * 3));
        value = radian - M_PI * 2 * (quotient + 1);
    }
    else if (radian < -M_PI)
    {
        quotient = -1.0 * (std::floor(radian / (M_PI * 3)) + 1);
        value = radian + M_PI * 2 * (quotient + 1);
    }
    else
    {
        value = radian;
    }
    return value;
}

void EstimateLocalization::publishMsg()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(X_(0, 0), X_(1, 0), 0.0));
    tf::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, X_(2, 0));
    transform.setRotation(quaternion);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_, child_frame_));
    setOdomMsg();
    position_pub_.publish(odom_);
}

void EstimateLocalization::setOdomMsg()
{
    odom_.pose.pose.position.x = X_(0, 0);
    odom_.pose.pose.position.y = X_(1, 0);
    odom_.pose.pose.position.z = 0.0;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, X_(2, 0));
    geometry_msgs::Quaternion geometry_quaternion;
    tf::quaternionTFToMsg(quaternion, geometry_quaternion);
    odom_.pose.pose.orientation = geometry_quaternion;
    odom_.twist.twist.linear.x = u_(0, 0);
    odom_.twist.twist.angular.z = u_(1, 0);
    int count = 0;
    for (int i = 0; i < P_.rows(); i++)
    {
        for (int j = 0; j < P_.cols(); j++)
        {
            odom_.pose.covariance[count] = P_(i, j);
            count++;
        }
    }
}

}; // namespace localization

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estimate_localization_using_ekf");
    localization::EstimateLocalization node;
    ros::Rate rate(node.rate_);
    while (ros::ok())
    {
        node.extentedKalmanFilter();
        node.publishMsg();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}