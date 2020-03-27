#include "pose_estimators/single_robot_pose_estimator.hpp"

SingleRobotPoseEstimator::SingleRobotPoseEstimator(
    std::string name, const std::shared_ptr<CloudGenerator>& cloud_gen,
    std::string map_frame, geometry_msgs::Pose initial_pose,
    std::string odom_topic)
    
    :name_(name), cloud_gen_(cloud_gen), rd_(), gen_(rd_())
{
    pose_estimate_.pose.pose = initial_pose;
    pose_estimate_.header.frame_id = map_frame;
    pose_estimate_.header.stamp = ros::Time::now();

    pose_estimate_.pose.covariance[0]  = 1e-6;
    pose_estimate_.pose.covariance[7]  = 1e-6;
    pose_estimate_.pose.covariance[35] = 1e-6;

    odom_sub_ = nh_.subscribe(
        odom_topic, 1, &SingleRobotPoseEstimator::odom_cb, this);
}

SingleRobotPoseEstimator::~SingleRobotPoseEstimator() {
    // dtor
}

void SingleRobotPoseEstimator::update_estimate() {
    
}

void SingleRobotPoseEstimator::odom_cb(
    const nav_msgs::OdometryConstPtr& msg)
{

}

double SingleRobotPoseEstimator::prob_normal_distribution(
    double a, double b2) 
{
    return (1.f/sqrt(2*M_PI*b2))*exp(-0.5 * a*a/b2);
}

double SingleRobotPoseEstimator::sample_normal_distribution(double b2) {
    std::normal_distribution<double> dist(0, sqrt(b2));
    return dist(gen_);
}

geometry_msgs::Pose SingleRobotPoseEstimator::sample_motion_model_odometry(
    const nav_msgs::Odometry& ut, const geometry_msgs::Pose& xt_1)
{
    double drot1 = atan2(ut.pose.pose.position.y - xt_1.position.y,
                        ut.pose.pose.position.x - xt_1.position.x);
    double dtrans = sqrt(pow(xt_1.position.x - ut.pose.pose.position.x, 2) +
                         pow(xt_1.position.y - ut.pose.pose.position.y, 2));
    double tht = 2*acos(ut.pose.pose.orientation.w);
    double tht_1 = 2*acos(xt_1.orientation.w);
    double drot2 = tht - tht_1 - drot1;

    double sdrot1 = drot1 - sample_normal_distribution(
        a1_ * drot1 * drot1 + a2_ * dtrans * dtrans);
    double sdtrans = dtrans - sample_normal_distribution(
        a3_ * dtrans * dtrans + a4_ * drot1 * drot1 + a4_ * drot2 * drot2);
    double sdrot2 = drot2 - sample_normal_distribution(
        a1_ * drot2 * drot2 + a2_ * dtrans * dtrans);

    geometry_msgs::Pose xt;
    xt.position.x = xt_1.position.x + sdtrans * cos(tht_1 + sdrot1);
    xt.position.y = xt_1.position.y + sdtrans * sin(tht_1 + sdrot1);
    tht = tht_1 + sdrot1 + sdrot2;
    xt.orientation.w = cos(tht/2.f);
    xt.orientation.z = sin(tht/2.f);

    return xt;
}

// TODO: Consider trying KLD_Sampling_MCL from Probabilistic Robotics
std::vector<geometry_msgs::Pose> 
    SingleRobotPoseEstimator::augmented_MCL(
        const std::vector<geometry_msgs::Pose>& Xt_1,
        const nav_msgs::Odometry& ut)
{
    static double wslow = 0;
    static double wfast = 0;

    // New set of poses
    std::vector<geometry_msgs::Pose> Xt;

    // Pairs of poses with weights
    std::vector<std::pair<geometry_msgs::Pose, float>> Xtbar;

    // Motion model estimates
    std::vector<geometry_msgs::Pose> xt;
    // Measurement model weights
    std::vector<float> wt;

    float wavg = 0;
    int M = Xt_1.size();

    for(int m = 0; m < M; m++) {
        // Update the motion model
        xt.push_back(sample_motion_model_odometry(ut, Xt_1[m]));
        // Update the measurement model
        // wt.push_back(measurement_model(zt, xt[m], m);
        wt.push_back(0.5);
        wavg += wt[m]/M;
        // Update the cloud + weights
        Xtbar.emplace_back(xt[m], wt[m]);
    }

    wslow = wslow + aslow_ * (wavg - wslow);
    wfast = wfast + afast_ * (wavg - wfast);

    std::uniform_real_distribution<double> dist(0, 1);
    float wtotal = wavg*M;
    for(int m = 0; m < M; m++) {
        float p = 1.0 - wfast/wslow;
        if(p < 0) p = 0;
        if(dist(gen_) < p) {
            // Add a random pose to Xt
            Xt.push_back(gen_random_valid_pose());
        }
        else {
            // Place a random point from the cloud into the new one, based
            // on its weight
            p = dist(gen_) * wtotal;
            for(const auto& pair : Xtbar) {
                p -= pair.second;
                if(p <= 0) {
                    Xt.push_back(pair.first);
                    break;
                }
            }
        }
    }
}

geometry_msgs::Pose SingleRobotPoseEstimator::gen_random_valid_pose() {
    static std::uniform_real_distribution<double> xdist(/*map dims*/-1, 1);
    static std::uniform_real_distribution<double> ydist(/*map dims*/-1, 1);
    static std::uniform_real_distribution<double> thdist(-M_PI, M_PI);
    geometry_msgs::Pose pose;
    do {
        pose.position.x = xdist(gen_);
        pose.position.y = ydist(gen_);
        double th = thdist(gen_);
        pose.orientation.z = sin(th/2);
        pose.orientation.w = cos(th/2);
    }
    while(/*pose is invalid*/false);

    return pose;
}
