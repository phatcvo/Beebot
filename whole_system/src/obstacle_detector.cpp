#include "local_map/obstacle_detector.h"

// コンストラクタ
ObstacleDetector::ObstacleDetector():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("ignore_dist", ignore_dist_);
    private_nh_.getParam("ignore_angle_range_list", ignore_angle_range_list_);

    // frame idの設定
    obs_poses_.header.frame_id = "base_link";

    // Subscriber
    sub_laser_ = nh_.subscribe("/scan", 1, &ObstacleDetector::laser_callback, this);

    // Publisher
    pub_obs_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map/obstacle", 1);
}

// laserのコールバック関数
void ObstacleDetector::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_      = *msg;
    flag_laser_ = true;
}

// 唯一，main文で実行する関数
void ObstacleDetector::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(flag_laser_)
            scan_obstacle(); // 障害物検知
        ros::spinOnce();     // コールバック関数の実行
        loop_rate.sleep();   // 周期が終わるまで待つ
    }
}

// 障害物検知
void ObstacleDetector::scan_obstacle()
{
    // 障害物情報のクリア
    obs_poses_.poses.clear();

    // 障害物検知
    for(int i=0; i<laser_.ranges.size(); i+=laser_step_)
    {
        // 角度とレーザ値の距離の算出
        const double angle = i * laser_.angle_increment + laser_.angle_min;
        double dist  = laser_.ranges[i];

        // はずれ値対策
        int index_incr = i;
        int index_decr = i;
        while(dist <= ignore_dist_)
        {
            if(index_incr++ < laser_.ranges.size())
                dist = laser_.ranges[index_incr];
            if(dist<=ignore_dist_ and 0<=index_decr--)
                dist = laser_.ranges[index_decr];
        }

        // 柱と被るレーザ値のスキップ
        if(is_ignore_angle(angle)) continue;

        // 障害物ポーズの格納
        geometry_msgs::Pose obs_pose;
        obs_pose.position.x = dist * cos(angle);
        obs_pose.position.y = dist * sin(angle);

        // 障害物ポーズの追加
        obs_poses_.poses.push_back(obs_pose);
    }

    pub_obs_poses_.publish(obs_poses_);
}

// 柱の場合、trueを返す
bool ObstacleDetector::is_ignore_angle(double angle)
{
    angle = abs(angle);
    const int size = ignore_angle_range_list_.size();

    for(int i=0; i<size/2; i++)
    {
        if(ignore_angle_range_list_[i*2] < angle and angle < ignore_angle_range_list_[i*2 + 1])
            return true;
    }

    if(size%2 == 1)
    {
        if(ignore_angle_range_list_[size-1] < angle)
            return true;
    }

    return false;
}
