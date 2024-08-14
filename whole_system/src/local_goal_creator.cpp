#include "local_goal_creator/local_goal_creator.h"

// コンストラクタ
LocalGoalCreator::LocalGoalCreator():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("index_step", index_step_);
    private_nh_.getParam("goal_index", goal_index_);
    private_nh_.getParam("target_dist_to_goal", target_dist_to_goal_);

    // frame idの設定
    local_goal_.header.frame_id = "map";

    // Subscriber
    sub_estimated_pose_ = nh_.subscribe("/estimated_pose", 1, &LocalGoalCreator::estimated_pose_callback, this);
    sub_global_path_    = nh_.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this);

    // Publisher
    pub_local_goal_ = nh_.advertise<geometry_msgs::PointStamped>("/local_goal", 1);
}

// estimated_poseのコールバック関数
void LocalGoalCreator::estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    estimated_pose_  = *msg;
}

// global_pathのコールバック関数
void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    global_path_      = *msg;
    flag_global_path_ = true;
}

// 唯一，main文で実行する関数
void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(flag_global_path_)
            update_goal(); // ゴールの更新
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

// ゴールの更新
void LocalGoalCreator::update_goal()
{
    double dist_to_goal = get_dist_to_goal(); // ゴールまでの距離を取得

    while(dist_to_goal < target_dist_to_goal_)
    {
        goal_index_  += index_step_;        // ゴール位置をステップ数だけ先に進める
        dist_to_goal  = get_dist_to_goal(); // ゴールまでの距離を取得

        if(goal_index_ >= global_path_.poses.size())
        {
            goal_index_ = global_path_.poses.size()-1; // グローバルゴールで固定
            break;
        }
    }

    // ゴール位置を取得し，パブリッシュ
    local_goal_.point.x = global_path_.poses[goal_index_].pose.position.x;
    local_goal_.point.y = global_path_.poses[goal_index_].pose.position.y;
    local_goal_.header.stamp = ros::Time::now();
    pub_local_goal_.publish(local_goal_);
}

// 現在位置-ゴール間の距離の取得
double LocalGoalCreator::get_dist_to_goal()
{
    const double dx = global_path_.poses[goal_index_].pose.position.x - estimated_pose_.pose.position.x;
    const double dy = global_path_.poses[goal_index_].pose.position.y - estimated_pose_.pose.position.y;

    return hypot(dx, dy);
}
