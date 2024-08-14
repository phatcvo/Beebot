#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>


// ===== クラス =====
class LocalGoalCreator
{
public:
    LocalGoalCreator(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数 ------
    // コールバック関数
    void estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void global_path_callback(const nav_msgs::Path::ConstPtr& msg);

    // その他の関数
    void   update_goal();      // ゴールの更新
    double get_dist_to_goal(); // 現在位置-ゴール間の距離の取得


    // ----- 変数 -----
    int    hz_;                  // ループ周波数 [Hz]
    int    index_step_;          // １回で更新するインデックス数
    int    goal_index_;          // グローバルパス内におけるローカルゴールのインデックス
    double target_dist_to_goal_; // 現在位置-ゴール間の距離 [m]

    // msg受け取りフラグ
    bool flag_global_path_ = false;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_global_path_;
    ros::Subscriber sub_estimated_pose_;

    // Publisher
    ros::Publisher pub_local_goal_;

    // 各種オブジェクト
    geometry_msgs::PointStamped local_goal_;     // local path用の目標位置
    geometry_msgs::PoseStamped  estimated_pose_; // 現在位置
    nav_msgs::Path              global_path_;    // グローバルパス
};

#endif
