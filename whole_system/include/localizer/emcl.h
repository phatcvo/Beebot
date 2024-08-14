/*
emcl: mcl with expansion resetting
*/

#ifndef EMCL_H
#define EMCL_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <random>

#include "localizer/odom_model.h"
#include "localizer/particle.h"
#include "localizer/pose.h"


class EMCL
{
public:
    EMCL(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数（引数あり）------
    // コールバック関数
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // その他の関数
    double get_median(std::vector<double>& data);            // 配列の中央値を返す
    double norm_rv(const double mean, const double stddev);  // ランダム変数生成関数（正規分布）
    double normalize_angle(double angle);                    // 適切な角度(-M_PI ~ M_PI)を返す


    // ----- 関数（引数なし）------
    void   initialize();               // パーティクルの初期化
    void   reset_weight();             // 重みの初期化
    void   broadcast_odom_state();     // map座標系とodom座標系の関係を報告
    void   localize();                 // 自己位置推定
    void   motion_update();            // 動作更新
    void   observation_update();       // 観測更新
    void   estimate_pose();            // 推定位置の決定
    void   mean_pose();                // 推定位置の決定（平均）
    void   weighted_mean_pose();       // 推定位置の決定（加重平均）
    void   max_weight_pose();          // 推定位置の決定（最大の重みを有するポーズ）
    void   median_pose();              // 推定位置の決定（中央値）
    void   normalize_belief();         // 重みの正規化
    void   expansion_resetting();      // 膨張リセット
    void   resampling();               // リサンプリング
    void   publish_estimated_pose();   // 推定位置のパブリッシュ
    void   publish_particles();        // パーティクルクラウドのパブリッシュ
    double calc_marginal_likelihood(); // 周辺尤度の算出


    // ----- 変数 -----
    int       hz_;                 // ループ周波数 [Hz]
    int       particle_num_;       // パーティクルの個数 [-]
    int       reset_counter = 0;   // 連続リセットの回数 [-]
    int       reset_count_limit_;  // 連続リセットの回数の上限 [-]
    int       laser_step_;         // 何本ずつレーザを見るか [-]
    double    move_dist_th_;       // ロボットの移動開始判断用（スタートからの距離の閾値）[m]
    double    init_x_;             // 初期位置 [m]
    double    init_y_;             // 初期位置 [m]
    double    init_yaw_;           // 初期姿勢 [rad]
    double    init_x_dev_;         // 初期位置xの標準偏差 [m]
    double    init_y_dev_;         // 初期位置yの標準偏差 [m]
    double    init_yaw_dev_;       // 初期姿勢の標準偏差 [rad]
    double    alpha_th_;           // リセットに関する平均尤度の閾値 [-]
    double    expansion_x_dev_;    // 膨張リセットの位置xの標準偏差 [m]
    double    expansion_y_dev_;    // 膨張リセットの位置yの標準偏差 [m]
    double    expansion_yaw_dev_;  // 膨張リセットの姿勢の標準偏差 [rad]
    double    sensor_noise_ratio_; // 距離に対するセンサノイズ比 [-]
    Pose      estimated_pose_;     // 推定位置
    OdomModel odom_model_;         // odometryのモデル

    // リスト
    std::vector<Particle> particles_;             // パーティクルクラウド（計算用）
    std::vector<double> ignore_angle_range_list_; // 柱に関する角度範囲の配列 [rad]

    // msg受け取りフラグ
    bool flag_map_   = false;
    bool flag_odom_  = false;
    bool flag_laser_ = false;

    // その他のフラグ
    bool flag_move_ = false; // 機体が動いたか判断用
    bool flag_init_noise_;   // 初期位置にノイズを加えるか
    bool flag_broadcast_;    // tf broadcastをするか
    bool is_visible_;        // パーティクルクラウドをパブリッシングするか

    // OdomModel関連
    double ff_;
    double fr_;
    double rf_;
    double rr_;

    // 正規分布用乱数
    std::random_device seed_gen_;
    std::default_random_engine engine_;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_map_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_laser_;

    // Publisher
    ros::Publisher pub_estimated_pose_;
    ros::Publisher pub_particle_cloud_;

    // 各種オブジェクト
    nav_msgs::OccupancyGrid    map_;                // map_serverから受け取るマップ
    nav_msgs::Odometry         last_odom_;          // 最新のodometry
    nav_msgs::Odometry         prev_odom_;          // 1制御周期前のodometry
    sensor_msgs::LaserScan     laser_;              // レーザ値
    geometry_msgs::PoseStamped estimated_pose_msg_; // 推定位置
    geometry_msgs::PoseArray   particle_cloud_msg_; // パーティクルクラウド（パブリッシュ用）
};

#endif
