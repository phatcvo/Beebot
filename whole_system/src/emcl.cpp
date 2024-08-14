/*
emcl: mcl with expansion resetting
*/

#include "localizer/emcl.h"

// コンストラクタ
EMCL::EMCL():private_nh_("~"), engine_(seed_gen_())
{
    // パラメータの取得(EMCL)
    private_nh_.getParam("flag_init_noise", flag_init_noise_);
    private_nh_.getParam("flag_broadcast", flag_broadcast_);
    private_nh_.getParam("is_visible", is_visible_);
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("particle_num", particle_num_);
    private_nh_.getParam("move_dist_th", move_dist_th_);
    private_nh_.getParam("init_x", init_x_);
    private_nh_.getParam("init_y", init_y_);
    private_nh_.getParam("init_yaw", init_yaw_);
    private_nh_.getParam("init_x_dev", init_x_dev_);
    private_nh_.getParam("init_y_dev", init_y_dev_);
    private_nh_.getParam("init_yaw_dev", init_yaw_dev_);
    private_nh_.getParam("alpha_th", alpha_th_);
    private_nh_.getParam("reset_count_limit", reset_count_limit_);
    private_nh_.getParam("expansion_x_dev", expansion_x_dev_);
    private_nh_.getParam("expansion_y_dev", expansion_y_dev_);
    private_nh_.getParam("expansion_yaw_dev", expansion_yaw_dev_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("sensor_noise_ratio", sensor_noise_ratio_);
    private_nh_.getParam("ignore_angle_range_list", ignore_angle_range_list_);
    // パラメータの取得(OdomModel)
    private_nh_.getParam("ff", ff_);
    private_nh_.getParam("fr", fr_);
    private_nh_.getParam("rf", rf_);
    private_nh_.getParam("rr", rr_);

    // --- 基本設定 ---
    // frame idの設定
    estimated_pose_msg_.header.frame_id = "map";
    particle_cloud_msg_.header.frame_id = "map";
    // メモリの確保
    particle_cloud_msg_.poses.reserve(particle_num_);
    // odometryのモデルの初期化
    odom_model_ = OdomModel(ff_, fr_, rf_, rr_);

    // Subscriber
    sub_map_   = nh_.subscribe("/map", 1, &EMCL::map_callback, this);
    sub_odom_  = nh_.subscribe("/roomba/odometry", 1, &EMCL::odom_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 1, &EMCL::laser_callback, this);

    // Publisher
    pub_estimated_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
    pub_particle_cloud_ = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);
}

// mapのコールバック関数
void EMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_      = *msg;
    flag_map_ = true;
}

// odometryのコールバック関数
void EMCL::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    prev_odom_ = last_odom_;
    last_odom_ = *msg;
    flag_odom_ = true;

    if(not flag_move_)
    {
        const double dx = prev_odom_.pose.pose.position.x;
        const double dy = prev_odom_.pose.pose.position.y;
        if(move_dist_th_ < hypot(dx, dy)) // 動き出したらフラグを立てる
            flag_move_ = true;
    }
}

// laserのコールバック関数
void EMCL::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_      = *msg;
    flag_laser_ = true;
}

// 唯一，main文で実行する関数
void EMCL::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定
    initialize();             // パーティクルの初期化

    while(ros::ok())
    {
        if(flag_map_ and flag_odom_ and flag_laser_)
        {
            broadcast_odom_state(); // map座標系とodom座標系の関係を報告
            if(flag_move_)
            {
                localize(); // 自己位置推定
            }
            else
            {
                publish_estimated_pose(); // 推定位置のパブリッシュ
                publish_particles();      // パーティクルクラウドのパブリッシュ
            }
        }
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

// パーティクル，推定位置の初期化
void EMCL::initialize()
{
    // 推定位置の初期化
    estimated_pose_.set(init_x_, init_y_, init_yaw_);

    Particle particle;

    for(int i=0; i<particle_num_; i++)
    {
        if(flag_init_noise_) // 初期位置近傍にパーティクルを配置
        {
            const double x   = norm_rv(init_x_,   init_x_dev_);
            const double y   = norm_rv(init_y_,   init_y_dev_);
            const double yaw = norm_rv(init_yaw_, init_yaw_dev_);
            particle.pose_.set(x, y, yaw);
            particle.pose_.normalize_angle();
        }
        else
        {
            const double x   = init_x_;
            const double y   = init_y_;
            const double yaw = init_yaw_;
            particle.pose_.set(x, y, yaw);
            particle.pose_.normalize_angle();
        }
        particles_.push_back(particle);
    }

    reset_weight(); // 重みの初期化
}

// ランダム変数生成関数（正規分布）
double EMCL::norm_rv(const double mean, const double stddev)
{
    std::normal_distribution<> norm_dist(mean, stddev);
    return norm_dist(engine_);
}

// 重みの初期化
void EMCL::reset_weight()
{
    for(auto& p : particles_)
        p.set_weight(1.0/particles_.size());
}

// map座標系とodom座標系の関係を報告
void EMCL::broadcast_odom_state()
{
    if(flag_broadcast_)
    {
        // TF Broadcasterの実体化
        static tf2_ros::TransformBroadcaster odom_state_broadcaster;

        // map座標系からみたbase_link座標系の位置と姿勢の取得
        const double map_to_base_yaw = estimated_pose_.yaw();
        const double map_to_base_x   = estimated_pose_.x();
        const double map_to_base_y   = estimated_pose_.y();

        // odom座標系からみたbase_link座標系の位置と姿勢の取得
        const double odom_to_base_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
        const double odom_to_base_x   = last_odom_.pose.pose.position.x;
        const double odom_to_base_y   = last_odom_.pose.pose.position.y;

        // map座標系からみたodom座標系の位置と姿勢の取得
        // (回転行列を使った単純な座標変換)
        const double map_to_odom_yaw = normalize_angle(map_to_base_yaw - odom_to_base_yaw);
        const double map_to_odom_x   = map_to_base_x - odom_to_base_x * cos(map_to_odom_yaw)
            + odom_to_base_y * sin(map_to_odom_yaw);
        const double map_to_odom_y   = map_to_base_y - odom_to_base_x * sin(map_to_odom_yaw)
            - odom_to_base_y * cos(map_to_odom_yaw);

        // yawからquaternionを作成
        tf2::Quaternion map_to_odom_quat;
        map_to_odom_quat.setRPY(0, 0, map_to_odom_yaw);

        // odom座標系の元となodomの位置姿勢情報格納用変数の作成
        geometry_msgs::TransformStamped odom_state;

        // 現在の時間の格納
        odom_state.header.stamp = ros::Time::now();

        // 親フレーム・子フレームの指定
        odom_state.header.frame_id = map_.header.frame_id;
        odom_state.child_frame_id  = last_odom_.header.frame_id;

        // map座標系からみたodom座標系の原点位置と方向の格納
        odom_state.transform.translation.x = map_to_odom_x;
        odom_state.transform.translation.y = map_to_odom_y;
        odom_state.transform.rotation.x    = map_to_odom_quat.x();
        odom_state.transform.rotation.y    = map_to_odom_quat.y();
        odom_state.transform.rotation.z    = map_to_odom_quat.z();
        odom_state.transform.rotation.w    = map_to_odom_quat.w();

        // tf情報をbroadcast(座標系の設定)
        odom_state_broadcaster.sendTransform(odom_state);
        // odom_state_broadcaster_.sendTransform(odom_state);
    }
}

// 適切な角度(-M_PI ~ M_PI)を返す
double EMCL::normalize_angle(double angle)
{
    while(M_PI  < angle) angle -= 2.0*M_PI;
    while(angle < -M_PI) angle += 2.0*M_PI;

    return angle;
}

// 自己位置推定
void EMCL::localize()
{
    motion_update();          // 動作更新
    observation_update();     // 観測更新（位置推定・リサンプリングを含む）
    publish_estimated_pose(); // 推定位置のパブリッシュ
    publish_particles();      // パーティクルクラウドのパブリッシュ
}

// 動作更新
void EMCL::motion_update()
{
    // quaternionからyawを算出
    const double last_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);
    const double prev_yaw = tf2::getYaw(prev_odom_.pose.pose.orientation);

    // 微小移動量を算出
    const double dx   = last_odom_.pose.pose.position.x - prev_odom_.pose.pose.position.x;
    const double dy   = last_odom_.pose.pose.position.y - prev_odom_.pose.pose.position.y;
    const double dyaw = normalize_angle(last_yaw - prev_yaw);

    // 1制御周期前のロボットから見た現在位置の距離と方位を算出
    const double length    = hypot(dx, dy);
    const double direction = normalize_angle(atan2(dy, dx) - prev_yaw);

    // 標準偏差を設定
    odom_model_.set_dev(length, dyaw);

    // 全パーティルクの移動
    for(auto& p : particles_)
        p.pose_.move(length, direction, dyaw, odom_model_.get_fw_noise(), odom_model_.get_rot_noise());
}

// 観測更新（位置推定・リサンプリングを含む）
void EMCL::observation_update()
{
    // 尤度計算
    for(auto& p : particles_)
    {
        const double L = p.likelihood(map_, laser_, sensor_noise_ratio_, laser_step_, ignore_angle_range_list_);
        p.set_weight(p.weight() * L);
    }

    // パーティクル1つのレーザ1本における平均尤度を算出
    const double alpha = calc_marginal_likelihood() / ((laser_.ranges.size()/laser_step_) * particles_.size());

    if(alpha < alpha_th_ and reset_counter < reset_count_limit_) // 尤度が小さ過ぎる場合
    {
        std::cout << "[resetting] alpha = " << std::fixed << std::setprecision(4) << alpha << std::endl;
        median_pose();         // 推定位置の決定（中央値）
        expansion_resetting(); // 膨張リセット
        reset_counter++;
    }
    else
    {
        std::cout << "[resampling] alpha = " << std::fixed << std::setprecision(4) << alpha << std::endl;
        estimate_pose(); // 推定位置の決定
        resampling();    // リサンプリング
        reset_counter = 0;
    }
}

// 周辺尤度の算出
double EMCL::calc_marginal_likelihood()
{
    double sum = 0.0;
    for(const auto& p : particles_)
        sum += p.weight();

    return sum;
}

// 推定位置の決定
void EMCL::estimate_pose()
{
    // mean_pose(); // 平均
    weighted_mean_pose(); // 加重平均
    // max_weight_pose(); // 最大の重みを有するポーズ
    // median_pose(); // 中央値
}

// 推定位置の決定（平均）
void EMCL::mean_pose()
{
    // 合計値
    double x_sum   = 0.0;
    double y_sum   = 0.0;
    double yaw_sum = 0.0;
    for(const auto& p : particles_)
    {
        x_sum   += p.pose_.x();
        y_sum   += p.pose_.y();
        yaw_sum += p.pose_.yaw();
    }

    // 平均値
    estimated_pose_.set(x_sum, y_sum, yaw_sum);
    estimated_pose_ /= particles_.size();
    estimated_pose_.normalize_angle();
}

// 推定位置の決定（加重平均）
void EMCL::weighted_mean_pose()
{
    // 重みの正規化
    normalize_belief();

    // 平均値
    double x_mean     = 0.0;
    double y_mean     = 0.0;
    double yaw_mean   = particles_[0].pose_.yaw();
    double max_weight = particles_[0].weight();
    for(const auto& p : particles_)
    {
        x_mean += p.pose_.x() * p.weight();
        y_mean += p.pose_.y() * p.weight();

        if(max_weight < p.weight())
        {
            yaw_mean   = p.pose_.yaw(); // 重みが最大のパーティクルの値を取得
            max_weight = p.weight();
        }
    }

    estimated_pose_.set(x_mean, y_mean, yaw_mean);
}

// 重みの正規化
void EMCL::normalize_belief()
{
    // 尤度の合計
    const double weight_sum = calc_marginal_likelihood();

    // 正規化
    for(auto& p : particles_)
        p.set_weight(p.weight() / weight_sum);
}

// 推定位置の決定（最大の重みを有するポーズ）
void EMCL::max_weight_pose()
{
    double max_weight = particles_[0].weight();
    estimated_pose_ = particles_[0].pose_;
    for(const auto& p : particles_)
    {
        if(max_weight < p.weight())
        {
            max_weight      = p.weight();
            estimated_pose_ = p.pose_;
        }
    }
}

// 推定位置の決定（中央値）
void EMCL::median_pose()
{
    std::vector<double> x_list;
    std::vector<double> y_list;
    std::vector<double> yaw_list;

    for(const auto& p : particles_)
    {
        x_list.push_back(p.pose_.x());
        y_list.push_back(p.pose_.y());
        yaw_list.push_back(p.pose_.yaw());
    }

    const double x_median   = get_median(x_list);
    const double y_median   = get_median(y_list);
    const double yaw_median = get_median(yaw_list);
    estimated_pose_.set(x_median, y_median, yaw_median);
}

// 配列の中央値を返す
double EMCL::get_median(std::vector<double>& data)
{
    sort(begin(data), end(data));
    if(data.size()%2 == 1)
        return data[(data.size()-1) / 2];
    else
        return (data[data.size()/2 - 1] + data[data.size()/2]) / 2.0;
}

// 膨張リセット
void EMCL::expansion_resetting()
{
    // ノイズを加える
    for(auto& p : particles_)
    {
        const double x   = norm_rv(p.pose_.x(),   expansion_x_dev_);
        const double y   = norm_rv(p.pose_.y(),   expansion_y_dev_);
        const double yaw = norm_rv(p.pose_.yaw(), expansion_yaw_dev_);
        p.pose_.set(x, y, yaw);
        p.pose_.normalize_angle();
    }

    // 重みを初期化
    reset_weight();
}

// リサンプリング（系統サンプリング）
void EMCL::resampling()
{
    // パーティクルの重みを積み上げたリストを作成
    std::vector<double> accum;
    accum.push_back(particles_[0].weight());
    for(int i=1; i<particles_.size(); i++)
        accum.push_back(accum.back() + particles_[i].weight());

    // サンプリングのスタート位置とステップを設定
    const std::vector<Particle> old(particles_);
    const double step  = accum.back() / particles_.size();
    const double start = (double)rand()/RAND_MAX * step; // 0 ~ W/N (W: sum of weight)

    // サンプリングするパーティクルのインデックスを保持
    std::vector<int> chosen_indexes;
    int tick=0;
    for(int i=0; i<particles_.size(); i++)
    {
        while(accum[tick] <= start + i*step)
        {
            tick++;
            if(tick == particles_.size())
            {
                ROS_ERROR("Resampling Failed"); // 配列の不正アクセス防止
                exit(1);
            }
        }
        chosen_indexes.push_back(tick);
    }

    // リサンプリング
    for(int i=0; i<particles_.size(); i++)
        particles_[i] = old[chosen_indexes[i]];

    // 重みを初期化
    reset_weight();
}

// 推定位置のパブリッシュ
void EMCL::publish_estimated_pose()
{
    estimated_pose_msg_.pose.position.x = estimated_pose_.x();
    estimated_pose_msg_.pose.position.y = estimated_pose_.y();

    // yawからquaternionを作成
    tf2::Quaternion q;
    q.setRPY(0, 0, estimated_pose_.yaw());
    tf2::convert(q, estimated_pose_msg_.pose.orientation);

    pub_estimated_pose_.publish(estimated_pose_msg_);
}

// パーティクルクラウドのパブリッシュ
void EMCL::publish_particles()
{
    if(is_visible_)
    {
        particle_cloud_msg_.poses.clear();
        /* --- パーティクルの数が変わる場合，ここでresize() --- */
        geometry_msgs::Pose pose;

        for(const auto& particle : particles_)
        {
            pose.position.x = particle.pose_.x();
            pose.position.y = particle.pose_.y();

            // yawからquaternionを作成
            tf2::Quaternion q;
            q.setRPY(0, 0, particle.pose_.yaw());
            tf2::convert(q, pose.orientation);

            particle_cloud_msg_.poses.push_back(pose);
        }

        pub_particle_cloud_.publish(particle_cloud_msg_);
    }
}
