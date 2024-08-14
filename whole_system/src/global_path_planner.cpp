/*
A* search algorithm
*/

#include "global_path_planner/global_path_planner.h"

// コンストラクタ
AStarPlanner::AStarPlanner():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("is_visible", is_visible_);
    private_nh_.getParam("sleep_time", sleep_time_);
    private_nh_.getParam("way_points_x", way_points_x_);
    private_nh_.getParam("way_points_y", way_points_y_);

    // --- 基本設定 ---
    // frame idの設定
    global_path_.header.frame_id  = "map";
    current_node_.header.frame_id = "map";
    // dataサイズの確保
    global_path_.poses.reserve(2000);

    // Subscriber
    sub_map_ = nh_.subscribe("/map/updated_map", 1, &AStarPlanner::map_callback, this);

    // Publisher
    pub_global_path_  = nh_.advertise<nav_msgs::Path>("/global_path", 1);
    if(is_visible_)
    {
        pub_current_path_ = nh_.advertise<nav_msgs::Path>("/current_path", 1);
        pub_node_point_   = nh_.advertise<geometry_msgs::PointStamped>("/current_node", 1);
    }
}

// mapのコールバック関数
void AStarPlanner::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_      = *msg;
    flag_map_ = true;
    ros::Duration(0.5).sleep(); // obstacle_expanderノードの終了を待つ(デバッグの都合)
}

// 唯一，main文で実行する関数
void AStarPlanner::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定

    while(ros::ok())
    {
        if(flag_map_)
            planning();    // グローバルパスの生成
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

// グローバルパスを生成
void AStarPlanner::planning()
{
    ROS_INFO_STREAM("----- Global Path Planner will begin ------");
    begin_ = ros::Time::now(); // 実行時間のスタートを設定
    const int phase_size = way_points_x_.size()-1;
    for(int phase=0; phase<phase_size; phase++)
    {
        // リストを空にする
        open_set_.clear();
        closed_set_.clear();

        // スタートノードとゴールノードを保持
        start_node_ = get_way_point(phase);
        goal_node_  = get_way_point(phase+1);

        // スタートノードをOpenリストに追加
        start_node_.cost = heuristic(start_node_); // f(s)=h(s)
        open_set_.push_back(start_node_);

        while(ros::ok())
        {
            // Openリストが空の場合
            if(open_set_.size() == 0)
            {
                ROS_WARN_STREAM("Open set is empty.."); // 探索失敗
                exit(4);
            }

            // Openリスト内で最もコストの小さいノードを現在のノードに指定
            Node current_node = select_current_node();
            show_node_point(current_node);

            // 経路の探索
            if(is_goal(current_node)) // ゴールに到達した場合
            {
                create_path(current_node);
                break; // 探索終了
            }
            else // それ以外の場合
            {
                transfer_node(current_node, open_set_, closed_set_); // 現在のノードをCloseリストに移動
                update_set(current_node); // 隣接ノードを基にOpenリスト・Closeリストを更新
            }
        }
    }
    pub_global_path_.publish(global_path_);
    show_exe_time(); // 実行時間を表示
    exit(0); // ノードの終了
}

// 経由点の取得
Node AStarPlanner::get_way_point(const int phase)
{
    Node way_point;
    way_point.index_x = int(round((way_points_x_[phase] - map_.info.origin.position.x) / map_.info.resolution));
    way_point.index_y = int(round((way_points_y_[phase] - map_.info.origin.position.y) / map_.info.resolution));

    if(is_obs(way_point))
    {
        ROS_ERROR_STREAM("The way point is inappropriate..");
        exit(3);
    }

    return way_point;
}

// ノードが障害物か判断
bool AStarPlanner::is_obs(const Node node)
{
    const int grid_index = node.index_x + (node.index_y * map_.info.width);
    return map_.data[grid_index] == 100;
}

// ヒューリスティック値を計算
double AStarPlanner::heuristic(const Node node)
{
    // ヒューリスティックの重み
    const double w = 1.0;

    // 2点間のユークリッド距離
    const double dx = double(node.index_x - goal_node_.index_x);
    const double dy = double(node.index_y - goal_node_.index_y);
    const double dist = hypot(dx, dy);

    return w * dist;
}

// Openリスト内で最もコストの小さいノードを取得
Node AStarPlanner::select_current_node()
{
    Node current_node = open_set_[0];
    double min_cost = open_set_[0].cost;

    for(const auto& open_node : open_set_)
    {
        if(open_node.cost < min_cost)
        {
            min_cost = open_node.cost;
            current_node = open_node;
        }
    }

    return current_node;
}

// スタートノードの場合、trueを返す
bool AStarPlanner::is_start(const Node node)
{
    return is_same_node(node, start_node_);
}

// ゴールノードの場合、trueを返す
bool AStarPlanner::is_goal(const Node node)
{
    return is_same_node(node, goal_node_);
}

// 2つが同じノードの場合、trueを返す
bool AStarPlanner::is_same_node(const Node n1, const Node n2)
{
    if(n1.index_x == n2.index_x and n1.index_y == n2.index_y)
        return true;
    else
        return false;
}

// waypoint間のパスを作成し，グローバルパスに追加
void AStarPlanner::create_path(Node current_node)
{
    nav_msgs::Path partial_path;
    partial_path.poses.push_back(calc_pose(current_node));

    while(not is_start(current_node))
    {
        for(int i=0; i<closed_set_.size() ;i++)
        {
            if(is_parent(i, current_node))
            {
                current_node = closed_set_[i];
                show_node_point(current_node);
                partial_path.poses.push_back(calc_pose(current_node));
                break;
            }
            if(i == closed_set_.size()-1)
            {
                ROS_ERROR_STREAM("The parent node is not found..");
                exit(6);
            }
        }
    }

    reverse(partial_path.poses.begin(), partial_path.poses.end());
    show_path(partial_path);
    global_path_.poses.insert(global_path_.poses.end(), partial_path.poses.begin(), partial_path.poses.end());
}

// ノードからポーズを計算
geometry_msgs::PoseStamped AStarPlanner::calc_pose(const Node node)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = node.index_x * map_.info.resolution + map_.info.origin.position.x;
    pose_stamped.pose.position.y = node.index_y * map_.info.resolution + map_.info.origin.position.y;

    return pose_stamped;
}

// Closeリストの特定のノードが親ノードか判断
bool AStarPlanner::is_parent(const int closed_node_index, const Node node)
{
    bool is_same_x = closed_set_[closed_node_index].index_x == node.parent_index_x;
    bool is_same_y = closed_set_[closed_node_index].index_y == node.parent_index_y;
    return  is_same_x and is_same_y;
}

// set1からset2にノードを移動
void AStarPlanner::transfer_node(const Node node, std::vector<Node>& set1, std::vector<Node>& set2)
{
    const int set1_node_index = search_node_from_set(node, set1); // リスト1からノードを探す
    if(set1_node_index == -1)
    {
        ROS_ERROR_STREAM("The same node is not found..");
        exit(6);
    }

    set1.erase(set1.begin() + set1_node_index); // リスト1からノードを削除
    set2.push_back(node); // リスト2にノードを追加
}

// 指定したリストに含まれるか検索
int AStarPlanner::search_node_from_set(const Node target_node, std::vector<Node>& set)
{
    for(int i=0; i<set.size(); i++)
        if(is_same_node(target_node, set[i]))
            return i; // インデックスを返す

    return -1; // 含まれない場合
}

// 隣接ノードを基にOpenリスト・Closeリストを更新
void AStarPlanner::update_set(const Node current_node)
{
    // 隣接ノードを宣言
    std::vector<Node> neighbor_nodes;

    // 現在のノードを基に隣接ノードを作成
    creat_neighbor_nodes(current_node, neighbor_nodes);

    // Openリスト・Closeリストを更新
    for(const auto& neighbor_node : neighbor_nodes)
    {
        // 障害物の場合
        if(is_obs(neighbor_node))
            continue;

        // リストに同一ノードが含まれるか調べる
        int flag;
        int node_index; // 同一ノードのインデックス
        std::tie(flag, node_index) = search_node(neighbor_node); // ノードを探す

        if(flag == -1) // OpenリストにもCloseリストにもない場合
        {
            open_set_.push_back(neighbor_node);
        }
        else if(flag == 1) // Openリストにある場合
        {
            if(neighbor_node.cost < open_set_[node_index].cost)
            {
                open_set_[node_index].cost = neighbor_node.cost;
                open_set_[node_index].parent_index_x = neighbor_node.parent_index_x;
                open_set_[node_index].parent_index_y = neighbor_node.parent_index_y;
            }
        }
        else if(flag == 2) // Closeリストにある場合
        {
            if(neighbor_node.cost < closed_set_[node_index].cost)
            {
                closed_set_.erase(closed_set_.begin() + node_index);
                open_set_.push_back(neighbor_node);
            }
        }
    }
}

// 現在のノードを基に隣接ノードを作成
void AStarPlanner::creat_neighbor_nodes(const Node current_node, std::vector<Node>& neighbor_nodes)
{
    // 動作モデルの作成
    std::vector<Motion> motion_set;
    creat_motion_model(motion_set);
    const int motion_num = motion_set.size();

    // 隣接ノードを作成
    for(int i=0; i<motion_num; i++)
    {
        Node neighbor_node = get_neighbor_node(current_node, motion_set[i]); // 隣接ノードを取得
        neighbor_nodes.push_back(neighbor_node);
    }
}

// 動作モデルを作成
void AStarPlanner::creat_motion_model(std::vector<Motion>& motion_set)
{
    motion_set.push_back(get_motion( 1,  0, 1)); // 前
    motion_set.push_back(get_motion( 0,  1, 1)); // 左
    motion_set.push_back(get_motion(-1,  0, 1)); // 後ろ
    motion_set.push_back(get_motion( 0, -1, 1)); // 右

    motion_set.push_back(get_motion(-1,-1, sqrt(2))); // 右後ろ
    motion_set.push_back(get_motion(-1, 1, sqrt(2))); // 左後ろ
    motion_set.push_back(get_motion( 1,-1, sqrt(2))); // 右前
    motion_set.push_back(get_motion( 1, 1, sqrt(2))); // 左前
}

// 動作を作成
Motion AStarPlanner::get_motion(const int dx, const int dy, const double cost)
{
    // 隣接したグリッドに移動しない場合
    if(1 < abs(dx) or 1 < abs(dy))
    {
        ROS_ERROR_STREAM("The motion is inappropriate..");
        exit(5);
    }

    Motion motion;
    motion.dx   = dx;
    motion.dy   = dy;
    motion.cost = cost;

    return motion;
}

// 隣接ノードを取得
Node AStarPlanner::get_neighbor_node(const Node current_node, const Motion motion)
{
    Node neighbor_node;

    // 移動
    neighbor_node.index_x = current_node.index_x + motion.dx;
    neighbor_node.index_y = current_node.index_y + motion.dy;

    // f値を記録
    neighbor_node.cost = (current_node.cost - heuristic(current_node)) + heuristic(neighbor_node) + motion.cost;

    // 親ノードを記録
    neighbor_node.parent_index_x = current_node.index_x;
    neighbor_node.parent_index_y = current_node.index_y;

    return neighbor_node;
}

// OpenリストまたはCloseリストに含まれるか調べる
std::tuple<int, int> AStarPlanner::search_node(const Node target_node)
{
    // Openリストに含まれるか検索
    const int open_node_index = search_node_from_set(target_node, open_set_);
    if(open_node_index != -1)
        return std::make_tuple(1, open_node_index);

    // Closeリストに含まれるか検索
    const int closed_node_index = search_node_from_set(target_node, closed_set_);
    if(closed_node_index != -1)
        return std::make_tuple(2, closed_node_index);

    // OpenリストにもCloseリストにもない場合
    return std::make_tuple(-1, -1);
}

// [デバッグ用] ノードをRvizに表示
void  AStarPlanner::show_node_point(const Node node)
{
    if(is_visible_)
    {
        current_node_.point.x = node.index_x * map_.info.resolution + map_.info.origin.position.x;
        current_node_.point.y = node.index_y * map_.info.resolution + map_.info.origin.position.y;
        pub_node_point_.publish(current_node_);
        ros::Duration(sleep_time_).sleep();
    }
}

// [デバッグ用] パスをRvizに表示
void  AStarPlanner::show_path(nav_msgs::Path& current_path)
{
    if(is_visible_)
    {
        current_path.header.frame_id = "map";
        pub_current_path_.publish(current_path);
        ros::Duration(sleep_time_).sleep();
    }
}

// 実行時間を表示（スタート時間beginを予め設定する）
void AStarPlanner::show_exe_time()
{
    ROS_INFO_STREAM("Duration = " << std::fixed << std::setprecision(2)
                    << ros::Time::now().toSec() - begin_.toSec() << "s");
}
