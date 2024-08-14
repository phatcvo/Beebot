/*
A* search algorithm
*/

#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>


// ===== 構造体 =====
struct Node
{
    int    index_x = 0;         // [cell]
    int    index_y = 0;         // [cell]
    int    parent_index_x = -1; // [cell]
    int    parent_index_y = -1; // [cell]
    double cost = 0.0;          // f値
};

struct Motion
{
    int dx; // [cell]
    int dy; // [cell]
    double cost;
};


// ===== クラス =====
class AStarPlanner
{
public:
    AStarPlanner(); // デフォルトコンストラクタ
    void process();

private:
    // ----- 関数（引数あり）------
    // コールバック関数
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // その他の関数
    void   update_set(const Node current_node);                                              // Open・Closeリストを更新
    void   create_path(Node current_node);                                                   // waypoint間のパスを作成
    void   creat_motion_model(std::vector<Motion>& motion_set);                              // 動作モデルを作成
    void   creat_neighbor_nodes(const Node current_node, std::vector<Node>& neighbor_nodes); // すべての隣接ノードを作成
    void   transfer_node(const Node node, std::vector<Node>& set1, std::vector<Node>& set2); // set1からset2にノードを移動
    void   show_node_point(const Node node);                                                 // ノードを表示(デバッグ用)
    void   show_path(nav_msgs::Path& current_path);                                          // パスを表示(デバッグ用)
    bool   is_obs(const Node node);                                                          // 障害物か判断
    bool   is_start(const Node node);                                                        // スタートか判断
    bool   is_goal(const Node node);                                                         // ゴールか判断
    bool   is_same_node(const Node n1, const Node n2);                                       // 同じノードか判断
    bool   is_parent(const int closed_node_index, const Node node);                          // 親ノードか判断
    int    search_node_from_set(const Node target_node, std::vector<Node>& set);             // 特定のリストに含まれるか検索
    double heuristic(const Node node);                                                       // ヒューリスティック関数
    Node   get_way_point(const int phase);                                                   // 経由点の取得
    Node   get_neighbor_node(const Node current_node, const Motion motion);                  // 隣接ノードを取得
    Motion get_motion(const int dx, const int dy, const double cost);                        // 動作を作成
    std::tuple<int, int> search_node(const Node target_node);                                // リストに含まれるか調べる
    geometry_msgs::PoseStamped calc_pose(const Node node);                                   // ノードからポーズを計算


    // ----- 関数（引数なし）------
    void planning();            // グローバルパスの生成
    void show_exe_time();       // 実行時間を表示（スタート時間beginを予め設定する）
    Node select_current_node(); // Openリスト内で最もコストの小さいノードを取得


    // ----- 変数 -----
    int  hz_;                          // ループ周波数 [Hz]
    bool is_visible_;                  // 可視化フラグ(パス・ノード)
    double sleep_time_;                // 次のpublishまでの時間 [s]
    Node start_node_;                  // スタートノード
    Node goal_node_;                   // ゴールノード
    std::vector<Node> open_set_;       // Openリスト
    std::vector<Node> closed_set_;     // Closeリスト
    std::vector<double> way_points_x_; // スタートとゴールを含む
    std::vector<double> way_points_y_; // スタートとゴールを含む

    // msg受け取りフラグ
    bool flag_map_ = false;

    // 実行時間表示用
    ros::Time begin_;


    // ----- その他のオブジェクト -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_map_;

    // Publisher
    ros::Publisher pub_global_path_;
    ros::Publisher pub_current_path_; // デバッグ用
    ros::Publisher pub_node_point_;   // デバッグ用

    // 各種オブジェクト
    nav_msgs::OccupancyGrid map_; // obstacle_expanderノードから受け取るマップ
    nav_msgs::Path global_path_;  // グローバルパス
    geometry_msgs::PointStamped current_node_; // デバッグ用
};

#endif
