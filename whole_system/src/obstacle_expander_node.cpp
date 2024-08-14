#include "global_path_planner/obstacle_expander.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "obstacle_expander"); // ノードの初期化
    ObstacleExpander obstacle_expander;
    obstacle_expander.process();

    return 0;
}
