#include "global_path_planner/global_path_planner.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "global_path_planner"); // ノードの初期化
    AStarPlanner a_star;
    a_star.process();

    return 0;
}
