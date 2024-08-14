#include "local_map/obstacle_detector.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "obstacle_detector"); // ノードの初期化
    ObstacleDetector obstacle_detector;
    obstacle_detector.process();

    return 0;
}
