#include "local_map/local_map_creator.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_map_creator"); // ノードの初期化
    LocalMapCreator local_map_creator;
    local_map_creator.process();

    return 0;
}
