#include "localizer/emcl.h"

// ===== メイン関数 =====
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "localizer"); // ノードの初期化
    EMCL emcl;
    emcl.process();

    return 0;
}
