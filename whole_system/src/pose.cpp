#include "localizer/pose.h"

// デフォルトコンストラクタ
Pose::Pose()
{
    x_   = 0.0;
    y_   = 0.0;
    yaw_ = 0.0;
}

// コンストラクタ
Pose::Pose(const double x, const double y, const double yaw)
{
    x_   = x;
    y_   = y;
    yaw_ = yaw;
}

// 代入演算子
Pose& Pose::operator =(const Pose& pose)
{
    x_   = pose.x_;
    y_   = pose.y_;
    yaw_ = pose.yaw_;

    return *this;
}

// 複合代入演算子/=
Pose& Pose::operator /=(const double a)
{
    x_   /= a;
    y_   /= a;
    yaw_ /= a;

    return *this;
}

// setter
void Pose::set(const double x, const double y, const double yaw)
{
    x_   = x;
    y_   = y;
    yaw_ = yaw;
}

// パーティクルの移動
void Pose::move(double length, double direction, double rotation, const double fw_noise, const double rot_noise)
{
    // ノイズを加える
    length    += fw_noise;
    direction += rot_noise;
    rotation  += rot_noise;

    // 移動
    x_   += length * cos(direction + yaw_);
    y_   += length * sin(direction + yaw_);
    yaw_ += rotation;
    normalize_angle();
}

// 適切な角度(-M_PI ~ M_PI)に変更
void Pose::normalize_angle()
{
    while(M_PI < yaw_ ) yaw_ -= 2.0*M_PI;
    while(yaw_ < -M_PI) yaw_ += 2.0*M_PI;
}