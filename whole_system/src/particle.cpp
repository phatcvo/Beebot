#include "localizer/particle.h"

// デフォルトコンストラクタ
Particle::Particle() : pose_(0.0, 0.0, 0.0)
{
    weight_ = 0.0;
}

// コンストラクタ
Particle::Particle(const double x, const double y, const double yaw, const double weight) : pose_(x, y, yaw)
{
    weight_ = weight;
}

// 代入演算子
Particle& Particle::operator =(const Particle& p)
{
    pose_   = p.pose_;
    weight_ = p.weight_;

    return *this;
}

// setter
void Particle::set_weight(const double weight)
{
    weight_ = weight;
}

// 尤度関数
double Particle::likelihood(const nav_msgs::OccupancyGrid& map, const sensor_msgs::LaserScan& laser,
        const double sensor_noise_ratio, const int laser_step, const std::vector<double>& ignore_angle_range_list)
{
    double L = 0.0; // 尤度

    // センサ情報からパーティクルの姿勢を評価
    for(int i=0; i<laser.ranges.size(); i+=laser_step)
    {
        const double angle = i * laser.angle_increment + laser.angle_min; // レーザ値の角度

        if(not is_ignore_angle(angle, ignore_angle_range_list)) // 柱と被るレーザ値のスキップ
        {
            const double range = calc_dist_to_wall(pose_.x(), pose_.y(), angle+pose_.yaw(), map,
                    laser.ranges[i], sensor_noise_ratio);
            L += norm_pdf(range, laser.ranges[i], laser.ranges[i] * sensor_noise_ratio);
        }
    }

    return L;
}

// 柱の場合、trueを返す
bool Particle::is_ignore_angle(double angle, const std::vector<double>& ignore_angle_range_list)
{
    angle = abs(angle);
    const int size = ignore_angle_range_list.size();

    for(int i=0; i<size/2; i++)
    {
        if(ignore_angle_range_list[i*2] < angle and angle < ignore_angle_range_list[i*2 + 1])
            return true;
    }

    if(size%2 == 1)
    {
        if(ignore_angle_range_list[size-1] < angle)
            return true;
    }

    return false;
}

// 壁までの距離を算出
double Particle::calc_dist_to_wall(double x, double y, const double laser_angle, const nav_msgs::OccupancyGrid& map,
        const double laser_range, const double sensor_noise_ratio)
{
    const double search_step = map.info.resolution;
    const double search_limit = laser_range;

    for(double dist=0.0; dist<search_limit; dist+=search_step)
    {
        x += search_step * cos(laser_angle);
        y += search_step * sin(laser_angle);

        const int grid_index = xy_to_grid_index(x, y, map.info);

        if(not in_map(grid_index, map.data.size()))
            return search_limit * 2.0;
        else if(map.data[grid_index] == -1)
            return search_limit * 2.0;
        else if(map.data[grid_index] == 100)
            return dist;
    }

    return search_limit * sensor_noise_ratio * 5.0;
}

// 座標からグリッドのインデックスを返す
int Particle::xy_to_grid_index(const double x, const double y, const nav_msgs::MapMetaData& map_info)
{
    const int index_x = int(round((x - map_info.origin.position.x) / map_info.resolution));
    const int index_y = int(round((y - map_info.origin.position.y) / map_info.resolution));

    return index_x + (index_y * map_info.width);
}

// マップ内の場合、trueを返す
bool Particle::in_map(const int grid_index, const int map_data_size)
{
    if(0 <= grid_index and grid_index < map_data_size)
        return true;
    else
        return false;
}

// 確率密度関数（正規分布）
double Particle::norm_pdf(const double x, const double mean, const double stddev)
{
    return 1.0/sqrt(2.0 * M_PI * pow(stddev, 2.0)) * exp(-pow((x - mean), 2.0)/(2.0*pow(stddev, 2.0)));
}
