#include <common_lib.h>
#include "use-ikfom.h"
#include <unordered_map>
#include <iostream>
#include <queue>
#include <time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define HASH_P 116101
#define MAX_N 10000000000

class VOXEL_LOC
{
public:
    int64_t x, y, z;

    VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOC &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }
};

namespace std
{
    template <>
    struct hash<VOXEL_LOC>
    {
        int operator()(const VOXEL_LOC &s) const
        {
            using std::hash;
            using std::size_t;
            return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
        }
    };
}

bool first_delet = true;

std::queue<pcl::PointCloud<pcl::PointXYZINormal>> cloud_queue;
std::queue<MTK::vect<3, double>> pose_queue;
std::queue<MTK::SubManifold<SO3, 3, 3>> rot_queue;
std::vector<pcl::PointCloud<pcl::PointXYZINormal>> feature_vec;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_feature_(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_delet(new pcl::PointCloud<pcl::PointXYZINormal>);

std::unordered_map<VOXEL_LOC, int> map_feature;
float voxel_box_size = 0.2;

bool in_range(pcl::PointXYZINormal p_in, MTK::vect<3, double> pose_in, pcl::PointXYZINormal p_judge, MTK::vect<3, double> pose_judge)
{
    pcl::PointXYZINormal point_1;
    point_1.x = p_in.x - pose_in[0];
    point_1.y = p_in.y - pose_in[1];
    point_1.z = p_in.z - pose_in[2];

    pcl::PointXYZINormal point_2;
    point_2.x = p_judge.x - pose_judge[0];
    point_2.y = p_judge.y - pose_judge[1];
    point_2.z = p_judge.z - pose_judge[2];

    double dis_1 = sqrt(pow(p_in.x - pose_in.x(), 2) + pow(p_in.y - pose_in.y(), 2));
    double dis_2 = sqrt(pow(p_judge.x - pose_in.x(), 2) + pow(p_judge.y - pose_in.y(), 2));

    if (point_1.x * point_2.x > 0 && point_1.y * point_2.y > 0 && dis_2 < dis_1)
        return true;
    return false;
}

PointCloudXYZI deletMovingObj(PointCloudXYZI::Ptr feats_undistort, state_ikfom state_point, PointCloudXYZI::Ptr featsFromMap)
{
    cloud_feature_->points.clear();
    clock_t start, end;
    start = clock();

    PointCloudXYZI::Ptr current_pc(new PointCloudXYZI());
    PointCloudXYZI::Ptr current_pc_all(new PointCloudXYZI());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_withoutmoving(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::VoxelGrid<pcl::PointXYZINormal> Voxelg;
    Voxelg.setLeafSize(0.3f, 0.3f, 0.3f);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
    sor.setMeanK(25);
    sor.setStddevMulThresh(1);

    for (int i = 0; i < feats_undistort->size(); i++) // 转换点云至世界坐标系
    {
        if (feats_undistort->points[i].x > 0 || /* abs(ptr->points[i].y) > 30 || */ feats_undistort->points[i].z > 0.5 || feats_undistort->points[i].z < 0)
        {
            V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
            V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
            pcl::PointXYZINormal po;
            po.x = p_global(0);
            po.y = p_global(1);
            po.z = p_global(2);
            po.intensity = feats_undistort->points[i].intensity;

            current_pc_all->points.push_back(po);
        }
        else
        {
            V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
            V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
            pcl::PointXYZINormal po;
            po.x = p_global(0);
            po.y = p_global(1);
            po.z = p_global(2);
            po.intensity = feats_undistort->points[i].intensity;

            current_pc->points.push_back(po);
            current_pc_all->points.push_back(po);
        }
    }
    current_pc->width = 1;
    current_pc->height = current_pc->points.size();

    Voxelg.setInputCloud(current_pc);
    Voxelg.filter(*current_pc);

    sor.setInputCloud(current_pc);
    sor.filter(*current_pc);

    cloud_queue.push(*current_pc);
    pose_queue.push(state_point.pos);
    rot_queue.push(state_point.rot);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZINormal>);

    if (cloud_queue.size() >= 5)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr current_1(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr current_2(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::Indices indices_tmp;
        pcl::Indices indices_moving;

        float Resolution = 1;
        float max_dis = 10.0;
        std::map<int, std::vector<pcl::PointXYZINormal>> map;
        for (int i = 0; i < cloud_queue.front().points.size(); i++)
        {
            double angle = atan2(cloud_queue.front().points[i].y - pose_queue.front().y(), cloud_queue.front().points[i].x - pose_queue.front().x());
            if (angle < 0)
                angle += 2 * M_PI;
            int k = angle / (M_PI / 180) / Resolution;
            map[k].push_back(cloud_queue.front().points[i]);
        }
        cout << "time_1: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

        for (int i = 0; i < int(360 / Resolution); i++)
        {
            if (map.find(i) == map.end())
            {
                double angle = i * (M_PI / 180) * Resolution;

                pcl::PointXYZINormal p;
                p.x = max_dis * cos(angle) + pose_queue.front().x();
                p.y = max_dis * sin(angle) + pose_queue.front().y();
                map[i].push_back(p);
            }
        }
        cout << "time_2: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

        std::vector<bool> cloud_idx(cloud_queue.back().points.size(), false);
        for (std::map<int, std::vector<pcl::PointXYZINormal>>::iterator it = map.begin(); it != map.end(); it++)
        {
            for (int i = 0; i < it->second.size(); i++)
            {
                double k = (it->second[i].y - pose_queue.back().y()) / (it->second[i].x - pose_queue.back().x());
                double b = it->second[i].y - k * it->second[i].x;

                for (int j = 0; j < cloud_queue.back().points.size(); j++)
                {
                    if (cloud_idx[j])
                        continue;
                    if (in_range(it->second[i], pose_queue.front(), cloud_queue.back().points[j], pose_queue.back()))
                    {
                        double dis = abs(k * cloud_queue.back().points[j].x + b - cloud_queue.back().points[j].y) / sqrt(1 + pow(k, 2));
                        if (dis < 0.1)
                        {
                            if (sqrt(pow(cloud_queue.back().points[j].x - it->second[i].x, 2) + pow(cloud_queue.back().points[j].y - it->second[i].y, 2)) < 0.1)
                            {
                                indices_tmp.push_back(j);
                                cloud_idx[j] = true;
                            }
                        }
                    }
                }
            }
        }
        cout << "time_3: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

        pcl::IndicesPtr indices(new std::vector<int>);
        sort(indices_tmp.begin(), indices_tmp.end());
        int index = 0;

        for (int i = 0; i < cloud_queue.back().points.size(); i++)
        {
            if (i != indices_tmp[index])
                cloud_feature->points.push_back(cloud_queue.back().points[i]);
            else
            {
                index++;
            }
        }
        // cout << cloud_queue.back().points.size() << " " << indices_tmp.size() << " " << cloud_feature->points.size() << endl;

        sor.setMeanK(15);
        sor.setInputCloud(cloud_feature);
        sor.filter(*cloud_feature);

        Voxelg.setInputCloud(cloud_feature);
        Voxelg.setLeafSize(1.0f, 1.0f, 10.0f);
        Voxelg.filter(*cloud_feature);

        cout << "time_4: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

        feature_vec.push_back(*cloud_feature);
        pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
        vector<int> pointIdxSearch;         // 保存下标
        vector<float> pointSquaredDistance; // 保存距离

        if (feature_vec.size() > 10)
        {
            std::vector<int> feature_idx(feature_vec[feature_vec.size() - 1].size(), 0);
            for (int i = 0; i < feature_vec[feature_vec.size() - 1].size(); i++)
            {
                for (int j = feature_vec.size() - 2; j >= 0; j--)
                {
                    kdtree.setInputCloud(feature_vec[j].makeShared());
                    kdtree.nearestKSearch(feature_vec[feature_vec.size() - 1][i], 1, pointIdxSearch, pointSquaredDistance);

                    if (feature_vec[feature_vec.size() - 1][i].x - feature_vec[j][pointIdxSearch[0]].x > 0 && pointSquaredDistance[0] < 0.5)
                    {
                        feature_idx[i]++;
                    }
                    if (feature_idx[i] > 6)
                        cloud_feature_->points.push_back(feature_vec[feature_vec.size() - 1][i]);
                }
            }
            feature_vec.erase(feature_vec.begin());
        }

        if (!cloud_feature_->points.empty())
        {
            if (first_delet)
            {
                cloud_delet = cloud_feature_;
                first_delet = false;
            }

            for (int i = 0; i < cloud_feature_->size(); i++)
            {
                auto &a_pt = cloud_feature_->points[i];
                Eigen::Vector3f pt(a_pt.x, a_pt.y, a_pt.z);
                float loc_xyz[3];
                for (int j = 0; j < 3; j++)
                {
                    loc_xyz[j] = pt[j] / voxel_box_size;
                    if (loc_xyz[j] < 0)
                    {
                        loc_xyz[j] -= 1.0; // 保证在float转int时，是向下取整的，如，int(-1.5) = -1，我们希望取整=-2，因此执行减1操作
                    }
                }
                VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
                auto iter = map_feature.find(position);
                if (iter != map_feature.end())
                {
                    iter->second++;
                }
                else
                {
                    map_feature[position] = 10;
                }
            }
            for (auto &a_map : map_feature)
            {
                a_map.second--;
            }
        }

        for (int i = 0; i < cloud_delet->points.size(); i++)
        {
            kdtree.setInputCloud(cloud_delet);
            for (int j = 0; j < cloud_feature->points.size(); j++)
            {
                kdtree.nearestKSearch(cloud_feature->points[i], 1, pointIdxSearch, pointSquaredDistance);
                if (cloud_delet->points[i].x - cloud_feature->points[pointIdxSearch[0]].x > 0 && pointSquaredDistance[0] > 0.1)
                {
                    cloud_delet->points[i] = cloud_feature->points[j];
                }
            }
        }

        std::vector<bool> ptr_idx(feats_undistort->size(), false);
        for (int i = 0; i < cloud_feature->points.size(); i++)
        {
            V3D p_global(cloud_feature->points[i].x, cloud_feature->points[i].y, cloud_feature->points[i].z);
            V3D p_body(state_point.offset_R_L_I.inverse() * ((state_point.rot.inverse() * (p_global - state_point.pos)) - state_point.offset_T_L_I));
            pcl::PointXYZINormal po;
            po.x = p_body(0);
            po.y = p_body(1);
            po.z = p_body(2);

            for (int j = 0; j < feats_undistort->points.size(); j++)
            {
                if (ptr_idx[j])
                    continue;
                if (sqrt(pow(po.x - feats_undistort->points[j].x, 2) +
                         pow(po.y - feats_undistort->points[j].y, 2)) < 0.5)
                {
                    ptr_idx[j] = true;
                    indices_moving.push_back(j);
                }
            }
        }
        cout << "time_6: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

        sort(indices_moving.begin(), indices_moving.end());
        index = 0;
        if (!indices_moving.empty())
            for (int i = 0; i < feats_undistort->points.size(); i++)
            {
                if (i != indices_moving[index])
                    cloud_withoutmoving->points.push_back(feats_undistort->points[i]);
                else
                    index++;
            }
        cout << "time_7: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

        *cloud_out = *current_1 + *current_2;
        cloud_out->width = 1;
        cloud_out->height = cloud_out->points.size();

        cloud_queue.pop();
        pose_queue.pop();
        rot_queue.pop();
    }

    if (!cloud_withoutmoving->empty())
        *feats_undistort = *cloud_withoutmoving;

    return *cloud_feature_;
}