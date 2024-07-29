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

std::queue<pcl::PointCloud<pcl::PointXYZINormal>> cloud_queue;
std::queue<MTK::vect<3, double>> pose_queue;
std::queue<MTK::SubManifold<SO3, 3, 3>> rot_queue;

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

    if (point_1.x * point_2.x > 0 && point_1.y * point_2.y > 0  && dis_2 < dis_1)
        return true;
    return false;
}

visualization_msgs::MarkerArray deletMovingObj(PointCloudXYZI::Ptr feats_undistort, state_ikfom state_point)
{
    clock_t start, end;
    start = clock();
    visualization_msgs::MarkerArray markerArray;

    PointCloudXYZI::Ptr current_pc(new PointCloudXYZI());
    PointCloudXYZI::Ptr current_pc_all(new PointCloudXYZI());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_withoutmoving(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::VoxelGrid<pcl::PointXYZINormal> Voxelg;
    Voxelg.setLeafSize(0.3f, 0.3f, 0.3f);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
    sor.setMeanK(25);
    sor.setStddevMulThresh(1);

    // sor.setInputCloud(feats_undistort);
    // sor.filter(*feats_undistort); // 去除噪点

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
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_featrue(new pcl::PointCloud<pcl::PointXYZINormal>);

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
            else
            {
                // double sum_x = 0, sum_y = 0;
                // for (int j = 0; j < map[i].size(); j++)
                // {
                //     sum_x += map[i][j].x;
                //     sum_y += map[i][j].y;
                // }
                // pcl::PointXYZINormal p;
                // p.x = sum_x / map[i].size();
                // p.y = sum_y / map[i].size();
                // map[i].clear();
                // map[i].push_back(p);
            }
        }
        cout << "time_2: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

        // for (std::map<int, std::vector<pcl::PointXYZINormal>>::iterator it = map.begin(); it != map.end(); it++)
        // {
        //     for (int i = 0; i < it->second.size(); i++)
        //     {
        // it->second[i].intensity = 100;
        // current_1->points.push_back(it->second[i]); // 上一帧的点云以及构建的虚拟点
        //     }
        // }

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
                        if (dis < 0.5)
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

        pcl::IndicesPtr indices;
        sort(indices_tmp.begin(), indices_tmp.end());
        int index = 0;

        for (int i = 0; i < cloud_queue.back().points.size(); i++)
        {
            if (i != indices_tmp[index])
                cloud_featrue->points.push_back(cloud_queue.back().points[i]);
            else
            {
                index++;
                // indices->push_back(i);
            }
        }
        cout << cloud_queue.back().points.size() << " " << indices_tmp.size() << " " << cloud_featrue->points.size() << endl;

        sor.setMeanK(15);
        sor.setInputCloud(cloud_featrue);
        sor.filter(*cloud_featrue);

        Voxelg.setInputCloud(cloud_featrue);
        Voxelg.setLeafSize(1.0f, 1.0f, 10.0f);
        Voxelg.filter(*cloud_featrue);

        pcl::KdTreeFLANN<pcl::PointXYZINormal> kdtree;
        vector<int> pointIdxSearch;         // 保存下标
        vector<float> pointSquaredDistance; // 保存距离

        kdtree.setInputCloud(current_pc_all);
        for (int i = 0; i < cloud_featrue->size(); i++)
        {
            kdtree.radiusSearch(cloud_featrue->points[i], 0.5, pointIdxSearch, pointSquaredDistance);
            // 1. 计算点云的均值
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (const auto &id : pointIdxSearch)
            {
                mean += Eigen::Vector3d(current_pc_all->points[id].x, current_pc_all->points[id].y, current_pc_all->points[id].z);
            }
            mean /= pointIdxSearch.size();

            // 2. 去中心化数据
            Eigen::MatrixXd centered(pointIdxSearch.size(), 3);
            for (size_t j = 0; j < pointIdxSearch.size(); ++j)
            {
                centered.row(j) = Eigen::Vector3d(current_pc_all->points[j].x, current_pc_all->points[j].y, current_pc_all->points[j].z) - mean;
            }

            // 3. 计算协方差矩阵
            Eigen::Matrix3d covariance = centered.transpose() * centered / double(pointIdxSearch.size());

            // 4. 特征值分解
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
            Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();   // PCA特征值，顺序从小到大
            Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors(); // PCA特征向量，按特征值顺序排列

            cout << eigen_values[1] / eigen_values[2] << ", ";

            visualization_msgs::Marker marker;
            marker.header.frame_id = "camera_init";
            marker.header.stamp = ros::Time::now();
            marker.ns = "basic_shapes";
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id = i;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            marker.scale.z = 0.5;
            marker.color.b = 0;
            marker.color.g = 0;
            marker.color.r = 255;
            marker.color.a = 1;

            geometry_msgs::Pose pose;
            pose.position.x = cloud_featrue->points[i].x;
            pose.position.y = cloud_featrue->points[i].y;
            pose.position.z = cloud_featrue->points[i].z;

            ostringstream str;
            str << eigen_values[1] / eigen_values[2];
            marker.text = str.str();
            marker.pose = pose;

            markerArray.markers.push_back(marker);

            // if (eigen_values[1] / eigen_values[2] > 0.1)
            //     cloud_featrue->erase(cloud_featrue->points.begin() + i);
        }
        cout << endl;
        cout << "time_4: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

        std::vector<bool> ptr_idx(feats_undistort->size(), false);
        for (int i = 0; i < cloud_featrue->points.size(); i++)
        {
            V3D p_global(cloud_featrue->points[i].x, cloud_featrue->points[i].y, cloud_featrue->points[i].z);
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

    return markerArray;
}