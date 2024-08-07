#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include "use-ikfom.h"
#include <common_lib.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/features/

using namespace std;

int point_filter_num = 1;
float blind = 1.0;

pcl::PointCloud<pcl::PointXYZINormal> pl_1, pl_2;
pcl::PointCloud<pcl::PointXYZINormal> pc_output, pc_laser;

int line_iteration;
int iter;
double dis_thresh;
int num_thresh;

class Vector3
{
public:
    double x, y, z;

    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    // Vector subtraction
    Vector3 operator-(const Vector3 &other) const
    {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    // Cross product
    Vector3 cross(const Vector3 &other) const
    {
        return Vector3(y * other.z - z * other.y,
                       z * other.x - x * other.z,
                       x * other.y - y * other.x);
    }

    // Dot product
    double dot(const Vector3 &other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    // Magnitude (norm)
    double norm() const
    {
        return std::sqrt(dot(*this));
    }
};

/**
 * @brief RANSAC(随机采样一致性算法),基于pcl实现
 * @param pc 输入点云
 * @param iter 迭代次数
 * @param dis_thresh 距离容忍度
 * @return 分割后的平面点云
 */
pcl::PointIndices
Ransac_pcl(const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZINormal>> &pc,
           int iter, double dis_thresh, int num_thresh = -1)
{
    pcl::PointIndices::Ptr out(new pcl::PointIndices);
    int Size_pc = pc->points.size();

    if (Size_pc <= 10) // 点云数量不够不能进行拟合
    {
        return {};
    }

    // 空间直线拟合
    for (int i = 0; i < iter; i++)
    {
        int p1, p2;
        // pcl::PointCloud<pcl::PointXYZINormal>::Ptr Tmp(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        double M, N, P;

        while (true)
        {
            // 随机选取两个点
            p1 = random() % Size_pc;
            p2 = random() % Size_pc;
            if (p1 == p2)
                continue;

            M = pc->points[p2].x - pc->points[p1].x;
            N = pc->points[p2].y - pc->points[p1].y;
            P = pc->points[p2].z - pc->points[p1].z;
            double norm = sqrt(M * M + N * N + P * P);
            M /= norm;
            N /= norm;
            P /= norm;

            if (abs(N) < 0.7)
                continue;

            break;
        }
        inliers->indices.push_back(p1);
        inliers->indices.push_back(p2);

        // 使用两个点，拟合一条空间直线
        //  # M = x2 - x1;
        //  # N = y2 - y1;
        //  # P = z2 - z1;
        //  空间直线方程表达式：(x - x1) / M = (y - y1) / N = (z - z1) / P;

        // 遍历整个输入点云，计算每个点云与直线的距离，并将内点存入临时变量点云中
        for (int j = 0; j < pc->points.size(); j++)
        {
            if (j != p1 && j != p2)
            {
                // 计算点到直线距离
                Vector3 tmp(pc->points[j].x - pc->points[p1].x, pc->points[j].y - pc->points[p1].y, pc->points[j].z - pc->points[p1].z);
                Vector3 tmp2 = tmp.cross(Vector3(M, N, P));
                double dis = tmp2.norm();

                if (dis < dis_thresh)
                    inliers->indices.push_back(j);
            }
        }

        if (inliers->indices.size() > out->indices.size() && inliers->indices.size() > num_thresh)
            out->indices = inliers->indices;
        inliers->indices.clear();
    }

    return *out;
}

void LineFeatureExtract(PointCloudXYZI::Ptr feats_undistort)
{
    clock_t start;
    start = clock();

    // 分割平面
    int num_iterations = 5; // 想要分割的平面数量
    pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZINormal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02); // 平面点到平面的距离阈值

    for (int i = 0; i < num_iterations; ++i)
    {
        seg.setInputCloud(feats_undistort);
        seg.segment(*plane_indices, *coefficients);

        // 如果找到了足够多的内点
        if (plane_indices->indices.size() > 100)
        {
            pcl::ExtractIndices<pcl::PointXYZINormal> extract;
            extract.setInputCloud(feats_undistort);
            extract.setIndices(plane_indices);
            extract.setNegative(true);
            extract.filter(pl_1);
        }
    }
    cout << "time_1: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

    pcl::PointCloud<pcl::PointXYZINormal> tmp;
    for (int i = 0; i < line_iteration; i++)
    {
        pcl::PointIndices index = Ransac_pcl(pl_1.makeShared(), iter, dis_thresh, num_thresh);
        // cout << "index.indices.size(): " << index.indices.size();
        pcl::copyPointCloud(pl_1, index.indices, tmp);
        for (int k = 0; k < tmp.points.size(); k++)
        {
            tmp.points[k].intensity = i;
            pc_output.push_back(tmp.points[k]);
        }

        pcl::ExtractIndices<pcl::PointXYZINormal> extract;
        extract.setInputCloud(pl_1.makeShared());
        extract.setIndices(boost::make_shared<pcl::PointIndices>(index));
        extract.setNegative(true);
        extract.filter(pl_1);
    }

    cout << "time_2: " << (double)(clock() - start) / CLOCKS_PER_SEC << endl;
}