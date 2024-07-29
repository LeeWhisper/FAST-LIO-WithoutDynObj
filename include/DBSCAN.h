#pragma once
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#pragma region 点云密度聚类的PCL代码实现
template <typename PointT>
bool dbscan(const pcl::PointCloud<PointT> &cloud_in, std::vector<pcl::Indices> &cluster_idx, const double &epsilon, const int &minpts)
{
    std::vector<bool> cloud_processed(cloud_in.size(), false);

    for (size_t i = 0; i < cloud_in.size(); ++i)
    {
        if (cloud_processed[i] != false)
        {
            continue;
        }
        pcl::Indices seed_queue;
        // 检查近邻点个数是否大于minpts判断该点是否为核心对象
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        tree->setInputCloud(cloud_in.makeShared());
        pcl::Indices k_indices;
        std::vector<float> k_distances;
        if (tree->radiusSearch(cloud_in.points[i], epsilon, k_indices, k_distances) >= minpts)
        {
            seed_queue.push_back(i);
            cloud_processed[i] = true;
        }
        else
        {
            continue;
        }

        int seed_index = 0;
        while (seed_index < seed_queue.size())
        {
            pcl::Indices indices;
            std::vector<float> dists;
            if (tree->radiusSearch(cloud_in.points[seed_queue[seed_index]], epsilon, indices, dists) < minpts) // 函数返回值为近邻数量
            {
                // 不满足小于minpts的点可能是边界点、噪点、也可能是簇的一部分，不标记为已处理。matlab中是当做噪点标记
                ++seed_index;
                continue;
            }
            for (size_t j = 0; j < indices.size(); ++j)
            {
                if (cloud_processed[indices[j]])
                {
                    continue;
                }
                seed_queue.push_back(indices[j]);
                cloud_processed[indices[j]] = true;
            }
            ++seed_index;
        }

        cluster_idx.push_back(seed_queue);
    }

    if (cluster_idx.size() == 0)
        return false;
    else
        return true;
}
#pragma endregion
