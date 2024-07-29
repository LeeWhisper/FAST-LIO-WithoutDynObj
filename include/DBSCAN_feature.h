#pragma once
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#pragma region 根据已知点构建聚类
template <typename PointT>
bool DBSCAN_feature(const pcl::PointCloud<PointT> &cloud_in, const pcl::PointCloud<PointT> &cloud_feature, std::vector<pcl::Indices> &cluster_idx, const double &epsilon, const int &minpts)
{
    std::vector<bool> index_in(cloud_in.size(), false);
    std::vector<bool> index_feature(cloud_feature.size(), false);

    typename pcl::search::KdTree<PointT>::Ptr tree_feature(new pcl::search::KdTree<PointT>());
    tree_feature->setInputCloud(cloud_feature.makeShared());

    typename pcl::search::KdTree<PointT>::Ptr tree_in(new pcl::search::KdTree<PointT>());
    tree_in->setInputCloud(cloud_in.makeShared());

    for (size_t i = 0; i < cloud_feature.size(); ++i)
    {
        if (index_feature[i] != false)
        {
            continue;
        }
        pcl::Indices seed_queue;
        pcl::Indices seed_queue_in;

        pcl::Indices k_indices;
        std::vector<float> k_distances;
        if (tree_in->radiusSearch(cloud_feature.points[i], epsilon, k_indices, k_distances) >= minpts)
        {
            tree_in->nearestKSearch(cloud_feature.points[i], 1, k_indices, k_distances);
            seed_queue_in.push_back(k_indices[0]); // 核心对象添加到队列中
            index_in[k_indices[0]] = true;

            seed_queue.push_back(i);
            index_feature[i] = true;
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
            if (tree_feature->radiusSearch(cloud_feature.points[seed_queue[seed_index]], epsilon, indices, dists) < minpts) // 函数返回值为近邻数量
            {
                // 不满足小于minpts的点可能是边界点、噪点、也可能是簇的一部分，不标记为已处理。matlab中是当做噪点标记
                ++seed_index;
                continue;
            }
            for (size_t j = 0; j < indices.size(); ++j)
            {
                if (index_feature[indices[j]])
                {
                    continue;
                }
                seed_queue.push_back(indices[j]);
                index_feature[indices[j]] = true;
            }
            ++seed_index;
        }

        int seed_index_in = 0;
        while (seed_index_in < seed_queue_in.size())
        {
            pcl::Indices indices;
            std::vector<float> dists;
            if (tree_in->radiusSearch(cloud_in.points[seed_queue_in[seed_index_in]], epsilon, indices, dists) < minpts) // 函数返回值为近邻数量
            {
                // 不满足小于minpts的点可能是边界点、噪点、也可能是簇的一部分，不标记为已处理。matlab中是当做噪点标记
                ++seed_index_in;
                continue;
            }
            for (size_t j = 0; j < indices.size(); ++j)
            {
                if (index_in[indices[j]])
                {
                    continue;
                }
                seed_queue_in.push_back(indices[j]);
                index_in[indices[j]] = true;
            }
            ++seed_index_in;
        }
        

        cluster_idx.push_back(seed_queue_in);
    }

    if (cluster_idx.size() == 0)
        return false;
    else
        return true;
}
#pragma endregion
