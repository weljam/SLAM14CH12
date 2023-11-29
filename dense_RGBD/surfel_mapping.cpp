//
// Created by gaoxiang on 19-4-25.
//

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/impl/mls.hpp>

// typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointXYZRGBNormal SurfelT;
typedef pcl::PointCloud<SurfelT> SurfelCloud;
typedef pcl::PointCloud<SurfelT>::Ptr SurfelCloudPtr;
// 把基本点云转化成光滑的带有法线信息的点云
// polynomial_order 多项式阶，多项式拟合所用的阶数
// @param input :点云输入
// @param radius :搜索半径
// @param polynomial_order ：阶数
SurfelCloudPtr reconstructSurface(
        const PointCloudPtr &input, float radius, int polynomial_order) {
    pcl::MovingLeastSquares<PointT, SurfelT> mls;// 滑动最小二乘，帮助点云更加光滑
    // 采用kdtree的方式搜索最临近点（因为不同于图片，点云具有无序性、稀疏性）
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    mls.setSearchMethod(tree);
    // 搜索半径
    mls.setSearchRadius(radius);
    // 计算法线
    mls.setComputeNormals(true);
    // 设置近邻高斯权重系数，一般为搜索半径平方
    mls.setSqrGaussParam(radius * radius);
    // 当输入阶数大于1时，采用多项式拟合
    mls.setPolynomialFit(polynomial_order > 1);
    // 设置阶数
    mls.setPolynomialOrder(polynomial_order);
    // 输入点云
    mls.setInputCloud(input);
    SurfelCloudPtr output(new SurfelCloud);
    // 处理结果保存到输出点云并放返回
    mls.process(*output);
    return (output);
}
// 为点云表面添加三角网格面元
pcl::PolygonMeshPtr triangulateMesh(const SurfelCloudPtr &surfels) {
    // 创建搜索树
    pcl::search::KdTree<SurfelT>::Ptr tree(new pcl::search::KdTree<SurfelT>);
    tree->setInputCloud(surfels);

    // 贪婪投影三角形
    pcl::GreedyProjectionTriangulation<SurfelT> gp3;
    // 多边形网格用于存储结果
    pcl::PolygonMeshPtr triangles(new pcl::PolygonMesh);

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.05);

    // 参数取典型值
    // 设置邻域大小
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    // 设置法线偏离最大角度
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    // 设置三角形的最大、最小角度
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    // 设置法线一致性
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud(surfels);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);

    return triangles;
}

int main(int argc, char **argv) {

    // Load the points
    PointCloudPtr cloud(new PointCloud);
    if (argc == 0 || pcl::io::loadPCDFile(argv[1], *cloud)) {
        cout << "failed to load point cloud!";
        return 1;
    }
    cout << "point cloud loaded, points: " << cloud->points.size() << endl;

    // Compute surface elements
    cout << "computing normals ... " << endl;
    double mls_radius = 0.05, polynomial_order = 2;
    auto surfels = reconstructSurface(cloud, mls_radius, polynomial_order);

    // Compute a greedy surface triangulation
    cout << "computing mesh ... " << endl;
    pcl::PolygonMeshPtr mesh = triangulateMesh(surfels);

    cout << "display mesh ... " << endl;
    pcl::visualization::PCLVisualizer vis;
    // 显示面元及其边线
    vis.addPolylineFromPolygonMesh(*mesh, "mesh frame");
    vis.addPolygonMesh(*mesh, "mesh");
    vis.resetCamera();
    vis.spin();
}