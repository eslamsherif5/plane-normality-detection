#include "normality_detection/NormalityDetection.h"

#ifndef PCD_DIR
// use pcl visualizer in case of dealing with a single .pcd file
#define VIS
#endif // PCD_DIR

int main(int argc, char **argv)
{
    // Init
    std::string dirIn, output_csv;

// if PCD_DIR is not defined --> pass a .pcd file to the executable
#ifndef PCD_DIR
    if (!argv[1])
    {
        std::cerr << "No .pcd file passed in arguments" << std::endl;
        exit(1);
    }
#endif // PCD_DIR

// if PCD_DIR is defined --> pass a directory of .pcd files to the executable
#ifdef PCD_DIR
    if (!argv[1])
    {
        std::cerr << "No directory passed in arguments" << std::endl;
        exit(1);
    }
#endif // PCD_DIR

// check for output .csv file name only if we are passing a directory
#ifdef PCD_DIR
    if (!argv[2])
    {
        std::cerr << "Output file [.csv] is not passed in the arguments list" << std::endl;
        exit(1);
    }
    output_csv = argv[2];
#endif // PCD_DIR

    dirIn = argv[1];

    NormalityDetection nd;

    // Reference Plane (for visualization only)
    pcl::ModelCoefficientsPtr refPlaneCoeffs(new pcl::ModelCoefficients);
    // 0x + 0y + 1z = 0 -> z normal on xy plane
    Eigen::Vector3f refNormal = Eigen::Vector3f::UnitZ();
    refPlaneCoeffs->values.push_back(refNormal[0]);
    refPlaneCoeffs->values.push_back(refNormal[1]);
    refPlaneCoeffs->values.push_back(refNormal[2]);
    refPlaneCoeffs->values.push_back(0);

#ifdef PCD_DIR
    // create .csv file
    int i = 1;
    csvfile csv(output_csv + ".csv"); // throws exceptions!
    // Header
    csv << "PCD_file"
        << "x"
        << "y"
        << "z" << endrow;

    // read all files in the given directory
    nd.readDir(dirIn, VERBOSE);
    // Check if the directory was already filtered.
    // If yes, skip the preprocessing/filtration routine
    // and read the filtered .pcd files directly
    std::vector<std::string> list;

    if (nd.dirFiltered)
        list = nd.filteredList;
    else
        list = nd.rawList;

    for (const auto &pcdFile : list)
    {
#endif // PCD_DIR

        // IO
        pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Preprocessing
        pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessedCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Filtering
        pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colouredCloud(new pcl::PointCloud<pcl::PointXYZRGB>); // for visualization only
        std::vector<pcl::PointIndices> regionGrowingClusters;                                        // list of all cluster indices
        int idx;
        pcl::PointCloud<pcl::PointXYZ>::Ptr largestClusterCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Plane fitting
        pcl::PointIndicesPtr planeInliers(new pcl::PointIndices);
        pcl::ModelCoefficientsPtr planeCoeffs(new pcl::ModelCoefficients);

#ifndef PCD_DIR
        std::string pcdFile = dirIn + "";
#endif // PCD_DIR

#ifdef PCD_DIR
        if (!(nd.dirFiltered))
        {
#endif
            // 1. [IO]/ Reading .pcd file(s)
            nd.readPcd(pcdFile, rawCloud, VERBOSE);

            // 2. [Preprocessing]/ specifying a region of interest

            nd.passThroughFilterZ(rawCloud, preprocessedCloud, VERBOSE);
            nd.passThroughFilterX(preprocessedCloud, VERBOSE);
            nd.passThroughFilterY(preprocessedCloud, VERBOSE);
#ifndef NEW_DATA
            nd.downsampleCloud(preprocessedCloud, VERBOSE);
#endif //NEW_DATA

            // 3. [Filtering]/ Currently using region growing clustering. We might need to implement/try other algos
            // TODO: implement a standalone function for the region growing algo containing step [3]

            // Apply Region Growing clustering algo
            // 3.1 Get cloud normals
            nd.estimateNormalsOMP(preprocessedCloud, cloudNormals, VERBOSE);

            // 3.2 Get all clusters out of region growing algo
            nd.regionGrowingClustering(preprocessedCloud, cloudNormals, colouredCloud, regionGrowingClusters, VERBOSE);

            // 3.3 Get largest cluster
            // hypothetically, the cluster that's most likely representing the plane
            // after removing most of the outliers and noise
            idx = nd.getLargestClusterIndex(regionGrowingClusters, VERBOSE);
            nd.pointCloudFromIndices(preprocessedCloud, regionGrowingClusters[idx], largestClusterCloud, VERBOSE);

#ifdef PCD_DIR
            // IO]/ Write largest cluster to a new .pcd file to avoid filtering everytime
            nd.writePcd("", pcdFile.substr(0, pcdFile.find("raw")) + "filtered.pcd", largestClusterCloud, VERBOSE);
        }
        else
        {
            // read .pcd files that end with 'filtered' and store in largestClusterCloud
            nd.readPcd(pcdFile, largestClusterCloud, VERBOSE);
        }
#endif // PCD_DIR

        // 4. [Plane fitting]/ get a plane using RANSAC
        // fit a plane using the largest cluster we got in the previous step
        nd.segPlane(planeInliers, planeCoeffs, largestClusterCloud, VERBOSE);
        // 4.1 get the normal to the fitted plane
        Eigen::Vector3f planeNormal;
        planeNormal[0] = planeCoeffs->values[0], planeNormal[1] = planeCoeffs->values[1], planeNormal[2] = planeCoeffs->values[2];

        // 5. [Pose estimation]/ get a rotation matrix(->then euler angles) based on the
        Eigen::Matrix3f rot_mat = nd.getRotationMatrix(planeNormal, VERBOSE);
        Eigen::Vector3f euler = nd.getEulerAngles(rot_mat, VERBOSE);
        // // Create individual rotation matrices for each axis
        // Eigen::AngleAxisf rollAngle(euler[0], Eigen::Vector3f::UnitX());
        // Eigen::AngleAxisf pitchAngle(euler[1], Eigen::Vector3f::UnitY());
        // Eigen::AngleAxisf yawAngle(euler[2], Eigen::Vector3f::UnitZ());

        // // Combine the individual rotation matrices
        // Eigen::Matrix3f rotationMatrix(rollAngle.toRotationMatrix() * pitchAngle.toRotationMatrix() * yawAngle.toRotationMatrix());

        // // Eigen::Quaterniond quaternion = rollAngle * pitchAngle * yawAngle;

        // // Convert the quaternion to a rotation matrix
        // // Eigen::Matrix3d rotationMatrix = quaternion.matrix();

        // std::cout << std::endl
        //           << rotationMatrix << std::endl
        //           << std::endl;

        // Eigen::Vector3f axis = refNormal.normalized().cross(planeNormal.normalized()).normalized();
        // float angle = std::acos(refNormal.normalized().dot(planeNormal.normalized()));
        // Eigen::AngleAxisf rotation(angle, axis);
        // Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix();
        // std::cout << std::endl
        // 		  << std::setprecision(4)
        // 		  << rotationMatrix << std::endl
        // 		  << std::endl;

        // Eigen::Vector3f euler2 = rotationMatrix.eulerAngles(0, 1, 2) * 180.0 / M_PI;

        // std::cout << std::endl
        //           << std::setprecision(6) << euler2[0] << ", " << euler2[1] << ", " << euler2[2] << std::endl
        //           << std::endl;

        // // planeNormal.cross(refNormal) / (planeNormal.cross(refNormal)).norm();
        // std::cout << std::endl
        //           << std::setprecision(6)
        //             << refNormal.normalized().dot(planeNormal.normalized())
        //           << std::endl;

        // // calculate the resultant angle (distance) between
        // float angle = std::acos(refNormal.dot(planeNormal) / (refNormal.norm() * planeNormal.norm())) * 180 / M_PI;

        // // std::cout << pcdFile.substr(0, pcdFile.find("raw")) + "filtered.pcd" << ": " << angle << std::endl;

        // std::cout << "angle = " << angle << std::endl;
#ifdef PCD_DIR
        // 6. [IO]/[Postprocessing]
        csv << pcdFile.substr(0, pcdFile.find("raw")) + "filtered.pcd" << euler[0] << euler[1] << euler[2] << endrow;

        nd.printProgress(list, pcdFile);

        // write a new file as a flag stating that the directory is filtered. you need to delete this file if you want to re-filter in PCD_DIR mode
        std::ofstream outfile(dirIn + "filtered_directory");
        outfile << "" << std::endl;
        outfile.close();
    }
#endif // PCD_DIR

#ifdef VIS
    pcl::visualization::PCLVisualizer::Ptr visRaw = nd.createVisualizer("Raw");
    visRaw->addPointCloud(rawCloud, "raw_cloud");
    visRaw->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "raw_cloud");

    pcl::visualization::PCLVisualizer::Ptr visNormals = nd.createVisualizer("Normals");
    visNormals->addPointCloud<pcl::PointXYZ>(preprocessedCloud, "preprocessed_cloud");
    visNormals->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "preprocessed_cloud");
    visNormals->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(preprocessedCloud, cloudNormals, 50, 0.0085, "normals_cloud");
    visNormals->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "normals_cloud");
    visNormals->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 2, "normals_cloud");

    pcl::visualization::PCLVisualizer::Ptr visRegionGrowing = nd.createVisualizer("Region Growing");
    visRegionGrowing->addPointCloud<pcl::PointXYZ>(largestClusterCloud, "largest_cluster");
    visRegionGrowing->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "largest_cluster");
    visRegionGrowing->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "largest_cluster");
    pcl::PointXYZ p1_fitted(planeNormal.x(), planeNormal.y(), planeNormal.z());
    visRegionGrowing->addLine(pcl::PointXYZ(0, 0, 0), p1_fitted, 1.0, 1.0, 0.0, "fitted_plane_normal");
    visRegionGrowing->addPlane(*planeCoeffs, "fitted_plane");
    visRegionGrowing->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 5.0, "fitted_plane_normal");
    visRegionGrowing->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "fitted_plane");
    pcl::PointXYZ p1_ref(refNormal.x(), refNormal.y(), refNormal.z());
    visRegionGrowing->addLine(pcl::PointXYZ(0, 0, 0), p1_ref, 0.0, 1.0, 0.0, "ref_plane_normal");
    visRegionGrowing->addPlane(*refPlaneCoeffs, "ref_plane");
    visRegionGrowing->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 5.0, "ref_plane_normal");
    visRegionGrowing->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "ref_plane");

    pcl::visualization::PCLVisualizer::Ptr visRegionGrowingColoured = nd.createVisualizer("Region Growing Coloured Segments");
    visRegionGrowingColoured->addPointCloud<pcl::PointXYZRGB>(colouredCloud, "coloured_cloud");
    visRegionGrowingColoured->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "coloured_cloud");

    while (!visRaw->wasStopped()) // || !nd.viewerStatus(viewer2))
    {
        visRaw->spinOnce(100);
        visNormals->spinOnce(100);
        visRegionGrowing->spinOnce(100);
        visRegionGrowingColoured->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
#endif // VIS

    return 0;
}