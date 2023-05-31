#include "normality_detection/NormalityDetection.h"

// #define NEW_DATA

// TODO revisit arguments names
// TODO downsampling
// TODO Statistical Outlier Removal
// TODO use integral images to get the cloud normals -> apply pass through filter to raw cloud -> get original indices of the resulting/removed cloud points -> remove the correspoding normals from the normals cloud => WHY? Waaaaaaay faster to estimate the normals from an ordered pointcloud file [width x height]
// TODO separate filtering algos, plane fitting, io/basic operations into 3 main classes

NormalityDetection::NormalityDetection()
{
}

NormalityDetection::~NormalityDetection()
{
}

void NormalityDetection::readDir(std::string dirIn, bool verbose)
{
	if (dirIn.empty())
	{
		std::cerr << "[IO-ReadDir] Empty directory passed. Exiting." << std::endl;
		// return;
		exit(1);
	}
	for (const auto &entry : std::filesystem::directory_iterator(dirIn))
	{
		std::string fileName = entry.path().string();
		this->allPcdList.push_back(fileName);
		// if (verbose)
		// {
		// std::cout << fileName << std::endl;
		// }
		if (fileName.find("filtered_directory") != std::string::npos)
		{
			this->dirFiltered = true;
		}
	}
	if (verbose)
	{
		std::cout << "\n[IO-ReadDir] Found " << this->allPcdList.size() << " files in directory [" << dirIn << "]." << std::endl;
	}
	if (this->dirFiltered)
	{
		if (verbose)
		{
			std::cout << "\n[IO-ReadDir] Files in directory [" << dirIn << "] have been already filtered. Skipping filtering step." << std::endl;
		}
		this->getFileList("filtered", this->filteredList, dirIn, true);
	}
	else
	{
		this->getFileList("raw", this->rawList, dirIn, true);
	}
}

void NormalityDetection::getFileList(std::string searchString, std::vector<std::string> &fileListOut, std::string dirIn, bool verbose)
{
	std::cout << "\nReading files with [" << searchString << "] string in the directory [" << dirIn << "]." << std::endl;
	for (auto &fileName : this->allPcdList)
	{
		if (fileName.find(".pcd") != std::string::npos)
		{
			if (fileName.find(searchString) != std::string::npos)
			{
				fileName.erase(fileName.size() - 4);
				if (verbose)
				{
					std::cout << fileName << std::endl;
				}
				fileListOut.push_back(fileName);
			}
		}
	}
	if (verbose)
	{
		if (fileListOut.empty())
			std::cout << "No files with [" << searchString << "] string were found." << std::endl;
	}
}

void NormalityDetection::filterPcd(std::vector<std::string> pcdFileList, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	for (const auto &pcdFile : pcdFileList)
	{
		// this->readPcd("", pcdFile + ".pcd", cloud, false);
		// this->passThroughFilterZ(cloud, zFilteredCloud, passThroughZ);
		// this->estimateNormalsOMP(zFilteredCloud, cloudNormals);
	}
}

void NormalityDetection::readPcd(std::string pcdFile, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool verbose)
{

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFile, *cloud) == -1) //* load the file
	{
		PCL_ERROR("\n[IO-ReadPcd] Couldn't read file %s\n", pcdFile.c_str());
		return;
	}
	if (verbose)
	{
		std::cout << "\n[IO-ReadPcd] Loaded ";
		std::cout << cloud->width << "x" << cloud->height << " ";
		std::cout << cloud->width * cloud->height;
		std::cout << " data points from [" << pcdFile << "]." << std::endl;
		// for (const auto &point : *cloud)
		// 	std::cout << "    " << point.x
		// 			  << " " << point.y
		// 			  << " " << point.z << std::endl;
	}
}

pcl::visualization::PCLVisualizer::Ptr NormalityDetection::createVisualizer(std::string viewerTitle)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewerTitle));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(0.10);
	viewer->initCameraParameters();
	viewer->setCameraClipDistances(0.001, 10000);
	// std::cout << "Loading cloud " << cloudId << " in viewer " << viewerTitle << std::endl;
	return viewer;
}

void NormalityDetection::passThroughFilterZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, bool verbose)
{
	pcl::PassThrough<pcl::PointXYZ>::Ptr passThroughZ(new pcl::PassThrough<pcl::PointXYZ>); // filtering object
	passThroughZ->setInputCloud(cloudIn);
	// points having values outside this interval for FieldName will be discarded
	passThroughZ->setFilterFieldName("z");

// old data
#ifndef NEW_DATA
	passThroughZ->setFilterLimits(0, 0.12);
#endif // NEW_DATA

// new data
#ifdef NEW_DATA
	passThroughZ->setFilterLimits(-0.2, -0.01);
#endif // NEW_DATA

	passThroughZ->filter(*cloudOut);

	if (verbose)
	{
		std::cout << "[PassThroughFilterZ] Removed " << (float)(cloudIn->size() - cloudOut->size()) / (float)cloudIn->size() * 100 << "% of data points." << std::endl;
		std::cout << "Output cloud size is " << cloudOut->width << "x" << cloudOut->height << std::endl;
	}
}

void NormalityDetection::passThroughFilterX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, bool verbose)
{
	pcl::PassThrough<pcl::PointXYZ>::Ptr passThroughZ(new pcl::PassThrough<pcl::PointXYZ>); // filtering object
	passThroughZ->setInputCloud(cloudIn);
	passThroughZ->setFilterFieldName("x");
	// points having values outside this interval for FieldName will be discarded
	passThroughZ->setFilterLimits(-0.5, 0.5);
	passThroughZ->filter(*cloudOut);
	// this->_passThroughZ->setNegative(true)
	if (verbose)
	{
		std::cout << "[PassThroughFilterX] Removed " << (float)(cloudIn->size() - cloudOut->size()) / (float)cloudIn->size() * 100 << "% of data points." << std::endl;
		std::cout << "Output cloud size is " << cloudOut->width << "x" << cloudOut->height << std::endl;
	}
}

void NormalityDetection::passThroughFilterY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, bool verbose)
{
	pcl::PassThrough<pcl::PointXYZ>::Ptr passThroughZ(new pcl::PassThrough<pcl::PointXYZ>); // filtering object
	passThroughZ->setInputCloud(cloudIn);
	passThroughZ->setFilterFieldName("y");
	// points having values outside this interval for FieldName will be discarded
	passThroughZ->setFilterLimits(-0.5, 0.5);
	passThroughZ->filter(*cloudOut);
	// this->_passThroughZ->setNegative(true)
	if (verbose)
	{
		std::cout << "[PassThroughFilterY] Removed " << (float)(cloudIn->size() - cloudOut->size()) / (float)cloudIn->size() * 100 << "% of data points." << std::endl;
		std::cout << "Output cloud size is " << cloudOut->width << "x" << cloudOut->height << std::endl;
	}
}

void NormalityDetection::writePcd(std::string dir, std::string pcdOut, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool verbose)
{
	pcl::io::savePCDFileASCII(dir + pcdOut, *cloud);

	if (verbose)
	{
		std::cout << "Saved ";
		std::cout << cloud->width << "x" << cloud->height << " ";
		std::cout << cloud->size() << " data points to [" << pcdOut << "]." << std::endl;
		// for (const auto &point : *cloud)
		// 	std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
	}
}

void NormalityDetection::segPlane(pcl::PointIndicesPtr inliersIndices, pcl::ModelCoefficientsPtr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, bool verbose)
{
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setInputCloud(cloud);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.0005);
	seg.segment(*inliersIndices, *coefficients);

	if (inliersIndices->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
		return;
	}

	if (verbose)
	{
		std::cerr << "Model coefficients: " << coefficients->values[0] << " "
				  << coefficients->values[1] << " "
				  << coefficients->values[2] << " "
				  << coefficients->values[3] << std::endl;

		// std::cerr << "Model inliersIndices: " << inliersIndices->indices.size() << std::endl;
		// for (const auto &idx : inliersIndices->indices)
		// 	std::cerr << idx << "    " << cloud->points[idx].x << " "
		// 			  << cloud->points[idx].y << " "
		// 			  << cloud->points[idx].z << std::endl;
	}
}

void NormalityDetection::estimateNormalsIntegralImages(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloudNormals, bool verbose)
{
	// for ordered cloud w*h
	// Create the normal estimation class, and pass the input dataset to it
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr ne(new pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>);
	// this->configNormalsEstimationIntegralImages(ne, 0.02f, 10.0f);
	ne->setInputCloud(cloudIn);
	ne->setNormalEstimationMethod(ne->AVERAGE_3D_GRADIENT);
	ne->setMaxDepthChangeFactor(0.02f);
	ne->setNormalSmoothingSize(10.0f);
	// Compute the features
	ne->compute(*cloudNormals);
	if (verbose)
	{
		std::cout << "\n[NormalsEstimationIntegralImages] Normals cloud size: " << cloudNormals->size() << " " << cloudNormals->width << "x" << cloudNormals->height << " point normals." << std::endl;
	}
}

void NormalityDetection::estimateNormalsOMP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloudNormals, bool verbose)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>::Ptr neOmp(new pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>);

	// Create an empty kdtree representation, and pass it to the normal estimation object
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	// this->configNormalsEstimationOMP(neOmp, tree, 1000);
	neOmp->setInputCloud(cloudIn);
	neOmp->setSearchMethod(tree);
	// Use all neighbors in a sphere of radius 3cm or 50 neighbour points
	// neOmp->setRadiusSearch(0.1);
	neOmp->setKSearch(5000);

	// Compute the features
	neOmp->compute(*cloudNormals);

	if (verbose)
	{
		std::cout << "\n[NormalsEstimationOMP] Normals cloud size: " << cloudNormals->size() << " " << cloudNormals->width << "x" << cloudNormals->height << " point normals." << std::endl;
	}
}

void NormalityDetection::regionGrowingClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloudNormals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colouredCloud, std::vector<pcl::PointIndices> &clusters, bool verbose)
{
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	if (cloudNormals->empty())
	{
		std::cout << "Empty normals cloud passed to region growing clustering. Returning empty vector of clusters." << std::endl;
	}

	if (cloudIn->empty())
	{
		std::cout << "Empty point cloud passed to region growing clustering. Returning empty vector of clusters." << std::endl;
	}

	reg.setInputCloud(cloudIn);
	reg.setInputNormals(cloudNormals);
	reg.setSearchMethod(tree);

// old data
#ifndef NEW_DATA
	reg.setMinClusterSize(20);
	reg.setMaxClusterSize(1000000);
	reg.setNumberOfNeighbours(30);
	reg.setSmoothnessThreshold(1.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);
#endif // NEW_DATA

// new data
#ifdef NEW_DATA
	reg.setMinClusterSize(10);
	reg.setMaxClusterSize(100000);
	reg.setNumberOfNeighbours(10);
	reg.setSmoothnessThreshold(0.025 / 180.0 * M_PI);
	reg.setCurvatureThreshold(0.025);
#endif // NEW_DATA

	reg.extract(clusters);
	colouredCloud = reg.getColoredCloud();

	if (verbose)
	{
		std::cout << "\n[Region Growing Clustering] Number of clusters is equal to " << clusters.size() << std::endl;
		// std::cout << "First cluster has " << clusters[0].indices.size() << " points." << std::endl;
		// std::cout << "These are the indices of the points of the initial" << std::endl
		// 		  << "cloud that belong to the first cluster:" << std::endl;

		// int counter = 0;
		// while (counter < clusters[0].indices.size())
		// {
		// 	std::cout << clusters[0].indices[counter] << ", ";
		// 	counter++;
		// 	if (counter % 10 == 0)
		// 		std::cout << std::endl;
		// }
		// std::cout << std::endl;
	}

	// return {clusters, *coloredCloud};
}

pcl::PointCloud<pcl::PointXYZ> NormalityDetection::XYZRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB> cloudIn)
{
	pcl::PointCloud<pcl::PointXYZ> cloudOut;
	cloudOut.header = cloudIn.header;
	cloudOut.is_dense = cloudIn.is_dense;
	cloudOut.height = cloudIn.height;
	cloudOut.width = cloudIn.width;
	for (const auto &pointIn : cloudIn)
	{
		pcl::PointXYZ pointOut;
		pointOut.x = pointIn.x;
		pointOut.y = pointIn.y;
		pointOut.z = pointIn.z;
		cloudOut.points.push_back(pointOut);
	}
	return cloudOut;
}

void NormalityDetection::pointCloudFromIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr mainCloud, pcl::PointIndices extractIndices, pcl::PointCloud<pcl::PointXYZ>::Ptr extractCloud, bool verbose)
{
	for (const auto &index : extractIndices.indices)
	{
		extractCloud->push_back(mainCloud->points[index]);
	}
	extractCloud->width = extractCloud->size();
	extractCloud->height = 1;
	extractCloud->is_dense = true;
	if (verbose)
	{
		std::cout << "[pointCloudFromIndices] Extracted Point Cloud with size " << extractCloud->width << "x" << extractCloud->height << "." << std::endl;
	}
}

int NormalityDetection::getLargestClusterIndex(std::vector<pcl::PointIndices> clusters, bool verbose)
{
	// don't start if the cluster is empty
	if (clusters.empty())
		return -1;

	std::vector<unsigned long> sizes;
	int index;

	for (const auto &cluster : clusters)
	{
		sizes.push_back(cluster.indices.size());
		// std::cout << cluster.indices.size() << std::endl;
	}
	auto it = std::max_element(sizes.begin(), sizes.end());

	// // if the iterator points to the end -> then the max element is at the end (size-1)
	// if (it == sizes.end())
	// {
	// 	index = sizes.size()-1;
	// }

	// if not all the above -> the max element is either 0 or n, where n < size-1
	// get the distance of iterator from the beginning of vector
	index = std::distance(sizes.begin(), it);
	if (verbose)
	{
		std::cout << "Index of maximum size cluster is " << index << std::endl;
		std::cout << "Size = " << *it << "x1" << std::endl;
	}
	return index;
}

Eigen::Matrix3f NormalityDetection::calcRotationMatrix(Eigen::Vector3f planeNormal, bool verbose)
{
	Eigen::Matrix3f rot_mat;
	rot_mat.col(1) = planeNormal.cross(Eigen::Vector3f::UnitX()) / (planeNormal.cross(Eigen::Vector3f::UnitX())).norm();
	rot_mat.col(0) = (rot_mat.col(1)).cross(planeNormal);
	rot_mat.col(2) = planeNormal;
	if (verbose)
	{
		std::cout << std::endl
				  << std::setprecision(4)
				  << rot_mat << std::endl
				  << std::endl;
	}
	return rot_mat;
}

short NormalityDetection::getIndex(std::vector<std::string> v, std::string element)
{
	if (v.empty())
		return -1;

	auto it = std::find(v.begin(), v.end(), element);
	int index;

	index = it - v.begin();
	// std::cout << "index " << index << " ";
	return index;
}

void NormalityDetection::printProgress(std::vector<std::string> v, std::string element)
{
	short index = static_cast<float>(this->getIndex(v, element) + 1);
	std::cout << std::setprecision(4) << "[" << static_cast<float>(index) / static_cast<float>(v.size()) * 100 << "%] "
			  << "File " << index << "/" << v.size() << std::endl;
	if (index == v.size())
	{
		std::cout << "Done!" << std::endl;
	}
}
