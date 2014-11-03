// PCL_Test1.cpp: define el punto de entrada de la aplicación de consola.
//

/*#include <pcl/range_image/range_image.h>

int main(int argc, char** argv) {
	pcl::PointCloud<pcl::PointXYZ> pointCloud;

	// Generate the data
	int lc = 0;
	for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
		for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
			pcl::PointXYZ point;
			point.x = 2.0f - y;
			point.y = y;
			point.z = z;
			pointCloud.points.push_back(point);
			++lc;
		}
	}
	pointCloud.width = (uint32_t)pointCloud.points.size();
	pointCloud.height = 1;

	// We now want to create a range image from the above point cloud, with a 1deg angular resolution
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;

	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	std::cout << rangeImage << "\n";
}*/

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int user_data;

void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;

}

void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile("./pcd/model.pcd", *cloud);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	viewer.showCloud(cloud);

	//use the following functions to get access to the underlying more advanced/powerful
	//PCLVisualizer

	//This will only get called once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//This will get called once per visualization iteration
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//you can also do cool processing here
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
		user_data++;
	}
	return 0;
}