#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_traits.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

// Include pose estimation library
#include <PoseEstimation/DescriptorUtil.h>
#include <PoseEstimation/PoseEstimation.h>

#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace PoseEstimation;
using namespace boost::property_tree;


typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT>::Ptr CloudT;

std::vector<cv::Mat> image_depth;
std::vector<cv::Mat> image_rgb;
std::vector<ros::Time> time_vector;

KMatrix<> detect (CloudT scene, std::vector<std::string> objects);
int findClosestFrameNumber(ros::Time t, std::vector<ros::Time> vector);
void getPose(int i);
void loadBag(const std::string &filename) 
{
	rosbag::Bag bag;
	bag.open(filename, rosbag::bagmode::Read);

	std::string l_cam_image = "camera0/rgb/image_raw";
	std::string r_cam_image = "camera0/depth_registered/image_raw";


	std::vector<std::string> topics;
	topics.push_back(l_cam_image);
	topics.push_back(r_cam_image);
	std::cout << topics.size() << std::endl;
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	std::cout << view.size() << std::endl;
	int d = 0, r = 0;

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		cv::Mat image;

		//		std::cout << m.getTime() << std::endl;
		sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
		if (image_msg != NULL && image_msg->encoding == "16UC1") { //depth image
			cv_bridge::CvImagePtr depth_image_ptr;
			time_vector.push_back(m.getTime());
			try {
				depth_image_ptr = cv_bridge::toCvCopy(image_msg ,sensor_msgs::image_encodings::TYPE_16UC1); //cv_bridge::toCvCopy(image_msg);
				image = depth_image_ptr->image;

				image_depth.push_back(image);
			} catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		} else {
			cv_bridge::CvImageConstPtr color_image_ptr;
			try {
				color_image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
				r++;
			} catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}

			image = color_image_ptr->image;
			image_rgb.push_back(image);
		}
	}

	std::cout << "d: " << image_depth.size() << ", r: " << image_rgb.size() << std::endl;
	std::cout << "time size: " << time_vector.size() << std::endl;
	//	std::string file = "/home/lilita/ACAT_git/PickAndPlace/Jar_PotYellow/adt_pick_and_place.xml";

	//	ptree tree;
	//	read_xml(file, tree);

	//	std::string main_object_name = tree.get_child("action-primitive").get_child("main-object").get<std::string>("name");
	//    std::string main_object_name = tree.get<std::string>("action-primitive.main-object.name");
	//    std::string time_stamp = tree.get<std::string>("action-primitive.action-chunks.action-chunk.main-object-act.start-point.timestamp");

	//    tree.put<std::string>("action-primitive.main-object.name", name);
	//    tree.put<std::string>("action-primitive.<xmlattr>.version","2.0");

	//	std::cout << main_object_name << std::endl;
	//	std::cout << "time:stamp: " << time_stamp << std::endl;
	//

	int _size = image_depth.size();
	if (_size > image_rgb.size()) _size = image_rgb.size();
	//*************************************************************
	ros::Time t (1408368397, 740000000);
	int _index = findClosestFrameNumber(t, time_vector);
	getPose(_index);

	bag.close();
}

void getPose(int i){
	std::cout << "------" << i << "--------" << std::endl;
	std::cout << time_vector[i] << std::endl;
	cv::Mat_<float> _depth = image_depth[i];
	cv::Mat_<cv::Vec3b> _rgb = image_rgb[i];

	cv::Mat_<float> d;
	d = 0.001f * cv::Mat_<float>(_depth);

	double dmin, dmax;
	cv::minMaxLoc(_depth, &dmin, &dmax);
	std::cout << "Depth [min max]: [" << dmin << " " << dmax << "]" << std::endl;

	float fx = 525.0f, fy = 525.0f, cx = 319.5f, cy = 239.5f;

	CloudT cloud (new pcl::PointCloud<PointT>);
	cloud->reserve(d.rows*d.cols);
	for(uint r = 0; r < d.rows; ++r) {
		for(uint c = 0; c < d.cols; ++c) {
			float drc = d[r][c];
			if(isnan(drc) || drc == 0.0f)
				continue;

			PointT p;
			p.z = drc;
			p.x = (float(c)-cx) * drc / fx;
			p.y = (float(r)-cy) * drc / fy;

			const cv::Vec3b& rgbrc = _rgb[r][c];
			p.r = rgbrc[2];
			p.g = rgbrc[1];
			p.b = rgbrc[0];

			cloud->push_back(p);
		}
	}
//	std::stringstream cloud_name;
//	cloud_name << i << "some_cloud.pcd";
//	pcl::io::savePCDFile(cloud_name.str(), *cloud);

	std::vector<std::string> objects;
	//objects.push_back("/home/lilita/ACAT_git/PickAndPlace/Jar_PotYellow/Jar.pcd");
	 objects.push_back("/home/lilita/ACAT_git/PickAndPlace/Jar_PotYellow/Pot_yellow.pcd");
	//objects.push_back("/home/lilita/ACAT_git/PickAndPlace/Jar_PotYellow/Pot_Orange.pcd");
	pcl::console::print_warn("Detecting objects Jar and Pot yellow\n");
	for (int i = 0; i < objects.size(); i++) {
		std::vector<std::string> objects_new;
		objects_new.push_back(objects.at(i));
		KMatrix<> pose = detect(cloud, objects_new);

		float k = 1000;
		KMatrix<double> T = KMatrix<double> (4, 4);
		T[0] = -9.74185705e-01;
		T[1] = -5.87171726e-02;
		T[2] = 2.17978090e-01;
		T[3] = 1.79816976e-01 * k;
		T[4] = -2.14628428e-01;
		T[5] = 5.40215135e-01;
		T[6] = -8.13696623e-01;
		T[7] = 1.56685543e+00 * k;
		T[8] = -6.99770972e-02;
		T[9] = -8.39475930e-01;
		T[10] = -5.38872302e-01;
		T[11] = 5.04855752e-01 * k;
		T[12] = 0;
		T[13] = 0;
		T[14] = 0;
		T[15] = 1;

		KMatrix<double> transformed = T * pose ;

		std::cout << transformed << std::endl;

		KQuaternion quaternion = transformed.rotationQuaternion();

		std::cout << transformed[3]/1000 << "\t" << transformed[7]/1000 << "\t" << transformed[11]/1000 << std::endl;
		std::cout << "quaternions: " << quaternion << std::endl;

	}
}

// Descriptor type used
typedef DescRGBN DescT;
Recognition<DescT>::Ptr rec;


KMatrix<> detect (CloudT scene, std::vector<std::string> objects){

	bool visualize = true;
	bool table = true;
	bool gravity = false;
	bool reverse = false;
	bool planar = false;
	bool verbose = false;

	float resolution = 5;
	float radius = 35;
	float threshold = 10;
	float fraction = 0.25;
	float far = 1500;
	float cothres = 0.25;
	int corrnum = 0;
	std::string method = "voting";


	if (boost::iequals(method, "voting"))
		rec.reset( new RecognitionVoting<DescT>(resolution, radius, threshold, fraction, table, planar, true, far) );
	else
		if (boost::iequals(method, "correspondence"))
			rec.reset( new RecognitionCorrespondenceVoting<DescT>(resolution, radius, threshold, fraction, table, planar, true, far) );
		else
			if (boost::iequals(method, "ransac"))
				rec.reset( new RecognitionRansac<DescT>(resolution, radius, threshold, fraction, table, planar, true, far) );
			else
				COVIS_THROW("Unknown recognition method: \"" << method << "\"!");

	rec->setCoplanarityFraction(cothres);
	rec->setGravity(gravity);
	rec->setReverse(reverse);
	if (corrnum > 0)
		rec->setCorrNum(corrnum);
	rec->setVerbose(verbose);
	rec->loadObjects(objects);


	ROS_INFO("Starting GLOBAL estimation...");

	// Start
	DescriptorUtil util;
	DescT::Vec scene_covis = util.fromPCL<pcl::PointXYZRGBNormal,DescT>(*scene);

	Detection::Vec globalres = rec->recognize(scene_covis);

	pcl::console::print_error("Found total: %d\n", globalres.size());

	const Detection& det = globalres[0];
	KMatrix<> pose = det.pose;
	Detection::Vec globalres_1;
	globalres_1.push_back(det);
	std::cout << "RMSE: " << det.rmse << "\t Inliers: " << det.inlierfrac << std::endl;
	if (visualize) {
		DescriptorUtil du;
		du.showDetections<DescT>(rec->getObjects(), rec->getScene(), globalres);
	}
	return pose;
}


int main(int argc, char **argv)
{
	loadBag(argv[1]);
}

int findClosestFrameNumber(ros::Time t, std::vector<ros::Time> vector){
	ros::Duration error (1000);
	int index = -1;
	for (int i = 0; i < vector.size(); i++){
		ros::Time new_time = vector[i];
		if ( fabs((new_time - t).toSec()) < fabs(error.toSec()) ){
			error = new_time - t;
			index = i;
		}
	}
	return index;
}
