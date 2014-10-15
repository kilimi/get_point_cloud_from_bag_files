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

std::vector<std::string> objects;
bool visualize;
bool table;
bool gravity;
bool planar;
bool verbose;

float resolution = 5;
float radius = 35;
float threshold = 10;
float fraction = 0.25;
float far = 1500;
float cothres = 0.25;
int corrnum = 0;
std::string method = "voting";
std::string calibration;
bool _save;


KMatrix<> detect (CloudT scene, std::vector<std::string> objects);
int findClosestFrameNumber(ros::Time t, std::vector<ros::Time> vector);
void getPose(int i);


void loadBag(const std::string &filename, ros::Time t)
{
	rosbag::Bag bag;
	bag.open(filename, rosbag::bagmode::Read);

	std::string l_cam_image = "camera0/rgb/image_raw";
	std::string r_cam_image = "camera0/depth_registered/image_raw";


	std::vector<std::string> topics;
	topics.push_back(l_cam_image);
	topics.push_back(r_cam_image);
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	int d = 0, r = 0;

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		cv::Mat image;

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

	//*************************************************************

	int _index = findClosestFrameNumber(t, time_vector);
	std::cout << "--------" << std::endl;
	std::cout << "Frame time: " << t << std::endl;
	std::cout << "Closest time: " << time_vector[_index] << std::endl;
	std::cout << "--------" << std::endl;
	getPose(_index);

	bag.close();
}

void getPose(int i){
	//get point cloud from RGB-D images
    cv::Mat_<float> _depth = image_depth[i];
	cv::Mat_<cv::Vec3b> _rgb = image_rgb[i];

	cv::Mat_<float> d;
	d = 0.001f * cv::Mat_<float>(_depth);

	double dmin, dmax;
	cv::minMaxLoc(_depth, &dmin, &dmax);
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

	if (_save) pcl::io::savePCDFile("point_cloud.pcd", *cloud);

	for (int i = 0; i < objects.size(); i++) {
		std::vector<std::string> objects_new;
		objects_new.push_back(objects.at(i));
		KMatrix<> pose = detect(cloud, objects_new);

		std::string file = calibration;

		cv::FileStorage fs;
		fs.open(file, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			cerr << "Failed to open " << file << endl;
		}
		cv::Mat M;
		fs["Transform"] >> M;
		float k = 1000;
		KMatrix<double> T1 = KMatrix<double> (4, 4);
		T1[0] = M.at<float>(0,0);
		T1[1] = M.at<float>(0,1);
		T1[2] = M.at<float>(0,2);
		T1[3] = M.at<float>(0,3) * k;
		T1[4] = M.at<float>(1,0);
		T1[5] = M.at<float>(1,1);
		T1[6] = M.at<float>(1,2);
		T1[7] = M.at<float>(1,3) * k;
		T1[8] = M.at<float>(2,0);
		T1[9] = M.at<float>(2,1);
		T1[10] = M.at<float>(2,2);
		T1[11] = M.at<float>(2,3) * k;
		T1[12] = M.at<float>(3,0);
		T1[13] = M.at<float>(3,1);
		T1[14] = M.at<float>(3,2);
		T1[15] = M.at<float>(3,3);

		KMatrix<double> transformed1 = T1 * pose ;

		std::cout << "\nPose: \n" << transformed1 << std::endl;
		std::cout << "\n" << std::endl;
		KQuaternion quaternion1 = transformed1.rotationQuaternion();

		pcl::console::print_value("Pose for xml file: \n\t%f\t%f\t%f\n quaternions: \n\t%f\t%f\t%f\t%f\n", transformed1[3]/1000,  transformed1[7]/1000,  transformed1[11]/1000,
				quaternion1[0], quaternion1[1], quaternion1[2], quaternion1[3]);

	}
}

// Descriptor type used
typedef DescRGBN DescT;
Recognition<DescT>::Ptr rec;


KMatrix<> detect (CloudT scene, std::vector<std::string> objects){

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
	rec->setReverse(false);
	if (corrnum > 0)
		rec->setCorrNum(corrnum);
	rec->setVerbose(verbose);
	rec->loadObjects(objects);


	ROS_INFO("Starting GLOBAL estimation...");

	// Start
	DescriptorUtil util;
	DescT::Vec scene_covis = util.fromPCL<pcl::PointXYZRGBNormal,DescT>(*scene);

	Detection::Vec globalres = rec->recognize(scene_covis);

	pcl::console::print_error("Found total: %d, displaying one\n", globalres.size());

	const Detection& det = globalres[0];
	KMatrix<> pose = det.pose;
	Detection::Vec globalres_1;
	globalres_1.push_back(det);
	pcl::console::print_warn("Best object: Penalty: %f\n" , det.penalty);
	if (visualize) {
		DescriptorUtil du;
		du.showDetections<DescT>(rec->getObjects(), rec->getScene(), globalres_1);
	}
	return pose;
}


int main(int argc, const char **argv)
{
	ProgramOptions po;

	// Flags
	po.addFlag('v', "visualize", "enable visualization");
	po.addFlag('t', "table", "remove dominant table in the scene");
	po.addFlag('s', "save", "save scene point cloud");

	po.addOption<float>("threshold", 10, "inlier threshold");
	po.addOption<float>("cothres", 0.25, "upper coplanarity fraction limit for detection pruning");

	// Positionals
	po.addPositional("bag", "bag file");
	po.addPositional("object", "object file");

	po.addPositional("seconds", "ros time seconds");
	po.addPositional("nanoseconds", "ros time nano seconds");

	po.addPositional("calibration", "calibration matrix");

	// Parse and print
	if ( !po.parse(argc, argv) )
		return 1;

	po.print();

	visualize = po.getFlag("visualize");
	table = po.getFlag("table");
	_save = po.getFlag("save");

	threshold = po.getValue<float>("threshold");
	cothres = po.getValue<float>("cothres");

	gravity = false;
	planar = false;
	verbose = false;
	objects.push_back(po.getValue("object"));
	calibration = po.getValue("calibration");

	ros::Time _time (po.getValue<int>("seconds"), po.getValue<int>("nanoseconds") * 1000000);
	loadBag(po.getValue("bag"), _time);
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
