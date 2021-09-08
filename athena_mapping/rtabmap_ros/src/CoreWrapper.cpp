/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.
Copyright (c) 2021 Xiaomi Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap_ros/CoreWrapper.h"

#include <stdio.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>

#include <visualization_msgs/msg/marker_array.hpp>

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>

#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Registration.h>
#include <rtabmap/core/Graph.h>

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
#include <octomap_msgs/conversions.h>
#include <rtabmap/core/OctoMap.h>
#endif
#endif

#define BAD_COVARIANCE 9999

//msgs
#include "rtabmap_ros/msg/info.hpp"
#include "rtabmap_ros/msg/map_data.hpp"
#include "rtabmap_ros/msg/map_graph.hpp"
#include "rtabmap_ros/srv/get_map.hpp"
#include "rtabmap_ros/srv/publish_map.hpp"
#include "rtabmap_ros/msg/path.hpp"

#include "rtabmap_ros/MsgConversion.h"

using namespace rtabmap;

namespace rtabmap_ros {

std::mutex trackingPoseMutex;

CoreWrapper::CoreWrapper(const rclcpp::NodeOptions & options) :
		athena_utils::LifecycleNode("rtabmap", options),
		CommonDataSubscriber(*this, false),
		paused_(false),
		pubLocalizationOnce_(false),
		lastSignatureIdInDB_(0),
		lastPose_(Transform::getIdentity()),
		lastPoseIntermediate_(false),
		latestNodeWasReached_(false),
		frameId_("base_link"),
		odomFrameId_(""),
		mapFrameId_("map"),
		groundTruthFrameId_(""), // e.g., "world"
		groundTruthBaseFrameId_(""), // e.g., "base_link_gt"
		configPath_(""),
		odomDefaultAngVariance_(0.001),
		odomDefaultLinVariance_(0.001),
		landmarkDefaultAngVariance_(0.001),
		landmarkDefaultLinVariance_(0.001),
		waitForTransform_(0.2),// 200 ms
		useActionForGoal_(false),
		useSavedMap_(true),
		genScan_(false),
		genScanMaxDepth_(4.0),
		genScanMinDepth_(0.0),
		scanCloudMaxPoints_(0),
		mapToOdom_(rtabmap::Transform::getIdentity()),
		transformThread_(0),
		trackingPoseSubThread_(0),
		tfThreadRunning_(false),
		interOdomSync_(0),
		slamErrorStatus_(0),
		configured_(false),
		processImage_(true),
		stereoToDepth_(false),
		odomSensorSync_(false),
		rate_(Parameters::defaultRtabmapDetectionRate()),
		createIntermediateNodes_(Parameters::defaultRtabmapCreateIntermediateNodes()),
		maxMappingNodes_(Parameters::defaultGridGlobalMaxNodes()),
		previousStamp_(0)
#ifdef WITH_MOVE_BASE_MSGS
,		mbClient_(0)
#endif
{ }

CoreWrapper::~CoreWrapper()
{
	if(transformThread_)
	{
		tfThreadRunning_ = false;
		transformThread_->join();
		delete transformThread_;
	}

	if(trackingPoseSubThread_)
	{
		trackingPoseSubThread_->join();
		delete trackingPoseSubThread_;
	}

	this->saveParameters(configPath_);

	printf("rtabmap: Saving database/long-term memory... (located at %s)\n", databasePath_.c_str());
	if(rtabmap_.getMemory())
	{
		// save the grid map
		float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
		cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
		if(!pixels.empty())
		{
			printf("rtabmap: 2D occupancy grid map saved.\n");
			rtabmap_.getMemory()->save2DMap(pixels, xMin, yMin, gridCellSize);
		}
	}

	rtabmap_.close();
	printf("rtabmap: Saving database/long-term memory...done! (located at %s, %ld MB)\n", databasePath_.c_str(), UFile::length(databasePath_)/(1024*1024));

	delete interOdomSync_;
#ifdef WITH_MOVE_BASE_MSGS
	delete mbClient_;
#endif
}

void CoreWrapper::loadParameters(const std::string & configFile, ParametersMap & parameters)
{
	if(!configFile.empty())
	{
		RCLCPP_INFO(this->get_logger(), "Loading parameters from %s", configFile.c_str());
		if(!UFile::exists(configFile.c_str()))
		{
			RCLCPP_WARN(this->get_logger(), "Config file doesn't exist! It will be generated...");
		}
		Parameters::readINI(configFile.c_str(), parameters);
	}
}

void CoreWrapper::saveParameters(const std::string & configFile)
{
	if(!configFile.empty())
	{
		printf("Saving parameters to %s\n", configFile.c_str());

		if(!UFile::exists(configFile.c_str()))
		{
			printf("Config file doesn't exist, a new one will be created.\n");
		}
		Parameters::writeINI(configFile.c_str(), parameters_);
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "Parameters are not saved! (No configuration file provided...)");
	}
}

void CoreWrapper::defaultCallback(const sensor_msgs::msg::Image::ConstSharedPtr imageMsg)
{
	if(!paused_)
	{
		rclcpp::Time stamp = imageMsg->header.stamp;
		if(stamp.seconds() == 0.0)
		{
			RCLCPP_WARN(this->get_logger(), "A null stamp has been detected in the input topic. Make sure the stamp is set.");
			return;
		}

		if(rate_>0.0f)
		{
			if(previousStamp_.seconds() > 0.0 && stamp.seconds() > previousStamp_.seconds() && stamp - previousStamp_ < rclcpp::Duration(1.0f/rate_))
			{
				return;
			}
		}
		previousStamp_ = stamp;

		if(!(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
			 imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
		{
			RCLCPP_ERROR(this->get_logger(), "Input type must be image=mono8,mono16,rgb8,bgr8");
			return;
		}

		cv_bridge::CvImageConstPtr ptrImage;
		if(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
		   imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "mono8");
		}
		else
		{
			ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		}

		// process data
		UTimer timer;
		if(rtabmap_.isIDsGenerated())
		{
			if(!rtabmap_.process(ptrImage->image.clone()))
			{
				RCLCPP_WARN(this->get_logger(), "RTAB-Map could not process the data received!");
			}
			else
			{
				this->publishStats(now());
			}
		}
		else if(!rtabmap_.isIDsGenerated())
		{
			RCLCPP_WARN(this->get_logger(), "Ignoring received image because its sequence ID=0. Please "
					 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
					 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
					 "when you need to have IDs output of RTAB-map synchronised with the source "
					 "image sequence ID.");
		}
		RCLCPP_INFO(this->get_logger(), "rtabmap: Update rate=%fs, Limit=%fs, Processing time = %fs (%d local nodes)",
				1.0f/rate_,
				rtabmap_.getTimeThreshold()/1000.0f,
				timer.ticks(),
				rtabmap_.getWMSize()+rtabmap_.getSTMSize());
	}
}

bool CoreWrapper::odomUpdate(const nav_msgs::msg::Odometry & odomMsg, rclcpp::Time stamp)
{
	if(!paused_)
	{
		Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg.pose.pose);
		if(!odom.isNull())
		{
			Transform odomTF = rtabmap_ros::getTransform(odomMsg.header.frame_id, frameId_, stamp, *tfBuffer_, waitForTransform_);
			if(odomTF.isNull())
			{
				static bool shown = false;
				if(!shown)
				{
					RCLCPP_WARN(this->get_logger(), "We received odometry message, but we cannot get the "
							"corresponding TF %s->%s at data stamp %fs (odom msg stamp is %fs). Make sure TF of odometry is "
							"also published to get more accurate pose estimation. This "
							"warning is only printed once.", odomMsg.header.frame_id.c_str(), frameId_.c_str(), stamp.seconds(), timestampFromROS(odomMsg.header.stamp));
					shown = true;
				}
				stamp = odomMsg.header.stamp;
			}
			else
			{
				odom = odomTF;
			}
		}

		if(!lastPose_.isIdentity() && !odom.isNull() && (odom.isIdentity() || (odomMsg.pose.covariance[0] >= BAD_COVARIANCE && odomMsg.twist.covariance[0] >= BAD_COVARIANCE)))
		{
			UWARN("Odometry is reset (identity pose or high variance (%f) detected). Increment map id!", MAX(odomMsg.pose.covariance[0], odomMsg.twist.covariance[0]));
			rtabmap_.triggerNewMap();
			covariance_ = cv::Mat();
		}

		lastPoseIntermediate_ = false;
		lastPose_ = odom;
		lastPoseStamp_ = stamp;

		// Only update variance if odom is not null
		if(!odom.isNull())
		{
			cv::Mat covariance;
			double variance = odomMsg.twist.covariance[0];
			if(variance == BAD_COVARIANCE || variance <= 0.0f)
			{
				//use the one of the pose
				covariance = cv::Mat(6,6,CV_64FC1, (void*)odomMsg.pose.covariance.data()).clone();
				covariance /= 2.0;
			}
			else
			{
				covariance = cv::Mat(6,6,CV_64FC1, (void*)odomMsg.twist.covariance.data()).clone();
			}

			if(uIsFinite(covariance.at<double>(0,0)) &&
				covariance.at<double>(0,0) != 1.0 &&
				covariance.at<double>(0,0)>0.0)
			{
				// Use largest covariance error (to be independent of the odometry frame rate)
				if(covariance_.empty() || covariance.at<double>(0,0) > covariance_.at<double>(0,0))
				{
					covariance_ = covariance;
				}
			}
		}

		// Throttle
		bool ignoreFrame = false;
		if(stamp.seconds() == 0.0)
		{
			RCLCPP_WARN(this->get_logger(), "A null stamp has been detected in the input topics. Make sure the stamp in all input topics is set.");
			ignoreFrame = true;
		}
		if(rate_>0.0f)
		{
			if(previousStamp_.seconds() > 0.0 && stamp.seconds() > previousStamp_.seconds() && stamp.seconds() - previousStamp_.seconds() < 1.0f/rate_)
			{
				ignoreFrame = true;
			}
		}
		if(ignoreFrame)
		{
			if(createIntermediateNodes_)
			{
				lastPoseIntermediate_ = true;
			}
			else
			{
				return false;
			}
		}
		else if(!ignoreFrame)
		{
			previousStamp_ = stamp;
		}

		return true;
	}
	return false;
}

bool CoreWrapper::odomTFUpdate(const rclcpp::Time & stamp)
{
	if(!paused_)
	{
		// Odom TF ready?
		Transform odom = rtabmap_ros::getTransform(odomFrameId_, frameId_, stamp, *tfBuffer_, waitForTransform_);
		if(odom.isNull())
		{
			return false;
		}

		if(!lastPose_.isIdentity() && odom.isIdentity())
		{
			UWARN("Odometry is reset (identity pose detected). Increment map id!");
			rtabmap_.triggerNewMap();
			covariance_ = cv::Mat();
		}

		lastPoseIntermediate_ = false;
		lastPose_ = odom;
		lastPoseStamp_ = stamp;

		bool ignoreFrame = false;
		if(stamp.seconds() == 0.0)
		{
			RCLCPP_WARN(this->get_logger(), "A null stamp has been detected in the input topics. Make sure the stamp in all input topics is set.");
			ignoreFrame = true;
		}
		if(rate_>0.0f)
		{
			if(previousStamp_.seconds() > 0.0 && stamp.seconds() > previousStamp_.seconds() && stamp.seconds() - previousStamp_.seconds() < 1.0f/rate_)
			{
				ignoreFrame = true;
			}
		}
		if(ignoreFrame)
		{
			if(createIntermediateNodes_)
			{
				lastPoseIntermediate_ = true;
			}
			else
			{
				return false;
			}
		}
		else if(!ignoreFrame)
		{
			previousStamp_ = stamp;
		}

		return true;
	}
	return false;
}

void CoreWrapper::commonDepthCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	std::string odomFrameId = odomFrameId_;
	if(odomMsg.get())
	{
		odomFrameId = odomMsg->header.frame_id;
		if(scan2dMsg.get())
		{
			if(!odomUpdate(*odomMsg, scan2dMsg->header.stamp))
			{
				return;
			}
		}
		else if(scan3dMsg.get())
		{
			if(!odomUpdate(*odomMsg, scan3dMsg->header.stamp))
			{
				return;
			}
		}
		else if(imageMsgs.size() == 0 || imageMsgs[0].get() == 0 || !odomUpdate(*odomMsg, imageMsgs[0]->header.stamp))
		{
			return;
		}
	}
	else if(scan2dMsg.get())
	{
		if(!odomTFUpdate(scan2dMsg->header.stamp))
		{
			return;
		}
	}
	else if(scan3dMsg.get())
	{
		if(!odomTFUpdate(scan3dMsg->header.stamp))
		{
			return;
		}
	}
	else if(imageMsgs.size() == 0 || imageMsgs[0].get() == 0 || !odomTFUpdate(imageMsgs[0]->header.stamp))
	{
		return;
	}

	commonDepthCallbackImpl(odomFrameId,
			userDataMsg,
			imageMsgs,
			depthMsgs,
			cameraInfoMsgs,
			scan2dMsg,
			scan3dMsg,
			odomInfoMsg);
}

void CoreWrapper::commonDepthCallbackImpl(
		const std::string & odomFrameId,
		const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::msg::CameraInfo> & cameraInfoMsgs,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	cv::Mat rgb;
	cv::Mat depth;
	std::vector<rtabmap::CameraModel> cameraModels;
	if(!rtabmap_ros::convertRGBDMsgs(
			imageMsgs,
			depthMsgs,
			cameraInfoMsgs,
			frameId_,
			odomSensorSync_?odomFrameId:"",
			lastPoseStamp_,
			rgb,
			depth,
			cameraModels,
			*tfBuffer_,
			waitForTransform_))
	{
		RCLCPP_ERROR(this->get_logger(), "Could not convert rgb/depth msgs! Aborting rtabmap update...");
		return;
	}

	UASSERT(uContains(parameters_, rtabmap::Parameters::kMemSaveDepth16Format()));
	if(!depth.empty() && depth.type() == CV_32FC1 && uStr2Bool(parameters_.at(Parameters::kMemSaveDepth16Format())))
	{
		depth = rtabmap::util2d::cvtDepthFromFloat(depth);
		static bool shown = false;
		if(!shown)
		{
			RCLCPP_WARN(this->get_logger(), "Save depth data to 16 bits format: depth type detected is "
				  "32FC1, use 16UC1 depth format to avoid this conversion "
				  "(or set parameter \"Mem/SaveDepth16Format=false\" to use "
				  "32bits format). This message is only printed once...");
			shown = true;
		}
	}

	LaserScan scan;
	bool genMaxScanPts = 0;
	if(scan2dMsg.get() == 0 && scan3dMsg.get() == 0 && !depth.empty() && genScan_)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud2d(new pcl::PointCloud<pcl::PointXYZ>);
		*scanCloud2d = util3d::laserScanFromDepthImages(
				depth,
				cameraModels,
				genScanMaxDepth_,
				genScanMinDepth_);
		genMaxScanPts += depth.cols;
		scan = LaserScan(rtabmap::util3d::laserScan2dFromPointCloud(*scanCloud2d), 0, genScanMaxDepth_, LaserScan::kXY);
	}
	else if(scan2dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScanMsg(
				*scan2dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				*tfBuffer_,
				waitForTransform_,
				// backward compatibility, project 2D scan in /base_link frame
				rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0))
		{
			RCLCPP_ERROR(this->get_logger(), "Could not convert laser scan msg! Aborting rtabmap update...");
			return;
		}
	}
	else if(scan3dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScan3dMsg(
				*scan3dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				*tfBuffer_,
				waitForTransform_,
				scanCloudMaxPoints_))
		{
			RCLCPP_ERROR(this->get_logger(), "Could not convert 3d laser scan msg! Aborting rtabmap update...");
			return;
		}
	}

	cv::Mat userData;
	if(userDataMsg.get())
	{
		userData = rtabmap_ros::userDataFromROS(*userDataMsg);
		UScopeMutex lock(userDataMutex_);
		if(!userData_.empty())
		{
			RCLCPP_WARN(this->get_logger(), "Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
			userData_ = cv::Mat();
		}
	}
	else
	{
		UScopeMutex lock(userDataMutex_);
		userData = userData_;
		userData_ = cv::Mat();
	}

	SensorData data(
			scan,
			rgb,
			depth,
			cameraModels,
			lastPoseIntermediate_?-1:0,
			rtabmap_ros::timestampFromROS(lastPoseStamp_),
			userData);

	OdometryInfo odomInfo;
	if(odomInfoMsg.get())
	{
		odomInfo = odomInfoFromROS(*odomInfoMsg);
	}

	process(lastPoseStamp_,
			data,
			lastPose_,
			odomFrameId,
			covariance_,
			odomInfo);
	covariance_ = cv::Mat();
}

void CoreWrapper::commonStereoCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
		const cv_bridge::CvImageConstPtr& leftImageMsg,
		const cv_bridge::CvImageConstPtr& rightImageMsg,
		const sensor_msgs::msg::CameraInfo& leftCamInfoMsg,
		const sensor_msgs::msg::CameraInfo& rightCamInfoMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	std::string odomFrameId = odomFrameId_;
	if(odomMsg.get())
	{
		odomFrameId = odomMsg->header.frame_id;
		if(scan2dMsg.get())
		{
			if(!odomUpdate(*odomMsg, scan2dMsg->header.stamp))
			{
				return;
			}
		}
		else if(scan3dMsg.get())
		{
			if(!odomUpdate(*odomMsg, scan3dMsg->header.stamp))
			{
				return;
			}
		}
		else if(leftImageMsg.get() == 0 || !odomUpdate(*odomMsg, leftImageMsg->header.stamp))
		{
			return;
		}
	}
	else if(scan2dMsg.get())
	{
		if(!odomTFUpdate(scan2dMsg->header.stamp))
		{
			return;
		}
	}
	else if(scan3dMsg.get())
	{
		if(!odomTFUpdate(scan3dMsg->header.stamp))
		{
			return;
		}
	}
	else if(leftImageMsg.get() == 0 || !odomTFUpdate(leftImageMsg->header.stamp))
	{
		return;
	}

	cv::Mat left;
	cv::Mat right;
	StereoCameraModel stereoModel;
	if(!rtabmap_ros::convertStereoMsg(
			leftImageMsg,
			rightImageMsg,
			leftCamInfoMsg,
			rightCamInfoMsg,
			frameId_,
			odomSensorSync_?odomFrameId:"",
			lastPoseStamp_,
			left,
			right,
			stereoModel,
			*tfBuffer_,
			waitForTransform_))
	{
		RCLCPP_ERROR(this->get_logger(), "Could not convert stereo msgs! Aborting rtabmap update...");
		return;
	}

	if(stereoToDepth_)
	{
		// cv::stereoBM() see "$ rosrun rtabmap_ros rtabmap --params | grep StereoBM" for parameters
		cv::Mat disparity = rtabmap::util2d::disparityFromStereoImages(
				left,
				right,
				parameters_);
		if(disparity.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "Could not compute disparity image (\"stereo_to_depth\" is true)!");
			return;
		}
		cv::Mat depth = rtabmap::util2d::depthFromDisparity(
						disparity,
						stereoModel.left().fx(),
						stereoModel.baseline());

		if(depth.empty())
		{
			RCLCPP_ERROR(this->get_logger(), "Could not compute depth image (\"stereo_to_depth\" is true)!");
			return;
		}
		UASSERT(depth.type() == CV_16UC1 || depth.type() == CV_32FC1);

		// move to common depth callback
		cv_bridge::CvImagePtr imgDepth(new cv_bridge::CvImage);
		if(depth.type() == CV_16UC1)
		{
			imgDepth->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
		}
		else // CV_32FC1
		{
			imgDepth->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		}
		imgDepth->image = depth;
		imgDepth->header = leftImageMsg->header;
		std::vector<cv_bridge::CvImageConstPtr> rgbImages(1);
		std::vector<cv_bridge::CvImageConstPtr> depthImages(1);
		std::vector<sensor_msgs::msg::CameraInfo> cameraInfos(1);
		rgbImages[0] = leftImageMsg;
		depthImages[0] = imgDepth;
		cameraInfos[0] = leftCamInfoMsg;

		commonDepthCallbackImpl(odomFrameId, rtabmap_ros::msg::UserData::SharedPtr(), rgbImages, depthImages, cameraInfos, scan2dMsg, scan3dMsg, odomInfoMsg);
		return;
	}

	LaserScan scan;
	if(scan2dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScanMsg(
				*scan2dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				*tfBuffer_,
				waitForTransform_,
				// backward compatibility, project 2D scan in /base_link frame
				rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0))
		{
			RCLCPP_ERROR(this->get_logger(), "Could not convert laser scan msg! Aborting rtabmap update...");
			return;
		}
	}
	else if(scan3dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScan3dMsg(
				*scan3dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				*tfBuffer_,
				waitForTransform_,
				scanCloudMaxPoints_))
		{
			RCLCPP_ERROR(this->get_logger(), "Could not convert 3d laser scan msg! Aborting rtabmap update...");
			return;
		}
	}

	cv::Mat userData;
	if(userDataMsg.get())
	{
		userData = rtabmap_ros::userDataFromROS(*userDataMsg);
		UScopeMutex lock(userDataMutex_);
		if(!userData_.empty())
		{
			RCLCPP_WARN(this->get_logger(), "Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
			userData_ = cv::Mat();
		}
	}
	else
	{
		UScopeMutex lock(userDataMutex_);
		userData = userData_;
		userData_ = cv::Mat();
	}

	SensorData data(
			scan,
			left,
			right,
			stereoModel,
			lastPoseIntermediate_?-1:0,
			rtabmap_ros::timestampFromROS(lastPoseStamp_),
			userData);

	OdometryInfo odomInfo;
	if(odomInfoMsg.get())
	{
		odomInfo = odomInfoFromROS(*odomInfoMsg);
	}

	process(lastPoseStamp_,
			data,
			lastPose_,
			odomFrameId,
			covariance_,
			odomInfo);

	covariance_ = cv::Mat();
}

void CoreWrapper::commonLaserScanCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
		const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan2dMsg,
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan3dMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	UASSERT(scan2dMsg.get() || scan3dMsg.get());
	std::string odomFrameId = odomFrameId_;
	if(odomMsg.get())
	{
		odomFrameId = odomMsg->header.frame_id;
		if(scan2dMsg.get())
		{
			if(!odomUpdate(*odomMsg, scan2dMsg->header.stamp))
			{
				return;
			}
		}
		else if(scan3dMsg.get())
		{
			if(!odomUpdate(*odomMsg, scan3dMsg->header.stamp))
			{
				return;
			}
		}
		else
		{
			return;
		}
	}
	else if(scan2dMsg.get())
	{
		if(!odomTFUpdate(scan2dMsg->header.stamp))
		{
			return;
		}
	}
	else if(scan3dMsg.get())
	{
		if(!odomTFUpdate(scan3dMsg->header.stamp))
		{
			return;
		}
	}
	else
	{
		return;
	}

	LaserScan scan;
	if(scan2dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScanMsg(
				*scan2dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				*tfBuffer_,
				waitForTransform_,
				// backward compatibility, project 2D scan in /base_link frame
				rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0))
		{
			RCLCPP_ERROR(this->get_logger(), "Could not convert laser scan msg! Aborting rtabmap update...");
			return;
		}
	}
	else if(scan3dMsg.get() != 0)
	{
		if(!rtabmap_ros::convertScan3dMsg(
				*scan3dMsg,
				frameId_,
				odomSensorSync_?odomFrameId:"",
				lastPoseStamp_,
				scan,
				*tfBuffer_,
				waitForTransform_,
				scanCloudMaxPoints_))
		{
			RCLCPP_ERROR(this->get_logger(), "Could not convert 3d laser scan msg! Aborting rtabmap update...");
			return;
		}
	}

	cv::Mat userData;
	if(userDataMsg.get())
	{
		userData = rtabmap_ros::userDataFromROS(*userDataMsg);
		UScopeMutex lock(userDataMutex_);
		if(!userData_.empty())
		{
			RCLCPP_WARN(this->get_logger(), "Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
			userData_ = cv::Mat();
		}
	}
	else
	{
		UScopeMutex lock(userDataMutex_);
		userData = userData_;
		userData_ = cv::Mat();
	}

	cv::Mat rgb = cv::Mat::zeros(2,1,CV_8UC1);
	cv::Mat depth = cv::Mat::zeros(2,1,CV_16UC1);
	CameraModel model(
			1,
			1,
			0.5,
			1,
			scan.localTransform()*Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
			0,
			cv::Size(1,2));

	SensorData data(
			scan,
			rgb,
			depth,
			model,
			lastPoseIntermediate_?-1:0,
			rtabmap_ros::timestampFromROS(lastPoseStamp_),
			userData);

	OdometryInfo odomInfo;
	if(odomInfoMsg.get())
	{
		odomInfo = odomInfoFromROS(*odomInfoMsg);
	}

	process(lastPoseStamp_,
			data,
			lastPose_,
			odomFrameId,
			covariance_,
			odomInfo);

	covariance_ = cv::Mat();
}

void CoreWrapper::commonOdomCallback(
		const nav_msgs::msg::Odometry::ConstSharedPtr & odomMsg,
		const rtabmap_ros::msg::UserData::ConstSharedPtr & userDataMsg,
		const rtabmap_ros::msg::OdomInfo::ConstSharedPtr& odomInfoMsg)
{
	UASSERT(odomMsg.get());
	std::string odomFrameId = odomFrameId_;

	odomFrameId = odomMsg->header.frame_id;
	if(!odomUpdate(*odomMsg, odomMsg->header.stamp))
	{
		return;
	}

	cv::Mat userData;
	if(userDataMsg.get())
	{
		userData = rtabmap_ros::userDataFromROS(*userDataMsg);
		UScopeMutex lock(userDataMutex_);
		if(!userData_.empty())
		{
			RCLCPP_WARN(this->get_logger(), "Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
			userData_ = cv::Mat();
		}
	}
	else
	{
		UScopeMutex lock(userDataMutex_);
		userData = userData_;
		userData_ = cv::Mat();
	}

	cv::Mat rgb = cv::Mat::zeros(2,1,CV_8UC1);
	cv::Mat depth = cv::Mat::zeros(2,1,CV_16UC1);
	CameraModel model(
			1,
			1,
			0.5,
			1,
			Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
			0,
			cv::Size(1,2));

	SensorData data(
			rgb,
			depth,
			model,
			lastPoseIntermediate_?-1:0,
			rtabmap_ros::timestampFromROS(lastPoseStamp_),
			userData);

	OdometryInfo odomInfo;
	if(odomInfoMsg.get())
	{
		odomInfo = odomInfoFromROS(*odomInfoMsg);
	}

	process(lastPoseStamp_,
			data,
			lastPose_,
			odomFrameId,
			covariance_,
			odomInfo);

	covariance_ = cv::Mat();
}

void CoreWrapper::process(
		const rclcpp::Time & stamp,
		SensorData & data,
		const Transform & odom,
		const std::string & odomFrameId,
		const cv::Mat & odomCovariance,
		const OdometryInfo & odomInfo)
{
	UTimer timer;
	if(rtabmap_.isIDsGenerated() || data.id() > 0)
	{
		// Add intermediate nodes?
		for(std::list<std::pair<nav_msgs::msg::Odometry, rtabmap_ros::msg::OdomInfo> >::iterator iter=interOdoms_.begin(); iter!=interOdoms_.end();)
		{
			if(rclcpp::Time(iter->first.header.stamp.sec, iter->first.header.stamp.nanosec) < lastPoseStamp_)
			{
				Transform interOdom = rtabmap_ros::transformFromPoseMsg(iter->first.pose.pose);
				if(!interOdom.isNull())
				{
					cv::Mat covariance;
					double variance = iter->first.twist.covariance[0];
					if(variance == BAD_COVARIANCE || variance <= 0.0f)
					{
						//use the one of the pose
						covariance = cv::Mat(6,6,CV_64FC1, (void*)iter->first.pose.covariance.data()).clone();
						covariance /= 2.0;
					}
					else
					{
						covariance = cv::Mat(6,6,CV_64FC1, (void*)iter->first.twist.covariance.data()).clone();
					}
					if(!uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
					{
						covariance = cv::Mat::eye(6,6,CV_64FC1);
						if(odomDefaultLinVariance_ > 0.0f)
						{
							covariance.at<double>(0,0) = odomDefaultLinVariance_;
							covariance.at<double>(1,1) = odomDefaultLinVariance_;
							covariance.at<double>(2,2) = odomDefaultLinVariance_;
						}
						if(odomDefaultAngVariance_ > 0.0f)
						{
							covariance.at<double>(3,3) = odomDefaultAngVariance_;
							covariance.at<double>(4,4) = odomDefaultAngVariance_;
							covariance.at<double>(5,5) = odomDefaultAngVariance_;
						}
					}

					cv::Mat rgb = cv::Mat::zeros(2,1,CV_8UC1);
					cv::Mat depth = cv::Mat::zeros(2,1,CV_16UC1);
					CameraModel model(
							1,
							1,
							0.5,
							1,
							Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
							0,
							cv::Size(1,2));
					SensorData interData(rgb, depth, model, -1, rtabmap_ros::timestampFromROS(iter->first.header.stamp));
					Transform gt;
					if(!groundTruthFrameId_.empty())
					{
						gt = rtabmap_ros::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, iter->first.header.stamp, *tfBuffer_, waitForTransform_);
					}
					interData.setGroundTruth(gt);

					std::map<std::string, float> externalStats;
					std::vector<float> odomVelocity;
					if(iter->second.time_estimation != 0.0f)
					{
						OdometryInfo info = odomInfoFromROS(iter->second);
						externalStats = rtabmap_ros::odomInfoToStatistics(info);

						if(info.interval>0.0)
						{
							odomVelocity.resize(6);
							float x,y,z,roll,pitch,yaw;
							info.transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
							odomVelocity[0] = x/info.interval;
							odomVelocity[1] = y/info.interval;
							odomVelocity[2] = z/info.interval;
							odomVelocity[3] = roll/info.interval;
							odomVelocity[4] = pitch/info.interval;
							odomVelocity[5] = yaw/info.interval;
						}
					}

					rtabmap_.process(interData, interOdom, covariance, odomVelocity, externalStats);
				}
				interOdoms_.erase(iter++);
			}
			else if(iter->first.header.stamp == lastPoseStamp_)
			{
				interOdoms_.erase(iter++);
				break;
			}
			else
			{
				break;
			}
		}

		//Add async stuff
		Transform groundTruthPose;
		if(!groundTruthFrameId_.empty())
		{
			groundTruthPose = rtabmap_ros::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, lastPoseStamp_, *tfBuffer_, waitForTransform_);
		}
		data.setGroundTruth(groundTruthPose);

		//global pose
		if(globalPose_.header.stamp.sec != 0 || globalPose_.header.stamp.nanosec != 0)
		{
			// assume sensor is fixed
			Transform sensorToBase = rtabmap_ros::getTransform(
					globalPose_.header.frame_id,
					frameId_,
					lastPoseStamp_,
					*tfBuffer_,
					waitForTransform_);
			if(!sensorToBase.isNull())
			{
				Transform globalPose = rtabmap_ros::transformFromPoseMsg(globalPose_.pose.pose);
				globalPose *= sensorToBase; // transform global pose from sensor frame to robot base frame

				// Correction of the global pose accounting the odometry movement since we received it
				Transform correction = rtabmap_ros::getTransform(
						frameId_,
						odomFrameId,
						rclcpp::Time(globalPose_.header.stamp.sec, globalPose_.header.stamp.nanosec),
						lastPoseStamp_,
						*tfBuffer_,
						waitForTransform_);
				if(!correction.isNull())
				{
					globalPose *= correction;
				}
				else
				{
					RCLCPP_WARN(this->get_logger(), "Could not adjust global pose accordingly to latest odometry pose. "
							"If odometry is small since it received the global pose and "
							"covariance is large, this should not be a problem.");
				}
				cv::Mat globalPoseCovariance = cv::Mat(6,6, CV_64FC1, (void*)globalPose_.pose.covariance.data()).clone();
				data.setGlobalPose(globalPose, globalPoseCovariance);
			}
		}
		globalPose_.header.stamp = rclcpp::Time(0);

		if(gps_.stamp() > 0.0)
		{
			data.setGPS(gps_);
		}
		gps_ = rtabmap::GPS();

		//tag detections
		Landmarks landmarks = rtabmap_ros::landmarksFromROS(
				tags_,
				frameId_,
				odomFrameId,
				lastPoseStamp_,
				*tfBuffer_,
				waitForTransform_,
				landmarkDefaultLinVariance_,
				landmarkDefaultAngVariance_);
		tags_.clear();
		if(!landmarks.empty())
		{
			data.setLandmarks(landmarks);
		}

		// IMU
		if(!imus_.empty())
		{
			double stampDiff = 0.0;
			Transform t = Transform::getClosestTransform(imus_, data.stamp(), &stampDiff);
			if(!t.isNull() && stampDiff == 0.0)
			{
				Eigen::Quaterniond q = t.getQuaterniond();
				data.setIMU(IMU(cv::Vec4d(q.x(), q.y(), q.z(), q.w()), cv::Mat::eye(3,3,CV_64FC1),
						cv::Vec3d(), cv::Mat(),
						cv::Vec3d(), cv::Mat(),
						Transform::getIdentity()));
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "We are receiving imu data (buffer=%d), but cannot interpolate "
						"imu transform at time %f (closest is at %f). IMU won't be added to graph.",
						(int)imus_.size(), data.stamp(), stampDiff);
			}
		}

		double timeRtabmap = 0.0;
		double timeUpdateMaps = 0.0;
		double timePublishMaps = 0.0;

		cv::Mat covariance = odomCovariance;
		if(covariance.empty() || !uIsFinite(covariance.at<double>(0,0)) || covariance.at<double>(0,0)<=0.0f)
		{
			covariance = cv::Mat::eye(6,6,CV_64FC1);
			if(odomDefaultLinVariance_ > 0.0f)
			{
				covariance.at<double>(0,0) = odomDefaultLinVariance_;
				covariance.at<double>(1,1) = odomDefaultLinVariance_;
				covariance.at<double>(2,2) = odomDefaultLinVariance_;
			}
			if(odomDefaultAngVariance_ > 0.0f)
			{
				covariance.at<double>(3,3) = odomDefaultAngVariance_;
				covariance.at<double>(4,4) = odomDefaultAngVariance_;
				covariance.at<double>(5,5) = odomDefaultAngVariance_;
			}
		}

		std::map<std::string, float> externalStats;
		std::vector<float> odomVelocity;
		if(odomInfo.timeEstimation != 0.0f)
		{
			externalStats = rtabmap_ros::odomInfoToStatistics(odomInfo);

			if(odomInfo.interval>0.0)
			{
				odomVelocity.resize(6);
				float x,y,z,roll,pitch,yaw;
				odomInfo.transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
				odomVelocity[0] = x/odomInfo.interval;
				odomVelocity[1] = y/odomInfo.interval;
				odomVelocity[2] = z/odomInfo.interval;
				odomVelocity[3] = roll/odomInfo.interval;
				odomVelocity[4] = pitch/odomInfo.interval;
				odomVelocity[5] = yaw/odomInfo.interval;
			}
		}
		if(rtabmapROSStats_.size())
		{
			externalStats.insert(rtabmapROSStats_.begin(), rtabmapROSStats_.end());
			rtabmapROSStats_.clear();
		}

		bool rtabmap_success = false;
		if (!processImage_) {
			setTrackingPoint(lastPoseStamp_);
			SensorData interData(cv::Mat(), data.depthRaw(), data.cameraModels(), data.id(), data.stamp(), data.userDataRaw());
			rtabmap_success = rtabmap_.process(interData, odom, covariance, odomVelocity, externalStats);
		} else {
			rtabmap_success = rtabmap_.process(data, odom, covariance, odomVelocity, externalStats);
		}
		//if(rtabmap_.process(data, odom, covariance, odomVelocity, externalStats))
		if(rtabmap_success)
		{
			timeRtabmap = timer.ticks();
			mapToOdomMutex_.lock();
			rtabmap::Transform lastMapToOdom = mapToOdom_;
			mapToOdom_ = rtabmap_.getMapCorrection();
			odomFrameId_ = odomFrameId;
			mapToOdomMutex_.unlock();

			rtabmap::Transform deltaOdom = (lastMapToOdom*odom).inverse()*mapToOdom_*odom;
			if(std::fabs(deltaOdom.theta()) > 0.174 || deltaOdom.getNormSquared() > 0.2*0.2)
			{
				mapsManager_.setMapRebuild(true);
			}

			if(data.id() < 0)
			{
				RCLCPP_INFO(this->get_logger(), "Intermediate node added");
			}
			else
			{
				// Publish local graph, info
				this->publishStats(stamp);
				if(localizationPosePub_ != nullptr && localizationPosePub_->get_subscription_count() &&
					!rtabmap_.getStatistics().localizationCovariance().empty())
				{
					geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
					poseMsg.header.frame_id = mapFrameId_;
					poseMsg.header.stamp = stamp;
					rtabmap_ros::transformToPoseMsg(mapToOdom_*odom, poseMsg.pose.pose);
					const cv::Mat & cov = rtabmap_.getStatistics().localizationCovariance();
					memcpy(poseMsg.pose.covariance.data(), cov.data, cov.total()*sizeof(double));
					localizationPosePub_->publish(poseMsg);
				}
				std::map<int, rtabmap::Transform> filteredPoses(rtabmap_.getLocalOptimizedPoses().lower_bound(1), rtabmap_.getLocalOptimizedPoses().end());

				// create a tmp signature with latest sensory data if latest signature was ignored
				std::map<int, rtabmap::Signature> tmpSignature;
				if(rtabmap_.getMemory() == 0 ||
					filteredPoses.size() == 0 ||
					rtabmap_.getMemory()->getLastSignatureId() != filteredPoses.rbegin()->first ||
					rtabmap_.getMemory()->getLastWorkingSignature() == 0 ||
					rtabmap_.getMemory()->getLastWorkingSignature()->sensorData().gridCellSize() == 0 ||
					(!mapsManager_.getOccupancyGrid()->isGridFromDepth() && data.laserScanRaw().is2d())) // 2d laser scan would fill empty space for latest data
				{
					SensorData tmpData = data;
					tmpData.setId(0);
					tmpSignature.insert(std::make_pair(0, Signature(0, -1, 0, data.stamp(), "", odom, Transform(), tmpData)));
					filteredPoses.insert(std::make_pair(0, mapToOdom_*odom));
				}

				if(maxMappingNodes_ > 0 && filteredPoses.size()>1)
				{
					std::map<int, Transform> nearestPoses;
					std::map<int, float> nodes = graph::findNearestNodes(filteredPoses, mapToOdom_*odom, maxMappingNodes_);
					for(std::map<int, float>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
					{
						std::map<int, Transform>::iterator pter = filteredPoses.find(iter->first);
						if(pter != filteredPoses.end())
						{
							nearestPoses.insert(*pter);
						}
					}
					//add latest/zero and make sure those on a planned path are not filtered
					std::set<int> onPath;
					if(rtabmap_.getPath().size())
					{
						std::vector<int> nextNodes = rtabmap_.getPathNextNodes();
						onPath.insert(nextNodes.begin(), nextNodes.end());
					}
					for(std::map<int, Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end(); ++iter)
					{
						if(iter->first == 0 || onPath.find(iter->first) != onPath.end())
						{
							nearestPoses.insert(*iter);
						}
						else if(onPath.empty())
						{
							break;
						}
					}

					filteredPoses = nearestPoses;
				}

				// Update maps
				filteredPoses = mapsManager_.updateMapCaches(
						filteredPoses,
						rtabmap_.getMemory(),
						false,
						false,
						tmpSignature);

				timeUpdateMaps = timer.ticks();

				mapsManager_.publishMaps(filteredPoses, stamp, mapFrameId_);

				// update goal if planning is enabled
				if(!currentMetricGoal_.isNull())
				{
					if(rtabmap_.getPath().size() == 0)
					{
#ifdef WITH_MOVE_BASE_MSGS
						// Don't send status yet if move_base actionlib is used unless it failed,
						// let move_base finish reaching the goal
						if(mbClient_ == 0 || rtabmap_.getPathStatus() <= 0)
						{
							if(rtabmap_.getPathStatus() > 0)
							{
								// Goal reached
								RCLCPP_INFO(this->get_logger(), "Planning: Publishing goal reached!");
							}
							else if(rtabmap_.getPathStatus() <= 0)
							{
								RCLCPP_WARN(this->get_logger(), "Planning: Plan failed!");
								if(mbClient_ && mbClient_->isServerConnected())
								{
									mbClient_->cancelGoal();
								}
							}

							if(goalReachedPub_ != nullptr && goalReachedPub_->get_subscription_count())
							{
								std_msgs::msg::Bool result;
								result.data = rtabmap_.getPathStatus() > 0;
								goalReachedPub_->publish(result);
							}
							currentMetricGoal_.setNull();
							lastPublishedMetricGoal_.setNull();
							goalFrameId_.clear();
							latestNodeWasReached_ = false;
						}
#endif
					}
					else
					{
						currentMetricGoal_ = rtabmap_.getPose(rtabmap_.getPathCurrentGoalId());
						if(!currentMetricGoal_.isNull())
						{
							// Adjust the target pose relative to last node
							if(rtabmap_.getPathCurrentGoalId() == rtabmap_.getPath().back().first && rtabmap_.getLocalOptimizedPoses().size())
							{
								if(latestNodeWasReached_ ||
								   rtabmap_.getLastLocalizationPose().getDistance(currentMetricGoal_) < rtabmap_.getLocalRadius())
								{
									latestNodeWasReached_ = true;
									Transform goalLocalTransform = Transform::getIdentity();
									if(!goalFrameId_.empty() && goalFrameId_.compare(frameId_) != 0)
									{
										Transform localT = rtabmap_ros::getTransform(frameId_, goalFrameId_, now(), *tfBuffer_, waitForTransform_);
										if(!localT.isNull())
										{
											goalLocalTransform = localT.inverse().to3DoF();
										}
									}
									currentMetricGoal_ *= rtabmap_.getPathTransformToGoal()*goalLocalTransform;
								}
							}

							// publish next goal with updated currentMetricGoal_
							publishCurrentGoal(stamp);

							// publish local path
							publishLocalPath(stamp);

							// publish global path
							publishGlobalPath(stamp);
						}
						else
						{
							RCLCPP_ERROR(this->get_logger(), "Planning: Local map broken, current goal id=%d (the robot may have moved to far from planned nodes)",
									rtabmap_.getPathCurrentGoalId());
							rtabmap_.clearPath(-1);
							if(goalReachedPub_ != nullptr && goalReachedPub_->get_subscription_count())
							{
								std_msgs::msg::Bool result;
								result.data = false;
								goalReachedPub_->publish(result);
							}
							currentMetricGoal_.setNull();
							lastPublishedMetricGoal_.setNull();
							goalFrameId_.clear();
							latestNodeWasReached_ = false;
						}
					}
				}

				timePublishMaps = timer.ticks();
			}
		}
		else
		{
			timeRtabmap = timer.ticks();
		}
		if(lastSignatureIdInDB_ && pubLocalizationOnce_)
		{
			if((rtabmap_.getLoopClosureId() > 0) && (rtabmap_.getLoopClosureId() <= lastSignatureIdInDB_))
			{
				RCLCPP_INFO(this->get_logger(), "relocation success.");
				if(relocalizationStatePub_->get_subscription_count())
				{
					std_msgs::msg::Bool relocalizationState;
					relocalizationState.data = true;
					relocalizationStatePub_->publish(relocalizationState);
					pubLocalizationOnce_ = false; // publish once
				}

				// relocation parameters recovery
				for(auto iter : partOfLocalizationParameters_)
					set_parameter(rclcpp::Parameter(iter.first, iter.second));
			}
		}
		RCLCPP_INFO(this->get_logger(), "rtabmap (%d): Rate=%.2fs, Limit=%.3fs, RTAB-Map=%.4fs, Maps update=%.4fs pub=%.4fs (local map=%d, WM=%d)",
				rtabmap_.getLastLocationId(),
				rate_>0?1.0f/rate_:0,
				rtabmap_.getTimeThreshold()/1000.0f,
				timeRtabmap,
				timeUpdateMaps,
				timePublishMaps,
				(int)rtabmap_.getLocalOptimizedPoses().size(),
				rtabmap_.getWMSize()+rtabmap_.getSTMSize());
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/HasSubscribers/"), mapsManager_.hasSubscribers()?1:0));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeRtabmap/ms"), timeRtabmap*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeUpdatingMaps/ms"), timeUpdateMaps*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimePublishing/ms"), timePublishMaps*1000.0f));
		rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeTotal/ms"), (timeRtabmap+timeUpdateMaps+timePublishMaps)*1000.0f));
	}
	else if(!rtabmap_.isIDsGenerated())
	{
		RCLCPP_WARN(this->get_logger(), "Ignoring received image because its sequence ID=0. Please "
				 "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
				 "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
				 "when you need to have IDs output of RTAB-map synchronized with the source "
				 "image sequence ID.");
	}

	// TODO. if other error status are needed, wait to change.
	if(errorStatusPub_->get_subscription_count() && slamErrorStatus_ != 0)
	{
		std_msgs::msg::UInt8 errorStatus;
		errorStatus.data = slamErrorStatus_;
		errorStatusPub_->publish(errorStatus);
		slamErrorStatus_ = 0;
	}
}

void CoreWrapper::userDataAsyncCallback(const rtabmap_ros::msg::UserData::SharedPtr dataMsg)
{
	if(!paused_)
	{
		UScopeMutex lock(userDataMutex_);
		static bool warningShow = false;
		if(!userData_.empty() && !warningShow)
		{
			RCLCPP_WARN(this->get_logger(), "Overwriting previous user data set. When asynchronous user "
					"data input topic rate is higher than "
					"map update rate (current %s=%f), only latest data is saved "
					"in the next node created. This message will is shown only once.",
					Parameters::kRtabmapDetectionRate().c_str(), rate_);
			warningShow = true;
		}
		userData_ = rtabmap_ros::userDataFromROS(*dataMsg);
	}
}

void CoreWrapper::globalPoseAsyncCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr globalPoseMsg)
{
	if(!paused_)
	{
		globalPose_ = *globalPoseMsg;
	}
}

void CoreWrapper::gpsFixAsyncCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gpsFixMsg)
{
	if(!paused_)
	{
		double error = 10.0;
		if(gpsFixMsg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
		{
			double variance = uMax3(gpsFixMsg->position_covariance.at(0), gpsFixMsg->position_covariance.at(4), gpsFixMsg->position_covariance.at(8));
			if(variance>0.0)
			{
				error = sqrt(variance);
			}
		}
		gps_ = rtabmap::GPS(
				timestampFromROS(gpsFixMsg->header.stamp),
				gpsFixMsg->longitude,
				gpsFixMsg->latitude,
				gpsFixMsg->altitude,
				error,
				0);
	}
}

#ifdef WITH_APRILTAG_ROS
void CoreWrapper::tagDetectionsAsyncCallback(const apriltag_ros::AprilTagDetectionArray tagDetections)
{
	if(!paused_)
	{
		for(unsigned int i=0; i<tagDetections.detections.size(); ++i)
		{
			if(tagDetections.detections[i].id.size() >= 1)
			{
				geometry_msgs::msg::PoseWithCovarianceStamped p = tagDetections.detections[i].pose;
				p.header = tagDetections.header;
				if(!tagDetections.detections[i].pose.header.frame_id.empty())
				{
					p.header.frame_id = tagDetections.detections[i].pose.header.frame_id;
				}
				if(!tagDetections.detections[i].pose.header.stamp.isZero())
				{
					p.header.stamp = tagDetections.detections[i].pose.header.stamp;
				}
				uInsert(tags_, std::make_pair(tagDetections.detections[i].id[0], p));
			}
		}
	}
}
#endif

void CoreWrapper::imuAsyncCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
	if(!paused_)
	{
		if(msg->orientation.x == 0 && msg->orientation.y == 0 && msg->orientation.z == 0 && msg->orientation.w == 0)
		{
			UERROR("IMU received doesn't have orientation set, it is ignored.");
		}
		else
		{
			double stamp = timestampFromROS(msg->header.stamp);
			rtabmap::Transform localTransform = rtabmap::Transform::getIdentity();
			if(frameId_.compare(msg->header.frame_id) != 0)
			{
				localTransform = getTransform(frameId_, msg->header.frame_id, msg->header.stamp, *tfBuffer_, waitForTransform_);
				if(localTransform.isNull())
				{
					return;
				}
			}

			Transform orientation(0,0,0, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
			imus_.insert(std::make_pair(stamp, orientation*localTransform.inverse()));
			if(imus_.size() > 1000)
			{
				imus_.erase(imus_.begin());
			}
		}
	}
}

void CoreWrapper::interOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	if(!paused_)
	{
		interOdoms_.push_back(std::make_pair(*msg, rtabmap_ros::msg::OdomInfo()));
	}
}

void CoreWrapper::interOdomInfoCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg1, const rtabmap_ros::msg::OdomInfo::ConstSharedPtr & msg2)
{
	if(!paused_)
	{
		interOdoms_.push_back(std::make_pair(*msg1, *msg2));
	}
}


void CoreWrapper::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
	Transform intialPose = rtabmap_ros::transformFromPoseMsg(msg->pose.pose);
	if(intialPose.isNull())
	{
		RCLCPP_ERROR(this->get_logger(), "Pose received is null!");
		return;
	}

	rtabmap_.setInitialPose(intialPose);
}

void trackingPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr poseMsg, std::deque<geometry_msgs::msg::PoseStamped> & trackingPoseMsgs)
{
	trackingPoseMutex.lock();
	trackingPoseMsgs.push_back(*poseMsg);
	while (trackingPoseMsgs.size() > 100)
		trackingPoseMsgs.pop_front();
	trackingPoseMutex.unlock();
}

void subscribeTrackingPose(std::deque<geometry_msgs::msg::PoseStamped> & trackingPoseMsgs)
{
	auto node = std::make_shared<rclcpp::Node>("_trackingPoseSub");
	std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> fnc = std::bind(&trackingPoseCallback, std::placeholders::_1, std::ref(trackingPoseMsgs));
	auto trackingPoseSub = node->create_subscription<geometry_msgs::msg::PoseStamped>("tracking_pose", rclcpp::SensorDataQoS(), fnc);

	rclcpp::spin(node);
	rclcpp::shutdown();
}

void CoreWrapper::setTrackingPoint(const rclcpp::Time & stamp)
{
	if(trackingPoseMsgs_.empty()) return;
	geometry_msgs::msg::PoseStamped trackingPose;
	trackingPoseMutex.lock();
	auto iter = std::lower_bound(trackingPoseMsgs_.begin(), trackingPoseMsgs_.end(), stamp,
			[](geometry_msgs::msg::PoseStamped a, rclcpp::Time b) -> bool { return rclcpp::Time(a.header.stamp) < b;});
	if(iter != trackingPoseMsgs_.end())
	{
		RCLCPP_WARN(this->get_logger(), "detla time between image and tracking pose: %f", stamp.seconds() - rclcpp::Time(iter->header.stamp).seconds());
		if(rclcpp::Time(trackingPoseMsgs_.back().header.stamp) - stamp >= rclcpp::Duration(2, 0))
		{
			trackingPoseMutex.unlock();
			return;
		}
		trackingPose = *iter;
	}
	else if(stamp - rclcpp::Time(trackingPoseMsgs_.back().header.stamp) < rclcpp::Duration(2, 0))
	{
		trackingPose = trackingPoseMsgs_.back();
	}
	else
	{
		trackingPoseMutex.unlock();
		return;
	}
	trackingPoseMutex.unlock();
	auto trackingPoseTransform = rtabmap_ros::getTransform(mapFrameId_, trackingPose.header.frame_id, trackingPose.header.stamp, *tfBuffer_, waitForTransform_);
	if(trackingPoseTransform.isNull())
	{
		RCLCPP_INFO(this->get_logger(), "Failed to get transform between map and camera_link frames");
		return;
	}
	cv::Mat trackingPoint = cv::Mat::zeros(4, 1, CV_32FC1);
	trackingPoint.at<float>(0, 0) = trackingPose.pose.position.x;
	trackingPoint.at<float>(1, 0) = trackingPose.pose.position.y;
	trackingPoint.at<float>(2, 0) = trackingPose.pose.position.z;
	cv::Mat point_mat = trackingPoseTransform.rotationMatrix() * trackingPoint.rowRange(0, 3) + trackingPoseTransform.translationMatrix();
	trackingPointsInMapFrame_.push_back(point_mat.at<float>(0, 0));
	trackingPointsInMapFrame_.push_back(point_mat.at<float>(1, 0));

	// rm the obstacle points cloud of tracked person
	trackingPoseTransform = rtabmap_ros::getTransform(frameId_, trackingPose.header.frame_id, trackingPose.header.stamp, *tfBuffer_, waitForTransform_);
	float x, y, z, roll, pitch, yaw;
	trackingPoseTransform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
	Transform trackingPointRotated = Transform(x, y, z,roll, pitch, yaw) * Transform(trackingPose.pose.position.x, trackingPose.pose.position.y, trackingPose.pose.position.z,
			trackingPose.pose.orientation.x, trackingPose.pose.orientation.y, trackingPose.pose.orientation.z, trackingPose.pose.orientation.w);
	trackingPoseTransform = rtabmap_ros::getTransform(odomFrameId_, frameId_, trackingPose.header.stamp, *tfBuffer_, waitForTransform_);
	trackingPoseTransform.getEulerAngles(roll, pitch, yaw);
	trackingPointRotated = Transform(0,0,0,roll,pitch,0) * trackingPointRotated;

	trackingPointsInOdomFrame_.push_back(trackingPointRotated.x());
	trackingPointsInOdomFrame_.push_back(trackingPointRotated.y());

	OccupancyGrid::trackingPoints.clear();
	std::copy(trackingPointsInOdomFrame_.begin(), trackingPointsInOdomFrame_.end(), std::back_inserter(OccupancyGrid::trackingPoints));
	trackingPointsInOdomFrame_.clear();
	mapsManager_.setTrackingPoints(trackingPointsInMapFrame_);
	trackingPointsInMapFrame_.clear();
}

void CoreWrapper::goalCommonCallback(
		int id,
		const std::string & label,
		const std::string & frameId,
		const Transform & pose,
		const rclcpp::Time & stamp,
		double * planningTime)
{
	UTimer timer;

	if(id == 0 && !label.empty() && rtabmap_.getMemory())
	{
		id = rtabmap_.getMemory()->getSignatureIdByLabel(label);
	}

	if(id > 0)
	{
		RCLCPP_INFO(this->get_logger(), "Planning: set goal to node %d", id);
	}
	else if(id < 0)
	{
		RCLCPP_INFO(this->get_logger(), "Planning: set goal to landmark %d", id);
	}
	else if(!pose.isNull())
	{
		RCLCPP_INFO(this->get_logger(), "Planning: set goal %s", pose.prettyPrint().c_str());
	}

	if(planningTime)
	{
		*planningTime = 0.0;
	}

	bool success = false;
	if((id != 0 && rtabmap_.computePath(id, true)) ||
	   (!pose.isNull() && rtabmap_.computePath(pose)))
	{
		if(planningTime)
		{
			*planningTime = timer.elapsed();
		}
		RCLCPP_INFO(this->get_logger(), "Planning: Time computing path = %f s", timer.ticks());
		const std::vector<std::pair<int, Transform> > & poses = rtabmap_.getPath();

		currentMetricGoal_.setNull();
		lastPublishedMetricGoal_.setNull();
		goalFrameId_.clear();
		latestNodeWasReached_ = false;
		if(poses.size() == 0)
		{
			RCLCPP_WARN(this->get_logger(), "Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
					rtabmap_.getGoalReachedRadius());
			rtabmap_.clearPath(1);
			if(goalReachedPub_ != nullptr && goalReachedPub_->get_subscription_count())
			{
				std_msgs::msg::Bool result;
				result.data = true;
				goalReachedPub_->publish(result);
			}
			success = true;
		}
		else
		{
			currentMetricGoal_ = rtabmap_.getPose(rtabmap_.getPathCurrentGoalId());
			if(!currentMetricGoal_.isNull())
			{
				RCLCPP_INFO(this->get_logger(), "Planning: Path successfully created (size=%d)", (int)poses.size());
				goalFrameId_ = frameId;

				// Adjust the target pose relative to last node
				if(rtabmap_.getPathCurrentGoalId() == rtabmap_.getPath().back().first && rtabmap_.getLocalOptimizedPoses().size())
				{
					if(rtabmap_.getLastLocalizationPose().getDistance(currentMetricGoal_) < rtabmap_.getLocalRadius())
					{
						latestNodeWasReached_ = true;
						Transform goalLocalTransform = Transform::getIdentity();
						if(!goalFrameId_.empty() && goalFrameId_.compare(frameId_) != 0)
						{
							Transform localT = rtabmap_ros::getTransform(frameId_, goalFrameId_, stamp, *tfBuffer_, waitForTransform_);
							if(!localT.isNull())
							{
								goalLocalTransform = localT.inverse().to3DoF();
							}
						}
						currentMetricGoal_ *= rtabmap_.getPathTransformToGoal() * goalLocalTransform;
					}
				}

				publishCurrentGoal(stamp);
				publishLocalPath(stamp);
				publishGlobalPath(stamp);

				// Just output the path on screen
				std::stringstream stream;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					if(iter != poses.begin())
					{
						stream << " ";
					}
					stream << iter->first;
				}
				RCLCPP_INFO(this->get_logger(), "Global path: [%s]", stream.str().c_str());
				success=true;
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Pose of node %d not found!? Cannot send a metric goal...", rtabmap_.getPathCurrentGoalId());
			}
		}
	}
	else if(!label.empty())
	{
		RCLCPP_ERROR(this->get_logger(), "Planning: Node with label \"%s\" not found!", label.c_str());
	}
	else if(pose.isNull())
	{
		if(id > 0)
		{
			RCLCPP_ERROR(this->get_logger(), "Planning: Could not plan to node %d! The node is not in map's graph (look for warnings before this message for more details).", id);
		}
		else if(id < 0)
		{
			RCLCPP_ERROR(this->get_logger(), "Planning: Could not plan to landmark %d! The landmark is not in map's graph (look for warnings before this message for more details).", id);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Planning: Node id should be > 0 !");
		}
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Planning: A node near the goal's pose not found! The pose may be to far from the graph (RGBD/LocalRadius=%f m)", rtabmap_.getLocalRadius());
	}

	if(!success)
	{
		rtabmap_.clearPath(-1);
		if(goalReachedPub_ != nullptr && goalReachedPub_->get_subscription_count())
		{
			std_msgs::msg::Bool result;
			result.data = false;
			goalReachedPub_->publish(result);
		}
	}
}

void CoreWrapper::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	Transform targetPose = rtabmap_ros::transformFromPoseMsg(msg->pose);
	if(targetPose.isNull())
	{
		RCLCPP_ERROR(this->get_logger(), "Pose received is null!");
		if(goalReachedPub_ != nullptr && goalReachedPub_->get_subscription_count())
		{
			std_msgs::msg::Bool result;
			result.data = false;
			goalReachedPub_->publish(result);
		}
		return;
	}

	// transform goal in /map frame
	if(mapFrameId_.compare(msg->header.frame_id) != 0)
	{
		Transform t = rtabmap_ros::getTransform(mapFrameId_, msg->header.frame_id, msg->header.stamp, *tfBuffer_, waitForTransform_);
		if(t.isNull())
		{
			RCLCPP_ERROR(this->get_logger(), "Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
					msg->header.frame_id.c_str(), mapFrameId_.c_str());
			if(goalReachedPub_->get_subscription_count())
			{
				std_msgs::msg::Bool result;
				result.data = false;
				goalReachedPub_->publish(result);
			}
			return;
		}
		targetPose = t * targetPose;
	}

	goalCommonCallback(0, "", "", targetPose, msg->header.stamp);
}

void CoreWrapper::goalNodeCallback(const rtabmap_ros::msg::Goal::SharedPtr msg)
{
	if(msg->node_id == 0 && msg->node_label.empty())
	{
		RCLCPP_ERROR(this->get_logger(), "Node id or label should be set!");
		if(goalReachedPub_ != nullptr && goalReachedPub_->get_subscription_count())
		{
			std_msgs::msg::Bool result;
			result.data = false;
			goalReachedPub_->publish(result);
		}
		return;
	}
	goalCommonCallback(msg->node_id, msg->node_label, msg->frame_id, Transform(), msg->header.stamp);
}

void CoreWrapper::processImageCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set localization mode");
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	pubLocalizationOnce_ = true;

	// increase the sccess rate of relocation at the begining
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kRtabmapLoopThr(), "0.11"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kVisMinInliers(), "15"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kVisMinInliersDistribution(), "0.0"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kIcpMaxTranslation(), "0.6"));

	if (req->data) {
		RCLCPP_INFO(this->get_logger(), "rtabmap: set processImage true");
		processImage_ = true;
		res->message = "set true, process image";
	} else {
		RCLCPP_INFO(this->get_logger(), "rtabmap: set processImage false");
		processImage_ = false;
		res->message = "set false, not process image";
	}
	res->success = true;
}

void CoreWrapper::resetRtabmap()
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Reset");
	rtabmap_.resetMemory();
	covariance_ = cv::Mat();
	lastPose_.setIdentity();
	lastPoseIntermediate_ = false;
	currentMetricGoal_.setNull();
	lastPublishedMetricGoal_.setNull();
	goalFrameId_.clear();
	latestNodeWasReached_ = false;
	mapsManager_.clear();
	previousStamp_ = rclcpp::Time(0);
	globalPose_.header.stamp = rclcpp::Time(0);
	gps_ = rtabmap::GPS();
	tags_.clear();
	userDataMutex_.lock();
	userData_ = cv::Mat();
	userDataMutex_.unlock();
	imus_.clear();
	interOdoms_.clear();
	lastSignatureIdInDB_ = 0;
}

void CoreWrapper::resetRtabmapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	resetRtabmap();
}

void CoreWrapper::pauseRtabmapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(paused_)
	{
		RCLCPP_WARN(this->get_logger(), "rtabmap: Already paused!");
	}
	else
	{
		paused_ = true;
		RCLCPP_INFO(this->get_logger(), "rtabmap: paused!");
		set_parameter(rclcpp::Parameter("is_rtabmap_paused", true));
	}
}

void CoreWrapper::resumeRtabmapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(!paused_)
	{
		RCLCPP_WARN(this->get_logger(), "rtabmap: Already running!");
	}
	else
	{
		paused_ = false;
		RCLCPP_INFO(this->get_logger(), "rtabmap: resumed!");
		set_parameter(rclcpp::Parameter("is_rtabmap_paused", false));
	}
}

void CoreWrapper::triggerNewMapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Trigger new map");
	rtabmap_.triggerNewMap();
}

void CoreWrapper::backupDatabaseCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "Backup: Saving memory...");
	rtabmap_.close();
	RCLCPP_INFO(this->get_logger(), "Backup: Saving memory... done!");

	covariance_ = cv::Mat();
	lastPose_.setIdentity();
	currentMetricGoal_.setNull();
	lastPublishedMetricGoal_.setNull();
	goalFrameId_.clear();
	latestNodeWasReached_ = false;
	userDataMutex_.lock();
	userData_ = cv::Mat();
	userDataMutex_.unlock();
	globalPose_.header.stamp = rclcpp::Time(0);
	gps_ = rtabmap::GPS();
	tags_.clear();

	RCLCPP_INFO(this->get_logger(), "Backup: Saving \"%s\" to \"%s\"...", databasePath_.c_str(), (databasePath_+".back").c_str());
	UFile::copy(databasePath_, databasePath_+".back");
	RCLCPP_INFO(this->get_logger(), "Backup: Saving \"%s\" to \"%s\"... done!", databasePath_.c_str(), (databasePath_+".back").c_str());

	RCLCPP_INFO(this->get_logger(), "Backup: Reloading memory...");
	rtabmap_.init(parameters_, databasePath_);
	RCLCPP_INFO(this->get_logger(), "Backup: Reloading memory... done!");
}

void CoreWrapper::setModeLocalizationCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set localization mode");
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
	// rtabmap_.parseParameters(parameters); // conflict with set_parameter(...)
	pubLocalizationOnce_ = true;

	// increase the sccess rate of relocation at the begining
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kRtabmapLoopThr(), "0.11"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kVisMinInliers(), "15"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kVisMinInliersDistribution(), "0.0"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kIcpMaxTranslation(), "0.6"));
}

void CoreWrapper::setModeMappingCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set mapping mode");
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
	set_parameter(rclcpp::Parameter(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
	// rtabmap_.parseParameters(parameters);
	processImage_ = true;

	if(pubLocalizationOnce_)
	{
		for(auto iter : partOfLocalizationParameters_)
			set_parameter(rclcpp::Parameter(iter.first, iter.second));
		pubLocalizationOnce_ = false;
	}
}

void CoreWrapper::setLogDebug(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set log level to Debug");
	ULogger::setLevel(ULogger::kDebug);
}
void CoreWrapper::setLogInfo(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set log level to Info");
	ULogger::setLevel(ULogger::kInfo);
}
void CoreWrapper::setLogWarn(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set log level to Warning");
	ULogger::setLevel(ULogger::kWarning);
}
void CoreWrapper::setLogError(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Set log level to Error");
	ULogger::setLevel(ULogger::kError);
}

void CoreWrapper::getMapDataCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_ros::srv::GetMap::Request> req,
		std::shared_ptr<rtabmap_ros::srv::GetMap::Response> res)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Getting map (global=%s optimized=%s graphOnly=%s)...",
			req->global?"true":"false",
			req->optimized?"true":"false",
			req->graph_only?"true":"false");
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, rtabmap::Link> constraints;

	if(req->graph_only)
	{
		rtabmap_.getGraph(
				poses,
				constraints,
				req->optimized,
				req->global,
				&signatures);
	}
	else
	{
		rtabmap_.get3DMap(
				signatures,
				poses,
				constraints,
				req->optimized,
				req->global);
	}

	//RGB-D SLAM data
	rtabmap_ros::mapDataToROS(poses,
		constraints,
		signatures,
		mapToOdom_,
		res->data);

	res->data.header.stamp = now();
	res->data.header.frame_id = mapFrameId_;
}

void CoreWrapper::getMapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
		std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
	// create the grid map
	float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
	cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);

	if(!pixels.empty())
	{
		//init
		res->map.info.resolution = gridCellSize;
		res->map.info.origin.position.x = 0.0;
		res->map.info.origin.position.y = 0.0;
		res->map.info.origin.position.z = 0.0;
		res->map.info.origin.orientation.x = 0.0;
		res->map.info.origin.orientation.y = 0.0;
		res->map.info.origin.orientation.z = 0.0;
		res->map.info.origin.orientation.w = 1.0;

		res->map.info.width = pixels.cols;
		res->map.info.height = pixels.rows;
		res->map.info.origin.position.x = xMin;
		res->map.info.origin.position.y = yMin;
		res->map.data.resize(res->map.info.width * res->map.info.height);

		memcpy(res->map.data.data(), pixels.data, res->map.info.width * res->map.info.height);

		res->map.header.frame_id = mapFrameId_;
		res->map.header.stamp = now();
	}
}

void CoreWrapper::getProbMapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
		std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
	// create the grid map
	float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
	cv::Mat pixels = mapsManager_.getGridProbMap(xMin, yMin, gridCellSize);

	if(!pixels.empty())
	{
		//init
		res->map.info.resolution = gridCellSize;
		res->map.info.origin.position.x = 0.0;
		res->map.info.origin.position.y = 0.0;
		res->map.info.origin.position.z = 0.0;
		res->map.info.origin.orientation.x = 0.0;
		res->map.info.origin.orientation.y = 0.0;
		res->map.info.origin.orientation.z = 0.0;
		res->map.info.origin.orientation.w = 1.0;

		res->map.info.width = pixels.cols;
		res->map.info.height = pixels.rows;
		res->map.info.origin.position.x = xMin;
		res->map.info.origin.position.y = yMin;
		res->map.data.resize(res->map.info.width * res->map.info.height);

		memcpy(res->map.data.data(), pixels.data, res->map.info.width * res->map.info.height);

		res->map.header.frame_id = mapFrameId_;
		res->map.header.stamp = now();
	}
}

void CoreWrapper::publishMapCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_ros::srv::PublishMap::Request> req,
		std::shared_ptr<rtabmap_ros::srv::PublishMap::Response>)
{
	RCLCPP_INFO(this->get_logger(), "rtabmap: Publishing map...");

	if (mapDataPub_ == nullptr || labelsPub_ == nullptr || mapGraphPub_ == nullptr || mapPathPub_ == nullptr) return;
	if(mapDataPub_->get_subscription_count() ||
	   (!req->graph_only && mapsManager_.hasSubscribers()) ||
	   (req->graph_only && (labelsPub_->get_subscription_count() || mapGraphPub_->get_subscription_count() || mapPathPub_->get_subscription_count())))
	{
		std::map<int, Transform> poses;
		std::multimap<int, rtabmap::Link> constraints;
		std::map<int, Signature > signatures;

		if(req->graph_only)
		{
			rtabmap_.getGraph(
					poses,
					constraints,
					req->optimized,
					req->global,
					&signatures);
		}
		else
		{
			rtabmap_.get3DMap(
					signatures,
					poses,
					constraints,
					req->optimized,
					req->global);
		}

		rclcpp::Time stampNow = now();
		if(mapDataPub_->get_subscription_count())
		{
			rtabmap_ros::msg::MapData::UniquePtr msg(new rtabmap_ros::msg::MapData);
			msg->header.stamp = stampNow;
			msg->header.frame_id = mapFrameId_;

			rtabmap_ros::mapDataToROS(poses,
				constraints,
				signatures,
				mapToOdom_,
				*msg);

			mapDataPub_->publish(std::move(msg));
		}

		if(mapGraphPub_->get_subscription_count())
		{
			rtabmap_ros::msg::MapGraph::UniquePtr msg(new rtabmap_ros::msg::MapGraph);
			msg->header.stamp = stampNow;
			msg->header.frame_id = mapFrameId_;

			rtabmap_ros::mapGraphToROS(poses,
				constraints,
				mapToOdom_,
				*msg);

			mapGraphPub_->publish(std::move(msg));
		}

		bool pubLabels = labelsPub_->get_subscription_count();
		visualization_msgs::msg::MarkerArray markers;
		if(landmarksPub_ != nullptr && (landmarksPub_->get_subscription_count() || pubLabels) && !poses.empty() && poses.begin()->first < 0)
		{
			geometry_msgs::msg::PoseArray::UniquePtr msg(new geometry_msgs::msg::PoseArray);
			msg->header.stamp = stampNow;
			msg->header.frame_id = mapFrameId_;
			for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end() && iter->first<0; ++iter)
			{
				geometry_msgs::msg::Pose p;
				rtabmap_ros::transformToPoseMsg(iter->second, p);
				msg->poses.push_back(p);

				if(pubLabels)
				{
					// Add landmark ids
					visualization_msgs::msg::Marker marker;
					marker.header.frame_id = mapFrameId_;
					marker.header.stamp = stampNow;
					marker.ns = "landmarks";
					marker.id = iter->first;
					marker.action = visualization_msgs::msg::Marker::ADD;
					marker.pose.position.x = iter->second.x();
					marker.pose.position.y = iter->second.y();
					marker.pose.position.z = iter->second.z();
					marker.pose.orientation.x = 0.0;
					marker.pose.orientation.y = 0.0;
					marker.pose.orientation.z = 0.0;
					marker.pose.orientation.w = 1.0;
					marker.scale.x = 1;
					marker.scale.y = 1;
					marker.scale.z = 0.35;
					marker.color.a = 0.5;
					marker.color.r = 1.0;
					marker.color.g = 1.0;
					marker.color.b = 0.0;
					marker.lifetime = rclcpp::Duration(2.0f/rate_);

					marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
					marker.text = uNumber2Str(iter->first);

					markers.markers.push_back(marker);
				}
			}

			landmarksPub_->publish(std::move(msg));
		}

		if(!req->graph_only && mapsManager_.hasSubscribers())
		{
			std::map<int, Transform> filteredPoses(poses.lower_bound(1), poses.end());
			if(maxMappingNodes_ > 0 && filteredPoses.size()>1)
			{
				std::map<int, Transform> nearestPoses;
				std::map<int, float> nodes = graph::findNearestNodes(filteredPoses, filteredPoses.rbegin()->second, maxMappingNodes_);
				for(std::map<int, float>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
				{
					std::map<int, Transform>::iterator pter = filteredPoses.find(iter->first);
					if(pter != filteredPoses.end())
					{
						nearestPoses.insert(*pter);
					}
				}
			}
			if(signatures.size())
			{
				filteredPoses = mapsManager_.updateMapCaches(
						filteredPoses,
						rtabmap_.getMemory(),
						false,
						false,
						signatures);
			}
			else
			{
				filteredPoses = mapsManager_.getFilteredPoses(filteredPoses);
			}
			mapsManager_.publishMaps(filteredPoses, stampNow, mapFrameId_);
		}

		bool pubPath = mapPathPub_->get_subscription_count();
		if(pubLabels || pubPath)
		{
			if(poses.size() && signatures.size())
			{
				nav_msgs::msg::Path path;
				if(pubPath)
				{
					path.poses.resize(poses.size());
				}
				int oi=0;
				for(std::map<int, Signature>::const_iterator iter=signatures.begin();
					iter!=signatures.end();
					++iter)
				{
					std::map<int, Transform>::const_iterator poseIter= poses.find(iter->first);
					if(poseIter!=poses.end())
					{
						if(pubLabels)
						{
							// Add labels
							if(!iter->second.getLabel().empty())
							{
								visualization_msgs::msg::Marker marker;
								marker.header.frame_id = mapFrameId_;
								marker.header.stamp = stampNow;
								marker.ns = "labels";
								marker.id = iter->first;
								marker.action = visualization_msgs::msg::Marker::ADD;
								marker.pose.position.x = poseIter->second.x();
								marker.pose.position.y = poseIter->second.y();
								marker.pose.position.z = poseIter->second.z();
								marker.pose.orientation.x = 0.0;
								marker.pose.orientation.y = 0.0;
								marker.pose.orientation.z = 0.0;
								marker.pose.orientation.w = 1.0;
								marker.scale.x = 1;
								marker.scale.y = 1;
								marker.scale.z = 0.5;
								marker.color.a = 0.7;
								marker.color.r = 1.0;
								marker.color.g = 0.0;
								marker.color.b = 0.0;

								marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
								marker.text = iter->second.getLabel();

								markers.markers.push_back(marker);
							}
							// Add node ids
							visualization_msgs::msg::Marker marker;
							marker.header.frame_id = mapFrameId_;
							marker.header.stamp = stampNow;
							marker.ns = "ids";
							marker.id = iter->first;
							marker.action = visualization_msgs::msg::Marker::ADD;
							marker.pose.position.x = poseIter->second.x();
							marker.pose.position.y = poseIter->second.y();
							marker.pose.position.z = poseIter->second.z();
							marker.pose.orientation.x = 0.0;
							marker.pose.orientation.y = 0.0;
							marker.pose.orientation.z = 0.0;
							marker.pose.orientation.w = 1.0;
							marker.scale.x = 1;
							marker.scale.y = 1;
							marker.scale.z = 0.2;
							marker.color.a = 0.5;
							marker.color.r = 1.0;
							marker.color.g = 1.0;
							marker.color.b = 1.0;
							marker.lifetime = rclcpp::Duration(2.0f/rate_);

							marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
							marker.text = uNumber2Str(iter->first);

							markers.markers.push_back(marker);
						}
						if(pubPath)
						{
							rtabmap_ros::transformToPoseMsg(poseIter->second, path.poses.at(oi).pose);
							path.poses.at(oi).header.frame_id = mapFrameId_;
							path.poses.at(oi).header.stamp = rclcpp::Time(iter->second.getStamp());
							++oi;
						}
					}
				}

				if(pubLabels && markers.markers.size())
				{
					labelsPub_->publish(markers);
				}
				if(pubPath && oi)
				{
					path.header.frame_id = mapFrameId_;
					path.header.stamp = stampNow;
					path.poses.resize(oi);
					mapPathPub_->publish(path);
				}
			}
		}
	}
	else
	{
		UWARN("No subscribers, don't need to publish!");
	}
}

void CoreWrapper::getPlanCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
		std::shared_ptr<nav_msgs::srv::GetPlan::Response> res)
{
	Transform pose = rtabmap_ros::transformFromPoseMsg(req->goal.pose);
	UTimer timer;
	if(!pose.isNull())
	{
		// transform goal in /map frame
		Transform coordinateTransform = Transform::getIdentity();
		if(mapFrameId_.compare(req->goal.header.frame_id) != 0)
		{
			coordinateTransform = rtabmap_ros::getTransform(mapFrameId_, req->goal.header.frame_id, rclcpp::Time(req->goal.header.stamp.sec, req->goal.header.stamp.nanosec), *tfBuffer_, waitForTransform_);
			if(coordinateTransform.isNull())
			{
				RCLCPP_ERROR(this->get_logger(), "Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
						req->goal.header.frame_id.c_str(), mapFrameId_.c_str());
				return;
			}
			pose = coordinateTransform * pose;
		}

		// To convert back the poses in goal frame
		coordinateTransform = coordinateTransform.inverse();

		if(rtabmap_.computePath(pose, req->tolerance))
		{
			RCLCPP_INFO(this->get_logger(), "Planning: Time computing path = %f s", timer.ticks());
			const std::vector<std::pair<int, Transform> > & poses = rtabmap_.getPath();
			res->plan.header.frame_id = req->goal.header.frame_id;
			res->plan.header.stamp = req->goal.header.stamp;
			if(poses.size() == 0)
			{
				RCLCPP_WARN(this->get_logger(), "Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
						rtabmap_.getGoalReachedRadius());
				// just set the goal directly
				res->plan.poses.resize(1);
				rtabmap_ros::transformToPoseMsg(coordinateTransform*pose, res->plan.poses[0].pose);
			}
			else
			{
				res->plan.poses.resize(poses.size());
				int oi = 0;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					res->plan.poses[oi].header = res->plan.header;
					rtabmap_ros::transformToPoseMsg(coordinateTransform*iter->second, res->plan.poses[oi].pose);
					++oi;
				}
				if(!rtabmap_.getPathTransformToGoal().isIdentity())
				{
					res->plan.poses.resize(res->plan.poses.size()+1);
					res->plan.poses[res->plan.poses.size()-1].header = res->plan.header;
					Transform p = rtabmap_.getPath().back().second*rtabmap_.getPathTransformToGoal();
					rtabmap_ros::transformToPoseMsg(coordinateTransform*p, res->plan.poses[res->plan.poses.size()-1].pose);
				}

				// Just output the path on screen
				std::stringstream stream;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					if(iter != poses.begin())
					{
						stream << " ";
					}
					stream << iter->first;
				}
				RCLCPP_INFO(this->get_logger(), "Planned path: [%s]", stream.str().c_str());
			}
		}
		rtabmap_.clearPath(0);
	}
}

void CoreWrapper::getPlanNodesCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_ros::srv::GetPlan::Request> req,
		std::shared_ptr<rtabmap_ros::srv::GetPlan::Response> res)
{
	Transform pose = rtabmap_ros::transformFromPoseMsg(req->goal.pose);
	UTimer timer;
	if(req->goal_node > 0 || !pose.isNull())
	{
		Transform coordinateTransform = Transform::getIdentity();
		// transform goal in /map frame
		if(mapFrameId_.compare(req->goal.header.frame_id) != 0)
		{
			coordinateTransform = rtabmap_ros::getTransform(mapFrameId_, req->goal.header.frame_id, req->goal.header.stamp, *tfBuffer_, waitForTransform_);
			if(coordinateTransform.isNull())
			{
				RCLCPP_ERROR(this->get_logger(), "Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
						req->goal.header.frame_id.c_str(), mapFrameId_.c_str());
				return;
			}
			if(!pose.isNull())
			{
				pose = coordinateTransform * pose;
			}
		}

		// To convert back the poses in goal frame
		coordinateTransform = coordinateTransform.inverse();

		if((req->goal_node > 0 && rtabmap_.computePath(req->goal_node, req->tolerance)) ||
		   (req->goal_node <= 0 && rtabmap_.computePath(pose, req->tolerance)))
		{
			RCLCPP_INFO(this->get_logger(), "Planning: Time computing path = %f s", timer.ticks());
			const std::vector<std::pair<int, Transform> > & poses = rtabmap_.getPath();
			res->plan.header.frame_id = mapFrameId_;
			if(req->goal_node > 0)
			{
				res->plan.header.stamp = now();
			}
			else
			{
				res->plan.header.stamp = req->goal.header.stamp;
			}
			if(poses.size() == 0)
			{
				RCLCPP_WARN(this->get_logger(), "Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
						rtabmap_.getGoalReachedRadius());
				if(!pose.isNull())
				{
					// just set the goal directly
					res->plan.poses.resize(1);
					res->plan.node_ids.resize(1);
					rtabmap_ros::transformToPoseMsg(coordinateTransform*pose, res->plan.poses[0]);
					res->plan.node_ids[0] = 0;
				}
			}
			else
			{
				res->plan.poses.resize(poses.size());
				res->plan.node_ids.resize(poses.size());
				int oi = 0;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					rtabmap_ros::transformToPoseMsg(coordinateTransform*iter->second, res->plan.poses[oi]);
					res->plan.node_ids[oi] = iter->first;
					++oi;
				}
				if(!rtabmap_.getPathTransformToGoal().isIdentity())
				{
					res->plan.poses.resize(res->plan.poses.size()+1);
					res->plan.node_ids.resize(res->plan.node_ids.size()+1);
					Transform p = rtabmap_.getPath().back().second*rtabmap_.getPathTransformToGoal();
					rtabmap_ros::transformToPoseMsg(coordinateTransform*p, res->plan.poses[res->plan.poses.size()-1]);
					res->plan.node_ids[res->plan.node_ids.size()-1] = 0;
				}

				// Just output the path on screen
				std::stringstream stream;
				for(std::vector<std::pair<int, Transform> >::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					if(iter != poses.begin())
					{
						stream << " ";
					}
					stream << iter->first;
				}
				RCLCPP_INFO(this->get_logger(), "Planned path: [%s]", stream.str().c_str());
			}
		}
		rtabmap_.clearPath(0);
	}
}

void CoreWrapper::setGoalCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_ros::srv::SetGoal::Request> req,
		std::shared_ptr<rtabmap_ros::srv::SetGoal::Response> res)
{
	double planningTime = 0.0;
	goalCommonCallback(req->node_id, req->node_label, req->frame_id, Transform(), now(), &planningTime);
	const std::vector<std::pair<int, Transform> > & path = rtabmap_.getPath();
	res->path_ids.resize(path.size());
	res->path_poses.resize(path.size());
	res->planning_time = planningTime;
	for(unsigned int i=0; i<path.size(); ++i)
	{
		res->path_ids[i] = path[i].first;
		rtabmap_ros::transformToPoseMsg(path[i].second, res->path_poses[i]);
	}
}

void CoreWrapper::cancelGoalCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<std_srvs::srv::Empty::Request>,
		std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	if(rtabmap_.getPath().size())
	{
		RCLCPP_WARN(this->get_logger(), "Goal cancelled!");
		rtabmap_.clearPath(0);
		currentMetricGoal_.setNull();
		lastPublishedMetricGoal_.setNull();
		goalFrameId_.clear();
		latestNodeWasReached_ = false;
		if(goalReachedPub_ != nullptr && goalReachedPub_->get_subscription_count())
		{
			std_msgs::msg::Bool result;
			result.data = false;
			goalReachedPub_->publish(result);
		}
	}
#ifdef WITH_MOVE_BASE_MSGS
	if(mbClient_ && mbClient_->isServerConnected())
	{
		mbClient_->cancelGoal();
	}
#endif
}

void CoreWrapper::setLabelCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_ros::srv::SetLabel::Request> req,
		std::shared_ptr<rtabmap_ros::srv::SetLabel::Response>)
{
	if(rtabmap_.labelLocation(req->node_id, req->node_label))
	{
		if(req->node_id > 0)
		{
			RCLCPP_INFO(this->get_logger(), "Set label \"%s\" to node %d", req->node_label.c_str(), req->node_id);
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Set label \"%s\" to last node", req->node_label.c_str());
		}
	}
	else
	{
		if(req->node_id > 0)
		{
			RCLCPP_ERROR(this->get_logger(), "Could not set label \"%s\" to node %d", req->node_label.c_str(), req->node_id);
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Could not set label \"%s\" to last node", req->node_label.c_str());
		}
	}
}

void CoreWrapper::listLabelsCallback(
		const std::shared_ptr<rmw_request_id_t>,
		const std::shared_ptr<rtabmap_ros::srv::ListLabels::Request>,
		std::shared_ptr<rtabmap_ros::srv::ListLabels::Response> res)
{
	if(rtabmap_.getMemory())
	{
		std::map<int, std::string> labels = rtabmap_.getMemory()->getAllLabels();
		res->ids = uKeys(labels);
		res->labels = uValues(labels);
		RCLCPP_INFO(this->get_logger(), "List labels service: %d labels found.", (int)res->labels.size());
	}
}

void CoreWrapper::publishStats(const rclcpp::Time & stamp)
{
	if (infoPub_ == nullptr || mapDataPub_ == nullptr || mapGraphPub_ == nullptr || localGridObstacle_ == nullptr ||
			localGridEmpty_ == nullptr || localGridGround_  == nullptr || landmarksPub_  == nullptr ||  labelsPub_  == nullptr)
		return;
	UDEBUG("Publishing stats...");
	const rtabmap::Statistics & stats = rtabmap_.getStatistics();

	if(infoPub_->get_subscription_count())
	{
		//RCLCPP_INFO(this->get_logger(), "Sending RtabmapInfo msg (last_id=%d)...", stat.refImageId());
		rtabmap_ros::msg::Info::UniquePtr msg(new rtabmap_ros::msg::Info);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_ros::infoToROS(stats, *msg);
		infoPub_->publish(std::move(msg));
	}

	if(mapDataPub_->get_subscription_count())
	{
		rtabmap_ros::msg::MapData::UniquePtr msg(new rtabmap_ros::msg::MapData);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		std::map<int, Signature> signatures;
		if(stats.getLastSignatureData().id() > 0)
		{
			signatures.insert(std::make_pair(stats.getLastSignatureData().id(), stats.getLastSignatureData()));
		}
		rtabmap_ros::mapDataToROS(
			stats.poses(),
			stats.constraints(),
			signatures,
			stats.mapCorrection(),
			*msg);

		mapDataPub_->publish(std::move(msg));
	}

	if(mapGraphPub_->get_subscription_count())
	{
		rtabmap_ros::msg::MapGraph::UniquePtr msg(new rtabmap_ros::msg::MapGraph);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;

		rtabmap_ros::mapGraphToROS(
			stats.poses(),
			stats.constraints(),
			stats.mapCorrection(),
			*msg);

		mapGraphPub_->publish(std::move(msg));
	}

	if(localGridObstacle_->get_subscription_count() && !stats.getLastSignatureData().sensorData().gridObstacleCellsRaw().empty())
	{
		pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(LaserScan::backwardCompatibility(stats.getLastSignatureData().sensorData().gridObstacleCellsRaw()));
		sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2);
		pcl_conversions::moveFromPCL(*cloud, *msg);
		msg->header.stamp = stamp;
		msg->header.frame_id = frameId_;
		localGridObstacle_->publish(std::move(msg));
	}
	if(localGridEmpty_->get_subscription_count() && !stats.getLastSignatureData().sensorData().gridEmptyCellsRaw().empty())
	{
		pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(LaserScan::backwardCompatibility(stats.getLastSignatureData().sensorData().gridEmptyCellsRaw()));
		sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2);
		pcl_conversions::moveFromPCL(*cloud, *msg);
		msg->header.stamp = stamp;
		msg->header.frame_id = frameId_;
		localGridEmpty_->publish(std::move(msg));
	}
	if(localGridGround_->get_subscription_count() && !stats.getLastSignatureData().sensorData().gridGroundCellsRaw().empty())
	{
		pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(LaserScan::backwardCompatibility(stats.getLastSignatureData().sensorData().gridGroundCellsRaw()));
		sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2);
		pcl_conversions::moveFromPCL(*cloud, *msg);
		msg->header.stamp = stamp;
		msg->header.frame_id = frameId_;
		localGridGround_->publish(std::move(msg));
	}

	bool pubLabels = labelsPub_->get_subscription_count();
	visualization_msgs::msg::MarkerArray markers;
	if((landmarksPub_->get_subscription_count() || pubLabels) && !stats.poses().empty() && stats.poses().begin()->first < 0)
	{
		geometry_msgs::msg::PoseArray::UniquePtr msg(new geometry_msgs::msg::PoseArray);
		msg->header.stamp = stamp;
		msg->header.frame_id = mapFrameId_;
		for(std::map<int, Transform>::const_iterator iter=stats.poses().begin(); iter!=stats.poses().end() && iter->first<0; ++iter)
		{
			geometry_msgs::msg::Pose p;
			rtabmap_ros::transformToPoseMsg(iter->second, p);
			msg->poses.push_back(p);

			if(pubLabels)
			{
				// Add landmark ids
				visualization_msgs::msg::Marker marker;
				marker.header.frame_id = mapFrameId_;
				marker.header.stamp = stamp;
				marker.ns = "landmarks";
				marker.id = iter->first;
				marker.action = visualization_msgs::msg::Marker::ADD;
				marker.pose.position.x = iter->second.x();
				marker.pose.position.y = iter->second.y();
				marker.pose.position.z = iter->second.z();
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 1;
				marker.scale.y = 1;
				marker.scale.z = 0.35;
				marker.color.a = 0.7;
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				marker.lifetime = rclcpp::Duration(2.0f/rate_);

				marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
				marker.text = uNumber2Str(iter->first);

				markers.markers.push_back(marker);
			}
		}

		landmarksPub_->publish(std::move(msg));
	}

	bool pubPath = mapPathPub_->get_subscription_count();
	if(pubLabels || pubPath)
	{
		if(stats.poses().size())
		{
			nav_msgs::msg::Path path;
			if(pubPath)
			{
				path.poses.resize(stats.poses().size());
			}
			int oi = 0;
			for(std::map<int, Transform>::const_iterator poseIter=stats.poses().begin();
				poseIter!=stats.poses().end();
				++poseIter)
			{
				if(pubLabels && rtabmap_.getMemory())
				{
					// Add labels
					std::map<int, std::string>::const_iterator lter = rtabmap_.getMemory()->getAllLabels().find(poseIter->first);
					if(lter != rtabmap_.getMemory()->getAllLabels().end() && !lter->second.empty())
					{
						visualization_msgs::msg::Marker marker;
						marker.header.frame_id = mapFrameId_;
						marker.header.stamp = stamp;
						marker.ns = "labels";
						marker.id = -poseIter->first;
						marker.action = visualization_msgs::msg::Marker::ADD;
						marker.pose.position.x = poseIter->second.x();
						marker.pose.position.y = poseIter->second.y();
						marker.pose.position.z = poseIter->second.z();
						marker.pose.orientation.x = 0.0;
						marker.pose.orientation.y = 0.0;
						marker.pose.orientation.z = 0.0;
						marker.pose.orientation.w = 1.0;
						marker.scale.x = 1;
						marker.scale.y = 1;
						marker.scale.z = 0.5;
						marker.color.a = 0.7;
						marker.color.r = 1.0;
						marker.color.g = 0.0;
						marker.color.b = 0.0;

						marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
						marker.text = lter->second;

						markers.markers.push_back(marker);
					}

					// Add node ids
					visualization_msgs::msg::Marker marker;
					marker.header.frame_id = mapFrameId_;
					marker.header.stamp = stamp;
					marker.ns = "ids";
					marker.id = poseIter->first;
					marker.action = visualization_msgs::msg::Marker::ADD;
					marker.pose.position.x = poseIter->second.x();
					marker.pose.position.y = poseIter->second.y();
					marker.pose.position.z = poseIter->second.z();
					marker.pose.orientation.x = 0.0;
					marker.pose.orientation.y = 0.0;
					marker.pose.orientation.z = 0.0;
					marker.pose.orientation.w = 1.0;
					marker.scale.x = 1;
					marker.scale.y = 1;
					marker.scale.z = 0.2;
					marker.color.a = 0.5;
					marker.color.r = 1.0;
					marker.color.g = 1.0;
					marker.color.b = 1.0;
					marker.lifetime = rclcpp::Duration(2.0f/rate_);

					marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
					marker.text = uNumber2Str(poseIter->first);

					markers.markers.push_back(marker);
				}
				if(pubPath)
				{
					rtabmap_ros::transformToPoseMsg(poseIter->second, path.poses.at(oi).pose);
					path.poses.at(oi).header.frame_id = mapFrameId_;
					path.poses.at(oi).header.stamp = stamp;
					++oi;
				}
			}

			if(pubLabels && markers.markers.size())
			{
				labelsPub_->publish(markers);
			}
			if(pubPath && oi)
			{
				path.header.frame_id = mapFrameId_;
				path.header.stamp = stamp;
				path.poses.resize(oi);
				mapPathPub_->publish(path);
			}
		}
	}
}

void CoreWrapper::publishCurrentGoal(const rclcpp::Time & stamp)
{
	if(!currentMetricGoal_.isNull() && currentMetricGoal_ != lastPublishedMetricGoal_)
	{
		RCLCPP_INFO(this->get_logger(), "Publishing next goal: %d -> %s",
				rtabmap_.getPathCurrentGoalId(), currentMetricGoal_.prettyPrint().c_str());

		geometry_msgs::msg::PoseStamped poseMsg;
		poseMsg.header.frame_id = mapFrameId_;
		poseMsg.header.stamp = stamp;
		rtabmap_ros::transformToPoseMsg(currentMetricGoal_, poseMsg.pose);
#ifdef WITH_MOVE_BASE_MSGS
		if(useActionForGoal_)
		{
			if(mbClient_ == 0 || !mbClient_->isServerConnected())
			{
				RCLCPP_INFO(this->get_logger(), "Connecting to move_base action server...");
				if(mbClient_ == 0)
				{
					mbClient_ = new MoveBaseClient("move_base", true);
				}
				mbClient_->waitForServer(rclcpp::Duration(5.0));
			}
			if(mbClient_ && mbClient_->isServerConnected())
			{
				move_base_msgs::MoveBaseGoal goal;
				goal.target_pose = poseMsg;

				mbClient_->sendGoal(goal,
						boost::bind(&CoreWrapper::goalDoneCb, this, _1, _2),
						boost::bind(&CoreWrapper::goalActiveCb, this),
						boost::bind(&CoreWrapper::goalFeedbackCb, this, _1));
				lastPublishedMetricGoal_ = currentMetricGoal_;
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Cannot connect to move_base action server (called \"%s\")!", this->getNodeHandle().resolveName("move_base").c_str());
			}
		}
#endif
		if(nextMetricGoalPub_ != nullptr && nextMetricGoalPub_->get_subscription_count())
		{
			nextMetricGoalPub_->publish(poseMsg);
			if(!useActionForGoal_)
			{
				lastPublishedMetricGoal_ = currentMetricGoal_;
			}
		}
	}
}

#ifdef WITH_MOVE_BASE_MSGS
// Called once when the goal completes
void CoreWrapper::goalDoneCb(const actionlib::SimpleClientGoalState& state,
             const move_base_msgs::MoveBaseResult::SharedPtr& result)
{
	bool ignore = false;
	if(!currentMetricGoal_.isNull())
	{
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			if(rtabmap_.getPath().size() &&
				rtabmap_.getPathCurrentGoalId() != rtabmap_.getPath().back().first &&
				(!uContains(rtabmap_.getLocalOptimizedPoses(), rtabmap_.getPath().back().first) || !latestNodeWasReached_))
			{
				RCLCPP_WARN(this->get_logger(), "Planning: move_base reached current goal but it is not "
						 "the last one planned by rtabmap. A new goal should be sent when "
						 "rtabmap will be able to retrieve next locations on the path.");
				ignore = true;
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "Planning: move_base success!");
			}
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Planning: move_base failed for some reason. Aborting the plan...");
		}

		if(!ignore && goalReachedPub_ != nullptr && goalReachedPub_->get_subscription_count())
		{
			std_msgs::msg::Bool result;
			result.data = state == actionlib::SimpleClientGoalState::SUCCEEDED;
			goalReachedPub_->publish(result);
		}
	}

	if(!ignore)
	{
		rtabmap_.clearPath(1);
		currentMetricGoal_.setNull();
		lastPublishedMetricGoal_.setNull();
		goalFrameId_.clear();
		latestNodeWasReached_ = false;
	}
}

// Called once when the goal becomes active
void CoreWrapper::goalActiveCb()
{
	//RCLCPP_INFO(this->get_logger(), "Planning: Goal just went active");
}

// Called every time feedback is received for the goal
void CoreWrapper::goalFeedbackCb(const move_base_msgs::MoveBaseFeedback::SharedPtr& feedback)
{
	//Transform basePosition = rtabmap_ros::transformFromPoseMsg(feedback->base_position.pose);
	//RCLCPP_INFO(this->get_logger(), "Planning: feedback base_position = %s", basePosition.prettyPrint().c_str());
}
#endif

void CoreWrapper::publishLocalPath(const rclcpp::Time & stamp)
{
	if(rtabmap_.getPath().size())
	{
		std::vector<std::pair<int, Transform> > poses = rtabmap_.getPathNextPoses();
		if(poses.size())
		{
			if((localPathPub_ != nullptr && localPathPub_->get_subscription_count()) || (localPathNodesPub_ != nullptr && localPathNodesPub_->get_subscription_count()))
			{
				nav_msgs::msg::Path path;
				rtabmap_ros::msg::Path pathNodes;
				path.header.frame_id = pathNodes.header.frame_id = mapFrameId_;
				path.header.stamp = pathNodes.header.stamp = stamp;
				path.poses.resize(poses.size());
				pathNodes.node_ids.resize(poses.size());
				pathNodes.poses.resize(poses.size());
				int oi = 0;
				for(std::vector<std::pair<int, Transform> >::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					path.poses[oi].header = path.header;
					rtabmap_ros::transformToPoseMsg(iter->second, path.poses[oi].pose);
					pathNodes.poses[oi] = path.poses[oi].pose;
					pathNodes.node_ids[oi] = iter->first;
					++oi;
				}
				if(localPathPub_ != nullptr && localPathPub_->get_subscription_count())
				{
					localPathPub_->publish(path);
				}
				if(localPathNodesPub_ != nullptr && localPathNodesPub_->get_subscription_count())
				{
					localPathNodesPub_->publish(pathNodes);
				}
			}
		}
	}
}

void CoreWrapper::publishGlobalPath(const rclcpp::Time & stamp)
{
	if(((globalPathPub_ != nullptr && globalPathPub_->get_subscription_count()) || (globalPathNodesPub_ != nullptr && globalPathNodesPub_->get_subscription_count())) && rtabmap_.getPath().size())
	{
		Transform pose = uValue(rtabmap_.getLocalOptimizedPoses(), rtabmap_.getPathCurrentGoalId(), Transform());
		if(!pose.isNull() && rtabmap_.getPathCurrentGoalIndex() < rtabmap_.getPath().size())
		{
			// transform the global path in the goal referential
			Transform t = pose * rtabmap_.getPath().at(rtabmap_.getPathCurrentGoalIndex()).second.inverse();

			nav_msgs::msg::Path path;
			rtabmap_ros::msg::Path pathNodes;
			path.header.frame_id = pathNodes.header.frame_id = mapFrameId_;
			path.header.stamp = pathNodes.header.stamp = stamp;
			path.poses.resize(rtabmap_.getPath().size());
			pathNodes.node_ids.resize(rtabmap_.getPath().size());
			pathNodes.poses.resize(rtabmap_.getPath().size());
			int oi = 0;
			for(std::vector<std::pair<int, Transform> >::const_iterator iter=rtabmap_.getPath().begin(); iter!=rtabmap_.getPath().end(); ++iter)
			{
				path.poses[oi].header = path.header;
				rtabmap_ros::transformToPoseMsg(t*iter->second, path.poses[oi].pose);
				pathNodes.poses[oi] = path.poses[oi].pose;
				pathNodes.node_ids[oi] = iter->first;
				++oi;
			}
			Transform goalLocalTransform = Transform::getIdentity();
			if(!goalFrameId_.empty() && goalFrameId_.compare(frameId_) != 0)
			{
				Transform localT = rtabmap_ros::getTransform(frameId_, goalFrameId_, stamp, *tfBuffer_, waitForTransform_);
				if(!localT.isNull())
				{
					goalLocalTransform = localT.inverse().to3DoF();
				}
			}

			if(!rtabmap_.getPathTransformToGoal().isIdentity() || !goalLocalTransform.isIdentity())
			{
				path.poses.resize(path.poses.size()+1);
				path.poses[path.poses.size()-1].header = path.header;
				pathNodes.node_ids.resize(pathNodes.node_ids.size()+1);
				pathNodes.poses.resize(pathNodes.poses.size()+1);
				Transform p = t * rtabmap_.getPath().back().second*rtabmap_.getPathTransformToGoal() * goalLocalTransform;
				rtabmap_ros::transformToPoseMsg(p, path.poses[path.poses.size()-1].pose);
				pathNodes.poses[pathNodes.poses.size()-1] = path.poses[path.poses.size()-1].pose;
				pathNodes.node_ids[pathNodes.node_ids.size()-1] = 0;
			}
			if(globalPathPub_ != nullptr && globalPathPub_->get_subscription_count())
			{
				globalPathPub_->publish(path);
			}
			if(globalPathNodesPub_ != nullptr && globalPathNodesPub_->get_subscription_count())
			{
				globalPathNodesPub_->publish(pathNodes);
			}
		}
	}
}

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
bool CoreWrapper::octomapBinaryCallback(
		octomap_msgs::GetOctomap::Request  &req,
		octomap_msgs::GetOctomap::Response &res)
{
	RCLCPP_INFO(this->get_logger(), "Sending binary map data on service request");
	res->map.header.frame_id = mapFrameId_;
	res->map.header.stamp = now();

	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	if(maxMappingNodes_ > 0 && poses.size()>1)
	{
		std::map<int, Transform> nearestPoses;
		std::vector<int> nodes = graph::findNearestNodes(poses, poses.rbegin()->second, maxMappingNodes_);
		for(std::vector<int>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
		{
			std::map<int, Transform>::iterator pter = poses.find(*iter);
			if(pter != poses.end())
			{
				nearestPoses.insert(*pter);
			}
		}
		poses = nearestPoses;
	}

	poses = mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), false, true);

	const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
	bool success = octomap->octree()->size() && octomap_msgs::binaryMapToMsg(*octomap->octree(), res->map);
	return success;
}

bool CoreWrapper::octomapFullCallback(
		octomap_msgs::GetOctomap::Request  &req,
		octomap_msgs::GetOctomap::Response &res)
{
	RCLCPP_INFO(this->get_logger(), "Sending full map data on service request");
	res->map.header.frame_id = mapFrameId_;
	res->map.header.stamp = now();

	std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
	if(maxMappingNodes_ > 0 && poses.size()>1)
	{
		std::map<int, Transform> nearestPoses;
		std::vector<int> nodes = graph::findNearestNodes(poses, poses.rbegin()->second, maxMappingNodes_);
		for(std::vector<int>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
		{
			std::map<int, Transform>::iterator pter = poses.find(*iter);
			if(pter != poses.end())
			{
				nearestPoses.insert(*pter);
			}
		}
		poses = nearestPoses;
	}

	poses = mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), false, true);

	const rtabmap::OctoMap * octomap = mapsManager_.getOctomap();
	bool success = octomap->octree()->size() && octomap_msgs::fullMapToMsg(*octomap->octree(), res->map);
	return success;
}
#endif
#endif

athena_utils::CallbackReturn CoreWrapper::on_configure(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "CoreWrapper Configuring");
	// TODO. if need reconfigure online, wait to change.
	if (configured_) return athena_utils::CallbackReturn::SUCCESS;
	char * rosHomePath = getenv("ROS_HOME");
	std::string workingDir = rosHomePath?rosHomePath:UDirectory::homeDir()+"/.ros";
	databasePath_ = workingDir+"/"+rtabmap::Parameters::getDefaultDatabaseName();
	globalPose_.header.stamp = rclcpp::Time(0);

	mapsManager_.init(*node_, this->get_name(), true);

	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	//	this->get_node_base_interface(),
	//	this->get_node_timers_interface());
	//tfBuffer_->setCreateTimerInterface(timer_interface);
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
	tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	bool publishTf = true;
	double tfDelay = 0.05; // 20 Hz
	double tfTolerance = 0.1; // 100 ms

	configPath_ = this->declare_parameter("config_path", configPath_);
	databasePath_ = this->declare_parameter("database_path", databasePath_);

	frameId_ = this->declare_parameter("frame_id", frameId_);
	this->get_parameter_or("odom_frame_id", odomFrameId_, odomFrameId_); // already declared in CommonDataSubscriber
	mapFrameId_ = this->declare_parameter("map_frame_id", mapFrameId_);
	groundTruthFrameId_ = this->declare_parameter("ground_truth_frame_id", groundTruthFrameId_);
	groundTruthBaseFrameId_ = this->declare_parameter("ground_truth_base_frame_id", frameId_);

	publishTf = this->declare_parameter("publish_tf", publishTf);
	tfDelay = this->declare_parameter("tf_delay", tfDelay);
	tfTolerance = this->declare_parameter("tf_tolerance", tfTolerance);

	odomDefaultAngVariance_ = this->declare_parameter("odom_tf_angular_variance", odomDefaultAngVariance_);
	odomDefaultLinVariance_ = this->declare_parameter("odom_tf_linear_variance", odomDefaultLinVariance_);

	landmarkDefaultAngVariance_ = this->declare_parameter("landmark_angular_variance", landmarkDefaultAngVariance_);
	landmarkDefaultLinVariance_ = this->declare_parameter("landmark_linear_variance", landmarkDefaultLinVariance_);

	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
#ifdef WITH_MOVE_BASE_MSGS
	useActionForGoal_ = this->declare_parameter("use_action_for_goal", useActionForGoal_);
#endif
	useSavedMap_ = this->declare_parameter("use_saved_map", useSavedMap_);
	genScan_ = this->declare_parameter("gen_scan", genScan_);
	genScanMaxDepth_ = this->declare_parameter("gen_scan_max_depth", genScanMaxDepth_);
	genScanMinDepth_ = this->declare_parameter("gen_scan_min_depth", genScanMinDepth_);
	scanCloudMaxPoints_ = this->declare_parameter("scan_cloud_max_points", scanCloudMaxPoints_);

	stereoToDepth_ = this->declare_parameter("stereo_to_depth", stereoToDepth_);
	odomSensorSync_ = this->declare_parameter("odom_sensor_sync", odomSensorSync_);

	RCLCPP_INFO(this->get_logger(), "rtabmap: frame_id      = %s", frameId_.c_str());
	if(!odomFrameId_.empty())
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: odom_frame_id = %s", odomFrameId_.c_str());
	}
	if(!groundTruthFrameId_.empty())
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: ground_truth_frame_id = %s -> ground_truth_base_frame_id = %s",
				groundTruthFrameId_.c_str(),
				groundTruthBaseFrameId_.c_str());
	}
	RCLCPP_INFO(this->get_logger(), "rtabmap: map_frame_id  = %s", mapFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "rtabmap: use_action_for_goal  = %s", useActionForGoal_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "rtabmap: tf_delay      = %f", tfDelay);
	RCLCPP_INFO(this->get_logger(), "rtabmap: tf_tolerance  = %f", tfTolerance);
	RCLCPP_INFO(this->get_logger(), "rtabmap: odom_sensor_sync   = %s", odomSensorSync_?"true":"false");
	if(this->isSubscribedToStereo())
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: stereo_to_depth = %s", stereoToDepth_?"true":"false");
	}

	relocalizationStatePub_ = this->create_publisher<std_msgs::msg::Bool>("relocalizationState", 1);
	infoPub_ = this->create_publisher<rtabmap_ros::msg::Info>("info", 1);
	mapDataPub_ = this->create_publisher<rtabmap_ros::msg::MapData>("mapData", 1);
	mapGraphPub_ = this->create_publisher<rtabmap_ros::msg::MapGraph>("mapGraph", 1);
	landmarksPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("landmarks", 1);
	labelsPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("labels", 1);
	mapPathPub_ = this->create_publisher<nav_msgs::msg::Path>("mapPath", 1);
	localGridObstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_grid_obstacle", 1);
	localGridEmpty_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_grid_empty", 1);
	localGridGround_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_grid_ground", 1);
	localizationPosePub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("localization_pose", 1);
	errorStatusPub_ = this->create_publisher<std_msgs::msg::UInt8>("slam_error_status", 1);
	initialPoseSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, std::bind(&CoreWrapper::initialPoseCallback, this, std::placeholders::_1));
	trackingPoseSubThread_ = new std::thread(subscribeTrackingPose, std::ref(trackingPoseMsgs_));

	// planning topics
	goalSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 1, std::bind(&CoreWrapper::goalCallback, this, std::placeholders::_1));
	goalNodeSub_ = this->create_subscription<rtabmap_ros::msg::Goal>("goal_node", 1, std::bind(&CoreWrapper::goalNodeCallback, this, std::placeholders::_1));
	nextMetricGoalPub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_out", 1);
	goalReachedPub_ = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 1);
	globalPathPub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
	localPathPub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 1);
	globalPathNodesPub_ = this->create_publisher<rtabmap_ros::msg::Path>("global_path_nodes", 1);
	localPathNodesPub_ = this->create_publisher<rtabmap_ros::msg::Path>("local_path_nodes", 1);

	configPath_ = uReplaceChar(configPath_, '~', UDirectory::homeDir());
	databasePath_ = uReplaceChar(databasePath_, '~', UDirectory::homeDir());
	if(configPath_.size() && configPath_.at(0) != '/')
	{
		configPath_ = UDirectory::currentDir(true) + configPath_;
	}
	if(databasePath_.size() && databasePath_.at(0) != '/')
	{
		databasePath_ = UDirectory::currentDir(true) + databasePath_;
	}

	ParametersMap allParameters = Parameters::getDefaultParameters();
	// remove Odom parameters
	for(ParametersMap::iterator iter=allParameters.begin(); iter!=allParameters.end();)
	{
		if(iter->first.find("Odom") == 0)
		{
			allParameters.erase(iter++);
		}
		else
		{
			++iter;
		}
	}
	uInsert(allParameters, ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "true")); // default true in ROS
	uInsert(allParameters, ParametersPair(Parameters::kRtabmapWorkingDirectory(), workingDir)); // change default to ~/.ros

	// load parameters
	loadParameters(configPath_, parameters_);
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end();)
	{
		if(iter->first.find("Odom") == 0)
		{
			parameters_.erase(iter++);
		}
		else
		{
			++iter;
		}
	}

	// declare parameters
	this->declare_parameter("is_rtabmap_paused", paused_);
	for(ParametersMap::iterator iter=allParameters.begin(); iter!=allParameters.end(); ++iter)
	{
		std::string vStr = this->declare_parameter(iter->first, iter->second);
		if(vStr.compare(iter->second)!=0)
		{
			RCLCPP_INFO(this->get_logger(), "Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());

			if(iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				vStr = uReplaceChar(vStr, '~', UDirectory::homeDir());
			}
			else if(iter->first.compare(Parameters::kKpDictionaryPath()) == 0)
			{
				vStr = uReplaceChar(vStr, '~', UDirectory::homeDir());
			}
			uInsert(parameters_, ParametersPair(iter->first, vStr));
		}
	}

	//parse input arguments
	std::vector<std::string> argList = get_node_options().arguments();
	char ** argv = new char*[argList.size()];
	bool deleteDbOnStart = false;
	for(unsigned int i=0; i<argList.size(); ++i)
	{
		argv[i] = &argList[i].at(0);
		if(strcmp(argv[i], "--delete_db_on_start") == 0 || strcmp(argv[i], "-d") == 0)
		{
			deleteDbOnStart = true;
		}
	}
	rtabmap::ParametersMap parameters = rtabmap::Parameters::parseArguments(argList.size(), argv);
	delete[] argv;
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		uInsert(parameters_, ParametersPair(iter->first, iter->second));
		RCLCPP_INFO(this->get_logger(), "Update RTAB-Map parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
	}

	// Backward compatibility
	for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=Parameters::getRemovedParameters().begin();
		iter!=Parameters::getRemovedParameters().end();
		++iter)
	{
		std::string paramValue;
		rclcpp::Parameter parameter;
		if(get_parameter(iter->first, parameter))
		{
			paramValue = parameter.as_string();
		}
		if(!paramValue.empty())
		{
			if(iter->second.first)
			{
				// can be migrated
				uInsert(parameters_, ParametersPair(iter->second.second, paramValue));
				RCLCPP_WARN(this->get_logger(), "Rtabmap: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
						iter->first.c_str(), iter->second.second.c_str(), paramValue.c_str());
			}
			else
			{
				if(iter->second.second.empty())
				{
					RCLCPP_ERROR(this->get_logger(), "Rtabmap: Parameter \"%s\" doesn't exist anymore!",
							iter->first.c_str());
				}
				else
				{
					RCLCPP_ERROR(this->get_logger(), "Rtabmap: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
							iter->first.c_str(), iter->second.second.c_str());
				}
			}
		}
	}

	// Backward compatibility (MapsManager)
	mapsManager_.backwardCompatibilityParameters(*node_, parameters_);

	bool subscribeScan2d = this->isSubscribedToScan2d();
	bool subscribeScan3d = this->isSubscribedToScan3d();
	this->get_parameter_or("subscribe_scan_cloud", subscribeScan3d, subscribeScan3d);
	if((subscribeScan2d || subscribeScan3d) && parameters_.find(Parameters::kGridFromDepth()) == parameters_.end())
	{
		RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to false (default true) as \"subscribe_scan\" or \"subscribe_scan_cloud\" is "
				"true. The occupancy grid map will be constructed from "
				"laser scans. To get occupancy grid map from cloud projection, set \"%s\" "
				"to true. To suppress this warning, "
				"add <param name=\"%s\" type=\"string\" value=\"false\"/>",
				Parameters::kGridFromDepth().c_str(),
				Parameters::kGridFromDepth().c_str(),
				Parameters::kGridFromDepth().c_str());
		parameters_.insert(ParametersPair(Parameters::kGridFromDepth(), "false"));
	}
	if((subscribeScan2d || subscribeScan3d) && parameters_.find(Parameters::kGridRangeMax()) == parameters_.end())
	{
		RCLCPP_INFO(this->get_logger(), "Setting \"%s\" parameter to 0 (default %f) as \"subscribe_scan\" or \"subscribe_scan_cloud\" is true.",
				Parameters::kGridRangeMax().c_str(),
				Parameters::defaultGridRangeMax());
		parameters_.insert(ParametersPair(Parameters::kGridRangeMax(), "0"));
	}
	if(subscribeScan3d && parameters_.find(Parameters::kIcpPointToPlaneRadius()) == parameters_.end())
	{
		RCLCPP_INFO(this->get_logger(), "Setting \"%s\" parameter to 0 (default %f) as \"subscribe_scan_cloud\" is true.",
				Parameters::kIcpPointToPlaneRadius().c_str(),
				Parameters::defaultIcpPointToPlaneRadius());
		parameters_.insert(ParametersPair(Parameters::kIcpPointToPlaneRadius(), "0"));
	}
	int regStrategy = Parameters::defaultRegStrategy();
	Parameters::parse(parameters_, Parameters::kRegStrategy(), regStrategy);
	if(parameters_.find(Parameters::kRGBDProximityPathMaxNeighbors()) == parameters_.end() &&
		(regStrategy == Registration::kTypeIcp || regStrategy == Registration::kTypeVisIcp))
	{
		if(subscribeScan2d)
		{
			RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 10 (default 0) as \"subscribe_scan\" is "
					"true and \"%s\" uses ICP. Proximity detection by space will be also done by merging close "
					"scans. To disable, set \"%s\" to 0. To suppress this warning, "
					"add <param name=\"%s\" type=\"string\" value=\"10\"/>",
					Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
					Parameters::kRegStrategy().c_str(),
					Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
					Parameters::kRGBDProximityPathMaxNeighbors().c_str());
			parameters_.insert(ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "10"));
		}
		else if(subscribeScan3d)
		{
			RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 1 (default 0) as \"subscribe_scan_cloud\" is "
					"true and \"%s\" uses ICP. To disable, set \"%s\" to 0. To suppress this warning, "
					"add <param name=\"%s\" type=\"string\" value=\"1\"/>",
					Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
					Parameters::kRegStrategy().c_str(),
					Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
					Parameters::kRGBDProximityPathMaxNeighbors().c_str());
			parameters_.insert(ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "1"));
		}
	}

	int estimationType = Parameters::defaultVisEstimationType();
	Parameters::parse(parameters_, Parameters::kVisEstimationType(), estimationType);
	int cameras = this->rgbdCameras();
	bool subscribeRGBD = this->isSubscribedToRGBD();
	if(subscribeRGBD && cameras> 1 && estimationType>0)
	{
		RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 0 (%d is not supported "
				"for multi-cameras) as \"subscribe_rgbd\" is "
				"true and \"rgbd_cameras\">1. Set \"%s\" to 0 to suppress this warning.",
				Parameters::kVisEstimationType().c_str(),
				estimationType,
				Parameters::kVisEstimationType().c_str());
		uInsert(parameters_, ParametersPair(Parameters::kVisEstimationType(), "0"));
	}

	// modify default parameters with those in the database
	if(!deleteDbOnStart)
	{
		ParametersMap dbParameters;
		rtabmap::DBDriver * driver = rtabmap::DBDriver::create();
		if(driver->openConnection(databasePath_))
		{
			try
			{
				dbParameters = driver->getLastParameters(); // parameter migration is already done
			}
			catch(const std::exception & ex)
			{
				RCLCPP_ERROR(this->get_logger(), "Read parameter from map db error...\n %s", ex.what());
				slamErrorStatus_ = 1; // database error.
			}
		}
		delete driver;
		for(ParametersMap::iterator iter=dbParameters.begin(); iter!=dbParameters.end(); ++iter)
		{
			if(iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
			{
				// ignore working directory
				continue;
			}
			if(parameters_.find(iter->first) == parameters_.end() &&
				allParameters.find(iter->first) != allParameters.end() &&
				allParameters.find(iter->first)->second.compare(iter->second) !=0)
			{
				RCLCPP_INFO(this->get_logger(), "Update RTAB-Map parameter \"%s\"=\"%s\" from database", iter->first.c_str(), iter->second.c_str());
				parameters_.insert(*iter);
			}
		}
	}
	ParametersMap modifiedParameters = parameters_;
	// Add all other parameters (not copied if already exists)
	parameters_.insert(allParameters.begin(), allParameters.end());

	// Update declared parameters
	std::vector<rclcpp::Parameter> rosParameters;
	for(ParametersMap::iterator iter=allParameters.begin(); iter!=allParameters.end(); ++iter)
	{
		rosParameters.push_back(rclcpp::Parameter(iter->first, iter->second));
	}
	this->set_parameters(rosParameters);

	if(parameters_.find(Parameters::kRtabmapDetectionRate()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kRtabmapDetectionRate(), rate_);
		RCLCPP_INFO(this->get_logger(), "RTAB-Map detection rate = %f Hz", rate_);
	}
	if(parameters_.find(Parameters::kRtabmapCreateIntermediateNodes()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kRtabmapCreateIntermediateNodes(), createIntermediateNodes_);
		if(createIntermediateNodes_)
		{
			RCLCPP_INFO(this->get_logger(), "Create intermediate nodes");
			if(rate_ == 0.0f)
			{
				bool interOdomInfo = false;
				if(get_parameter("subscribe_inter_odom_info", interOdomInfo))
				{
					RCLCPP_INFO(this->get_logger(), "Subscribe to inter odom + info messages");
					interOdomSync_ = new message_filters::Synchronizer<MyExactInterOdomSyncPolicy>(MyExactInterOdomSyncPolicy(100), interOdomSyncSub_, interOdomInfoSyncSub_);
					interOdomSync_->registerCallback(std::bind(&CoreWrapper::interOdomInfoCallback, this, std::placeholders::_1, std::placeholders::_2));
					interOdomSyncSub_.subscribe(node_, "inter_odom");
					interOdomInfoSyncSub_.subscribe(node_, "inter_odom_info");
				}
				else
				{
					RCLCPP_INFO(this->get_logger(), "Subscribe to inter odom messages");
					interOdomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("inter_odom", 100, std::bind(&CoreWrapper::interOdomCallback, this, std::placeholders::_1));
				}

			}
		}
	}
	if(parameters_.find(Parameters::kGridGlobalMaxNodes()) != parameters_.end())
	{
		Parameters::parse(parameters_, Parameters::kGridGlobalMaxNodes(), maxMappingNodes_);
		if(maxMappingNodes_>0)
		{
			RCLCPP_INFO(this->get_logger(), "Max mapping nodes = %d", maxMappingNodes_);
		}
	}

	if(deleteDbOnStart)
	{
		if(UFile::erase(databasePath_) == 0)
		{
			RCLCPP_INFO(this->get_logger(), "rtabmap: Deleted database \"%s\" (--delete_db_on_start or -d are set).", databasePath_.c_str());
		}
	}

	if(databasePath_.size())
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: Using database from \"%s\" (%ld MB).", databasePath_.c_str(), UFile::length(databasePath_)/(1024*1024));
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: database_path parameter not set, the map will not be saved.");
	}

	mapsManager_.setParameters(parameters_);

	// Init RTAB-Map
	try
	{
		rtabmap_.init(parameters_, databasePath_);
	}
	catch(std::exception& ex)
	{
		RCLCPP_ERROR(this->get_logger(), "map init error...\n %s", ex.what());
		slamErrorStatus_ = 1; // database error error.
	}
	catch(...)
	{
		RCLCPP_ERROR(this->get_logger(), "map init error...\n %s");
		slamErrorStatus_ = 1; // database error error.
	}
	RCLCPP_INFO(this->get_logger(), "lastSignatureIdInDB: %d", rtabmap_.getLastLocationId());
	lastSignatureIdInDB_ = rtabmap_.getLastLocationId();

	if(rtabmap_.getMemory() && useSavedMap_)
	{
		float xMin, yMin, gridCellSize;
		cv::Mat map = rtabmap_.getMemory()->load2DMap(xMin, yMin, gridCellSize);
		if(!map.empty())
		{
			RCLCPP_INFO(this->get_logger(), "rtabmap: 2D occupancy grid map loaded (%dx%d).", map.cols, map.rows);
			mapsManager_.set2DMap(map, xMin, yMin, gridCellSize, rtabmap_.getLocalOptimizedPoses(), rtabmap_.getMemory());
		}
	}

	if(databasePath_.size() && rtabmap_.getMemory())
	{
		RCLCPP_INFO(this->get_logger(), "rtabmap: Database version = \"%s\".", rtabmap_.getMemory()->getDatabaseVersion().c_str());
	}

	// setup services
	processImageSrv_ = this->create_service<std_srvs::srv::SetBool>("process_image_in_localization", std::bind(&CoreWrapper::processImageCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	resetSrv_ = this->create_service<std_srvs::srv::Empty>("reset", std::bind(&CoreWrapper::resetRtabmapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	pauseSrv_ = this->create_service<std_srvs::srv::Empty>("pause", std::bind(&CoreWrapper::pauseRtabmapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	resumeSrv_ = this->create_service<std_srvs::srv::Empty>("resume", std::bind(&CoreWrapper::resumeRtabmapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	triggerNewMapSrv_ = this->create_service<std_srvs::srv::Empty>("trigger_new_map", std::bind(&CoreWrapper::triggerNewMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	backupDatabase_ = this->create_service<std_srvs::srv::Empty>("backup", std::bind(&CoreWrapper::backupDatabaseCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setModeLocalizationSrv_ = this->create_service<std_srvs::srv::Empty>("set_mode_localization", std::bind(&CoreWrapper::setModeLocalizationCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setModeMappingSrv_ = this->create_service<std_srvs::srv::Empty>("set_mode_mapping", std::bind(&CoreWrapper::setModeMappingCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	getMapDataSrv_ = this->create_service<rtabmap_ros::srv::GetMap>("get_map_data", std::bind(&CoreWrapper::getMapDataCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	getMapSrv_ = this->create_service<nav_msgs::srv::GetMap>("get_map", std::bind(&CoreWrapper::getMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	getProbMapSrv_ = this->create_service<nav_msgs::srv::GetMap>("get_prob_map", std::bind(&CoreWrapper::getProbMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	publishMapDataSrv_ = this->create_service<rtabmap_ros::srv::PublishMap>("publish_map", std::bind(&CoreWrapper::publishMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	getPlanSrv_ = this->create_service<nav_msgs::srv::GetPlan>("get_plan", std::bind(&CoreWrapper::getPlanCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	getPlanNodesSrv_ = this->create_service<rtabmap_ros::srv::GetPlan>("get_plan_nodes", std::bind(&CoreWrapper::getPlanNodesCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setGoalSrv_ = this->create_service<rtabmap_ros::srv::SetGoal>("set_goal", std::bind(&CoreWrapper::setGoalCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

	cancelGoalSrv_ = this->create_service<std_srvs::srv::Empty>("cancel_goal", std::bind(&CoreWrapper::cancelGoalCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setLabelSrv_ = this->create_service<rtabmap_ros::srv::SetLabel>("set_label", std::bind(&CoreWrapper::setLabelCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	listLabelsSrv_ = this->create_service<rtabmap_ros::srv::ListLabels>("list_labels", std::bind(&CoreWrapper::listLabelsCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	octomapBinarySrv_ = this->create_service<std_srvs::srv::Empty>("octomap_binary", std::bind(&CoreWrapper::octomapBinaryCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	octomapFullSrv_ = this->create_service<std_srvs::srv::Empty>("octomap_full", std::bind(&CoreWrapper::octomapFullCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
#endif
#endif
	//private services
	setLogDebugSrv_ = this->create_service<std_srvs::srv::Empty>("log_debug", std::bind(&CoreWrapper::setLogDebug, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setLogInfoSrv_ = this->create_service<std_srvs::srv::Empty>("log_info", std::bind(&CoreWrapper::setLogInfo, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setLogWarnSrv_ = this->create_service<std_srvs::srv::Empty>("log_warning", std::bind(&CoreWrapper::setLogWarn, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	setLogErrorSrv_ = this->create_service<std_srvs::srv::Empty>("log_error", std::bind(&CoreWrapper::setLogError, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

	int optimizeIterations = 0;
	Parameters::parse(parameters_, Parameters::kOptimizerIterations(), optimizeIterations);
	if(publishTf && optimizeIterations != 0)
	{
		tfThreadRunning_ = true;
		transformThread_ = new std::thread([=](){
			if(tfDelay == 0)
				return;
			rclcpp::Rate r(1.0 / tfDelay);
			while(tfThreadRunning_)
			{
				if(!odomFrameId_.empty())
				{
					mapToOdomMutex_.lock();
					rclcpp::Time tfExpiration = now() + rclcpp::Duration(tfTolerance*10e9);
					geometry_msgs::msg::TransformStamped msg;
					msg.child_frame_id = odomFrameId_;
					msg.header.frame_id = mapFrameId_;
					msg.header.stamp = tfExpiration;
					rtabmap_ros::transformToGeometryMsg(mapToOdom_, msg.transform);
					tfBroadcaster_->sendTransform(msg);
					mapToOdomMutex_.unlock();
				}
				r.sleep();
			}
		});
	}
	else if(publishTf)
	{
		RCLCPP_WARN(this->get_logger(), "Graph optimization is disabled (%s=0), the tf between frame \"%s\" and odometry frame will not be published. You can safely ignore this warning if you are using map_optimizer node.",
				Parameters::kOptimizerIterations().c_str(), mapFrameId_.c_str());
	}

	RCLCPP_INFO(this->get_logger(), "Setup callbacks");
	//setupCallbacks(*node_); // do it at the end
	if(!this->isDataSubscribed())
	{
		bool isRGBD = uStr2Bool(parameters_.at(Parameters::kRGBDEnabled()).c_str());
		if(isRGBD)
		{
			RCLCPP_WARN(this->get_logger(), "ROS param subscribe_depth, subscribe_stereo and subscribe_rgbd are false, but RTAB-Map "
					  "parameter \"%s\" is true! Please set subscribe_depth, subscribe_stereo or subscribe_rgbd "
					  "to true to use rtabmap node for RGB-D SLAM, set \"%s\" to false for loop closure "
					  "detection on images-only or set subscribe_rgb to true to localize a single RGB camera against pre-built 3D map.",
					  Parameters::kRGBDEnabled().c_str(),
					  Parameters::kRGBDEnabled().c_str());
		}
		auto node = rclcpp::Node::make_shared("rtabmap");
		image_transport::TransportHints hints(node_.get());
		defaultSub_ = image_transport::create_subscription(node.get(), "image", std::bind(&CoreWrapper::defaultCallback, this, std::placeholders::_1), hints.getTransport());


		RCLCPP_INFO(this->get_logger(), "\n%s subscribed to:\n   %s", get_name(), defaultSub_.getTopic().c_str());
	}
	else if(!this->isSubscribedToDepth() &&
			!this->isSubscribedToStereo() &&
			!this->isSubscribedToRGBD() &&
			!this->isSubscribedToRGB() &&
			(this->isSubscribedToScan2d() || this->isSubscribedToScan3d() || this->isSubscribedToOdom()))
	{
		RCLCPP_WARN(this->get_logger(), "There is no image subscription, bag-of-words loop closure detection will be disabled...");
		int kpMaxFeatures = Parameters::defaultKpMaxFeatures();
		int registrationStrategy = Parameters::defaultRegStrategy();
		Parameters::parse(parameters_, Parameters::kKpMaxFeatures(), kpMaxFeatures);
		Parameters::parse(parameters_, Parameters::kRegStrategy(), registrationStrategy);
		bool updateParams = false;
		if(kpMaxFeatures!= -1)
		{
			uInsert(parameters_, ParametersPair(Parameters::kKpMaxFeatures(), "-1"));
			RCLCPP_WARN(this->get_logger(), "Setting %s=-1 (bag-of-words disabled)", Parameters::kKpMaxFeatures().c_str());
			updateParams = true;
		}
		if((this->isSubscribedToScan2d() || this->isSubscribedToScan3d()) && registrationStrategy != 1)
		{
			uInsert(parameters_, ParametersPair(Parameters::kRegStrategy(), "1"));
			RCLCPP_WARN(this->get_logger(), "Setting %s=1 (ICP)", Parameters::kRegStrategy().c_str());
			updateParams = true;

			if(modifiedParameters.find(Parameters::kRGBDProximityPathMaxNeighbors()) == modifiedParameters.end())
			{
				if(this->isSubscribedToScan2d())
				{
					RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 10 (default 0) as \"subscribe_scan\" is "
							"true and \"%s\" uses ICP. Proximity detection by space will be also done by merging close "
							"scans. To disable, set \"%s\" to 0. To suppress this warning, "
							"add <param name=\"%s\" type=\"string\" value=\"10\"/>",
							Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
							Parameters::kRegStrategy().c_str(),
							Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
							Parameters::kRGBDProximityPathMaxNeighbors().c_str());
					uInsert(parameters_, ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "10"));
				}
				else if(this->isSubscribedToScan3d())
				{
					RCLCPP_WARN(this->get_logger(), "Setting \"%s\" parameter to 1 (default 0) as \"subscribe_scan_cloud\" is "
							"true and \"%s\" uses ICP. To disable, set \"%s\" to 0. To suppress this warning, "
							"add <param name=\"%s\" type=\"string\" value=\"1\"/>",
							Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
							Parameters::kRegStrategy().c_str(),
							Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
							Parameters::kRGBDProximityPathMaxNeighbors().c_str());
					uInsert(parameters_, ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "1"));
				}
			}
		}
		if(updateParams)
		{
			rtabmap_.parseParameters(parameters_);
		}
	}

	userDataAsyncSub_ = this->create_subscription<rtabmap_ros::msg::UserData>("user_data_async", 1, std::bind(&CoreWrapper::userDataAsyncCallback, this, std::placeholders::_1));
	globalPoseAsyncSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("global_pose", 1, std::bind(&CoreWrapper::globalPoseAsyncCallback, this, std::placeholders::_1));
	gpsFixAsyncSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", 1, std::bind(&CoreWrapper::gpsFixAsyncCallback, this, std::placeholders::_1));
#ifdef WITH_APRILTAG_ROS
	tagDetectionsSub_ = this->create_subscription<apriltag_ros::msg::AprilTagDetectionArray>("tag_detections", 1, std::bind(&CoreWrapper::tagDetectionsAsyncCallback, this, std::placeholders::_1));
#endif
	imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 100, std::bind(&CoreWrapper::imuAsyncCallback, this, std::placeholders::_1));

	parametersClient_ = std::make_shared<rclcpp::SyncParametersClient>(node_); // this
	auto on_parameter_event_callback =
			[this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
			{
				for(size_t i=0; i<event->changed_parameters.size(); ++i)
				{
					std::string key = event->changed_parameters[i].name;
					if(parameters_.find(key) != parameters_.end())
					{
						std::string vStr = event->changed_parameters[i].value.string_value;
						RCLCPP_INFO(this->get_logger(), "Setting RTAB-Map parameter \"%s\"=\"%s\"", key.c_str(), vStr.c_str());
						parameters_.at(key) = vStr;
					}
				}
				RCLCPP_INFO(this->get_logger(), "rtabmap: Updating parameters");
				if(parameters_.find(Parameters::kRtabmapDetectionRate()) != parameters_.end())
				{
					rate_ = uStr2Float(parameters_.at(Parameters::kRtabmapDetectionRate()));
					RCLCPP_INFO(this->get_logger(), "RTAB-Map rate detection = %f Hz", rate_);
				}
				rtabmap_.parseParameters(parameters_);
				mapsManager_.setParameters(parameters_);
			};

	// record relocation parameters
	std::vector<std::string> keys;
	keys.push_back(rtabmap::Parameters::kRtabmapLoopThr());
	keys.push_back(rtabmap::Parameters::kVisMinInliers());
	keys.push_back(rtabmap::Parameters::kVisMinInliersDistribution());
	keys.push_back(rtabmap::Parameters::kIcpMaxTranslation());
	for(auto key : keys)
	{
		if(parameters_.find(key) != parameters_.end())
		{
			partOfLocalizationParameters_.insert(std::make_pair(key, parameters_.at(key)));
			RCLCPP_INFO(this->get_logger(), "Setting RTAB-Map parameter \"%s\"=\"%s\"",	key.c_str(), parameters_.at(key).c_str());
		}
	}

	// Setup callback for changes to parameters.
	parameterEventSub_ = parametersClient_->on_parameter_event(on_parameter_event_callback);

	if(slamErrorStatus_ == 1)
		resetRtabmap();
	configured_ = true;
	RCLCPP_INFO(this->get_logger(), "%s Configured.", get_name());
	return athena_utils::CallbackReturn::SUCCESS;
}

athena_utils::CallbackReturn CoreWrapper::on_activate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "CoreWrapper Activating");
	setupCallbacks(*node_, errorStatusPub_);
	if(!paused_)
	{
		RCLCPP_WARN(this->get_logger(), "rtabmap: Already running!");
	}
	else
	{
		paused_ = false;
		RCLCPP_INFO(this->get_logger(), "rtabmap: resumed!");
		set_parameter(rclcpp::Parameter("is_rtabmap_paused", false));
	}

	mapsManager_.on_activate();
	
	errorStatusPub_->on_activate();
	relocalizationStatePub_->on_activate();
	infoPub_->on_activate();
	mapDataPub_->on_activate();
	mapGraphPub_->on_activate();
	landmarksPub_->on_activate();
	labelsPub_->on_activate();
	mapPathPub_->on_activate();
	localGridObstacle_->on_activate();
	localGridEmpty_->on_activate();
	localGridGround_->on_activate();
	localizationPosePub_->on_activate();

	// planning topics
	nextMetricGoalPub_->on_activate();
	goalReachedPub_->on_activate();
	globalPathPub_->on_activate();
	localPathPub_->on_activate();
	globalPathNodesPub_->on_activate();
	localPathNodesPub_->on_activate();

	covariance_ = cv::Mat();
	lastPose_.setIdentity();
	currentMetricGoal_.setNull();
	lastPublishedMetricGoal_.setNull();
	goalFrameId_.clear();
	latestNodeWasReached_ = false;
	userDataMutex_.lock();
	userData_ = cv::Mat();
	userDataMutex_.unlock();
	globalPose_.header.stamp = rclcpp::Time(0);
	gps_ = rtabmap::GPS();
	tags_.clear();
	try
	{
		rtabmap_.init(parameters_, databasePath_);
	}
	catch(std::exception& ex)
	{
		RCLCPP_ERROR(this->get_logger(), "map init error...\n %s", ex.what());
		slamErrorStatus_ = 1; // database error error.
	}
	catch(...)
	{
		RCLCPP_ERROR(this->get_logger(), "map init error...\n %s");
		slamErrorStatus_ = 1; // database error error.
	}
	lastSignatureIdInDB_ = rtabmap_.getLastLocationId();
	RCLCPP_INFO(this->get_logger(), "lastSignatureIdInDB_: %d", lastSignatureIdInDB_);

	return athena_utils::CallbackReturn::SUCCESS;
}

athena_utils::CallbackReturn CoreWrapper::on_deactivate(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "CoreWrapper Deactivating");

	if(paused_)
	{
		RCLCPP_WARN(this->get_logger(), "rtabmap: Already paused!");
	}
	else
	{
		paused_ = true;
		RCLCPP_INFO(this->get_logger(), "rtabmap: paused!");
		set_parameter(rclcpp::Parameter("is_rtabmap_paused", true));
	}
	if(pubLocalizationOnce_)
	{
		for(auto iter : partOfLocalizationParameters_)
			set_parameter(rclcpp::Parameter(iter.first, iter.second));
		pubLocalizationOnce_ = false;
	}

	// save map data
	this->saveParameters(configPath_);
	if(rtabmap_.getMemory())
	{
		// save the grid map
		float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
		cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
		if(!pixels.empty())
			rtabmap_.getMemory()->save2DMap(pixels, xMin, yMin, gridCellSize);
	}
	rtabmap_.close();
	RCLCPP_INFO(this->get_logger(), "rtabmap: Saving database/long-term memory...done! (located at %s, %ld MB)\n", databasePath_.c_str(), UFile::length(databasePath_)/(1024*1024));

	mapsManager_.on_deactivate();

	errorStatusPub_->on_deactivate();
	relocalizationStatePub_->on_deactivate();
	infoPub_->on_deactivate();
	mapDataPub_->on_deactivate();
	mapGraphPub_->on_deactivate();
	landmarksPub_->on_deactivate();
	labelsPub_->on_deactivate();
	mapPathPub_->on_deactivate();
	localGridObstacle_->on_deactivate();
	localGridEmpty_->on_deactivate();
	localGridGround_->on_deactivate();
	localizationPosePub_->on_deactivate();

	nextMetricGoalPub_->on_deactivate();
	goalReachedPub_->on_deactivate();
	globalPathPub_->on_deactivate();
	localPathPub_->on_deactivate();
	globalPathNodesPub_->on_deactivate();
	localPathNodesPub_->on_deactivate();

	return athena_utils::CallbackReturn::SUCCESS;
}

athena_utils::CallbackReturn CoreWrapper::on_cleanup(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "CoreWrapper Cleaning up");
	if(configured_) return athena_utils::CallbackReturn::SUCCESS;
	errorStatusPub_.reset();
	relocalizationStatePub_.reset();
	infoPub_.reset();
	mapDataPub_.reset();
	mapGraphPub_.reset();
	landmarksPub_.reset();
	labelsPub_.reset();
	mapPathPub_.reset();
	localGridObstacle_.reset();
	localGridEmpty_.reset();
	localGridGround_.reset();
	localizationPosePub_.reset();

	nextMetricGoalPub_.reset();
	goalReachedPub_.reset();
	globalPathPub_.reset();
	localPathPub_.reset();
	globalPathNodesPub_.reset();
	localPathNodesPub_.reset();

	RCLCPP_INFO(this->get_logger(), "%s Completed Cleaning up", get_name());
	return athena_utils::CallbackReturn::SUCCESS;
}

athena_utils::CallbackReturn CoreWrapper::on_shutdown(const rclcpp_lifecycle::State &)
{
	RCLCPP_INFO(this->get_logger(), "CoreWrapper Shutting down");

	return athena_utils::CallbackReturn::SUCCESS;
}



}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::CoreWrapper)
