/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <string>
#if defined(ROS_AVAILABLE) || defined(DOXYGEN)
#ifndef OV_MSCKF_PARSE_ROSHANDLER_H
#define OV_MSCKF_PARSE_ROSHANDLER_H


#include <rclcpp/rclcpp.hpp>
#include "core/VioManagerOptions.h"
#include <athena_utils/lifecycle_node.hpp>


namespace ov_msckf {



    /**
     * @brief This function will load paramters from the ros node handler / paramter server
     * This is the recommended way of loading parameters as compared to the command line version.
     * @param nh ROS node handler
     * @return A fully loaded VioManagerOptions object
     */
    VioManagerOptions parse_ros_nodehandler(athena_utils::LifecycleNode *node) {
        // Our vio manager options with defaults
        VioManagerOptions params;

        // ESTIMATOR ======================================================================
        // Main EKF parameters
        // declare parameters
        node->declare_parameter<bool>("use_fej", params.state_options.do_fej);
        node->declare_parameter<bool>("use_imuavg", params.state_options.imu_avg);
        node->declare_parameter<bool>("use_rk4int", params.state_options.use_rk4_integration);
        node->declare_parameter<bool>("calib_cam_extrinsics", params.state_options.do_calib_camera_pose);
        node->declare_parameter<bool>("calib_cam_intrinsics", params.state_options.do_calib_camera_intrinsics);
        node->declare_parameter<bool>("calib_cam_timeoffset", params.state_options.do_calib_camera_timeoffset);
        node->declare_parameter<int>("max_clones", params.state_options.max_clone_size);
        node->declare_parameter<int>("max_slam", params.state_options.max_slam_features);
        node->declare_parameter<int>("max_slam_in_update", params.state_options.max_slam_in_update);
        node->declare_parameter<int>("max_msckf_in_update", params.state_options.max_msckf_in_update);
        node->declare_parameter<int>("max_aruco", params.state_options.max_aruco_features);
        node->declare_parameter<int>("max_cameras", params.state_options.num_cameras);
        node->declare_parameter<double>("dt_slam_delay", params.dt_slam_delay);
        // get parameters
        node->get_parameter("use_fej", params.state_options.do_fej);
        node->get_parameter("use_imuavg", params.state_options.imu_avg);
        node->get_parameter("use_rk4int", params.state_options.use_rk4_integration);
        node->get_parameter("calib_cam_extrinsics", params.state_options.do_calib_camera_pose);
        node->get_parameter("calib_cam_intrinsics", params.state_options.do_calib_camera_intrinsics);
        node->get_parameter("calib_cam_timeoffset", params.state_options.do_calib_camera_timeoffset);
        node->get_parameter("max_clones", params.state_options.max_clone_size);
        node->get_parameter("max_slam", params.state_options.max_slam_features);
        node->get_parameter("max_slam_in_update", params.state_options.max_slam_in_update);
        node->get_parameter("max_msckf_in_update", params.state_options.max_msckf_in_update);
        node->get_parameter("max_aruco", params.state_options.max_aruco_features);
        node->get_parameter("max_cameras", params.state_options.num_cameras);
        node->get_parameter("dt_slam_delay", params.dt_slam_delay);

        // Enforce that we have enough cameras to run
        if(params.state_options.num_cameras < 1) {
            printf(RED "VioManager(): Specified number of cameras needs to be greater than zero\n" RESET);
            printf(RED "VioManager(): num cameras = %d\n" RESET, params.state_options.num_cameras);
            std::exit(EXIT_FAILURE);
        }

        // Read in stereo pair information
	std::vector<double> stereo_pairs;
        node->declare_parameter<std::vector<double>>("stereo_pairs", stereo_pairs);
	try{
       	    node->get_parameter("stereo_pairs", stereo_pairs);
	}
	catch(...){
            printf(RED "VioManager(): No stereo pair get\n" RESET);
	}
        if(stereo_pairs.size() % 2 != 0) {
            printf(RED "VioManager(): Specified number of stereo pair IDs needs to be even\n" RESET);
            printf(RED "VioManager(): Example: (0,1,2,3) -> stereo tracking between 01 and 23\n" RESET);
            std::exit(EXIT_FAILURE);
        }
        for(size_t i=0; i<stereo_pairs.size(); i++) {
            if(std::count(stereo_pairs.begin(),stereo_pairs.end(),stereo_pairs.at(i)) != 1) {
                printf(RED "VioManager(): You can do stereo tracking between unique ids\n" RESET);
                printf(RED "VioManager(): %ld showed up multiple times\n" RESET,stereo_pairs.at(i));
                std::exit(EXIT_FAILURE);
            }
            //if(stereo_pairs.at(i) >= params.state_options.num_cameras) {
            //    printf(RED "VioManager(): Stereo pair has an id larger then the max camera\n" RESET);
            //    printf(RED "VioManager(): %d is >= than %d\n" RESET,stereo_pairs.at(i),params.state_options.num_cameras);
            //    std::exit(EXIT_FAILURE);
            //}
        }
        std::vector<int> valid_stereo_pairs;
        for(size_t i=0; i<stereo_pairs.size(); i+=2) {
            if(stereo_pairs.at(i) >= params.state_options.num_cameras || stereo_pairs.at(i+1) >= params.state_options.num_cameras) {
                printf(RED "ignoring invalid stereo pair: %ld, %ld\n" RESET, stereo_pairs.at(i), stereo_pairs.at(i+1));
                continue;
            }
            params.stereo_pairs.emplace_back((int)stereo_pairs.at(i),(int)stereo_pairs.at(i+1));
            valid_stereo_pairs.push_back((int)stereo_pairs.at(i));
            valid_stereo_pairs.push_back((int)stereo_pairs.at(i+1));
        }

        // Calculate number of unique image camera image streams
        params.state_options.num_unique_cameras = (int)params.stereo_pairs.size();
        for(int i=0; i<params.state_options.num_cameras; i++) {
            if(std::find(valid_stereo_pairs.begin(),valid_stereo_pairs.end(),i)!=valid_stereo_pairs.end())
                continue;
            params.state_options.num_unique_cameras++;
        }

        // Read in what representation our feature is
        std::string feat_rep_msckf_str = "GLOBAL_3D";
        std::string feat_rep_slam_str = "GLOBAL_3D";
        std::string feat_rep_aruco_str = "GLOBAL_3D";
        node->declare_parameter<std::string>("feat_rep_msckf", feat_rep_msckf_str);
        node->declare_parameter<std::string>("feat_rep_slam", feat_rep_slam_str);
        node->declare_parameter<std::string>("feat_rep_aruco", feat_rep_aruco_str);
        node->get_parameter("feat_rep_msckf", feat_rep_msckf_str);
        node->get_parameter("feat_rep_slam", feat_rep_slam_str);
        node->get_parameter("feat_rep_aruco", feat_rep_aruco_str);

        // Set what representation we should be using
        std::transform(feat_rep_msckf_str.begin(), feat_rep_msckf_str.end(),feat_rep_msckf_str.begin(), ::toupper);
        std::transform(feat_rep_slam_str.begin(), feat_rep_slam_str.end(),feat_rep_slam_str.begin(), ::toupper);
        std::transform(feat_rep_aruco_str.begin(), feat_rep_aruco_str.end(),feat_rep_aruco_str.begin(), ::toupper);
        params.state_options.feat_rep_msckf = LandmarkRepresentation::from_string(feat_rep_msckf_str);
        params.state_options.feat_rep_slam = LandmarkRepresentation::from_string(feat_rep_slam_str);
        params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string(feat_rep_aruco_str);
        if(params.state_options.feat_rep_msckf == LandmarkRepresentation::Representation::UNKNOWN ||
        params.state_options.feat_rep_slam == LandmarkRepresentation::Representation::UNKNOWN ||
        params.state_options.feat_rep_aruco == LandmarkRepresentation::Representation::UNKNOWN) {
            printf(RED "VioManager(): invalid feature representation specified:\n" RESET);
            printf(RED "\t- GLOBAL_3D\n" RESET);
            printf(RED "\t- GLOBAL_FULL_INVERSE_DEPTH\n" RESET);
            printf(RED "\t- ANCHORED_3D\n" RESET);
            printf(RED "\t- ANCHORED_FULL_INVERSE_DEPTH\n" RESET);
            printf(RED "\t- ANCHORED_MSCKF_INVERSE_DEPTH\n" RESET);
            printf(RED "\t- ANCHORED_INVERSE_DEPTH_SINGLE\n" RESET);
            std::exit(EXIT_FAILURE);
        }

        // Filter initialization
        node->declare_parameter<double>("init_window_time", params.init_window_time);
        node->declare_parameter<double>("init_imu_thresh", params.init_imu_thresh);
        node->get_parameter("init_window_time", params.init_window_time);
        node->get_parameter("init_imu_thresh", params.init_imu_thresh);

        // Zero velocity update
        node->declare_parameter<bool>("try_zupt", params.try_zupt);
        node->declare_parameter<int>("zupt_chi2_multipler", params.zupt_options.chi2_multipler);
        node->declare_parameter<double>("zupt_max_velocity", params.zupt_max_velocity);
        node->declare_parameter<double>("zupt_noise_multiplier", params.zupt_noise_multiplier);
        node->get_parameter("try_zupt", params.try_zupt);
        node->get_parameter("zupt_chi2_multipler", params.zupt_options.chi2_multipler);
        node->get_parameter("zupt_max_velocity", params.zupt_max_velocity);
        node->get_parameter("zupt_noise_multiplier", params.zupt_noise_multiplier);

        // Recording of timing information to file
        node->declare_parameter<bool>("record_timing_information", params.record_timing_information);
        node->declare_parameter<std::string>("record_timing_filepath", params.record_timing_filepath);
        node->get_parameter("record_timing_information", params.record_timing_information);
        node->get_parameter("record_timing_filepath", params.record_timing_filepath);

        // NOISE ======================================================================

        // Our noise values for inertial sensor
        node->declare_parameter<double>("gyroscope_noise_density", params.imu_noises.sigma_w);
        node->declare_parameter<double>("accelerometer_noise_density", params.imu_noises.sigma_a);
        node->declare_parameter<double>("gyroscope_random_walk", params.imu_noises.sigma_wb);
        node->declare_parameter<double>("accelerometer_random_walk", params.imu_noises.sigma_ab);
        node->get_parameter("gyroscope_noise_density", params.imu_noises.sigma_w);
        node->get_parameter("accelerometer_noise_density", params.imu_noises.sigma_a);
        node->get_parameter("gyroscope_random_walk", params.imu_noises.sigma_wb);
        node->get_parameter("accelerometer_random_walk", params.imu_noises.sigma_ab);

        // Read in update parameters
        node->declare_parameter<double>("up_msckf_sigma_px", params.msckf_options.sigma_pix);
        node->declare_parameter<int>("up_msckf_chi2_multipler", params.msckf_options.chi2_multipler);
        node->declare_parameter<double>("up_slam_sigma_px", params.slam_options.sigma_pix);
        node->declare_parameter<int>("up_slam_chi2_multipler", params.slam_options.chi2_multipler);
        node->declare_parameter<double>("up_aruco_sigma_px", params.aruco_options.sigma_pix);
        node->declare_parameter<int>("up_aruco_chi2_multipler", params.aruco_options.chi2_multipler);
        node->get_parameter("up_msckf_sigma_px", params.msckf_options.sigma_pix);
        node->get_parameter("up_msckf_chi2_multipler", params.msckf_options.chi2_multipler);
        node->get_parameter("up_slam_sigma_px", params.slam_options.sigma_pix);
        node->get_parameter("up_slam_chi2_multipler", params.slam_options.chi2_multipler);
        node->get_parameter("up_aruco_sigma_px", params.aruco_options.sigma_pix);
        node->get_parameter("up_aruco_chi2_multipler", params.aruco_options.chi2_multipler);

        // STATE ======================================================================
        // Timeoffset from camera to IMU
        node->declare_parameter<double>("calib_camimu_dt", params.calib_camimu_dt);
        node->get_parameter("calib_camimu_dt", params.calib_camimu_dt);

        // Global gravity
        std::vector<double> gravity = {params.gravity(0), params.gravity(1), params.gravity(2)};
        node->declare_parameter<std::vector<double>>("gravity", gravity);
        node->get_parameter("gravity", gravity);
        assert(gravity.size()==3);
        params.gravity << gravity.at(0),gravity.at(1),gravity.at(2);


        // TRACKERS ======================================================================

        // Tracking flags
        node->declare_parameter<bool>("use_stereo", params.use_stereo);
        node->declare_parameter<bool>("use_klt", params.use_klt);
        node->declare_parameter<bool>("use_aruco", params.use_aruco);
        node->declare_parameter<bool>("downsize_aruco", params.downsize_aruco);
        node->declare_parameter<bool>("downsample_cameras", params.downsample_cameras);
        node->declare_parameter<bool>("multi_threading", params.use_multi_threading);
        node->get_parameter("use_stereo", params.use_stereo);
        node->get_parameter("use_klt", params.use_klt);
        node->get_parameter("use_aruco", params.use_aruco);
        node->get_parameter("downsize_aruco", params.downsize_aruco);
        node->get_parameter("downsample_cameras", params.downsample_cameras);
        node->get_parameter("multi_threading", params.use_multi_threading);

        // General parameters
        node->declare_parameter<int>("num_pts", params.num_pts);
        node->declare_parameter<int>("fast_threshold", params.fast_threshold);
        node->declare_parameter<int>("grid_x", params.grid_x);
        node->declare_parameter<int>("grid_y", params.grid_y);
        node->declare_parameter<int>("min_px_dist", params.min_px_dist);
        node->declare_parameter<double>("knn_ratio", params.knn_ratio);
        node->get_parameter("num_pts", params.num_pts);
        node->get_parameter("fast_threshold", params.fast_threshold);
        node->get_parameter("grid_x", params.grid_x);
        node->get_parameter("grid_y", params.grid_y);
        node->get_parameter("min_px_dist", params.min_px_dist);
        node->get_parameter("knn_ratio", params.knn_ratio);

        // Feature initializer parameters
        node->declare_parameter<bool>("fi_triangulate_1d", params.featinit_options.triangulate_1d);
        node->declare_parameter<bool>("fi_refine_features", params.featinit_options.refine_features);
        node->declare_parameter<int>("fi_max_runs", params.featinit_options.max_runs);
        node->declare_parameter<double>("fi_init_lamda", params.featinit_options.init_lamda);
        node->declare_parameter<double>("fi_max_lamda", params.featinit_options.max_lamda);
        node->declare_parameter<double>("fi_min_dx", params.featinit_options.min_dx);
        node->declare_parameter<double>("fi_min_dcost", params.featinit_options.min_dcost);
        node->declare_parameter<double>("fi_lam_mult", params.featinit_options.lam_mult);
        node->declare_parameter<double>("fi_min_dist", params.featinit_options.min_dist);
        node->declare_parameter<double>("fi_max_dist", params.featinit_options.max_dist);
        node->declare_parameter<double>("fi_max_baseline", params.featinit_options.max_baseline);
        node->declare_parameter<double>("fi_max_cond_number", params.featinit_options.max_cond_number);
        node->get_parameter("fi_triangulate_1d", params.featinit_options.triangulate_1d);
        node->get_parameter("fi_refine_features", params.featinit_options.refine_features);
        node->get_parameter("fi_max_runs", params.featinit_options.max_runs);
        node->get_parameter("fi_init_lamda", params.featinit_options.init_lamda);
        node->get_parameter("fi_max_lamda", params.featinit_options.max_lamda);
        node->get_parameter("fi_min_dx", params.featinit_options.min_dx);
        node->get_parameter("fi_min_dcost", params.featinit_options.min_dcost);
        node->get_parameter("fi_lam_mult", params.featinit_options.lam_mult);
        node->get_parameter("fi_min_dist", params.featinit_options.min_dist);
        node->get_parameter("fi_max_dist", params.featinit_options.max_dist);
        node->get_parameter("fi_max_baseline", params.featinit_options.max_baseline);
        node->get_parameter("fi_max_cond_number", params.featinit_options.max_cond_number);

        // SIMULATION ======================================================================

        // Load the groundtruth trajectory and its spline
        node->declare_parameter<std::string>("sim_traj_path", params.sim_traj_path);
        node->declare_parameter<double>("sim_distance_threshold", params.sim_distance_threshold);
        node->declare_parameter<bool>("sim_do_perturbation", params.sim_do_perturbation);
        node->get_parameter("sim_traj_path", params.sim_traj_path);
        node->get_parameter("sim_distance_threshold", params.sim_distance_threshold);
        node->get_parameter("sim_do_perturbation", params.sim_do_perturbation);

        // Read in sensor simulation frequencies
        node->declare_parameter<double>("sim_freq_cam", params.sim_freq_cam);
        node->declare_parameter<double>("sim_freq_imu", params.sim_freq_imu);
        node->get_parameter("sim_freq_cam", params.sim_freq_cam);
        node->get_parameter("sim_freq_imu", params.sim_freq_imu);

        // Load the seeds for the random number generators
        node->declare_parameter<int>("sim_seed_state_init", params.sim_seed_state_init);
        node->declare_parameter<int>("sim_seed_preturb", params.sim_seed_preturb);
        node->declare_parameter<int>("sim_seed_measurements", params.sim_seed_measurements);
        node->get_parameter("sim_seed_state_init", params.sim_seed_state_init);
        node->get_parameter("sim_seed_preturb", params.sim_seed_preturb);
        node->get_parameter("sim_seed_measurements", params.sim_seed_measurements);

        //====================================================================================
        //====================================================================================
        //====================================================================================

        // Loop through through, and load each of the cameras
        for(int i=0; i<params.state_options.num_cameras; i++) {

            // If our distortions are fisheye or not!
            bool is_fisheye = false;
            node->declare_parameter<bool>("cam"+std::to_string(i)+"_is_fisheye", is_fisheye);
            node->get_parameter("cam"+std::to_string(i)+"_is_fisheye", is_fisheye);

            // If the desired fov we should simulate
            std::vector<int64_t> matrix_wh;
            std::vector<int64_t> matrix_wd_default = {752,480};
            node->declare_parameter<std::vector<int64_t>>("cam"+std::to_string(i)+"_wh", matrix_wd_default);
            node->get_parameter("cam"+std::to_string(i)+"_wh", matrix_wh); 
            matrix_wh.at(0) /= (params.downsample_cameras) ? 2.0 : 1.0;
            matrix_wh.at(1) /= (params.downsample_cameras) ? 2.0 : 1.0;
            std::pair<int,int> wh(matrix_wh.at(0),matrix_wh.at(1));

            // Camera intrinsic properties
            Eigen::Matrix<double,8,1> cam_calib;
            std::vector<double> matrix_k, matrix_d;
            std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
            std::vector<double> matrix_d_default = {-0.05016623064875603, 0.057663965970277786, -0.0005494342185556889, 0.00038652922376058996};
            node->declare_parameter<std::vector<double>>("cam"+std::to_string(i)+"_k", matrix_k_default);
            node->declare_parameter<std::vector<double>>("cam"+std::to_string(i)+"_d", matrix_d_default);
            node->get_parameter("cam"+std::to_string(i)+"_k", matrix_k);
            node->get_parameter("cam"+std::to_string(i)+"_d", matrix_d);
            matrix_k.at(0) /= (params.downsample_cameras) ? 2.0 : 1.0;
            matrix_k.at(1) /= (params.downsample_cameras) ? 2.0 : 1.0;
            matrix_k.at(2) /= (params.downsample_cameras) ? 2.0 : 1.0;
            matrix_k.at(3) /= (params.downsample_cameras) ? 2.0 : 1.0;
            cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);

            // Our camera extrinsics transform
            Eigen::Matrix4d T_CtoI;
            std::vector<double> matrix_TCtoI;
            std::vector<double> matrix_TtoI_default = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

            // Read in from ROS, and save into our eigen mat
            node->declare_parameter<std::vector<double>>("T_C"+std::to_string(i)+"toI", matrix_TtoI_default);
            node->get_parameter("T_C"+std::to_string(i)+"toI", matrix_TCtoI);
            T_CtoI << matrix_TCtoI.at(0),matrix_TCtoI.at(1),matrix_TCtoI.at(2),matrix_TCtoI.at(3),
                    matrix_TCtoI.at(4),matrix_TCtoI.at(5),matrix_TCtoI.at(6),matrix_TCtoI.at(7),
                    matrix_TCtoI.at(8),matrix_TCtoI.at(9),matrix_TCtoI.at(10),matrix_TCtoI.at(11),
                    matrix_TCtoI.at(12),matrix_TCtoI.at(13),matrix_TCtoI.at(14),matrix_TCtoI.at(15);

            // Load these into our state
            Eigen::Matrix<double,7,1> cam_eigen;
            cam_eigen.block(0,0,4,1) = rot_2_quat(T_CtoI.block(0,0,3,3).transpose());
            cam_eigen.block(4,0,3,1) = -T_CtoI.block(0,0,3,3).transpose()*T_CtoI.block(0,3,3,1);

            // Insert
            params.camera_fisheye.insert({i, is_fisheye});
            params.camera_intrinsics.insert({i, cam_calib});
            params.camera_extrinsics.insert({i, cam_eigen});
            params.camera_wh.insert({i, wh});

        }

        // Success, lets returned the parsed options
        return params;

    }



}


#endif //OV_MSCKF_PARSE_ROSHANDLER_H
#endif //ROS_AVAILABLE
