#pragma once

#include <cmath>
#include <cstdlib>
#include <signal.h>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <visualization_msgs/Marker.h>
#include <sstream>
#include <iomanip>
#include <iostream>

#include <dynamic_reconfigure/server.h>
#include <peak_ros/dynamic_variablesConfig.h>

#include <fftw3.h>
#include "PeakMicroPulseHandler/peak_handler.h"

#include "peak_ros/Ascan.h"
#include "peak_ros/Observation.h"
#include "peak_ros/StreamData.h"
#include "peak_ros/TakeSingleMeasurement.h"


namespace peak_namespace {

class PeakNodelet : public nodelet::Nodelet {
public:
    PeakNodelet();

private:
    virtual void                       onInit();

    template <typename ParamType>
    ParamType                          paramHandler(std::string param_name, ParamType& param_value);

    void                               initHardware();
    void                               prePopulateAScanMessage();
    void                               prePopulateBScanMessage();
    void                               prePopulateGatedBScanMessage();
    void                               prePopulateGateTopMessage();
    void                               prePopulateGateBottomMessage();
    bool                               streamDataSrvCb(peak_ros::StreamData::Request& request,
                                                       peak_ros::StreamData::Response& response);
    bool                               takeMeasurementSrvCb(peak_ros::TakeSingleMeasurement::Request& request,
                                                            peak_ros::TakeSingleMeasurement::Response& response);
    void                               takeMeasurement();
    void                               populateAScanMessage();
    void                               populateBScanMessage(const peak_ros::Observation& obs_msg);
    void                               timerCb(const ros::TimerEvent& /*event*/);
    void                               reconfigureCallback(peak_ros::dynamic_variablesConfig &config, uint32_t level);

    void                               publishDepthMarker(float avg_depth);
    void                               publishAngleMarker(float frontwall_angle);

    float                              calculateFrontwallAngleRANSAC(const std::vector<float>& y_positions,
                                                                      const std::vector<float>& depths);

    // Hilbert transform: returns the real part of the analytic signal via FFTW3
    std::vector<float>                 computeHilbertReal(const std::vector<int>& signal);

    // -------------------------------------------------------------------
    // ROS handles
    // -------------------------------------------------------------------
    ros::NodeHandle                    nh_;
    ros::Rate                          rate_;
    ros::Timer                         timer_;
    ros::Publisher                     ascan_publisher_;
    ros::Publisher                     bscan_publisher_;
    ros::Publisher                     gated_bscan_publisher_;
    ros::Publisher                     gate_top_publisher_;
    ros::Publisher                     gate_bottom_publisher_;
    ros::Publisher                     depth_marker_publisher_;
    ros::Publisher                     angle_marker_publisher_;
    ros::Publisher                     frontwall_angle_publisher_;
    ros::Publisher                     frontwall_depth_publisher_;
    ros::ServiceServer                 single_measure_service_;
    ros::ServiceServer                 stream_service_;

    boost::shared_ptr<dynamic_reconfigure::Server<peak_ros::dynamic_variablesConfig>> dr_server_;
    dynamic_reconfigure::Server<peak_ros::dynamic_variablesConfig>::CallbackType      dr_callback_;

    // -------------------------------------------------------------------
    // Node config
    // -------------------------------------------------------------------
    std::string                        node_name_;
    std::string                        ns_;
    std::string                        package_path_;
    int                                digitisation_rate_;
    int                                acquisition_rate_;
    std::string                        peak_address_;
    int                                peak_port_;
    std::string                        mps_file_;
    bool                               profile_;

    // TCG
    bool                               use_tcg_;
    float                              amp_factor_;
    float                              depth_factor_;
    float                              tcg_limit_;

    // Boundary conditions
    bool                               immersion_;

    // Gates
    float                              gate_front_wall_;
    float                              depth_to_skip_;
    float                              gate_back_wall_;
    float                              max_depth_;
    bool                               zero_to_front_wall_;
    bool                               show_front_wall_;

    // -------------------------------------------------------------------
    // Data
    // -------------------------------------------------------------------
    PeakHandler                        peak_handler_;
    const PeakHandler::OutputFormat*   ltpa_data_ptr_;

    peak_ros::Observation              ltpa_msg_;
    sensor_msgs::PointCloud2           bscan_cloud_;
    sensor_msgs::PointCloud2           gated_bscan_cloud_;
    sensor_msgs::PointCloud2           gate_top_cloud_;
    sensor_msgs::PointCloud2           gate_bottom_cloud_;

    bool                               stream_;

    // -------------------------------------------------------------------
    // Front wall tracking
    // -------------------------------------------------------------------
    std::vector<float>                 depth_front_wall_values_;
    std::vector<float>                 element_positions_;
    const size_t                       max_depth_samples_ = 100;

    float                              frontwall_angle_ = 0.0f;

    // RANSAC parameters
    const int                          ransac_iterations_ = 500;
    const float                        ransac_threshold_  = 0.00000000005f;
};

} // namespace peak_namespace