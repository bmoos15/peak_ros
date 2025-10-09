#pragma once

#include <cmath>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "PeakMicroPulseHandler/peak_handler.h"

#include "peak_ros/Ascan.h"
#include "peak_ros/Observation.h"

#include "peak_ros/StreamData.h"
#include "peak_ros/TakeSingleMeasurement.h"

#include <fftw3.h>
#include <iostream>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <sstream>
#include <iomanip>

#include <dynamic_reconfigure/server.h>
#include <peak_ros/dynamic_variablesConfig.h>


namespace peak_namespace {

class PeakNodelet : public nodelet::Nodelet {
public:
    PeakNodelet();
    //~PeakNode();

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

    ros::NodeHandle                    nh_;
    ros::Rate                          rate_;
    int                                digitisation_rate_;
    std::string                        node_name_;
    std::string                        ns_;
    std::string                        package_path_;
    bool                               profile_;

    // Config
    int                                acquisition_rate_;
    std::string                        peak_address_;
    int                                peak_port_;
    std::string                        mps_file_;

    // TCG
    bool                               use_tcg_;
    float                              amp_factor_;
    float                              depth_factor_;
    float                              tcg_limit_;

    //Boundary Conditions
    bool                               immersion_;

    // Gates
    float                              gate_front_wall_;
    float                              depth_to_skip_;
    float                              gate_back_wall_;
    float                              max_depth_;
    bool                               zero_to_front_wall_;
    bool                               show_front_wall_;

    // Input
    PeakHandler                        peak_handler_;
    const PeakHandler::OutputFormat*   ltpa_data_ptr_;

    // Output
    peak_ros::Observation              ltpa_msg_;
    sensor_msgs::PointCloud2           bscan_cloud_;
    sensor_msgs::PointCloud2           gated_bscan_cloud_;
    sensor_msgs::PointCloud2           gate_top_cloud_;
    sensor_msgs::PointCloud2           gate_bottom_cloud_;

    bool                               stream_;

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

    ros::Timer                         timer_;

    // Add dynamic reconfigure server
    boost::shared_ptr<dynamic_reconfigure::Server<peak_ros::dynamic_variablesConfig>> dr_server_;
    dynamic_reconfigure::Server<peak_ros::dynamic_variablesConfig>::CallbackType dr_callback_;
    
    // Dynamic reconfigure callback
    void reconfigureCallback(peak_ros::dynamic_variablesConfig &config, uint32_t level);


    // Variables for tracking average depth
    std::vector<float> depth_front_wall_values_;
    const size_t max_depth_samples_ = 100;  // Maximum samples for rolling buffer

    // Function to publish depth marker
    void publishDepthMarker(float avg_depth);

    // ============================================================================
    // ADDITIONAL MEMBER VARIABLES FOR FRONT WALL ANGLE CALCULATION
    // ============================================================================

    // Storage for front wall depth and position data
    std::vector<float> element_positions_;  // Y positions corresponding to depths

    // RANSAC parameters
    const int ransac_iterations_ = 500;      // Number of RANSAC iterations
    const float ransac_threshold_ = 0.00000000005f;  // Inlier threshold (2mm in meters)

    // Front wall angle (in degrees)
    float frontwall_angle_ = 0.0f;


    // ============================================================================
    // ADDITIONAL METHOD DECLARATIONS
    // ============================================================================

    /**
     * @brief Calculate front wall angle using RANSAC line fitting
     * @param y_positions Vector of element Y positions
     * @param depths Vector of corresponding front wall depths
     * @return Calculated angle in degrees
     */
    float calculateFrontwallAngleRANSAC(const std::vector<float>& y_positions, 
                                         const std::vector<float>& depths);

    /**
     * @brief Publish front wall angle as text visualization marker
     * @param frontwall_angle Angle to publish in degrees
     */
    void publishAngleMarker(float frontwall_angle);

    // ============================================================================
    // ADDITIONAL INCLUDES NEEDED
    // ============================================================================

    // Add to your includes at the top of peak_nodelet.h:
    // #include <std_msgs/Float32.h>
    // #include <cmath>
    // #include <cstdlib>  // for rand()

    // ============================================================================
    // NOTES ON IMPLEMENTATION
    // ============================================================================

    /*
     * RANSAC Algorithm Overview:
     * 1. Randomly select 2 points from the dataset
     * 2. Fit a line through these points (z = slope * y + intercept)
     * 3. Count how many other points are within threshold distance (inliers)
     * 4. Repeat for N iterations
     * 5. Keep the model with the most inliers
     * 6. Convert slope to angle: angle = atan(dz/dy)
     *
     * Benefits of RANSAC:
     * - Robust to outliers (bad detections, noise)
     * - Works well with moderate amounts of bad data
     * - Simple and efficient for line fitting
     *
     * Parameters to tune:
     * - ransac_iterations_: More iterations = better fit but slower (100 is typical)
     * - ransac_threshold_: Larger = more tolerant to noise, smaller = tighter fit
     *                      (0.002m = 2mm is reasonable for ultrasonic data)
     * - max_depth_samples_: Number of historical measurements to keep
     *                       (100 gives good smoothing without too much lag)
     *
     * Published Topics:
     * - /peak/frontwall_angle (std_msgs/Float32): Numerical angle value
     * - /peak/angle_text_marker (visualization_msgs/Marker): Visual text display
     *
     * The angle is calculated from the slope of the fitted line:
     * - Positive angle: front wall slopes upward in +Y direction
     * - Negative angle: front wall slopes downward in +Y direction
     * - Zero angle: front wall is perpendicular to array
     */
    

};

} // namespace peak_namespace