/*
 * CRITICAL FIXES APPLIED:
 * =======================
 * 
 * FIX 1: Depth Calculation (Line ~645)
 * -------------------------------------
 * The depth calculation was incorrect - it was missing the conversion factor for digitisation_rate.
 * 
 * WRONG (original code):
 *   float tof = (float)(i - t) / (float)obs_msg.digitisation_rate;
 *   float z = tof * n / 2.0f;
 * 
 * CORRECT (fixed to match peak_nodelet.cpp):
 *   float dt = 1.0f / ((float)obs_msg.digitisation_rate * 1000000.0f);
 *   float z = (float)((float)i * (float)n * dt / 2.0f);
 * 
 * The digitisation_rate is in MHz, so it must be multiplied by 1,000,000 to convert to Hz
 * before calculating the time step (dt) in seconds. This was causing the B-scan depths 
 * to be grossly incorrect (off by a factor of 1,000,000).
 * 
 * FIX 2: Reference Frame (Lines ~595, 601, 607, 613, 486)
 * --------------------------------------------------------
 * The postprocessor was transforming points from ltpa frame to base_link frame and publishing
 * in base_link. However, peak_nodelet publishes directly in the ltpa frame without transformation.
 * 
 * WRONG (original):
 *   - Compute points in ltpa coordinates
 *   - Transform each point to base_link using TF
 *   - Publish point cloud with frame_id = "base_link"
 * 
 * CORRECT (fixed to match peak_nodelet.cpp):
 *   - Compute points in ltpa coordinates
 *   - Publish point cloud with frame_id = "ltpa" (no transformation)
 *   - Let TF system handle visualization transforms in RViz
 * 
 * This fix removes all transformPoint() calls and sets all point cloud frame_ids to "ltpa".
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <peak_ros/Observation.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
// Add this include at the top of ascan_postprocessor.cpp (with the other includes)
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <string>
#include <map>
#include <limits>
#include <cmath>
#include <sstream>
#include <iomanip>

class AScanPostProcessor {
public:
    AScanPostProcessor() : tf_buffer_(ros::Duration(300.0)), tf_listener_(tf_buffer_) {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Load parameters
        private_nh.param<std::string>("input_bag", input_bag_path_, "");
        private_nh.param<std::string>("output_bag", output_bag_path_, "output.bag");
        private_nh.param<bool>("use_tcg", use_tcg_, true);
        private_nh.param<float>("amp_factor", amp_factor_, 0.0);
        private_nh.param<float>("depth_factor", depth_factor_, 10.0); // in mm, will be converted
        private_nh.param<float>("tcg_limit", tcg_limit_, 1.0);
        private_nh.param<float>("gate_front_wall", gate_front_wall_, 0.5);
        private_nh.param<float>("depth_to_skip", depth_to_skip_, 35.0); // in mm, will be converted
        private_nh.param<float>("gate_back_wall", gate_back_wall_, 0.5);
        private_nh.param<float>("max_depth", max_depth_, 30.0); // in mm, will be converted
        private_nh.param<bool>("zero_to_front_wall", zero_to_front_wall_, true);
        private_nh.param<bool>("show_front_wall", show_front_wall_, true);
        private_nh.param<bool>("immersion", immersion_, true);
        private_nh.param<int>("ransac_iterations", ransac_iterations_, 100);
        private_nh.param<float>("ransac_threshold", ransac_threshold_, 0.001);

        // Convert mm to m
        depth_factor_ *= 0.001f;
        depth_to_skip_ *= 0.001f;
        max_depth_ *= 0.001f;

        // Publishers for reconstructed data
        bscan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/peak/reconstructed_b_scan", 10, true);
        gated_bscan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/peak/reconstructed_gated_b_scan", 10, true);
        detections_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/peak/detections", 10, true);  // Added detections publisher
        gate_top_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/peak/reconstructed_gate_top", 10, true);
        gate_bottom_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/peak/reconstructed_gate_bottom", 10, true);
        depth_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/peak/reconstructed_depth_marker", 10, true);
        angle_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/peak/reconstructed_angle_marker", 10, true);
        
        // Publisher for TF messages from bag (for real-time visualization of recorded robot state)
        tf_pub_ = nh.advertise<tf2_msgs::TFMessage>("/tf", 10);
        
        // Publisher for joint states (for robot_state_publisher to publish end-effector transforms)
        joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

        // Static transform broadcaster for publishing static transforms
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

        initializePointClouds();
        initializeAccumulatedDetections();  // Initialize accumulated detections
        
        // Wait a moment for TF listener to be ready
        ros::Duration(0.5).sleep();
        
        ROS_INFO("Post-processor initialized and listening for transforms");
    }

    void processBag() {
        if (input_bag_path_.empty()) {
            ROS_ERROR("No input bag file specified! Use _input_bag:=<path>");
            return;
        }

        ROS_INFO("Processing bag file: %s", input_bag_path_.c_str());

        rosbag::Bag input_bag;
        rosbag::Bag output_bag;

        try {
            input_bag.open(input_bag_path_, rosbag::bagmode::Read);
            output_bag.open(output_bag_path_, rosbag::bagmode::Write);
            
            // Write static transforms to bag at the beginning
            // Use the first timestamp from the bag
            rosbag::View view_for_time(input_bag);
            ros::Time bag_start_time = view_for_time.getBeginTime();
            writeStaticTransformsToBag(output_bag, bag_start_time);

            // Only read A-scan data, angles, and depths from bag
            // TF should come from static_transform_publishers in launch file
            std::vector<std::string> topics;
            topics.push_back("/peak/a_scans");
            topics.push_back("/peak/frontwall_angle");
            topics.push_back("/peak/frontwall_depth");
            topics.push_back("/tf");  // Include dynamic TF if any
            topics.push_back("/joint_states");  // Include joint states for robot_state_publisher

            rosbag::View view(input_bag, rosbag::TopicQuery(topics));

            peak_ros::Observation current_observation;
            std_msgs::Float32 current_angle;
            std_msgs::Float32 current_depth;
            bool has_observation = false;
            bool has_angle = false;
            bool has_depth = false;

            ROS_INFO("Starting bag processing. Static transforms should be available from launch file.");
            ROS_INFO("Filtering duplicate TF messages from bag file...");
            
            // Track published transforms to avoid duplicates
            // Key: "timestamp_frame_id_parent_id", Value: true
            std::map<std::string, bool> published_transforms;
            int total_transforms = 0;
            int filtered_duplicates = 0;
            
            // Process initial TF messages if they exist in the bag
            for (rosbag::MessageInstance const m : view) {
                // Process dynamic TF messages from bag
                tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
                if (tf_msg != nullptr) {
                    // Filter out duplicate transforms
                    tf2_msgs::TFMessage filtered_msg;
                    
                    int msg_count = tf_msg->transforms.size();
                    
                    for (const auto& transform : tf_msg->transforms) {
                        total_transforms++;
                        
                        // Create unique key for this transform
                        std::stringstream key;
                        key << transform.header.stamp.sec << "_" 
                            << transform.header.stamp.nsec << "_"
                            << transform.child_frame_id << "_"
                            << transform.header.frame_id;
                        
                        // Only add if we haven't seen this exact transform before
                        if (published_transforms.find(key.str()) == published_transforms.end()) {
                            filtered_msg.transforms.push_back(transform);
                            published_transforms[key.str()] = true;
                            
                            // Don't add directly to buffer - tf_listener will do it when we publish
                            // This avoids duplicate warnings from buffer trying to add it twice
                        } else {
                            filtered_duplicates++;
                            if (filtered_duplicates <= 10) {
                                ROS_WARN("DUPLICATE FOUND: %s -> %s at %d.%09d",
                                         transform.header.frame_id.c_str(),
                                         transform.child_frame_id.c_str(),
                                         transform.header.stamp.sec,
                                         transform.header.stamp.nsec);
                            }
                        }
                    }
                    
                    // ROS_INFO_THROTTLE(1.0, "Processing TF: received %d transforms, publishing %lu (filtered %d duplicates so far)",
                    //                   msg_count, filtered_msg.transforms.size(), filtered_duplicates);
                    
                    // Only publish if there are transforms after filtering
                    if (!filtered_msg.transforms.empty()) {
                        tf_pub_.publish(filtered_msg);
                        
                        // Write filtered message to output bag (not the original with duplicates)
                        output_bag.write(m.getTopic(), m.getTime(), filtered_msg);
                    }
                    
                    continue;
                }
                
                // Process joint states messages from bag (if they exist)
                sensor_msgs::JointState::ConstPtr joint_msg = m.instantiate<sensor_msgs::JointState>();
                if (joint_msg != nullptr) {
                    // Republish joint states for robot_state_publisher
                    joint_states_pub_.publish(joint_msg);
                    output_bag.write(m.getTopic(), m.getTime(), m);
                    continue;
                }

                // Collect A-scan observation
                peak_ros::Observation::ConstPtr obs_msg = m.instantiate<peak_ros::Observation>();
                if (obs_msg != nullptr) {
                    current_observation = *obs_msg;
                    has_observation = true;
                    output_bag.write(m.getTopic(), m.getTime(), m);
                    continue;
                }

                // Collect angle
                std_msgs::Float32::ConstPtr angle_msg = m.instantiate<std_msgs::Float32>();
                if (angle_msg != nullptr && m.getTopic() == "/peak/frontwall_angle") {
                    current_angle = *angle_msg;
                    has_angle = true;
                    output_bag.write(m.getTopic(), m.getTime(), m);
                    continue;
                }

                // Collect depth
                if (angle_msg != nullptr && m.getTopic() == "/peak/frontwall_depth") {
                    current_depth = *angle_msg;
                    has_depth = true;
                    output_bag.write(m.getTopic(), m.getTime(), m);

                    // When we have all three pieces of data, process and publish
                    if (has_observation && has_angle && has_depth) {
                        processObservation(current_observation, current_angle.data, current_depth.data);

                        // Write reconstructed data to output bag
                        output_bag.write("/peak/reconstructed_b_scan", current_observation.header.stamp, bscan_cloud_);
                        output_bag.write("/peak/reconstructed_gated_b_scan", current_observation.header.stamp, gated_bscan_cloud_);
                        output_bag.write("/peak/reconstructed_gate_top", current_observation.header.stamp, gate_top_cloud_);
                        output_bag.write("/peak/reconstructed_gate_bottom", current_observation.header.stamp, gate_bottom_cloud_);

                        // Accumulate detections from gated B-scan
                        accumulateDetections(gated_bscan_cloud_);
                        
                        ROS_INFO_THROTTLE(10.0, "Accumulated detections so far: %d points", accumulated_detections_.width);

                        // Publish for real-time visualization if nodes are running
                        bscan_pub_.publish(bscan_cloud_);
                        gated_bscan_pub_.publish(gated_bscan_cloud_);
                        gate_top_pub_.publish(gate_top_cloud_);
                        gate_bottom_pub_.publish(gate_bottom_cloud_);

                        publishDepthMarker(current_depth.data, current_observation.header.stamp);
                        publishAngleMarker(current_angle.data, current_observation.header.stamp);

                        // Reset flags
                        has_observation = false;
                        has_angle = false;
                        has_depth = false;
                        
                        // Allow ROS callbacks to process
                        ros::spinOnce();
                    }
                }
            }

            input_bag.close();

            // Write the final accumulated detections message at the end
            ROS_INFO("Preparing to write accumulated detections...");
            ROS_INFO("Total accumulated detection points: %d", accumulated_detections_.width);
            
            if (accumulated_detections_.width > 0) {
                ros::Time final_timestamp = ros::Time::now();
                accumulated_detections_.header.stamp = final_timestamp;
                output_bag.write("/peak/detections", final_timestamp, accumulated_detections_);
                
                // Also publish for real-time visualization
                detections_pub_.publish(accumulated_detections_);
                
                ROS_INFO("Successfully written accumulated detections to bag: %d points", accumulated_detections_.width);
            } else {
                ROS_WARN("No detections accumulated - detections message not written to bag");
            }

            output_bag.close();

            ROS_INFO("Processing complete. Output saved to: %s", output_bag_path_.c_str());

        } catch (rosbag::BagException& e) {
            ROS_ERROR("Error processing bag file: %s", e.what());
        }
    }

private:

    // Add this helper function in the private section of the AScanPostProcessor class
    // This creates a quaternion from RPY angles, matching static_transform_publisher convention
    // static_transform_publisher uses: x y z yaw pitch roll
    // So we pass in (yaw, pitch, roll) to match the launch file order
    geometry_msgs::Quaternion createQuaternionFromYPR(double yaw, double pitch, double roll) {
        tf2::Quaternion quat;
        // tf2::Quaternion::setRPY takes (roll, pitch, yaw) - NOTE THE ORDER!
        quat.setRPY(roll, pitch, yaw);
        
        geometry_msgs::Quaternion quat_msg;
        quat_msg.x = quat.x();
        quat_msg.y = quat.y();
        quat_msg.z = quat.z();
        quat_msg.w = quat.w();
        
        return quat_msg;
    }

    // Helper function to create all static transforms and write to bag
    void writeStaticTransformsToBag(rosbag::Bag& bag, const ros::Time& stamp) {
        tf2_msgs::TFMessage static_transforms;
        
        // World to base_link
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "world";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0;
        tf.transform.translation.z = 0;
        tf.transform.rotation.x = 0;
        tf.transform.rotation.y = 0;
        tf.transform.rotation.z = 0;
        tf.transform.rotation.w = 1;
        static_transforms.transforms.push_back(tf);
        
        // link_6 to link_coupling1
        tf.header.frame_id = "link_6";
        tf.child_frame_id = "link_coupling1";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0;
        tf.transform.translation.z = 0.02;
        tf.transform.rotation = createQuaternionFromYPR(0, 0, -1.5708);
        static_transforms.transforms.push_back(tf);
        
        // link_coupling1 to link_coupling2
        tf.header.frame_id = "link_coupling1";
        tf.child_frame_id = "link_coupling2";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0.005;
        tf.transform.translation.z = 0;
        tf.transform.rotation = createQuaternionFromYPR(0, 0, 0);
        static_transforms.transforms.push_back(tf);
        
        // link_coupling2 to ft_sensor
        tf.header.frame_id = "link_coupling2";
        tf.child_frame_id = "ft_sensor";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0;
        tf.transform.translation.z = 0;
        tf.transform.rotation = createQuaternionFromYPR(0, 0, -3.14159);
        static_transforms.transforms.push_back(tf);
        
        // ft_sensor to link_ee
        tf.header.frame_id = "ft_sensor";
        tf.child_frame_id = "link_ee";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0.0435;
        tf.transform.translation.z = 0;
        tf.transform.rotation = createQuaternionFromYPR(0, 0, 3.14159);
        static_transforms.transforms.push_back(tf);
        
        // link_ee to probe_mount
        tf.header.frame_id = "link_ee";
        tf.child_frame_id = "probe_mount";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0;
        tf.transform.translation.z = 0;
        tf.transform.rotation = createQuaternionFromYPR(0, -1.5708, -1.5708);
        static_transforms.transforms.push_back(tf);
        
        // probe_mount to tool_center_point
        tf.header.frame_id = "probe_mount";
        tf.child_frame_id = "tool_center_point";
        tf.transform.translation.x = 0.0125;
        tf.transform.translation.y = -0.016;
        tf.transform.translation.z = -0.245;
        tf.transform.rotation = createQuaternionFromYPR(1.5708, 0, 3.14159);
        static_transforms.transforms.push_back(tf);
        
        // tool_center_point to ltpa
        tf.header.frame_id = "tool_center_point";
        tf.child_frame_id = "ltpa";
        tf.transform.translation.x = 0.032;
        tf.transform.translation.y = 0;
        tf.transform.translation.z = 0;
        tf.transform.rotation = createQuaternionFromYPR(1.5708, 0, 0);
        static_transforms.transforms.push_back(tf);
        
        // ltpa to frontwall_depth
        tf.header.frame_id = "ltpa";
        tf.child_frame_id = "frontwall_depth";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0;
        tf.transform.translation.z = -0.017;
        tf.transform.rotation.x = 0;
        tf.transform.rotation.y = 0;
        tf.transform.rotation.z = 0;
        tf.transform.rotation.w = 1;
        static_transforms.transforms.push_back(tf);
        
        // ltpa to frontwall_angle
        tf.header.frame_id = "ltpa";
        tf.child_frame_id = "frontwall_angle";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0;
        tf.transform.translation.z = -0.01;
        tf.transform.rotation.x = 0;
        tf.transform.rotation.y = 0;
        tf.transform.rotation.z = 0;
        tf.transform.rotation.w = 1;
        static_transforms.transforms.push_back(tf);
        
        // Write to /tf_static topic
        bag.write("/tf_static", stamp, static_transforms);
        
        ROS_INFO("Written %lu static transforms to output bag on /tf_static", static_transforms.transforms.size());
    }

    void initializePointClouds() {
        // B-scan
        sensor_msgs::PointCloud2Modifier bscan_modifier(bscan_cloud_);
        bscan_modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "Amplitudes", 1, sensor_msgs::PointField::FLOAT32);
        bscan_cloud_.height = 1;
        bscan_cloud_.is_dense = true;

        // Gated B-scan
        sensor_msgs::PointCloud2Modifier gated_bscan_modifier(gated_bscan_cloud_);
        gated_bscan_modifier.setPointCloud2Fields(5,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "Amplitudes", 1, sensor_msgs::PointField::FLOAT32,
            "TimeofFlight", 1, sensor_msgs::PointField::FLOAT32);
        gated_bscan_cloud_.height = 1;
        gated_bscan_cloud_.is_dense = true;

        // Gate top (with RGB for red color in RViz)
        sensor_msgs::PointCloud2Modifier gate_top_modifier(gate_top_cloud_);
        gate_top_modifier.setPointCloud2Fields(6,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::PointField::FLOAT32,
            "Amplitudes", 1, sensor_msgs::PointField::FLOAT32,
            "TimeofFlight", 1, sensor_msgs::PointField::FLOAT32);
        gate_top_cloud_.height = 1;
        gate_top_cloud_.is_dense = true;

        // Gate bottom (with RGB for red color in RViz)
        sensor_msgs::PointCloud2Modifier gate_bottom_modifier(gate_bottom_cloud_);
        gate_bottom_modifier.setPointCloud2Fields(6,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::PointField::FLOAT32,
            "Amplitudes", 1, sensor_msgs::PointField::FLOAT32,
            "TimeofFlight", 1, sensor_msgs::PointField::FLOAT32);
        gate_bottom_cloud_.height = 1;
        gate_bottom_cloud_.is_dense = true;
    }

    void initializeAccumulatedDetections() {
        // Initialize accumulated detections with the same structure as gated B-scan
        sensor_msgs::PointCloud2Modifier detections_modifier(accumulated_detections_);
        detections_modifier.setPointCloud2Fields(5,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "Amplitudes", 1, sensor_msgs::PointField::FLOAT32,
            "TimeofFlight", 1, sensor_msgs::PointField::FLOAT32);
        accumulated_detections_.height = 1;
        accumulated_detections_.width = 0;
        accumulated_detections_.is_dense = false;  // May contain NaN values
        accumulated_detections_.header.frame_id = "base_link";  // Accumulate in base_link so they don't move with sensor
    }

    // // Helper function to transform a point from ltpa frame to base_link frame
    // bool transformPoint(float& x, float& y, float& z, const ros::Time& stamp) {
    //     try {
    //         // Get transform from ltpa to base_link
    //         // Use the closest available transform since bag timestamps may not be perfectly synced
    //         geometry_msgs::TransformStamped transform_stamped;
            
    //         // Try to get transform at the requested time, but allow some tolerance
    //         // by using the "canTransform" with timeout
    //         if (!tf_buffer_.canTransform("base_link", "ltpa", stamp, ros::Duration(0.1))) {
    //             // If exact time not available, use closest available
    //             transform_stamped = tf_buffer_.lookupTransform("base_link", "ltpa", ros::Time(0), ros::Duration(1.0));
    //             ROS_DEBUG_THROTTLE(10.0, "Using latest transform instead of exact timestamp %.3f", stamp.toSec());
    //         } else {
    //             transform_stamped = tf_buffer_.lookupTransform("base_link", "ltpa", stamp, ros::Duration(1.0));
    //         }
            
    //         // Create point in ltpa frame
    //         geometry_msgs::PointStamped point_ltpa;
    //         point_ltpa.header.frame_id = "ltpa";
    //         point_ltpa.header.stamp = stamp;
    //         point_ltpa.point.x = x;
    //         point_ltpa.point.y = y;
    //         point_ltpa.point.z = z;
            
    //         // Transform to base_link frame
    //         geometry_msgs::PointStamped point_base_link;
    //         tf2::doTransform(point_ltpa, point_base_link, transform_stamped);
            
    //         // Update the point coordinates
    //         x = point_base_link.point.x;
    //         y = point_base_link.point.y;
    //         z = point_base_link.point.z;
            
    //         return true;
    //     } catch (tf2::TransformException& ex) {
    //         ROS_WARN_THROTTLE(1.0, "Failed to transform point from ltpa to base_link at time %.3f: %s", 
    //                         stamp.toSec(), ex.what());
    //         return false;
    //     }
    // }

    // MODIFIED accumulateDetections function:
    void accumulateDetections(const sensor_msgs::PointCloud2& gated_scan) {
        // Extract only non-NaN points from gated scan to add to accumulated detections
        sensor_msgs::PointCloud2ConstIterator<float> iterX(gated_scan, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iterY(gated_scan, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iterZ(gated_scan, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iterAmps(gated_scan, "Amplitudes");
        sensor_msgs::PointCloud2ConstIterator<float> iterTof(gated_scan, "TimeofFlight");

        // CRITICAL FIX: Look up the transform ONCE for this entire observation
        // This ensures frontwall and backwall use the SAME transform
        geometry_msgs::TransformStamped transform_stamped;
        bool transform_available = false;
        
        try {
            // Try to get transform at the requested time
            if (!tf_buffer_.canTransform("base_link", "ltpa", gated_scan.header.stamp, ros::Duration(0.1))) {
                // If exact time not available, use closest available
                // But do this lookup ONCE and reuse it for all points
                transform_stamped = tf_buffer_.lookupTransform("base_link", "ltpa", ros::Time(0), ros::Duration(1.0));
                ROS_DEBUG_THROTTLE(10.0, "Using latest transform for timestamp %.3f", gated_scan.header.stamp.toSec());
            } else {
                transform_stamped = tf_buffer_.lookupTransform("base_link", "ltpa", gated_scan.header.stamp, ros::Duration(1.0));
            }
            transform_available = true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "Failed to get transform from ltpa to base_link at time %.3f: %s - skipping this observation", 
                            gated_scan.header.stamp.toSec(), ex.what());
            return;  // Skip this entire observation if transform not available
        }

        // Count valid (non-NaN) points and transform them to base_link using the SAME transform
        std::vector<float> valid_x, valid_y, valid_z, valid_amps, valid_tof;
        
        for (size_t i = 0; i < gated_scan.width; ++i, ++iterX, ++iterY, ++iterZ, ++iterAmps, ++iterTof) {
            if (!std::isnan(*iterX) && !std::isnan(*iterY) && !std::isnan(*iterZ)) {
                // Create point in ltpa frame
                geometry_msgs::PointStamped point_ltpa;
                point_ltpa.header.frame_id = "ltpa";
                point_ltpa.header.stamp = gated_scan.header.stamp;
                point_ltpa.point.x = *iterX;
                point_ltpa.point.y = *iterY;
                point_ltpa.point.z = *iterZ;
                
                // Transform to base_link frame using the SAME transform for all points
                geometry_msgs::PointStamped point_base_link;
                tf2::doTransform(point_ltpa, point_base_link, transform_stamped);
                
                // Add transformed point to valid points list
                valid_x.push_back(point_base_link.point.x);
                valid_y.push_back(point_base_link.point.y);
                valid_z.push_back(point_base_link.point.z);
                valid_amps.push_back(*iterAmps);
                valid_tof.push_back(*iterTof);
            }
        }

        if (valid_x.empty()) {
            ROS_DEBUG_THROTTLE(5.0, "No valid detections in this scan (all NaN)");
            return;  // No valid detections in this scan
        }

        ROS_DEBUG_THROTTLE(5.0, "Found %zu valid detections in this scan", valid_x.size());

        // Resize accumulated detections to include new points
        size_t old_size = accumulated_detections_.width;
        size_t new_size = old_size + valid_x.size();
        
        sensor_msgs::PointCloud2Modifier modifier(accumulated_detections_);
        modifier.resize(new_size);
        accumulated_detections_.width = new_size;

        // Add new points to accumulated detections
        sensor_msgs::PointCloud2Iterator<float> accum_iterX(accumulated_detections_, "x");
        sensor_msgs::PointCloud2Iterator<float> accum_iterY(accumulated_detections_, "y");
        sensor_msgs::PointCloud2Iterator<float> accum_iterZ(accumulated_detections_, "z");
        sensor_msgs::PointCloud2Iterator<float> accum_iterAmps(accumulated_detections_, "Amplitudes");
        sensor_msgs::PointCloud2Iterator<float> accum_iterTof(accumulated_detections_, "TimeofFlight");

        // Skip to the end of existing points
        for (size_t i = 0; i < old_size; ++i) {
            ++accum_iterX;
            ++accum_iterY;
            ++accum_iterZ;
            ++accum_iterAmps;
            ++accum_iterTof;
        }

        // Add new valid points
        for (size_t i = 0; i < valid_x.size(); ++i) {
            *accum_iterX = valid_x[i];
            *accum_iterY = valid_y[i];
            *accum_iterZ = valid_z[i];
            *accum_iterAmps = valid_amps[i];
            *accum_iterTof = valid_tof[i];

            ++accum_iterX;
            ++accum_iterY;
            ++accum_iterZ;
            ++accum_iterAmps;
            ++accum_iterTof;
        }
    }       
 

    void processObservation(const peak_ros::Observation& obs_msg, float frontwall_angle, float frontwall_depth) {
        // DEBUG: Print velocity information
        ROS_INFO_ONCE("=== VELOCITY DEBUG INFO ===");
        ROS_INFO_ONCE("obs_msg.vel_couplant = %.2f m/s", obs_msg.vel_couplant);
        ROS_INFO_ONCE("obs_msg.vel_material = %.2f m/s", obs_msg.vel_material);
        ROS_INFO_ONCE("obs_msg.vel_wedge = %.2f m/s", obs_msg.vel_wedge);
        ROS_INFO_ONCE("immersion_ (parameter) = %s", immersion_ ? "true" : "false");
        ROS_INFO_ONCE("Expected: Start with n=1500 (water), switch to n=6420 (steel) after frontwall");
        ROS_INFO_ONCE("=========================");
        
        // Resize point clouds
        int total_points = obs_msg.num_ascans * obs_msg.ascan_length;
        sensor_msgs::PointCloud2Modifier bscan_modifier(bscan_cloud_);
        bscan_modifier.resize(total_points);
        bscan_cloud_.header = obs_msg.header;
        bscan_cloud_.header.frame_id = "ltpa";  // Publish in ltpa frame like real-time system
        bscan_cloud_.width = total_points;

        sensor_msgs::PointCloud2Modifier gated_bscan_modifier(gated_bscan_cloud_);
        gated_bscan_modifier.resize(total_points);
        gated_bscan_cloud_.header = obs_msg.header;
        gated_bscan_cloud_.header.frame_id = "ltpa";  // Publish in ltpa frame like real-time system
        gated_bscan_cloud_.width = total_points;

        sensor_msgs::PointCloud2Modifier gate_top_modifier(gate_top_cloud_);
        gate_top_modifier.resize(total_points);
        gate_top_cloud_.header = obs_msg.header;
        gate_top_cloud_.header.frame_id = "ltpa";  // Publish in ltpa frame like real-time system
        gate_top_cloud_.width = total_points;

        sensor_msgs::PointCloud2Modifier gate_bottom_modifier(gate_bottom_cloud_);
        gate_bottom_modifier.resize(total_points);
        gate_bottom_cloud_.header = obs_msg.header;
        gate_bottom_cloud_.header.frame_id = "ltpa";  // Publish in ltpa frame like real-time system
        gate_bottom_cloud_.width = total_points;

        // Create iterators for B-scan
        sensor_msgs::PointCloud2Iterator<float> bscan_iterX(bscan_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> bscan_iterY(bscan_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> bscan_iterZ(bscan_cloud_, "z");
        sensor_msgs::PointCloud2Iterator<float> bscan_iterAmps(bscan_cloud_, "Amplitudes");

        // Create iterators for gated B-scan
        sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterX(gated_bscan_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterY(gated_bscan_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterZ(gated_bscan_cloud_, "z");
        sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterAmps(gated_bscan_cloud_, "Amplitudes");
        sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterTof(gated_bscan_cloud_, "TimeofFlight");

        // Create iterators for gate top
        sensor_msgs::PointCloud2Iterator<float> gate_top_iterX(gate_top_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> gate_top_iterY(gate_top_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> gate_top_iterZ(gate_top_cloud_, "z");
        sensor_msgs::PointCloud2Iterator<float> gate_top_iterRGB(gate_top_cloud_, "rgb");
        sensor_msgs::PointCloud2Iterator<float> gate_top_iterAmps(gate_top_cloud_, "Amplitudes");

        // Create iterators for gate bottom
        sensor_msgs::PointCloud2Iterator<float> gate_bottom_iterX(gate_bottom_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> gate_bottom_iterY(gate_bottom_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> gate_bottom_iterZ(gate_bottom_cloud_, "z");
        sensor_msgs::PointCloud2Iterator<float> gate_bottom_iterRGB(gate_bottom_cloud_, "rgb");
        sensor_msgs::PointCloud2Iterator<float> gate_bottom_iterAmps(gate_bottom_cloud_, "Amplitudes");

        float nan_value = std::numeric_limits<float>::quiet_NaN();
        int element_i = 0;

        // FIRST PASS: Find maximum amplitude within the depth window (depth_to_skip to depth_to_skip + max_depth)
        float window_max_amplitude = 0.0f;
        
        for (const auto& ascan : obs_msg.ascans) {
            int t = 0;
            float n = obs_msg.vel_material;
            if (immersion_) {
                n = 1500.0f;
            }
            int i = 0;
            bool found_front_wall_temp = false;
            float depth_front_wall_temp = 0.0f;
            float dt = 1.0f / ((float)obs_msg.digitisation_rate * 1000000.0f);

            for (auto amplitude : ascan.amplitudes) {
                float z = (float)((float)i * (float)n * dt / 2.0f);
                
                if (immersion_ && n != 1500.0f) {
                    z = z + (float)((float)t * 1500.0f * dt / 2.0f) - (float)((float)t * (float)obs_msg.vel_material * dt / 2.0f);
                }
                
                // Detect front wall to know when to switch velocity (for depth calculation accuracy)
                float temp_normalized = (float)amplitude / (float)obs_msg.max_amplitude;
                if (!found_front_wall_temp && temp_normalized > gate_front_wall_) {
                    found_front_wall_temp = true;
                    depth_front_wall_temp = z;
                    if (immersion_) {
                        n = obs_msg.vel_material;
                        t = i;
                    }
                }
                
                // Check if this sample is within the depth window
                if (found_front_wall_temp) {
                    float z_relative = z - depth_front_wall_temp;
                    if (z_relative >= depth_to_skip_ && z_relative <= (depth_to_skip_ + max_depth_)) {
                        float abs_amplitude = std::abs((float)amplitude);
                        if (abs_amplitude > window_max_amplitude) {
                            window_max_amplitude = abs_amplitude;
                        }
                    }
                }
                
                ++i;
            }
        }
        
        // Use window max if found, otherwise fall back to observation max
        float normalization_factor = (window_max_amplitude > 0.0f) ? window_max_amplitude : (float)obs_msg.max_amplitude;
        
        ROS_INFO_THROTTLE(1.0, "Window max amplitude: %.2f, Observation max: %d, Using: %.2f", 
                          window_max_amplitude, obs_msg.max_amplitude, normalization_factor);

        // SECOND PASS: Process A-scans with window-based normalization
        element_i = 0;
        for (const auto& ascan : obs_msg.ascans) {
            float depth_front_wall = 0.0f;
            float depth_back_wall = 0.0f;
            float amp_front_wall = 0.0f;
            float amp_back_wall = 0.0f;
            bool found_front_wall = false;
            bool found_back_wall = false;
            float z_gates_top = nan_value;
            float z_gates_bottom = nan_value;

            int t = 0;
            float n = obs_msg.vel_material;  // Start with material velocity (like peak_nodelet)
            if (immersion_) {
                n = 1500.0f;  // Override to water velocity if immersion mode
            }
            int i = 0;
            
            // CRITICAL FIX: Calculate dt - convert digitisation rate (MHz) to seconds
            // This was the bug - missing the 1000000.0f conversion factor
            float dt = 1.0f / ((float)obs_msg.digitisation_rate * 1000000.0f);
            
            // DEBUG: Track velocity for this A-scan
            bool velocity_switched = false;
            int frontwall_sample = -1;

            for (auto amplitude : ascan.amplitudes) {
                // Calculate depth
                float z = (float)((float)i * (float)n * dt / 2.0f);
                
                // If in sample (immersion mode and velocity changed), adjust for water-sample transition
                if (immersion_ && n != 1500.0f){
                    z = z + (float)((float)t * 1500.0f * dt / 2.0f) - (float)((float)t * (float)obs_msg.vel_material * dt / 2.0f);
                    
                    // DEBUG: Log adjustment
                    if (!velocity_switched && element_i == 0) {
                        velocity_switched = true;
                    }
                }
                
                float tof = z;  // Time of flight equals depth for compatibility

                float x = 0.0f;  // Simplified - you may need to calculate actual x based on element geometry
                float y = (float)element_i * (float)obs_msg.element_pitch * 0.001f; // Convert mm to m

                // Apply TCG if enabled
                float normalised_amplitude;
                if (use_tcg_ && z > 0.0f) {
                    float amplitude_tcg = (float)amplitude * std::pow(10.0f, (amp_factor_ * (z / depth_factor_) / 20.0f));
                    
                    if (amplitude_tcg > normalization_factor * tcg_limit_) {
                        amplitude_tcg = normalization_factor * tcg_limit_;
                    } else if (amplitude_tcg < -normalization_factor * tcg_limit_) {
                        amplitude_tcg = -normalization_factor * tcg_limit_;
                    }
                    normalised_amplitude = amplitude_tcg / normalization_factor;
                } else {
                    normalised_amplitude = (float)amplitude / normalization_factor;
                }

                // Populate B-scan (all data)
                // Publish directly in ltpa frame - no transformation needed
                *bscan_iterX = x;
                *bscan_iterY = y;
                *bscan_iterZ = z;
                *bscan_iterAmps = normalised_amplitude;

                ++bscan_iterX;
                ++bscan_iterY;
                ++bscan_iterZ;
                ++bscan_iterAmps;

                // Gate processing
                float gated_x = nan_value;
                float gated_y = nan_value;
                float gated_z = nan_value;
                float gated_amplitude = nan_value;
                float gated_tof = nan_value;

                // Front wall gate
                if (!found_front_wall && normalised_amplitude > gate_front_wall_) {
                    depth_front_wall = z;

                    if (zero_to_front_wall_) {
                        z = 0.0f;
                    }

                    amp_front_wall = normalised_amplitude;
                    gated_amplitude = amp_front_wall;
                    found_front_wall = true;

                    if (immersion_) {
                        float old_n = n;
                        n = obs_msg.vel_material;
                        t = i;
                        frontwall_sample = i;
                        if (element_i == 0) {
                            ROS_INFO_ONCE("=== FRONTWALL DETECTED (element 0) ===");
                            ROS_INFO_ONCE("Switching velocity: %.1f -> %.1f m/s at sample %d", old_n, n, t);
                            ROS_INFO_ONCE("obs_msg.vel_material = %.1f m/s", obs_msg.vel_material);
                        }
                    }

                    if (show_front_wall_) {
                        gated_x = x;
                        gated_y = y;
                        gated_z = z;
                        gated_tof = tof;
                    }

                // Back wall gate
                } else if (found_front_wall &&
                          z < max_depth_ + depth_front_wall + depth_to_skip_ &&
                          z > (depth_to_skip_ + depth_front_wall) &&
                          normalised_amplitude > gate_back_wall_) {

                    depth_back_wall = z;

                    if (zero_to_front_wall_) {
                        z = z - depth_front_wall;
                    }

                    amp_back_wall = normalised_amplitude;
                    gated_amplitude = amp_back_wall;
                    gated_tof = depth_back_wall;

                    found_back_wall = true;

                    gated_x = x;
                    gated_y = y;
                    gated_z = z;
                }

                // Populate gated B-scan
                // Publish directly in ltpa frame - no transformation needed
                *gated_bscan_iterX = gated_x;
                *gated_bscan_iterY = gated_y;
                *gated_bscan_iterZ = gated_z;
                *gated_bscan_iterAmps = gated_amplitude;
                *gated_bscan_iterTof = gated_tof;

                ++gated_bscan_iterX;
                ++gated_bscan_iterY;
                ++gated_bscan_iterZ;
                ++gated_bscan_iterAmps;
                ++gated_bscan_iterTof;

                // Gate visualization
                if (found_front_wall) {
                    z_gates_top = depth_front_wall + depth_to_skip_;
                    z_gates_bottom = z_gates_top + max_depth_;
                }

                // Publish gate points directly in ltpa frame
                // Pack red color (255, 0, 0) into float for RViz
                uint8_t r = 255, g = 0, b = 0;
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                float rgb_float = *reinterpret_cast<float*>(&rgb);
                
                *gate_top_iterX = x;
                *gate_top_iterY = y;
                *gate_top_iterZ = z_gates_top;
                *gate_top_iterRGB = rgb_float;
                *gate_top_iterAmps = normalised_amplitude;

                ++gate_top_iterX;
                ++gate_top_iterY;
                ++gate_top_iterZ;
                ++gate_top_iterRGB;
                ++gate_top_iterAmps;


                *gate_bottom_iterX = x;
                *gate_bottom_iterY = y;
                *gate_bottom_iterZ = z_gates_bottom;
                *gate_bottom_iterRGB = rgb_float;
                *gate_bottom_iterAmps = normalised_amplitude;

                ++gate_bottom_iterX;
                ++gate_bottom_iterY;
                ++gate_bottom_iterZ;
                ++gate_bottom_iterRGB;
                ++gate_bottom_iterAmps;

                ++i;
            }

            ++element_i;
        }

        // ROS_INFO_THROTTLE(1.0, "Processed observation with %d A-scans, angle: %.2f deg, depth: %.4f m",
        //                   obs_msg.num_ascans, frontwall_angle, frontwall_depth);
    }

    void publishDepthMarker(float avg_depth, const ros::Time& stamp) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "frontwall_depth";
        marker.header.stamp = stamp;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << "Average Frontwall Depth: " << (avg_depth * 1000.0f) << " mm";
        marker.text = ss.str();

        marker.scale.z = 0.01;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.id = 0;
        marker.ns = "depth_measurement";

        depth_marker_pub_.publish(marker);
    }

    void publishAngleMarker(float frontwall_angle, const ros::Time& stamp) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "frontwall_angle";
        marker.header.stamp = stamp;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << "Frontwall Angle: " << frontwall_angle << " Degrees";
        marker.text = ss.str();

        marker.scale.z = 0.01;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.id = 1;
        marker.ns = "angle_measurement";

        angle_marker_pub_.publish(marker);
    }

    // Member variables
    std::string input_bag_path_;
    std::string output_bag_path_;

    // Parameters
    bool use_tcg_;
    float amp_factor_;
    float depth_factor_;
    float tcg_limit_;
    float gate_front_wall_;
    float depth_to_skip_;
    float gate_back_wall_;
    float max_depth_;
    bool zero_to_front_wall_;
    bool show_front_wall_;
    bool immersion_;
    int ransac_iterations_;
    float ransac_threshold_;

    // Publishers
    ros::Publisher bscan_pub_;
    ros::Publisher gated_bscan_pub_;
    ros::Publisher detections_pub_;  // Added detections publisher
    ros::Publisher gate_top_pub_;
    ros::Publisher gate_bottom_pub_;
    ros::Publisher depth_marker_pub_;
    ros::Publisher angle_marker_pub_;
    ros::Publisher tf_pub_;  // Publisher for TF messages from bag
    ros::Publisher joint_states_pub_;  // Publisher for joint states

    // Point clouds
    sensor_msgs::PointCloud2 bscan_cloud_;
    sensor_msgs::PointCloud2 gated_bscan_cloud_;
    sensor_msgs::PointCloud2 gate_top_cloud_;
    sensor_msgs::PointCloud2 gate_bottom_cloud_;
    sensor_msgs::PointCloud2 accumulated_detections_;  // Accumulated detections from all observations

    // TF - with longer cache time for bag playback
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ascan_postprocessor");

    AScanPostProcessor processor;
    processor.processBag();

    return 0;
}