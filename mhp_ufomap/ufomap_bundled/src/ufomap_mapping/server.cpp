/**
 * UFOMap Mapping
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap_mapping
 * License: BSD 3
 *
 * @Modified by: Heiko Renz, 2024, Institute of Control Theory and Systems Engineering, TU Dortmund University, Germany
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// UFO
#include <ufomap_bundled/UFOMapStamped.h>
#include <ufomap_bundled/ufomap_mapping/server.h>
#include <ufomap_bundled/ufomap_msgs/conversions.h>
#include <ufomap_bundled/ufomap_ros/conversions.h>

// STD
#include <chrono>
#include <future>
#include <numeric>
void calcInfoGainGPU(int num, int threads, ufo::map::OccupancyMapColor* map,
                     std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>>* start_points,
                     std::map<int, std::vector<ufo::map::Point3>>* endpoints, double max_range, Eigen::MatrixXd* result,
                     Eigen::Vector4d* time_metrics);
namespace ufomap_mapping {
Server::Server(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, int num_startpoints, int num_scaling_gridpoints)
    : nh_(nh), nh_priv_(nh_priv), tf_listener_(tf_buffer_), cs_(nh_priv), num_startpoints_random_(num_startpoints)
{
    // Set up map
    double resolution                = nh_priv_.param("resolution", 0.05);
    resolution_                      = resolution;
    ufo::map::DepthType depth_levels = nh_priv_.param("depth_levels", 16);

    information_distribution_ = nh_priv_.param("information_distribution", false);

    // Automatic pruning is disabled so we can work in multiple threads for subscribers,
    // services and publishers
    if (nh_priv_.param("color_map", false))
    {
        ROS_WARN_ONCE("Color map is enabled");
        map_.emplace<ufo::map::OccupancyMapColor>(resolution, depth_levels, false);
    }
    else
    {
        ROS_WARN_ONCE("Color map is disabled");
        map_.emplace<ufo::map::OccupancyMap>(resolution, depth_levels, false);
    }
    // Enable min/max change detection
    std::visit(
        [this](auto& map) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
            {
                map.enableMinMaxChangeDetection(true);
            }
        },
        map_);

    // Set up dynamic reconfigure server
    cs_.setCallback(boost::bind(&Server::configCallback, this, _1, _2));

    // Set up publisher
    info_pub_      = nh_priv_.advertise<diagnostic_msgs::DiagnosticStatus>("info", 10, false);
    arrow_pub_     = nh_priv_.advertise<visualization_msgs::Marker>("info_dist_points", 1, false);        // Plotting arrows for each ray
    start_pub_     = nh_priv_.advertise<visualization_msgs::Marker>("info_dist_points_start", 1, false);  // Plotting markers for each start point
    info_dist_pub_ = nh_priv_.advertise<ufomap_bundled::MsgInfoDist>("info_dist", 10, false);  //  Publishing information gains for start points
    info_dist_pub_cloud_ =
        nh_priv_.advertise<sensor_msgs::PointCloud2>("info_dist_cloud", 10, false);                //  Publishing information gains for start points
    gpu_time_pub_ = nh_priv_.advertise<std_msgs::Float64MultiArray>("info_gpu_times", 10, false);  //  Publishing information gains for start points
    // Enable services
    get_map_server_      = nh_priv_.advertiseService("get_map", &Server::getMapCallback, this);
    clear_volume_server_ = nh_priv_.advertiseService("clear_volume", &Server::clearVolumeCallback, this);
    reset_server_        = nh_priv_.advertiseService("reset", &Server::resetCallback, this);
    save_map_server_     = nh_priv_.advertiseService("save_map", &Server::saveMapCallback, this);

    scaling_factor_gridpoints_ = num_scaling_gridpoints;
}

void Server::cloudCallback(sensor_msgs::PointCloud2::ConstPtr const& msg)
{
    ufo::math::Pose6 transform;
    try
    {
        transform =
            ufomap_ros::rosToUfo(tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp, transform_timeout_).transform);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1, "%s", ex.what());
        return;
    }

    std::visit(
        [this, &msg, &transform](auto& map) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
            {
                auto start = std::chrono::steady_clock::now();

                // Update map
                ufo::map::PointCloudColor cloud;
                ufomap_ros::rosToUfo(*msg, cloud);
                cloud.transform(transform, true);

                map.insertPointCloudDiscrete(transform.translation(), cloud, max_range_, insert_depth_, simple_ray_casting_, early_stopping_, async_);

                double integration_time =
                    std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();

                if (0 == num_integrations_ || integration_time < min_integration_time_)
                {
                    min_integration_time_ = integration_time;
                }
                if (integration_time > max_integration_time_)
                {
                    max_integration_time_ = integration_time;
                }
                accumulated_integration_time_ += integration_time;
                ++num_integrations_;

                // Clear robot
                if (clear_robot_)
                {
                    start = std::chrono::steady_clock::now();

                    try
                    {
                        transform = ufomap_ros::rosToUfo(
                            tf_buffer_.lookupTransform(frame_id_, robot_frame_id_, msg->header.stamp, transform_timeout_).transform);
                    }
                    catch (tf2::TransformException& ex)
                    {
                        ROS_WARN_THROTTLE(1, "%s", ex.what());
                        return;
                    }

                    ufo::map::Point3 r(robot_radius_, robot_radius_, robot_height_ / 2.0);
                    ufo::geometry::AABB aabb(transform.translation() - r, transform.translation() + r);
                    map.setValueVolume(aabb, map.getClampingThresMin(), clearing_depth_);

                    double clear_time = std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();
                    if (0 == num_clears_ || clear_time < min_clear_time_)
                    {
                        min_clear_time_ = clear_time;
                    }
                    if (clear_time > max_clear_time_)
                    {
                        max_clear_time_ = clear_time;
                    }
                    accumulated_clear_time_ += clear_time;
                    ++num_clears_;
                }

                // Publish update
                if (!map_pub_.empty() && update_part_of_map_ && map.validMinMaxChange() &&
                    (!last_update_time_.isValid() || (msg->header.stamp - last_update_time_) >= update_rate_))
                {
                    bool can_update = true;
                    if (update_async_handler_.valid())
                    {
                        can_update = std::future_status::ready == update_async_handler_.wait_for(std::chrono::seconds(0));
                    }

                    if (can_update)
                    {
                        last_update_time_ = msg->header.stamp;
                        start             = std::chrono::steady_clock::now();

                        ufo::geometry::AABB aabb(map.minChange(), map.maxChange());
                        // TODO: should this be here?
                        map.resetMinMaxChangeDetection();

                        update_async_handler_ = std::async(std::launch::async, [this, aabb, stamp = msg->header.stamp]() {
                            std::visit(
                                [this, &aabb, stamp](auto& map) {
                                    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
                                    {
                                        for (int i = 0; i < map_pub_.size(); ++i)
                                        {
                                            if (map_pub_[i] && (0 < map_pub_[i].getNumSubscribers() || map_pub_[i].isLatched()))
                                            {
                                                ufomap_bundled::UFOMapStamped::Ptr msg(new ufomap_bundled::UFOMapStamped);
                                                if (ufomap_bundled::ufoToMsg(map, msg->map, aabb, compress_, i))
                                                {
                                                    msg->header.stamp    = stamp;
                                                    msg->header.frame_id = frame_id_;
                                                    map_pub_[i].publish(msg);
                                                }
                                            }
                                        }
                                    }
                                },
                                map_);
                        });

                        double update_time =
                            std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();
                        if (0 == num_updates_ || update_time < min_update_time_)
                        {
                            min_update_time_ = update_time;
                        }
                        if (update_time > max_update_time_)
                        {
                            max_update_time_ = update_time;
                        }
                        accumulated_update_time_ += update_time;
                        ++num_updates_;
                    }
                }

                // Information distribution
                // Idea: Sample different start points around a POI and check the perspective to build up an information distribution
                if (information_distribution_)
                {
                    ROS_WARN_ONCE("Information distribution is enabled");

                    double far_dist = 3.8;  // From Azure Kinect Specifications

                    // Define the POI (Point of Interest) in which direct environment we want to calculate the information distribution
                    ufo::map::Point3 poi_world(1.37, 0.16, 0.2);  // as world coordinate
                    auto start_time = std::chrono::steady_clock::now();
                    // define the start points around the POI from which we want to calculate the information distribution
                    if (!calculateStartPoints(poi_world, 1.0)) ROS_ERROR("UFOMap_server: Could not calculate start points");
                    // define the ray endpoints for each start point
                    if (!calculateRayEndpoints(far_dist)) ROS_ERROR("UFOMap_server: Could not calculate ray endpoints");

                    // Time measurement
                    double perspective_time =
                        std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time).count();

                    if (0 == num_perspectives_ || perspective_time < min_perspective_time_)
                    {
                        min_perspective_time_ = perspective_time;
                    }
                    if (perspective_time > max_perspective_time_)
                    {
                        max_perspective_time_ = perspective_time;
                    }
                    accumulated_perspective_time_ += perspective_time;
                    ++num_perspectives_;

                    start_time = std::chrono::steady_clock::now();

                    // CPU information distribution
                    if (false)
                    {

                        for (auto start_it = start_points_.begin(); start_it != start_points_.end(); start_it++)
                        {

                            std::tuple start = *start_it;
                            
                            // Calculate the information distribution for each perspective
                            if (!calculateInformationForPerspective(map, std::get<0>(start)))
                                ROS_ERROR("UFOMap_server: Could not calculate information distribution for perspective");

                        }
                    }
                    double distribution_time =
                        std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time).count();


                    if (0 == num_distributions_ || distribution_time < min_distribution_time_)
                    {
                        min_distribution_time_ = distribution_time;
                    }
                    if (distribution_time > max_distribution_time_)
                    {
                        max_distribution_time_ = distribution_time;
                    }
                    accumulated_distribution_time_ += distribution_time;
                    ++num_distributions_;

                    // GPU information distribution
                    if (true)
                    {
                        // Prepare output vector
                        info_metric_matrix_.resize(start_points_.size(), ray_endpoints_[0].size());

                        // Check if the map is a color map
                        if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMapColor>)
                        {
                            auto start_time = std::chrono::steady_clock::now();

                            ufo::map::OccupancyMapColor map_color(map);
                            // Calculate the information distribution for each perspective on the GPU
                            calcInfoGainGPU(ray_endpoints_[0].size(), 1024, &map_color, &start_points_, &ray_endpoints_, -1.0, &info_metric_matrix_,
                                            &time_metric_vec_);  // Max 1024 Threads per Block#

                            // Time measurement
                            double distribution_gpu_time =
                                std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time).count();

                            if (0 == num_distributions_gpu_ || distribution_gpu_time < min_distribution_gpu_time_)
                            {
                                min_distribution_gpu_time_ = distribution_gpu_time;
                            }
                            if (distribution_gpu_time > max_distribution_gpu_time_)
                            {
                                max_distribution_gpu_time_ = distribution_gpu_time;
                            }
                            accumulated_distribution_gpu_time_ += distribution_gpu_time;
                            ++num_distributions_gpu_;

                            // Publish the information distribution
                            // ufomap_bundled::MsgInfoDist msg;
                            sensor_msgs::PointCloud2 msg_cloud;
                            // toMessage(msg);
                            toMessage(msg_cloud);
                            // info_dist_pub_.publish(msg);
                            info_dist_pub_cloud_.publish(msg_cloud);

                            // Publish the GPU times
                            std_msgs::Float64MultiArray msg_gpu_times;
                            msg_gpu_times.data.resize(4);
                            msg_gpu_times.data[0] = time_metric_vec_[0];
                            msg_gpu_times.data[1] = time_metric_vec_[1];
                            msg_gpu_times.data[2] = time_metric_vec_[2];
                            msg_gpu_times.data[3] = time_metric_vec_[3];
                            gpu_time_pub_.publish(msg_gpu_times);
                        }
                        else
                        {
                            ROS_ERROR("UFOMap_server: Map is not a color map");
                        }
                    }
                }
                publishInfo();
            }
        },
        map_);
}

void Server::publishInfo()
{
    if (verbose_)
    {
        printf("\nTimings:\n");
        if (0 != num_integrations_)
        {
            printf("\tIntegration time (s): %5d %09.6f\t(%09.6f +- %09.6f)\n", num_integrations_, accumulated_integration_time_,
                   accumulated_integration_time_ / num_integrations_, max_integration_time_);
        }
        if (0 != num_clears_)
        {
            printf("\tClear time (s):       %5d %09.6f\t(%09.6f +- %09.6f)\n", num_clears_, accumulated_clear_time_,
                   accumulated_clear_time_ / num_clears_, max_clear_time_);
        }
        if (0 != num_updates_)
        {
            printf("\tUpdate time (s):      %5d %09.6f\t(%09.6f +- %09.6f)\n", num_updates_, accumulated_update_time_,
                   accumulated_update_time_ / num_updates_, max_update_time_);
        }
        if (0 != num_wholes_)
        {
            printf("\tWhole time (s):       %5d %09.6f\t(%09.6f +- %09.6f)\n", num_wholes_, accumulated_whole_time_,
                   accumulated_whole_time_ / num_wholes_, max_whole_time_);
        }
    }

    if (info_pub_ && 0 < info_pub_.getNumSubscribers())
    {
        diagnostic_msgs::DiagnosticStatus msg;
        msg.level = diagnostic_msgs::DiagnosticStatus::OK;
        msg.name  = "UFOMap mapping timings";
        msg.values.resize(21);
        msg.values[0].key    = "Min integration time (s)";
        msg.values[0].value  = std::to_string(min_integration_time_);
        msg.values[1].key    = "Max integration time (s)";
        msg.values[1].value  = std::to_string(max_integration_time_);
        msg.values[2].key    = "Average integration time (s)";
        msg.values[2].value  = std::to_string(accumulated_integration_time_ / num_integrations_);
        msg.values[3].key    = "Min clear time (s)";
        msg.values[3].value  = std::to_string(min_clear_time_);
        msg.values[4].key    = "Max clear time (s)";
        msg.values[4].value  = std::to_string(max_clear_time_);
        msg.values[5].key    = "Average clear time (s)";
        msg.values[5].value  = std::to_string(accumulated_clear_time_ / num_clears_);
        msg.values[6].key    = "Min update time (s)";
        msg.values[6].value  = std::to_string(min_update_time_);
        msg.values[7].key    = "Max update time (s)";
        msg.values[7].value  = std::to_string(max_update_time_);
        msg.values[8].key    = "Average update time (s)";
        msg.values[8].value  = std::to_string(accumulated_update_time_ / num_updates_);
        msg.values[9].key    = "Min distribution time (s)";
        msg.values[9].value  = std::to_string(min_distribution_time_);
        msg.values[10].key   = "Max distribution time (s)";
        msg.values[10].value = std::to_string(max_distribution_time_);
        msg.values[11].key   = "Average distribution time (s)";
        msg.values[11].value = std::to_string(accumulated_distribution_time_ / num_distributions_);
        msg.values[12].key   = "Min whole time (s)";
        msg.values[12].value = std::to_string(min_whole_time_);
        msg.values[13].key   = "Max whole time (s)";
        msg.values[13].value = std::to_string(max_whole_time_);
        msg.values[14].key   = "Average whole time (s)";
        msg.values[14].value = std::to_string(accumulated_whole_time_ / num_wholes_);
        msg.values[15].key   = "Min perspective time (s)";
        msg.values[15].value = std::to_string(min_perspective_time_);
        msg.values[16].key   = "Max perspective time (s)";
        msg.values[16].value = std::to_string(max_perspective_time_);
        msg.values[17].key   = "Average perspective time (s)";
        msg.values[17].value = std::to_string(accumulated_perspective_time_ / num_perspectives_);
        msg.values[18].key   = "Min distribution GPU time (s)";
        msg.values[18].value = std::to_string(min_distribution_gpu_time_);
        msg.values[19].key   = "Max distribution GPU time (s)";
        msg.values[19].value = std::to_string(max_distribution_gpu_time_);
        msg.values[20].key   = "Average distribution GPU time (s)";
        msg.values[20].value = std::to_string(accumulated_distribution_gpu_time_ / num_distributions_gpu_);
        info_pub_.publish(msg);
    }
}

void Server::mapConnectCallback(ros::SingleSubscriberPublisher const& pub, int depth)
{
    // When a new node subscribes we will publish the whole map to that node.

    // TODO: Make this async

    std::visit(
        [this, &pub, depth](auto& map) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
            {
                auto start = std::chrono::steady_clock::now();

                ufomap_bundled::UFOMapStamped::Ptr msg(new ufomap_bundled::UFOMapStamped);
                if (ufomap_bundled::ufoToMsg(map, msg->map, compress_, depth))
                {
                    msg->header.stamp    = ros::Time::now();
                    msg->header.frame_id = frame_id_;
                    pub.publish(msg);
                }

                double whole_time = std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();
                if (0 == num_wholes_ || whole_time < min_whole_time_)
                {
                    min_whole_time_ = whole_time;
                }
                if (whole_time > max_whole_time_)
                {
                    max_whole_time_ = whole_time;
                }
                accumulated_whole_time_ += whole_time;
                ++num_wholes_;
            }
        },
        map_);
}

bool Server::getMapCallback(ufomap_bundled::GetMap::Request& request, ufomap_bundled::GetMap::Response& response)
{
    std::visit(
        [this, &request, &response](auto& map) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
            {
                ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
                response.success                 = ufomap_bundled::ufoToMsg(map, response.map, bv, request.compress, request.depth);
            }
            else
            {
                response.success = false;
            }
        },
        map_);
    return true;
}

bool Server::clearVolumeCallback(ufomap_bundled::ClearVolume::Request& request, ufomap_bundled::ClearVolume::Response& response)
{
    std::visit(
        [this, &request, &response](auto& map) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
            {
                ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
                for (auto& b : bv)
                {
                    map.setValueVolume(b, map.getClampingThresMin(), request.depth);
                }
                response.success = true;
            }
            else
            {
                response.success = false;
            }
        },
        map_);
    return true;
}

bool Server::resetCallback(ufomap_bundled::Reset::Request& request, ufomap_bundled::Reset::Response& response)
{
    std::visit(
        [this, &request, &response](auto& map) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
            {
                map.clear(request.new_resolution, request.new_depth_levels);
                response.success = true;
            }
            else
            {
                response.success = false;
            }
        },
        map_);
    return true;
}

bool Server::saveMapCallback(ufomap_bundled::SaveMap::Request& request, ufomap_bundled::SaveMap::Response& response)
{
    std::visit(
        [this, &request, &response](auto& map) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
            {
                ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
                response.success                 = map.write(request.filename, bv, request.compress, request.depth, 1, request.compression_level);
            }
            else
            {
                response.success = false;
            }
        },
        map_);
    return true;
}

void Server::timerCallback(ros::TimerEvent const& event)
{
    std_msgs::Header header;
    header.stamp    = ros::Time::now();
    header.frame_id = frame_id_;

    if (!map_pub_.empty())
    {
        for (int i = 0; i < map_pub_.size(); ++i)
        {
            if (map_pub_[i] && (0 < map_pub_[i].getNumSubscribers() || map_pub_[i].isLatched()))
            {
                std::visit(
                    [this, &header, i](auto& map) {
                        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
                        {
                            auto start = std::chrono::steady_clock::now();

                            ufomap_bundled::UFOMapStamped::Ptr msg(new ufomap_bundled::UFOMapStamped);
                            if (ufomap_bundled::ufoToMsg(map, msg->map, compress_, i))
                            {
                                msg->header = header;
                                map_pub_[i].publish(msg);
                            }

                            double whole_time =
                                std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();
                            if (0 == num_wholes_ || whole_time < min_whole_time_)
                            {
                                min_whole_time_ = whole_time;
                            }
                            if (whole_time > max_whole_time_)
                            {
                                max_whole_time_ = whole_time;
                            }
                            accumulated_whole_time_ += whole_time;
                            ++num_wholes_;
                        }
                    },
                    map_);
            }
        }
    }
    publishInfo();
}

void Server::configCallback(ufomap_bundled::ServerConfig& config, uint32_t level)
{
    // Read parameters
    frame_id_ = config.frame_id;

    verbose_ = config.verbose;

    max_range_          = config.max_range;
    insert_depth_       = config.insert_depth;
    simple_ray_casting_ = config.simple_ray_casting;
    early_stopping_     = config.early_stopping;
    async_              = config.async;

    clear_robot_    = config.clear_robot;
    robot_frame_id_ = config.robot_frame_id;
    robot_height_   = config.robot_height;
    robot_radius_   = config.robot_radius;
    clearing_depth_ = config.clearing_depth;

    compress_           = config.compress;
    update_part_of_map_ = config.update_part_of_map;
    publish_depth_      = config.publish_depth;

    // scaling_factor_gridpoints_ = config.scaling_factor_gridpoints;

    std::visit(
        [this, &config](auto& map) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
            {
                map.setProbHit(config.prob_hit);
                map.setProbMiss(config.prob_miss);
                map.setClampingThresMin(config.clamping_thres_min);
                map.setClampingThresMax(config.clamping_thres_max);
            }
        },
        map_);

    transform_timeout_.fromSec(config.transform_timeout);

    // Set up publisher
    if (map_pub_.empty() || map_pub_[0].isLatched() != config.map_latch || map_queue_size_ != config.map_queue_size)
    {
        map_pub_.resize(publish_depth_ + 1);
        for (int i = 0; i < map_pub_.size(); ++i)
        {
            map_queue_size_         = config.map_queue_size;
            std::string final_topic = i == 0 ? "map" : "map_depth_" + std::to_string(i);
            map_pub_[i] =
                nh_priv_.advertise<ufomap_bundled::UFOMapStamped>(final_topic, map_queue_size_, boost::bind(&Server::mapConnectCallback, this, _1, i),
                                                                  ros::SubscriberStatusCallback(), ros::VoidConstPtr(), config.map_latch);
        }
    }

    // Set up subscriber
    if (!cloud_sub_ || cloud_in_queue_size_ != config.cloud_in_queue_size)
    {
        cloud_in_queue_size_ = config.cloud_in_queue_size;
        cloud_sub_           = nh_.subscribe("cloud_in", cloud_in_queue_size_, &Server::cloudCallback, this);
    }
    save_map_sub_ = nh_.subscribe("/save_map_topic", 10, &Server::saveMapTopicCallback, this);

    // Set up timer
    if (!pub_timer_ || pub_rate_ != config.pub_rate)
    {
        pub_rate_ = config.pub_rate;
        if (0 < pub_rate_)
        {
            pub_timer_ = nh_priv_.createTimer(ros::Rate(pub_rate_), &Server::timerCallback, this);
        }
        else
        {
            pub_timer_.stop();
        }
    }

    // Set up update rate
    if (config.update_rate != 0)
    {
        update_rate_ = ros::Duration(1.0 / config.update_rate);
    }
    else
    {
        update_rate_ = ros::Duration(0.0);
    }
}

void Server::plotArrow(const ufo::map::Point3& start, const ufo::map::Point3& end, const int id, bool mark_start, bool blue) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp    = ros::Time::now();
    marker.ns              = "info_dist_points";
    marker.id              = id;
    marker.type            = visualization_msgs::Marker::ARROW;
    marker.action          = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start_p, end_p;
    start_p.x      = start.x();
    start_p.y      = start.y();
    start_p.z      = start.z();
    end_p.x        = end.x();
    end_p.y        = end.y();
    end_p.z        = end.z();
    marker.scale.x = marker.scale.x = marker.scale.y = 0.01;
    if (blue)
    {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
    }
    else
    {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    }
    marker.color.a = 0.5;
    marker.points.push_back(start_p);
    marker.points.push_back(end_p);
    arrow_pub_.publish(marker);
}

bool Server::calculateRayEndpoints(const double far_dist)
{
    int plot_one = 0;
    ray_endpoints_.clear();
    for (auto start_it = start_points_.begin(); start_it != start_points_.end(); start_it++)
    {

        // Define the start point for the information distribution
        std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>& start = *start_it;
        // Define the ray endpoints for the information distribution in the current
        // perspective (all endpoints on the far plane of the depth camera
        ufo::map::Point3 end(std::get<1>(start).x() + std::get<2>(start).axis()[0] * far_dist,
                             std::get<1>(start).y() + std::get<2>(start).axis()[1] * far_dist,
                             std::get<1>(start).z() + std::get<2>(start).axis()[2] * far_dist);

        // Calculate the frustrum points (fixed since we assume a fixed camera perspective along the z-axis)
        double v_foi = 65 * M_PI / 180;  // From Azure Kinect Specifications
        double h_foi = 75 * M_PI / 180;  // From Azure Kinect Specifications
        double height = tan(v_foi / 2) * far_dist;
        double width  = tan(h_foi / 2) * far_dist;

        frustrum_points_.clear();
        frustrum_points_.insert({"end", end});
        frustrum_points_.insert({"start", std::get<1>(start)});
        frustrum_points_.insert({"up_left", ufo::map::Point3(end(0) + width, end(1) - height, end(2))});
        frustrum_points_.insert({"up_right", ufo::map::Point3(end(0) + width, end(1) + height, end(2))});
        frustrum_points_.insert({"down_left", ufo::map::Point3(end(0) - width, end(1) - height, end(2))});
        frustrum_points_.insert({"down_right", ufo::map::Point3(end(0) - width, end(1) + height, end(2))});

        if (arrow_cnt_ > 1000000)
        {
            arrow_cnt_ = 0;
        }

        if (plot_arrows_ && plot_one == 10)
        {
            plotArrow(std::get<1>(start), end, arrow_cnt_, true);
            arrow_cnt_++;

            plotArrow(std::get<1>(start), frustrum_points_.at("up_left"), arrow_cnt_, true, true);
            arrow_cnt_++;

            plotArrow(std::get<1>(start), frustrum_points_.at("up_right"), arrow_cnt_, true, true);
            arrow_cnt_++;

            plotArrow(std::get<1>(start), frustrum_points_.at("down_left"), arrow_cnt_, true, true);
            arrow_cnt_++;

            plotArrow(std::get<1>(start), frustrum_points_.at("down_right"), arrow_cnt_, true, true);
            arrow_cnt_++;
        }
        // Variant 1: Only the center of the far plane
        // Variant 2: The four corners of the far plane + the center of the far plane
        // Variant 3: Full grid of endpoints on the far plane (using thk octomap
        // resolution of the smallest possible node size with a scaling factor of 100)
        std::vector<ufo::map::Point3> ray_endpoints;
        switch (gridpoint_mode_)
        {
            case CENTER:
                ray_endpoints.resize(1);
                ray_endpoints.at(0) = end;
                break;
            case FRUSTRUM:
                ray_endpoints.resize(5);
                ray_endpoints.at(0) = end;
                ray_endpoints.at(1) = frustrum_points_.at("up_left");
                ray_endpoints.at(2) = frustrum_points_.at("up_right");
                ray_endpoints.at(3) = frustrum_points_.at("down_left");
                ray_endpoints.at(4) = frustrum_points_.at("down_right");
                break;
            case FULL:
                // Calculate the number of gridpoints in the x and y direction
                double scale_res = resolution_ * scaling_factor_gridpoints_;
                int num_x        = (int)(2 * width / scale_res);
                int num_y        = (int)(2 * height / scale_res);

                // Resize the vector of ray endpoints
                ray_endpoints.clear();
                ray_endpoints.resize((num_x + 1) * (num_y + 1));
                // Iterate over the gridpoints
                for (int i = 0; i <= num_x; i++)
                {
                    for (int j = 0; j <= num_y; j++)
                    {
                        if ((i == 0 && j == 0) || (i == num_x && j == num_y)) continue;
                        // Calculate the current ray endpoint
                        ufo::map::Point3 stepUp;
                        ufo::map::Point3 stepRight;
                        stepUp    = (frustrum_points_.at("up_left") - frustrum_points_.at("down_left")) / num_y;
                        stepRight = (frustrum_points_.at("down_right") - frustrum_points_.at("down_left")) / num_x;
                        ufo::map::Point3 ray_endpoint;
                        ray_endpoint                    = frustrum_points_.at("down_left") + stepUp * j + stepRight * i;
                        ray_endpoints.at(i * num_y + j) = ray_endpoint;

                        // Plot the current ray endpoint (only perspective 10 due to visibility)
                        if (plot_arrows_ && plot_one == 10)
                        {
                            if (arrow_cnt_ > 1000000)
                            {
                                arrow_cnt_ = 0;
                            }
                            plotArrow(std::get<1>(start), ray_endpoints.at(i * num_y + j), arrow_cnt_);
                            arrow_cnt_++;
                        }
                    }
                }
                break;
        }
        ray_endpoints_.insert({std::get<0>(start), ray_endpoints});
        plot_one++;
    }
    return true;
}

bool Server::calculateInformationForPerspective(auto& map, int start_point_id)
{
    accumulated_info_val_normalized_ = 0.0;
    for (auto endpoint_it = ray_endpoints_[start_point_id].begin(); endpoint_it != ray_endpoints_[start_point_id].end(); endpoint_it++)
    {
        double info_val = 0;
        // Get the keys of all the nodes on the ray
        ufo::map::CodeRay allNodesOnCast = map.computeRay(std::get<1>(start_points_.at(start_point_id)), *endpoint_it);

        // Iterate over the ufomap nodes on the ray
        int cnt       = 0;
        bool occupied = false;
        for (auto it = allNodesOnCast.begin(); it != allNodesOnCast.end(); it++)
        {
            // Search for the node in the octree

            if (!occupied)
            {
                cnt++;
                ufo::map::OccupancyState state = map.getState(*it);
                switch (state)
                {
                    case ufo::map::OccupancyState::occupied:  // If the node is occupied, we
                                                              // want to know less about it
                                                              // --> Onyl small increae
                        info_val = info_val + (1 - map.getOccupancy(*it));
                        occupied = true;
                        break;
                    case ufo::map::OccupancyState::free:  // If the node is free,  we
                                                          // want to know less about it
                                                          // --> Onyl small increae
                        info_val = info_val + (map.getOccupancy(*it));
                        break;
                    case ufo::map::OccupancyState::unknown:  // If the node is unknown -->
                                                             // We want to know more
                        info_val++;
                        break;
                }
            }
        }

        if (cnt > 0) accumulated_info_val_normalized_ += (info_val / cnt);
    }
    return true;
}

bool Server::calculateStartPoints(const ufo::map::Point3& poi, double distance)
{
    // Calculate the number of start points
    // cubical grid of start points around the poi with size distance
    // currently only the 8 corner points
    switch (startpoint_mode_)
    {
        case CORNER: {  // Case as explicit code block
            int num_start_points_corner = 8;
            // Resize the vector of start points
            start_points_.resize(num_start_points_corner);
            // Generate corner points with orientation towards POI
            for (int i = 0; i < num_start_points_corner; i++)
            {
                ufo::map::Point3 start_point =
                    ufo::map::Point3(poi(0) + distance / 2 * (i % 2 == 0 ? 1 : -1), poi(1) + distance / 2 * ((i / 2) % 2 == 0 ? 1 : -1),
                                     poi(2) + distance / 2 * ((i / 4) % 2 == 0 ? 1 : -1));
                Eigen::AngleAxisd axis(0,
                                       Eigen::Vector3d{poi.x() - start_point.x(), poi.y() - start_point.y(), poi.z() - start_point.z()}.normalized());
                start_points_.at(i) = std::make_tuple(i, start_point, axis);
            }
            break;
        }
        case RANDOM: {
            // Resize the vector of start points
            start_points_.resize(num_startpoints_random_);

            // Define the area around the POI in which we want to generate random start points based on the distance input
            // Eigen::Rand::P8_mt19937_64 urng_{42};  // Using same seed to get same startpoints each cycle --> if you want changing startpoints comment here and uncomment in header file

            // Muller Method for sampling uniformly distributed points on a sphere and transform them into the sphere
            Eigen::MatrixXf randX(num_startpoints_random_, 3);
            randX = Eigen::Rand::normal<Eigen::MatrixXf>(num_startpoints_random_, 3, urng_);
            Eigen::VectorXf randU(num_startpoints_random_);
            randU = Eigen::Rand::uniformReal<Eigen::VectorXf>(num_startpoints_random_, 1, urng_);
            randU = randU.array().pow(1.0 / 3.0) * (distance);

            Eigen::MatrixXf randX_norm = (randX.rowwise().normalized().array().colwise() * randU.array()).matrix();
            for (int i = 0; i < num_startpoints_random_; i++)
            {
                ufo::map::Point3 start_point = ufo::map::Point3(randX_norm(i, 0) + poi(0), randX_norm(i, 1) + poi(1), randX_norm(i, 2) + poi(2));
                Eigen::AngleAxisd axis(0,
                                       Eigen::Vector3d{poi.x() - start_point.x(), poi.y() - start_point.y(), poi.z() - start_point.z()}.normalized());
                start_points_.at(i) = std::make_tuple(i, start_point, axis);
            }
            break;
        }
        default:
            ROS_ERROR("UFOMap_server: Startpoint mode not implemented");
            return false;
    }
    if (plot_startpoints_)
    {
        plotStartPoints();
    }
    return true;
}

void Server::saveMapTopicCallback(std_msgs::Bool::ConstPtr const& msg)
{
    if (msg->data && !saved_)
    {
        std::visit(
            [this](auto& map) {
                if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
                {
                    std::string map_filename;
                    nh_.param<std::string>("map_filename", map_filename, "noName.um");
                    map.write(map_filename);
                    saved_ = true;
                    ROS_INFO("Map saved");
                }
            },
            map_);
    }
}

void Server::plotStartPoints() const
{
    // Plot the start point
    visualization_msgs::Marker marker_start;
    marker_start.header.frame_id = frame_id_;
    marker_start.header.stamp    = ros::Time::now();
    marker_start.ns              = "info_dist_points_start";
    marker_start.id              = 0;
    marker_start.type            = visualization_msgs::Marker::POINTS;
    marker_start.action          = visualization_msgs::Marker::ADD;

    for (auto start_it = start_points_.begin(); start_it != start_points_.end(); start_it++)
    {
        geometry_msgs::Point p;
        p.x = std::get<1>(*start_it).x();
        p.y = std::get<1>(*start_it).y();
        p.z = std::get<1>(*start_it).z();
        marker_start.points.push_back(p);
    }
    marker_start.scale.x = marker_start.scale.z = marker_start.scale.y = 0.02;
    marker_start.color.r                                               = 0.0f;
    marker_start.color.g                                               = 1.0f;
    marker_start.color.b                                               = 0.0f;
    marker_start.color.a                                               = 1.0;
    start_pub_.publish(marker_start);
}

void Server::toMessage(ufomap_bundled::MsgInfoDist& msg) const
{
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = frame_id_;

    msg.points.resize(start_points_.size());
    msg.info_gains.resize(start_points_.size());
    for (int i = 0; i < start_points_.size(); i++)
    {
        msg.points[i].x   = std::get<1>(start_points_[i]).x();
        msg.points[i].y   = std::get<1>(start_points_[i]).y();
        msg.points[i].z   = std::get<1>(start_points_[i]).z();
        msg.info_gains[i] = info_metric_matrix_.row(i).mean();
    }
}

void Server::toMessage(sensor_msgs::PointCloud2& msg) const
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.resize(start_points_.size());
    for (int i = 0; i < start_points_.size(); i++)
    {
        float intensity = info_metric_matrix_.row(i).mean();
        pcl::PointXYZI point;
        point.x         = std::get<1>(start_points_[i]).x();
        point.y         = std::get<1>(start_points_[i]).y();
        point.z         = std::get<1>(start_points_[i]).z();
        point.intensity = intensity;
        cloud.at(i)     = point;
    }

    pcl::toROSMsg(cloud, msg);
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = frame_id_;
}

}  // namespace ufomap_mapping
