#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/conversions.h"
#include <laszip/laszip_api.h>
#include <fstream> 
#include <iostream>
#include <string>
#include <thread> 
#include <filesystem>
#include <nlohmann/json.hpp>
#include "rosbag2_cpp/reader.hpp"
#include "rclcpp/serialization.hpp"

struct Point3Di
{
	Eigen::Vector3d point;
	double timestamp;
    float intensity;
    int index_pose;
    uint8_t lidarid;
	int index_point;
};

struct TrajectoryPose
{
    double timestamp_ns;
    double x_m;
    double y_m;
    double z_m;
    double qw;
    double qx;
    double qy;
    double qz;
    Eigen::Affine3d pose;
};

namespace fs = std::filesystem;
std::vector<Point3Di> points_global;

std::vector<TrajectoryPose> trajectory;
std::vector<std::vector<TrajectoryPose>> chunks_trajectory;

bool saveLaz(const std::string &filename, const std::vector<Point3Di> &points_global)
{

    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    double max_x{std::numeric_limits<double>::lowest()};
    double max_y{std::numeric_limits<double>::lowest()};
    double max_z{std::numeric_limits<double>::lowest()};
    double min_x = 1000000000000.0;
    double min_y = 1000000000000.0;
    double min_z = 1000000000000.0;

    for (auto &p : points_global)
    {
        if (p.point.x() < min_x)
        {
            min_x = p.point.x();
        }
        if (p.point.x() > max_x)
        {
            max_x = p.point.x();
        }

        if (p.point.y() < min_y)
        {
            min_y = p.point.y();
        }
        if (p.point.y() > max_y)
        {
            max_y = p.point.y();
        }

        if (p.point.z() < min_z)
        {
            min_z = p.point.z();
        }
        if (p.point.z() > max_z)
        {
            max_z = p.point.z();
        }
    }

    std::cout << "processing: " << filename << "points " << points_global.size() << std::endl;

    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: creating laszip writer\n");
        return false;
    }

    // get a pointer to the header of the writer so we can populate it

    laszip_header *header;

    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
        return false;
    }

    // populate the header

    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0); // see LAS specification for details
    header->version_major = 1;
    header->version_minor = 2;
    //    header->file_creation_day = 120;
    //    header->file_creation_year = 2013;
    header->point_data_format = 1;
    header->point_data_record_length = 0;
    header->number_of_point_records = points_global.size();
    header->number_of_points_by_return[0] = points_global.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length = 28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = max_x;
    header->min_x = min_x;
    header->max_y = max_y;
    header->min_y = min_y;
    header->max_z = max_z;
    header->min_z = min_z;

    // optional: use the bounding box and the scale factor to create a "good" offset
    // open the writer
    laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);

    if (laszip_open_writer(laszip_writer, filename.c_str(), compress))
    {
        fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
        return false;
    }

    fprintf(stderr, "writing file '%s' %scompressed\n", filename.c_str(), (compress ? "" : "un"));

    // get a pointer to the point of the writer that we will populate and write

    laszip_point *point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i = 0; i < points_global.size(); i++)
    {
        const auto &p = points_global[i];
        point->intensity = p.intensity;
        point->gps_time = p.timestamp * 1e9;
        p_count++;
        coordinates[0] = p.point.x();
        coordinates[1] = p.point.y();
        coordinates[2] = p.point.z();
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }

        if (laszip_write_point(laszip_writer))
        {
            fprintf(stderr, "DLL ERROR: writing point %I64d\n", p_count);
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr, "DLL ERROR: getting point count\n");
        return false;
    }

    fprintf(stderr, "successfully written %I64d points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: closing laszip writer\n");
        return false;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
        return false;
    }

    std::cout << "exportLaz DONE" << std::endl;
    return true;
}

bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        std::cout << "can not save file: '" << file_name << "'" << std::endl;
        std::cout << "if You can see only '' it means there is no filename assigned to poses, please read manual or contact me januszbedkowski@gmail.com" << std::endl;
        std::cout << "To assign filename to poses please use following two buttons in multi_view_tls_registration_step_2" << std::endl;
        std::cout << "1: update initial poses from RESSO file" << std::endl;
        std::cout << "2: update poses from RESSO file" << std::endl;
        return false;
    }

    outfile << m_poses.size() << std::endl;
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        outfile << filenames[i] << std::endl;
        outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
        outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
        outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
        outfile << "0 0 0 1" << std::endl;
    }
    outfile.close();

    return true;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_bag> <output_directory>" << std::endl;
        return 1;
    }
    
    const std::string input_bag = argv[1];
    const std::string output_directory = argv[2];
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializationPointCloud2;
    rclcpp::Serialization<nav_msgs::msg::Odometry> serializationOdom;

    std::cout << "Processing bag: " << input_bag << std::endl;
    rosbag2_cpp::Reader bag;
  
    bag.open(input_bag);
  
    while (bag.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = bag.read_next();

        if (msg->topic_name == "/kiss/local_map") {
            RCLCPP_INFO(rclcpp::get_logger("KissFrame"), "Received message on topic: /kiss/local_map");
        
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        
            serializationPointCloud2.deserialize_message(&serialized_msg, cloud_msg.get());
        
            if (!cloud_msg || cloud_msg->data.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("KissFrame"), "Error: Empty PointCloud2 message!");
                return 1;
            }
    
            pcl::PointCloud<pcl::PointXYZ> cloud;
            size_t num_points = cloud_msg->width * cloud_msg->height;  // Suma punktów
            uint8_t* data_ptr = cloud_msg->data.data();
    
            RCLCPP_INFO(rclcpp::get_logger("KissFrame"), "Processing %zu points", num_points);
            
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

            for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z)
            {
                pcl::PointXYZ point;
                point.x = *iter_x;
                point.y = *iter_y;
                point.z = *iter_z;
            
                cloud.points.push_back(point);
            
                Point3Di point_global;
            
                if (cloud_msg->header.stamp.sec != 0 || cloud_msg->header.stamp.nanosec != 0)
                {
                    uint64_t sec_in_ms = static_cast<uint64_t>(cloud_msg->header.stamp.sec) * 1000ULL;
                    uint64_t ns_in_ms = static_cast<uint64_t>(cloud_msg->header.stamp.nanosec) / 1'000'000ULL;
                    point_global.timestamp = sec_in_ms + ns_in_ms;
                }
            
                point_global.point = Eigen::Vector3d(point.x, point.y, point.z);
                point_global.intensity = 0;  
                point_global.index_pose = static_cast<int>(i);
                point_global.lidarid = 0;
                point_global.index_point = static_cast<int>(i);
            
                points_global.push_back(point_global);
            }
            

            RCLCPP_INFO(rclcpp::get_logger("KissFrame"), "Processed %zu points!", cloud.points.size());
        }
        
        if (msg->topic_name == "/kiss/odometry") {
            RCLCPP_INFO(rclcpp::get_logger("KissOdometry"), "Received message on topic: /kiss/odometry");
        
            auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            serializationOdom.deserialize_message(&serialized_msg, odom_msg.get());
        
            if (!odom_msg) {
                RCLCPP_ERROR(rclcpp::get_logger("KissOdometry"), "Odometry message deserialization error!");
                return 1;
            }
        
            double x = odom_msg->pose.pose.position.x;
            double y = odom_msg->pose.pose.position.y;
            double z = odom_msg->pose.pose.position.z;
        
            double qx = odom_msg->pose.pose.orientation.x;
            double qy = odom_msg->pose.pose.orientation.y;
            double qz = odom_msg->pose.pose.orientation.z;
            double qw = odom_msg->pose.pose.orientation.w;
        
            TrajectoryPose pose;
            
            uint64_t sec_in_ms = static_cast<uint64_t>(odom_msg->header.stamp.sec) * 1000ULL;
            uint64_t ns_in_ms = static_cast<uint64_t>(odom_msg->header.stamp.nanosec) / 1'000'000ULL;
            pose.timestamp_ns = sec_in_ms + ns_in_ms;
        
            pose.x_m = x;
            pose.y_m = y;
            pose.z_m = z;
            pose.qw = qw;
            pose.qx = qx;
            pose.qy = qy;
            pose.qz = qz;

            pose.pose = Eigen::Affine3d::Identity();
            
            Eigen::Vector3d trans(x, y, z);
            Eigen::Quaterniond q(qw, qx, qy, qz);

            
            pose.pose.translation() = trans;
            pose.pose.linear() = q.toRotationMatrix();

            trajectory.push_back(pose);
        
            RCLCPP_INFO(rclcpp::get_logger("KissOdometry"), "Added position to trajectory: x=%.3f, y=%.3f, z=%.3f", x, y, z);
        }
    }

    // std::cout << "start transforming point to global coordinate system" << std::endl;

    // for(int i = 0; i < points_global.size(); i++){
    //     //auto lower = std::lower_bound(trajectory.begin(), trajectory.end(), points_global[i].timestamp);
    //     //auto index_pose = std::distance(trajectory.begin(), lower);

    //     if(i % 1000000 == 0){
    //         std::cout << "computed: " << i << " of " << points_global.size() << std::endl;
    //     }
        
    //     auto lower = std::lower_bound(trajectory.begin(), trajectory.end(), points_global[i].timestamp,
    //                                           [](TrajectoryPose lhs, double rhs) -> bool
    //                                           { return lhs.timestamp_ns < rhs; });


    //                                           //std::vector<TrajectoryPose>
    //     int index_pose = std::distance(trajectory.begin(), lower) - 1;
    //     if (index_pose >= 0 && index_pose < trajectory.size())
    //     {
    //         points_global[i].point = trajectory[index_pose].pose * points_global[i].point;
    //     }
    // }
    
                   
    // std::cout << "transforming point to global coordinate system finished" << std::endl;
    std::cout << "start loading pc" << std::endl;
    std::cout << "loading pc finished" << std::endl;

    std::vector<std::vector<Point3Di>> chunks_pc;

    int counter = 0;
    std::vector<Point3Di> chunk;

    for (int i = 0; i < points_global.size(); i++)
    {
        chunk.push_back(points_global[i]);

        if (chunk.size() > 2000000)
        {
            counter++;
            chunks_pc.push_back(chunk);
            chunk.clear();
            std::cout << "adding chunk [" << counter << "]" << std::endl;
        }
    }

    // remaining pc
    std::cout << "reamaining points: " << chunk.size() << std::endl;

    if (chunk.size() > 1000000)
    {
        chunks_pc.push_back(chunk);
    }

    std::cout << "cleaning points" << std::endl;
    points_global.clear();
    std::cout << "points cleaned" << std::endl;

    /////////////////////////
    std::cout << "start indexing chunks_trajectory" << std::endl;
    chunks_trajectory.resize(chunks_pc.size());

    for (int i = 0; i < trajectory.size(); i++)
    {
        if(i % 1000 == 0){
            std::cout << "computing [" << i + 1 << "] of: " << trajectory.size() << std::endl;
        }
        for (int j = 0; j < chunks_pc.size(); j++)
        {
            if (chunks_pc[j].size() > 0)
            {
                if (trajectory[i].timestamp_ns >= chunks_pc[j][0].timestamp &&
                    trajectory[i].timestamp_ns < chunks_pc[j][chunks_pc[j].size() - 1].timestamp)
                {
                    chunks_trajectory[j].push_back(trajectory[i]);
                }
            }
        }
    }

    for (const auto &trj : chunks_trajectory)
    {
        std::cout << "number of trajectory elements: " << trj.size() << std::endl;
    }

    /////////////////////////
    std::cout << "start transforming chunks_pc to local coordinate system" << std::endl;
    for (int i = 0; i < chunks_pc.size(); i++)
    {
        std::cout << "computing [" << i + 1 << "] of: " << chunks_pc.size() << std::endl;
        // auto m_inv = chunks_trajectory[i][0].
        if (chunks_trajectory[i].size() == 0){
            continue;
        }

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();

        Eigen::Affine3d first_affine_inv = first_affine.inverse();

        for (auto &p : chunks_pc[i])
        {
            p.point = first_affine_inv * p.point;
            // std::cout << p.point << std::endl;
        }
    }

    ////////////////////////
    if (fs::exists(output_directory)) {
        std::cout << "Directory already exists." << std::endl;
    } else {
    
        try {
            if (fs::create_directory(output_directory)) {
                std::cout << "Directory has been created." << std::endl;
            } else {
                std::cerr << "Failed to create directory " << std::endl;
                return 1; 
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error creating directory: " << e.what() << std::endl;
            return 1; 
        }
    }
    
    fs::path outwd = output_directory;

    Eigen::Vector3d offset(0, 0, 0); // --obliczyc
    int cc = 0;
    for (int i = 0; i < chunks_trajectory.size(); i++)
    {
        for (int j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            offset += trans_curr;
            cc++;
        }
    }
    offset /= cc;

    std::vector<Eigen::Affine3d>
        m_poses;
    std::vector<std::string> file_names;

    for (int i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].size() == 0){
            continue;
        }
        if (chunks_trajectory[i].size() == 0)
        {
            continue;
        }

        fs::path path(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        path /= filename;
        std::cout << "saving to: " << path << " number of points: " << chunks_pc[i].size() << std::endl;
        saveLaz(path.string(), chunks_pc[i]);
        file_names.push_back(filename);

        std::string trajectory_filename = ("trajectory_lio_" + std::to_string(i) + ".csv");
        fs::path pathtrj(outwd);
        pathtrj /= trajectory_filename;
        std::cout << "saving to: " << pathtrj << std::endl;

        std::ofstream outfile;
        outfile.open(pathtrj);
        if (!outfile.good())
        {
            std::cout << "can not save file: " << pathtrj << std::endl;
            return 1;
        }

        outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds" << std::endl;

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();

        Eigen::Affine3d first_affine_inv = first_affine.inverse();
        m_poses.push_back(first_affine);

        for (int j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            Eigen::Quaterniond q_curr(chunks_trajectory[i][j].qw, chunks_trajectory[i][j].qx, chunks_trajectory[i][j].qy, chunks_trajectory[i][j].qz);

            Eigen::Affine3d first_affine_curr = Eigen::Affine3d::Identity();
            first_affine_curr.translation() = trans_curr;
            first_affine_curr.linear() = q_curr.toRotationMatrix();

            auto pose = first_affine_inv * first_affine_curr;
            // auto pose = worker_data_concatenated[i].intermediate_trajectory[0].inverse() * worker_data_concatenated[i].intermediate_trajectory[j];

            outfile
                << std::setprecision(20) << chunks_trajectory[i][j].timestamp_ns * 1e9 << " " << std::setprecision(10)

                << pose(0, 0) << " "
                << pose(0, 1) << " "
                << pose(0, 2) << " "
                << pose(0, 3) << " "
                << pose(1, 0) << " "
                << pose(1, 1) << " "
                << pose(1, 2) << " "
                << pose(1, 3) << " "
                << pose(2, 0) << " "
                << pose(2, 1) << " "
                << pose(2, 2) << " "
                << pose(2, 3) << " "
                // Position (x, y, z)
                // << chunks_trajectory[i][j].x_m << " "  // x
                // << chunks_trajectory[i][j].y_m << " "  // y
                // << chunks_trajectory[i][j].z_m << " "  // z
                // << chunks_trajectory[i][j].qw << " "   // qw
                // << chunks_trajectory[i][j].qx << " "   // qx
                // << chunks_trajectory[i][j].qy << " "   // qy
                // << chunks_trajectory[i][j].qz << " "   // qz
                << std::setprecision(20) << chunks_trajectory[i][j].timestamp_ns * 1e9 << " " << std::setprecision(10)
                << std::endl;
        }
        outfile.close();
    }

    for (auto &m : m_poses)
    {
        m.translation() -= offset;
    }

    fs::path path(outwd);
    path /= "lio_initial_poses.reg";
    save_poses(path.string(), m_poses, file_names);
    fs::path path2(outwd);
    path2 /= "poses.reg";
    save_poses(path2.string(), m_poses, file_names);

    fs::path path3(outwd);
    path3 /= "session.json";

    // save session file
    std::cout << "saving file: '" << path3 << "'" << std::endl;

    nlohmann::json jj;
    nlohmann::json j;
    j["offset_x"] = 0.0;
    j["offset_y"] = 0.0;
    j["offset_z"] = 0.0;
    j["folder_name"] = outwd;
    j["out_folder_name"] = outwd;
    j["poses_file_name"] = path2.string();
    j["initial_poses_file_name"] = path.string();
    j["out_poses_file_name"] = path2.string();
    j["lidar_odometry_version"] = "HdMap";

    jj["Session Settings"] = j;

    nlohmann::json jlaz_file_names;
    for (int i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].size() == 0)
        {
            continue;
        }
        if (chunks_trajectory[i].size() == 0)
        {
            continue;
        }
        
        fs::path path(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        path /= filename;
        std::cout << "adding file: " << path << std::endl;

        nlohmann::json jfn{
            {"file_name", path.string()}};
        jlaz_file_names.push_back(jfn);
    }
    jj["laz_file_names"] = jlaz_file_names;

    std::ofstream fs(path3.string());
    fs << jj.dump(2);
    fs.close();

return 0;
}