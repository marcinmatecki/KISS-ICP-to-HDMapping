#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/conversions.h"
#include <laszip/laszip_api.h>
#include <fstream> 
#include <iostream>
#include <string>

struct Point3Di
{
	Eigen::Vector3d point;
	double timestamp;
    float intensity;
    int index_pose;
    uint8_t lidarid;
	int index_point;
};

std::vector<Point3Di> points_global;

void saveOdometryDataToFile(const std::string &filename,
                             double x, double y, double z,
                             double qx, double qy, double qz, double qw)
{
    std::ofstream file;
    file.open(filename, std::ios::app); 

    if (!file.is_open())
    {
        std::cerr << "Błąd otwierania pliku!" << std::endl;
        return;
    }

    file << x << "," << y << "," << z << ","
         << qx << "," << qy << "," << qz << "," << qw << "\n";

    file.close();
    std::cout << "Dane zapisane do pliku " << filename << std::endl;
}


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
        // std::cout << p.timestamp << std::endl;
        //  point->user_data = 0;//p.line_id;
        //  point->classification = p.point.tag;
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

// Funkcja callback dla danych odometrycznych
void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Pobierz dane o pozycji
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // Pobierz dane o orientacji (kwaternion)
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    // Zapisz dane do pliku
    saveOdometryDataToFile("odometry_data.csv", x, y, z, qx, qy, qz, qw);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Otrzymano dane odometryczne:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pozycja: x = %f, y = %f, z = %f", 
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Orientacja: qx = %f, qy = %f, qz = %f, qw = %f",
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Stwórz obiekt PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Sprawdź dane w PointCloud2
    uint8_t* data_ptr = msg->data.data();
    size_t point_step = msg->point_step;

    // Rozpakuj punkty z danych
    for (size_t i = 0; i < msg->width; ++i)
    {
        pcl::PointXYZ point;

        // Wypełnij punkty x, y, z
        point.x = *reinterpret_cast<float*>(data_ptr + i * point_step);
        point.y = *reinterpret_cast<float*>(data_ptr + i * point_step + 4);
        point.z = *reinterpret_cast<float*>(data_ptr + i * point_step + 8);

        cloud.points.push_back(point);

        Point3Di point_global;
        point_global.point = Eigen::Vector3d(point.x, point.y, point.z);
        points_global.push_back(point_global);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Otrzymano %zu punktów z PointCloud2", cloud.points.size());

    // Wyświetl pierwsze 5 punktów
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Punkt %zu: x = %f, y = %f, z = %f", 
                    i, cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    }
    
    saveLaz("out.laz", points_global);
//test3
}

int main(int argc, char **argv)
{
    // Inicjalizacja ROS2
    rclcpp::init(argc, argv);

    // Tworzymy node
    auto node = std::make_shared<rclcpp::Node>("pointcloud_subscriber");

    // Subskrybujemy temat /kiss/odometry
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/kiss/odometry", 10000, odometryCallback);

    // Subskrybujemy temat /kiss/pointcloud
    auto point_cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/kiss/frame", 10000, pointCloudCallback);

    // Czekamy na wiadomości
    rclcpp::spin(node);

    // Zakończenie pracy z ROS2
    rclcpp::shutdown();
    return 0;
}
