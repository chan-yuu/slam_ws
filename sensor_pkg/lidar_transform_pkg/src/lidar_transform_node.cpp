#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <vector>

class LidarTransformNode
{
public:
    LidarTransformNode() : private_nh_("~")
    {
        // === 1. 参数声明与默认值设置 ===
        private_nh_.param<double>("x", transform_x_, -0.0);
        private_nh_.param<double>("y", transform_y_, 0.0);
        private_nh_.param<double>("z", transform_z_, 0.0);
        private_nh_.param<double>("roll", transform_roll_, 0.02658);
        private_nh_.param<double>("pitch", transform_pitch_, 0.05501);
        private_nh_.param<double>("yaw", transform_yaw_, 0.0);

        // 初始化矩阵
        update_transform_matrix();

        // === 2. 创建订阅者与发布者 ===
        sub_ = nh_.subscribe("/rslidar_points", 10, &LidarTransformNode::topic_callback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/rslidar_points_imu", 10);
            
        ROS_INFO("Lidar Transform Node Started (ROS 1).");
        ROS_INFO("Use 'rosparam set /lidar_transform_node/x 1.0' to tune.");
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    Eigen::Matrix4f transform_matrix_;

    // 缓存当前的参数值
    double transform_x_, transform_y_, transform_z_;
    double transform_roll_, transform_pitch_, transform_yaw_;

    // 在ROS 1中，为了避免引入dynamic_reconfigure及其复杂的配置，
    // 我们在回调中查询 cached param。这对于调试标定参数来说足够高效。
    void check_params_and_update()
    {
        bool changed = false;
        if (private_nh_.getParamCached("x", transform_x_)) changed = true;
        if (private_nh_.getParamCached("y", transform_y_)) changed = true;
        if (private_nh_.getParamCached("z", transform_z_)) changed = true;
        if (private_nh_.getParamCached("roll", transform_roll_)) changed = true;
        if (private_nh_.getParamCached("pitch", transform_pitch_)) changed = true;
        if (private_nh_.getParamCached("yaw", transform_yaw_)) changed = true;

        if (changed) {
            update_transform_matrix();
        }
    }

    void update_transform_matrix()
    {
        // 旋转顺序: Z-Y-X (RPY)
        Eigen::AngleAxisf rollAngle(transform_roll_, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(transform_pitch_, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(transform_yaw_, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        Eigen::Translation3f t(transform_x_, transform_y_, transform_z_);

        transform_matrix_ = (t * q).matrix();

        ROS_INFO("Updated Matrix -> T: [%.2f, %.2f, %.2f], RPY: [%.4f, %.4f, %.4f]",
            transform_x_, transform_y_, transform_z_, transform_roll_, transform_pitch_, transform_yaw_);
    }

    void topic_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 检查是否有参数更新
        check_params_and_update();

        // 转换 ROS -> PCL (保留 XYZI)
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud_in);

        // 执行矩阵变换
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud_in, *cloud_out, transform_matrix_);

        // 转换 PCL -> ROS
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_out, output_msg);

        // 修改 Header
        output_msg.header = msg->header; 
        output_msg.header.frame_id = "rslidar"; // 目标坐标系

        pub_.publish(output_msg);
    }
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "lidar_transform_node");
    LidarTransformNode node;
    ros::spin();
    return 0;
}