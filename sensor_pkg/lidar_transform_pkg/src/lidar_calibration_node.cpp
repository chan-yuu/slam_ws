#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>
#include <deque>
#include <numeric>
#include <vector>

class LidarCalibrationNode
{
public:
    LidarCalibrationNode() : private_nh_("~")
    {
        // === 1. 初始化参数 (设置默认值) ===
        // 使用 private_nh_ 读取私有参数
        private_nh_.param<double>("roi_min_x", p_min_x_, 2.0);
        private_nh_.param<double>("roi_max_x", p_max_x_, 15.0);
        private_nh_.param<double>("roi_min_z", p_min_z_, -3.0);
        private_nh_.param<double>("roi_max_z", p_max_z_, 0.5);
        private_nh_.param<double>("ransac_threshold", p_ransac_thresh_, 0.20);
        private_nh_.param<int>("window_size", p_window_size_, 50);

        // 创建订阅者
        sub_ = nh_.subscribe("/rslidar_points", 10, &LidarCalibrationNode::topic_callback, this);
        
        ROS_INFO("=== Lidar Calibration Node Started (ROS 1) ===");
        ROS_INFO("Dynamic Params supported: roi_min_x, roi_max_x, etc.");
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_; // 用于处理参数
    ros::Subscriber sub_;

    // 参数变量
    double p_min_x_, p_max_x_, p_min_z_, p_max_z_, p_ransac_thresh_;
    int p_window_size_;

    // 滑动窗口
    std::deque<double> pitch_history_;
    std::deque<double> roll_history_;
    std::deque<double> height_history_;

    // 模拟参数更新 (ROS 1 Polling 方式，替代 dynamic_reconfigure 以简化代码)
    void update_params() {
        // getParamCached 会检查参数服务器是否有更新，开销很小
        bool updated = false;
        if (private_nh_.getParamCached("roi_min_x", p_min_x_)) updated = true;
        if (private_nh_.getParamCached("roi_max_x", p_max_x_)) updated = true;
        if (private_nh_.getParamCached("roi_min_z", p_min_z_)) updated = true;
        if (private_nh_.getParamCached("roi_max_z", p_max_z_)) updated = true;
        if (private_nh_.getParamCached("ransac_threshold", p_ransac_thresh_)) updated = true;
        
        int new_window_size;
        if (private_nh_.getParamCached("window_size", new_window_size)) {
            if (new_window_size != p_window_size_) {
                p_window_size_ = new_window_size;
                updated = true;
            }
        }

        if (updated) {
            // 如果参数变了，清空历史数据以避免干扰
            // 注意：这里为了简化，每次有参数更新都清空，实际使用中可能太频繁
            // 但对于手动调参来说是合理的
             // pitch_history_.clear();
             // roll_history_.clear();
             // height_history_.clear();
             // ROS_INFO("Parameters updated via rosparam.");
        }
    }

    void topic_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 0. 尝试更新参数
        update_params();

        // 1. 转 PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 2. 降采样 (加速)
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.15f, 0.15f, 0.15f); // 降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*cloud_filtered);

        // 3. 直通滤波
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(p_min_x_, p_max_x_);
        pass.filter(*cloud_filtered);

        pass.setFilterFieldName("z");
        pass.setFilterLimits(p_min_z_, p_max_z_);
        pass.filter(*cloud_filtered);

        if (cloud_filtered->points.size() < 50) return;

        // 4. RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(p_ransac_thresh_);
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) return;

        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];

        // 强制法向量向上
        if (c < 0) { a = -a; b = -b; c = -c; d = -d; }

        // 过滤非水平面
        if (c < 0.8) return; 

        // 5. 计算瞬时角度 (Rad)
        double cur_pitch_rad = std::atan2(a, c);
        double cur_roll_rad  = std::atan2(-b, c);
        double cur_height    = std::abs(d);

        // 6. 更新滑动窗口
        update_history(pitch_history_, cur_pitch_rad);
        update_history(roll_history_, cur_roll_rad);
        update_history(height_history_, cur_height);

        // 7. 计算平均值
        double avg_pitch_rad = get_average(pitch_history_);
        double avg_roll_rad  = get_average(roll_history_);
        double avg_height    = get_average(height_history_);

        // 8. 打印结果
        static int print_count = 0;
        if (++print_count % 5 == 0) { // 降频打印
            print_statistics(avg_pitch_rad, avg_roll_rad, avg_height);
        }
    }

    void print_statistics(double pitch_rad, double roll_rad, double height) {
        double pitch_deg = pitch_rad * 180.0 / M_PI;
        double roll_deg  = roll_rad  * 180.0 / M_PI;

        ROS_INFO_STREAM( 
            "\n========================================\n"
            << "STATISTICS (Window: " << (int)pitch_history_.size() << " frames)\n"
            << "----------------------------------------\n"
            << "TARGET CORRECTION:\n"
            << "  Pitch : " << std::fixed << std::setprecision(5) << pitch_rad << " rad  | " << std::fixed << std::setprecision(4) << pitch_deg << " deg\n"
            << "  Roll  : " << std::fixed << std::setprecision(5) << roll_rad << " rad  | " << std::fixed << std::setprecision(4) << roll_deg << " deg\n"
            << "  Height: " << std::fixed << std::setprecision(3) << height << " m\n"
            << "========================================");
    }

    void update_history(std::deque<double>& history, double val) {
        history.push_back(val);
        if (history.size() > (size_t)p_window_size_) {
            history.pop_front();
        }
    }

    double get_average(const std::deque<double>& history) {
        if (history.empty()) return 0.0;
        double sum = std::accumulate(history.begin(), history.end(), 0.0);
        return sum / history.size();
    }
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "lidar_calibration_node");
    LidarCalibrationNode node;
    ros::spin();
    return 0;
}