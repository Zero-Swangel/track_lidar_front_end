#include <cmath>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <lidar_front_end_msgs/FrontendOutput.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <PATH.h>

class param_loader{
public: 
    void load(ros::NodeHandle& node){
        /* String */
        string_list.push_back("point_cloud_subscribe_topic");
        string_list.push_back("local_point_cloud_publish_topic");
        string_list.push_back("local_point_cloud_publish_frame_id");
        string_list.push_back("global_point_cloud_publish_topic");
        string_list.push_back("global_point_cloud_publish_frame_id");
        string_list.push_back("bounding_box_publish_topic");
        string_list.push_back("bounding_box_publish_frame_id");
        string_list.push_back("output_publish_topic");
        string_list.push_back("output_publish_frame_id");
        string_list.push_back("current_pose_sub_topic");

        /* Float */
        float_list.push_back("matrix_rotation");
        float_list.push_back("startup_duration");
        float_list.push_back("radius_outlier_r_preprocess");
        float_list.push_back("radius_outlier_min_preprocess");
        float_list.push_back("pass_through_x_min_plane");
        float_list.push_back("pass_through_x_max_plane");
        float_list.push_back("pass_through_y_min_plane");
        float_list.push_back("pass_through_y_max_plane");
        float_list.push_back("pass_through_x_min_preprocess");
        float_list.push_back("pass_through_x_max_preprocess");
        float_list.push_back("pass_through_y_min_preprocess");
        float_list.push_back("pass_through_y_max_preprocess");
        float_list.push_back("pass_through_z_min_preprocess");
        float_list.push_back("pass_through_z_max_preprocess");
        float_list.push_back("pass_through_x_min_alar");
        float_list.push_back("pass_through_x_max_alar");
        float_list.push_back("pass_through_y_min_alar");
        float_list.push_back("pass_through_y_max_alar");
        float_list.push_back("SAC_distance_threshold");
        float_list.push_back("global_show_VoxelFilter_size");
        float_list.push_back("if_use_ndt");
        float_list.push_back("ndt_min_probability");
        float_list.push_back("ndt_VoxelFilter_size");
        float_list.push_back("ndt_StepSize");
        float_list.push_back("ndt_Resolution");
        float_list.push_back("ndt_MaximumIterations");
        float_list.push_back("icp_VoxelFilter_size");
        float_list.push_back("icp_MaxCorrespondenceDistance");
        float_list.push_back("icp_EuclideanFitnessEpsilon");
        float_list.push_back("icp_MaximumIterations");
        float_list.push_back("source_distance_min");
        float_list.push_back("source_distance_max");
        float_list.push_back("relocalization_iteration");
        float_list.push_back("relocalization_rand_distance");
        float_list.push_back("relocalization_squared_distance_error");
        float_list.push_back("relocalization_squared_distance_correct");
        float_list.push_back("final_correct_num");
        float_list.push_back("if_remove_wall");
        float_list.push_back("euclidean_cluster_distance");
        float_list.push_back("euclidean_cluster_min_size");
        float_list.push_back("euclidean_cluster_max_size");
        float_list.push_back("bbox_cone_judge_min_x");
        float_list.push_back("bbox_cone_judge_max_x");
        float_list.push_back("bbox_cone_judge_min_y");
        float_list.push_back("bbox_cone_judge_max_y");
        float_list.push_back("bbox_cone_judge_div_min");
        float_list.push_back("bbox_cone_judge_div_max");
        float_list.push_back("cloud_queue_size");
        float_list.push_back("local_map_frame_size");
        float_list.push_back("distance_threshold");
        float_list.push_back("lidar2imu_theta");
        float_list.push_back("lidar2imu_x");
        float_list.push_back("lidar2imu_y");

        LoadParam(node);

        if(CheckParam()){
            ROS_INFO("Config loaded.");
        }else{
            ROS_ERROR("Load config yaml file first.");
            ROS_INFO("Aborting program.");
            abort();
        }
    }

    std::string getString(std::string var_name){
        // 该判断包含遍历，可能会影响程序运行速度，去除后不检查变量是否存在
        if(string_map.find(var_name) == string_map.end()){
            ROS_ERROR("String %s not found. ", var_name.c_str());
            abort();
        }
        return string_map[var_name];
    }

    float getFloat(std::string var_name){
        // 该判断包含遍历，可能会影响程序运行速度，去除后不检查变量是否存在
        if(float_map.find(var_name) == float_map.end()){
            ROS_ERROR("Float %s not found. ", var_name.c_str());
            abort();
        }
        return float_map[var_name];
    }

private: 
    std::vector<std::string> string_list;
    std::vector<std::string> float_list;
    std::map<std::string, std::string> string_map;
    std::map<std::string, float> float_map;

    void LoadParam(ros::NodeHandle& node){
        for(std::vector<std::string>::iterator \
            it = string_list.begin(); it != string_list.end(); it++){
            node.param<std::string> (*it, string_map[*it], "NULL");
        }
        for(std::vector<std::string>::iterator \
            it = float_list.begin(); it != float_list.end(); it++){
            node.param<float> (*it, float_map[*it], 0.);
        }
    }

    bool CheckParam(){
        for(std::map<std::string, std::string>::iterator \
            string_it = string_map.begin(); string_it != string_map.end(); string_it++){
            if(string_map[string_it->first] == "NULL"){
                ROS_ERROR("Failed loading string %s", string_it->first.c_str());
                return false;
            }
        }
        for(std::map<std::string, float>:: iterator \
            float_it = float_map.begin(); float_it != float_map.end(); float_it++){
            if(float_map[float_it->first] == 0.){
                ROS_ERROR("Failed loading float %s", float_it->first.c_str());
                return false;
            }
        }
        return true;
    }
}config;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
Eigen::Matrix4f imu(Eigen::Matrix4f::Identity());
ros::Time time_stamp;

// Subscribe点云
void Callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::fromROSMsg(*msg, *cloud_ptr);
    time_stamp = msg->header.stamp;
}

Eigen::Matrix4f fromPoseToMatrix(const geometry_msgs::Pose &pose);
void imu_Callback(const geometry_msgs::PoseStamped& msg){
    imu = fromPoseToMatrix(msg.pose);
    // time_stamp = msg.header.stamp;
}

// Publish点云
void CloudPublisher(ros::Publisher publisher, \
                const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud){
    sensor_msgs::PointCloud2 cloud_output;
    pcl::toROSMsg(*cloud, cloud_output);
    publisher.publish(cloud_output);
}

// Matrix转Odometry
void getOdometryFromMatrix4f(const Eigen::Matrix4f& matrix, nav_msgs::Odometry& odometry, ros::Time time_stamp){
    odometry.header.frame_id = config.getString("output_publish_frame_id");
    odometry.header.stamp = ros::Time(time_stamp);
    odometry.child_frame_id = config.getString("output_publish_frame_id");

    odometry.pose.pose.position.x = matrix(0,3);
    odometry.pose.pose.position.y = matrix(1,3);
    odometry.pose.pose.position.z = matrix(2,3);

    Eigen::Quaternionf q (matrix.block<3,3>(0,0));
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
}

Eigen::Matrix4f fromPoseToMatrix(const geometry_msgs::Pose &pose)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // 平移
    transform(0, 3) = pose.position.x;
    transform(1, 3) = pose.position.y;
    transform(2, 3) = pose.position.z;

    // 余弦矩阵 -> 四元数
    transform.block<3, 3>(0, 0) = \
        Eigen::Quaternionf( pose.orientation.w, 
                            pose.orientation.x, 
                            pose.orientation.y, 
                            pose.orientation.z ).toRotationMatrix();
    return transform;
}

void fromBoxToPointCloud(jsk_recognition_msgs::BoundingBoxArray& box_array, \
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
        cloud->points.clear();
        for(auto &box : box_array.boxes){
            pcl::PointXYZ point;
            point.x = box.pose.position.x;
            point.y = box.pose.position.y;
            point.z = box.pose.position.z;
            cloud->points.push_back(point);
        }
        // cloud.header.frame_id = config.getString("global_map_publish_frame_id");
        cloud->header.stamp = ros::Time::now().toSec();
    }

// 直通滤波
void PassThrough(std::string field_name, float min, float max, bool pass_nagetive, \
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, \
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered){
    pcl::PassThrough<pcl::PointXYZ> pass_through;
    pass_through.setInputCloud(cloud);
    pass_through.setFilterFieldName(field_name);
    pass_through.setFilterLimits(min, max);
    pass_through.setFilterLimitsNegative(pass_nagetive);
    pass_through.filter(*cloud_filtered);
}

// 体素滤波
void VoxelGrid(float size, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, \
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered){
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_grid;
    // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(size, size, size);
    // voxel_grid.setDownsampleAllData(true);
    voxel_grid.filter(*cloud_filtered);
}

// 半径滤波
void RadiusOutlier(float radius, float min, bool pass_nagetive, \
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, \
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered){
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
    radius_outlier.setInputCloud(cloud);
    radius_outlier.setRadiusSearch(radius);
    radius_outlier.setMinNeighborsInRadius(min);
    radius_outlier.setNegative(pass_nagetive);
    radius_outlier.filter(*cloud_filtered);
}

// 条件滤波
void ConditionRemoval(float x_min, float x_max, float y_min, float y_max, bool pass_nagetive,\
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, \
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered){
    pcl::ConditionalRemoval<pcl::PointXYZ> condition_removal;
    if(pass_nagetive){
        pcl::ConditionOr<pcl::PointXYZ>::Ptr or_condition (new pcl::ConditionOr<pcl::PointXYZ> ());
        or_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> \
            ("x", pcl::ComparisonOps::LT, x_min)));
        or_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> \
            ("x", pcl::ComparisonOps::GT, x_max)));
        or_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> \
            ("y", pcl::ComparisonOps::LT, y_min)));
        or_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> \
            ("y", pcl::ComparisonOps::GT, y_max)));
        condition_removal.setCondition(or_condition);
    }else{
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr and_condition (new pcl::ConditionAnd<pcl::PointXYZ> ());
        and_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> \
            ("x", pcl::ComparisonOps::GT, x_min)));
        and_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> \
            ("x", pcl::ComparisonOps::LT, x_max)));
        and_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> \
            ("y", pcl::ComparisonOps::GT, y_min)));
        and_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> \
            ("y", pcl::ComparisonOps::LT, y_max)));
        condition_removal.setCondition(and_condition);
    }
    condition_removal.setInputCloud(cloud);
    condition_removal.setKeepOrganized(false);
    condition_removal.filter(*cloud_filtered);
}

// 去除地面
void RANSACPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, \
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, \
                Eigen::Vector4f& vector){
    if (cloud->size() > 0){
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> SAC_segmentation;
        SAC_segmentation.setOptimizeCoefficients(true);
        SAC_segmentation.setModelType(pcl::SACMODEL_PLANE);
        SAC_segmentation.setMethodType(pcl::SAC_RANSAC);
        SAC_segmentation.setMaxIterations(10000);
        SAC_segmentation.setDistanceThreshold(config.getFloat("SAC_distance_threshold"));
        SAC_segmentation.setInputCloud(cloud);
        SAC_segmentation.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
            ROS_ERROR("error! Could not found any inliers!");

        // pcl::ExtractIndices<pcl::PointXYZ> extractor;
        // extractor.setInputCloud(cloud);
        // extractor.setIndices(inliers);
        // extractor.setNegative(true);
        // extractor.filter(*cloud_filtered);

        vector = {coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};
    }else
        ROS_ERROR("no data found!");
}

// 旋转点云
void Rotate( pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector4f plane_vector){
    Eigen::Vector3f vector_before = plane_vector.head<3>();
    Eigen::Vector3f vector_after(0, 0, 1);

    if(cloud->size() > 0){
        if (vector_before.cross(vector_after).norm() < 0.01){
            const float z_t = (-plane_vector(3)) / plane_vector(2);
            for (auto &point : cloud->points)
                point.z -= z_t;
            return;
        }

        Eigen::Vector3f b;
        Eigen::Matrix3f A;
        A << vector_before(0), vector_before(1), vector_before(2), \
            0, 0, 1,  \
            1, 0, 0 ;
        b << -plane_vector(3), 0, 10;
        Eigen::Vector3f point_A = A.colPivHouseholderQr().solve(b);

        A << vector_before(0), vector_before(1), vector_before(2), \
            0, 0, 1,  \
            1, 0, 0 ;
        b << -plane_vector(3), 0, 20;
        Eigen::Vector3f point_B = A.colPivHouseholderQr().solve(b);
        
        Eigen::Vector3f rotate_axis = (point_B - point_A).normalized();

        const float cos_theta = vector_before.normalized().dot(vector_after);
        const float theta = acos(cos_theta);
        const float sin_theta = sin(theta);

        size_t index = 0;
        for(auto &point : cloud->points){
            const Eigen::Vector3f v = Eigen::Vector3f(point.x, point.y, point.z) - point_A;
            const Eigen::Vector3f VRR = v.dot(rotate_axis) * rotate_axis;
            const Eigen::Vector3f V_VRR = v - VRR;

            Eigen::Vector3f res_point;
            res_point = rotate_axis.cross(V_VRR) * sin_theta + \
                        V_VRR * cos_theta + VRR + point_A;
            point.x = res_point(0);
            point.y = res_point(1);
            point.z = res_point(2);
        }
    }else
        ROS_ERROR("rotation failed!");
}

// 生成旋转矩阵
Eigen::Matrix4f CreateRoatationMatrix(Eigen::Vector3f angle_before, Eigen::Vector3f angle_after){
    angle_before.normalize();
    angle_after.normalize();
    float angle = acos(angle_before.dot(angle_after));
    Eigen::Vector3f p_rotate = angle_before.cross(angle_after);
    p_rotate.normalize();
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle)+ p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle)) - p_rotate[2] * sin(angle);
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));
    return rotationMatrix;
}

// 预处理
void Preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered_ptr){
    //去除NaN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_ptr, *cloud_filtered_ptr, indices);

    //找地面
    Eigen::Vector4f plane_vector;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
    PassThrough("x", config.getFloat("pass_through_x_min_plane"), config.getFloat("pass_through_x_max_plane"), false, cloud_filtered_ptr, cloud_passthrough);
    PassThrough("y", config.getFloat("pass_through_y_min_plane"), config.getFloat("pass_through_y_max_plane"), false, cloud_filtered_ptr, cloud_passthrough);
    RANSACPlane(cloud_passthrough, cloud_passthrough, plane_vector);

    //水平校准(二选一)
    if(config.getFloat("matrix_rotation")){
        Eigen::Vector3f standard(0, 0, 1);
        Eigen::Vector3f angle_before =  plane_vector.head<3>();
        Eigen::Matrix4f rotationMatrix = CreateRoatationMatrix(angle_before, standard);
        rotationMatrix(2, 3) = plane_vector[3] / plane_vector[2];
        pcl::transformPointCloud(*cloud_filtered_ptr, *cloud_filtered_ptr, rotationMatrix);
    }else{
        Rotate(cloud_filtered_ptr, plane_vector);
    }

    //框选视野
    PassThrough("y", config.getFloat("pass_through_y_min_preprocess"), config.getFloat("pass_through_y_max_preprocess"), false, cloud_filtered_ptr, cloud_filtered_ptr);
    PassThrough("z", config.getFloat("pass_through_z_min_preprocess"), config.getFloat("pass_through_z_max_preprocess"), false, cloud_filtered_ptr, cloud_filtered_ptr);
    PassThrough("x", config.getFloat("pass_through_x_min_preprocess"), config.getFloat("pass_through_x_max_preprocess"), false, cloud_filtered_ptr, cloud_filtered_ptr);

    //去除鼻翼
    ConditionRemoval(config.getFloat("pass_through_x_min_alar"), config.getFloat("pass_through_x_max_alar"), \
                    config.getFloat("pass_through_y_min_alar"), config.getFloat("pass_through_y_max_alar"), true, \
                    cloud_filtered_ptr, cloud_filtered_ptr);
}

// NDT配准矩阵
bool NDT(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, Eigen::Matrix4f imu_to_lidar, \
        Eigen::Matrix4f& current_pose, Eigen::Matrix4f& last_imu_pose, Eigen::Matrix4f& imu_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered){

    Eigen::Matrix4f guess_matrix = current_pose * imu_to_lidar.inverse() * last_imu_pose.inverse() * imu_pose * imu_to_lidar;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target(new pcl::PointCloud<pcl::PointXYZ>);

    VoxelGrid(config.getFloat("ndt_VoxelFilter_size"), source, filtered_source);
    VoxelGrid(config.getFloat("ndt_VoxelFilter_size"), target, filtered_target);

    if(filtered_target->points.size() < filtered_source->points.size()*1.5){
        ROS_ERROR("ndt.registration failded! lose frame!");
        current_pose = guess_matrix;
        return false;
    }else{
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setStepSize(config.getFloat("ndt_StepSize"));
        ndt.setResolution(config.getFloat("ndt_Resolution"));
        ndt.setMaximumIterations(config.getFloat("ndt_MaximumIterations"));
        ndt.setInputSource(filtered_source);
        ndt.setInputTarget(filtered_target);
        ndt.align(*cloud_filtered, guess_matrix);

        const float ndtProbability = ndt.getTransformationProbability();
        if (ndtProbability < config.getFloat("ndt_min_probability")){
            current_pose = guess_matrix;
            ROS_ERROR("FAILED.  NDT Probability: %f", ndtProbability);
            return false;
        }else{
            current_pose = ndt.getFinalTransformation();
            // ROS_INFO("NDT Probability: %f", ndtProbability);  
            return true;
        }
    }
    return false;
}

float PointDistance(pcl::PointXYZ point1, pcl::PointXYZ point2){
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) + \
            (point1.y - point2.y)*(point1.y - point2.y) + \
            (point1.z - point2.z)*(point1.z - point2.z));
}
bool CheckDistance(std::vector<pcl::PointXYZ> vector, float min, float max){
    if(vector.size() == 0)
        return false;
    assert(vector.size() == 4);
    for(int i=0; i<3; i++){
        for(int j=i+1; j<4; j++){
            if(PointDistance(vector[i], vector[j]) < min)
                return false;
            if(PointDistance(vector[i], vector[j]) > max)
                return false;
        }
    }
    return true;
}
// ReLocalization(box_array[转成点云], bounding_box_queue, imu_to_lidar, current_pose, last_imu_pose, imu_pose);
bool ReLocalization(ros::NodeHandle& node, jsk_recognition_msgs::BoundingBoxArray box_array, std::deque<pcl::PointCloud<pcl::PointXYZ>> bounding_box_queue, \
                    Eigen::Matrix4f imu_to_lidar, Eigen::Matrix4f& current_pose, Eigen::Matrix4f& last_imu_pose, Eigen::Matrix4f& imu_pose){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    int best_correct_num = -1;
    float best_error = std::numeric_limits<float>::max();
    Eigen::Matrix4f best_transform(Eigen::Matrix4f::Identity());

    ros::Publisher source_pub = node.advertise<sensor_msgs::PointCloud2>("/source_cloud", 100, true);
    ros::Publisher target_pub = node.advertise<sensor_msgs::PointCloud2>("/target_cloud", 100, true);

    fromBoxToPointCloud(box_array, cloud_source_ptr);
    for(auto &box_cloud : bounding_box_queue){
        pcl::copyPointCloud(box_cloud, *cloud_target_ptr);
        // assert(cloud_source_ptr->points.size() != 0);
        // assert(cloud_target_ptr->points.size() != 0);
        // 防止聚类失败时直接去世
        if(cloud_source_ptr->points.size() == 0){
            ROS_ERROR("FAILED relocalization! no source input! ");
            return false;
        }
        if(cloud_target_ptr->points.size() == 0){
            ROS_ERROR("FAILED relocalization! no target input! ");
            return false;
        }

        // ROS_INFO("source size: %ld", cloud_source_ptr->points.size());
        // ROS_INFO("target size: %ld", cloud_target_ptr->points.size());

        // 平面化(可以去除)
        for(int i=0; i<cloud_source_ptr->points.size(); i++){
            cloud_source_ptr->points[i].z = 0;
        }
        for(int i=0; i<cloud_target_ptr->points.size(); i++){
            cloud_target_ptr->points[i].z = 0;
        }

        std::vector<pcl::PointXYZ> rand_vec_source;
        std::vector<pcl::PointXYZ> rand_vec_target;

        

        std::srand((unsigned int)(time(NULL)));
        ros::Time start = ros::Time::now(); // 正式开始

        for(int iteration=0; iteration < config.getFloat("relocalization_iteration"); iteration++){
            rand_vec_source.clear();
            rand_vec_target.clear();
            int rand;

            // 生成四个随机点
            int rand_times = 0; // 防止桶过少时死循环
            while(!CheckDistance(rand_vec_source, config.getFloat("source_distance_min"), config.getFloat("source_distance_max"))){
                if(rand_times > 500)
                    return false;
                rand_vec_source.clear();
                for(int i=0; i<4; i++){
                    rand = std::rand();
                    int p = rand % (cloud_source_ptr->points.size());
                    rand_vec_source.push_back(cloud_source_ptr->points[p]);
                }
                rand_times++;
            }
            // ROS_INFO("source points");

            // 记录距离信息(可以用vector替换，按ij遍历的顺序排列就行)
            float source_distance_01 = PointDistance(rand_vec_source[0], rand_vec_source[1]);
            float source_distance_02 = PointDistance(rand_vec_source[0], rand_vec_source[2]);
            float source_distance_03 = PointDistance(rand_vec_source[0], rand_vec_source[3]);
            float source_distance_12 = PointDistance(rand_vec_source[1], rand_vec_source[2]);
            float source_distance_13 = PointDistance(rand_vec_source[1], rand_vec_source[3]);
            float source_distance_23 = PointDistance(rand_vec_source[2], rand_vec_source[3]);
            float target_distance_01 = 0;
            float target_distance_02 = 0;
            float target_distance_03 = 0;
            float target_distance_12 = 0;
            float target_distance_13 = 0;
            float target_distance_23 = 0;

            std::vector<int> point_indexs; // 没用了
            point_indexs.clear();
            { // 随机第一点
                rand = std::rand();
                int p = rand % (cloud_target_ptr->points.size());
                rand_vec_target.push_back(cloud_target_ptr->points[p]);
                point_indexs.push_back(p);
            }
            // ROS_INFO("target point 1");

            float min_distance = config.getFloat("relocalization_rand_distance"); // 改小了会更精确但也会更慢，改大了会比较看运气
            std::vector<pcl::PointXYZ> point_list;

            // 生成第二点
            point_list.clear();
            for(int i=0; i<cloud_target_ptr->points.size(); i++){
                target_distance_01 = PointDistance(rand_vec_target[0], cloud_target_ptr->points[i]);
                if(fabs(target_distance_01 - source_distance_01) > min_distance)
                    continue;
                point_list.push_back(cloud_target_ptr->points[i]);
            }
            if(point_list.size()>0){
                rand = std::rand();
                int p = rand % (point_list.size());
                rand_vec_target.push_back(point_list[p]);
                point_indexs.push_back(p);
            }
            // ROS_INFO("target point 2");

            // 生成第三点
            point_list.clear();
            for(int i=0; i<cloud_target_ptr->points.size(); i++){
                target_distance_02 = PointDistance(rand_vec_target[0], cloud_target_ptr->points[i]);
                target_distance_12 = PointDistance(rand_vec_target[1], cloud_target_ptr->points[i]);
                if(fabs(target_distance_02- source_distance_02) > min_distance)
                    continue;
                if(fabs(target_distance_12- source_distance_12) > min_distance)
                    continue;
                point_list.push_back(cloud_target_ptr->points[i]);
            }
            if(point_list.size()>0){
                rand = std::rand();
                int p = rand % (point_list.size());
                rand_vec_target.push_back(point_list[p]);
                point_indexs.push_back(p);
            }
            // ROS_INFO("target point 3");

            // 生成第四点
            point_list.clear();
            for(int i=0; i<cloud_target_ptr->points.size(); i++){
                target_distance_03 = PointDistance(rand_vec_target[0], cloud_target_ptr->points[i]);
                target_distance_13 = PointDistance(rand_vec_target[1], cloud_target_ptr->points[i]);
                target_distance_23 = PointDistance(rand_vec_target[2], cloud_target_ptr->points[i]);
                if(fabs(target_distance_03- source_distance_03) > min_distance)
                    continue;
                if(fabs(target_distance_13- source_distance_13) > min_distance)
                    continue;
                if(fabs(target_distance_23- source_distance_23) > min_distance)
                    continue;
                point_list.push_back(cloud_target_ptr->points[i]);
            }
            if(point_list.size()>0){
                rand = std::rand();
                int p = rand % (point_list.size());
                rand_vec_target.push_back(point_list[p]);
                point_indexs.push_back(p);
            }
            // ROS_INFO("target point 4");

            // 不足四个点，跳过
            if(rand_vec_target.size() != 4)
                continue;

            // 对应点距离过大，跳过
            if(PointDistance(rand_vec_source[0], rand_vec_target[0]) > 2)
                continue;
            if(PointDistance(rand_vec_source[1], rand_vec_target[1]) > 2)
                continue;
            if(PointDistance(rand_vec_source[2], rand_vec_target[2]) > 2)
                continue;
            if(PointDistance(rand_vec_source[3], rand_vec_target[3]) > 2)
                continue;

            // 控制点距离过近，跳过(如果前面没问题的话这里按道理不会出现过近)
            // if(PointDistance(rand_vec_target[0], rand_vec_target[1]) < 0.05)
            //     continue;
            // if(PointDistance(rand_vec_target[0], rand_vec_target[2]) < 0.05)
            //     continue;
            // if(PointDistance(rand_vec_target[0], rand_vec_target[3]) < 0.05)
            //     continue;
            // if(PointDistance(rand_vec_target[1], rand_vec_target[2]) < 0.05)
            //     continue;
            // if(PointDistance(rand_vec_target[1], rand_vec_target[3]) < 0.05)
            //     continue;
            // if(PointDistance(rand_vec_target[2], rand_vec_target[3]) < 0.05)
            //     continue;

            // svd配准选取的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr svd_cloud_source_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr svd_cloud_target_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr svd_cloud_output_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            for(int i=0; i<4; i++){
                svd_cloud_source_ptr->points.push_back(rand_vec_source[i]);
                svd_cloud_target_ptr->points.push_back(rand_vec_target[i]);
                assert(svd_cloud_source_ptr->points.size() == 4);
                assert(svd_cloud_target_ptr->points.size() == 4);
            }

            pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;
            pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 svd_transformation;
            svd.estimateRigidTransformation (*svd_cloud_source_ptr, *svd_cloud_target_ptr, svd_transformation);
            // std::cout << "transform: n" << svd_transformation << std::endl;

            pcl::transformPointCloud(*cloud_source_ptr, *svd_cloud_output_ptr, svd_transformation);

            CloudPublisher(source_pub, cloud_source_ptr);
            CloudPublisher(target_pub, cloud_target_ptr);

            // 神奇bug，前十来次循环会有不明问题
            // 导致正常的点云通过正常的变换矩阵变换得到一个全在一个点上的点云
            // 导致best_error直接变成0
            if(iteration < 20){
                // std::cout << "pass iteration " << iteration << std::endl;
                continue;
            }

            // KDTree计算配准得分
            int K = 4;
            int correct_num = 0;
            float this_error = 0;
            std::vector<int> point_index(K);
            std::vector<float> point_squared_distance(K);
            std::vector<float> point_distances;
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(cloud_target_ptr);

            for(int i=0; i<svd_cloud_output_ptr->points.size(); i++){
                if(kdtree.nearestKSearch(svd_cloud_output_ptr->points[i], K, point_index, point_squared_distance)){
                    if(point_squared_distance[0] > config.getFloat("relocalization_squared_distance_error")){
                        this_error += point_squared_distance[0];
                    }
                    if(point_squared_distance[0] < config.getFloat("relocalization_squared_distance_correct")){
                        correct_num++;
                        point_distances.push_back(point_squared_distance[0]);
                    }
                }else{
                    this_error = std::numeric_limits<float>::max();
                    break;
                }
            }
            if(correct_num == 0){
                this_error = std::numeric_limits<float>::max();
            }

            if((this_error < best_error)&&(correct_num >= best_correct_num)){
                best_error = this_error;
                best_correct_num = correct_num;
                best_transform = svd_transformation;
                // pcl::transformPointCloud(*cloud_source_ptr, *cloud_transformed_ptr, best_transform);
                // std::cout << "====================" << std::endl;
                // std::cout << "best_error: " << best_error << std::endl;
                // // std::cout << "this_error: " << this_error << std::endl;
                // std::cout << "correct_num: " << correct_num << std::endl;
                // // std::cout << "best_transform: \n" << best_transform << std::endl;
                // std::cout << "Time: " << (ros::Time::now() - start).toSec() << std::endl;
                // std::cout << "iteration: " << iteration << std::endl;
            }
        }
        if(best_correct_num > config.getFloat("final_correct_num")){
            current_pose = best_transform;
            return true;
        }
    }
    return false;
}

// 欧式聚类
void EuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, \
                      jsk_recognition_msgs::BoundingBoxArray& box_array, \
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered){
    std::vector<pcl::PointIndices> inlier;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointIndices::Ptr extract_inlier(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    pcl::copyPointCloud(*cloud, *cloud_2d);
    for (auto &point : cloud_2d->points)
        point.z = 0;

    ece.setInputCloud(cloud_2d);
	ece.setClusterTolerance(config.getFloat("euclidean_cluster_distance"));
	ece.setMinClusterSize(config.getFloat("euclidean_cluster_min_size"));
	ece.setMaxClusterSize(config.getFloat("euclidean_cluster_max_size"));
	ece.setSearchMethod(tree);
	ece.extract(inlier);

    box_array.boxes.clear();

    for(std::vector<pcl::PointIndices>::const_iterator it = inlier.begin(); it != inlier.end(); it++){
        float sum_x=0, sum_y=0, sum_z=0;
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();
        extract_inlier->indices.clear();

        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
            const float x = cloud->points[*pit].x;
            const float y = cloud->points[*pit].y;
            const float z = cloud->points[*pit].z;
            sum_x += x;
            sum_y += y;
            sum_z += z;
            if(x < min_x) min_x = x;
            if(y < min_y) min_y = y;
            if(z < min_z) min_z = z;
            if(x > max_x) max_x = x;
            if(y > max_y) max_y = y;
            if(z > max_z) max_z = z;
            extract_inlier->indices.push_back(*pit);
        }
        
        jsk_recognition_msgs::BoundingBox box;
        box.header.frame_id = config.getString("bounding_box_publish_frame_id");
        box.pose.position.x = sum_x / it->indices.size();
        box.pose.position.y = sum_y / it->indices.size();
        box.pose.position.z = sum_z / it->indices.size();
        box.dimensions.x = max_x > min_x ? max_x - min_x : min_x - max_x;
        box.dimensions.y = max_y > min_y ? max_y - min_y : min_y - max_y;
        box.dimensions.z = max_z > min_z ? max_z - min_z : min_z - max_z;

        if( (box.dimensions.x > config.getFloat("bbox_cone_judge_max_x")) || \
            (box.dimensions.x < config.getFloat("bbox_cone_judge_min_x")) || \
            (box.dimensions.y > config.getFloat("bbox_cone_judge_max_y")) || \
            (box.dimensions.y < config.getFloat("bbox_cone_judge_min_y")) || \
            (box.dimensions.z / box.dimensions.y < config.getFloat("bbox_cone_judge_div_min")) || \
            (box.dimensions.z / box.dimensions.y > config.getFloat("bbox_cone_judge_div_max")) || \
            (box.dimensions.z / box.dimensions.x < config.getFloat("bbox_cone_judge_div_min")) || \
            (box.dimensions.z / box.dimensions.x > config.getFloat("bbox_cone_judge_div_max")) ){
            if(config.getFloat("if_remove_wall") == 1){
                extract.setInputCloud(cloud);
                extract.setIndices(extract_inlier);
                extract.setNegative(true);
                extract.filter(*cloud_filtered);
            }
        }else{
            box_array.boxes.push_back(box);
        }
    }
    // RadiusOutlier(config.getFloat("radius_outlier_r_preprocess"), config.getFloat("radius_outlier_min_preprocess"), false, \
    //             cloud_filtered, cloud_filtered);
}

void Transform(Eigen::Matrix4f current_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_transformed, \
                jsk_recognition_msgs::BoundingBoxArray box_array, jsk_recognition_msgs::BoundingBoxArray& box_array_transformed, pcl::PointCloud<pcl::PointXYZ>::Ptr& box_cloud){
            for (auto &box : box_array.boxes){
                pcl::PointXYZ box_point;
                Eigen::Vector4f point(box.pose.position.x, box.pose.position.y, box.pose.position.z, 1);
                point = current_pose * point;
                box.pose.position.x = point(0);
                box.pose.position.y = point(1);
                box.pose.position.z = point(2);
                box_point.x = point(0);
                box_point.y = point(1);
                box_point.z = point(2);
                box_array_transformed.boxes.push_back(box);
                box_cloud->points.push_back(box_point);
            }
            pcl::transformPointCloud(*cloud, *cloud_transformed, current_pose);
}

// 加入地图
void PushMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, \
            std::deque<pcl::PointCloud<pcl::PointXYZ>>& cloud_queue, \
            pcl::PointCloud<pcl::PointXYZ>::Ptr& local_map, \
            pcl::PointCloud<pcl::PointXYZ>::Ptr& global_map){
    cloud_queue.push_back(*cloud);
    size_t localmap_frame_num = 0;
    local_map->points.clear();
    for (auto cloud = cloud_queue.rbegin(); cloud != cloud_queue.rend() && \
        localmap_frame_num < config.getFloat("local_map_frame_size") ; cloud++, localmap_frame_num++){
        *local_map += *cloud;
    }
    VoxelGrid(config.getFloat("global_show_VoxelFilter_size"), cloud, cloud);
    *global_map += *cloud;
}

void InitLidar2Imu(Eigen::Matrix4f& imu_to_lidar){
    const float sin_theta = sin(config.getFloat("lidar2imu_theta") * 3.1415 / 180);
    const float cos_theta = cos(config.getFloat("lidar2imu_theta") * 3.1415 / 180);
    imu_to_lidar(0, 0) =  cos_theta;
    imu_to_lidar(0, 1) = -sin_theta;
    imu_to_lidar(1, 0) =  sin_theta;
    imu_to_lidar(1, 1) =  cos_theta;
    imu_to_lidar(0, 3) =  config.getFloat("lidar2imu_x");
    imu_to_lidar(1, 3) =  config.getFloat("lidar2imu_y");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "NODE_NAME");
    ros::NodeHandle node("~");
    config.load(node);
    
    ros::Subscriber cloud_subscriber = node.subscribe(config.getString("point_cloud_subscribe_topic"), 100, Callback);
    ros::Subscriber imu_subscriber = node.subscribe(config.getString("current_pose_sub_topic"), 100, imu_Callback);
    ros::Publisher local_map_publisher = node.advertise<sensor_msgs::PointCloud2>(config.getString("local_point_cloud_publish_topic"), 100, true);
    ros::Publisher global_map_publisher = node.advertise<sensor_msgs::PointCloud2>(config.getString("global_point_cloud_publish_topic"), 100, true);
    ros::Publisher bbox_publisher = node.advertise<jsk_recognition_msgs::BoundingBoxArray>(config.getString("bounding_box_publish_topic"), 100, true);
    ros::Publisher output_publisher = node.advertise<lidar_front_end_msgs::FrontendOutput>(config.getString("output_publish_topic"), 100, true);

    std::deque<pcl::PointCloud<pcl::PointXYZ>> cloud_queue;
    std::deque<pcl::PointCloud<pcl::PointXYZ>> bounding_box_queue;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_transformed_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    jsk_recognition_msgs::BoundingBoxArray box_array;
    jsk_recognition_msgs::BoundingBoxArray local_box_array;

    Eigen::Matrix4f current_pose(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f imu_to_lidar(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f last_imu_pose(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f imu_pose(Eigen::Matrix4f::Identity());

    InitLidar2Imu(imu_to_lidar);

    lidar_front_end_msgs::FrontendOutput output;
    nav_msgs::Odometry output_odometry;

    float last_keyFrame_x;
    float last_keyFrame_y;
    float last_keyFrame_z;

    bool if_add_key_frame;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr static_global_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/walker-ubuntu/Walkerspace/ros_ws/global_map.pcd", *static_global_map_ptr) == 0){
    //     ROS_INFO("Load PointCloud file successfully. \ncloud size: %ld", static_global_map_ptr->points.size());
    // }else{
    //     ROS_ERROR("Faild loading PointCloud file. ");
    //     abort();
    // }
    // static_global_map_ptr->header.frame_id = "map";

    ros::Duration(config.getFloat("startup_duration")).sleep();
    ros::Rate rate(50);
    while(ros::ok()){
        ros::spinOnce();
        if_add_key_frame = true;
        ros::Time begin = ros::Time::now();

        Preprocess(cloud_filtered_ptr);

        if(cloud_filtered_ptr->points.size() == 0){
            ROS_ERROR("No cloud  input! ");
            continue;
        }
        if(imu.size() == 0){
            ROS_ERROR("No imu input! ");
            continue;
        }

        if(cloud_queue.empty()){;
            last_imu_pose = imu;
            imu_pose = imu;


            last_keyFrame_x = 0;
            last_keyFrame_y = 0;
            last_keyFrame_z = 0;
            cloud_queue.push_back(*cloud_filtered_ptr);

            pcl::copyPointCloud(*cloud_filtered_ptr, *local_map_ptr);

            VoxelGrid(config.getFloat("global_show_VoxelFilter_size"), cloud_filtered_ptr, cloud_filtered_ptr);
            pcl::copyPointCloud(*cloud_filtered_ptr, *global_map_ptr);

        }else{
            last_imu_pose = imu_pose;
            imu_pose = imu;

            ros::Time middle = ros::Time::now();

            EuclideanCluster(cloud_filtered_ptr, box_array, cloud_filtered_ptr);

            if(config.getFloat("if_use_ndt") == 1){
                if(NDT(cloud_filtered_ptr, local_map_ptr, imu_to_lidar, current_pose, last_imu_pose, imu_pose , cloud_transformed_ptr)){
                    if_add_key_frame = true;
                }else{
                    // ROS_INFO("try relocalization");
                    if(ReLocalization(node, box_array, bounding_box_queue, imu_to_lidar, current_pose, last_imu_pose, imu_pose)){
                        ROS_INFO("relocalization success");
                        if_add_key_frame = true;
                    }else{
                        ROS_ERROR("relocalization failed");
                        if_add_key_frame = false; // false
                    }

                    // current_pose = current_pose * imu_to_lidar.inverse() * last_imu_pose.inverse() * imu_pose * imu_to_lidar;
                    // if_add_key_frame = false;
                }
            }else{
                current_pose = current_pose * imu_to_lidar.inverse() * last_imu_pose.inverse() * imu_pose * imu_to_lidar;
            }

            Eigen::AngleAxis<float> current_pose_angleAxis(current_pose.block<3, 3>(0, 0));
            current_pose_angleAxis.axis()(0) = 0;
            current_pose_angleAxis.axis()(1) = 0;
            current_pose.block<3, 3>(0, 0) = current_pose_angleAxis.toRotationMatrix();
            current_pose(2, 3) = 0;

            local_box_array.boxes.clear();
            box_transformed_ptr->points.clear();
            Transform(current_pose, cloud_filtered_ptr, cloud_transformed_ptr, box_array, local_box_array, box_transformed_ptr);

            const float delta_x = fabs(current_pose(0, 3) - last_keyFrame_x);
            const float delta_y = fabs(current_pose(1, 3) - last_keyFrame_y);
            const float delta_z = fabs(current_pose(2, 3) - last_keyFrame_z);
            const float distance = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);

            if(if_add_key_frame && (distance > config.getFloat("distance_threshold") || cloud_queue.size() <= config.getFloat("cloud_queue_size"))){
                last_keyFrame_x = current_pose(0, 3);
                last_keyFrame_y = current_pose(1, 3);
                last_keyFrame_z = current_pose(2, 3);
                PushMap(cloud_transformed_ptr, cloud_queue, local_map_ptr, global_map_ptr);
                bounding_box_queue.push_back(*box_transformed_ptr);
                while(bounding_box_queue.size()>20){
                    bounding_box_queue.pop_front();
                }

                local_map_ptr->header.stamp = time_stamp.toSec();
                CloudPublisher(local_map_publisher, local_map_ptr);

                global_map_ptr->header.stamp = time_stamp.toSec();
                CloudPublisher(global_map_publisher, global_map_ptr);

                local_box_array.header.stamp = time_stamp;
                local_box_array.header.frame_id = config.getString("bounding_box_publish_frame_id");
                bbox_publisher.publish(local_box_array);

                getOdometryFromMatrix4f(current_pose, output_odometry, time_stamp);
                output.header.frame_id = config.getString("output_publish_frame_id");
                output.header.stamp = time_stamp;
                output.bboxArray = local_box_array;
                output.odometry = output_odometry;
                output_publisher.publish(output);

                ROS_INFO("\n==========\nPreprocess: %fms\nTransform: %fms\nCones: %ld\n==========", \
                    (middle - begin).toSec()*1000, (ros::Time::now() - middle).toSec()*1000, local_box_array.boxes.size());

            }else{
                PushMap(cloud_transformed_ptr, cloud_queue, local_map_ptr, global_map_ptr);

                // ROS_INFO("Didn't add frame. distance: %f", distance);
            }
        }
        rate.sleep();
    }
    return 0;
}
