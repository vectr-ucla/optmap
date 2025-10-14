#include "optmap/optmap.h"

using std::placeholders::_1;
using std::placeholders::_2;

std::string get_save_folder(std::string save_folder) {
    int next = 2;
    std::string new_save_folder = save_folder + "/optmap";
    while (std::filesystem::exists(new_save_folder)) {
        new_save_folder = save_folder + "/optmap_" + std::to_string(next);
        next++;
    }

    std::filesystem::create_directory(new_save_folder);
    return new_save_folder;
}

void service_full(OptMapNode* node, const std::shared_ptr<custom_interfaces::srv::OptmapFull::Request> req, std::shared_ptr<custom_interfaces::srv::OptmapFull::Response> res) {
    std::cout << "\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Starting map optimization. \nTotal number of available features: %d \nParameters:\n - num_scans=%d\n - save_folder=%s\n - save_features=%d\n - publish_poses=%d\n - save_poses=%d\n - publish_map=%d\n - save_map=%d\n - publish_scans=%d\n - save_scans=%d\n\n",
            node->get_num_features(), req->num_scans, req->output_folder.c_str(), req->save_features, req->pub_pose, req->save_pose, req->pub_map, req->save_map, req->pub_scans, req->save_scans);
    auto total_start_time = std::chrono::steady_clock::now();
    
    if (req->save_features || req->save_map || req->save_pose || req->save_scans) {
        std::string save_folder = get_save_folder(req->output_folder);
        node->set_save_folder(save_folder);
    }

    node->set_save_features(req->save_features);
    node->set_publish_poses(req->pub_pose);
    node->set_save_poses(req->save_pose);
    node->set_publish_map(req->pub_map);
    node->set_save_map(req->save_map);
    node->set_publish_scans(req->pub_scans);
    node->set_save_scans(req->save_scans);

    node->optimize_streaming(req->num_scans);

    auto total_end_time = std::chrono::steady_clock::now();
    std::cout << "OptMap Total Time: " << std::chrono::duration<double, std::milli>(total_end_time - total_start_time).count() << " ms" << std::endl;

    res->success = true;
    if (res->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "true.");
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "OptMap Service Full Failed!");
    }
}

void service_position(OptMapNode* node, const std::shared_ptr<custom_interfaces::srv::OptmapPosition::Request> req, std::shared_ptr<custom_interfaces::srv::OptmapPosition::Response> res) {
    std::cout << "\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Starting map optimization. \nTotal number of available features: %d \nParameters:\n - num_scans=%d\n - save_folder=%s\n - save_features=%d\n - publish_poses=%d\n - save_poses=%d\n - publish_map=%d\n - save_map=%d\n - publish_scans=%d\n - save_scans=%d\n\n",
            node->get_num_features(), req->num_scans, req->output_folder.c_str(), req->save_features, req->pub_pose, req->save_pose, req->pub_map, req->save_map, req->pub_scans, req->save_scans);
    auto total_start_time = std::chrono::steady_clock::now();
    
    std::vector<float> x;
    x.insert(x.end(), req->x.begin(), req->x.end());
    std::vector<float> y;
    y.insert(y.end(), req->y.begin(), req->y.end());
    std::vector<float> z;
    z.insert(z.end(), req->z.begin(), req->z.end());
    std::vector<float> r;
    r.insert(r.end(), req->r.begin(), req->r.end());

    if (req->save_features || req->save_map || req->save_pose || req->save_scans) {
        std::string save_folder = get_save_folder(req->output_folder);
        node->set_save_folder(save_folder);
    }

    node->set_save_features(req->save_features);
    node->set_publish_poses(req->pub_pose);
    node->set_save_poses(req->save_pose);
    node->set_publish_map(req->pub_map);
    node->set_save_map(req->save_map);
    node->set_publish_scans(req->pub_scans);
    node->set_save_scans(req->save_scans);

    node->optimize_streaming(req->num_scans, &x, &y, &z, &r);

    auto total_end_time = std::chrono::steady_clock::now();
    std::cout << "OptMap Total Time: " << std::chrono::duration<double, std::milli>(total_end_time - total_start_time).count() << " ms" << std::endl;

    res->success = true;
    if (res->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "true.");
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "OptMap Service Position Failed!");
    }
}

void service_position_and_time(OptMapNode* node, const std::shared_ptr<custom_interfaces::srv::OptmapPositionAndTime::Request> req, std::shared_ptr<custom_interfaces::srv::OptmapPositionAndTime::Response> res) {
    std::cout << "\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Starting map optimization. \nTotal number of available features: %d \nParameters:\n - num_scans=%d\n - save_folder=%s\n - save_features=%d\n - publish_poses=%d\n - save_poses=%d\n - publish_map=%d\n - save_map=%d\n - publish_scans=%d\n - save_scans=%d\n\n",
            node->get_num_features(), req->num_scans, req->output_folder.c_str(), req->save_features, req->pub_pose, req->save_pose, req->pub_map, req->save_map, req->pub_scans, req->save_scans);
    auto total_start_time = std::chrono::steady_clock::now();
    
    std::vector<float> x;
    x.insert(x.end(), req->x.begin(), req->x.end());
    std::vector<float> y;
    y.insert(y.end(), req->y.begin(), req->y.end());
    std::vector<float> z;
    z.insert(z.end(), req->z.begin(), req->z.end());
    std::vector<float> r;
    r.insert(r.end(), req->r.begin(), req->r.end());

    if (req->save_features || req->save_map || req->save_pose || req->save_scans) {
        std::string save_folder = get_save_folder(req->output_folder);
        node->set_save_folder(save_folder);
    }

    node->set_save_features(req->save_features);
    node->set_publish_poses(req->pub_pose);
    node->set_save_poses(req->save_pose);
    node->set_publish_map(req->pub_map);
    node->set_save_map(req->save_map);
    node->set_publish_scans(req->pub_scans);
    node->set_save_scans(req->save_scans);

    node->optimize_streaming(req->num_scans, &x, &y, &z, &r, req->t1, req->t2);

    auto total_end_time = std::chrono::steady_clock::now();
    std::cout << "OptMap Total Time: " << std::chrono::duration<double, std::milli>(total_end_time - total_start_time).count() << " ms" << std::endl;

    res->success = true;
    if (res->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "true.");
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "OptMap Service Position and Time Failed!");
    }
}

void service_set_voxelization(OptMapNode* node, const std::shared_ptr<custom_interfaces::srv::OptmapSetVoxelization::Request> req, std::shared_ptr<custom_interfaces::srv::OptmapSetVoxelization::Response> res) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Setting voxelization to %f\n", req->voxelization);

    node->set_voxelization(req->voxelization);
    
    res->success = true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OptMapNode>();

    rclcpp::Service<custom_interfaces::srv::OptmapFull>::SharedPtr serviceF = 
        node->create_service<custom_interfaces::srv::OptmapFull>("optmap_full", std::bind(service_full, node.get(), _1, _2));
    rclcpp::Service<custom_interfaces::srv::OptmapPosition>::SharedPtr serviceP = 
        node->create_service<custom_interfaces::srv::OptmapPosition>("optmap_position", std::bind(service_position, node.get(), _1, _2));
    rclcpp::Service<custom_interfaces::srv::OptmapPositionAndTime>::SharedPtr servicePT = 
        node->create_service<custom_interfaces::srv::OptmapPositionAndTime>("optmap_position_and_time", std::bind(service_position_and_time, node.get(), _1, _2));
    rclcpp::Service<custom_interfaces::srv::OptmapSetVoxelization>::SharedPtr serviceVoxel = 
        node->create_service<custom_interfaces::srv::OptmapSetVoxelization>("optmap_set_voxelization", std::bind(service_set_voxelization, node.get(), _1, _2));
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
}
