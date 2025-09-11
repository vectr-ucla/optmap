#include "optmap/optmap.h"

std::string get_save_folder(std::string save_folder) {
    int next = 2;
    std::string new_save_folder = save_folder + "/optmap";
    while (boost::filesystem::exists(new_save_folder)) {
        new_save_folder = save_folder + "/optmap_" + std::to_string(next);
        next++;
    }

    std::filesystem::create_directory(new_save_folder);
    return new_save_folder;
}

bool service_full(OptMapNode* node, optmap::optmap_full::Request& req, optmap::optmap_full::Response& res) {
    std::cout << "\n";
    ROS_INFO("Starting map optimization. \nTotal number of available features: %d \nParameters:\n - num_keyframes=%d\n - save_folder=%s\n - save_features=%d\n - use_initial_sol=%d\n - publish_poses=%d\n - save_poses=%d\n - publish_map=%d\n - save_map=%d\n - publish_scans=%d\n - save_scans=%d\n\n",
            node->get_num_features(), req.num_keyframes, req.output_folder.c_str(), req.save_features, req.use_initial_sol, req.pub_pose, req.save_pose, req.pub_map, req.save_map, req.pub_scans, req.save_scans);
    auto total_start_time = std::chrono::steady_clock::now();
    
    if (req.save_features || req.save_map || req.save_pose || req.save_scans) {
        std::string save_folder = get_save_folder(req.output_folder);
        node->set_save_folder(save_folder);
    }

    node->set_save_features(req.save_features);
    node->set_publish_poses(req.pub_pose);
    node->set_save_poses(req.save_pose);
    node->set_publish_map(req.pub_map);
    node->set_save_map(req.save_map);
    node->set_publish_scans(req.pub_scans);
    node->set_save_scans(req.save_scans);

    node->optimize_streaming(req.num_keyframes);

    auto total_end_time = std::chrono::steady_clock::now();
    std::cout << "OptMap Total Time: " << std::chrono::duration<double, std::milli>(total_end_time - total_start_time).count() << " ms" << std::endl;

    res.success = true;
    return res.success;
}

bool service_position(OptMapNode* node, optmap::optmap_position::Request& req, optmap::optmap_position::Response& res) {
    std::cout << "\n";
    ROS_INFO("Starting map optimization. \nTotal number of available features: %d \nParameters:\n - num_keyframes=%d\n - save_folder=%s\n - save_features=%d\n - use_initial_sol=%d\n - publish_poses=%d\n - save_poses=%d\n - publish_map=%d\n - save_map=%d\n - publish_scans=%d\n - save_scans=%d\n\n",
            node->get_num_features(), req.num_keyframes, req.output_folder.c_str(), req.save_features, req.use_initial_sol, req.pub_pose, req.save_pose, req.pub_map, req.save_map, req.pub_scans, req.save_scans);
    auto total_start_time = std::chrono::steady_clock::now();
    
    std::vector<float> x;
    x.insert(x.end(), req.x.begin(), req.x.end());
    std::vector<float> y;
    y.insert(y.end(), req.y.begin(), req.y.end());
    std::vector<float> z;
    z.insert(z.end(), req.z.begin(), req.z.end());
    std::vector<float> r;
    r.insert(r.end(), req.r.begin(), req.r.end());

    if (req.save_features || req.save_map || req.save_pose || req.save_scans) {
        std::string save_folder = get_save_folder(req.output_folder);
        node->set_save_folder(save_folder);
    }

    node->set_save_features(req.save_features);
    node->set_publish_poses(req.pub_pose);
    node->set_save_poses(req.save_pose);
    node->set_publish_map(req.pub_map);
    node->set_save_map(req.save_map);
    node->set_publish_scans(req.pub_scans);
    node->set_save_scans(req.save_scans);

    node->optimize_streaming(req.num_keyframes, &x, &y, &z, &r);

    auto total_end_time = std::chrono::steady_clock::now();
    std::cout << "OptMap Total Time: " << std::chrono::duration<double, std::milli>(total_end_time - total_start_time).count() << " ms" << std::endl;

    res.success = true;
    return res.success;
}

bool service_position_and_time(OptMapNode* node, optmap::optmap_position_and_time::Request& req, optmap::optmap_position_and_time::Response& res) {
    std::cout << "\n";
    ROS_INFO("Starting map optimization. \nTotal number of available features: %d \nParameters:\n - num_keyframes=%d\n - save_folder=%s\n - save_features=%d\n - use_initial_sol=%d\n - publish_poses=%d\n - save_poses=%d\n - publish_map=%d\n - save_map=%d\n - publish_scans=%d\n - save_scans=%d\n\n",
            node->get_num_features(), req.num_keyframes, req.output_folder.c_str(), req.save_features, req.use_initial_sol, req.pub_pose, req.save_pose, req.pub_map, req.save_map, req.pub_scans, req.save_scans);
    auto total_start_time = std::chrono::steady_clock::now();
    
    std::vector<float> x;
    x.insert(x.end(), req.x.begin(), req.x.end());
    std::vector<float> y;
    y.insert(y.end(), req.y.begin(), req.y.end());
    std::vector<float> z;
    z.insert(z.end(), req.z.begin(), req.z.end());
    std::vector<float> r;
    r.insert(r.end(), req.r.begin(), req.r.end());

    if (req.save_features || req.save_map || req.save_pose || req.save_scans) {
        std::string save_folder = get_save_folder(req.output_folder);
        node->set_save_folder(save_folder);
    }

    node->set_save_features(req.save_features);
    node->set_publish_poses(req.pub_pose);
    node->set_save_poses(req.save_pose);
    node->set_publish_map(req.pub_map);
    node->set_save_map(req.save_map);
    node->set_publish_scans(req.pub_scans);
    node->set_save_scans(req.save_scans);

    node->optimize_streaming(req.num_keyframes, &x, &y, &z, &r, req.t1, req.t2);

    auto total_end_time = std::chrono::steady_clock::now();
    std::cout << "OptMap Total Time: " << std::chrono::duration<double, std::milli>(total_end_time - total_start_time).count() << " ms" << std::endl;

    res.success = true;
    return res.success;
}

bool service_set_voxelization(OptMapNode* node, optmap::optmap_set_voxelization::Request& req, optmap::optmap_set_voxelization::Response& res) {
    ROS_INFO("Setting voxelization to %f\n", req.voxelization);

    node->set_voxelization(req.voxelization);
    
    res.success = true;
    return res.success;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "optmap_node");
    ros::NodeHandle nh("~");

    OptMapNode node(nh);
    ros::ServiceServer serviceF = nh.advertiseService<optmap::optmap_full::Request,
                                                     optmap::optmap_full::Response>("optmap_full", boost::bind(service_full, &node, _1, _2));
    ros::ServiceServer serviceP = nh.advertiseService<optmap::optmap_position::Request,
                                                     optmap::optmap_position::Response>("optmap_position", boost::bind(service_position, &node, _1, _2));
    ros::ServiceServer servicePT = nh.advertiseService<optmap::optmap_position_and_time::Request,
                                                     optmap::optmap_position_and_time::Response>("optmap_position_and_time", boost::bind(service_position_and_time, &node, _1, _2));
    ros::ServiceServer serviceVoxel = nh.advertiseService<optmap::optmap_set_voxelization::Request,
                                                     optmap::optmap_set_voxelization::Response>("optmap_set_voxelization", boost::bind(service_set_voxelization, &node, _1, _2));

    ros::spin();
    return 0;
}