#include "optmap/optmap.h"

OptMapNode::OptMapNode(ros::NodeHandle node_handle) : nh(node_handle) {
    this->num_threads = omp_get_max_threads();
    ros::param::param<std::string>("~optmap/version", this->version, "1.0.0");
    ros::param::param<bool>("~optmap/debug", this->debug, false);
    ros::param::param<float>("~optmap/fraction_eval_scans", this->fraction_eval_scans, 0.1);
    ros::param::param<float>("~optmap/voxelize", this->voxelize, 0.5);
    ros::param::param<int>("~optmap/unchanged_iters_max", this->unchanged_iters_max, 100);
    ros::param::param<bool>("~optmap/dynamic_reordering", this->d_r, true);
    ros::param::param<float>("~optmap/sorted_set_factor", this->sorted_set_factor, 1.5);
    ros::param::param<float>("~optmap/dist_heur_radius", this->dist_heur_radius, 1.0);
    ros::param::param<std::string>("~optmap/frames/map", this->map_frame, "map");
    ros::param::param<float>("~optmap/box_min_x", this->x_min, -std::numeric_limits<float>::infinity());
    ros::param::param<float>("~optmap/box_max_x", this->x_max, std::numeric_limits<float>::infinity());
    ros::param::param<float>("~optmap/box_min_y", this->y_min, -std::numeric_limits<float>::infinity());
    ros::param::param<float>("~optmap/box_max_y", this->y_max, std::numeric_limits<float>::infinity());
    ros::param::param<float>("~optmap/box_min_z", this->z_min, -std::numeric_limits<float>::infinity());
    ros::param::param<float>("~optmap/box_max_z", this->z_max, std::numeric_limits<float>::infinity());

    this->pose_sub = this->nh.subscribe("pose", 1000, &OptMapNode::callbackPose, this, ros::TransportHints().tcpNoDelay());
    this->descriptor_sub = this->nh.subscribe("descriptor", 1000, &OptMapNode::callbackDescriptor, this, ros::TransportHints().tcpNoDelay());
    this->pc_sub = this->nh.subscribe("pointcloud", 20, &OptMapNode::callbackPointCloud, this, ros::TransportHints().tcpNoDelay());
    this->pose_updates_sub = this->nh.subscribe("pose_updates", 1000, &OptMapNode::callbackPoseUpdates, this, ros::TransportHints().tcpNoDelay());

    this->map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("map", 1, true);
    this->map_scans_pub = this->nh.advertise<sensor_msgs::PointCloud2>("map_scans", 1, true);
    this->poses_pub = this->nh.advertise<geometry_msgs::PoseArray>("poses", 1000, true);
    this->markers_pub = this->nh.advertise<visualization_msgs::MarkerArray>("markers", 1000, true);

    this->save_folder = ".";
    this->save_features = false;
    this->publish_poses = false;
    this->save_poses = false;
    this->publish_map = false;
    this->save_map = false;
    this->publish_scans = false;
    this->save_scans = false;

    this->most_recent_pose = -1;
    this->most_recent_desc = -1;
    this->most_recent_cloud = -1;

    this->optimization_done = false;
    this->optimization_score = std::numeric_limits<float>::infinity();
    this->worker_last_seen_optimization_score = std::numeric_limits<float>::infinity();

    Eigen::Vector3f last_del_position;
    last_del_position.x() = 0; last_del_position.y() = 0; last_del_position.z() = 0;
    Eigen::Quaternionf last_del_orientation;
    last_del_orientation.w() = 1; last_del_orientation.x() = 0; last_del_orientation.y() = 0; last_del_orientation.z() = 0;
    this->last_del_pose = std::make_pair(last_del_position, last_del_orientation);
    this->last_updated_pose_index = -1;

    ros::Time current_time = ros::Time::now();
    this->pc_foldername = "./temp-" + std::to_string(current_time.sec) + "-" + std::to_string(current_time.nsec);
    std::filesystem::create_directory(this->pc_foldername);
}

OptMapNode::~OptMapNode() {
    clear_temp_pc_storage();
}

void OptMapNode::clear_temp_pc_storage() {
    int ret = std::filesystem::remove_all(this->pc_foldername);
    std::cout << "Deleted " << ret << " temporary items." << std::endl;
    if (ret == 0) {
        ROS_INFO("Failed to delete temporary storage for optmap");
    }
}

void OptMapNode::callbackPose(optmap::OptmapPoseConstPtr msg) {
    int scan_index = msg->id;
    this->most_recent_pose = scan_index;

    Feature::Pose pose {
        Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
        Eigen::Quaternionf(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z)
    };

    featureList.append_feature_data(scan_index, [pose = std::move(pose)](Feature& feature) {
        feature.set_pose(pose);
    });
}

void OptMapNode::callbackDescriptor(optmap::DescriptorConstPtr msg) {
    int scan_index = msg->id;
    this->most_recent_desc = scan_index;

    Feature::Descriptor desc = Eigen::Map<const Eigen::VectorXf>(msg->data.data(), msg->data.size());

    featureList.append_feature_data(scan_index, [desc = std::move(desc)](Feature& feature) {
        feature.set_descriptor(desc);
    });

    screen_output();

    // Apply the latest loop closure del pose to new features
    if ((this->last_updated_pose_index < featureList.get_num_features_all_features() - 1) && (this->last_updated_pose_index != -1)) {
        int k = this->last_updated_pose_index + 1;
        while (k < featureList.get_num_features_all_features()) {
            featureList.at_all_features(k).set_del_pose(this->last_del_pose);
            k++;
        }
    }
}

void OptMapNode::callbackPointCloud(sensor_msgs::PointCloud2::ConstPtr pc) {
    int scan_index = pc->header.seq;
    this->most_recent_cloud = scan_index;
    this->recent_time = pc->header.stamp.toSec();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
    pcl::fromROSMsg(*pc, *cloud);

    // split into 100 subfolders for faster indexing
    std::string folder_path = this->pc_foldername + "/" + std::to_string(scan_index % 100);
    std::string cloud_path = folder_path + "/" + std::to_string(scan_index / 100) + ".pcd";
    try {
        if (!std::filesystem::exists(folder_path)) {
            std::filesystem::create_directories(folder_path);
        }
        int ret = pcl::io::savePCDFileBinary(cloud_path, *cloud);
        if (ret != 0) {
            ROS_ERROR("Did not save deskewed point cloud %d correctly", scan_index);
            throw ret;
        }
    }
    catch (...) {
        ROS_ERROR("Error writing to point cloud storage. Check storage.");
        clear_temp_pc_storage();
        return;
    }

    ros::Time timestamp = pc->header.stamp;
    featureList.append_feature_data(scan_index, [timestamp, cloud_path](Feature& feature) {
        feature.set_timestamp(timestamp);
        feature.set_cloud_path(cloud_path);
    });
}

// Current: only updates position, not orientation.
// We look at features that aren't fully uninitialized becauese
// descriptors may fall behind despite us having corresponding poses
// pose_k = (k-i)/(j-i) * (del_pose_j - del_pose_i) + del_pose_i + pose_k, where del is the change in pose due to the loop closure.
void OptMapNode::callbackPoseUpdates(optmap::OptmapPoseArrayPtr msg) {
    if (msg->poses.size() == 0 || featureList.get_num_features_all_features() == 0) { return; }

    std::unique_lock<std::mutex> featureListLock(featureList.features_mutex);

    // scan_index, del_position
    std::vector<std::pair<int, std::pair<Eigen::Vector3f, Eigen::Quaternionf>>> del_positions;

    // fill del_poses. also insert pose updates at beginning and end for boundary conditions    
    int curr_feature_index = 0;
    for (int i = 0; i < msg->poses.size(); i++) {
        int scan_index = msg->poses.at(i).id;

        Eigen::Vector3f new_position;
        new_position[0] = msg->poses.at(i).pose.position.x;
        new_position[1] = msg->poses.at(i).pose.position.y;
        new_position[2] = msg->poses.at(i).pose.position.z;
        Eigen::Quaternionf new_orientation;
        new_orientation.w() = msg->poses.at(i).pose.orientation.w;
        new_orientation.x() = msg->poses.at(i).pose.orientation.x;
        new_orientation.y() = msg->poses.at(i).pose.orientation.y;
        new_orientation.z() = msg->poses.at(i).pose.orientation.z;

        // increase curr_feature so it is the index of the last feature before the current pose update. but make sure it has a pose
        while (true) {
            int next_feature_index = curr_feature_index + 1;
            
            // find the next feature that has its pose initialized
            while (next_feature_index < featureList.get_num_features_all_features() && !(featureList.at_all_features(next_feature_index).get_init_flags() & Feature::POSE_INIT)) {
                next_feature_index++;
            }

            if (next_feature_index < featureList.get_num_features_all_features() && featureList.at_all_features(next_feature_index).get_scan_index() <= scan_index) {
                curr_feature_index = next_feature_index;
            }
            else {
                break;
            }
        }

        Eigen::Vector3f old_position = featureList.at_all_features(curr_feature_index).get_pose().first;
        Eigen::Quaternionf old_orientation = featureList.at_all_features(curr_feature_index).get_pose().second;
        del_positions.push_back(std::make_pair(scan_index, std::make_pair(new_position - old_position, new_orientation)));
    }

    // you want a del_pose at or before the first feature index so you can interpolate. same with the last feature
    if (msg->poses.at(0).id > featureList.at(0).get_scan_index()) {
        del_positions.insert(del_positions.begin(), std::make_pair(featureList.at(0).get_scan_index(), std::make_pair(del_positions.at(0).second.first, del_positions.at(0).second.second)));
    }
    int last_scan_index = featureList.at_all_features(featureList.get_num_features_all_features() - 1).get_scan_index();
    if (msg->poses.at(msg->poses.size() - 1).id < last_scan_index) {
        del_positions.push_back(std::make_pair(last_scan_index, std::make_pair(del_positions.at(del_positions.size()-1).second.first, del_positions.at(del_positions.size()-1).second.second)));
    }
    if (del_positions.size() == 1) { return; }

    int curr_pose_update = 0;
    for (int k = 0; k < featureList.get_num_features_all_features(); k++) {
        // only increase the curr pose update if there's at least two more updates left so you can interpolate
        while (curr_pose_update+2 < del_positions.size() && featureList.at_all_features(k).get_scan_index() >= del_positions.at(curr_pose_update+1).first) {
            curr_pose_update++;
        }

        int curr_scan_index = featureList.at_all_features(k).get_scan_index();
        int i = del_positions.at(curr_pose_update).first;
        int j = del_positions.at(curr_pose_update+1).first;
        Eigen::Vector3f del_position_i = del_positions.at(curr_pose_update).second.first;
        Eigen::Vector3f del_position_j = del_positions.at(curr_pose_update+1).second.first;
        Eigen::Vector3f del_position = (static_cast<float>(curr_scan_index) - i) / (j - i) * (del_position_j - del_position_i) + del_position_i;
        Eigen::Quaternionf new_orientation_i = del_positions.at(curr_pose_update).second.second;
        Eigen::Quaternionf new_orientation_j = del_positions.at(curr_pose_update+1).second.second;
        Eigen::Quaternionf new_orientation = new_orientation_i.slerp((static_cast<float>(curr_scan_index) - i) / (j - i), new_orientation_j);

        // Find change in quaternion values
        Eigen::Quaternionf old_orientation = featureList.at_all_features(k).get_pose().second;
        Eigen::Quaternionf del_orientation;
        del_orientation.w() = 1;
        del_orientation.x() = 0;
        del_orientation.y() = 0;
        del_orientation.z() = 0; // TODO: Apply orientation updates

        Feature::Pose del_pose = std::make_pair(del_position, del_orientation);
        featureList.at_all_features(k).set_del_pose(del_pose);

        this->last_del_pose = del_pose;
    }

    this->last_updated_pose_index = featureList.get_num_features_all_features() - 1;
}

void OptMapNode::optimize_streaming(int num_scans, std::vector<float>* x, std::vector<float>* y, std::vector<float>* z, std::vector<float>* r, ros::Time t1, ros::Time t2) {
    
    // check if distances done caluclating
    int queue_size = featureList.get_distances_queue_size();
    if (queue_size != 0) {
        ROS_ERROR("Still computing feature distances, try again later. Features left: %d", queue_size);
        return;
    }

    // acquire locks on the features lists so that they cannot be updated while the optimization is running
    std::lock_guard<std::mutex> features_lock(featureList.features_mutex);
    std::lock_guard<std::mutex> distances_lock(featureList.distances_mutex);

    std::vector<std::pair<int, float>> features = featureList.get_features_basic();

    // Apply position and time constraints
    // First check if any position constraints are given
    int num_points;
    if (r == NULL) {
        num_points = 0;
    }
    else {
        num_points = r->size();
    }
    std::vector<Eigen::Vector3f> centers;
    for (int i=0; i<num_points; i++) {
        Eigen::Vector3f center(x->at(i), y->at(i), z->at(i));
        centers.push_back(center);
    }

    // TODO: Apply time constraints even if position constraints are not given
    if (num_points != 0) {
        for (int i = 0; i < features.size(); i++) {
            Feature& feature = featureList.at(features[i].first);
            bool constr_met = false;
            for (int j = 0; j < num_points; j++) {
                if (t1 <= feature.get_timestamp() && feature.get_timestamp() <= t2 && (centers[j] - feature.get_pose().first).norm() <= r->at(j)) {
                    constr_met = true;
                }
                if (constr_met) { break; }
            }
            if (!constr_met) {
                features.erase(features.begin() + i);
                i--;
            }
        }
    }

    if (features.empty()) {
        ROS_WARN("No scans in selected region.");
        return;
    }

    optimize_streaming_helper(features, num_scans);
    auto wait_start_time = std::chrono::steady_clock::now();

    // Wait for map build
    {
        std::lock_guard<std::mutex> lock(build_map_inputs_mtx);
        optimization_done = true;
        build_map_cv.notify_one();
    }
    if (build_map_thread.joinable()) {
        build_map_thread.join();
    }

    auto wait_end_time = std::chrono::steady_clock::now();
    std::cout << "Time waiting for map build: " << std::chrono::duration<double, std::milli>(wait_end_time-wait_start_time).count() << "\n\n";
    if (debug) {
        this->tot_num_features = featureList.get_num_features();
        this->num_features = features.size();
        this->map_build_time = std::chrono::duration<double, std::milli>(wait_end_time - wait_start_time).count();
    }

    // output
    if (this->save_features) { this->save_scans = true; } // save pointclouds if saving features

    if (this->publish_map) {
        sensor_msgs::PointCloud2 cloud_ros;
        pcl::toROSMsg(*result_map, cloud_ros);
        cloud_ros.header.frame_id = this->map_frame;
        this->map_pub.publish(cloud_ros);
        
        ROS_INFO("Published map.");
    }

    if (this->save_map) {
        std::string save_path = save_folder + "/optmap.pcd";
        pcl::io::savePCDFileBinary(save_path, *result_map);

        ROS_INFO("Saved map to %s.", save_path.c_str());
    }

    if (this->publish_poses || this->save_poses) {
        build_pose_array(result_indices);
    }

    if (this->publish_scans || this->save_scans) {
        export_scans(result_indices);
    }

    if (this->save_features) {
        save_features_to_folder(result_indices);
    }

    if (this->debug) {
        Debug();
    }

    std::cout << std::endl;
}

void OptMapNode::optimize_streaming_helper(std::vector<std::pair<int,float>> features, int num_scans) {
    #pragma omp declare reduction(min_score_and_index : OptMapNode::ScoreIndexPair : \
        omp_out = (omp_out.score < omp_in.score) ? omp_out : omp_in) \
        initializer(omp_priv = {std::numeric_limits<float>::infinity(), -1})

    auto start_time = std::chrono::steady_clock::now();

    if (features.size() < num_scans) {
        num_scans = features.size();
        ROS_WARN("Requested optmap output with more scans than are currently available");
    }

    // algorithm parameters
    float beta = 0.5f;      // required percentage of OPT to add it as a scan
    float epsilon = 0.01f;  // used to decide how many values of OPT are tested
    float max_score = 1.f;  // L(e0), value of exemplar-based clustering loss function for auxilary element. Should be the maximum possible loss.
    float max_dist = 1.f;   // d(e0, v) for any v in S. Also the maximum distance, since this will maximize the loss.

    // Extract the feature indices and generate a shuffled list
    std::vector<int> feature_indices_(features.size());
    for (int i = 0; i < features.size(); i++) {
        feature_indices_[i] = features[i].first;
    }
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(features.begin(), features.end(), g);
    std::vector<int> feature_indices_shuffled(features.size());
    for (int i = 0; i < features.size(); i++) {
        feature_indices_shuffled[i] = features[i].first;
    }
    
    // Evaluation variables (num_eval_scans <= feature_indices_shuffled.size() depending on fraction_eval_scans)
    int num_eval_scans = (feature_indices_shuffled.size() * fraction_eval_scans > 1) ? feature_indices_shuffled.size() * fraction_eval_scans : feature_indices_shuffled.size();
    float default_score = max_score;

    // Initial solution evenly spaces scans based on total trajectory distance (between descriptors)
    auto start_time_init_sol = std::chrono::steady_clock::now();
    std::vector<int> heur_sol = gen_heur_sol(feature_indices_, num_scans);
    auto end_time_init_sol = std::chrono::steady_clock::now();
    if (debug) {
        this->init_sol_time = std::chrono::duration<double, std::milli>(end_time_init_sol - start_time_init_sol).count();
    }
    
    // Find value of initial solution in a batch
    float heur_sol_score = 0;
    float sample_traj_dist = 0;
    for (int i=0; i<num_eval_scans; i++) {
        float min_dist = 1;
        float element_weight = featureList.get_element_weighting(feature_indices_shuffled[i]);
        sample_traj_dist += element_weight; // weighting is dist from previous element in traj
        for (int j=0; j<heur_sol.size(); j++) {
            float dist = featureList.get_distance_between_descriptors(feature_indices_shuffled[i], heur_sol[j]);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        heur_sol_score += min_dist * element_weight;
    }

    // set of OPT values
    float opt_lower_bound = max_score - heur_sol_score / sample_traj_dist;
    float opt_upper_bound = max_score;

    if (debug) {
        this->beta_output = beta;
        this->num_scans_output = num_scans;
        this->lower_bound = opt_lower_bound;
        this->upper_bound = opt_upper_bound;
    }

    int begin = std::ceil(std::log(opt_lower_bound) / std::log(1+epsilon));
    int end = std::floor(std::log(opt_upper_bound) / std::log(1+epsilon));
    std::vector<float> O;
    for (int i = begin; i <= end; i++) {
        O.push_back(std::pow(1+epsilon, i));
    }

    // scan data for each threshold 
    std::vector<std::vector<int>> curr_feature_indices(O.size());
    std::vector<float> prev_scores(O.size(), default_score);
    std::vector<std::vector<float>> prev_min_dists(O.size(), std::vector<float>(features.size(), max_dist));

    // The following three partitions determine stream order and are dynamically ordered during optimization
    int sorted_partition_size;
    if (d_r) {
        if (sorted_set_factor == -1) {
            sorted_partition_size = features.size();
        }
        else {
            sorted_partition_size = std::min(static_cast<int>(sorted_set_factor * num_scans), static_cast<int>(features.size()));
        }
    }
    else {
        sorted_partition_size = features.size();
    }
    std::vector<std::pair<int,float>> unsorted_partition = features;
    for (int i=0; i<unsorted_partition.size(); i++) {
        unsorted_partition[i].second = 2;   // Ignore weights in ordering
    }
    std::vector<std::pair<int,float>> sorted_partition;
    if (unsorted_partition.size() < sorted_partition_size) {
        sorted_partition = unsorted_partition;
        unsorted_partition.clear();
    }
    else {
        sorted_partition.insert(sorted_partition.begin(), unsorted_partition.end() - sorted_partition_size, unsorted_partition.end());
        unsorted_partition.erase(unsorted_partition.end() - sorted_partition_size, unsorted_partition.end());
    }
    std::vector<std::pair<int,float>> checked_partition;

    // Make an unordered map for feature_indices_shuffled
    std::unordered_map<int, size_t> fis_id_to_index;
    for (size_t i = 0; i < feature_indices_shuffled.size(); i++) {
        fis_id_to_index[feature_indices_shuffled[i]] = i;
    }

    // The following three arrays should match the previous three partitions in content and order
    std::vector<std::vector<float>> expected_values_unsrt(features.size(), std::vector<float>(O.size(), 2));
    std::vector<std::vector<float>> expected_values_srt;
    if (expected_values_unsrt.size() < sorted_partition_size) {
        expected_values_srt = expected_values_unsrt;
        expected_values_unsrt.clear();
    }
    else {
        expected_values_srt.insert(expected_values_srt.begin(), expected_values_unsrt.end() - sorted_partition_size, expected_values_unsrt.end());
        expected_values_unsrt.erase(expected_values_unsrt.end() - sorted_partition_size, expected_values_unsrt.end());
    }
    
    // set up map building thread
    optimization_done = false;
    optimization_score = std::numeric_limits<float>::infinity();
    worker_last_seen_optimization_score = std::numeric_limits<float>::infinity();
    if (!build_map_thread.joinable()) {
        build_map_thread = std::thread(&OptMapNode::build_map_worker, this);
    }

    // after how many iterations of having the same solution is the worker thread notified
    int unchanged_iters_thresh = std::min(unchanged_iters_max, (int) std::ceil(feature_indices_shuffled.size() / num_scans));

    int num_full_solutions = 0;
    int best_index = -1;
    int unchanged_iters = 0;
    for (int i = 0; i < feature_indices_shuffled.size(); i++) {

        if (num_full_solutions >= O.size()) {
            break;
        }

        // Refill sorted list if empty
        if (sorted_partition.size() == 0) {
            int new_srt_size = std::min(sorted_partition_size, static_cast<int>(unsorted_partition.size()));
            int new_unsrt_size = unsorted_partition.size() - new_srt_size;

            // Update the scores for the soon to be added portions of the unsorted partition
            for (int j=0; j<new_srt_size; j++) {
                unsorted_partition[j].second = (std::accumulate(expected_values_unsrt[j].begin(), expected_values_unsrt[j].end(), 0.0f) / expected_values_unsrt[j].size());
            }

            // sort sorted partition and sorted expected values
            std::vector<int> ind(new_srt_size);
            std::iota(ind.begin(), ind.end(), new_srt_size);
            std::sort(ind.begin(), ind.end(), [&](int a, int b) {
                return unsorted_partition[a].second < unsorted_partition[b].second;
            });
            for (int j = 0; j < ind.size(); j++) {
                sorted_partition.push_back(unsorted_partition[ind[j] + new_unsrt_size]);
                expected_values_srt.push_back(expected_values_unsrt[ind[j] + new_unsrt_size]);
            }
            unsorted_partition.erase(unsorted_partition.end() - new_srt_size, unsorted_partition.end());
            expected_values_unsrt.erase(expected_values_unsrt.end() - new_srt_size, expected_values_unsrt.end());
        }

        // Next stream element is the highest expected value in sorted partition
        int index = sorted_partition.back().first;
        checked_partition.push_back(sorted_partition.back());
        sorted_partition.pop_back();
        expected_values_srt.pop_back();

        ScoreIndexPair iter_min = {std::numeric_limits<float>::infinity(), -1};

        #pragma omp parallel for num_threads(this->num_threads) schedule(dynamic, 4) reduction(min_score_and_index:iter_min)
        for (int j = 0; j < O.size(); j++) {

            // skip if cardinality constraint or OPT reached
            if (curr_feature_indices[j].size() >= num_scans) {
                continue;
            }

            // test adding the current scan as a scan
            float exemp_score = oracle_exemplar(index, feature_indices_shuffled, num_eval_scans, prev_min_dists[j], prev_scores[j], sample_traj_dist);
            float marginal_gain = prev_scores[j] - exemp_score;  // (max_score - exemp_score) - (max_score - prev_scores[j])

            float threshold = (O[j]*beta - (max_score - prev_scores[j])) / (num_scans - curr_feature_indices[j].size());
            
            if (marginal_gain >= threshold) {
                // Loop over all elements now to update expected values
                for (int k = 0; k < sorted_partition.size(); k++) {
                    float dist = featureList.get_distance_between_descriptors(index, sorted_partition[k].first);
                    int fis_index = fis_id_to_index[sorted_partition[k].first];
                    if (dist < prev_min_dists[j][fis_index]){
                        prev_min_dists[j][fis_index] = dist;
                    }
                    if (d_r) {
                        float overlap_desc = overlap_from_desc(dist);
                        float overlap_pose = overlap_from_pose(featureList.at(index).get_pose(), featureList.at(sorted_partition[k].first).get_pose());
                        float full_value = 1;
                        float disc_value = full_value * (1-overlap_desc) + full_value * (1-overlap_pose);
                        float cur_value = expected_values_srt[k][j];
                        if (disc_value < cur_value) {
                            expected_values_srt[k][j] = disc_value;
                        }
                    }
                }
                for (int k = 0; k < unsorted_partition.size(); k++) {
                    float dist = featureList.get_distance_between_descriptors(index, unsorted_partition[k].first);
                    int fis_index = fis_id_to_index[unsorted_partition[k].first];
                    if (dist < prev_min_dists[j][fis_index]){
                        prev_min_dists[j][fis_index] = dist;
                    }
                    float overlap_desc = overlap_from_desc(dist);
                    float overlap_pose = overlap_from_pose(featureList.at(index).get_pose(), featureList.at(unsorted_partition[k].first).get_pose());
                    float full_value = 1;
                    float disc_value = full_value * (1-overlap_desc) + full_value * (1-overlap_pose);
                    float cur_value = expected_values_unsrt[k][j];
                    if (disc_value < cur_value) {
                        expected_values_unsrt[k][j] = disc_value;
                    }
                }
                for (int k = 0; k < checked_partition.size(); k++) {
                    float dist = featureList.get_distance_between_descriptors(index, checked_partition[k].first);
                    int fis_index = fis_id_to_index[checked_partition[k].first];
                    if (dist < prev_min_dists[j][fis_index]){
                        prev_min_dists[j][fis_index] = dist;
                    }
                }

                prev_scores[j] = exemp_score;
                curr_feature_indices[j].push_back(index);

                if (curr_feature_indices[j].size() >= num_scans) {
                    #pragma omp atomic
                    num_full_solutions++;
                }

                // Perform reduction on prev_scores[j]
                if (exemp_score < iter_min.score) {
                    iter_min.score = prev_scores[j];
                    iter_min.index = j;
                }
            }
        }

        #pragma omp parallel for num_threads(this->num_threads) schedule(dynamic, 4)
        for (int j=0; j<sorted_partition.size(); j++) {
            // The heuristic score can be the average of the expected scores, the min, or something else
            sorted_partition[j].second = (std::accumulate(expected_values_srt[j].begin(), expected_values_srt[j].end(), 0.0f) / expected_values_srt[j].size());
            // sorted_partition[j].second = *std::min_element(expected_values_srt[j].begin(), expected_values_srt[j].end());
        }

        // Re-sort the sorted partition
        if (d_r) {
            std::vector<int> ind(sorted_partition.size());
            std::iota(ind.begin(), ind.end(), 0);
            std::sort(ind.begin(), ind.end(), [&](int a, int b) {
                return sorted_partition[a].second < sorted_partition[b].second;
            });
            std::vector<std::pair<int,float>> sorted_partition_temp(sorted_partition.size());
            std::vector<std::vector<float>> expected_values_srt_temp(sorted_partition.size(), std::vector<float>(O.size(), 0));
            for (int j = 0; j < ind.size(); j++) {
                sorted_partition_temp[j] = sorted_partition[ind[j]];
                expected_values_srt_temp[j] = expected_values_srt[ind[j]];
            }
            sorted_partition = sorted_partition_temp;
            expected_values_srt = expected_values_srt_temp;
        }

        // update best solution
        if (iter_min.score < optimization_score) {
            best_index = iter_min.index;
            unchanged_iters = 0;
        }
        else {
            unchanged_iters += 1;

            // Notify worker thread if solution hasn't changed in "long enough"
            if (unchanged_iters == unchanged_iters_thresh) {
                {
                    std::lock_guard<std::mutex> lock(build_map_inputs_mtx);
                    result_indices = curr_feature_indices[best_index];
                }
                optimization_score = prev_scores[best_index];
                build_map_cv.notify_one();
            }
        }
    }

    // Notify at the end if hasn't notified for this solution
    if (unchanged_iters < unchanged_iters_thresh) {
        {
            std::lock_guard<std::mutex> lock(build_map_inputs_mtx);
            result_indices = curr_feature_indices[best_index];
        }
        optimization_score = prev_scores[best_index];
        build_map_cv.notify_one();
    }

    auto end_time = std::chrono::steady_clock::now();

    std::cerr << "Selected feature indices: ";
    std::sort(result_indices.begin(), result_indices.end());
    for (int index : result_indices) {
        std::cerr << index << " ";
    }
    std::cerr << "\n";
    std::cerr << "score: " << (max_score - optimization_score) << "\n";
    std::cerr << "optimization time: " << std::chrono::duration<double, std::milli>(end_time - start_time).count() << std::endl;

    if (debug) {
        this->num_sols = O.size();
        this->best_sol_opt_guess = O[best_index];
        this->best_sol_score = (max_score - optimization_score);
        this->size_of_best_sol = curr_feature_indices[best_index].size();
        this->init_sol_time = init_sol_time;
        this->optimization_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    }
}

std::vector<int> OptMapNode::gen_heur_sol(const std::vector<int>& eval_feature_indices, int num_scans) {
    // Find the total trajectory distance in descriptor space
    float traj_length = 0;
    for (int i=0; i<eval_feature_indices.size(); i++) {
        traj_length += featureList.get_element_weighting(eval_feature_indices[i]);
    }

    // Select scans based on equaidistant sampling
    std::vector<int> init_sol;
    init_sol.push_back(eval_feature_indices[0]);
    float dist_thresh = traj_length / static_cast<float>(num_scans);
    float dist = 0;
    for (int i=1; i<eval_feature_indices.size(); i++) {
        dist += featureList.get_distance_between_descriptors(eval_feature_indices[i], eval_feature_indices[i-1]);
        if (dist >= dist_thresh) {
            init_sol.push_back(eval_feature_indices[i]);
            dist = 0;
        }
    }
    if (init_sol.size() < num_scans) {
        init_sol.push_back(eval_feature_indices.back());
    }

    return init_sol;
}

float OptMapNode::overlap_from_desc(float dist) {
    if (dist > 1.1) {
        return 0;
    }
    else {
        // fourth order polynomial fit to overlap area of spherical caps on 255-hypersphere
        return std::min(1., 0.7237*std::pow(dist, 4) - 2.4406*std::pow(dist, 3) + 1.1421*std::pow(dist, 2) - 0.3387*dist + 1.0109);
    }
}

float OptMapNode::overlap_from_pose(const Feature::Pose& pose1, const Feature::Pose& pose2) {
    Eigen::Vector3f pos_diff = pose1.first - pose2.first;
    float dist = pos_diff.norm();
    if (dist > (0.9 * this->dist_heur_radius)) {
        return 0;
    }
    else {
        return (-std::log10((dist/this->dist_heur_radius)+0.1));
    }
}

void OptMapNode::build_map_worker() {
    while (true) {
        std::unique_lock<std::mutex> lock(build_map_inputs_mtx);

        // if just waiting on optimization to finish, sleep
        if (!optimization_done && optimization_score == worker_last_seen_optimization_score) {
            build_map_cv.wait(lock, [this]{
                return (optimization_done || optimization_score != worker_last_seen_optimization_score);
            });
        }

        std::vector<int> feature_indices = result_indices;
        lock.unlock();

        // done
        if (optimization_done && optimization_score == worker_last_seen_optimization_score) {
            break;
        }

        worker_last_seen_optimization_score = optimization_score;
        result_map = build_pointcloud_map(feature_indices);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OptMapNode::build_pointcloud_map(const std::vector<int>& feature_indices) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr map (boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>());

    std::optional<Feature::Pose> del_pose;
    for (int index : feature_indices) {
        Feature& feature = featureList.at(index);

        // cancel execution if a new lowest score was computed
        if (optimization_score != worker_last_seen_optimization_score) {
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
        std::string cloud_path = feature.get_cloud_path();
        pcl::io::loadPCDFile<pcl::PointXYZ> (cloud_path, *pc);

        if (this->voxelize >= 0.05) {
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setLeafSize(this->voxelize, this->voxelize, this->voxelize);
            vg.setInputCloud(pc);
            vg.filter(*pc);
        }

        // filter out points outside bounding box
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setMin(Eigen::Vector4f(this->x_min, this->y_min, this->z_min, 1.0));
        crop.setMax(Eigen::Vector4f(this->x_max, this->y_max, this->z_max, 1.0));
        crop.setInputCloud(pc);
        crop.filter(*pc);

        // if there's a pose update
        if (feature.get_del_pose().has_value()) {
            del_pose = feature.get_del_pose().value();
        }

        if (del_pose.has_value()) {
            // TODO: Apply correct orientation updates
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block(0, 3, 3, 1) = del_pose.value().first;
            T.block(0, 0, 3, 3) = del_pose.value().second.toRotationMatrix();

            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*pc, *transformed_cloud, T);
            pc = transformed_cloud;
        }

        *map += *pc;
    }

    return map;
}

geometry_msgs::PoseArray::Ptr OptMapNode::build_pose_array(const std::vector<int>& feature_indices) {
    geometry_msgs::PoseArray::Ptr pa (boost::make_shared<geometry_msgs::PoseArray>());
    pa->header.frame_id = this->map_frame;
    
    visualization_msgs::MarkerArray mArray;
    visualization_msgs::Marker node;
    node.header.frame_id = this->map_frame;
    node.header.stamp = ros::Time::now();
    node.action = visualization_msgs::Marker::ADD;
    node.type = visualization_msgs::Marker::SPHERE_LIST;
    node.ns = "nodes";
    node.id = 0;
    node.pose.orientation.w = 1;
    node.scale.x = 1; node.scale.y = 1; node.scale.z = 1;
    node.color.r = 0; node.color.g = 1; node.color.b = 0;
    node.color.a = 0.75;
    node.lifetime = ros::Duration();

    {
        for (int feature_index : feature_indices) {
            Feature& feature = featureList.at(feature_index);
            const Feature::Pose& pose = feature.get_pose();

            geometry_msgs::Pose p;
            p.position.x = pose.first.x();
            p.position.y = pose.first.y();
            p.position.z = pose.first.z();
            p.orientation.w = pose.second.w();
            p.orientation.x = pose.second.x();
            p.orientation.y = pose.second.y();
            p.orientation.z = pose.second.z();

            std::optional<Feature::Pose> del_pose = feature.get_del_pose();
            if (del_pose.has_value()) {
                // TODO: Apply del orientation
                p.position.x += del_pose.value().first.x();
                p.position.y += del_pose.value().first.y();
                p.position.z += del_pose.value().first.z();
            }

            pa->poses.push_back(p);

            geometry_msgs::Point pt;
            pt.x = pose.first.x();
            pt.y = pose.first.y();
            pt.z = pose.first.z();        
            node.points.push_back(pt);
        }
    }

    if (this->publish_poses) {
        this->poses_pub.publish(*pa);
        mArray.markers.push_back(node);
        this->markers_pub.publish(mArray);

        ROS_INFO("Published poses and markers.");
    }

    if (this->save_poses) {
        std::ofstream posesTxt;
        posesTxt.open(save_folder + "/poses.txt");

        if (!posesTxt.is_open()) {
            ROS_ERROR("Error opening %s", (save_folder + "/poses.txt").c_str());
        }

        else {
            /*
            FORMAT:
            length
            index px py pz ow ox oy oz
            index px py pz ow ox oy oz
            ...
            */
            posesTxt << feature_indices.size() << std::endl;
            for (int index : feature_indices) {
                Feature& f = featureList.at(index);
                Feature::Pose pose = f.get_pose();
                posesTxt << index << " " << pose.first.x() << " " << pose.first.y() << " " << pose.first.z() << " " << pose.second.w() << " " << pose.second.x() << " " << pose.second.y() << " " << pose.second.z() << std::endl;
            }

            posesTxt.close();
            ROS_INFO("Saved poses to %s.", (save_folder + "/poses.txt").c_str());
        }
    }

    return pa;
}

// TODO: transform according to Feature::del_pose
void OptMapNode::export_scans(const std::vector<int>& feature_indices) {
    if (this->save_scans) {
        std::filesystem::create_directory(save_folder + "/map_scans");
    }

    for (int index : feature_indices) {
        if (this->publish_scans) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
            std::string cloud_path = featureList.at(index).get_cloud_path();
            pcl::io::loadPCDFile<pcl::PointXYZ> (cloud_path, *pc);

            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(*pc, cloud_ros);
            cloud_ros.header.frame_id = this->map_frame;
            this->map_scans_pub.publish(cloud_ros);
        }

        if (this->save_scans) {
            std::string new_cloud_path = save_folder + "/map_scans/" + std::to_string(index) + ".pcd";
            std::filesystem::copy_file(featureList.at(index).get_cloud_path(), new_cloud_path);
        }
    }

    if (this->publish_scans) { ROS_INFO("Published map scans."); }
    if (this->save_scans) { ROS_INFO("Saved map scans to %s.", (save_folder + "/map_scans").c_str()); }
}

/* 
save_folder/
    clouds/
    features.txt

features.txt format:
[NUM FEATURES]
[m_scan_index #1]
[m_timestamp.sec #1] [m_timestamp.nsec #1]
[m_pose.position.x #1] [m_pose.position.y #1] [m_pose.position.z #1] [m_pose.orientation.w #1] [m_pose.orientation.x #1] [m_pose.orientation.y #1] [m_pose.orientation.z #1]
[m_descriptor[0] #1] [m_descriptor[1] #1] ... [m_descriptor[255] #1]
[m_cloud_path #1]
[m_scan_index #2]
...
[m_cloud_path #2]
...
[m_cloud_path #n]
*/

void OptMapNode::save_features_to_folder(const std::vector<int>& feature_indices) {
    if (!std::filesystem::exists(save_folder)) {
        std::filesystem::create_directory(save_folder);
    }
    std::ofstream featuresTxt;
    featuresTxt.open(save_folder + "/features.txt");

    if (!featuresTxt.is_open()) {
        ROS_ERROR("Error opening %s", (save_folder + "/features.txt").c_str());
        return;
    }

    featuresTxt << feature_indices.size() << std::endl;
    for (int index : feature_indices) {
        Feature& f = featureList.at(index);
        Feature::Pose pose = f.get_pose();
        Feature::Descriptor desc = f.get_descriptor();

        featuresTxt << f.get_scan_index() << std::endl;
        featuresTxt << f.get_timestamp().sec << " " << f.get_timestamp().nsec << std::endl;
        featuresTxt << pose.first.x() << " " << pose.first.y() << " " << pose.first.z() << " " << pose.second.w() << " " << pose.second.x() << " " << pose.second.y() << " " << pose.second.z() << std::endl;
        for (int i = 0; i < desc.rows(); i++)
            featuresTxt << desc[i] << " ";
        featuresTxt << std::endl;
        featuresTxt << save_folder + "/map_scans/" + std::to_string(f.get_scan_index()) + ".pcd" << std::endl;
    }
    
    featuresTxt.close();
    ROS_INFO("Saved features to %s", save_folder.c_str());
}

void OptMapNode::screen_output() {

    // Current Time (of most recent scan)
    std::time_t curr_time = this->recent_time;
    std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();

    // Assume time comes in order
    if (this->first_time.empty()) {
        this->first_time = asc_time;
    }
    else {
        this->last_time = asc_time;
    }

    // get RSS
    std::ifstream stat_file("/proc/self/stat");
    std::string line;
    std::getline(stat_file, line);
    std::istringstream iss(line);

    // Skip the first 22 fields
    std::string ignore;
    for (int i = 0; i < 23; ++i) {
        iss >> ignore;
    }

    long rss_pages = 0;
    iss >> rss_pages;

    long page_size_bytes = sysconf(_SC_PAGESIZE);
    long rss_MB = rss_pages * page_size_bytes / 1024 / 1024;

    std::cout << "---------- OptMap " + this->version + " ----------\n" \
              << "Most recent scan time: " << asc_time << "\n" \
              << "Most recent scan indices:\n" \
              << "Feature: " << featureList.at(featureList.get_num_features() - 1).get_scan_index() << "    Pose: " << this->most_recent_pose << "    Descriptor: " << this->most_recent_desc << "    Pointcloud: " << this->most_recent_cloud << "\n" \
              << "Fully initialized features: " << featureList.get_num_features() << "    Uninitialized: " << (featureList.get_num_features_all_features() - featureList.get_num_features()) << "\n" \
              << "Distances queue size: " << featureList.get_distances_queue_size() << "\n" \
              << "RAM Usage (RSS): " << rss_MB << " MB \n" \
              << "----------------------------\n";

    if (debug) {
        this->max_mem_usage = std::max(this->max_mem_usage, static_cast<float>(rss_MB));
    }
}

void OptMapNode::Debug() {
    if (!std::filesystem::exists(save_folder)) {
        std::filesystem::create_directory(save_folder);
    }
    std::string debug_output = save_folder + "/optmap_debug.txt";
    std::ofstream op(debug_output);
    if (op.is_open()) {
        op << "OptMap Version " << this->version << "\n";
        op << "First scan time: " << this->first_time << "\n";
        op << "Last scan time:  " << this->last_time << "\n";
        op << "\n\n";
        op << "Parameters\n";
        op << "------------------\n";
        op << "Voxel size: " << this->voxelize << "\n";
        op << "Fraction of scans to evaluate: " << this->fraction_eval_scans << "\n";
        op << "Unchanged iterations max: " << this->unchanged_iters_max << "\n";
        op << "Number of threads: " << this->num_threads << "\n";
        op << "Number of scans: " << this->num_scans_output << "\n";
        op << "Save folder: " << this->save_folder << "\n";
        op << "Beta: " << this->beta_output << "\n";
        if (d_r) {
            op << "Dynmaic Reordering: True\n";
            op << "Sorted set factor: " << this->sorted_set_factor << "\n";
            op << "Distance heuristic radius: " << this->dist_heur_radius << "\n";
        }
        else {
            op << "Dynmaic Reordering: False\n";
        }
        op << "\nGround Set Data\n";
        op << "------------------\n";
        op << "Total number of features: " << this->tot_num_features << "\n";
        op << "Number of features after filter constraints: " << this->num_features << "\n";
        op << "\nInitial Bounds\n";
        op << "------------------\n";
        op << "Lower bound: " << this->lower_bound << "\n";
        op << "Upper bound: " << this->upper_bound << "\n";
        op << "\nBest Solution Info\n";
        op << "------------------\n";
        op << "Number of solution sets: " << this->num_sols << "\n";
        op << "OPT guess of best solution: " << this->best_sol_opt_guess << "\n";
        op << "Best solution score: " << this->best_sol_score << "\n";
        op << "Size of best solution: " << this->size_of_best_sol << "\n";
        op << "\nComputation Times\n";
        op << "------------------\n";
        op << "Time to compute initial solution: " << this->init_sol_time << " ms\n";
        op << "Optimization time: " << this->optimization_time << " ms\n";
        op << "Time waiting for map to build: " << this->map_build_time << " ms\n";
        op << "Max memory usage: " << this->max_mem_usage << " MB\n";
        op.close();
    } else {
        std::cerr << "Failed to open file at " << debug_output << std::endl;
    }
}
