#include "optmap/SortedFeatureList.h"

SortedFeatureList::SortedFeatureList() {
    ros::param::param<float>("~optmap/unique_scan_dist", unique_scan_dist, 0.01);

    prev_feature_index = -1;
    total_traj_dist = 0;
    dist_prev_included_element = 0;

    descriptor_distances_thread_active = true;
    descriptor_distances_thread = std::thread(&SortedFeatureList::compute_descriptor_distances, this);
}

SortedFeatureList::~SortedFeatureList() {
    // Wake up the worker thread to exit
    {
        std::lock_guard<std::mutex> lock(descriptor_distances_queue_mutex);
        descriptor_distances_thread_active = false;
    }
    descriptor_distances_queue_cv.notify_all();
    if (descriptor_distances_thread.joinable()) {
        descriptor_distances_thread.join();
    }
}

int SortedFeatureList::get_distances_queue_size() {
    return descriptor_distances_queue.size();
}

void SortedFeatureList::clear() {
    std::lock_guard<std::mutex> features_lock(this->features_mutex);
    std::lock_guard<std::mutex> distances_lock(this->distances_mutex);
    std::unique_lock<std::mutex> distances_queue_lock(this->descriptor_distances_queue_mutex);

    contents.clear();
    descriptor_distances.clear();
    std::queue<int>().swap(descriptor_distances_queue);  // clear the queue

    prev_feature_index = -1;
    descriptor_distances_thread_active = true;
}

void SortedFeatureList::append_feature_data(int scan_index, const std::function<void(Feature &)> &modify_feature_func) {
    std::lock_guard<std::mutex> lock(this->features_mutex);

    // We choose to discard any features that come in if the most recent feature has a higher scan index
    if (prev_feature_index != -1 && contents[prev_feature_index].get_scan_index() >= scan_index) {
        return;
    }

    auto feature_lb = std::lower_bound(contents.begin() + prev_feature_index + 1, contents.end(), scan_index,
        [](Feature& f, int scan_index) {
            return f.get_scan_index() < scan_index;
        }
    );
    int feature_index = feature_lb - contents.begin();

    bool exists = (feature_index < contents.size() && contents[feature_index].get_scan_index() == scan_index);
    if (!exists) {
        Feature feature(scan_index);
        contents.insert(contents.begin() + feature_index, std::move(feature));
    }
    
    modify_feature_func(contents[feature_index]);
    if (contents[feature_index].is_fully_init()) {

        float feat_dist_from_prev = (prev_feature_index == -1) ? 0 : (contents[feature_index].get_descriptor() - 
                                                                    contents[prev_feature_index].get_descriptor()).norm();

        // Assume very small differences are noise
        if (feat_dist_from_prev < 0.005 && prev_feature_index != -1) {
            return;
        }

        feat_dist_from_prev += dist_prev_included_element;

        bool distance_thresh_met = (prev_feature_index == -1 || feat_dist_from_prev >= unique_scan_dist);
        if (!distance_thresh_met) { 
            dist_prev_included_element = feat_dist_from_prev;
            return; 
        }

        dist_prev_included_element = 0;

        contents[feature_index].set_element_weighting(feat_dist_from_prev);
        features_basic.push_back({features_basic.size(), feat_dist_from_prev});
        total_traj_dist += feat_dist_from_prev;
        
        // we erase all features between the previously set feature and the current one under the assumption
        // that an update to that feature was dropped, and thus it will never be fully initialized
        contents.erase(contents.begin() + prev_feature_index + 1, contents.begin() + feature_index);
        int num_erased = feature_index - (prev_feature_index + 1);
        prev_feature_index = feature_index - num_erased;

        std::lock_guard<std::mutex> lock(descriptor_distances_queue_mutex);
        descriptor_distances_queue.push(prev_feature_index);
        descriptor_distances_queue_cv.notify_one();

        if (descriptor_distances_queue.size() > 20) {
            ROS_WARN("Distances queue has size %ld. Possible backup.\n", descriptor_distances_queue.size());
        }
    }
}

void SortedFeatureList::compute_descriptor_distances() {
    while (descriptor_distances_thread_active) {
        int feature_index = -1;

        {
            std::unique_lock<std::mutex> lock(this->descriptor_distances_queue_mutex);

            // Wait for an item in the queue or wait to be stopped
            descriptor_distances_queue_cv.wait(lock, [this]() {
                return !descriptor_distances_queue.empty() || !descriptor_distances_thread_active;
            });

            if (!descriptor_distances_thread_active) return;

            feature_index = descriptor_distances_queue.front();
            descriptor_distances_queue.pop();
        }

        {
            std::lock_guard<std::mutex> features_lock(this->features_mutex);
            std::lock_guard<std::mutex> distances_lock(this->distances_mutex);

            descriptor_distances.push_back(std::vector<float>());
            for (int i = 0; i < feature_index; i++) {
                float dist = (contents[feature_index].get_descriptor() - contents[i].get_descriptor()).norm();
                descriptor_distances[feature_index].push_back(dist);
                descriptor_distances[i].push_back(dist);
            }
            descriptor_distances[feature_index].push_back(0);  // distance from itself
        }
    }
}
