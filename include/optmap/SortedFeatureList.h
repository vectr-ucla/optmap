#pragma once

#include "optmap/Feature.h"

/*
This class maintains a collection of feature objects (see `include/optmap/Feature.h`) sorted
by their scan number.

Adding to/modifying the collection is done by specifying a scan number (so that the caller
does not have to be concerned with the order of operations required to maintain sortedness).
Accessing an item in the collection is done by specifying an index into the underlying list.

Only features that are "fully-initialized" (all their fields are set), and thus usable, are kept.

This class also maintains a matrix of feature descriptor distances, which is updated asynchronously.
*/

class SortedFeatureList {
    public:
        SortedFeatureList();
        ~SortedFeatureList();

        void update_unique_scan_dist(float usd) { this->unique_scan_dist = usd; }

        // accessing the feature returned by this method requires acquiring the lock on `features_mutex`
        inline Feature& at(int feature_index) {
            assert(feature_index < get_num_features());
            return contents[feature_index];
        }

        // returns the number of fully-initialized (meaning all fields are set) features
        inline int get_num_features() const {
            return prev_feature_index + 1;
        }

        // calling this function requires acquiring the lock on `distances_mutex`
        inline float get_distance_between_descriptors(int feature_index1, int feature_index2) const {
            assert(feature_index1 < get_num_features() && feature_index2 < get_num_features());
            return descriptor_distances[feature_index1][feature_index2];
        };

        inline std::vector<std::pair<int,float>> get_features_basic() {
            return features_basic;
        }

        inline float get_total_traj_dist() const {
            return total_traj_dist;
        }

        inline const float get_element_weighting(int feature_index) const {
            assert(feature_index < get_num_features());
            return contents[feature_index].get_element_weighting();
        }

        // these two functions allow for accessing features that are not fully initialized
        Feature& at_all_features(int feature_index) {
            return contents[feature_index];
        }

        int get_num_features_all_features() const {
            return contents.size();
        }

        // the number of descriptors that don't have an entry in the distance matrix
        int get_distances_queue_size();

        // clear all data
        void clear();

        /*
        If `scan_index` is greater than the scan index of the last fully-initialized feature, then the 
        feature with the corresponding scan index is (1) first created if it doesn't exist, then (2)
        updated by calling `modify_feature_func` on it.

        Otherwise, nothing happens. We do this to account for dropped and out-of-order data messages.
        
        In the case the feature becomes fully-initialized, and is outside the given distance threshold
        (see cfg/optmap.yaml) from its neighboring features, the feature is marked as such and preceeding
        features that are not so are deleted from memory.

        @param
            `scan_index`: scan index of the feature to be updated / added if it doesn't exist
        @param
            `modify_feature_func`: function to modify the feature
        */
        void append_feature_data(int scan_index, const std::function<void(Feature &)> &modify_feature_func);
        
        // accessing the `contents` field requires acquiring this lock
        std::mutex features_mutex;

        // calling `get_distance_between_descriptors` requires acquiring this lock
        std::mutex distances_mutex;

    private:
        // Runs on a separate thread. Computes distances between feature locations
        void compute_descriptor_distances();

        // stores the features sorted in ascending order by scan index
        std::vector<Feature> contents;

        // distance matrix for feature descriptors. indexed by a feature's
        // corresponding index in `contents`, not by `scan_index`.
        std::vector<std::vector<float>> descriptor_distances;

        std::vector<std::pair<int,float>> features_basic;
        int prev_feature_index;
        float total_traj_dist;
        float dist_prev_included_element;

        // Worker thread, queue, and synchronization primitives
        std::thread descriptor_distances_thread;
        std::queue<int> descriptor_distances_queue;
        std::mutex descriptor_distances_queue_mutex;
        std::condition_variable descriptor_distances_queue_cv;
        bool descriptor_distances_thread_active;

        // config parameter. Distance requirement between consecutive features
        float unique_scan_dist;
};