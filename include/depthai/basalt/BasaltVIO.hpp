#pragma once
#define SOPHUS_USE_BASIC_LOGGING

#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/utility/Pimpl.hpp"

namespace dai {
namespace node {

/**
 * @brief Basalt Visual Inertial Odometry node. Performs VIO on stereo images and IMU data.

*/
class BasaltVIO : public NodeCRTP<ThreadedHostNode, BasaltVIO> {

   public:
    enum class LinearizationType { ABS_QR, ABS_SC, REL_SC };
    enum class MatchingGuessType { SAME_PIXEL, REPROJ_FIX_DEPTH, REPROJ_AVG_DEPTH };
    enum class KeyframeMargCriteria { KF_MARG_DEFAULT, KF_MARG_FORWARD_VECTOR };

    struct VioConfig {
        std::string optical_flow_type;
        int optical_flow_detection_grid_size;
        int optical_flow_detection_num_points_cell;
        int optical_flow_detection_min_threshold;
        int optical_flow_detection_max_threshold;
        bool optical_flow_detection_nonoverlap;
        float optical_flow_max_recovered_dist2;
        int optical_flow_pattern;
        int optical_flow_max_iterations;
        int optical_flow_levels;
        float optical_flow_epipolar_error;
        int optical_flow_skip_frames;
        MatchingGuessType optical_flow_matching_guess_type;
        float optical_flow_matching_default_depth;
        float optical_flow_image_safe_radius;                    // Use to mask black corners in cameras
        bool optical_flow_recall_enable;                         // Enable feature/landmark recall
        bool optical_flow_recall_all_cams;                       // Recall in all cameras, not just cam0
        bool optical_flow_recall_num_points_cell;                // Respect gridcell feature limit
        bool optical_flow_recall_over_tracking;                  // Always perform recall, even on already tracked features
        bool optical_flow_recall_update_patch_viewpoint;         // Update feature patch when succesfully recalled
        float optical_flow_recall_max_patch_dist;                // Maximum distance in % of width to accept a recall, or 0
        std::vector<float> optical_flow_recall_max_patch_norms;  // Maximum patch residual norm to accept a recall

        LinearizationType vio_linearization_type;
        bool vio_sqrt_marg;

        int vio_max_states;
        int vio_max_kfs;
        int vio_min_frames_after_kf;
        float vio_new_kf_keypoints_thresh;
        bool vio_debug;
        bool vio_extended_logging;

        //  double vio_outlier_threshold;
        //  int vio_filter_iteration;
        int vio_max_iterations;

        double vio_obs_std_dev;
        double vio_obs_huber_thresh;
        double vio_min_triangulation_dist;

        bool vio_enforce_realtime;

        bool vio_use_lm;
        double vio_lm_lambda_initial;
        double vio_lm_lambda_min;
        double vio_lm_lambda_max;

        bool vio_scale_jacobian;

        double vio_init_pose_weight;
        double vio_init_ba_weight;
        double vio_init_bg_weight;

        bool vio_marg_lost_landmarks;
        bool vio_fix_long_term_keyframes;
        double vio_kf_marg_feature_ratio;
        KeyframeMargCriteria vio_kf_marg_criteria;  // Keyframe removal criteria

        double mapper_obs_std_dev;
        double mapper_obs_huber_thresh;
        int mapper_detection_num_points;
        double mapper_num_frames_to_match;
        double mapper_frames_to_match_threshold;
        double mapper_min_matches;
        double mapper_ransac_threshold;
        double mapper_min_track_length;
        double mapper_max_hamming_distance;
        double mapper_second_best_test_ratio;
        int mapper_bow_num_bits;
        double mapper_min_triangulation_dist;
        bool mapper_no_factor_weights;
        bool mapper_use_factors;

        bool mapper_use_lm;
        double mapper_lm_lambda_min;
        double mapper_lm_lambda_max;
    };
    constexpr static const char* NAME = "BasaltVIO";
    BasaltVIO();
    ~BasaltVIO();

    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    std::string leftInputName = "left";
    std::string rightInputName = "right";

    void buildInternal() override;
    /**
     * Input left image on which VIO is performed.
     */
    Input& left = inputs[leftInputName];
    /**
     * Input right image on which VIO is performed.
     */
    Input& right = inputs[rightInputName];
    /**
     * Input IMU data.
     */
    Input imu{*this, {"inIMU", DEFAULT_GROUP, false, 0, {{DatatypeEnum::IMUData, true}}}};

    /**
     * Output transform data.
     */
    Output transform{*this, {"transform", DEFAULT_GROUP, {{DatatypeEnum::TransformData, true}}}};
    /**
     * Output passthrough of left image.
     */
    Output passthrough{*this, {"imgPassthrough", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, true}}}};

    void setImuUpdateRate(int rate) {
        imuUpdateRate = rate;
    }
    void setConfigPath(const std::string& path) {
        configPath = path;
    }
    void setConfig(const VioConfig& config);
    void setUseSpecTranslation(bool use) {
        useSpecTranslation = use;
    }
    void setLocalTransform(const std::shared_ptr<TransformData>& transform);
    void setImuExtrinsics(const std::shared_ptr<TransformData>& imuExtr);
    void setAccelBias(const std::vector<double>& accelBias);
    void setAccelNoiseStd(const std::vector<double>& accelNoiseStd);
    void setGyroBias(const std::vector<double>& gyroBias);
    void setGyroNoiseStd(const std::vector<double>& gyroNoiseStd);
    void setDefaultVIOConfig();
    void runSyncOnHost(bool runOnHost);

   private:
    // pimpl
    class Impl;
    Pimpl<Impl> pimpl;

    void run() override;
    void initialize(std::vector<std::shared_ptr<ImgFrame>> frames);
    void stereoCB(std::shared_ptr<ADatatype> in);
    void imuCB(std::shared_ptr<ADatatype> imuData);
    void stop() override;
    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 0, {{DatatypeEnum::MessageGroup, true}}}};
    std::shared_ptr<ImgFrame> leftImg;
    std::mutex imgMtx;
    bool initialized = false;
    std::string configPath = "";
    int imuUpdateRate = 200;
    int threadNum = 1;
    bool useSpecTranslation = true;
    std::optional<std::shared_ptr<TransformData>> imuExtrinsics;
    std::optional<std::vector<double>> accelBias;
    std::optional<std::vector<double>> accelNoiseStd;
    std::optional<std::vector<double>> gyroBias;
    std::optional<std::vector<double>> gyroNoiseStd;
};
}  // namespace node
}  // namespace dai
