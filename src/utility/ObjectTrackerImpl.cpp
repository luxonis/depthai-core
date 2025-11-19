// Implementation adapted from https://github.com/Postroggy/OC_SORT_CPP/tree/master

#include "ObjectTrackerImpl.hpp"

#include <fmt/base.h>

#include "eigen3/Eigen/Dense"
#include "properties/ObjectTrackerProperties.hpp"

namespace dai {
namespace impl {

#define pi 3.1415926
#define LARGE 1000000

#define SWAP_INDICES(a, b)     \
    {                          \
        int_t _temp_index = a; \
        a = b;                 \
        b = _temp_index;       \
    }

#define ASSERT(cond)
#define PRINTF(fmt, ...)
#define PRINT_COST_ARRAY(a, n)
#define PRINT_INDEX_ARRAY(a, n)

typedef signed int int_t;
typedef unsigned int uint_t;
typedef float cost_t;
typedef char boolean;
typedef enum fp_t { FP_1 = 1, FP_2 = 2, FP_DYNAMIC = 3 } fp_t;

extern int_t lapjv_internal(const uint_t n, cost_t* cost[], int_t* x, int_t* y);

extern int_t lapmod_internal(const uint_t n, cost_t* cc, uint_t* ii, uint_t* kk, int_t* x, int_t* y, fp_t fp_version);
float execLapjv(
    const std::vector<std::vector<float>>& cost, std::vector<int>& rowsol, std::vector<int>& colsol, bool extend_cost, float cost_limit, bool return_cost);

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> speed_direction_batch(const Eigen::MatrixXf& dets, const Eigen::MatrixXf& tracks);
Eigen::MatrixXf iou_batch(const Eigen::MatrixXf& bboxes1, const Eigen::MatrixXf& bboxes2);
Eigen::MatrixXf giou_batch(const Eigen::MatrixXf& bboxes1, const Eigen::MatrixXf& bboxes2);
std::tuple<std::vector<Eigen::Matrix<int, 1, 2>>, std::vector<int>, std::vector<int>> associate(
    Eigen::MatrixXf detections, Eigen::MatrixXf trackers, float iou_threshold, Eigen::MatrixXf velocities, Eigen::MatrixXf previous_obs_, float vdc_weight);
/**
 * Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
[x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
the aspect ratio
 * @param bbox
 * @return z
 */
Eigen::VectorXf convert_bbox_to_z(Eigen::VectorXf bbox);
Eigen::VectorXf speed_direction(Eigen::VectorXf bbox1, Eigen::VectorXf bbox2);
Eigen::VectorXf convert_x_to_bbox(Eigen::VectorXf x);
Eigen::VectorXf k_previous_obs(const std::map<int, Eigen::VectorXf>& observations, int cur_age, int k);

Tracker::~Tracker() = default;
class TrackletExt : public Tracklet {
   public:
    uint32_t ageSinceStatusUpdate = 1;
    uint32_t detectedCount = 1;

    void update(const Rect& rect, const Point3f& spatialCoordinates = Point3f(0, 0, 0)) {
        this->roi = rect;
        this->spatialCoordinates = spatialCoordinates;
        ++this->age;
        ++this->ageSinceStatusUpdate;
    }

    void match(const Rect& rect) {
        ++detectedCount;
        update(rect);
    }

    void updateStatus(Tracklet::TrackingStatus status) {
        this->status = status;
        ageSinceStatusUpdate = 1;
    }
};

class OCSTracker::State {
   public:
    class KalmanFilterNew {
       public:
        KalmanFilterNew() = default;
        KalmanFilterNew(int dim_x_, int dim_z_);
        void predict();
        void update(Eigen::VectorXf z_);
        void freeze();
        void unfreeze();
        KalmanFilterNew& operator=(const KalmanFilterNew&) = delete;

       public:
        int dim_z = 4;
        int dim_x = 7;
        int dim_u = 0;
        // state: This is the Kalman state variable [7,1].
        Eigen::VectorXf x;
        // P: Covariance matrix. Initially declared as an identity matrix. Data type is float. [7,7].
        Eigen::MatrixXf P;
        // Q: Process noise covariance matrix. [7,7].
        Eigen::MatrixXf Q;
        // B: Control matrix. Not used in target tracking. [n,n].
        Eigen::MatrixXf B;
        // F: Prediction matrix / state transition matrix. [7,7].
        Eigen::Matrix<float, 7, 7> F;
        // H: Observation model / matrix. [4,7].
        Eigen::Matrix<float, 4, 7> H;
        // R: Observation noise covariance matrix. [4,4].
        Eigen::Matrix<float, 4, 4> R;
        // _alpha_sq: Fading memory control, controlling the update weight. Float.
        float _alpha_sq = 1.0;
        // M: Measurement matrix, converting state vector x to measurement vector z. [7,4]. It has the opposite effect of matrix H.
        Eigen::MatrixXf M;
        // z: Measurement vector. [4,1].
        Eigen::VectorXf z;
        /* The following variables are intermediate variables used in calculations */
        // K: Kalman gain. [7,4].
        Eigen::MatrixXf K;
        // y: Measurement residual. [4,1].
        Eigen::MatrixXf y;
        // S: Measurement residual covariance.
        Eigen::MatrixXf S;
        // SI: Transpose of measurement residual covariance (simplified for subsequent calculations).
        Eigen::MatrixXf SI;
        // Identity matrix of size [dim_x,dim_x], used for convenient calculations. This cannot be changed.
        const Eigen::MatrixXf I = Eigen::MatrixXf::Identity(dim_x, dim_x);
        // There will always be a copy of x, P after predict() is called.
        // If there is a need to assign values between two Eigen matrices, the precondition is that they should be initialized properly, as this ensures that
        // the number of columns and rows are compatible.
        Eigen::VectorXf x_prior;
        Eigen::MatrixXf P_prior;
        // there will always be a copy of x,P after update() is called
        Eigen::VectorXf x_post;
        Eigen::MatrixXf P_post;
        // keeps all observations. When there is a 'z', it is directly pushed back.
        std::vector<Eigen::VectorXf> history_obs;
        // The following is newly added by ocsort.
        // Used to mark the tracking state (whether there is still a target matching this trajectory), default value is false.
        bool observed = false;
        std::vector<Eigen::VectorXf> new_history;  // Used to create a virtual trajectory.

        struct Data {
            Eigen::VectorXf x;
            Eigen::MatrixXf P;
            Eigen::MatrixXf Q;
            Eigen::MatrixXf B;
            Eigen::MatrixXf F;
            Eigen::MatrixXf H;
            Eigen::MatrixXf R;
            float _alpha_sq = 1.;
            Eigen::MatrixXf M;
            Eigen::VectorXf z;
            Eigen::MatrixXf K;
            Eigen::MatrixXf y;
            Eigen::MatrixXf S;
            Eigen::MatrixXf SI;
            Eigen::VectorXf x_prior;
            Eigen::MatrixXf P_prior;
            Eigen::VectorXf x_post;
            Eigen::MatrixXf P_post;
            std::vector<Eigen::VectorXf> history_obs;
            bool observed = false;
            // The following is to determine whether the data has been saved due to freezing.
            bool IsInitialized = false;
        };
        struct Data attr_saved;
    };
    class KalmanBoxTracker {
       public:
        KalmanBoxTracker(){};
        KalmanBoxTracker(Eigen::VectorXf bbox_, int cls_, int delta_t_ = 3);
        void update(Eigen::Matrix<float, 5, 1>* bbox_, int cls_);
        Eigen::RowVectorXf predict();
        Eigen::VectorXf get_state();

       public:
        Eigen::VectorXf bbox;  // [5,1]
        std::shared_ptr<KalmanFilterNew> kf;
        int time_since_update;
        std::vector<Eigen::VectorXf> history;  //  [4,1]
        int hits;
        int hit_streak;
        int age = 0;
        float conf;
        int cls;
        Eigen::RowVectorXf last_observation = Eigen::RowVectorXf::Zero(5);
        std::map<int, Eigen::VectorXf> observations;
        std::vector<Eigen::VectorXf> history_observations;
        Eigen::RowVectorXf velocity = Eigen::RowVectorXf::Zero(2);  // [2,1]
        int delta_t;
        bool remove = false;
    };
    class ClassState {
       public:
        float det_thresh;
        int max_age;
        int min_hits;
        float iou_threshold;
        int delta_t;
        std::function<Eigen::MatrixXf(const Eigen::MatrixXf&, const Eigen::MatrixXf&)> asso_func;
        float inertia;
        bool use_byte;
        std::vector<KalmanBoxTracker> trackers;
        std::vector<TrackletExt> tracklets;
        int frame_count;
        State* parent;

        void prep() {
            trackers.erase(std::remove_if(trackers.begin(), trackers.end(), [](const KalmanBoxTracker& t) { return t.remove; }), trackers.end());
            tracklets.erase(
                std::remove_if(tracklets.begin(), tracklets.end(), [](const TrackletExt& t) { return t.status == Tracklet::TrackingStatus::REMOVED; }),
                tracklets.end());
        }
        void remove_tracker(size_t idx) {
            if(idx < trackers.size()) {
                trackers[idx].remove = true;
                tracklets[idx].updateStatus(Tracklet::TrackingStatus::REMOVED);
                --parent->num_trackers;
            } else {
                throw std::runtime_error("Index out of bounds in remove_tracker");
            }
        }
        void remove_tracklets(const std::vector<int32_t>& ids) {
            for(const auto& id : ids) {
                auto it = std::find_if(tracklets.begin(), tracklets.end(), [id](const TrackletExt& t) { return t.id == id; });
                if(it != tracklets.end()) {
                    remove_tracker(std::distance(tracklets.begin(), it));
                }
            }
        }
        ClassState(State* parent,
                   float det_thresh_,
                   int max_age_ = 30,
                   int min_hits_ = 3,
                   float iou_threshold_ = 0.3,
                   int delta_t_ = 3,
                   std::string asso_func_ = "iou",
                   float inertia_ = 0.2,
                   bool use_byte_ = false);

        std::vector<Eigen::RowVectorXf> update(const std::vector<ImgDetection>& detections, const std::vector<Point3f>& spatialData, bool trackOnly = false);
    };

   private:
    float det_thresh;
    int max_age;
    int min_hits;
    float iou_threshold;
    int delta_t;
    std::string asso_func;
    float inertia;
    bool use_byte;
    uint32_t max_id = 0;
    TrackerIdAssignmentPolicy id_assignment_policy;
    bool track_by_class = false;
    uint32_t max_trackers;
    std::atomic<uint32_t> num_trackers = 0;
    std::unordered_map<uint32_t, ClassState> class_states;

    uint32_t get_next_id() {
        switch(id_assignment_policy) {
            case TrackerIdAssignmentPolicy::UNIQUE_ID:
                return max_id++;
            case TrackerIdAssignmentPolicy::SMALLEST_ID: {
                uint32_t id = 0;
                std::vector<uint32_t> ids;
                for(const auto& [_, cs] : class_states) {
                    ids.reserve(ids.size() + cs.tracklets.size());
                    for(const auto& t : cs.tracklets) ids.push_back(t.id);
                }
                std::sort(ids.begin(), ids.end());
                for(const auto& t : ids) {
                    if(t != id) {
                        break;
                    }
                    ++id;
                }
                if(id >= max_id) {
                    max_id = id + 1;  // Ensure max_id is always greater than the largest id
                }
                return id;
            }
        }
        throw std::runtime_error("Unknown TrackerIdAssignmentPolicy");
    }

   public:
    State(float det_thresh_,
          int max_age_ = 30,
          int min_hits_ = 3,
          float iou_threshold_ = 0.3,
          TrackerIdAssignmentPolicy id_assignment_policy_ = TrackerIdAssignmentPolicy::UNIQUE_ID,
          bool track_by_class_ = false,
          uint32_t max_trackers_ = 100,
          int delta_t_ = 3,
          std::string asso_func_ = "iou",
          float inertia_ = 0.2,
          bool use_byte_ = false);

    std::vector<Eigen::RowVectorXf> update(const std::vector<ImgDetection>& detections, const std::vector<Point3f>& spatialData, bool trackOnly = false);
    void remove_tracklets(const std::vector<int32_t>& ids) {
        for(auto& [_, cs] : class_states) {
            cs.remove_tracklets(ids);
        }
    }
    std::vector<Tracklet> get_tracklets() const {
        std::vector<Tracklet> tracklets;
        for(const auto& [index, cs] : class_states) {
            tracklets.reserve(tracklets.size() + cs.tracklets.size());
            for(const auto& t : cs.tracklets) {
                tracklets.push_back((Tracklet)t);
            }
        }
        return tracklets;
    }
};

OCSTracker::State::KalmanBoxTracker::KalmanBoxTracker(Eigen::VectorXf bbox_, int cls_, int delta_t_) {
    bbox = std::move(bbox_);
    delta_t = delta_t_;
    kf = std::make_shared<KalmanFilterNew>(7, 4);
    kf->F << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1;
    kf->H << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    kf->R.block(2, 2, 2, 2) *= 10.0;
    kf->P.block(4, 4, 3, 3) *= 1000.0;
    kf->P *= 10.0;
    kf->Q.bottomRightCorner(1, 1)(0, 0) *= 0.01f;
    kf->Q.block(4, 4, 3, 3) *= 0.01;
    kf->x.head<4>() = convert_bbox_to_z(bbox);
    time_since_update = 0;
    history.clear();
    hits = 0;
    hit_streak = 0;
    age = 0;
    conf = bbox(4);
    cls = cls_;
    last_observation.fill(-1);
    observations.clear();
    history_observations.clear();
    velocity.fill(0);
}
void OCSTracker::State::KalmanBoxTracker::update(Eigen::Matrix<float, 5, 1>* bbox_, int cls_) {
    if(bbox_ != nullptr) {
        conf = (*bbox_)[4];
        cls = cls_;
        if(int(last_observation.sum()) >= 0) {
            Eigen::VectorXf previous_box_tmp;
            for(int dt = delta_t; dt > 0; --dt) {
                auto it = observations.find(age - dt);
                if(it != observations.end()) {
                    previous_box_tmp = it->second;
                    break;
                }
            }
            if(0 == previous_box_tmp.size()) {
                previous_box_tmp = last_observation;
            }
            const int maxSize = 300;
            if(observations.size() > maxSize) {
                observations.erase(observations.begin());
            }
            velocity = speed_direction(previous_box_tmp, *bbox_);
        }
        last_observation = *bbox_;
        observations[age] = *bbox_;
        history_observations.push_back(*bbox_);
        time_since_update = 0;
        history.clear();
        hits += 1;
        hit_streak += 1;
        Eigen::VectorXf tmp = convert_bbox_to_z(*bbox_);
        kf->update(tmp);
    } else {
        kf->update(Eigen::VectorXf());
    }
}

template <typename Matrix>
std::ostream& operator<<(std::ostream& os, const std::vector<Matrix>& v) {
    os << "{";
    for(auto it = v.begin(); it != v.end(); ++it) {
        os << "(" << *it << ")\n";
        if(it != v.end() - 1) os << ",";
    }
    os << "}\n";
    return os;
}

OCSTracker::State::ClassState::ClassState(State* parent_,
                                          float det_thresh_,
                                          int max_age_,
                                          int min_hits_,
                                          float iou_threshold_,
                                          int delta_t_,
                                          std::string asso_func_,
                                          float inertia_,
                                          bool use_byte_) {
    /*Sets key parameters for SORT*/
    parent = parent_;
    max_age = max_age_;
    min_hits = min_hits_;
    iou_threshold = iou_threshold_;
    trackers.clear();
    frame_count = 0;
    det_thresh = det_thresh_;
    delta_t = delta_t_;
    std::unordered_map<std::string, std::function<Eigen::MatrixXf(const Eigen::MatrixXf&, const Eigen::MatrixXf&)>> ASSO_FUNCS{{"iou", iou_batch},
                                                                                                                               {"giou", giou_batch}};
    ;
    asso_func = ASSO_FUNCS[asso_func_];
    inertia = inertia_;
    use_byte = use_byte_;
}
std::vector<Eigen::RowVectorXf> OCSTracker::State::ClassState::update(const std::vector<ImgDetection>& detections,
                                                                      const std::vector<Point3f>& spatialData,
                                                                      bool trackOnly) {
    /*
     * dets: (n,7): [[x1,y1,x2,y2,confidence_score, class, idx],...[...]]
     * Params:
    dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
    Requires: this method must be called once for each frame even with empty detections (use np.empty((0, 5)) for frames without detections).
    Returns the a similar array, where the last column is the object ID.
    NOTE: The number of objects returned may differ from the number of detections provided.
     */

    if(detections.size() != spatialData.size()) {
        throw std::runtime_error("Number of detections and spatial data points must match");
    }

    prep();

    Eigen::Matrix<float, Eigen::Dynamic, 7> dets(detections.size(), 7);

    for(size_t i = 0; i < detections.size(); i++) {
        const auto& detection = detections[i];
        // Convert normalized coordinates to absolute coordinates
        dets(i, 0) = detection.xmin;        // x1
        dets(i, 1) = detection.ymin;        // y1
        dets(i, 2) = detection.xmax;        // x2
        dets(i, 3) = detection.ymax;        // y2
        dets(i, 4) = detection.confidence;  // confidence
        dets(i, 5) = detection.label;       // class_id
        dets(i, 6) = i;
    }

    frame_count += 1;
    Eigen::Matrix<float, Eigen::Dynamic, 4> xyxys = dets.leftCols(4);
    Eigen::Matrix<float, 1, Eigen::Dynamic> confs = dets.col(4);
    Eigen::Matrix<float, 1, Eigen::Dynamic> clss = dets.col(5);
    Eigen::MatrixXf output_results = dets;
    auto inds_low = confs.array() > 0.1;
    auto inds_high = confs.array() < det_thresh;
    auto inds_second = inds_low && inds_high;
    Eigen::Matrix<float, Eigen::Dynamic, 7> dets_second;
    Eigen::Matrix<bool, 1, Eigen::Dynamic> remain_inds = (confs.array() > det_thresh);
    Eigen::Matrix<float, Eigen::Dynamic, 7> dets_first;
    for(int i = 0; i < output_results.rows(); i++) {
        if(true == inds_second(i)) {
            dets_second.conservativeResize(dets_second.rows() + 1, Eigen::NoChange);
            dets_second.row(dets_second.rows() - 1) = output_results.row(i);
        }
        if(true == remain_inds(i)) {
            dets_first.conservativeResize(dets_first.rows() + 1, Eigen::NoChange);
            dets_first.row(dets_first.rows() - 1) = output_results.row(i);
        }
    }
    /*get predicted locations from existing trackers.*/
    Eigen::MatrixXf trks = Eigen::MatrixXf::Zero(trackers.size(), 5);
    std::vector<int> to_del;
    std::vector<Eigen::RowVectorXf> ret;
    for(int i = 0; i < trks.rows(); i++) {
        Eigen::RowVectorXf pos = trackers[i].predict();
        trks.row(i) << pos(0), pos(1), pos(2), pos(3), 0;
    }
    Eigen::MatrixXf velocities = Eigen::MatrixXf::Zero(trackers.size(), 2);
    Eigen::MatrixXf last_boxes = Eigen::MatrixXf::Zero(trackers.size(), 5);
    Eigen::MatrixXf k_observations = Eigen::MatrixXf::Zero(trackers.size(), 5);
    for(uint32_t i = 0; i < trackers.size(); i++) {
        velocities.row(i) = trackers[i].velocity;
        last_boxes.row(i) = trackers[i].last_observation;
        k_observations.row(i) = k_previous_obs(trackers[i].observations, trackers[i].age, delta_t);
    }
    /////////////////////////
    ///  Step1 First round of association
    ////////////////////////
    std::vector<Eigen::Matrix<int, 1, 2>> matched;
    std::vector<int> unmatched_dets;
    std::vector<int> unmatched_trks;
    auto result = associate(dets_first, trks, iou_threshold, velocities, k_observations, inertia);
    matched = std::get<0>(result);
    unmatched_dets = std::get<1>(result);
    unmatched_trks = std::get<2>(result);
    for(auto m : matched) {
        Eigen::Matrix<float, 5, 1> tmp_bbox;
        tmp_bbox = dets_first.block<1, 5>(m(0), 0);
        uint32_t index = dets_first(m(0), 6);
        trackers[m(1)].update(&(tmp_bbox), dets_first(m(0), 5));
        tracklets[m(1)].update(Rect(tmp_bbox(0), tmp_bbox(1), tmp_bbox(2) - tmp_bbox(0), tmp_bbox(3) - tmp_bbox(1)), spatialData[index]);
        tracklets[m(1)].srcImgDetection = detections[index];
        if(tracklets[m(1)].status == Tracklet::TrackingStatus::LOST) {
            tracklets[m(1)].updateStatus(Tracklet::TrackingStatus::TRACKED);
        } else if(tracklets[m(1)].status == Tracklet::TrackingStatus::NEW && tracklets[m(1)].age >= min_hits) {
            tracklets[m(1)].updateStatus(Tracklet::TrackingStatus::TRACKED);
        }
    }

    ///////////////////////
    /// Step2 Second round of associaton by OCR to find lost tracks back
    //////////////////////
    if(true == use_byte && dets_second.rows() > 0 && unmatched_trks.size() > 0) {
        Eigen::MatrixXf u_trks(unmatched_trks.size(), trks.cols());
        int index_for_u_trks = 0;
        for(auto i : unmatched_trks) {
            u_trks.row(index_for_u_trks++) = trks.row(i);
        }
        Eigen::MatrixXf iou_left = asso_func(dets_second, u_trks);
        // Eigen::MatrixXf iou_left = asso_func(dets_second, u_trks);
        if(iou_left.maxCoeff() > iou_threshold) {
            /**
                NOTE: by using a lower threshold, e.g., self.iou_threshold - 0.1, you may
                get a higher performance especially on MOT17/MOT20 datasets. But we keep it
                uniform here for simplicity
             * */
            std::vector<std::vector<float>> iou_matrix(iou_left.rows(), std::vector<float>(iou_left.cols()));
            for(int i = 0; i < iou_left.rows(); i++) {
                for(int j = 0; j < iou_left.cols(); j++) {
                    iou_matrix[i][j] = -iou_left(i, j);
                }
            }
            std::vector<int> rowsol, colsol;
            execLapjv(iou_matrix, rowsol, colsol, true, 0.01, true);
            std::vector<std::vector<int>> matched_indices;
            for(uint32_t i = 0; i < rowsol.size(); i++) {
                if(rowsol.at(i) >= 0) {
                    matched_indices.push_back({colsol.at(rowsol.at(i)), rowsol.at(i)});
                }
            }

            std::vector<int> to_remove_trk_indices;
            for(auto m : matched_indices) {
                int det_ind = m[0];
                int trk_ind = unmatched_trks[m[1]];
                if(iou_left(m[0], m[1]) < iou_threshold) continue;

                Eigen::Matrix<float, 5, 1> tmp_box;
                tmp_box = dets_second.block<1, 5>(det_ind, 0);
                uint32_t index = dets_second(det_ind, 6);
                trackers[trk_ind].update(&tmp_box, dets_second(det_ind, 5));
                tracklets[trk_ind].update(Rect(tmp_box(0), tmp_box(1), tmp_box(2) - tmp_box(0), tmp_box(3) - tmp_box(1)), spatialData[index]);
                tracklets[trk_ind].srcImgDetection = detections[index];
                if(tracklets[trk_ind].status == Tracklet::TrackingStatus::LOST) {
                    tracklets[trk_ind].updateStatus(Tracklet::TrackingStatus::TRACKED);
                } else if(tracklets[trk_ind].status == Tracklet::TrackingStatus::NEW && tracklets[trk_ind].age >= min_hits) {
                    tracklets[trk_ind].updateStatus(Tracklet::TrackingStatus::TRACKED);
                }
                to_remove_trk_indices.push_back(trk_ind);
            }
            std::vector<int> tmp_res1(unmatched_trks.size());
            sort(unmatched_trks.begin(), unmatched_trks.end());
            sort(to_remove_trk_indices.begin(), to_remove_trk_indices.end());
            auto end1 =
                set_difference(unmatched_trks.begin(), unmatched_trks.end(), to_remove_trk_indices.begin(), to_remove_trk_indices.end(), tmp_res1.begin());
            tmp_res1.resize(end1 - tmp_res1.begin());
            unmatched_trks = tmp_res1;
        }
    }

    if(unmatched_dets.size() > 0 && unmatched_trks.size() > 0) {
        Eigen::MatrixXf left_dets(unmatched_dets.size(), 7);
        int inx_for_dets = 0;
        for(auto i : unmatched_dets) {
            left_dets.row(inx_for_dets++) = dets_first.row(i);
        }
        Eigen::MatrixXf left_trks(unmatched_trks.size(), last_boxes.cols());
        int indx_for_trk = 0;
        for(auto i : unmatched_trks) {
            left_trks.row(indx_for_trk++) = last_boxes.row(i);
        }
        Eigen::MatrixXf iou_left = asso_func(left_dets, left_trks);
        if(iou_left.maxCoeff() > iou_threshold) {
            /**
                NOTE: by using a lower threshold, e.g., self.iou_threshold - 0.1, you may
                get a higher performance especially on MOT17/MOT20 datasets. But we keep it
                uniform here for simplicity
             * */
            std::vector<std::vector<float>> iou_matrix(iou_left.rows(), std::vector<float>(iou_left.cols()));
            for(int i = 0; i < iou_left.rows(); i++) {
                for(int j = 0; j < iou_left.cols(); j++) {
                    iou_matrix[i][j] = -iou_left(i, j);
                }
            }
            std::vector<int> rowsol, colsol;
            execLapjv(iou_matrix, rowsol, colsol, true, 0.01, true);
            std::vector<std::vector<int>> rematched_indices;
            for(uint32_t i = 0; i < rowsol.size(); i++) {
                if(rowsol.at(i) >= 0) {
                    rematched_indices.push_back({colsol.at(rowsol.at(i)), rowsol.at(i)});
                }
            }
            std::vector<int> to_remove_det_indices;
            std::vector<int> to_remove_trk_indices;
            for(auto i : rematched_indices) {
                int det_ind = unmatched_dets[i.at(0)];
                int trk_ind = unmatched_trks[i.at(1)];
                if(iou_left(i.at(0), i.at(1)) < iou_threshold) {
                    continue;
                }
                ////////////////////////////////
                ///  Step3  update status of second matched tracks
                ///////////////////////////////
                Eigen::Matrix<float, 5, 1> tmp_bbox;
                tmp_bbox = dets_first.block<1, 5>(det_ind, 0);
                uint32_t index = dets_first(det_ind, 6);
                trackers.at(trk_ind).update(&tmp_bbox, dets_first(det_ind, 5));
                tracklets[trk_ind].update(Rect(tmp_bbox(0), tmp_bbox(1), tmp_bbox(2) - tmp_bbox(0), tmp_bbox(3) - tmp_bbox(1)), spatialData[index]);
                tracklets[trk_ind].srcImgDetection = detections[index];
                if(tracklets[trk_ind].status == Tracklet::TrackingStatus::LOST) {
                    tracklets[trk_ind].updateStatus(Tracklet::TrackingStatus::TRACKED);
                } else if(tracklets[trk_ind].status == Tracklet::TrackingStatus::NEW && trackers[trk_ind].hit_streak >= min_hits) {
                    tracklets[trk_ind].updateStatus(Tracklet::TrackingStatus::TRACKED);
                }
                to_remove_det_indices.push_back(det_ind);
                to_remove_trk_indices.push_back(trk_ind);
            }
            std::vector<int> tmp_res(unmatched_dets.size());
            sort(unmatched_dets.begin(), unmatched_dets.end());
            sort(to_remove_det_indices.begin(), to_remove_det_indices.end());
            auto end =
                set_difference(unmatched_dets.begin(), unmatched_dets.end(), to_remove_det_indices.begin(), to_remove_det_indices.end(), tmp_res.begin());
            tmp_res.resize(end - tmp_res.begin());
            unmatched_dets = tmp_res;
            std::vector<int> tmp_res1(unmatched_trks.size());
            sort(unmatched_trks.begin(), unmatched_trks.end());
            sort(to_remove_trk_indices.begin(), to_remove_trk_indices.end());
            auto end1 =
                set_difference(unmatched_trks.begin(), unmatched_trks.end(), to_remove_trk_indices.begin(), to_remove_trk_indices.end(), tmp_res1.begin());
            tmp_res1.resize(end1 - tmp_res1.begin());
            unmatched_trks = tmp_res1;
        }
    }

    for(auto m : unmatched_trks) {
        trackers.at(m).update(nullptr, 0);
        if(!trackOnly) {
            if(tracklets[m].status == Tracklet::TrackingStatus::TRACKED) {
                tracklets[m].updateStatus(Tracklet::TrackingStatus::LOST);
            } else if(tracklets[m].status == Tracklet::TrackingStatus::NEW) {
                remove_tracker(m);
            }
        }
    }
    ///////////////////////////////
    /// Step4 Initialize new tracks and remove expired tracks
    ///////////////////////////////
    /*create and initialise new trackers for unmatched detections*/
    for(int i : unmatched_dets) {
        if(parent->num_trackers < parent->max_trackers) {
            Eigen::RowVectorXf tmp_bbox = dets_first.block(i, 0, 1, 5);
            uint32_t index = dets_first(i, 6);
            int cls_ = int(dets(i, 5));
            KalmanBoxTracker trk = KalmanBoxTracker(tmp_bbox, cls_, delta_t);
            ++parent->num_trackers;
            trackers.push_back(trk);
            tracklets.push_back(TrackletExt{Tracklet{Rect(tmp_bbox(0), tmp_bbox(1), tmp_bbox(2) - tmp_bbox(0), tmp_bbox(3) - tmp_bbox(1)),
                                                     (int)parent->get_next_id(),
                                                     cls_,
                                                     1,
                                                     Tracklet::TrackingStatus::NEW,
                                                     detections[index],
                                                     spatialData[index]}});
        }
    }
    for(int i = trackers.size() - 1; i >= 0; i--) {
        Eigen::Matrix<float, 1, 4> d;
        int last_observation_sum = trackers.at(i).last_observation.sum();
        if(last_observation_sum < 0) {
            d = trackers.at(i).get_state();
        } else {
            /**
                this is optional to use the recent observation or the kalman filter prediction,
                we didn't notice significant difference here
             * */
            d = trackers.at(i).last_observation.block(0, 0, 1, 4);
        }
        if(trackers.at(i).time_since_update < 1 && ((trackers.at(i).hit_streak >= min_hits) | (frame_count <= min_hits))) {
            // +1 as MOT benchmark requires positive
            Eigen::RowVectorXf tracking_res(6);
            tracking_res << d(0), d(1), d(2), d(3), trackers.at(i).cls, trackers.at(i).conf;
            ret.push_back(tracking_res);
        }
        // remove dead tracklets
        if(trackers.at(i).time_since_update > max_age) {
            remove_tracker(i);
        }
    }
    return ret;
}

std::vector<Eigen::RowVectorXf> OCSTracker::State::update(const std::vector<ImgDetection>& detections,
                                                          const std::vector<Point3f>& spatialData,
                                                          bool trackOnly) {
    std::vector<Eigen::RowVectorXf> ret;
    std::unordered_map<uint32_t, std::pair<std::vector<ImgDetection>, std::vector<Point3f>>> dets_map;
    for(auto& [index, cs] : class_states) {
        dets_map.try_emplace(index, std::pair<std::vector<ImgDetection>, std::vector<Point3f>>());
    }
    for(size_t i = 0; i < detections.size(); i++) {
        uint32_t index = track_by_class ? detections[i].label : 0;
        dets_map[index].first.push_back(detections[i]);
        if(i <= spatialData.size())
            dets_map[index].second.push_back(spatialData[i]);
        else
            dets_map[index].second.push_back(Point3f(0, 0, 0));
    }
    for(auto& [index, dets] : dets_map) {
        class_states.try_emplace(index, this, det_thresh, max_age, min_hits, iou_threshold, delta_t, asso_func, inertia, use_byte);
        auto out = class_states.at(index).update(dets.first, dets.second, trackOnly);
        ret.insert(ret.end(), out.begin(), out.end());
    }
    return ret;
}
OCSTracker::State::State(float det_thresh_,
                         int max_age_,
                         int min_hits_,
                         float iou_threshold_,
                         TrackerIdAssignmentPolicy id_assignment_policy_,
                         bool track_by_class_,
                         uint32_t max_trackers_,
                         int delta_t_,
                         std::string asso_func_,
                         float inertia_,
                         bool use_byte_) {
    max_age = max_age_;
    min_hits = min_hits_;
    iou_threshold = iou_threshold_;
    id_assignment_policy = id_assignment_policy_;
    track_by_class = track_by_class_;
    max_trackers = max_trackers_;
    det_thresh = det_thresh_;
    delta_t = delta_t_;
    asso_func = asso_func_;
    inertia = inertia_;
    use_byte = use_byte_;
}

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> speed_direction_batch(const Eigen::MatrixXf& dets, const Eigen::MatrixXf& tracks) {
    Eigen::VectorXf CX1 = (dets.col(0) + dets.col(2)) / 2.0;
    Eigen::VectorXf CY1 = (dets.col(1) + dets.col(3)) / 2.f;
    Eigen::MatrixXf CX2 = (tracks.col(0) + tracks.col(2)) / 2.f;
    Eigen::MatrixXf CY2 = (tracks.col(1) + tracks.col(3)) / 2.f;
    Eigen::MatrixXf dx = CX1.transpose().replicate(tracks.rows(), 1) - CX2.replicate(1, dets.rows());
    Eigen::MatrixXf dy = CY1.transpose().replicate(tracks.rows(), 1) - CY2.replicate(1, dets.rows());
    Eigen::MatrixXf norm = (dx.array().square() + dy.array().square()).sqrt() + 1e-6f;
    dx = dx.array() / norm.array();
    dy = dy.array() / norm.array();
    return std::make_tuple(dy, dx);
}
Eigen::MatrixXf iou_batch(const Eigen::MatrixXf& bboxes1, const Eigen::MatrixXf& bboxes2) {
    Eigen::Matrix<float, Eigen::Dynamic, 1> a = bboxes1.col(0);  // bboxes1[..., 0] (n1,1)
    Eigen::Matrix<float, 1, Eigen::Dynamic> b = bboxes2.col(0);  // bboxes2[..., 0] (1,n2)
    Eigen::MatrixXf xx1 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(1);  // bboxes1[..., 1]
    b = bboxes2.col(1);  // bboxes2[..., 1]
    Eigen::MatrixXf yy1 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(2);  // bboxes1[..., 2]
    b = bboxes2.col(2);  // bboxes1[..., 2]
    Eigen::MatrixXf xx2 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    a = bboxes1.col(3);  // bboxes1[..., 3]
    b = bboxes2.col(3);  // bboxes1[..., 3]
    Eigen::MatrixXf yy2 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    Eigen::MatrixXf w = (xx2 - xx1).cwiseMax(0);
    Eigen::MatrixXf h = (yy2 - yy1).cwiseMax(0);
    Eigen::MatrixXf wh = w.array() * h.array();
    a = (bboxes1.col(2) - bboxes1.col(0)).array() * (bboxes1.col(3) - bboxes1.col(1)).array();
    b = (bboxes2.col(2) - bboxes2.col(0)).array() * (bboxes2.col(3) - bboxes2.col(1)).array();

    Eigen::MatrixXf part1_ = a.replicate(1, b.cols());
    Eigen::MatrixXf part2_ = b.replicate(a.rows(), 1);
    Eigen::MatrixXf Sum = part1_ + part2_ - wh;
    return wh.cwiseQuotient(Sum);
}
/**
 *
 * @param bboxes1 (n1,7)
 * @param bboxes2 (n2,5)
 * @return (n1,n2)
 */
Eigen::MatrixXf giou_batch(const Eigen::MatrixXf& bboxes1, const Eigen::MatrixXf& bboxes2) {
    Eigen::Matrix<float, Eigen::Dynamic, 1> a = bboxes1.col(0);  // bboxes1[..., 0] (n1,1)
    Eigen::Matrix<float, 1, Eigen::Dynamic> b = bboxes2.col(0);  // bboxes2[..., 0] (1,n2)
    Eigen::MatrixXf xx1 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(1);  // bboxes1[..., 1]
    b = bboxes2.col(1);  // bboxes2[..., 1]
    Eigen::MatrixXf yy1 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(2);  // bboxes1[..., 2]
    b = bboxes2.col(2);  // bboxes1[..., 2]
    Eigen::MatrixXf xx2 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    a = bboxes1.col(3);  // bboxes1[..., 3]
    b = bboxes2.col(3);  // bboxes1[..., 3]
    Eigen::MatrixXf yy2 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    Eigen::MatrixXf w = (xx2 - xx1).cwiseMax(0);
    Eigen::MatrixXf h = (yy2 - yy1).cwiseMax(0);
    Eigen::MatrixXf wh = w.array() * h.array();
    a = (bboxes1.col(2) - bboxes1.col(0)).array() * (bboxes1.col(3) - bboxes1.col(1)).array();
    b = (bboxes2.col(2) - bboxes2.col(0)).array() * (bboxes2.col(3) - bboxes2.col(1)).array();

    Eigen::MatrixXf part1_ = a.replicate(1, b.cols());
    Eigen::MatrixXf part2_ = b.replicate(a.rows(), 1);
    Eigen::MatrixXf Sum = part1_ + part2_ - wh;
    Eigen::MatrixXf iou = wh.cwiseQuotient(Sum);

    a = bboxes1.col(0);
    b = bboxes2.col(0);
    Eigen::MatrixXf xxc1 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    a = bboxes1.col(1);  // bboxes1[..., 1]
    b = bboxes2.col(1);  // bboxes2[..., 1]
    Eigen::MatrixXf yyc1 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
    a = bboxes1.col(2);  // bboxes1[..., 2]
    b = bboxes2.col(2);  // bboxes1[..., 2]
    Eigen::MatrixXf xxc2 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
    a = bboxes1.col(3);  // bboxes1[..., 3]
    b = bboxes2.col(3);  // bboxes1[..., 3]
    Eigen::MatrixXf yyc2 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));

    Eigen::MatrixXf wc = xxc2 - xxc1;
    Eigen::MatrixXf hc = yyc2 - yyc1;

    if((wc.array() > 0).all() && (hc.array() > 0).all())
        return iou;
    else {
        Eigen::MatrixXf area_enclose = wc.array() * hc.array();
        Eigen::MatrixXf giou = iou.array() - (area_enclose.array() - wh.array()) / area_enclose.array();
        giou = (giou.array() + 1) / 2.0;
        return giou;
    }
}

std::tuple<std::vector<Eigen::Matrix<int, 1, 2>>, std::vector<int>, std::vector<int>> associate(
    Eigen::MatrixXf detections, Eigen::MatrixXf trackers, float iou_threshold, Eigen::MatrixXf velocities, Eigen::MatrixXf previous_obs_, float vdc_weight) {
    if(trackers.rows() == 0) {
        std::vector<int> unmatched_dets;
        for(int i = 0; i < detections.rows(); i++) {
            unmatched_dets.push_back(i);
        }
        return std::make_tuple(std::vector<Eigen::Matrix<int, 1, 2>>(), unmatched_dets, std::vector<int>());
    }
    Eigen::MatrixXf Y, X;
    auto result = speed_direction_batch(detections, previous_obs_);
    Y = std::get<0>(result);
    X = std::get<1>(result);
    Eigen::MatrixXf inertia_Y = velocities.col(0);
    Eigen::MatrixXf inertia_X = velocities.col(1);
    Eigen::MatrixXf inertia_Y_ = inertia_Y.replicate(1, Y.cols());
    Eigen::MatrixXf inertia_X_ = inertia_X.replicate(1, X.cols());
    Eigen::MatrixXf diff_angle_cos = inertia_X_.array() * X.array() + inertia_Y_.array() * Y.array();

    diff_angle_cos = (diff_angle_cos.array().min(1.0f).max(-1.0f)).matrix();

    Eigen::MatrixXf diff_angle = Eigen::acos(diff_angle_cos.array());

    diff_angle = (pi / 2.0 - diff_angle.array().abs()).array() / (pi);
    Eigen::Array<bool, 1, Eigen::Dynamic> valid_mask = Eigen::Array<bool, Eigen::Dynamic, 1>::Ones(previous_obs_.rows());
    valid_mask = valid_mask.array() * ((previous_obs_.col(4).array() >= 0).transpose()).array();
    Eigen::MatrixXf iou_matrix = iou_batch(detections, trackers);
    Eigen::MatrixXf scores = detections.col(detections.cols() - 3).replicate(1, trackers.rows());
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> valid_mask_ = (valid_mask.transpose()).replicate(1, X.cols());
    Eigen::MatrixXf angle_diff_cost;
    auto valid_float = valid_mask_.cast<float>();
    auto intermediate_result = (valid_float.array() * diff_angle.array() * vdc_weight).transpose();
    angle_diff_cost.noalias() = (intermediate_result.array() * scores.array()).matrix();

    Eigen::Matrix<float, Eigen::Dynamic, 2> matched_indices(0, 2);
    if(std::min(iou_matrix.cols(), iou_matrix.rows()) > 0) {
        Eigen::MatrixXf a = (iou_matrix.array() > iou_threshold).cast<float>();
        float sum1 = (a.rowwise().sum()).maxCoeff();
        float sum0 = (a.colwise().sum()).maxCoeff();

        if((fabs((double)sum1 - 1) < 1e-12) && (fabs((double)sum0 - 1) < 1e-12)) {
            for(int i = 0; i < a.rows(); i++) {
                for(int j = 0; j < a.cols(); j++) {
                    if(a(i, j) > 0) {
                        Eigen::RowVectorXf row(2);
                        row << i, j;
                        matched_indices.conservativeResize(matched_indices.rows() + 1, Eigen::NoChange);
                        matched_indices.row(matched_indices.rows() - 1) = row;
                    }
                }
            }
        } else {
            Eigen::MatrixXf cost_matrix = iou_matrix.array() + angle_diff_cost.array();

            std::vector<std::vector<float>> cost_iou_matrix(cost_matrix.rows(), std::vector<float>(cost_matrix.cols()));
            for(int i = 0; i < cost_matrix.rows(); i++) {
                for(int j = 0; j < cost_matrix.cols(); j++) {
                    cost_iou_matrix[i][j] = -cost_matrix(i, j);
                }
            }

            std::vector<int> rowsol, colsol;
            execLapjv(cost_iou_matrix, rowsol, colsol, true, 0.01, true);
            for(uint32_t i = 0; i < rowsol.size(); i++) {
                if(rowsol.at(i) >= 0) {
                    Eigen::RowVectorXf row(2);
                    row << colsol.at(rowsol.at(i)), rowsol.at(i);
                    matched_indices.conservativeResize(matched_indices.rows() + 1, Eigen::NoChange);
                    matched_indices.row(matched_indices.rows() - 1) = row;
                }
            }
        }
    } else {
        matched_indices = Eigen::MatrixXf(0, 2);
    }
    std::vector<int> unmatched_detections;
    for(int i = 0; i < detections.rows(); i++) {
        if((matched_indices.col(0).array() == i).sum() == 0) {
            unmatched_detections.push_back(i);
        }
    }
    std::vector<int> unmatched_trackers;
    for(int i = 0; i < trackers.rows(); i++) {
        if((matched_indices.col(1).array() == i).sum() == 0) {
            unmatched_trackers.push_back(i);
        }
    }
    std::vector<Eigen::Matrix<int, 1, 2>> matches;
    Eigen::Matrix<int, 1, 2> tmp;
    for(int i = 0; i < matched_indices.rows(); i++) {
        tmp = (matched_indices.row(i)).cast<int>();
        if(iou_matrix(tmp(0), tmp(1)) < iou_threshold) {
            unmatched_detections.push_back(tmp(0));
            unmatched_trackers.push_back(tmp(1));
        } else {
            matches.push_back(tmp);
        }
    }
    if(matches.size() == 0) {
        matches.clear();
    }
    return std::make_tuple(matches, unmatched_detections, unmatched_trackers);
}

Eigen::RowVectorXf OCSTracker::State::KalmanBoxTracker::predict() {
    if(kf->x[6] + kf->x[2] <= 0) kf->x[6] *= 0.0f;
    kf->predict();
    age += 1;
    if(time_since_update > 0) hit_streak = 0;
    time_since_update += 1;

    auto vec_out = convert_x_to_bbox(kf->x);
    history.push_back(vec_out);
    return vec_out;
}
Eigen::VectorXf OCSTracker::State::KalmanBoxTracker::get_state() {
    return convert_x_to_bbox(kf->x);
}

OCSTracker::State::KalmanFilterNew::KalmanFilterNew(int dim_x_, int dim_z_) {
    dim_x = dim_x_;
    dim_z = dim_z_;
    x = Eigen::VectorXf::Zero(dim_x_, 1);
    P = Eigen::MatrixXf::Identity(dim_x_, dim_x_);
    Q = Eigen::MatrixXf::Identity(dim_x_, dim_x_);
    B = Eigen::MatrixXf::Identity(dim_x_, dim_x_);
    F = Eigen::MatrixXf::Identity(dim_x_, dim_x_);
    H = Eigen::MatrixXf::Zero(dim_z_, dim_x_);
    R = Eigen::MatrixXf::Identity(dim_z_, dim_z_);
    M = Eigen::MatrixXf::Zero(dim_x_, dim_z_);
    z = Eigen::VectorXf::Zero(dim_z_, 1);
    /*
        gain and residual are computed during the innovation step. We
        save them so that in case you want to inspect them for various
        purposes
    * */
    K = Eigen::MatrixXf::Zero(dim_x_, dim_z_);
    y = Eigen::VectorXf::Zero(dim_x_, 1);
    S = Eigen::MatrixXf::Zero(dim_z_, dim_z_);
    SI = Eigen::MatrixXf::Zero(dim_z_, dim_z_);

    x_prior = x;
    P_prior = P;
    x_post = x;
    P_post = P;
};
void OCSTracker::State::KalmanFilterNew::predict() {
    /**Predict next state (prior) using the Kalman filter state propagation
equations.
Parameters
----------
u : np.array, default 0
    Optional control vector.
B : np.array(dim_x, dim_u), or None
    Optional control transition matrix; a value of None
    will cause the filter to use `self.B`.
F : np.array(dim_x, dim_x), or None
    Optional state transition matrix; a value of None
    will cause the filter to use `self.F`.
Q : np.array(dim_x, dim_x), scalar, or None
    Optional process noise matrix; a value of None will cause the
    filter to use `self.Q`.
    */
    // x = Fx+Bu
    x = F * x;
    // P = FPF' + Q
    P.noalias() = _alpha_sq * ((F * P) * F.transpose()) + Q;
    x_prior = x;
    P_prior = P;
}
void OCSTracker::State::KalmanFilterNew::update(Eigen::VectorXf z_) {
    /*
    Add a new measurement (z) to the Kalman filter.
    If z is None, nothing is computed. However, x_post and P_post are
    updated with the prior (x_prior, P_prior), and self.z is set to None.
    Parameters
    ----------
    z : (dim_z, 1): array_like
        measurement for this update. z can be a scalar if dim_z is 1,
        otherwise it must be convertible to a column vector.
        If you pass in a value of H, z must be a column vector the
        of the correct size.
    R : np.array, scalar, or None
        Optionally provide R to override the measurement noise for this
        one call, otherwise  self.R will be used.
    H : np.array, or None
        Optionally provide H to override the measurement function for this
        one call, otherwise self.H will be used.
     * */
    history_obs.push_back(z_);
    if(z_.size() == 0) {
        if(true == observed) freeze();
        observed = false;

        z = Eigen::VectorXf::Zero(dim_z, 1);
        x_post = x;
        P_post = P;
        y = Eigen::VectorXf::Zero(dim_z, 1);
        return;
    }

    if(false == observed) unfreeze();
    observed = true;

    // y = z - Hx
    y.noalias() = z_ - H * x;

    Eigen::MatrixXf PHT;
    PHT.noalias() = P * H.transpose();
    // S = HPH' + R
    S.noalias() = H * PHT + R;

    SI = S.inverse();
    // K = PH'SI
    K.noalias() = PHT * SI;
    x.noalias() = x + K * y;
    /*This is more numerically stable and works for non-optimal K vs the
     * equation P = (I-KH)P usually seen in the literature.*/
    Eigen::MatrixXf I_KH;
    I_KH.noalias() = I - K * H;
    Eigen::MatrixXf P_INT;
    P_INT.noalias() = (I_KH * P);
    P.noalias() = (P_INT * I_KH.transpose()) + ((K * R) * K.transpose());
    // save the measurement and posterior state
    z = z_;
    x_post = x;
    P_post = P;
}
void OCSTracker::State::KalmanFilterNew::freeze() {
    attr_saved.IsInitialized = true;
    attr_saved.x = x;
    attr_saved.P = P;
    attr_saved.Q = Q;
    attr_saved.B = B;
    attr_saved.F = F;
    attr_saved.H = H;
    attr_saved.R = R;
    attr_saved._alpha_sq = _alpha_sq;
    attr_saved.M = M;
    attr_saved.z = z;
    attr_saved.K = K;
    attr_saved.y = y;
    attr_saved.S = S;
    attr_saved.SI = SI;
    attr_saved.x_prior = x_prior;
    attr_saved.P_prior = P_prior;
    attr_saved.x_post = x_post;
    attr_saved.P_post = P_post;
    attr_saved.history_obs = history_obs;
}
void OCSTracker::State::KalmanFilterNew::unfreeze() {
    if(true == attr_saved.IsInitialized) {
        new_history = history_obs;
        x = attr_saved.x;
        P = attr_saved.P;
        Q = attr_saved.Q;
        B = attr_saved.B;
        F = attr_saved.F;
        H = attr_saved.H;
        R = attr_saved.R;
        _alpha_sq = attr_saved._alpha_sq;
        M = attr_saved.M;
        z = attr_saved.z;
        K = attr_saved.K;
        y = attr_saved.y;
        S = attr_saved.S;
        SI = attr_saved.SI;
        x_prior = attr_saved.x_prior;
        P_prior = attr_saved.P_prior;
        x_post = attr_saved.x_post;
        history_obs.erase(history_obs.end() - 1);
        Eigen::VectorXf box1;
        Eigen::VectorXf box2;
        int lastNotNullIndex = -1;
        int secondLastNotNullIndex = -1;
        for(int i = new_history.size() - 1; i >= 0; i--) {
            if(new_history[i].size() > 0) {
                if(lastNotNullIndex == -1) {
                    // The current element is the last non -NULLPTR element
                    lastNotNullIndex = i;
                    box2 = new_history.at(lastNotNullIndex);
                } else if(secondLastNotNullIndex == -1) {
                    // The current element is the last second non -NULLPTR element
                    secondLastNotNullIndex = i;
                    box1 = new_history.at(secondLastNotNullIndex);
                    break;
                }
            }
        }

        double time_gap = lastNotNullIndex - secondLastNotNullIndex;

        double x1 = (double)box1[0];
        double x2 = (double)box2[0];
        double y1 = (double)box1[1];
        double y2 = (double)box2[1];
        double w1 = (double)std::sqrt(box1[2] * box1[3]);
        double h1 = (double)std::sqrt(box1[2] / box1[3]);
        double w2 = (double)std::sqrt(box2[2] * box2[3]);
        double h2 = (double)std::sqrt(box2[2] / box2[3]);

        double dx = (x2 - x1) / time_gap;
        double dy = (y1 - y2) / time_gap;
        double dw = (w2 - w1) / time_gap;
        double dh = (h2 - h1) / time_gap;

        for(int i = 0; i < time_gap; i++) {
            /*
                The default virtual trajectory generation is by linear
                motion (constant speed hypothesis), you could modify this
                part to implement your own.
             */
            double x = x1 + (i + 1) * dx;
            double y = y1 + (i + 1) * dy;
            double w = w1 + (i + 1) * dw;
            double h = h1 + (i + 1) * dh;
            double s = w * h;
            double r = w / (h * 1.0);
            Eigen::VectorXf new_box(4, 1);
            new_box << x, y, s, r;
            /*
                I still use predict-update loop here to refresh the parameters,
                but this can be faster by directly modifying the internal parameters
                as suggested in the paper. I keep this naive but slow way for
                easy read and understanding
             */

            // y = z - Hx
            this->y.noalias() = new_box - this->H * this->x;

            Eigen::MatrixXf PHT;
            PHT.noalias() = this->P * this->H.transpose();
            // S = HPH' + R
            this->S.noalias() = this->H * PHT + this->R;

            this->SI = (this->S).inverse();
            // K = PH'SI
            this->K.noalias() = PHT * this->SI;

            this->x.noalias() = this->x + this->K * this->y;

            /*This is more numerically stable and works for non-optimal K vs the
             * equation P = (I-KH)P usually seen in the literature.*/
            Eigen::MatrixXf I_KH;
            I_KH.noalias() = this->I - this->K * this->H;
            Eigen::MatrixXf P_INT;
            P_INT.noalias() = (I_KH * this->P);
            this->P.noalias() = (P_INT * I_KH.transpose()) + ((this->K * this->R) * (this->K).transpose());
            // save the measurement and posterior state
            this->z = new_box;
            this->x_post = this->x;
            this->P_post = this->P;
            if(i != (time_gap - 1)) predict();
        }
    } /* if attr_saved is null, do nothing */
}

Eigen::VectorXf convert_bbox_to_z(Eigen::VectorXf bbox) {
    double w = (double)(bbox[2] - bbox[0]);
    double h = (double)(bbox[3] - bbox[1]);
    double x = (double)bbox[0] + w / 2.0;
    double y = (double)bbox[1] + h / 2.0;
    double s = w * h;
    double r = w / (h + 1e-6);
    Eigen::MatrixXf z(4, 1);
    z << x, y, s, r;
    return z;
}
Eigen::VectorXf speed_direction(Eigen::VectorXf bbox1, Eigen::VectorXf bbox2) {
    double cx1 = (double)(bbox1[0] + bbox1[2]) / 2.0;
    double cy1 = (double)(bbox1[1] + bbox1[3]) / 2.0;
    double cx2 = (double)(bbox2[0] + bbox2[2]) / 2.0;
    double cy2 = (double)(bbox2[1] + bbox2[3]) / 2.0;
    Eigen::VectorXf speed(2, 1);
    speed << cy2 - cy1, cx2 - cx1;
    double norm = sqrt(pow(cy2 - cy1, 2) + pow(cx2 - cx1, 2)) + 1e-6;
    return speed / norm;
}
Eigen::VectorXf convert_x_to_bbox(Eigen::VectorXf x) {
    float w = std::sqrt(x(2) * x(3));
    float h = x(2) / w;
    Eigen::VectorXf bbox = Eigen::VectorXf::Ones(4, 1);
    bbox << x(0) - w / 2, x(1) - h / 2, x(0) + w / 2, x(1) + h / 2;
    return bbox;
}
Eigen::VectorXf k_previous_obs(const std::map<int, Eigen::VectorXf>& observations, int cur_age, int k) {
    if(observations.empty()) return Eigen::VectorXf::Constant(5, -1.0);

    for(int i = 0; i < k; ++i) {
        int dt = k - i;
        auto it = observations.find(cur_age - dt);
        if(it != observations.end()) return it->second;
    }
    auto max_iter = observations.rbegin();
    return max_iter->second;
}

/** Column-reduction and reduction transfer for a dense cost matrix.
 */
int_t _ccrrt_dense(const uint_t n, cost_t* cost[], int_t* free_rows, int_t* x, int_t* y, cost_t* v) {
    int_t n_free_rows;
    boolean* unique;

    for(uint_t i = 0; i < n; i++) {
        x[i] = -1;
        v[i] = LARGE;
        y[i] = 0;
    }
    for(uint_t i = 0; i < n; i++) {
        for(uint_t j = 0; j < n; j++) {
            const cost_t c = cost[i][j];
            if(c < v[j]) {
                v[j] = c;
                y[j] = i;
            }
            PRINTF("i=%d, j=%d, c[i,j]=%f, v[j]=%f y[j]=%d\n", i, j, c, v[j], y[j]);
        }
    }
    PRINT_COST_ARRAY(v, n);
    PRINT_INDEX_ARRAY(y, n);
    unique = new boolean[n];

    memset(unique, 1, n);
    {
        int_t j = n;
        do {
            j--;
            const int_t i = y[j];
            if(x[i] < 0) {
                x[i] = j;
            } else {
                unique[i] = 1;
                y[j] = -1;
            }
        } while(j > 0);
    }
    n_free_rows = 0;
    for(uint_t i = 0; i < n; i++) {
        if(x[i] < 0) {
            free_rows[n_free_rows++] = i;
        } else if(unique[i]) {
            const int_t j = x[i];
            cost_t min = LARGE;
            for(uint_t j2 = 0; j2 < n; j2++) {
                if(j2 == (uint_t)j) {
                    continue;
                }
                const cost_t c = cost[i][j2] - v[j2];
                if(c < min) {
                    min = c;
                }
            }
            PRINTF("v[%d] = %f - %f\n", j, v[j], min);
            v[j] -= min;
        }
    }
    delete[] unique;
    return n_free_rows;
}

/** Augmenting row reduction for a dense cost matrix.
 */
int_t _carr_dense(const uint_t n, cost_t* cost[], const uint_t n_free_rows, int_t* free_rows, int_t* x, int_t* y, cost_t* v) {
    uint_t current = 0;
    int_t new_free_rows = 0;
    uint_t rr_cnt = 0;
    PRINT_INDEX_ARRAY(x, n);
    PRINT_INDEX_ARRAY(y, n);
    PRINT_COST_ARRAY(v, n);
    PRINT_INDEX_ARRAY(free_rows, n_free_rows);
    while(current < n_free_rows) {
        int_t i0;
        int_t j1, j2;
        cost_t v1, v2, v1_new;
        boolean v1_lowers;

        rr_cnt++;
        PRINTF("current = %d rr_cnt = %d\n", current, rr_cnt);
        const int_t free_i = free_rows[current++];
        j1 = 0;
        v1 = cost[free_i][0] - v[0];
        j2 = -1;
        v2 = LARGE;
        for(uint_t j = 1; j < n; j++) {
            PRINTF("%d = %f %d = %f\n", j1, v1, j2, v2);
            const cost_t c = cost[free_i][j] - v[j];
            if(c < v2) {
                if(c >= v1) {
                    v2 = c;
                    j2 = j;
                } else {
                    v2 = v1;
                    v1 = c;
                    j2 = j1;
                    j1 = j;
                }
            }
        }
        i0 = y[j1];
        v1_new = v[j1] - (v2 - v1);
        v1_lowers = v1_new < v[j1];
        PRINTF("%d %d 1=%d,%f 2=%d,%f v1'=%f(%d,%g) \n", free_i, i0, j1, v1, j2, v2, v1_new, v1_lowers, v[j1] - v1_new);
        if(rr_cnt < current * n) {
            if(v1_lowers) {
                v[j1] = v1_new;
            } else if(i0 >= 0 && j2 >= 0) {
                j1 = j2;
                i0 = y[j2];
            }
            if(i0 >= 0) {
                if(v1_lowers) {
                    free_rows[--current] = i0;
                } else {
                    free_rows[new_free_rows++] = i0;
                }
            }
        } else {
            PRINTF("rr_cnt=%d >= %d (current=%d * n=%d)\n", rr_cnt, current * n, current, n);
            if(i0 >= 0) {
                free_rows[new_free_rows++] = i0;
            }
        }
        x[free_i] = j1;
        y[j1] = free_i;
    }
    return new_free_rows;
}

/** Find columns with minimum d[j] and put them on the SCAN list.
 */
uint_t _find_dense(const uint_t n, uint_t lo, cost_t* d, int_t* cols) {
    uint_t hi = lo + 1;
    cost_t mind = d[cols[lo]];
    for(uint_t k = hi; k < n; k++) {
        int_t j = cols[k];
        if(d[j] <= mind) {
            if(d[j] < mind) {
                hi = lo;
                mind = d[j];
            }
            cols[k] = cols[hi];
            cols[hi++] = j;
        }
    }
    return hi;
}

// Scan all columns in TODO starting from arbitrary column in SCAN
// and try to decrease d of the TODO columns using the SCAN column.
int_t _scan_dense(const uint_t n, cost_t* cost[], uint_t* plo, uint_t* phi, cost_t* d, int_t* cols, int_t* pred, int_t* y, cost_t* v) {
    uint_t lo = *plo;
    uint_t hi = *phi;
    cost_t h, cred_ij;

    while(lo != hi) {
        int_t j = cols[lo++];
        const int_t i = y[j];
        const cost_t mind = d[j];
        h = cost[i][j] - v[j] - mind;
        PRINTF("i=%d j=%d h=%f\n", i, j, h);
        // For all columns in TODO
        for(uint_t k = hi; k < n; k++) {
            j = cols[k];
            cred_ij = cost[i][j] - v[j] - h;
            if(cred_ij < d[j]) {
                d[j] = cred_ij;
                pred[j] = i;
                if(cred_ij == mind) {
                    if(y[j] < 0) {
                        return j;
                    }
                    cols[k] = cols[hi];
                    cols[hi++] = j;
                }
            }
        }
    }
    *plo = lo;
    *phi = hi;
    return -1;
}

/** Single iteration of modified Dijkstra shortest path algorithm as explained in the JV paper.
 *
 * This is a dense matrix version.
 *
 * \return The closest free column index.
 */
int_t find_path_dense(const uint_t n, cost_t* cost[], const int_t start_i, int_t* y, cost_t* v, int_t* pred) {
    uint_t lo = 0, hi = 0;
    int_t final_j = -1;
    uint_t n_ready = 0;
    int_t* cols;
    cost_t* d;

    cols = new int_t[n];
    d = new cost_t[n];

    for(uint_t i = 0; i < n; i++) {
        cols[i] = i;
        pred[i] = start_i;
        d[i] = cost[start_i][i] - v[i];
    }
    PRINT_COST_ARRAY(d, n);
    while(final_j == -1) {
        // No columns left on the SCAN list.
        if(lo == hi) {
            PRINTF("%d..%d -> find\n", lo, hi);
            n_ready = lo;
            hi = _find_dense(n, lo, d, cols);
            PRINTF("check %d..%d\n", lo, hi);
            PRINT_INDEX_ARRAY(cols, n);
            for(uint_t k = lo; k < hi; k++) {
                const int_t j = cols[k];
                if(y[j] < 0) {
                    final_j = j;
                }
            }
        }
        if(final_j == -1) {
            PRINTF("%d..%d -> scan\n", lo, hi);
            final_j = _scan_dense(n, cost, &lo, &hi, d, cols, pred, y, v);
            PRINT_COST_ARRAY(d, n);
            PRINT_INDEX_ARRAY(cols, n);
            PRINT_INDEX_ARRAY(pred, n);
        }
    }

    PRINTF("found final_j=%d\n", final_j);
    PRINT_INDEX_ARRAY(cols, n);
    {
        const cost_t mind = d[cols[lo]];
        for(uint_t k = 0; k < n_ready; k++) {
            const int_t j = cols[k];
            v[j] += d[j] - mind;
        }
    }

    delete[] cols;
    delete[] d;

    return final_j;
}

/** Augment for a dense cost matrix.
 */
int_t _ca_dense(const uint_t n, cost_t* cost[], const uint_t n_free_rows, int_t* free_rows, int_t* x, int_t* y, cost_t* v) {
    int_t* pred;

    pred = new int_t[n];

    for(int_t* pfree_i = free_rows; pfree_i < free_rows + n_free_rows; pfree_i++) {
        int_t i = -1, j;
        uint_t k = 0;

        PRINTF("looking at free_i=%d\n", *pfree_i);
        j = find_path_dense(n, cost, *pfree_i, y, v, pred);
        ASSERT(j >= 0);
        ASSERT(j < n);
        while(i != *pfree_i) {
            PRINTF("augment %d\n", j);
            PRINT_INDEX_ARRAY(pred, n);
            i = pred[j];
            PRINTF("y[%d]=%d -> %d\n", j, y[j], i);
            y[j] = i;
            PRINT_INDEX_ARRAY(x, n);
            SWAP_INDICES(j, x[i]);
            k++;
            if(k >= n) {
                ASSERT(FALSE);
            }
        }
    }
    delete[] pred;
    return 0;
}

/**
 * Solve dense sparse LAP.
 */
int lapjv_internal(const uint_t n, cost_t* cost[], int_t* x, int_t* y) {
    int ret;
    int_t* free_rows;
    cost_t* v;

    free_rows = new int_t[n];
    v = new cost_t[n];

    ret = _ccrrt_dense(n, cost, free_rows, x, y, v);
    int i = 0;
    while(ret > 0 && i < 2) {
        ret = _carr_dense(n, cost, ret, free_rows, x, y, v);
        i++;
    }
    if(ret > 0) {
        ret = _ca_dense(n, cost, ret, free_rows, x, y, v);
    }
    delete[] v;
    delete[] free_rows;
    return ret;
}
float execLapjv(
    const std::vector<std::vector<float>>& cost, std::vector<int>& rowsol, std::vector<int>& colsol, bool extend_cost, float cost_limit, bool return_cost) {
    std::vector<std::vector<float>> cost_c;
    cost_c.assign(cost.begin(), cost.end());
    std::vector<std::vector<float>> cost_c_extended;
    int n_rows = cost.size();
    int n_cols = cost[0].size();
    rowsol.resize(n_rows);
    colsol.resize(n_cols);
    int n = 0;
    if(n_rows == n_cols) {
        n = n_rows;
    } else {
        if(!extend_cost) {
            throw std::runtime_error("The `extend_cost` variable should set True");
        }
    }

    if(extend_cost || cost_limit < std::numeric_limits<float>::max()) {
        n = n_rows + n_cols;
        cost_c_extended.resize(n);
        for(size_t i = 0; i < cost_c_extended.size(); i++) cost_c_extended[i].resize(n);

        if(cost_limit < std::numeric_limits<float>::max()) {
            for(size_t i = 0; i < cost_c_extended.size(); i++) {
                for(size_t j = 0; j < cost_c_extended[i].size(); j++) {
                    cost_c_extended[i][j] = cost_limit / 2.0f;
                }
            }
        } else {
            float cost_max = -1;
            for(size_t i = 0; i < cost_c.size(); i++) {
                for(size_t j = 0; j < cost_c[i].size(); j++) {
                    if(cost_c[i][j] > cost_max) cost_max = cost_c[i][j];
                }
            }
            for(size_t i = 0; i < cost_c_extended.size(); i++) {
                for(size_t j = 0; j < cost_c_extended[i].size(); j++) {
                    cost_c_extended[i][j] = cost_max + 1;
                }
            }
        }

        for(size_t i = n_rows; i < cost_c_extended.size(); i++) {
            for(size_t j = n_cols; j < cost_c_extended[i].size(); j++) {
                cost_c_extended[i][j] = 0;
            }
        }
        for(int i = 0; i < n_rows; i++) {
            for(int j = 0; j < n_cols; j++) {
                cost_c_extended[i][j] = cost_c[i][j];
            }
        }

        cost_c.clear();
        cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
    }

    float** cost_ptr;
    cost_ptr = new float*[n];
    for(int i = 0; i < n; i++) cost_ptr[i] = new float[n];
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            cost_ptr[i][j] = cost_c[i][j];
        }
    }
    int* x_c = new int[n];
    int* y_c = new int[n];
    int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
    if(ret != 0) {
        throw std::runtime_error("The result of lapjv_internal() is invalid.");
    }
    float opt = 0.0;
    if(n != n_rows) {
        for(int i = 0; i < n; i++) {
            if(x_c[i] >= n_cols) x_c[i] = -1;
            if(y_c[i] >= n_rows) y_c[i] = -1;
        }
        for(int i = 0; i < n_rows; i++) {
            rowsol[i] = x_c[i];
        }
        for(int i = 0; i < n_cols; i++) {
            colsol[i] = y_c[i];
        }

        if(return_cost) {
            for(size_t i = 0; i < rowsol.size(); i++) {
                if(rowsol[i] != -1) {
                    opt += cost_ptr[i][rowsol[i]];
                }
            }
        }
    } else if(return_cost) {
        for(size_t i = 0; i < rowsol.size(); i++) {
            opt += cost_ptr[i][rowsol[i]];
        }
    }

    for(int i = 0; i < n; i++) {
        delete[] cost_ptr[i];
    }
    delete[] cost_ptr;
    delete[] x_c;
    delete[] y_c;
    return opt;
}

OCSTracker::OCSTracker(const ObjectTrackerProperties& properties)
    : maxObjectsToTrack(properties.maxObjectsToTrack),
      trackerIdAssignmentPolicy(properties.trackerIdAssignmentPolicy),
      trackingPerClass(properties.trackingPerClass),
      occlusionRatioThreshold(properties.occlusionRatioThreshold),
      trackletMaxLifespan(properties.trackletMaxLifespan),
      trackletBirthThreshold(properties.trackletBirthThreshold) {}
OCSTracker::~OCSTracker() {}
void OCSTracker::init(const ImgFrame& /* frame */, const std::vector<ImgDetection>& detections, const std::vector<Point3f>& spatialData) {
    this->state = std::make_unique<State>(0.f,
                                          this->trackletMaxLifespan,
                                          this->trackletBirthThreshold,
                                          this->occlusionRatioThreshold,
                                          this->trackerIdAssignmentPolicy,
                                          this->trackingPerClass,
                                          this->maxObjectsToTrack,
                                          1,
                                          "giou",
                                          0.3941737016672115,
                                          true);
    this->state->update(detections, spatialData);
}
void OCSTracker::update(const ImgFrame& /* frame */, const std::vector<ImgDetection>& detections, const std::vector<Point3f>& spatialData) {
    if(!this->state) {
        throw std::runtime_error("OCSTracker is not initialized. Call init() first.");
    }
    this->state->update(detections, spatialData);
}
void OCSTracker::track(const ImgFrame& /* frame */) {
    if(!this->state) {
        throw std::runtime_error("OCSTracker is not initialized. Call init() first.");
    }
    this->state->update(std::vector<ImgDetection>(), std::vector<Point3f>(), true);
}
void OCSTracker::configure(const ObjectTrackerConfig& config) {
    if(!this->state) {
        throw std::runtime_error("OCSTracker is not initialized. Call init() first.");
    }
    this->state->remove_tracklets(config.trackletIdsToRemove);
}
std::vector<Tracklet> OCSTracker::getTracklets() const {
    if(!this->state) {
        throw std::runtime_error("OCSTracker is not initialized. Call init() first.");
    }
    return this->state->get_tracklets();
}

}  // namespace impl
}  // namespace dai
