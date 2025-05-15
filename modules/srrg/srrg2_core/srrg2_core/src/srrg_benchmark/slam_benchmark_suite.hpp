#pragma once
#include "srrg_messages/instances.h"
#include <srrg_geometry/geometry3d.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages/message_handlers/message_source_platform.h>
#include <numeric>
namespace srrg2_core {

  // ds SLAM benchmark facility TODO move me?
  template <typename EstimateType_>
  class SLAMBenchmarkSuite {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType                       = EstimateType_;
    using TranslationVectorType              = Vector_<float, EstimateType::Dim>;
    using RotationMatrixType                 = MatrixN_<float, EstimateType::Dim>;
    static constexpr size_t Dimension        = EstimateType::Dim;
    static constexpr size_t AngularDimension = (EstimateType::Dim == 2) ? 1 : EstimateType::Dim;
    using RotationVectorType                 = Vector_<float, AngularDimension>;

    struct AbsoluteEstimateStamped {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      AbsoluteEstimateStamped(const EstimateType& estimate_, const double& timestamp_seconds_) :
        estimate(estimate_),
        timestamp_seconds(timestamp_seconds_) {
      }
      EstimateType estimate;
      double timestamp_seconds;
    };
    using AbsoluteEstimateStampedVector =
      std::vector<AbsoluteEstimateStamped, Eigen::aligned_allocator<AbsoluteEstimateStamped>>;

    struct RelativeEstimateStamped {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      RelativeEstimateStamped(const EstimateType& estimate_source_in_target_,
                              const double& timestamp_seconds_source_,
                              const double& timestamp_seconds_target_) :
        estimate(estimate_source_in_target_),
        timestamp_seconds_source(timestamp_seconds_source_),
        timestamp_seconds_target(timestamp_seconds_target_) {
      }
      EstimateType estimate;
      double timestamp_seconds_source;
      double timestamp_seconds_target;
    };
    using RelativeEstimateStampedVector =
      std::vector<RelativeEstimateStamped, Eigen::aligned_allocator<RelativeEstimateStamped>>;

    virtual ~SLAMBenchmarkSuite() {
    }

    //! loads messages from a dataset file
    virtual void loadDataset(const std::string& filepath_,
                             const size_t& number_of_message_packs_to_read_ = -1,
                             const size_t& number_of_message_pack_to_start_ = 0) = 0;

    virtual void loadDatasetWithTF(const std::string& filepath_,
                                   MessageSourcePlatformPtr platform_source_,
                                   const size_t& number_of_message_packs_to_read_ = -1,
                                   const size_t& number_of_message_pack_to_start_ = 0) {
    }
    //! loads ground truth poses from file (s)
    virtual void loadGroundTruth(const std::string& filepath_,
                                 const std::string& filepath_additional_ = std::string()) = 0;

    virtual void loadGroundTruthFromBOSS(const std::string& filepath_, const std::string& topic_) {
    }

    //! fetch the next message from memory (buffered with loadDataset)
    virtual BaseSensorMessagePtr getMessage() = 0;

    //! resets benchmark suite (e.g. moves message position to start)
    virtual void reset() = 0;

  public:
    //! set computed SLAM pose estimate
    void setPoseEstimate(const EstimateType& estimate_,
                         const double& timestamp_seconds_,
                         const double processing_duration_seconds_ = 0) {
      assert(timestamp_seconds_ >= 0);

      // ds if previous element has identical or higher timestamp
      if (!_estimated_poses.empty()) {
        if (_estimated_poses.back().timestamp_seconds >= timestamp_seconds_) {
          throw std::runtime_error("SLAMBenchmarkSuite::setEstimate|ERROR: "
                                   "previous estimate is ahead of provided timestamp" +
                                   std::to_string(timestamp_seconds_));
        }
      }

      // ds create an absolute pose entry
      _estimated_poses.push_back(AbsoluteEstimateStamped(estimate_, timestamp_seconds_));

      // ds add processing duration
      _processing_durations_seconds.push_back(processing_duration_seconds_);
    }

    //! computes performance statistics
    void compute() {
      assert(_estimated_poses.size() == _processing_durations_seconds.size());
      std::cerr << "SLAMBenchmarkSuite::compute|translation # DOF: " << Dimension << std::endl;
      std::cerr << "SLAMBenchmarkSuite::compute|rotation # DOF: " << AngularDimension << std::endl;
      std::cerr << "SLAMBenchmarkSuite::compute|relative ground truth poses: "
                << _relative_ground_truth_poses.size()
                << " (maximum delta (s): " << _maximum_timestamp_delta_seconds << ")" << std::endl;
      std::cerr << "SLAMBenchmarkSuite::compute|absolute estimated poses: "
                << _estimated_poses.size() << std::endl;
      if (_estimated_poses.empty()) {
        std::cerr << "SLAMBenchmarkSuite::compute|no estimated poses, ignoring call" << std::endl;
        return;
      }
      if (_relative_ground_truth_poses.empty()) {
        std::cerr << "SLAMBenchmarkSuite::compute|no ground truth poses set, ignoring call"
                  << std::endl;
        return;
      }

      // ds sort relative ground truth measurements by timestamps in increasing order for evaluation
      // ds the provided pose estimates must already be ordered correspondingly
      std::sort(_relative_ground_truth_poses.begin(),
                _relative_ground_truth_poses.end(),
                [](const RelativeEstimateStamped& a_, const RelativeEstimateStamped& b_) {
                  return a_.timestamp_seconds_source < b_.timestamp_seconds_source;
                });

      // ds compute total processing duration and standard deviation
      const double total_processing_duration_seconds = std::accumulate(
        _processing_durations_seconds.begin(), _processing_durations_seconds.end(), 0.0);
      const double mean_processing_duration_seconds =
        total_processing_duration_seconds / _processing_durations_seconds.size();
      double standard_deviation_mean_processing_duration_seconds = 0;
      for (const double& processing_duration_seconds : _processing_durations_seconds) {
        const double delta = processing_duration_seconds - mean_processing_duration_seconds;
        standard_deviation_mean_processing_duration_seconds += delta * delta;
      }
      standard_deviation_mean_processing_duration_seconds = std::sqrt(
        standard_deviation_mean_processing_duration_seconds / _processing_durations_seconds.size());

      // ds error result buffers: translation
      std::vector<TranslationVectorType> absolute_translation_errors_squared;
      std::vector<TranslationVectorType> absolute_translation_errors;

      // ds relative errors are decoupled as either of the coordinates can be zero
      std::vector<std::vector<Vector1f>> relative_translation_errors;
      for (size_t d = 0; d < Dimension; ++d) {
        relative_translation_errors.push_back(std::vector<Vector1f>(0));
      }

      // ds error result buffers: rotation
      std::vector<RotationVectorType> absolute_rotation_errors_squared;
      std::vector<RotationVectorType> absolute_rotation_errors;

      // ds relative errors are decoupled as either of the coordinates can be zero
      std::vector<std::vector<Vector1f>> relative_rotation_errors;
      for (size_t d = 0; d < AngularDimension; ++d) {
        relative_rotation_errors.push_back(std::vector<Vector1f>(0));
      }

      // ds loop over consecutive pairs of estimates TODO refactor this bullcrap
      size_t index_gt_latest = 0;
      std::cerr << std::setprecision(15); // ds making timestamps readable on console
      for (size_t i = 1; i < _estimated_poses.size(); ++i) {
        const AbsoluteEstimateStamped& source_estimate(_estimated_poses[i - 1]);
        const AbsoluteEstimateStamped& target_estimate(_estimated_poses[i]);

        // ds retrieve closest ground truth poses for current estimates
        EstimateType relative_truth(EstimateType::Identity());

        // ds look for corresponding ground truth poses and update latest (increasing by definition)
        bool found_ground_truth_sample = false;
        for (size_t index_gt = index_gt_latest;
             index_gt < _relative_ground_truth_poses.size() && !found_ground_truth_sample;
             ++index_gt) {
          // ds check if source timestamp is close enough and le than the next candidate
          double current_timestamp_delta_seconds =
            std::fabs(_relative_ground_truth_poses[index_gt].timestamp_seconds_source -
                      source_estimate.timestamp_seconds);
          double next_timestamp_delta_seconds = std::numeric_limits<double>::max();
          if (index_gt + 1 < _relative_ground_truth_poses.size()) {
            next_timestamp_delta_seconds =
              std::fabs(_relative_ground_truth_poses[index_gt + 1].timestamp_seconds_source -
                        source_estimate.timestamp_seconds);
          }
          if (current_timestamp_delta_seconds < _maximum_timestamp_delta_seconds &&
              current_timestamp_delta_seconds <= next_timestamp_delta_seconds) {
            //          std::cerr << index_gt << " found source timestamp (s): "
            //                    << _relative_ground_truth_poses[index_gt].timestamp_seconds_source
            //                    << " ~ "
            //                    << source_estimate.timestamp_seconds << std::endl;
            EstimateType current_relative_truth(_relative_ground_truth_poses[index_gt].estimate);

            // ds browse through remaining poses to find the matching target timestamp
            while (index_gt < _relative_ground_truth_poses.size()) {
              //            std::cerr << index_gt << " current target timestamp (s): "
              //                      <<
              //                      _relative_ground_truth_poses[index_gt].timestamp_seconds_target
              //                      << " ~ "
              //                      << target_estimate.timestamp_seconds << std::endl;

              // ds check if target timestamp is close enough and le than the next candidate
              current_timestamp_delta_seconds =
                std::fabs(_relative_ground_truth_poses[index_gt].timestamp_seconds_target -
                          target_estimate.timestamp_seconds);
              next_timestamp_delta_seconds = std::numeric_limits<double>::max();
              if (index_gt + 1 < _relative_ground_truth_poses.size()) {
                next_timestamp_delta_seconds =
                  std::fabs(_relative_ground_truth_poses[index_gt + 1].timestamp_seconds_target -
                            target_estimate.timestamp_seconds);
              }
              if (current_timestamp_delta_seconds < _maximum_timestamp_delta_seconds &&
                  current_timestamp_delta_seconds <= next_timestamp_delta_seconds) {
                // ds move latest timestamp to found target and escape inner loop
                index_gt_latest           = index_gt;
                found_ground_truth_sample = true;
                relative_truth            = current_relative_truth;
                //              std::cerr << "----------------------------------" << std::endl;
                break;
              }

              // ds if we're already ahead in time
              if (_relative_ground_truth_poses[index_gt].timestamp_seconds_target >
                  target_estimate.timestamp_seconds) {
                // ds terminate search - and outer loop as well
                //              std::cerr << "skip timestamp ahead in time (s): "
                //                        <<
                //                        _relative_ground_truth_poses[index_gt].timestamp_seconds_target
                //                        << std::endl;
                index_gt = _relative_ground_truth_poses.size();
                break;
              }

              // ds propagate relative estimate
              current_relative_truth =
                _relative_ground_truth_poses[index_gt].estimate * current_relative_truth;

              // ds advance
              ++index_gt;
            }
          }
        }
        if (!found_ground_truth_sample) {
          std::cerr
            << "SLAMBenchmarkSuite::compute|WARNING: unable to find ground truth sample for "
               "poses with timestamp (s): {"
            << source_estimate.timestamp_seconds << ", " << target_estimate.timestamp_seconds << "}"
            << std::endl;
          continue;
        }

        // ds compute relative estimate
        const EstimateType relative_estimate =
          target_estimate.estimate.inverse() * source_estimate.estimate;

        // ds compute identity error
        const EstimateType identity_error = relative_estimate.inverse() * relative_truth;
        const TranslationVectorType translation_delta(identity_error.translation());
        const TranslationVectorType translation_truth(relative_truth.translation());

        // ds compute and store errors in translation
        _storeErrorNorms(translation_delta,
                         translation_truth,
                         absolute_translation_errors_squared,
                         absolute_translation_errors,
                         relative_translation_errors);

        // ds compute angles (mapping to the corresponding overload in SO2/SO3
        const RotationVectorType angles_truth =
          _getEulerAngles(static_cast<RotationMatrixType>(relative_truth.linear()));
        RotationVectorType angles_delta =
          _getEulerAngles(static_cast<RotationMatrixType>(identity_error.linear()));
        for (size_t d = 0; d < AngularDimension; ++d) {
          angles_delta[d] = std::fabs(angles_delta[d]);
          if (angles_delta[d] > M_PI) {
            angles_delta[d] -= M_PI;
          }
        }

        // ds compute and store errors in rotation
        _storeErrorNorms(angles_delta,
                         angles_truth,
                         absolute_rotation_errors_squared,
                         absolute_rotation_errors,
                         relative_rotation_errors);
      }

      // ds working buffers for printing TODO ugly until refactored
      TranslationVectorType mean_translation_error;
      TranslationVectorType standard_deviation_translation_error;
      RotationVectorType mean_rotation_error;
      RotationVectorType standard_deviation_rotation_error;

      // ds final result
      std::cerr << FG_BGREEN(
                     "\n8========================== BENCHMARK RESULT ==========================D")
                << std::endl;
      std::cerr << "\nSYSTEM PERFORMANCE" << std::endl;
      std::cerr << " > dataset file path: '" << _dataset_path << "'" << std::endl;
      std::cerr << " > ground truth file path: '" << _ground_truth_path << "'" << std::endl;
      std::cerr << " > # processed messages: " << _estimated_poses.size() << std::endl;
      std::cerr << " > # relative pose estimates: " << _estimated_poses.size() - 1 << std::endl;
      std::cerr << " > # relative ground truth pose estimates: "
                << _relative_ground_truth_poses.size() << std::endl;
      std::cerr << " > total processing duration (s): " << total_processing_duration_seconds
                << std::endl;
      std::cerr << " > mean processing duration (s): " << mean_processing_duration_seconds
                << " (std deviation: " << standard_deviation_mean_processing_duration_seconds << ")"
                << std::endl;
      std::cerr << " > FPS (!): " << _estimated_poses.size() / total_processing_duration_seconds
                << std::endl;

      std::cerr << "\nTRANSLATION ERROR" << std::endl;
      if (_computeMeanAndStandardDeviation(absolute_translation_errors_squared,
                                           _mean_translation_rmse,
                                           _standard_deviation_translation_rmse)) {
        std::cerr << " > absolute RMSE / ATE (m) per dimension: " << std::endl;
        for (size_t d = 0; d < Dimension; ++d) {
          _mean_translation_rmse(d) = std::sqrt(_mean_translation_rmse(d));
          _standard_deviation_translation_rmse(d) =
            std::sqrt(_standard_deviation_translation_rmse(d));
          std::cerr << "      [" << _getLabelTranslationAxis(d) << "] " << _mean_translation_rmse(d)
                    << " (standard deviation: " << _standard_deviation_translation_rmse(d)
                    << ", samples: " << absolute_translation_errors_squared.size() << "/"
                    << _estimated_poses.size() - 1 << ")" << std::endl;
        }
      }
      if (_computeMeanAndStandardDeviation(absolute_translation_errors,
                                           mean_translation_error,
                                           standard_deviation_translation_error)) {
        std::cerr << " > absolute ME (m) per dimension: " << std::endl;
        for (size_t d = 0; d < Dimension; ++d) {
          std::cerr << "      [" << _getLabelTranslationAxis(d) << "] " << mean_translation_error[d]
                    << " (standard deviation: " << standard_deviation_translation_error[d]
                    << ", samples: " << absolute_translation_errors.size() << "/"
                    << _estimated_poses.size() - 1 << ")" << std::endl;
        }
      }
      std::cerr << " > relative ME (%) per dimension: " << std::endl;
      for (size_t d = 0; d < Dimension; ++d) {
        Vector1f mean(Vector1f::Zero());
        Vector1f standard_deviation(Vector1f::Zero());
        if (_computeMeanAndStandardDeviation(
              relative_translation_errors[d], mean, standard_deviation)) {
          std::cerr << "      [" << _getLabelTranslationAxis(d) << "] " << mean[0] * 100
                    << " (standard deviation: " << standard_deviation[0] * 100
                    << ", samples: " << relative_translation_errors[d].size() << "/"
                    << _estimated_poses.size() - 1 << ")" << std::endl;
        }
      }

      std::cerr << "\nROTATION ERROR" << std::endl;
      if (_computeMeanAndStandardDeviation(absolute_rotation_errors_squared,
                                           _mean_rotation_rmse,
                                           _standard_deviation_rotation_rmse)) {
        std::cerr << " > absolute RMSE / ATE (deg) per dimension: " << std::endl;
        for (size_t d = 0; d < AngularDimension; ++d) {
          _mean_rotation_rmse(d)               = std::sqrt(_mean_rotation_rmse(d));
          _standard_deviation_rotation_rmse(d) = std::sqrt(_standard_deviation_rotation_rmse(d));
          std::cerr << "      [" << _getLabelRotationAngle(d) << "] " << _mean_rotation_rmse(d)
                    << " (standard deviation: " << _standard_deviation_rotation_rmse(d)
                    << ", samples: " << absolute_rotation_errors_squared.size() << "/"
                    << _estimated_poses.size() - 1 << ")" << std::endl;
        }
      }
      if (_computeMeanAndStandardDeviation(
            absolute_rotation_errors, mean_rotation_error, standard_deviation_rotation_error)) {
        std::cerr << " > absolute ME (deg) per dimension: " << std::endl;
        for (size_t d = 0; d < AngularDimension; ++d) {
          std::cerr << "      [" << _getLabelRotationAngle(d) << "] "
                    << mean_rotation_error[d] / M_PI * 180
                    << " (standard deviation: " << standard_deviation_rotation_error[d] / M_PI * 180
                    << ", samples: " << absolute_rotation_errors.size() << "/"
                    << _estimated_poses.size() - 1 << ")" << std::endl;
        }
      }
      std::cerr << " > relative ME (%) per dimension: " << std::endl;
      for (size_t d = 0; d < AngularDimension; ++d) {
        Vector1f mean(Vector1f::Zero());
        Vector1f standard_deviation(Vector1f::Zero());
        if (_computeMeanAndStandardDeviation(
              relative_rotation_errors[d], mean, standard_deviation)) {
          std::cerr << "      [" << _getLabelRotationAngle(d) << "] " << mean[0] * 100
                    << " (standard deviation: " << standard_deviation[0] * 100
                    << ", samples: " << relative_rotation_errors[d].size() << "/"
                    << _estimated_poses.size() - 1 << ")" << std::endl;
        }
      }

      std::cerr << FG_BGREEN(
                     "\n8======================================================================D\n")
                << std::endl;
    }

    // ds perform regression check over all metrics TODO check other metrics than RMSE as well
    bool isRegression(const TranslationVectorType& maximum_mean_translation_rmse_,
                      const TranslationVectorType& maximum_standard_deviation_translation_rmse_,
                      const RotationVectorType& maximum_mean_rotation_rmse_,
                      const RotationVectorType& maximum_standard_deviation_rotation_rmse_) {
      bool is_regression = false;

      // ds evaluate regression in translation estimation
      for (size_t i = 0; i < Dimension; ++i) {
        if (_mean_translation_rmse(i) > maximum_mean_translation_rmse_(i)) {
          std::cerr << FG_RED("regression in TRANSLATION mean RMSE [")
                    << _getLabelTranslationAxis(i) << FG_RED("] ") << _mean_translation_rmse(i)
                    << " > " << maximum_mean_translation_rmse_(i) << std::endl;
          is_regression = true;
        }
        if (_standard_deviation_translation_rmse(i) >
            maximum_standard_deviation_translation_rmse_(i)) {
          std::cerr << FG_RED("regression in TRANSLATION sdev RMSE [")
                    << _getLabelTranslationAxis(i) << FG_RED("] ")
                    << _standard_deviation_translation_rmse(i) << " > "
                    << maximum_standard_deviation_translation_rmse_(i) << std::endl;
          is_regression = true;
        }
      }

      // ds evaluate regression in orientation estimation
      for (size_t i = 0; i < AngularDimension; ++i) {
        if (_mean_rotation_rmse(i) > maximum_mean_rotation_rmse_(i)) {
          std::cerr << FG_RED("regression in ROTATION mean RMSE [") << _getLabelRotationAngle(i)
                    << FG_RED("] ") << _mean_rotation_rmse(i) << " > "
                    << maximum_mean_rotation_rmse_(i) << std::endl;
          is_regression = true;
        }
        if (_standard_deviation_rotation_rmse(i) > maximum_standard_deviation_rotation_rmse_(i)) {
          std::cerr << FG_RED("regression in ROTATION sdev RMSE [") << _getLabelRotationAngle(i)
                    << FG_RED("] ") << _standard_deviation_rotation_rmse(i) << " > "
                    << maximum_standard_deviation_rotation_rmse_(i) << std::endl;
          is_regression = true;
        }
      }
      return is_regression;
    }

    // ds individual getters
    const TranslationVectorType& getMeanTranslationRMSE() const {
      return _mean_translation_rmse;
    }
    const TranslationVectorType& getStandardDeviationTranslationRMSE() const {
      return _standard_deviation_translation_rmse;
    }
    const RotationVectorType& getMeanRotationRMSE() const {
      return _mean_rotation_rmse;
    }
    const RotationVectorType& getStandardDeviationRotationRMSE() const {
      return _standard_deviation_rotation_rmse;
    }

    // ds save trajectory to disk in inheriting dataset format
    virtual void writeTrajectoryToFile(const std::string& filename_ = "trajectory.txt") const = 0;

  protected:
    // ds local helper functions to have a human-understandable error values
    Vector1f _getEulerAngles(const Matrix2f& rotation_) const {
      return Vector1f(std::fabs(atan2(rotation_(1, 0), rotation_(0, 0))));
    }
    Vector3f _getEulerAngles(const Matrix3f& rotation_) const {
      return rotation_.eulerAngles(0, 1, 2);
    }

    // ds stores different error norms for translations and rotation angle vectors
    template <int Dimension_>
    bool _storeErrorNorms(const Vector_<float, Dimension_>& error_,
                          const Vector_<float, Dimension_>& ground_truth_delta_,
                          std::vector<Vector_<float, Dimension_>>& squared_norms_,
                          std::vector<Vector_<float, Dimension_>>& norms_,
                          std::vector<std::vector<Vector1f>>& relative_norms_) {
      using VectorType = Vector_<float, Dimension_>;
      VectorType error_translation_squared_norm(VectorType::Zero());
      VectorType error_translation_norm(VectorType::Zero());
      VectorType relative_truth_norm(VectorType::Zero());
      for (size_t i = 0; i < Dimension_; ++i) {
        error_translation_norm[i]         = std::fabs(error_[i]);
        error_translation_squared_norm[i] = error_translation_norm[i] * error_translation_norm[i];
        relative_truth_norm[i]            = std::fabs(ground_truth_delta_[i]);
      }

      // ds accumulate statistics (skipping very small translations for relative computation)
      squared_norms_.push_back(error_translation_squared_norm);
      norms_.push_back(error_translation_norm);

      // ds compute relative error in each dimension (and avoid division by zero)
      for (size_t i = 0; i < Dimension_; ++i) {
        if (relative_truth_norm[i] > 0.01) {
          relative_norms_[i].push_back(
            Vector1f(error_translation_norm[i] / relative_truth_norm[i]));
        }
      }
      return true;
    }

    // ds computes mean and std dev statistics for translations and rotation angle vectors
    template <int Dimension_>
    bool _computeMeanAndStandardDeviation(const std::vector<Vector_<float, Dimension_>>& values_,
                                          Vector_<float, Dimension_>& mean_,
                                          Vector_<float, Dimension_>& standard_deviation_) {
      using VectorType = Vector_<float, Dimension_>;
      if (values_.empty()) {
        // ds unable to compute statistics
        return false;
      }

      // ds compute mean
      VectorType accumulated_values(VectorType::Zero());
      for (const VectorType& value : values_) {
        for (size_t i = 0; i < Dimension_; ++i) {
          accumulated_values[i] += value[i];
        }
      }
      for (size_t i = 0; i < Dimension_; ++i) {
        mean_[i] = accumulated_values[i] / values_.size();
      }

      // ds compute standard deviation
      VectorType accumulated_variances(VectorType::Zero());
      for (const VectorType& value : values_) {
        for (size_t i = 0; i < Dimension_; ++i) {
          const float delta = value[i] - mean_[i];
          accumulated_variances[i] += delta * delta;
        }
      }
      for (size_t i = 0; i < Dimension_; ++i) {
        standard_deviation_[i] = std::sqrt(accumulated_variances[i] / values_.size());
      }

      // ds success
      return true;
    }

    // ds readability
    const std::string _getLabelTranslationAxis(const size_t& index_) {
      assert(index_ < 3);
      const std::vector<std::string> labels = {"x", "y", "z"};
      return labels[index_];
    }
    const std::string _getLabelRotationAngle(const size_t& index_) {
      assert(index_ < 3);
      const std::vector<std::string> labels = {"roll", "pitch", "yaw"};
      if (AngularDimension == 1) {
        return "yaw";
      } else {
        return labels[index_];
      }
    }

  protected:
    // ds maximum permitted timestamp difference for pose comparison (depends per dataset)
    // ds change this value in the suite class instead of using a setter in the benchmark executable
    double _maximum_timestamp_delta_seconds = 0.1;

    // ds stamped pose estimates provided by SLAM system
    AbsoluteEstimateStampedVector _estimated_poses;

    std::string _dataset_path      = "";
    std::string _ground_truth_path = "";

    // ds REQUIRED relative ground truth measurements
    RelativeEstimateStampedVector _relative_ground_truth_poses;

    // ds processing durations (added with estimate)
    std::vector<double> _processing_durations_seconds;

    // ds running variables
    TranslationVectorType _mean_translation_rmse               = TranslationVectorType::Zero();
    TranslationVectorType _standard_deviation_translation_rmse = TranslationVectorType::Zero();
    RotationVectorType _mean_rotation_rmse                     = RotationVectorType();
    RotationVectorType _standard_deviation_rotation_rmse       = RotationVectorType();
  };

  using SLAMBenchmarkSuiteSE2Ptr = std::unique_ptr<SLAMBenchmarkSuite<Isometry2f>>;
  using SLAMBenchmarkSuiteSE3Ptr = std::unique_ptr<SLAMBenchmarkSuite<Isometry3f>>;

} // namespace srrg2_core
