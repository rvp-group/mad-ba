#pragma once
#include <dirent.h>
#include <gtest/gtest.h>
#include <random>

#include "srrg_geometry/geometry_defs.h"

// ds helper macros aimed to extend gtest macros
#define ASSERT_IN_RANGE(VALUE, MIN, MAX) \
  ASSERT_GE(VALUE, MIN);                 \
  ASSERT_LE(VALUE, MAX)

#define ASSERT_NOTNULL(POINTER) ASSERT_NE(POINTER, nullptr)
#define ASSERT_NULL(POINTER) ASSERT_EQ(POINTER, nullptr)

#define ASSERT_EQ_VECTOR3F(VALUE, VALUE_REFERENCE)                                       \
  std::cerr << "ASSERT_EQ_VECTOR3F|deprecated, change to: ASSERT_EQ_EIGEN" << std::endl; \
  ASSERT_EQ(VALUE(0), VALUE_REFERENCE(0));                                               \
  ASSERT_EQ(VALUE(1), VALUE_REFERENCE(1));                                               \
  ASSERT_EQ(VALUE(2), VALUE_REFERENCE(2))

#define ASSERT_EQ_EIGEN(VALUE_, VALUE_REFERENCE_)                        \
  ASSERT_EQ(VALUE_.rows(), VALUE_REFERENCE_.rows());                     \
  ASSERT_EQ(VALUE_.cols(), VALUE_REFERENCE_.cols());                     \
  for (decltype(VALUE_.rows()) r = 0; r < VALUE_.rows(); ++r) {          \
    for (decltype(VALUE_.cols()) c = 0; c < VALUE_.cols(); ++c) {        \
      ASSERT_EQ(VALUE_.matrix()(r, c), VALUE_REFERENCE_.matrix()(r, c)); \
    }                                                                    \
  }

#define ASSERT_NEAR_VECTOR3F(VALUE, VALUE_REFERENCE, TOLERANCE)                              \
  std::cerr << "ASSERT_NEAR_VECTOR3F|deprecated, change to: ASSERT_NEAR_EIGEN" << std::endl; \
  ASSERT_NEAR(VALUE(0), VALUE_REFERENCE(0), TOLERANCE);                                      \
  ASSERT_NEAR(VALUE(1), VALUE_REFERENCE(1), TOLERANCE);                                      \
  ASSERT_NEAR(VALUE(2), VALUE_REFERENCE(2), TOLERANCE)

#define ASSERT_NEAR_EIGEN(VALUE_, VALUE_REFERENCE_, TOLERANCE_)                        \
  ASSERT_EQ(VALUE_.rows(), VALUE_REFERENCE_.rows());                                   \
  ASSERT_EQ(VALUE_.cols(), VALUE_REFERENCE_.cols());                                   \
  for (decltype(VALUE_.rows()) r = 0; r < VALUE_.rows(); ++r) {                        \
    for (decltype(VALUE_.cols()) c = 0; c < VALUE_.cols(); ++c) {                      \
      ASSERT_NEAR(VALUE_.matrix()(r, c), VALUE_REFERENCE_.matrix()(r, c), TOLERANCE_); \
    }                                                                                  \
  }

#define ASSERT_LT_ABS(VALUE, VALUE_REFERENCE) ASSERT_LT(std::fabs(VALUE), VALUE_REFERENCE)

//! unittest namespace, not to be used in any other context!
namespace srrg2_test {

  //! current user folder used for test execution
  static std::string filepath_user_folder = "";

  //! current srrg source folder used for test execution
  static std::string filepath_folder_srrg = "";

  //! current srrg2 source folder used for test execution
  static std::string filepath_folder_srrg2 = "";

  //! flag that is set to true if test data path is set correctly
  static bool is_test_data_available = false;

  //! parse current user folder from CLI parameters
  void parseUserFolderUNIX(const std::string& filepath_) {
    // ds determine local srrg test data path
    // ds TODO remove this mother of all evil
    const size_t index_file_root_slash = filepath_.find_first_of("/");
    if (index_file_root_slash == std::string::npos) {
      std::cerr << "parseUserFolder|ERROR: unable to infer file root (/) from CLI" << std::endl;
      return;
    }
    const size_t index_root_folder_slash = filepath_.find_first_of("/", index_file_root_slash + 1);
    if (index_root_folder_slash == std::string::npos) {
      std::cerr << "parseUserFolder|WARNING: unable to infer home "
                   "folder (/root, /home) from CLI"
                << std::endl;
      return;
    }
    const size_t index_home_folder_slash =
      filepath_.find_first_of("/", index_root_folder_slash + 1);
    if (index_home_folder_slash == std::string::npos) {
      std::cerr << "parseUserFolder|WARNING: unable to infer user folder(/ "
                   "home / user) from CLI "
                << std::endl;
      return;
    }

    // ds assemble user folder (might be invalid at this point)
    filepath_user_folder = filepath_.substr(0, index_home_folder_slash);

    // ds check if we are in the root folder (CI case) if home is not present
    if (filepath_user_folder.find("home") == std::string::npos) {
      std::cerr << "parseUserFolder|WARNING: home folder not found - assuming root" << std::endl;
      filepath_user_folder = filepath_.substr(0, index_root_folder_slash);
    }
    filepath_folder_srrg  = filepath_user_folder + "/source/srrg/";
    filepath_folder_srrg2 = filepath_user_folder + "/source/srrg2/";

    // ds check if the derived folders do not exist
    if (opendir(filepath_folder_srrg.c_str()) == nullptr) {
      std::cerr << "parseUserFolder|WARNING: srrg folder not existing: '" + filepath_folder_srrg +
                     "'"
                << std::endl;
      return;
    }
    if (opendir(filepath_folder_srrg2.c_str()) == nullptr) {
      std::cerr << "parseUserFolder|WARNING: srrg2 folder not existing: '" + filepath_folder_srrg2 +
                     "'"
                << std::endl;
      return;
    }

    // ds fine if still here
    std::cerr << "parseUserFolderUNIX|using folder SRRG: '" << filepath_folder_srrg << "'"
              << std::endl;
    std::cerr << "parseUserFolderUNIX|using folder SRRG2: '" << filepath_folder_srrg2 << "'"
              << std::endl;
    is_test_data_available = true;
  }

  //! test entry points
  int runTests(int argc_, char** argv_, const bool& use_test_folder_ = false) {
    if (use_test_folder_) {
      parseUserFolderUNIX(argv_[0]);
    }
    testing::InitGoogleTest(&argc_, argv_);
    return RUN_ALL_TESTS();
  }

  template <typename RealType_ = float>
  class RandomNumberGeneratorBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RandomNumberGeneratorBase() {
      // ds constant seed for reproducible tests
      _random_device = std::mt19937(0);
    }
    virtual ~RandomNumberGeneratorBase() {
    }
    virtual RealType_ getRandomScalar(const RealType_& mean_,
                                      const RealType_& standard_deviation_) = 0;

  protected:
    // ds random number generation
    std::mt19937 _random_device;
  };

  template <typename RealType_ = float>
  class RandomNumberGeneratorUniform : public RandomNumberGeneratorBase<RealType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RandomNumberGeneratorUniform() {
      // ds constant seed for reproducible tests
      srand(0);
    }
    virtual ~RandomNumberGeneratorUniform() {
    }
    virtual RealType_ getRandomScalar(const RealType_& mean_,
                                      const RealType_& standard_deviation_) override {
      const RealType_ minimum(mean_ - standard_deviation_);
      const RealType_ maximum(mean_ + standard_deviation_);
      std::uniform_real_distribution<RealType_> distribution(minimum, maximum);
      return distribution(this->_random_device);
    }
    template <int Dimension_>
    srrg2_core::Vector_<RealType_, Dimension_>
    getRandomVector(const srrg2_core::Vector_<RealType_, Dimension_>& mean_,
                    const srrg2_core::Vector_<RealType_, Dimension_>& standard_deviation_) {
      const srrg2_core::Vector_<RealType_, Dimension_> minimum(mean_ - standard_deviation_);
      const srrg2_core::Vector_<RealType_, Dimension_> maximum(mean_ + standard_deviation_);
      srrg2_core::Vector_<RealType_, Dimension_> random_vector;
      for (int i = 0; i < Dimension_; ++i) {
        std::uniform_real_distribution<RealType_> distribution(minimum(i), maximum(i));
        random_vector(i) = distribution(this->_random_device);
      }
      return random_vector;
    }
  };

  template <typename RealType_ = float>
  class RandomNumberGeneratorGaussian : public RandomNumberGeneratorBase<RealType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~RandomNumberGeneratorGaussian() {
    }
    virtual RealType_ getRandomScalar(const RealType_& mean_,
                                      const RealType_& standard_deviation_) override {
      std::normal_distribution<RealType_> distribution(mean_, standard_deviation_);
      return distribution(this->_random_device);
    }
    template <int Dimension_>
    srrg2_core::Vector_<RealType_, Dimension_>
    getRandomVector(const srrg2_core::Vector_<RealType_, Dimension_>& mean_,
                    const srrg2_core::Vector_<RealType_, Dimension_>& standard_deviation_) {
      srrg2_core::Vector_<RealType_, Dimension_> random_vector;
      for (int i = 0; i < Dimension_; ++i) {
        std::normal_distribution<RealType_> distribution(mean_(i), standard_deviation_(i));
        random_vector(i) = distribution(this->_random_device);
      }
      return random_vector;
    }
  };

} // namespace srrg2_test
