#pragma once
#include "srrg_data_structures/matrix.h"
#include <Eigen/Core>
#include <cstdint>
#include <opencv2/opencv.hpp>
#if CV_VERSION_MAJOR > 3
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/imgproc/types_c.h>
#endif

namespace srrg2_core {

#define CV_32FC5 CV_32FC(5)

  enum ImageType {
    TYPE_8UC1    = CV_8UC1,
    TYPE_16UC1   = CV_16UC1,
    TYPE_32SC1   = CV_32SC1,
    TYPE_32FC1   = CV_32FC1,
    TYPE_64FC1   = CV_64FC1,
    TYPE_8UC3    = CV_8UC3,
    TYPE_32FC3   = CV_32FC3,
    TYPE_8UC4    = CV_8UC4,
    TYPE_32FC4   = CV_32FC4,
    TYPE_32FC5   = CV_32FC5,
    TYPE_UNKNOWN = -1
  };

  // TODO: check this note by mc
  // mc I can also use the sqrt directly here
  // E.g. FOUR_PIXELS = 2
  // I can avoid sqrt
  enum DOWNSAMPLE_PIXELS {
    FOUR_PIXELS       = 4,
    NINE_PIXELS       = 9,
    SIXTEEN_PIXELS    = 16,
    TWENTYFIVE_PIXELS = 25
  };

  // bdc to be compliant w/ opencv, we need a cv::Size equivalent
  struct ImageSize {
    size_t width;
    size_t height;

    //! @brief ImageSize constructor
    ImageSize(const size_t& width_, const size_t& height_) : width(width_), height(height_) {
    }

    //! @brief compares sizes of the Image
    const bool operator==(const ImageSize& img_size) const;

    //! @brief compares sizes of the Image
    const bool operator!=(const ImageSize& img_size) const;

    //! @brief compares sizes of the Image with cv::Size object
    const bool operator!=(const cv::Size& cv_size) const;

    //! @brief compares sizes of the Image with cv::Size object
    const bool operator==(const cv::Size& cv_size) const;
  };

  class BaseImage {
  public:
    virtual ~BaseImage();

    //! @brief exports the image the image to CV
    virtual void toCv(cv::Mat& dest_) const = 0;

    //! @brief imports the image from CV
    virtual void fromCv(const cv::Mat& dest_) = 0;

    //! @brief access to the raw data array
    virtual char* rawData() const = 0;

    //! @brief returns size of the image
    virtual const ImageSize imageSize() const = 0;

    //! @brief cv type of the matrix
    const ImageType& type() const {
      return _type;
    }

    //! @brief returns the number of channels in a matrix
    const int numChannels() const {
      return _num_channels;
    }

    /* TODO: implement these two methods
       virtual BaseImage* load(istream& is);
       virtual void save(ostream& is);
    */

    //! @brief returns rows of the associated matrix
    virtual const std::size_t rows() const = 0;

    //! @brief returns cols of the associated matrix
    virtual const std::size_t cols() const = 0;

    //! @brief writes on this image from buffer
    virtual void fromBuffer(const uint8_t* buffer) = 0;

    //! @brief scales all the entries by scale_factor_
    virtual void scale(float scale_factor_) = 0;

    //! @brief builds a new image from a cv::Mat
    static BaseImage* newImagefromCv(const cv::Mat& other_);

  protected:
    //! @brief protected ctor, we do not allow the creation of plain BaseImage objects
    BaseImage(const ImageType& type_, int num_channels) :
      _type(type_),
      _num_channels(num_channels) {
    }

    ImageType _type;
    int _num_channels;
  };

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  class Image_ : public Matrix_<typename Eigen::Matrix<Scalar_, NumChannels_, 1>>,
                 public BaseImage {
  public:
    using Scalar       = Scalar_;
    using CVVecType    = cv::Vec<Scalar, NumChannels_>;
    using EigenVecType = Eigen::Matrix<Scalar, NumChannels_, 1>;
    using MatrixType   = Matrix_<EigenVecType>;
    using Traits       = typename MatrixType::Traits;
    using CellType     = typename MatrixType::CellType;

    //! @brief Image c'tor
    Image_(const int rows = 0, const int cols = 0) :
      MatrixType(rows, cols),
      BaseImage(ImageType_, NumChannels_) {
    }

    //! @brief `assign` operator
    Image_<Scalar, ImageType_, NumChannels_>& operator=(const MatrixType& m) {
      MatrixType::assign(*this, m);
      return *this;
    }

    //! @brief returns rows of the associated matrix w/ data
    const std::size_t rows() const override {
      return MatrixType::rows();
    }

    //! @brief returns cols of the associated matrix w/ data
    const std::size_t cols() const override {
      return MatrixType::cols();
    }

    //! @brief returns size of the image (can be compared w/ cv::Size())
    const ImageSize imageSize() const override {
      return ImageSize(MatrixType::cols(), MatrixType::rows());
    }

    //! @brief returns pointer to rawData of the associated matrix
    inline char* rawData() const override {
      return (char*) MatrixType::data().data();
    }

    //! @brief converts the image to a cv::Mat
    void toCv(cv::Mat& dest_) const override;

    //! @brief populate this image from cv::Mat
    void fromCv(const cv::Mat& src_) override;

    //! @brief converts the matrix to image
    void matrix2image(srrg2_core::BaseImage& dest_, const srrg2_core::Matrix_<float>& matrix_);

    //! @brief converts the matrix to cv::Mat and plots it
    void matrix2cv(const std::string& image_name_,
                   const srrg2_core::Matrix_<float>& matrix_,
                   const float& rows_,
                   const float& cols_);
    //! @brief converts the matrix to cv::Mat and plots it while coloring pixels in idx_vector
    //! red
    void matrixWithPoints2cv(const std::string& image_name_,
                             const srrg2_core::Matrix_<float>& matrix_,
                             const srrg2_core::StdVectorEigenVector2i& idx_vector_,
                             const float& rows_,
                             const float& cols_);

    //! @brief populate this image a buffer
    void fromBuffer(const uint8_t* buffer) override;

    //! @brief converts to a selected type
    template <typename DestImageType_, typename ScaleType_>
    void convertTo(DestImageType_& dest, const ScaleType_& s = ScaleType_(1)) const;

    //! @brief scales the entries of the image by scale_factor_
    void scale(float scale_factor_) override;

    //! @brief saturate all the entries of the image to saturate_val_
    void saturate(CellType saturate_val_);

    //! @brief performs downsampling given the specified pixels
    void downsample(Image_<Scalar, ImageType_, NumChannels_>& dest, DOWNSAMPLE_PIXELS npixels);

    //! @brief computes the integral image
    void integralImage(Image_<Scalar, ImageType_, NumChannels_>& output_) const;

    //! @brief blur the image using the provided window size
    void blur(Image_<Scalar, ImageType_, NumChannels_>& output_, const size_t& window_);

    //! @brief performs downsampling given the specified scale for rows and cols
    void downsample(Image_<Scalar, ImageType_, NumChannels_>& dest, int row_scale, int col_scale);
  };

  template <class Scalar_, ImageType ImageType_>
  class Image_<Scalar_, ImageType_, 1> : public Matrix_<Scalar_>, public BaseImage {
  public:
    using Scalar     = Scalar_;
    using MatrixType = Matrix_<Scalar>;
    using MaskImage  = Image_<uint8_t, TYPE_8UC1, 1>;
    using Traits     = typename MatrixType::Traits;
    using CellType   = typename MatrixType::CellType;

    //! @brief Image c'tor from rows and cols
    Image_(const int rows = 0, const int cols = 0) :
      MatrixType(rows, cols),
      BaseImage(ImageType_, 1) {
    }

    //! @brief Image c'tor from size
    Image_(const ImageSize& size) : MatrixType(size.height, size.width), BaseImage(ImageType_, 1) {
    }

    //! @brief copy operator
    Image_<Scalar, ImageType_, 1>& operator=(const MatrixType& m) {
      MatrixType::assign(*this, m);
      return *this;
    }

    //! @brief returns rows of the associated matrix w/ data
    const std::size_t rows() const override {
      return MatrixType::rows();
    }

    //! @brief returns cols of the associated matrix w/ data
    const std::size_t cols() const override {
      return MatrixType::cols();
    }

    //! @brief returns size of the image (can be compared w/ cv::Size())
    const ImageSize imageSize() const override {
      return ImageSize(MatrixType::cols(), MatrixType::rows());
    }

    //! @brief returns pointer to rawData of the associated matrix
    inline char* rawData() const override {
      return (char*) MatrixType::data().data();
    }

    //! @brief converts the image to a cv::Mat
    void toCv(cv::Mat& dest_) const override;

    //! @brief populate this image from cv::Mat
    void fromCv(const cv::Mat& src_) override;

    //! @brief populate this image a buffer
    void fromBuffer(const uint8_t* buffer) override;

    //! @brief converts to a selected type
    template <typename DestImageType_, typename ScaleType_>
    void convertTo(DestImageType_& dest, const ScaleType_& s) const;

    //! @brief scales the entries of the image by scale_factor_
    void scale(float scale_factor_) override;

    //! @brief saturate all the entries of the image to saturate_val_
    void saturate(CellType saturate_val_);

    //! @brief performs downsampling given the specified pixels
    void downsample(Image_<Scalar, ImageType_, 1>& dest, DOWNSAMPLE_PIXELS npixels);

    //! @brief converts the matrix to image
    void matrix2image(srrg2_core::BaseImage& dest_, const srrg2_core::Matrix_<float>& matrix_);

    //! @brief converts the matrix to cv::Mat and plots it
    void matrix2cv(const std::string& image_name_,
                   const srrg2_core::Matrix_<float>& matrix_,
                   const float& rows_,
                   const float& cols_);

    //! @brief converts the matrix to cv::Mat and plots it while coloring pixels in idx_vector
    //! red
    void matrixWithPoints2cv(const std::string& image_name_,
                             const srrg2_core::Matrix_<float>& matrix_,
                             const srrg2_core::StdVectorEigenVector2i& idx_vector_,
                             const float& rows_,
                             const float& cols_);

    //! @brief returns a mask w/ {1,0} depending on the required value
    //! TODO: should the behavior depend on the type:
    //! e.g. w/ float returns 1 if less or eq?
    MaskImage operator==(const size_t& value_) const;

    MaskImage operator>=(const size_t& value_) const;

    //! @brief performs downsampling given the specified scale for rows and cols
    void downsample(Image_<Scalar, ImageType_, 1>& dest, int row_scale, int col_scale);
  };

  using ImageUInt8     = Image_<uint8_t, TYPE_8UC1, 1>;
  using ImageUInt16    = Image_<uint16_t, TYPE_16UC1, 1>;
  using ImageInt       = Image_<int32_t, TYPE_32SC1, 1>;
  using ImageFloat     = Image_<float, TYPE_32FC1, 1>;
  using ImageDouble    = Image_<double, TYPE_64FC1, 1>;
  using ImageVector3uc = Image_<uint8_t, TYPE_8UC3, 3>;
  using ImageVector3f  = Image_<float, TYPE_32FC3, 3>;
  using ImageVector4uc = Image_<uint8_t, TYPE_8UC4, 4>;
  using ImageVector4f  = Image_<float, TYPE_32FC4, 4>;
  using ImageVector5f  = Image_<float, TYPE_32FC5, 5>;

} // namespace srrg2_core

// bdc includes implementation of template methods
#include "image.hpp"
