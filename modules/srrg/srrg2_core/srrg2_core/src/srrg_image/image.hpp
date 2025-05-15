namespace srrg2_core {

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::toCv(cv::Mat& dest_) const {
    dest_.create(this->rows(), this->cols(), ImageType_);
    for (size_t r = 0; r < this->rows(); ++r) {
      CVVecType* dest_ptr         = dest_.ptr<CVVecType>(r);
      const EigenVecType* src_ptr = this->rowPtr(r);
      for (size_t c = 0; c < this->cols(); ++c, ++dest_ptr, ++src_ptr) {
        CVVecType& dest_item         = *dest_ptr;
        const EigenVecType& src_item = *src_ptr;
        for (int n = 0; n < NumChannels_; ++n) {
          dest_item[n] = src_item[n];
        }
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::fromCv(const cv::Mat& src_) {
    assert(src_.type() == ImageType_);
    MatrixType::resize(src_.rows, src_.cols);
    for (int r = 0; r < src_.rows; ++r) {
      const CVVecType* src_ptr = src_.ptr<CVVecType>(r);
      EigenVecType* dest_ptr   = this->rowPtr(r);
      for (int c = 0; c < src_.cols; ++c, ++dest_ptr, ++src_ptr) {
        const CVVecType& src_item = *src_ptr;
        EigenVecType& dest_item   = *dest_ptr;
        for (int n = 0; n < NumChannels_; ++n) {
          dest_item[n] = src_item[n];
        }
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::matrix2cv(const std::string& image_name_,
                                                            const Matrix_<float>& matrix_,
                                                            const float& rows_,
                                                            const float& cols_) {
    srrg2_core::ImageUInt8 matrix_image;
    matrix2image(matrix_image, matrix_);
    cv::Mat matrix_cv;
    matrix_image.toCv(matrix_cv);
    cv::Mat colored_matrix_cv;
    cv::cvtColor(matrix_cv, colored_matrix_cv, cv::COLOR_GRAY2BGR);
    cv::Mat scaled_matrix_cv;
    static cv::Size cv_image_size(rows_, cols_);
    cv::resize(colored_matrix_cv, scaled_matrix_cv, cv_image_size);
    cv::imshow(image_name_, scaled_matrix_cv);
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::matrixWithPoints2cv(
    const std::string& image_name_,
    const Matrix_<float>& matrix_,
    const StdVectorEigenVector2i& idx_vector_,
    const float& rows_,
    const float& cols_) {
    srrg2_core::ImageUInt8 matrix_image;
    matrix2image(matrix_image, matrix_);
    cv::Mat matrix_cv;
    matrix_image.toCv(matrix_cv);
    cv::Mat colored_matrix_cv;
    cv::cvtColor(matrix_cv, colored_matrix_cv, cv::COLOR_GRAY2BGR);
    // bb take reference to color of each point in vector idx_vector and change its color
    for (auto point : idx_vector_) {
      cv::Vec3b& color = colored_matrix_cv.at<cv::Vec3b>(point.x(), point.y());
      color            = cv::Vec3b(0, 0, 255);
    }
    cv::Mat scaled_matrix_cv;
    static cv::Size cv_image_size(rows_, cols_);
    cv::resize(colored_matrix_cv, scaled_matrix_cv, cv_image_size);
    cv::imshow(image_name_, scaled_matrix_cv);
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::matrix2image(BaseImage& dest_,
                                                               const Matrix_<float>& matrix_) {
    if (dest_.type() == ImageType::TYPE_8UC1) {
      ImageUInt8& dest = dynamic_cast<ImageUInt8&>(dest_);
      // bb find the maximum value in the matrix
      size_t rows = matrix_.rows();
      size_t cols = matrix_.cols();
      dest.resize(rows, cols);
      std::vector<float> data;
      for (size_t row = 0; row < rows; ++row) {
        for (size_t col = 0; col < cols; ++col) {
          data.emplace_back(matrix_(row, col));
        }
      }
      float md = *std::max_element(data.begin(), data.end()) /* / 127*/;
      float id = 255. / md;
      for (size_t i = 0; i < data.size(); ++i) {
        dest.data()[i] = std::max(255 - id * data[i], 0.f);
      }
      return;
    }
    throw std::runtime_error("Unsupported image type!!!!!!!!");
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::fromBuffer(const uint8_t* buffer) {
    const Scalar* src_ptr = reinterpret_cast<const Scalar*>(buffer);
    for (size_t r = 0; r < this->rows(); ++r) {
      EigenVecType* dest_ptr = this->rowPtr(r);
      for (size_t c = 0; c < this->cols(); ++c, ++dest_ptr) {
        EigenVecType& dest_item = *dest_ptr;
        for (int n = 0; n < NumChannels_; ++n, ++src_ptr) {
          dest_item[n] = *src_ptr;
        }
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  template <typename DestImageType_, typename ScaleType_>
  void Image_<Scalar_, ImageType_, NumChannels_>::convertTo(DestImageType_& dest,
                                                            const ScaleType_& s) const {
    assert(dest.numChannels() == numChannels());
    typedef typename DestImageType_::EigenVecType DestVecType;

    dest.resize(rows(), cols());
    for (size_t r = 0; r < dest.rows(); ++r) {
      const EigenVecType* src_ptr = this->rowPtr(r);
      DestVecType* dest_ptr       = dest.rowPtr(r);
      for (size_t c = 0; c < cols(); ++c, ++src_ptr, ++dest_ptr) {
        const EigenVecType& src_item = *src_ptr;
        DestVecType& dest_item       = *dest_ptr;
        for (int n = 0; n < NumChannels_; ++n) {
          dest_item[n] = src_item[n] * s;
        }
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::scale(float scale_factor_) {
    for (size_t r = 0; r < this->rows(); ++r) {
      for (size_t c = 0; c < this->cols(); ++c) {
        (*this)(r, c) *= scale_factor_;
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::saturate(CellType saturate_val_) {
    for (size_t r = 0; r < this->rows(); ++r) {
      for (size_t c = 0; c < this->cols(); ++c) {
        if(Traits::compareMax((*this)(r, c), saturate_val_))
          (*this)(r, c) = saturate_val_;
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::downsample(
    Image_<Scalar, ImageType_, NumChannels_>& dest,
    DOWNSAMPLE_PIXELS npixels) {
    assert(this->type() == ImageType_);

    const float base =
      sqrt(static_cast<float>(npixels)); // TODO use map between DOWNSAMPLE_PIXELS and its base?
    const float inv_base    = 1 / base;
    const float inv_npixels = 1 / (float) npixels;

    int new_rows = this->rows() * inv_base;
    int new_cols = this->cols() * inv_base;

    if (this->rows() % new_rows) {
      throw std::runtime_error(
        "Image::downsample: the rows of dest should divide perfectly the rows of this");
    }
    if (this->cols() % new_cols) {
      throw std::runtime_error(
        "Image::downsample: the cols of dest should divide perfectly the cols of this");
    }

    dest.resize(new_rows, new_cols);
    Eigen::Matrix<double, NumChannels_, 1> tmp;
    int r_idx_start, r_idx_end, c_idx_start, c_idx_end;
    for (int r = 0; r < new_rows; ++r) {
      r_idx_start = r * base;
      r_idx_end   = r_idx_start + base;
      for (int c = 0; c < new_cols; ++c) {
        c_idx_start = c * base;
        c_idx_end   = c_idx_start + base;
        tmp.setZero();
        for (int rr = r_idx_start; rr < r_idx_end; ++rr) {
          for (int cc = c_idx_start; cc < c_idx_end; ++cc) {
            tmp += (*this)(rr, cc).template cast<double>();
          }
        }
        dest(r, c) = (tmp * inv_npixels).template cast<Scalar>();
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::downsample(
    Image_<Scalar, ImageType_, NumChannels_>& dest,
    int row_scale, int col_scale) {
    
    assert(this->type() == ImageType_);

    int d_rows = this->rows() / row_scale;
    int d_cols = this->cols() / col_scale;

    dest.resize(d_rows, d_cols);
    dest.fill(0);

    ImageInt counts(d_rows, d_cols);
    counts.fill(0);

    for (int r = 0; r < this->rows(); ++r) {
      int dr = r / row_scale;
      for (int c = 0; c < this->cols(); ++c) {
        int dc  = c / col_scale;
        dest.at(dr, dc) += this->at(r, c);
        ++counts.at(dr, dc);
      }
    }
    for (int r = 0; r < d_rows; ++r) {
      for (int c = 0; c < d_cols; ++c) {
        int cnt = counts.at(r, c);
        if (cnt)
          dest.at(r, c) *= (1. / cnt);
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::integralImage(
    Image_<Scalar, ImageType_, NumChannels_>& dest_) const {
    const int rows = this->rows();
    const int cols = this->cols();
    dest_.resize(rows, cols);
    Eigen::Matrix<Scalar, NumChannels_, 1> A, F, E;
    A.setZero();
    F.setZero();
    E.setZero();
    for (int r = 0; r < rows; ++r) {
      const Eigen::Matrix<Scalar, NumChannels_, 1>* D = this->rowPtr(r);
      Eigen::Matrix<Scalar, NumChannels_, 1>* I       = dest_.rowPtr(r);
      for (int c = 0; c < cols; ++c, ++I, ++D) {
        if (r > 1) {
          E = dest_.at(r - 1, c);
        } else {
          E.setZero();
        }

        if (r > 1 && c > 1)
          A = dest_.at(r - 1, c - 1);
        else
          A.setZero();

        if (c > 1)
          F = dest_.at(r, c - 1);
        else
          F.setZero();

        *I = *D + E + F - A;
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void
  Image_<Scalar_, ImageType_, NumChannels_>::blur(Image_<Scalar, ImageType_, NumChannels_>& dest_,
                                                  const size_t& window_) {
    const int rows = this->rows();
    const int cols = this->cols();
    dest_.resize(rows, cols);
    dest_.fill(Eigen::Matrix<Scalar, NumChannels_, 1>::Zero());
    Image_<Scalar_, ImageType_, NumChannels_> integral;

    this->integralImage(integral);
    for (unsigned int r = window_; r < rows - window_; ++r) {
      const Eigen::Matrix<Scalar, NumChannels_, 1>* up_row_ptr =
        integral.rowPtr(r - window_) + window_;
      const Eigen::Matrix<Scalar, NumChannels_, 1>* down_row_ptr =
        integral.rowPtr(r + window_) + window_;
      Eigen::Matrix<Scalar, NumChannels_, 1>* dest_row_ptr = dest_.rowPtr(r) + window_;

      for (unsigned int c = window_; c < cols - window_;
           ++c, ++down_row_ptr, ++up_row_ptr, ++dest_row_ptr) {
        Eigen::Matrix<Scalar, NumChannels_, 1> m11   = *(down_row_ptr + window_);
        Eigen::Matrix<Scalar, NumChannels_, 1> m00   = *(up_row_ptr - window_);
        Eigen::Matrix<Scalar, NumChannels_, 1> m01   = *(down_row_ptr - window_);
        Eigen::Matrix<Scalar, NumChannels_, 1> m10   = *(up_row_ptr + window_);
        Eigen::Matrix<Scalar, NumChannels_, 1> n_sum = m11 + m00 - m01 - m10;
        if (n_sum.dot(n_sum) > 0.2)
          *dest_row_ptr = n_sum.normalized();
        else
          *dest_row_ptr = Eigen::Matrix<Scalar, NumChannels_, 1>::Zero();
      }
    }
  }

  template <class Scalar_, ImageType ImageType_>
  void Image_<Scalar_, ImageType_, 1>::toCv(cv::Mat& dest_) const {
    dest_.create(this->rows(), this->cols(), ImageType_);
    for (size_t r = 0; r < this->rows(); ++r) {
      Scalar* dest_ptr      = dest_.ptr<Scalar>(r);
      const Scalar* src_ptr = this->rowPtr(r);
      memcpy(dest_ptr, src_ptr, sizeof(Scalar) * this->cols());
    }
  }

  template <class Scalar_, ImageType ImageType_>
  void Image_<Scalar_, ImageType_, 1>::fromCv(const cv::Mat& src_) {
    assert(src_.type() == ImageType_);
    MatrixType::resize(src_.rows, src_.cols);
    for (size_t r = 0; r < this->rows(); ++r) {
      const Scalar* src_ptr = src_.ptr<Scalar>(r);
      Scalar* dest_ptr      = this->rowPtr(r);
      memcpy(dest_ptr, src_ptr, sizeof(Scalar) * this->cols());
    }
  }

  // bb partial template specialization
  template <class Scalar_, ImageType ImageType_>
  void Image_<Scalar_, ImageType_, 1>::matrix2cv(const std::string& image_name_,
                                                 const Matrix_<float>& matrix_,
                                                 const float& rows_,
                                                 const float& cols_) {
    srrg2_core::ImageUInt8 matrix_image;
    matrix2image(matrix_image, matrix_);
    cv::Mat matrix_cv;
    matrix_image.toCv(matrix_cv);
    cv::Mat colored_matrix_cv;
    cv::cvtColor(matrix_cv, colored_matrix_cv, cv::COLOR_GRAY2BGR);
    cv::Mat scaled_matrix_cv;
    static cv::Size cv_image_size(rows_, cols_);
    cv::resize(colored_matrix_cv, scaled_matrix_cv, cv_image_size);
    cv::imshow(image_name_, scaled_matrix_cv);
    cv::waitKey(10);
  }

  template <class Scalar_, ImageType ImageType_>
  void
  Image_<Scalar_, ImageType_, 1>::matrixWithPoints2cv(const std::string& image_name_,
                                                      const Matrix_<float>& matrix_,
                                                      const StdVectorEigenVector2i& idx_vector_,
                                                      const float& rows_,
                                                      const float& cols_) {
    using cv::Mat;
    using cv::Point;
    using cv::Scalar;
    using cv::Size;
    srrg2_core::ImageUInt8 matrix_image;
    matrix2image(matrix_image, matrix_);
    Mat matrix_cv;
    matrix_image.toCv(matrix_cv);
    Mat colored_matrix_cv;
    cv::cvtColor(matrix_cv, colored_matrix_cv, cv::COLOR_GRAY2BGR);
    // bb take reference to color of each point in vector idx_vector and change its color
    for (auto point : idx_vector_) {
      //      cv::Vec3b& color = colored_matrix_cv.at<cv::Vec3b>(point.x(), point.y());
      //      color            = cv::Vec3b(0, 0, 255);
      // bb plot a circle on each point of idx_vector_
      cv::circle(colored_matrix_cv,
                 Point(point.x(), point.y()),
                 8,
                 Scalar(0, 0, 255),
                 cv::FILLED,
                 cv::LINE_8);
    }
    Mat scaled_matrix_cv;
    static Size cv_image_size(rows_, cols_);
    cv::resize(colored_matrix_cv, scaled_matrix_cv, cv_image_size);
    cv::imshow(image_name_, scaled_matrix_cv);
    cv::waitKey(10);
  }

  template <class Scalar_, ImageType ImageType_>
  void Image_<Scalar_, ImageType_, 1>::matrix2image(BaseImage& dest_,
                                                    const Matrix_<float>& matrix_) {
    if (dest_.type() == ImageType::TYPE_8UC1) {
      ImageUInt8& dest = dynamic_cast<ImageUInt8&>(dest_);
      // bb find the maximum value in the matrix
      size_t rows = matrix_.rows();
      size_t cols = matrix_.cols();
      dest.resize(rows, cols);
      std::vector<float> data;
      for (size_t row = 0; row < rows; ++row) {
        for (size_t col = 0; col < cols; ++col) {
          data.emplace_back(matrix_(row, col));
        }
      }
      float md = *std::max_element(data.begin(), data.end()) /* / 127*/;
      float id = 255. / md;
      for (size_t i = 0; i < data.size(); ++i) {
        dest.data()[i] = std::max(255 - id * data[i], 0.f);
      }
      return;
    }
    throw std::runtime_error("Unsupported image type!!!!!!!!");
  }

  template <class Scalar_, ImageType ImageType_>
  void Image_<Scalar_, ImageType_, 1>::fromBuffer(const uint8_t* buffer) {
    const Scalar* src_ptr = reinterpret_cast<const Scalar*>(buffer);
    for (size_t r = 0; r < this->rows(); ++r, src_ptr += cols()) {
      Scalar* dest_ptr = this->rowPtr(r);
      memcpy(dest_ptr, src_ptr, sizeof(Scalar) * this->cols());
    }
  }

  template <class Scalar_, ImageType ImageType_>
  template <typename DestImageType_, typename ScaleType_>
  void Image_<Scalar_, ImageType_, 1>::convertTo(DestImageType_& dest, const ScaleType_& s) const {
    assert(dest.numChannels() == numChannels());
    typedef typename DestImageType_::Scalar DestScalar;
    dest.resize(rows(), cols());
    for (size_t r = 0; r < dest.rows(); ++r) {
      const Scalar* src_ptr = this->rowPtr(r);
      DestScalar* dest_ptr  = dest.rowPtr(r);
      for (size_t c = 0; c < cols(); ++c, ++src_ptr, ++dest_ptr) {
        *dest_ptr = (DestScalar)(*src_ptr * s);
      }
    }
  }

  template <class Scalar_, ImageType ImageType_>
  void Image_<Scalar_, ImageType_, 1>::scale(float scale_factor_) {
    for (size_t r = 0; r < this->rows(); ++r) {
      for (size_t c = 0; c < this->cols(); ++c) {
        (*this)(r, c) *= scale_factor_;
      }
    }
  }

  template <class Scalar_, ImageType ImageType_>
  void Image_<Scalar_, ImageType_, 1>::saturate(CellType saturate_val_) {
    for (size_t r = 0; r < this->rows(); ++r) {
      for (size_t c = 0; c < this->cols(); ++c) {
        if(Traits::compareMax((*this)(r, c), saturate_val_))
          (*this)(r, c) = saturate_val_;
      }
    }
  }

  template <class Scalar_, ImageType ImageType_>
  void Image_<Scalar_, ImageType_, 1>::downsample(Image_<Scalar, ImageType_, 1>& dest,
                                                  DOWNSAMPLE_PIXELS npixels) {
    const float base =
      sqrt(static_cast<float>(npixels)); // TODO use map between DOWNSAMPLE_PIXELS and its base?
    const float inv_base    = 1 / base;
    const float inv_npixels = 1 / (float) npixels;

    int new_rows = this->rows() * inv_base;
    int new_cols = this->cols() * inv_base;

    if (this->rows() % new_rows) {
      throw std::runtime_error(
        "Image::downsample: the rows of dest should divide perfectly the rows of this");
    }
    if (this->cols() % new_cols) {
      throw std::runtime_error(
        "Image::downsample: the cols of dest should divide perfectly the cols of this");
    }

    dest.resize(new_rows, new_cols);
    double tmp = 0;
    int r_idx_start, r_idx_end, c_idx_start, c_idx_end;
    for (int r = 0; r < new_rows; ++r) {
      r_idx_start = r * base;
      r_idx_end   = r_idx_start + base;
      for (int c = 0; c < new_cols; ++c) {
        c_idx_start = c * base;
        c_idx_end   = c_idx_start + base;
        tmp         = 0;
        for (int rr = r_idx_start; rr < r_idx_end; ++rr) {
          for (int cc = c_idx_start; cc < c_idx_end; ++cc) {
            tmp += (*this)(rr, cc);
          }
        }

        dest(r, c) = tmp * inv_npixels;
      }
    }
  }

  template <class Scalar_, ImageType ImageType_>
  void Image_<Scalar_, ImageType_, 1>::downsample(
    Image_<Scalar, ImageType_, 1>& dest,
    int row_scale, int col_scale) {
    
    assert(this->type() == ImageType_);

    int d_rows = this->rows() / row_scale;
    int d_cols = this->cols() / col_scale;

    dest.resize(d_rows, d_cols);
    dest.fill(0);

    ImageInt counts(d_rows, d_cols);
    counts.fill(0);

    for (size_t r = 0; r < this->rows(); ++r) {
      int dr = r / row_scale;
      for (size_t c = 0; c < this->cols(); ++c) {
        int dc  = c / col_scale;
        dest.at(dr, dc) += this->at(r, c);
        ++counts.at(dr, dc);
      }
    }
    for (int r = 0; r < d_rows; ++r) {
      for (int c = 0; c < d_cols; ++c) {
        int cnt = counts.at(r, c);
        if (cnt)
          dest.at(r, c) *= (1. / cnt);
      }
    }
  }


  template <class Scalar_, ImageType ImageType_>
  Image_<uint8_t, TYPE_8UC1, 1> Image_<Scalar_, ImageType_, 1>::
  operator==(const size_t& value_) const {
    using MaskImage = Image_<uint8_t, TYPE_8UC1, 1>;
    MaskImage mask(this->rows(), this->cols());

    for (size_t r = 0; r < this->rows(); ++r) {
      const Scalar* src_ptr = this->rowPtr(r);
      uint8_t* dest_ptr     = mask.rowPtr(r);
      for (size_t c = 0; c < this->cols(); ++c, ++src_ptr, ++dest_ptr) {
        *dest_ptr = (*src_ptr == value_);
      }
    }
    return mask;
  }

  template <class Scalar_, ImageType ImageType_>
  Image_<uint8_t, TYPE_8UC1, 1> Image_<Scalar_, ImageType_, 1>::
  operator>=(const size_t& value_) const {
    using MaskImage = Image_<uint8_t, TYPE_8UC1, 1>;
    MaskImage mask(this->rows(), this->cols());

    for (size_t r = 0; r < this->rows(); ++r) {
      const Scalar* src_ptr = this->rowPtr(r);
      uint8_t* dest_ptr     = mask.rowPtr(r);
      for (size_t c = 0; c < this->cols(); ++c, ++src_ptr, ++dest_ptr) {
        *dest_ptr = (*src_ptr >= value_);
      }
    }
    return mask;
  }

} // namespace srrg2_core
