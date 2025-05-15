namespace srrg2_core {
  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::Matrix_() {
    clear();
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::Matrix_(std::size_t rows_, std::size_t cols_) {
    _cols = 0;
    resize(rows_, cols_);
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::Matrix_(const Matrix_& other) {
    assign(other);
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>& Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::
  operator=(const Matrix_& other) {
    assign(other);
    return *this;
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  void Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::fill(const CellType_& value) {
    std::fill(_data.begin(), _data.end(), value);
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  void Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::updateEightNeighborOffsets() {
    if (rows() < 2 || cols() < 2) {
      // TODO: che cazz? (cit. dom)
      memset(_eight_neighbors_offsets, 0, 8 * sizeof(int));
      return;
    }
    int k = 0;
    for (int r = -1; r <= 1; ++r) {
      for (int c = -1; c <= 1; ++c) {
        if (!r && !c) {
          continue;
        }
        _eight_neighbors_offsets[k] = _cols * r + c;
        ++k;
      }
    }
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  void Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::resize(std::size_t rows_,
                                                               std::size_t cols_) {
    // if size is ok, do nothing
    if (rows_ == _row_ptrs.size() && _cols == cols_) {
      return;
    }
    std::size_t num_elements = rows_ * cols_;
    if (!num_elements) {
      clear();
    }
    _cols = cols_;
    _row_ptrs.resize(rows_);
    _data.resize(num_elements);
    reindex();
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  void Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::copyRegionTo(Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>& dest,
                                                                     const size_t r_min, const size_t r_max,
                                                                     const size_t c_min, const size_t c_max) const {
    if (! inside(r_min, c_min)
        || ! inside (r_max, c_max))
      throw std::runtime_error(__PRETTY_FUNCTION__);
    dest.resize(r_max-r_min, c_max-c_min);
    for (size_t r=0; r<dest.rows(); ++r) {
      memcpy(&dest.at(r,0), &at(r+r_min, c_min), dest.cols()*sizeof(CellType));
    }
  }
  
  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  void Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::clear() {
    _cols = 0;
    _row_ptrs.clear();
    _data.clear();
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  void Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::reindex() {
    CellType* row_ptr = &_data[0];
    for (std::size_t r = 0; r < rows(); r++) {
      _row_ptrs[r] = row_ptr;
      row_ptr += _cols;
    }
    updateEightNeighborOffsets();
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  void Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::assign(const Matrix_& other) {
    if (!other._data.size()) {
      clear();
      return;
    }
    _cols = other._cols;
    _data = other._data;
    _row_ptrs.resize(other._row_ptrs.size());
    reindex();
  }

  // mc bilinear interpolation
  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  bool Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::getSubPixel(
    CellType_& interpolated_value_,
    const Vector2f& interpolation_point_) const {
    using namespace std;
    if (!inside(interpolation_point_.x(), interpolation_point_.y())) {
      return false;
    }
    int x0 = interpolation_point_.x(); // rows
    int y0 = interpolation_point_.y(); // cols
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    if (!inside(x1, y1)) {
      return false;
    }

    const float dx  = interpolation_point_.x() - (float) x0;
    const float dy  = interpolation_point_.y() - (float) y0;
    const float dx1 = 1.f - dx;
    const float dy1 = 1.f - dy;
    Traits::setZero(interpolated_value_);
    Traits::sumAndScale(interpolated_value_, (*this)(x0, y0), dy1 * dx1);
    Traits::sumAndScale(interpolated_value_, (*this)(x0, y1), dy1 * dx);
    Traits::sumAndScale(interpolated_value_, (*this)(x1, y0), dy * dx1);
    Traits::sumAndScale(interpolated_value_, (*this)(x1, y1), dy * dx);
    Traits::postInterpolate(interpolated_value_);
    return true;
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  CellType_ Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::getMax() const {
    return *std::max_element(_data.begin(), _data.end(), Traits::compareMin);
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  CellType_ Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::getMin() const {
    return *std::min_element(_data.begin(), _data.end(), Traits::compareMax);
  }

  template <typename CellType_,
            typename AllocatorType_,
            typename CellTraits_,
            template <typename C, typename A > class VectorType_>
  Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_> Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_>::block (
    std::size_t row_, 
    std::size_t col_, 
    std::size_t delta_row_, 
    std::size_t delta_col_) const {
      Matrix_<CellType_, AllocatorType_, CellTraits_, VectorType_> block;
      block._cols = delta_col_;
      block._data.resize(delta_row_ * delta_col_);
      int k = 0;
      for (std::size_t row_index = row_; row_index < row_ + delta_row_; ++row_index) {
        for (std::size_t col_index = col_; col_index < col_ + delta_col_; ++col_index) {
          block._data[k] = _row_ptrs[row_index][col_index];
          k++;
        }
      }
      block._row_ptrs.resize(delta_row_);
      block.reindex();
      return block;
    }

} // namespace srrg2_core
