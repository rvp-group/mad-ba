#pragma once
#include "srrg_geometry/geometry_defs.h"
//#include "srrg_image/image.h"
#include <vector>
namespace srrg2_core {
  /**
     Implements a simplistic 2D array class, that supports assign(not a physical copy) and resize
     Order is row major. An array of row pointers is dynamically constructed
     for fast scattered access
     @param CellType_ the type of the cell
     @param AllocatorType the allocator
  */

  template <typename CellType_>
  struct DefaultCellTraits {
    inline static void setZero(CellType_& dest) {
      dest *= 0;
    }

    inline static void sumAndScale(CellType_& dest, const CellType_& src, float scale) {
      dest += src * scale;
    }

    inline static void postInterpolate(CellType_& dest __attribute__((unused))) {
    }

    inline static bool compareMin(const CellType_& first, const CellType_& second) {
      return first < second;
    }

    inline static bool compareMax(const CellType_& first, const CellType_& second) {
      return first > second;
    }
  };

template <typename CellType_,
          typename AllocatorType_=std::allocator<CellType_>,
          typename CellTraits_=DefaultCellTraits<CellType_>,
          template <typename, typename > class VectorType_=std::vector>  class Matrix_ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using CellType       = CellType_;
    using value_type     = CellType_;
    using AllocatorType  = AllocatorType_;
    using Traits         = CellTraits_;
    using VectorType     = VectorType_<CellType, AllocatorType_>;
    using iterator       = typename VectorType::iterator;
    using const_iterator = typename VectorType::const_iterator;
    using ThisType       = Matrix_<CellType, AllocatorType, Traits, VectorType_>;
    //! constructs a matrix of this type
    //! the matrix might not have a buffer
    Matrix_();

    Matrix_(std::size_t rows_, std::size_t cols_);

    Matrix_(const Matrix_& other);

    ThisType& operator=(const Matrix_& other);

    void copyRegionTo(ThisType& dest,
                      const size_t r_min, const size_t r_max,
                      const size_t c_min, const size_t c_max) const;

    inline iterator begin() {
      return _data.begin();
    }
    inline const_iterator begin() const {
      return _data.begin();
    }
    inline iterator end() {
      return _data.end();
    }
    inline const_iterator end() const {
      return _data.end();
    }
    inline size_t size() const {
      return _data.size();
    }

    inline std::size_t rows() const {
      return _row_ptrs.size();
    }

    inline std::size_t cols() const {
      return _cols;
    }

    inline std::size_t capacity() const {
      return _data.capacity();
    }

    inline void reserve(int num_elements) {
      _data.reserve(num_elements);
    }

    
    inline Eigen::Vector2i pos(const CellType* c) const {
      std::ptrdiff_t index = c - &_data[0];
      assert(index >= 0 && index < (int) _data.size() && "outside ptr");
      div_t divresult = div(static_cast<int>(index), static_cast<int>(cols()));
      return Eigen::Vector2i(divresult.quot, divresult.rem);
    }

    inline const CellType* rowPtr(int row) const {
      return _row_ptrs[row];
    }

    inline CellType* rowPtr(int row) {
      return _row_ptrs[row];
    }

    inline const std::vector<CellType, AllocatorType>& data() const {
      return _data;
    }

    inline std::vector<CellType, AllocatorType>& data() {
      return _data;
    }

    inline bool onBorder(std::size_t row, std::size_t col) const {
      return row == 0 || col == 0 || row == rows() - 1 || col == _cols - 1;
    }

    inline bool inside(std::size_t row, std::size_t col) const {
      return row >= 0 && col >= 0 && row < rows() && col < _cols;
    }

    inline bool onBorder(const Eigen::Vector2i& pos_) const {
      return onBorder(pos_(0), pos_(1));
    }

    inline bool inside(const Eigen::Vector2i& pos_) const {
      return inside(pos_(0), pos_(1));
    }

    inline CellType& at(std::size_t row, std::size_t col) {
      if (col >= _cols) {
        throw std::out_of_range("[Matrix::at] columns index out of range");
      }
      return _row_ptrs.at(row)[col];
    }

    inline const CellType& at(std::size_t row, std::size_t col) const {
      if (col >= _cols) {
        throw std::out_of_range("[const Matrix::at] columns index out of range");
      }
      return _row_ptrs.at(row)[col];
    }

    // tg determine the index in _data which correspond to [row,col]
    // considering a row major structure
    inline size_t indexAt(const std::size_t& row, const std::size_t& col) const {
      if (col >= _cols) {
        throw std::out_of_range("[Matrix::indexAt] columns index out of range");
      }
      if (row >= _row_ptrs.size()) {
        throw std::out_of_range("[Matrix::indexAt] rows index out of range");
      }
      return row * _cols + col;
    }

    // tg const and non-const accessor to the underlying vector structure (_data)
    inline CellType& operator[](const std::size_t& index) {
      if (index >= _data.size()) {
        throw std::out_of_range("[Matrix::[]] index out of range for underlying vector");
      }
      return _data[index];
    }

    inline const CellType& operator[](const std::size_t& index) const {
      if (index >= _data.size()) {
        throw std::out_of_range("[Matrix::[]] index out of range for underlying vector");
      }
      return _data[index];
    }

    inline CellType& at(const std::size_t& index) {
      if (index >= _data.size()) {
        throw std::out_of_range("[Matrix::at] index out of range for underlying vector");
      }
      return _data[index];
    }

    inline const CellType& at(const std::size_t& index) const {
      if (index >= _data.size()) {
        throw std::out_of_range("[Matrix::at] index out of range for underlying vector");
      }
      return _data[index];
    }
    // tg const and non-const accessor to matrix cells
    inline CellType& at(const Eigen::Vector2i& pos_) {
      return at(pos_(0), pos_(1));
    }

    inline const CellType& at(const Eigen::Vector2i& pos_) const {
      return at(pos_(0), pos_(1));
    }

    inline const CellType& operator()(std::size_t row, std::size_t col) const {
      return _row_ptrs[row][col];
    }

    inline CellType& operator()(std::size_t row, std::size_t col) {
      return _row_ptrs[row][col];
    }

    inline const CellType& operator()(const Eigen::Vector2i& pos_) const {
      return _row_ptrs[pos_(0)][pos_(1)];
    }

    inline CellType& operator()(const Eigen::Vector2i& pos_) {
      return _row_ptrs[pos_(0)][pos_(1)];
    }

    inline const int* eightNeighborOffsets() const {
      return _eight_neighbors_offsets;
    }

    void resize(std::size_t rows_, std::size_t cols_);

    void clear();

    void fill(const CellType& value);

    bool getSubPixel(CellType& interpolated_value_, const Vector2f& interpolation_point_) const;

    CellType getMax() const;

    CellType getMin() const;

    ThisType block(std::size_t row_, std::size_t col_, std::size_t delta_row_, std::size_t delta_col_) const;

  template <typename DestCellType_,
            typename DestAllocatorType_,
            typename DestCellTraits_,
            template <typename, typename > class DestVectorType_>
  void castTo(Matrix_<DestCellType_,  DestAllocatorType_, DestCellTraits_, DestVectorType_ >& dest) {
      dest.resize(rows(), cols());
      for (size_t i = 0; i < dest._data.size(); ++i) {
        dest._data[i] = static_cast<DestCellType_>(_data[i]);
      }
    }

  protected:
    void reindex();
    void assign(const Matrix_& other);
    void updateEightNeighborOffsets();

    std::size_t _cols;
    std::vector<CellType*> _row_ptrs;
    VectorType _data;

    int _eight_neighbors_offsets[8];
  };

  typedef Matrix_<int> MatrixInt;
  typedef Matrix_<float> MatrixFloat;
  typedef Matrix_<double> MatrixDouble;

} // namespace srrg2_core

#include "matrix.hpp"
