#include "sparse_block_matrix.h"
#include "matrix_block_factory.h"
#include <iostream>

namespace srrg2_solver {
  using namespace std;

  Triplets::Triplets() : nnz(0), rows(0), cols(0) {
  }

  SparseBlockMatrix::SparseBlockMatrix(const std::vector<int>& row_block_dims,
                                       const std::vector<int>& col_block_dims) :
    _row_block_dims(row_block_dims),
    _col_block_dims(col_block_dims),
    _cols(col_block_dims.size()) {
    recomputeOffsets();
  }

  bool SparseBlockMatrix::isStructureEqual(const SparseBlockMatrix& other) const {
    if (other.blockRows() != blockRows()) {
      return false;
    }
    if (other.blockCols() != blockCols()) {
      return false;
    }
    for (int r = 0; r < blockRows(); ++r) {
      if (_row_block_dims[r] != other._row_block_dims[r]) {
        return false;
      }
    }

    for (int c = 0; c < blockCols(); ++c) {
      if (_col_block_dims[c] != other._col_block_dims[c]) {
        return false;
      }
    }
    return true;
  }

  void SparseBlockMatrix::copyValues(SparseBlockMatrix& dest) const {
    assert(isStructureEqual(dest) && "SparseBlockMatrix::copyValues|structure different");
    for (int c = 0; c < blockCols(); ++c) {
      const IntBlockMap& col = _cols[c];

      for (auto it = col.begin(); it != col.end(); ++it) {
        const MatrixBlockBase* src_block = it->second.get();
        MatrixBlockBase* dest_block      = dest.blockAt(it->first, c, true);
        src_block->copyTo(dest_block);
      }
    }
  }

  void SparseBlockMatrix::setZero() {
    for (size_t r = 0; r < _cols.size(); ++r) {
      IntBlockMap& col = _cols[r];
      for (auto it = col.begin(); it != col.end(); ++it) {
        it->second.get()->setZero();
      }
    }
  }

  void SparseBlockMatrix::recomputeOffsets() {
    _row_block_offsets.resize(blockRows() + 1);
    _col_block_offsets.resize(blockCols() + 1);
    int row_acc = 0;
    int r;
    for (r = 0; r < blockRows(); ++r) {
      _row_block_offsets[r] = row_acc;
      row_acc += _row_block_dims[r];
    }
    _row_block_offsets.back() = row_acc;

    int col_acc = 0;
    int c;
    for (c = 0; c < blockCols(); ++c) {
      _col_block_offsets[c] = col_acc;
      col_acc += _col_block_dims[c];
    }
    _col_block_offsets.back() = col_acc;
  }

  void SparseBlockMatrix::blockErase(int block_row, int block_col) {
    assert(block_col>=0 && block_col < blockCols() && "col index out of range");
    assert(block_row>=0 && block_row < blockRows() && "row index out of range");
    IntBlockMap& col = _cols[block_col];
    // MatrixBlockBase* this_block=nullptr;
    auto it = col.find(block_row);
    if (it != col.end()) {
      col.erase(it);
    }
  }

  MatrixBlockBase* SparseBlockMatrix::blockRelease(int block_row, int block_col) {
    assert(block_col>=0 && block_col < blockCols() && "col index out of range");
    assert(block_row>=0 && block_row < blockRows() && "row index out of range");
    IntBlockMap& col = _cols[block_col];
    MatrixBlockBase* this_block = nullptr;
    auto it                     = col.find(block_row);
    if (it != col.end()) {
      this_block = it->second.release();
      col.erase(it);
    }
    return this_block;
  }

  MatrixBlockBase* SparseBlockMatrix::blockAt(int block_row, int block_col, bool create) {
    assert(block_col>=0 && block_col < blockCols() && "col index out of range");
    assert(block_row>=0 && block_row < blockRows() && "row index out of range");
    IntBlockMap& col = _cols[block_col];
    MatrixBlockBase* this_block = nullptr;
    const IntPair& dims         = blockDims(block_row, block_col);
    auto it = col.find(block_row);
    if (it == col.end()) {
      if (create) {
        MatrixBlockFactory* factory = MatrixBlockFactory::instance();
        this_block                  = factory->createBlock(dims.first, dims.second);
        col.insert(std::make_pair(block_row, MatrixBlockBasePtr(this_block)));
      } else {
        this_block = nullptr;
      }
    } else {
      this_block = it->second.get();
    }

    assert(!this_block || (this_block->rows() == dims.first && this_block->cols() == dims.second &&
                           "dimension mismatch, something serious happened"));
    return this_block;
  }

  const MatrixBlockBase* SparseBlockMatrix::blockAt(int block_row, int block_col) const {
    assert(block_col>=0 && block_col < blockCols() && "col index out of range");
    assert(block_row>=0 && block_row < blockRows() && "row index out of range");
    const IntBlockMap& col = _cols[block_col];
    auto it = col.find(block_row);
    if (it == col.end()) {
      return nullptr;
    }
    MatrixBlockBase* this_block = it->second.get();
    assert(!this_block || (this_block->rows() == _row_block_dims[block_row] &&
                           this_block->cols() == _col_block_dims[block_col] &&
                           "dimension mismatch, something serious happened"));
    return this_block;
  }

  int SparseBlockMatrix::numNonZeros(bool is_symmetric_) const {
    int count = 0;
    for (int c = 0; c < blockCols(); ++c) {
      const IntBlockMap& col = _cols[c];
      for (auto it = col.begin(); it != col.end(); ++it) {
        assert(it->second);
        const MatrixBlockBase* b = it->second.get();
        if (is_symmetric_ && it->first == c) {
          count += b->rows() * (b->rows() + 1) / 2;
        } else {
          count += b->rows() * b->cols();
        }
      }
    }
    return count;
  }

  int SparseBlockMatrix::numNonZeroBlocks() const {
    int count = 0;
    for (int c = 0; c < blockRows(); ++c) {
      const IntBlockMap& col = _cols[c];
      count += col.size();
    }
    return count;
  }

  void SparseBlockMatrix::allocateFull() {
    for (int c = 0; c < blockCols(); ++c) {
      for (int r = 0; r < blockRows(); ++r) {
        blockAt(r, c, true);
      }
    }
  }

  void SparseBlockMatrix::printLayout(PrintMode mode, std::ostream& os) const {
    using namespace std;
    for (int r = 0; r < blockRows(); ++r) {
      for (int c = 0; c < blockCols(); ++c) {
        const MatrixBlockBase* block = blockAt(r, c);
        IntPair dims                 = blockDims(r, c);
        char open_bracket            = block ? '[' : '(';
        char closed_bracket          = block ? ']' : ')';

        switch (mode) {
          case PrintFillIn:
            os << (int) (block != 0) << " ";
            break;
          case PrintDim:
            os << open_bracket << dims.first << ',' << dims.second << closed_bracket;
            break;
          case PrintOffset:
            os << open_bracket << dims.first << ',' << dims.second << closed_bracket;
            break;
          case PrintValues:
            os << "block: [" << r << "," << c << "]" << endl;
            if (block) {
              block->print(os);
            }
            break;
          default:;
        }
      }
      os << endl;
    }
  }

  bool SparseBlockMatrix::isLayoutSymmetric() const {
    if (_row_block_dims.size() != _col_block_dims.size()) {
      return false;
    }
    for (size_t i = 0; i < _row_block_dims.size(); ++i) {
      if (_row_block_dims[i] != _col_block_dims[i]) {
        return false;
      }
    }
    return true;
  }

  float SparseBlockMatrix::symmetricProduct(const SparseBlockMatrix& x) const {
    assert(isLayoutSymmetric() && "SparseBlockMatrix::symmetricProduct| matrix is not symmetric");
    assert(x.blockRows() == blockCols() &&
           "SparseBlockMatrix::symmetricProduct| size mismatch between vector and matrix");
    assert(x.blockCols() == 1 &&
           "SparseBlockMatrix::symmetricProduct| size mismatch between vector and matrix");
    // tg extract number of block cols (assumes symmetric layout)
    int num_block_cols_matrix      = blockCols();
    // tg initialize result
    float result                   = 0;
    // tg factory and result block
    MatrixBlockFactory* factory    = MatrixBlockFactory::instance();
    MatrixBlock_<1, 1> sym_product;
    // tg scale for the off-diagonal blocks
    float off_scale = 2.f;
    // tg for each block columns
    for (int c = 0; c < num_block_cols_matrix; ++c) {
      // tg take the block column of the matrix
      const IntBlockMap& col = _cols[c];
      // tg take the corresponding block of the vector
      const MatrixBlockBase* root_block = x.blockAt(c, 0);
      if (!root_block) {
        continue;
      }
      // tg for each element in the matrix column compute x(r)^T * A(r,c) * x(c)
      // with r that goes from 0 to num_rows-1
      // the output of this product is one term of the summation that gives us x^T * A * x
      for (auto it = col.begin(); it != col.end(); ++it) {
        // tg takes the row index (r) and block in the matrix column
        const int row_block_index    = it->first;
        const MatrixBlockBase* matrix_block = it->second.get();
        // tg take the corresponding block in the vector x (aka x(r))
        const MatrixBlockBase* vector_block = x.blockAt(row_block_index, 0);
        // tg transpose the block for the multiplication
        MatrixBlockBasePtr transpose_block(factory->createBlock(1, vector_block->rows()));
        vector_block->transposeTo(transpose_block.get());
        // tg create result of the multiplication (aka x(r)^T * A(r,c)) and compute
        MatrixBlockBasePtr tmp_block(factory->createBlock(1, matrix_block->cols()));
        transpose_block->matrixProduct(tmp_block.get(), matrix_block);
        // tg compute the final product x(r)^T * A(r,c) * x(c)
        tmp_block->matrixProduct(&sym_product, root_block);
        // tg handle off-diagonal blocks, in fact we represent a symmetric matrix by its upper
        // triangular structure
        if (row_block_index != c) {
          sym_product.scale(off_scale);
        }
        // tg accumulate the result
        result += sym_product._matrix(0, 0);
      }
    }
    return result;
  }

  //! returns the coefficients along the diagonal
  void SparseBlockMatrix::getDiagonal(std::vector<float>& diagonal) const {
    assert(isLayoutSymmetric() && "layout not symmetric");
    diagonal.resize(rows());
    memset(diagonal.data(), 0, diagonal.size() * sizeof(float));
    int k = 0;
    for (int ib = 0; ib < blockRows(); ++ib) {
      const MatrixBlockBase* b = blockAt(ib, ib);
      if (!b) {
        k += _col_block_dims[ib];
        continue;
      }
      for (int ii = 0; ii < b->cols(); ++ii, ++k) {
        diagonal[k] = b->at(ii, ii);
      }
    }
  }

  //! sets the coefficients along the diagonal
  void SparseBlockMatrix::setDiagonal(const std::vector<float>& diagonal) {
    assert(isLayoutSymmetric() && "layout not symmetric");
    assert((int) diagonal.size() == rows() && "wrong dimension for diagonal");
    int k = 0;
    for (int ib = 0; ib < blockRows(); ++ib) {
      MatrixBlockBase* b = blockAt(ib, ib, true);
      for (int ii = 0; ii < b->cols(); ++ii, ++k) {
        b->at(ii, ii) = diagonal[k];
      }
    }
  }

  //! returns the coefficients along the diagonal
  void SparseBlockMatrix::getDenseVector(std::vector<float>& v) const {
    assert(cols() == 1 && "not a vector");
    v.resize(rows());
    memset(v.data(), 0, v.size() * sizeof(float));
    const IntBlockMap& col = _cols[0];
    for (IntBlockMap::const_iterator it = col.begin(); it != col.end(); ++it) {
      const int block_row      = it->first;
      const MatrixBlockBase* b = it->second.get();
      int offset               = _row_block_offsets[block_row];
      for (int r = 0; r < b->rows(); ++r, ++offset) {
        v[offset] = b->at(r, 0);
      }
    }
  }

    //! returns the coefficients along the diagonal
  void SparseBlockMatrix::fromDenseVector(const std::vector<float>& v) {
    assert(cols() == 1 && "not a vector");
    assert((int) v.size() != rows() && "size mismatch");
    const IntBlockMap& col = _cols[0];
    for (IntBlockMap::const_iterator it = col.begin(); it != col.end(); ++it) {
      const int block_row      = it->first;
      MatrixBlockBase* b = it->second.get();
      int offset               = _row_block_offsets[block_row];
      for (int r = 0; r < b->rows(); ++r, ++offset) {
        b->at(r, 0) = v[offset];
      }
    }
  }

  void SparseBlockMatrix::toCCS(int* row_indicies_,
                                int* col_pointers_,
                                double* values_,
                                bool is_symmetric_) const {
    assert(row_indicies_ && col_pointers_ && values_ &&
           "SparseBlockMatrix::toStructureCCS|Target destination is NULL");
    int nz = 0;
    for (int bc = 0; bc < blockCols(); ++bc) {
      int col_idx_start  = _col_block_offsets[bc];
      int col_block_size = _col_block_dims[bc];
      for (int c = 0; c < col_block_size; ++c) {
        *col_pointers_                  = nz;
        const IntBlockMap& block_column = _cols[bc];
        for (auto it = block_column.begin(); it != block_column.end(); ++it) {
          const MatrixBlockBase* block = it->second.get();
          int row_idx_start            = _row_block_offsets[it->first];
          int elems_to_copy            = block->rows();
          if (is_symmetric_ && row_idx_start == col_idx_start) {
            // copy only the upper
            elems_to_copy = c + 1;
          }
          for (int r = 0; r < elems_to_copy; ++r, ++row_indicies_, ++row_idx_start, ++values_) {
            *values_       = static_cast<double>(block->at(r, c));
            *row_indicies_ = row_idx_start;
            ++nz;
          }
        }
        ++col_pointers_;
      }
    }
    *col_pointers_ = nz;
  }

  void SparseBlockMatrix::fillCCS(double* values_, bool is_symmetric_) const {
    assert(values_ && "SparseBlockMatrix::fillCCS|Target destination is NULL");
    for (int bc = 0; bc < blockCols(); ++bc) {
      int col_idx_start  = _col_block_offsets[bc];
      int col_block_size = _col_block_dims[bc];
      for (int c = 0; c < col_block_size; ++c) {
        const IntBlockMap& block_column = _cols[bc];
        for (auto it = block_column.begin(); it != block_column.end(); ++it) {
          const MatrixBlockBase* block = it->second.get();
          int row_idx_start            = _row_block_offsets[it->first];
          int elems_to_copy            = block->rows();
          if (is_symmetric_ && row_idx_start == col_idx_start) {
            // copy only the upper
            elems_to_copy = c + 1;
          }
          for (int r = 0; r < elems_to_copy; ++r, ++values_) {
            *values_ = static_cast<double>(block->at(r, c));
          }
        }
      }
    }
  }


    void SparseBlockMatrix::fillEigenSparse(Eigen::SparseMatrix<double>& target, bool is_symmetric_) const {
    target=Eigen::SparseMatrix<double>(rows(), cols());
    for (int bc = 0; bc < blockCols(); ++bc) {
      const auto& block_column=_cols[bc];
      int col_idx_start  = _col_block_offsets[bc];
      int col_block_size = _col_block_dims[bc];
      for (auto it = block_column.begin(); it != block_column.end(); ++it) {
        const MatrixBlockBase* block = it->second.get();
        int rc                       = it->first;
        int row_idx_start            = _row_block_offsets[rc];
        int row_block_size           = block->rows();
        if (is_symmetric_) {
          if (row_idx_start == col_idx_start) {
            for (int c=0; c<col_block_size; ++c) {
              for (int r=0; r<=c; ++r) {
                target.insert(row_idx_start+r, col_idx_start+c)=block->at(r, c);
                if (c!=r)
                  target.insert(col_idx_start+c, row_idx_start+r)=block->at(r, c);
              }
            }
          } else {
            for (int c=0; c<col_block_size; ++c) {
              for (int r=0; r<row_block_size; ++r) {
                  target.insert(row_idx_start+r, col_idx_start+c)=block->at(r, c);
                  target.insert(col_idx_start+c, row_idx_start+r)=block->at(r, c);
                }
            }
          }
        } else { // not symmetric
          for (int c=0; c<col_block_size; ++c) {
            for (int r=0; r<row_block_size; ++r) {
              target.insert(row_idx_start+r, col_idx_start+c)=block->at(r, c);
            }
          }
        }
      }
    }
  }
} // namespace srrg2_solver
