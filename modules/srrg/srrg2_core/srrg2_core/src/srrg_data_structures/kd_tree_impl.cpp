#include "kd_tree.h"
#include <Eigen/Eigenvalues>

#ifdef __SRRG_PARALLEL_KDTREE__
#include <omp.h>
#endif

namespace srrg2_core {


  template <class T, size_t D>
  KDTree_<T, D>::TreeNode::TreeNode(KDTree_<T, D>* tree_, int node_num) :
    _tree(tree_),
    _node_num(node_num),
    _node_type(KDTreeNodeType::Leaf) {
    _mean.setZero();
    _normal.setZero();
    _left_child  = 0;
    _right_child = 0;
    _min_index   = -1;
    _max_index   = -1;
    _num_points  = 0;
  }

  template <class T, size_t D>
  KDTree_<T, D>::TreeNode::TreeNode(KDTree_<T, D>* tree_,
                       int node_num,
                       const VectorTD& mean_,
                       const VectorTD& normal_,
                       TreeNode* left_child,
                       TreeNode* right_child) :
    _tree(tree_),
    _node_num(node_num),
    _node_type(KDTreeNodeType::Middle) {
    _min_index = _max_index = -1;
    assert(normal_.rows() == D);
    assert(mean_.rows() == D);
    _normal      = normal_;
    _mean        = mean_;
    _num_points  = 0;
    _left_child  = left_child;
    _right_child = right_child;
    if (_left_child) {
      _num_points += _left_child->_num_points;
    }
    if (_right_child) {
      _num_points += _right_child->_num_points;
    }
  }

  //! dtor
  template <class T, size_t D>
  KDTree_<T, D>::TreeNode::~TreeNode() {
    if (_left_child) {
      delete _left_child;
      _left_child = 0;
    }
    if (_right_child) {
      delete _right_child;
      _right_child = 0;
    }
  }
  
  template <class T, size_t D>
  void
  KDTree_<T, D>::TreeNode::
  findNeighbors(VectorTDVector& answers,
                std::vector<int>& indices,
                const VectorTD& query,
                const T maximum_distance,
                KDTreeSearchType search_type) const {
    assert(maximum_distance >= 0);
    const T maximum_squared_distance=pow(maximum_distance,2);
    switch (_node_type) {
    case KDTreeNodeType::Leaf: {
      // ds return all points in this leaf that satisfy the distance
      assert(_max_index >= _min_index);
      const size_t number_of_points = _max_index - _min_index;
      size_t s=answers.size();
      answers.reserve(s+number_of_points);
      indices.reserve(s+number_of_points);
      int k=0;
      for (size_t i = _min_index; i < _max_index; ++i) {
        assert(i < _tree->_points.size());
        const T squared_distance = (_tree->_points[i] - query).squaredNorm();
        if (squared_distance < maximum_squared_distance) {
          answers.emplace_back(_tree->_points[i]);
          indices.emplace_back(_tree->_indices[i]);
          ++k;
        }
      }
      //std::cerr << k << "/" << number_of_points << "/" << indices.size() << " " ;
      return;
    }
      

    case KDTreeNodeType::Middle: {
      T plane_distance = planeDistance(query);
      //std:: cerr << " - {" << plane_distance << "} ";
      if (search_type==Complete) {
        //std::cerr <<"2(";
        //std::cerr << "answers_size (before): " << answers.size() ;
        //std::cerr << "l(";
        _left_child->findNeighbors(answers, indices, query, maximum_distance, search_type);
        //std::cerr << "answers_size (left): " << answers.size() << std::endl;
        //std::cerr << ")r(";
        _right_child->findNeighbors(answers, indices, query, maximum_distance, search_type);
        //std::cerr << "answers_size (right): " << answers.size() << std::endl;
        //std::cerr <<"))";
      } else if (plane_distance < 0) {
        //std::cerr << "l(";
        _left_child->findNeighbors(answers, indices, query, maximum_distance, search_type);
        //std::cerr << ")";
      } else {
        //std::cerr << "r(";
        _right_child->findNeighbors(answers, indices, query, maximum_distance, search_type);
        //std::cerr << ")";
      }
      return;
    }
    default:
      std::cerr << "KDTree::findNeighbors|ERROR, sanity check (" << _tree
                << ") : " << _tree->sanityCheck() << std::endl;
      throw std::runtime_error("KDTree::findNeighbors|ERROR, unknown node type");
    }
  }

  template <class T, size_t D>
  KDTree_<T, D>::KDTree_(const VectorTDVector& points_, T max_leaf_range, size_t min_leaf_points) {
      good_queries = 0;
      _num_nodes   = 0;
      _points      = points_;
      _aux_points  = points_;
      _indices.resize(points_.size());
      _aux_indices.resize(points_.size());
      _min_leaf_points = min_leaf_points;
      for (size_t i = 0; i < _indices.size(); ++i) {
        _indices[i] = i;
      }
      _root = _buildTree(0, points_.size(), max_leaf_range, 0);
    }

    //! dtor
  template <class T, size_t D>
  KDTree_<T, D>::~KDTree_() {
      if (_root) {
        delete _root;
      }
      _root = 0;
    }

  template <class T, size_t D>
  bool KDTree_<T, D>::sanityCheck() const {
    std::vector<int> checked_indices(_points.size());
    std::fill(checked_indices.begin(), checked_indices.end(), -1);
    const size_t k = sanityCheck(checked_indices, 0, _root);
    if (k != _points.size()) {
      throw std::runtime_error("KDTree::sanityCheck(void)|ERROR, illegal size reported");
    }
    std::sort(checked_indices.begin(), checked_indices.end(), std::less<int>());
    for (size_t i = 0; i < checked_indices.size(); i++) {
      if (i != static_cast<size_t>(checked_indices[i])) {
        throw std::runtime_error("KDTree::sanityCheck(void)|ERROR, missing indices");
        }
    }
    return true;
  }

  template <class T, size_t D>
  int KDTree_<T, D>::sanityCheck(std::vector<int>& checked_indices, int k, const TreeNode* node) const {
    if (!node) {
      return k;
    }
    switch (node->_node_type) {
    case KDTreeNodeType::Leaf:
      for (size_t i = node->_min_index; i < node->_max_index; i++) {
        int idx = _indices[i];
        if (checked_indices[k] != -1) {
          throw std::runtime_error("KDTree::sanityCheck|ERROR, writing on an occupied index");
        }
        checked_indices[k] = idx;
        k++;
      }
      return k;
    case KDTreeNodeType::Middle:
      k = sanityCheck(checked_indices, k, node->_left_child);
      k = sanityCheck(checked_indices, k, node->_right_child);
      return k;
    default:
      throw std::runtime_error("KDTree::sanityCheck|ERROR, illegal type index");
    }
  }


  template <class T, size_t D>
  void KDTree_<T, D>::findNeighbors(VectorTDVector& answers,
                     std::vector<int>& indices,
                     const KDTree_<T, D>::VectorTD& query,
                     const T max_distance,
                     KDTreeSearchType search_type) const {
    if (!_root) {
      throw std::runtime_error("KDTree::findNeighbors|ERROR, no root node");
    }
    answers.clear();
    indices.clear();
    good_queries++;
    _root->findNeighbors(answers, indices, query, max_distance, search_type);
    //std::cerr << "indices.size: " << indices.size() << std::endl;
  }


  template <class T, size_t D>
  T KDTree_<T, D>::findNeighbor(VectorTD& answer,
                                int& index,
                                const VectorTD& query,
                                const T max_distance,
                                const KDTreeSearchType search_type) const {
    if (!_root) {
      throw std::runtime_error("KDTree::findNeighbors|ERROR, no root node");
    }
    index=-1;
    
    // approx search, we take the quick path unrolling tail recursion
    T max_distance2 = pow(max_distance, 2);
    if (search_type==Approximate) {
      TreeNode* current=_root;
      while(! current->_node_type==Leaf) {
        assert(current->_node_type==Middle && "node is not middle");
        T plane_distance = current->planeDistance(query);
        if(plane_distance<0) {
          current=current->_left_child;
        } else {
          current=current->_right_child;
        }
      }
      assert(current->_node_type==Leaf && "node is leaf");
      assert(current->_max_index >= current->_min_index);
      T best_distance=std::numeric_limits<T>::max();
      for (size_t i = current->_min_index; i < current->_max_index; ++i) {
        const T squared_distance = (_points[i] - query).squaredNorm();
        if (squared_distance < best_distance && squared_distance < max_distance2) {
          best_distance=squared_distance;
          answer=_points[i];
          index=_indices[i];
        }
      }
      return best_distance;
    }

    //exact search, we need a buffer
    VectorTDVector answers;
    std::vector<int> indices;
    _root->findNeighbors(answers, indices, query, max_distance, search_type);
    T best_distance=std::numeric_limits<T>::max();
    for (size_t i=0; i<answers.size(); ++i) {
      T current_distance = (query-answers[i]).squaredNorm();
      if (current_distance<max_distance) {
        answer=answers[i];
        index=indices[i];
      }
    }
    return best_distance;
  }
      
  template <class T, size_t D>
  T KDTree_<T, D>::_splitPoints(KDTree_<T, D>::VectorTD& mean,
                               KDTree_<T, D>::VectorTD& normal,
                               size_t& num_left_points,
                               const size_t min_index,
                               const size_t max_index) {
    // if points empty, nothing to do
    if (min_index == max_index) {
      return 0;
    }

    const size_t num_points            = max_index - min_index;
    const T inverse_num_points         = 1.0 / num_points;
    VectorTD sum                       = VectorTD::Zero();
    Eigen::Matrix<T, D, D> squared_sum = Eigen::Matrix<T, D, D>::Zero();
    Eigen::Matrix<T, D, D> covariance  = Eigen::Matrix<T, D, D>::Zero();
    for (size_t i = min_index; i < max_index; ++i) {
      sum += _points[i];
      squared_sum += _points[i] * _points[i].transpose();
    }
    mean       = sum * inverse_num_points;
    covariance = squared_sum * inverse_num_points - mean * mean.transpose();

    // eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, D, D>> solver;
    solver.compute(covariance, Eigen::ComputeEigenvectors);
    normal = solver.eigenvectors().col(D - 1).normalized();

    // the following var will contain the range of points along the normal
    // vector
    T max_distance_from_plane = 0;

    // run through the points and split them in the left or the right set
    size_t left_index  = min_index;
    size_t right_index = max_index;

    size_t num_left  = 0;
    size_t num_right = 0;
    for (size_t i = min_index; i < max_index; ++i) {
      T distance_from_plane = normal.dot(_points[i] - mean);
      if (fabs(distance_from_plane) > max_distance_from_plane) {
        max_distance_from_plane = fabs(distance_from_plane);
      }

      bool side = distance_from_plane < 0;
      if (side) {
        _aux_points[left_index]  = _points[i];
        _aux_indices[left_index] = _indices[i];
        left_index++;
        num_left++;
      } else {
        right_index--;
        _aux_points[right_index]  = _points[i];
        _aux_indices[right_index] = _indices[i];
        num_right++;
      }
    }
    assert(max_index - min_index == num_right + num_left);
    for (size_t i = min_index; i < max_index; ++i) {
      _points[i]  = _aux_points[i];
      _indices[i] = _aux_indices[i];
    }

    num_left_points = num_left;
    return max_distance_from_plane;
  }

  template <class T, size_t D>
    typename KDTree_<T, D>::TreeNode*
    KDTree_<T, D>::_buildTree(const size_t min_index, const size_t max_index, const T max_leaf_range, int level) {
      const size_t num_points = max_index - min_index;
      if (!num_points) {
        return 0;
      }


      VectorTD mean;
      VectorTD normal;
      size_t num_left_points = 0;

      const T range = _splitPoints(mean, normal, num_left_points, min_index, max_index);
      assert(range >= 0);
      TreeNode* node = 0;
      if (range < max_leaf_range || num_points < _min_leaf_points) {
        node = new TreeNode(this, _num_nodes);
        node->_min_index=min_index;
        node->_max_index=max_index;
        node->_num_points = num_points;
        //std::cerr << "leaf: " << num_points << std::endl;
      } else {
        TreeNode *left_tree, *right_tree;

#ifdef __SRRG_PARALLEL_KDTREE__
        int num_threads = omp_get_max_threads();
        int split_level = -1;
        if (level > 0)
          split_level = floor(log(num_threads) / log(2));

        if (split_level == level) {
#pragma omp parallel sections
          {
#pragma omp section
            {
              left_tree =
                _buildTree(min_index, min_index + num_left_points, max_leaf_range, level + 1);
            }
#pragma omp section
            {
              right_tree =
                _buildTree(min_index + num_left_points, max_index, max_leaf_range, level + 1);
            }
          }
        } else {
#endif //__SRRG_PARALLEL_KDTREE__

          left_tree = _buildTree(min_index, min_index + num_left_points, max_leaf_range, level + 1);
          right_tree =
            _buildTree(min_index + num_left_points, max_index, max_leaf_range, level + 1);
#ifdef __SRRG_PARALLEL_KDTREE__
        }
#endif
        node = new TreeNode(this, _num_nodes, mean, normal, left_tree, right_tree);
      }
      _num_nodes++;
      return node;
    }

  template <class T, size_t D>
  void KDTree_<T, D>::_printKDTree(TreeNode* node) {
    std::cerr << "(";
    switch (node->_node_type) {
    case Leaf:
      std::cerr << node->_max_index - node->_min_index;
      // for (size_t i = node->_min_index; i < node->_max_index; ++i) {
      //   std::cerr << _points[i].transpose() << std::endl;
      // }
      break;
    case Middle:
      _printKDTree(node->_left_child);
      _printKDTree(node->_right_child);
      break;
    }
    std::cerr << ")";

  }

} // namespace srrg2_core
