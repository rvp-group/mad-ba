#pragma once
#include <iostream>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace srrg2_core {

  /**
     KDTree: implements a KDTree, it requires a type T
     and a dimension D
  */
  enum KDTreeNodeType { Leaf = 0x0, Middle = 0x1 };
  enum KDTreeSearchType { Approximate = 0x0, Complete=0x1};

  template <class T, size_t D>
  class KDTree_ {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // typedef for defining a vector variable sized points
    using VectorTD       = Eigen::Matrix<T, D, 1>;
    using VectorTDVector = std::vector<VectorTD, Eigen::aligned_allocator<VectorTD>>;
    /**
       TreeNode class. Represents a base class for a node in the search tree.

       if the type is a Middle node
       it represents a splitting plane, and has 2 child nodes
       that refer to the set of points to the two sides of the splitting plane.
       A splitting plane is parameterized as a point on a plane and as a normal
       to the plane.

       If the type is a leaf
       it represents a bucket containing points in a neighborhood
       To avoid copies, the bucket is kept as a range
       _min_index.._max_indexindices and the points
       in the _points and _indices
    */
    class TreeNode {
    public:
      friend class KDTree_;
      //! ctor
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      TreeNode(KDTree_<T, D>* tree_, int node_num = 0);
      TreeNode(KDTree_<T, D>* tree_,
               int node_num,
               const VectorTD& mean_,
               const VectorTD& normal_,
               TreeNode* left_child  = 0,
               TreeNode* right_child = 0);
      //! dtor
      ~TreeNode();


      //! function to search for all the points near to the query point
      //! it does a complete search, by going left, right or in both directions
      //! depending on the threshold
      //! @param answer: the neighbors found
      //! @param query: the point to search
      //! @param maximum distance allowed for a point
      //! @returns void
      void findNeighbors(VectorTDVector& answers,
                         std::vector<int>& indices,
                         const VectorTD& query,
                         const T maxumum_distance,
                         KDTreeSearchType search_type) const;
      
      inline  T planeDistance(const VectorTD& query_point) const {
        assert (_node_type==Middle && "bookkeeping error, node is not middle");
        return _normal.dot(query_point - _mean);
      }
      

    protected:
      KDTree_<T, D>* _tree;
      int _node_num;
      KDTreeNodeType _node_type;
      size_t _min_index;
      size_t _max_index;
      VectorTD _normal;
      VectorTD _mean;
      size_t _num_points;
      TreeNode* _left_child;
      TreeNode* _right_child;
    };

    //! ctor
    KDTree_(const VectorTDVector& points_, T max_leaf_range, size_t min_leaf_points = 20);
    
    //! dtor
    ~KDTree_();
    
    bool sanityCheck() const;
    int sanityCheck(std::vector<int>& checked_indices, int k, const TreeNode* node) const;
    //! num_nodes accessor
    inline size_t numNodes() const {
      return _num_nodes;
    }

    inline size_t numPoints() const {
      return _points.size();
    }

    //! function to search for all the points in the same leaf of the query
    //! point
    //! @param answers: the neighbors found
    //! @param indices: the indices of the neighbors found
    //! @param query: the query point
    //! @param max_distance: maximum distance allowed for a point
    //! @returns void

    void findNeighbors(VectorTDVector& answers,
                       std::vector<int>& indices,
                       const VectorTD& query,
                       const T max_distance,
                       const KDTreeSearchType search_type=Approximate) const;

    T findNeighbor(VectorTD& answer,
                   int& index,
                   const VectorTD& query,
                   const T distance,
                   const KDTreeSearchType search_type=Approximate) const;
    
    inline void printKDTree() { _printKDTree(_root); }

    mutable int good_queries;

  protected:
    /**
       Partitions a point vector in two vectors, computing the splitting plane
       as the largest eigenvalue of the point covariance
       @param mean: the returned mean of the splitting plane
       @param normal: the normal of the splitting plane
       @param left: the returned left vector of points
       @param right: the returned right vector of points
       @param points: the array of points
       @returns the distance of the farthest point from the plane
    */
    T _splitPoints(VectorTD& mean,
                   VectorTD& normal,
                   size_t& num_left_points,
                   const size_t min_index,
                   const size_t max_index);
    
    //! function to build the tree
    //! @param points: the points
    //! @param max_leaf_range: specify the size of the "box" below which a leaf
    //! node is generated returns the root of the search tree
    TreeNode*
    _buildTree(const size_t min_index, const size_t max_index, const T max_leaf_range, int level);
    
    void _printKDTree(TreeNode* node);
    
    size_t _num_nodes;
    size_t _min_leaf_points;
    TreeNode* _root;
    VectorTDVector _aux_points;
    VectorTDVector _points;
    std::vector<int> _indices;
    std::vector<int> _aux_indices;
  };

} // namespace srrg2_core

#include "kd_tree_impl.cpp"
