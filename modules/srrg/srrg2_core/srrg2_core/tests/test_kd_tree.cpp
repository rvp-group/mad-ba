#include "srrg_data_structures/kd_tree.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(KDTree, NearestNeighborPerfect) {
  constexpr size_t number_of_points = 1000;

  // ds generate scenario
  KDTree_<float, 3>::VectorTDVector points_database;
  KDTree_<float, 3>::VectorTDVector points_query;
  points_database.reserve(number_of_points);
  points_query.reserve(number_of_points);
  for (size_t i = 0; i < number_of_points; ++i) {
    Vector3f point(i, i * 2, i * 3);
    points_database.emplace_back(point);
    points_query.emplace_back(point);
  }

  // ds organize database points in a tree
  const KDTree_<float, 3> database(points_database, 1.0f, 10);

  // ds for each query we should retrieve the exact, best database entry
  for (size_t i = 0; i < number_of_points; ++i) {
    Vector3f neighbor    = Vector3f::Zero();
    int index_neighbor   = 0;
    const float distance = database.findNeighbor(neighbor, index_neighbor, points_query[i], 1.0f);
    ASSERT_EQ(neighbor.x(), points_database[i].x());
    ASSERT_EQ(neighbor.y(), points_database[i].y());
    ASSERT_EQ(neighbor.z(), points_database[i].z());
    ASSERT_EQ(static_cast<size_t>(index_neighbor), i);
    ASSERT_EQ(distance, 0.0f);
  }
}

TEST(KDTree, NearestNeighborsPerfect) {
  constexpr size_t number_of_points = 1000;

  // ds generate scenario
  KDTree_<float, 3>::VectorTDVector points_database;
  KDTree_<float, 3>::VectorTDVector points_query;
  points_database.reserve(number_of_points);
  points_query.reserve(number_of_points);
  for (size_t i = 0; i < number_of_points; ++i) {
    Vector3f point(i, i * 2, i * 3);
    points_database.emplace_back(point);
    points_query.emplace_back(point);
  }

  // ds organize database points in a tree
  const KDTree_<float, 3> database(points_database, 1.0f, 10);

  // ds for each query we should retrieve the exact, best database entry
  for (size_t i = 0; i < number_of_points; ++i) {
    const Vector3f point_query = points_query[i];
    KDTree_<float, 3>::VectorTDVector neighbors;
    std::vector<int> indices_neighbors;
    database.findNeighbors(neighbors, indices_neighbors, point_query, 1.0f, KDTreeSearchType::Complete);
    ASSERT_FALSE(neighbors.empty());

    // ds neighbor with the smallest distance must have the same index
    size_t index_best   = 0;
    float distance_best = std::numeric_limits<float>::max();
    for (size_t j = 0; j < neighbors.size(); ++j) {
      const Vector3f& neighbor = neighbors[j];

      // ds verify database integrity
      const int index_neighbor = indices_neighbors[j];
      ASSERT_EQ(neighbor.x(), points_database[index_neighbor].x());
      ASSERT_EQ(neighbor.y(), points_database[index_neighbor].y());
      ASSERT_EQ(neighbor.z(), points_database[index_neighbor].z());

      // ds check if better
      const float distance = (neighbor - point_query).norm();
      if (distance < distance_best) {
        index_best    = j;
        distance_best = distance;
      }
    }

    // ds perfect match must've been found
    ASSERT_EQ(distance_best, 0.0f);
    ASSERT_EQ(static_cast<size_t>(indices_neighbors[index_best]), i);
  }
}

TEST(KDTree, MaximumLeafRange) {
  constexpr size_t number_of_points = 1000;

  // ds generate scenario
  KDTree_<float, 3>::VectorTDVector points;
  points.reserve(number_of_points);
  for (size_t i = 0; i < number_of_points; ++i) {
    if (i < number_of_points / 2) {
      points.emplace_back(Vector3f(0, 0, 0));
    } else {
      points.emplace_back(Vector3f(1, 1, 1));
    }
  }

  // ds organize database points in a tree - range too high for split
  const KDTree_<float, 3> database_flat(points, 1.0f, 10);
  ASSERT_EQ(database_flat.numNodes(), size_t(1) /*no splitting happening*/);
  ASSERT_EQ(database_flat.numPoints(), number_of_points);

  // ds organize database points in a tree - range sufficient for split
  const KDTree_<float, 3> database_deep(points, 0.1f, 10);
  ASSERT_EQ(database_deep.numNodes(), size_t(3) /* root node + 2 leafs*/);
  ASSERT_EQ(database_deep.numPoints(), number_of_points);
}

TEST(KDTree, MinimumNumberOfLeafPoints) {
  constexpr size_t number_of_points = 1000;

  // ds generate scenario
  KDTree_<float, 3>::VectorTDVector points;
  points.reserve(number_of_points);
  for (size_t i = 0; i < number_of_points; ++i) {
    points.emplace_back(Vector3f(i, i * 2, i * 3));
  }

  // ds minimum number of points cannot be met
  const KDTree_<float, 3> database_0(points, 0.1f, number_of_points + 1);
  ASSERT_EQ(database_0.numNodes(), size_t(1) /*no splitting happening*/);
  ASSERT_EQ(database_0.numPoints(), number_of_points);

  // ds minimum number of points can be met marginally, significant depth gain
  const KDTree_<float, 3> database_1(points, 0.1f, number_of_points / 10);
  ASSERT_GT(database_1.numNodes(), 2 * database_0.numNodes());
  ASSERT_EQ(database_1.numPoints(), number_of_points);

  // ds minimum number of points is always met until the last (maximum depth)
  const KDTree_<float, 3> database_2(points, 0.1f, 1);
  ASSERT_EQ(database_2.numNodes(), size_t(2 * number_of_points - 1));
  ASSERT_EQ(database_2.numPoints(), number_of_points);
}

