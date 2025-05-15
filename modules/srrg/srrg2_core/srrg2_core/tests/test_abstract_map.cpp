#include <iostream>

#include "srrg_data_structures/abstract_map.h"
#include "srrg_data_structures/abstract_ptr_map.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

using AbstractIntFloatMap       = AbstractMap_<int, float>;
using AbstractIntFloatPtrMap    = AbstractPtrMap_<int, float, std::shared_ptr<float>>;
using AbstractIntFloatRawPtrMap = AbstractMap_<int, float*>;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(DummyData, AbstractIntFloatMap) {
  // ia I need this shitty vectors because I want also to check KEYS of the abstract map
  std::vector<int> ref_indices({1, 5, 2});
  std::vector<float> ref_values({0.1f, 0.5f, 0.2f});
  std::vector<int> ref_indices_sorted  = ref_indices;
  std::vector<float> ref_values_sorted = ref_values;
  std::sort(ref_indices_sorted.begin(), ref_indices_sorted.end());
  std::sort(ref_values_sorted.begin(), ref_values_sorted.end());

  AbstractIntFloatMap m;

  // ia populate
  {
    size_t i = 0;
    for (; i < ref_indices.size(); ++i) {
      m.insert(std::make_pair(ref_indices[i], ref_values[i]));
    }

    // ia access
    i = 0;
    for (auto it = m.begin(); it != m.end(); ++it, ++i) {
      ASSERT_EQ(ref_indices_sorted[i], (*it).first);
      ASSERT_EQ(ref_values_sorted[i], (*it).second);
    }
  }

  {
    // ia search
    auto it = m.find(3);
    ASSERT_TRUE(it == m.end());

    // ia erase
    const auto ref_key   = ref_indices[1];
    const auto ref_value = ref_values[1];

    ref_values.erase(ref_values.begin() + 1);
    ref_indices.erase(ref_indices.begin() + 1);

    auto ref_key_sorted_it =
      std::find(ref_indices_sorted.begin(), ref_indices_sorted.end(), ref_key);
    ref_indices_sorted.erase(ref_key_sorted_it);

    auto ref_value_sorted_it =
      std::find(ref_values_sorted.begin(), ref_values_sorted.end(), ref_value);
    ref_values_sorted.erase(ref_value_sorted_it);

    it = m.find(ref_key);
    ASSERT_FALSE(it == m.end());
    ASSERT_EQ(it.key(), ref_key);
    ASSERT_EQ(it.value(), ref_value);

    m.erase(it);
    ASSERT_EQ(m.size(), ref_indices.size());
    ASSERT_EQ(m.size(), ref_values.size());
  }

  // ia final check
  {
    size_t i = 0;
    for (auto it = m.begin(); it != m.end(); ++it, ++i) {
      ASSERT_EQ(ref_indices[i], (*it).first);
      ASSERT_EQ(ref_values[i], (*it).second);
    }
  }
}

TEST(DummyData, AbstractIntFloatPtrMap) {
  std::vector<int> ref_indices({1, 5, 2});
  std::vector<float> ref_values({0.1f, 0.5f, 0.2f});
  std::vector<int> ref_indices_sorted  = ref_indices;
  std::vector<float> ref_values_sorted = ref_values;
  std::sort(ref_indices_sorted.begin(), ref_indices_sorted.end());
  std::sort(ref_values_sorted.begin(), ref_values_sorted.end());

  AbstractIntFloatPtrMap m;

  // ia populate
  {
    size_t i = 0;
    for (; i < ref_indices.size(); ++i) {
      const auto val = ref_values[i];
      m.insert(std::make_pair(ref_indices[i], std::make_shared<float>(val)));
    }

    // ia access
    i = 0;
    for (auto it = m.begin(); it != m.end(); ++it, ++i) {
      ASSERT_EQ(ref_indices_sorted[i], (*it).first);
      ASSERT_EQ(ref_values_sorted[i], *(*it).second);
    }
  }

  {
    // ia search
    auto it = m.find(3);
    ASSERT_TRUE(it == m.end());

    // ia erase
    const auto ref_key   = ref_indices[1];
    const auto ref_value = ref_values[1];

    ref_values.erase(ref_values.begin() + 1);
    ref_indices.erase(ref_indices.begin() + 1);

    auto ref_key_sorted_it =
      std::find(ref_indices_sorted.begin(), ref_indices_sorted.end(), ref_key);
    ref_indices_sorted.erase(ref_key_sorted_it);

    auto ref_value_sorted_it =
      std::find(ref_values_sorted.begin(), ref_values_sorted.end(), ref_value);
    ref_values_sorted.erase(ref_value_sorted_it);

    it = m.find(ref_key);
    ASSERT_FALSE(it == m.end());
    ASSERT_EQ(it.key(), ref_key);
    ASSERT_EQ(*it.value(), ref_value);

    m.erase(it);
    ASSERT_EQ(m.size(), ref_indices.size());
    ASSERT_EQ(m.size(), ref_values.size());
  }

  // ia final check
  {
    size_t i = 0;
    for (auto it = m.begin(); it != m.end(); ++it, ++i) {
      ASSERT_EQ(ref_indices[i], (*it).first);
      ASSERT_EQ(ref_values[i], *(*it).second);
    }
  }
}

TEST(DummyData, AbstractIntFloatRawPtrMap) {
  std::vector<int> ref_indices({1, 5, 2});
  std::vector<float> ref_values({0.1f, 0.5f, 0.2f});
  std::vector<int> ref_indices_sorted  = ref_indices;
  std::vector<float> ref_values_sorted = ref_values;
  std::sort(ref_indices_sorted.begin(), ref_indices_sorted.end());
  std::sort(ref_values_sorted.begin(), ref_values_sorted.end());

  AbstractIntFloatRawPtrMap m;

  // ia populate
  {
    size_t i = 0;
    for (; i < ref_indices.size(); ++i) {
      const auto val = ref_values[i];
      m.insert(std::make_pair(ref_indices[i], new float(val)));
    }

    // ia access
    i = 0;
    for (auto it = m.begin(); it != m.end(); ++it, ++i) {
      ASSERT_EQ(ref_indices_sorted[i], (*it).first);
      ASSERT_EQ(ref_values_sorted[i], *(*it).second);
    }
  }

  {
    // ia search
    auto it = m.find(3);
    ASSERT_TRUE(it == m.end());

    // ia erase
    const auto ref_key   = ref_indices[1];
    const auto ref_value = ref_values[1];

    ref_values.erase(ref_values.begin() + 1);
    ref_indices.erase(ref_indices.begin() + 1);

    auto ref_key_sorted_it =
      std::find(ref_indices_sorted.begin(), ref_indices_sorted.end(), ref_key);
    ref_indices_sorted.erase(ref_key_sorted_it);

    auto ref_value_sorted_it =
      std::find(ref_values_sorted.begin(), ref_values_sorted.end(), ref_value);
    ref_values_sorted.erase(ref_value_sorted_it);

    it = m.find(ref_key);
    ASSERT_FALSE(it == m.end());
    ASSERT_EQ(it.key(), ref_key);
    ASSERT_EQ(*it.value(), ref_value);
    // ia remember to deallocate memory
    delete it.value();

    m.erase(it);
    ASSERT_EQ(m.size(), ref_indices.size());
    ASSERT_EQ(m.size(), ref_values.size());
  }

  // ia final check
  {
    size_t i = 0;
    for (auto it = m.begin(); it != m.end(); ++it, ++i) {
      ASSERT_EQ(ref_indices[i], (*it).first);
      ASSERT_EQ(ref_values[i], *(*it).second);
      // ia remember to deallocate memory
      delete it.value();
    }
  }
}
