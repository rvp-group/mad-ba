#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>

#include "srrg_image/image_data.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

static constexpr size_t image_rows = 100;
static constexpr size_t image_cols = 100;
static constexpr float step        = 1 / (float) (image_rows * image_cols);
using ImageDataPtr                 = std::shared_ptr<ImageData>;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

static void writeImageData(const std::string& filename_, const ImageDataPtr& image_data_) {
#ifndef NDEBUG
  std::cerr << "writing image [ " << filename_ << " ]\n";
#endif
  std::ofstream stream(filename_, std::ios::out);
  image_data_->write(stream);
  stream.close();
}

static void readImageData(const std::string& filename_, const ImageDataPtr& image_data_) {
#ifndef NDEBUG
  std::cerr << "reading image [ " << filename_ << " ]\n";
#endif
  std::ifstream stream(filename_, std::ios::in);
  if (!image_data_->read(stream)) {
    throw std::runtime_error("cannot read image [ " + filename_ + " ] ");
  }
  stream.close();
}

TEST(DummyData, ImageUInt8) {
  using ImageType               = ImageUInt8;
  using DataType                = uint8_t;
  static constexpr DataType max = std::numeric_limits<DataType>::max();
  std::unique_ptr<ImageType> image(new ImageType(image_rows, image_cols));
  ASSERT_NOTNULL(image);

  float value = 0;
  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      const DataType scaled = value * max;
      image->at(r, c)       = scaled;
      value += step;
    }
  }

  // ia image changes ownership to the ImageData
  ImageDataPtr image_data(new ImageData);
  image_data->setImagePtr(std::move(image));

  // ia save
  const std::string image_filename = "image_uint8." + image_data->const_extension();
  writeImageData(image_filename, image_data);

  // ia reload and check
  ImageDataPtr b_image_data(new ImageData);
  b_image_data->setImagePtr(std::unique_ptr<ImageType>(new ImageType));
  readImageData(image_filename, b_image_data);

  ASSERT_EQ(b_image_data->image()->rows(), image_rows);
  ASSERT_EQ(b_image_data->image()->cols(), image_cols);

  auto reference = dynamic_cast<ImageType*>(image_data->image());
  auto query     = dynamic_cast<ImageType*>(b_image_data->image());
  ASSERT_NOTNULL(reference);
  ASSERT_NOTNULL(query);

  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      ASSERT_EQ(reference->at(r, c), query->at(r, c));
    }
  }

  std::remove(image_filename.c_str());
  ASSERT_FALSE(srrg2_core::isAccessible(image_filename));
}

TEST(DummyData, ImageUInt16) {
  using ImageType               = ImageUInt16;
  using DataType                = uint16_t;
  static constexpr DataType max = std::numeric_limits<DataType>::max();
  std::unique_ptr<ImageType> image(new ImageType(image_rows, image_cols));
  ASSERT_NOTNULL(image);

  float value = 0;
  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      const DataType scaled = value * max;
      image->at(r, c)       = scaled;
      value += step;
    }
  }

  // ia image changes ownership to the ImageData
  ImageDataPtr image_data(new ImageData);
  image_data->setImagePtr(std::move(image));

  // ia save
  const std::string image_filename = "image_uint16." + image_data->const_extension();
  writeImageData(image_filename, image_data);

  // ia reload and check
  ImageDataPtr b_image_data(new ImageData);
  b_image_data->setImagePtr(std::unique_ptr<ImageType>(new ImageType));
  readImageData(image_filename, b_image_data);

  ASSERT_EQ(b_image_data->image()->rows(), image_rows);
  ASSERT_EQ(b_image_data->image()->cols(), image_cols);

  auto reference = dynamic_cast<ImageType*>(image_data->image());
  auto query     = dynamic_cast<ImageType*>(b_image_data->image());
  ASSERT_NOTNULL(reference);
  ASSERT_NOTNULL(query);

  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      ASSERT_EQ(reference->at(r, c), query->at(r, c));
    }
  }

  std::remove(image_filename.c_str());
  ASSERT_FALSE(srrg2_core::isAccessible(image_filename));
}

TEST(DummyData, ImageInt32) {
  using ImageType               = ImageInt;
  using DataType                = int;
  static constexpr DataType max = std::numeric_limits<DataType>::max();
  std::unique_ptr<ImageType> image(new ImageType(image_rows, image_cols));
  ASSERT_NOTNULL(image);

  float value = 0;
  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      const DataType scaled = value * max;
      image->at(r, c)       = scaled;
      value += step;
    }
  }

  // ia image changes ownership to the ImageData
  ImageDataPtr image_data(new ImageData);
  image_data->setImagePtr(std::move(image));

  // ia save
  const std::string image_filename = "image_int32." + image_data->const_extension();
  writeImageData(image_filename, image_data);

  // ia reload and check
  ImageDataPtr b_image_data(new ImageData);
  b_image_data->setImagePtr(std::unique_ptr<ImageType>(new ImageType));
  readImageData(image_filename, b_image_data);

  ASSERT_EQ(b_image_data->image()->rows(), image_rows);
  ASSERT_EQ(b_image_data->image()->cols(), image_cols);

  auto reference = dynamic_cast<ImageType*>(image_data->image());
  auto query     = dynamic_cast<ImageType*>(b_image_data->image());
  ASSERT_NOTNULL(reference);
  ASSERT_NOTNULL(query);

  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      ASSERT_EQ(reference->at(r, c), query->at(r, c));
    }
  }

  std::remove(image_filename.c_str());
  ASSERT_FALSE(srrg2_core::isAccessible(image_filename));
}

TEST(DummyData, ImageFloat32) {
  using ImageType = ImageFloat;
  // using DataType  = float;
  // static constexpr DataType max = std::numeric_limits<DataType>::max();
  std::unique_ptr<ImageType> image(new ImageType(image_rows, image_cols));
  ASSERT_NOTNULL(image);

  float value = 0;
  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      image->at(r, c) = value;
      value += step;
    }
  }

  // ia image changes ownership to the ImageData
  ImageDataPtr image_data(new ImageData);
  image_data->setImagePtr(std::move(image));

  // ia save
  const std::string image_filename = "image_float32." + image_data->const_extension();
  writeImageData(image_filename, image_data);

  // ia reload and check
  ImageDataPtr b_image_data(new ImageData);
  b_image_data->setImagePtr(std::unique_ptr<ImageType>(new ImageType));
  readImageData(image_filename, b_image_data);

  ASSERT_EQ(b_image_data->image()->rows(), image_rows);
  ASSERT_EQ(b_image_data->image()->cols(), image_cols);

  auto reference = dynamic_cast<ImageType*>(image_data->image());
  auto query     = dynamic_cast<ImageType*>(b_image_data->image());
  ASSERT_NOTNULL(reference);
  ASSERT_NOTNULL(query);

  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      ASSERT_EQ(reference->at(r, c), query->at(r, c));
    }
  }

  std::remove(image_filename.c_str());
  ASSERT_FALSE(srrg2_core::isAccessible(image_filename));
}

TEST(DummyData, ImageFloat64) {
  using ImageType = ImageDouble;
  using DataType  = double;
  // static constexpr DataType max = std::numeric_limits<DataType>::max();
  std::unique_ptr<ImageType> image(new ImageType(image_rows, image_cols));
  ASSERT_NOTNULL(image);

  float value = 0;
  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      image->at(r, c) = (DataType)(value);
      value += step;
    }
  }

  // ia image changes ownership to the ImageData
  ImageDataPtr image_data(new ImageData);
  image_data->setImagePtr(std::move(image));

  // ia save
  const std::string image_filename = "image_float64." + image_data->const_extension();
  writeImageData(image_filename, image_data);

  // ia reload and check
  ImageDataPtr b_image_data(new ImageData);
  b_image_data->setImagePtr(std::unique_ptr<ImageType>(new ImageType));
  readImageData(image_filename, b_image_data);

  ASSERT_EQ(b_image_data->image()->rows(), image_rows);
  ASSERT_EQ(b_image_data->image()->cols(), image_cols);

  auto reference = dynamic_cast<ImageType*>(image_data->image());
  auto query     = dynamic_cast<ImageType*>(b_image_data->image());
  ASSERT_NOTNULL(reference);
  ASSERT_NOTNULL(query);

  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      ASSERT_EQ(reference->at(r, c), query->at(r, c));
    }
  }

  std::remove(image_filename.c_str());
  ASSERT_FALSE(srrg2_core::isAccessible(image_filename));
}

TEST(DummyData, ImageVec3UInt8) {
  using ImageType               = ImageVector3uc;
  using DataType                = uint8_t;
  static constexpr DataType max = std::numeric_limits<DataType>::max();
  std::unique_ptr<ImageType> image(new ImageType(image_rows, image_cols));
  ASSERT_NOTNULL(image);

  float value = 0;
  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      const DataType scaled = value * max;
      image->at(r, c) << scaled, 0, scaled;
      value += step;
    }
  }

  // ia image changes ownership to the ImageData
  ImageDataPtr image_data(new ImageData);
  image_data->setImagePtr(std::move(image));

  // ia save
  const std::string image_filename = "image_vec3uc." + image_data->const_extension();
  writeImageData(image_filename, image_data);

  // ia reload and check
  ImageDataPtr b_image_data(new ImageData);
  b_image_data->setImagePtr(std::unique_ptr<ImageType>(new ImageType));
  readImageData(image_filename, b_image_data);

  ASSERT_EQ(b_image_data->image()->rows(), image_rows);
  ASSERT_EQ(b_image_data->image()->cols(), image_cols);

  auto reference = dynamic_cast<ImageType*>(image_data->image());
  auto query     = dynamic_cast<ImageType*>(b_image_data->image());
  ASSERT_NOTNULL(reference);
  ASSERT_NOTNULL(query);

  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      ASSERT_EQ_EIGEN(reference->at(r, c), query->at(r, c));
    }
  }

  std::remove(image_filename.c_str());
  ASSERT_FALSE(srrg2_core::isAccessible(image_filename));
}

TEST(DummyData, ImageVec4UInt8) {
  using ImageType               = ImageVector4uc;
  using DataType                = uint8_t;
  static constexpr DataType max = std::numeric_limits<DataType>::max();
  std::unique_ptr<ImageType> image(new ImageType(image_rows, image_cols));
  ASSERT_NOTNULL(image);

  float value = 0;
  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      const DataType scaled = value * max;
      image->at(r, c) << scaled, 0, scaled, scaled;
      value += step;
    }
  }

  // ia image changes ownership to the ImageData
  ImageDataPtr image_data(new ImageData);
  image_data->setImagePtr(std::move(image));

  // ia save
  const std::string image_filename = "image_vec4uc." + image_data->const_extension();
  writeImageData(image_filename, image_data);

  // ia reload and check
  ImageDataPtr b_image_data(new ImageData);
  b_image_data->setImagePtr(std::unique_ptr<ImageType>(new ImageType));
  readImageData(image_filename, b_image_data);

  ASSERT_EQ(b_image_data->image()->rows(), image_rows);
  ASSERT_EQ(b_image_data->image()->cols(), image_cols);

  auto reference = dynamic_cast<ImageType*>(image_data->image());
  auto query     = dynamic_cast<ImageType*>(b_image_data->image());
  ASSERT_NOTNULL(reference);
  ASSERT_NOTNULL(query);

  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      ASSERT_EQ_EIGEN(reference->at(r, c), query->at(r, c));
    }
  }

  std::remove(image_filename.c_str());
  ASSERT_FALSE(srrg2_core::isAccessible(image_filename));
}

TEST(DummyData, ImageVec3float32) {
  using ImageType = ImageVector3f;
  // using DataType  = float;
  // static constexpr DataType max = std::numeric_limits<DataType>::max();
  std::unique_ptr<ImageType> image(new ImageType(image_rows, image_cols));
  ASSERT_NOTNULL(image);

  float value = 0;
  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      image->at(r, c) << value, 0, value;
      value += step;
    }
  }

  // ia image changes ownership to the ImageData
  ImageDataPtr image_data(new ImageData);
  image_data->setImagePtr(std::move(image));

  // ia save
  const std::string image_filename = "image_vec3f." + image_data->const_extension();
  writeImageData(image_filename, image_data);

  // ia reload and check
  ImageDataPtr b_image_data(new ImageData);
  b_image_data->setImagePtr(std::unique_ptr<ImageType>(new ImageType));
  readImageData(image_filename, b_image_data);

  ASSERT_EQ(b_image_data->image()->rows(), image_rows);
  ASSERT_EQ(b_image_data->image()->cols(), image_cols);

  auto reference = dynamic_cast<ImageType*>(image_data->image());
  auto query     = dynamic_cast<ImageType*>(b_image_data->image());
  ASSERT_NOTNULL(reference);
  ASSERT_NOTNULL(query);

  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      ASSERT_EQ_EIGEN(reference->at(r, c), query->at(r, c));
    }
  }

  std::remove(image_filename.c_str());
  ASSERT_FALSE(srrg2_core::isAccessible(image_filename));
}

TEST(DummyData, ImageVec4float32) {
  using ImageType = ImageVector4f;
  // using DataType  = float;
  // static constexpr DataType max = std::numeric_limits<DataType>::max();
  std::unique_ptr<ImageType> image(new ImageType(image_rows, image_cols));
  ASSERT_NOTNULL(image);

  float value = 0;
  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      image->at(r, c) << value, 0, value, value;
      value += step;
    }
  }

  // ia image changes ownership to the ImageData
  ImageDataPtr image_data(new ImageData);
  image_data->setImagePtr(std::move(image));

  // ia save
  const std::string image_filename = "image_vec4f." + image_data->const_extension();
  writeImageData(image_filename, image_data);

  // ia reload and check
  ImageDataPtr b_image_data(new ImageData);
  b_image_data->setImagePtr(std::unique_ptr<ImageType>(new ImageType));
  readImageData(image_filename, b_image_data);

  ASSERT_EQ(b_image_data->image()->rows(), image_rows);
  ASSERT_EQ(b_image_data->image()->cols(), image_cols);

  auto reference = dynamic_cast<ImageType*>(image_data->image());
  auto query     = dynamic_cast<ImageType*>(b_image_data->image());
  ASSERT_NOTNULL(reference);
  ASSERT_NOTNULL(query);

  for (size_t r = 0; r < image_rows; ++r) {
    for (size_t c = 0; c < image_cols; ++c) {
      ASSERT_EQ_EIGEN(reference->at(r, c), query->at(r, c));
    }
  }

  std::remove(image_filename.c_str());
  ASSERT_FALSE(srrg2_core::isAccessible(image_filename));
}