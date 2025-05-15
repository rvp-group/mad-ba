#include <iostream>
#include <limits>
#include <srrg_image/image_data.h>

#define GETNAME(var) #var

#warning "IA THIS SHOULD BE COMPLETELY REWRITTEN AS UNIT TEST"

using namespace srrg2_core;

std::vector<std::string> files;

void saveImage(char* path, const std::string& var_name, ImageData* image_data) {
  char buf[1024];

  if (std::string(path) != "") {
    sprintf(buf, "%s/%s_img.%s", path, var_name.c_str(), image_data->extension().c_str());
    files.push_back(std::string(buf));
  }
  std::ofstream os(buf);
  printf("type: %d\trows: %ld\tcols: %ld\n",
         image_data->image()->type(),
         image_data->image()->rows(),
         image_data->image()->cols());
  image_data->write(os);
  os.close();
}

void loadImage(const std::string& file_name, ImageData* image_data) {
  std::ifstream is;
  is.open(file_name);
  std::cerr << "reading image: " << file_name << std::endl;
  if (image_data->read(is)) {
    const BaseImage* bi = image_data->image();
    printf("type: %d\trows: %ld\tcols: %ld\n", bi->type(), bi->rows(), bi->cols());
  }
  is.close();
}

int main(int argc, char** argv) {
  if (argc <= 1) {
    printf("NO argv[1]\n");
    exit(-1);
  }

  int rows = 200;
  int cols = 100;

  const float step = 1 / (float) (rows * cols);
  int i            = 0;
  float value      = 0;

  // UINT8
  std::cerr << std::endl << "uint8" << std::endl;
  ImageData* image_data_uint8 = new ImageData();
  ImageUInt8* uint8           = new ImageUInt8(rows, cols);
  const uint8_t max_uint8     = std::numeric_limits<uint8_t>::max();

  std::cerr << "Creating Image" << std::endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const uint8_t scaled_uint8 = value * max_uint8;
      uint8->at(r, c)            = scaled_uint8;
      value += step;
    }
  }
  std::cerr << "Saving Image" << std::endl;
  image_data_uint8->setImage(uint8);
  saveImage(argv[1], GETNAME(uint8), image_data_uint8);
  delete uint8;

  std::cerr << "Loading Image" << std::endl;
  ImageUInt8* uint8_new = new ImageUInt8;
  image_data_uint8->setImage(uint8_new);
  loadImage(files[i], image_data_uint8);

  delete uint8_new;
  delete image_data_uint8;
  ++i;

  // UINT16
  std::cerr << std::endl << "uint16" << std::endl;
  ImageData* image_data_uint16 = new ImageData();
  ImageUInt16* uint16          = new ImageUInt16(rows, cols);
  const uint16_t max_uint16    = std::numeric_limits<uint16_t>::max();

  std::cerr << "Creating Image" << std::endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      uint16->at(r, c) = value * max_uint16;
      value += step;
    }
  }
  std::cerr << "Saving Image" << std::endl;
  image_data_uint16->setImage(uint16);
  saveImage(argv[1], GETNAME(uint16), image_data_uint16);
  delete uint16;

  std::cerr << "Loading Image" << std::endl;
  ImageUInt16* uint16_new = new ImageUInt16;
  image_data_uint16->setImage(uint16_new);
  loadImage(files[i], image_data_uint16);
  ++i;
  delete uint16_new;
  delete image_data_uint16;

  // int32
  std::cerr << std::endl << "int32" << std::endl;
  ImageData* image_data_int = new ImageData();
  ImageInt* int32           = new ImageInt(rows, cols);

  std::cerr << "Creating Image" << std::endl;
  const int max_int = std::numeric_limits<int>::max();
  value             = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      int32->at(r, c) = value * max_int;
      value += step;
    }
  }
  std::cerr << "Saving Image" << std::endl;
  image_data_int->setImage(int32);
  saveImage(argv[1], GETNAME(int32), image_data_int);
  delete int32;

  std::cerr << "Loading Image" << std::endl;
  ImageInt* int32_new = new ImageInt;
  image_data_int->setImage(int32_new);
  loadImage(files[i], image_data_int);
  ++i;
  delete int32_new;
  delete image_data_int;

  // float32
  std::cerr << std::endl << "float32" << std::endl;
  ImageData* image_data_float = new ImageData();
  ImageFloat* float32         = new ImageFloat(rows, cols);

  std::cerr << "Creating Image" << std::endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      float32->at(r, c) = value;
      value += step;
    }
  }

  std::cerr << "Saving Image" << std::endl;
  image_data_float->setImage(float32);
  saveImage(argv[1], GETNAME(float32), image_data_float);
  delete float32;

  std::cerr << "Loading Image" << std::endl;
  ImageFloat* float32_new = new ImageFloat;
  image_data_float->setImage(float32_new);
  loadImage(files[i], image_data_float);
  delete float32_new;
  delete image_data_float;
  ++i;

  // float64
  std::cerr << std::endl << "float64" << std::endl;
  ImageData* image_data_float64 = new ImageData();
  ImageDouble* float64          = new ImageDouble(rows, cols);
  std::cerr << "Creating Image" << std::endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      float64->at(r, c) = (double) value;
      value += step;
    }
  }
  std::cerr << "Saving Image" << std::endl;
  image_data_float64->setImage(float64);
  saveImage(argv[1], GETNAME(float64), image_data_float64);
  delete float64;

  std::cerr << "Loading Image" << std::endl;
  ImageDouble* float64_new = new ImageDouble;
  image_data_float64->setImage(float64_new);
  loadImage(files[i], image_data_float64);
  ++i;
  delete float64_new;
  delete image_data_float64;

  // vec3uc
  std::cerr << std::endl << "vec3uc" << std::endl;
  ImageData* image_data_vec3uc = new ImageData();
  ImageVector3uc* vec3uc       = new ImageVector3uc(rows, cols);
  std::cerr << "Creating Image" << std::endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const uint8_t scaled_uint8 = value * max_uint8;
      vec3uc->at(r, c) << scaled_uint8, scaled_uint8, scaled_uint8;
      value += step;
    }
  }
  std::cerr << "Saving Image" << std::endl;
  image_data_vec3uc->setImage(vec3uc);
  saveImage(argv[1], GETNAME(vec3uc), image_data_vec3uc);
  delete vec3uc;

  std::cerr << "Loading Image" << std::endl;
  ImageVector3uc* vec3uc_new = new ImageVector3uc;
  image_data_vec3uc->setImage(vec3uc_new);
  loadImage(files[i], image_data_vec3uc);
  ++i;
  delete vec3uc_new;
  delete image_data_vec3uc;

  // vec3f
  std::cerr << std::endl << "vec3f" << std::endl;
  ImageData* image_data_vec3f = new ImageData();
  ImageVector3f* vec3f        = new ImageVector3f(rows, cols);
  std::cerr << "Creating Image" << std::endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      vec3f->at(r, c) << value, value, value;
      value += step;
    }
  }
  std::cerr << "Saving Image" << std::endl;
  image_data_vec3f->setImage(vec3f);
  saveImage(argv[1], GETNAME(vec3f), image_data_vec3f);
  delete vec3f;

  std::cerr << "Loading Image" << std::endl;
  ImageVector3f* vec3f_new = new ImageVector3f;
  image_data_vec3f->setImage(vec3f_new);
  loadImage(files[i], image_data_vec3f);
  ++i;
  delete vec3f_new;
  delete image_data_vec3f;

  // vec4uc
  std::cerr << std::endl << "vec4uc" << std::endl;
  ImageData* image_data_vec4uc = new ImageData();
  ImageVector4uc* vec4uc       = new ImageVector4uc(rows, cols);

  std::cerr << "Creating Image" << std::endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const uint8_t scaled_uint8 = value * max_uint8;
      vec4uc->at(r, c) << scaled_uint8, scaled_uint8, scaled_uint8, scaled_uint8;
      value += step;
    }
  }
  std::cerr << "Saving Image" << std::endl;
  image_data_vec4uc->setImage(vec4uc);
  saveImage(argv[1], GETNAME(vec4uc), image_data_vec4uc);
  delete vec4uc;

  std::cerr << "Loading Image" << std::endl;
  ImageVector4uc* vec4uc_new = new ImageVector4uc;
  image_data_vec4uc->setImage(vec4uc_new);
  loadImage(files[i], image_data_vec4uc);
  ++i;
  delete vec4uc_new;
  delete image_data_vec4uc;

  // vec4f
  std::cerr << std::endl << "vec4f" << std::endl;
  ImageData* image_data_vec4f = new ImageData();
  ImageVector4f* vec4f        = new ImageVector4f(rows, cols);

  std::cerr << "Creating Image" << std::endl;
  value = 0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      vec4f->at(r, c) << value, value, value, value;
      value += step;
    }
  }

  std::cerr << "Saving Image" << std::endl;
  image_data_vec4f->setImage(vec4f);
  saveImage(argv[1], GETNAME(vec4f), image_data_vec4f);
  delete vec4f;
  std::cerr << "Loading Image" << std::endl;
  ImageVector4f* vec4f_new = new ImageVector4f;
  image_data_vec4f->setImage(vec4f_new);
  loadImage(files[i], image_data_vec4f);
  ++i;
  delete vec4f_new;
  delete image_data_vec4f;

  return 0;
}
