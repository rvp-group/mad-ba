#include "image_data.h"
#include <opencv2/opencv.hpp>

const std::string DAT_EXTENSION = "dat";
const std::string PNG_EXTENSION = "png";
const std::string PGM_EXTENSION = "pgm";

namespace srrg2_core {
  using namespace std;

  ImageData::ImageData() : BLOB() {
    _image_ptr.reset(0);
  }

  bool ImageData::read(std::istream& is) {
    // ia WHYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY
    // BaseImage* _image = image();
    std::vector<unsigned char> buf;
    const int block_size = 4096;
    int bytes_read       = 0;
    while (is) {
      size_t old_size = buf.size();
      buf.resize(buf.size() + block_size);
      is.read(reinterpret_cast<char*>(&buf[old_size]), block_size);
      bytes_read += is.gcount();
    }
    if (bytes_read > 0) {
      buf.resize(bytes_read);
      //      std::cerr << "buf size: " << buf.size() << std::endl;
      cv::Mat read_image = cv::imdecode(buf, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
      if (!read_image.empty()) {
        //        std::cerr << "converting: " << std::endl;
        if (_image_ptr == nullptr) {
          // _image = BaseImage::newImagefromCv(read_image);
          // _image_ptr.reset(_image);
          _image_ptr.reset(BaseImage::newImagefromCv(read_image));
        } else {
          // _image->fromCv(read_image);
          _image_ptr->fromCv(read_image);
        }
        return true;
      }
      if (_decodeImage(buf)) {
        return true;
      }
    }
    return false;
  }

  void ImageData::write(std::ostream& os) const {
    assert(_image_ptr != nullptr && "ImageData::write|ERROR, invalid image");
    // ia WHYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY
    // const BaseImage* _image = image();
    std::vector<unsigned char> buf;
    cv::Mat image_to_write;

    if (_image_ptr->type() == TYPE_UNKNOWN) {
      throw std::runtime_error("Unknown image type!");
    }
    _image_ptr->toCv(image_to_write);
    if (_image_ptr->type() == TYPE_8UC3 || _image_ptr->type() == TYPE_32FC3) {
      cv::cvtColor(image_to_write, image_to_write, CV_BGR2RGB);
    }

    if (const_extension() == DAT_EXTENSION) {
      _encodeImage(os);
    } else {
      cv::imencode(std::string(".") + const_extension(), image_to_write, buf);
      os.write(reinterpret_cast<char*>(&buf[0]), buf.size());
    }
  }

  void ImageData::_encodeImage(std::ostream& os_) const {
    assert(_image_ptr != nullptr && "ImageData::_encodeImage|ERROR, invalid image");
    // ia WHYYYYYYYYYYYYYYYY
    // const BaseImage* _image = image();
    ImageHeader header;
    header.type = _image_ptr->type();
    header.rows = _image_ptr->rows();
    header.cols = _image_ptr->cols();

    const std::size_t size = header.rows * header.cols;

    os_.write((char*) &header, sizeof(ImageHeader));

    const std::size_t element_size = CV_ELEM_SIZE(_image_ptr->type());

    // Syscall param writev(vector[...]) points to uninitialised byte(s)
    os_.write(_image_ptr->rawData(), size * element_size);
    //-----------???--------------
  }

  bool ImageData::_decodeImage(const std::vector<unsigned char>& buffer_) {
    // ia read the header
    ImageHeader* header = (ImageHeader*) &buffer_[0];
    std::vector<unsigned char> data_buffer(buffer_.begin() + sizeof(ImageHeader), buffer_.end());

    // const std::size_t element_size = CV_ELEM_SIZE(header->type);
    // const std::size_t vector_size = header->rows*header->cols;

    // ia working with smart pointer, forget the goddamn .get method
    // ia Mircolosi master of smart pointers from what I see below
    // BaseImage* _image = _image_ptr.get();
    // if (_image) {
    //   delete _image;
    // }
    // _image = 0;

    int rows = header->rows;
    int cols = header->cols;
    switch (header->type) {
      case TYPE_8UC1:
        _image_ptr.reset(new ImageUInt8(rows, cols));
        break;
      case TYPE_16UC1:
        _image_ptr.reset(new ImageUInt16(rows, cols));
        break;
      case TYPE_32SC1:
        _image_ptr.reset(new ImageInt(rows, cols));
        break;
      case TYPE_32FC1:
        _image_ptr.reset(new ImageFloat(rows, cols));
        break;
      case TYPE_64FC1:
        _image_ptr.reset(new ImageDouble(rows, cols));
        break;
      case TYPE_8UC3:
        _image_ptr.reset(new ImageVector3uc(rows, cols));
        break;
      case TYPE_32FC3:
        _image_ptr.reset(new ImageVector3f(rows, cols));
        break;
      case TYPE_8UC4:
        _image_ptr.reset(new ImageVector4uc(rows, cols));
        break;
      case TYPE_32FC4:
        _image_ptr.reset(new ImageVector4f(rows, cols));
        break;
      default:
        throw std::runtime_error("ImageData::_decodeImage|ERROR, unknown type" +
                                 std::to_string(header->type) + " ]");
        return false;
    }
    _image_ptr->fromBuffer(&data_buffer[0]);
    return true;
  }

  // TODO make extension() const in BLOB and avoid this shit down here
  const std::string& ImageData::const_extension() const {
    // ia WHYYYYYYYYYYYYYYY
    // const BaseImage* _image = image();
    switch (_image_ptr->type()) {
      case TYPE_8UC3:
      case TYPE_8UC1:
        return PNG_EXTENSION;
      case TYPE_16UC1:
        return PGM_EXTENSION;
      default:
        return DAT_EXTENSION;
    }
  }

  const std::string& ImageData::extension() {
    return const_extension();
  }

  BOSS_REGISTER_BLOB(ImageData)
  BOSS_REGISTER_BLOB(ImageUInt8Data)
  BOSS_REGISTER_BLOB(ImageUInt16Data)
  BOSS_REGISTER_BLOB(ImageIntData)
  BOSS_REGISTER_BLOB(ImageFloatData)
  BOSS_REGISTER_BLOB(ImageDoubleData)
  BOSS_REGISTER_BLOB(ImageVector3ucData)
  BOSS_REGISTER_BLOB(ImageVector3fData)
  BOSS_REGISTER_BLOB(ImageVector4ucData)
  BOSS_REGISTER_BLOB(ImageVector4fData)

} // namespace srrg2_core
