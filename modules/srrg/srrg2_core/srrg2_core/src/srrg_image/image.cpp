#include "image.h"

namespace srrg2_core {

  BaseImage::~BaseImage() {
  }

  BaseImage* BaseImage::newImagefromCv(const cv::Mat& other_) {
    BaseImage* new_image = 0;
    switch (other_.type()) {
      case TYPE_8UC1:new_image = new ImageUInt8(other_.rows, other_.cols);
        break;
      case TYPE_16UC1:new_image = new ImageUInt16(other_.rows, other_.cols);
        break;
      case TYPE_32SC1:new_image = new ImageInt(other_.rows, other_.cols);
        break;
      case TYPE_32FC1:new_image = new ImageFloat(other_.rows, other_.cols);
        break;
      case TYPE_64FC1:new_image = new ImageDouble(other_.rows, other_.cols);
        break;
      case TYPE_8UC3:new_image = new ImageVector3uc(other_.rows, other_.cols);
        break;
      case TYPE_32FC3:new_image = new ImageVector3f(other_.rows, other_.cols);
        break;
      case TYPE_8UC4:new_image = new ImageVector4uc(other_.rows, other_.cols);
        break;
      case TYPE_32FC4:new_image = new ImageVector4f(other_.rows, other_.cols);
        break;
      default:throw std::runtime_error("Unknown type");
        return 0;
    }
    new_image->fromCv(other_);
    return new_image;
  }

  const bool ImageSize::operator==(const ImageSize& img_size) const {
    if(width == img_size.width &&
       height == img_size.height)
      return true;
    return false;
  }

  const bool ImageSize::operator!=(const ImageSize& img_size) const {
    if(width != img_size.width ||
       height != img_size.height)
      return true;
    return false;
  }

  const bool ImageSize::operator==(const cv::Size& cv_size) const {
    if(width == static_cast<size_t>(cv_size.width) &&
       height == static_cast<size_t>(cv_size.height) )
      return true;
    return false;
  }    
    
  const bool ImageSize::operator!=(const cv::Size& cv_size) const {
    if(width != static_cast<size_t>(cv_size.width) ||
       height != static_cast<size_t>(cv_size.height) )
      return true;
    return false;
  }
    


  
}
