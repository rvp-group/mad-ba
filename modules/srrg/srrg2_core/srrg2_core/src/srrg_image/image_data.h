#pragma once
#include "image.h"
#include "srrg_boss/blob.h"
#include <memory>

typedef int CvImageType;
namespace srrg2_core {

  struct ImageHeader {
    ImageType type;
    int rows;
    int cols;
  };

  // generic image
  class ImageData : public BLOB {
  public:
    ImageData();
    virtual ~ImageData() {
    }

    virtual inline void setImage(BaseImage* image_) {
      _image_ptr.reset(image_);
    }

    virtual inline const BaseImage* image() const {
      return _image_ptr.get();
    }

    virtual inline BaseImage* image() {
      return _image_ptr.get();
    }

    // ia move initialization only (since unique ptrs)
    inline void setImagePtr(std::unique_ptr<BaseImage>&& image_) {
      _image_ptr = std::move(image_);
    }

    virtual bool read(std::istream& is);
    virtual void write(std::ostream& os) const;
    const std::string& const_extension() const;
    virtual const std::string& extension();

  protected:
    void _encodeImage(std::ostream& os_) const;
    bool _decodeImage(const std::vector<unsigned char>& buffer_);

    std::unique_ptr<BaseImage> _image_ptr;
  };

  // type specific image. No need for indirection;
  template <typename ImageType_>
  class ImageData_ : public ImageData, public ImageType_ {
  public:
    using ImageType = ImageType_;

    virtual inline void setImage(BaseImage* image_) {
      throw std::runtime_error("no");
    }

    virtual inline const BaseImage* image() const {
      return this;
    }

    virtual inline BaseImage* image() {
      return this;
    }
  };

  using ImageUInt8Data     = ImageData_<ImageUInt8>;
  using ImageUInt16Data    = ImageData_<ImageUInt16>;
  using ImageIntData       = ImageData_<ImageInt>;
  using ImageFloatData     = ImageData_<ImageFloat>;
  using ImageDoubleData    = ImageData_<ImageDouble>;
  using ImageVector3ucData = ImageData_<ImageVector3uc>;
  using ImageVector3fData  = ImageData_<ImageVector3f>;
  using ImageVector4ucData = ImageData_<ImageVector4uc>;
  using ImageVector4fData  = ImageData_<ImageVector4f>;
  using ImageVector5fData  = ImageData_<ImageVector5f>;

  using ImageDataBLOBReference          = BLOBReference<ImageData>;
  using ImageUInt8DataBLOBReference     = BLOBReference<ImageUInt8Data>;
  using ImageUInt16DataBLOBReference    = BLOBReference<ImageUInt16Data>;
  using ImageIntDataBLOBReference       = BLOBReference<ImageIntData>;
  using ImageFloatDataBLOBReference     = BLOBReference<ImageFloatData>;
  using ImageDoubleDataBLOBReference    = BLOBReference<ImageDoubleData>;
  using ImageVector3ucDataBLOBReference = BLOBReference<ImageVector3ucData>;
  using ImageVector3fDataBLOBReference  = BLOBReference<ImageVector3fData>;
  using ImageVector4ucDataBLOBReference = BLOBReference<ImageVector4ucData>;
  using ImageVector4fDataBLOBReference  = BLOBReference<ImageVector4fData>;
  using ImageVector5fDataBLOBReference  = BLOBReference<ImageVector5fData>;

} // namespace srrg2_core
