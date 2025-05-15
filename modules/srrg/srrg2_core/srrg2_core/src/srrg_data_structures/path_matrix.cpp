#include "path_matrix.h"
#include <srrg_system_utils/shell_colors.h>

namespace srrg2_core {
  using namespace std;

  float PathMatrix::maxValue(const Type channel) const {
    float max_val = 0;
    for (const PathMatrixCell& cell : _data) {
      if (!cell.parent) {
        continue;
      }
      switch (channel) {
        case Distance:
          max_val = std::max(max_val, cell.distance);
          break;
        case Cost:
          max_val = std::max(max_val, cell.cost);
          break;
        case Parent:
          throw std::runtime_error("unhandled enum");
      }
    }
    return max_val;
  }

  void PathMatrix::toImage(BaseImage& dest_, const Type channel) const {
    if (channel == Parent) {
      if (dest_.type() != ImageType::TYPE_32SC1) {
        throw std::runtime_error("exporting parent only supports type TYPE_32SC1");
      }
      ImageInt& dest = dynamic_cast<ImageInt&>(dest_);
      dest.resize(rows(), cols());
      const PathMatrixCell* start = &(_data[0]);
      for (size_t i = 0; i < data().size(); ++i) {
        int d;
        const PathMatrixCell& cell   = _data[i];
        const PathMatrixCell* parent = cell.parent;
        if (!parent) {
          d = -1;
        } else {
          d = (parent - start) % 65536;
        }
        dest.data()[i] = d;
      }
      return;
    }

    if (channel == Distance) {
      if (dest_.type() == ImageType::TYPE_32FC1) {
        ImageFloat& dest = dynamic_cast<ImageFloat&>(dest_);
        dest.resize(rows(), cols());
        for (size_t i = 0; i < _data.size(); ++i) {
          dest.data()[i] = data()[i].distance;
        }
        return;
      }
      if (dest_.type() == ImageType::TYPE_8UC1) {
        ImageUInt8& dest = dynamic_cast<ImageUInt8&>(dest_);
        dest.resize(rows(), cols());
        float md = maxValue(channel);
        float id = 255. / sqrt(md);
        for (size_t i = 0; i < _data.size(); ++i) {
          dest.data()[i] = std::max(255 - id * sqrt(data()[i].distance), 0.f);
        }
        return;
      }
    }

    if (channel == Cost) {
      if (dest_.type() == ImageType::TYPE_32FC1) {
        ImageFloat& dest = dynamic_cast<ImageFloat&>(dest_);
        dest.resize(rows(), cols());
        for (size_t i = 0; i < _data.size(); ++i) {
          dest.data()[i] = data()[i].cost;
        }
        return;
      }
      if (dest_.type() == ImageType::TYPE_8UC1) {
        ImageUInt8& dest = dynamic_cast<ImageUInt8&>(dest_);
        dest.resize(rows(), cols());
        float md = maxValue(channel) /* / 127*/;
        float id = 255. / md;
        for (size_t i = 0; i < _data.size(); ++i) {
          dest.data()[i] = std::max(255 - id * data()[i].cost, 0.f);
        }
        return;
      }
    }
    throw std::runtime_error("unsupported export");
  }

  void PathMatrix::fromImage(const BaseImage& src_, const Type channel) {
    if (channel == Parent) {
      if (src_.type() != ImageType::TYPE_32SC1) {
        throw std::runtime_error("exporting parent only supports type TYPE_32SC1");
      }
      const ImageInt& src = dynamic_cast<const ImageInt&>(src_);
      resize(src.rows(), src.cols());
      // const int* start=&(src.data()[0]);
      for (size_t i = 0; i < data().size(); ++i) {
        PathMatrixCell& cell = _data[i];
        size_t idx           = src.data()[i];
        if (idx < 0 || idx > src.data().size()) {
          cell.parent   = 0;
          cell.distance = std::numeric_limits<float>::max();
          continue;
        }
        cell.parent = &(data()[idx]);
      }
      return;
    }

    if (channel == Distance) {
      if (src_.type() == ImageType::TYPE_32FC1) {
        const ImageFloat& src = dynamic_cast<const ImageFloat&>(src_);
        resize(src.rows(), src.cols());
        for (size_t i = 0; i < _data.size(); ++i) {
          data()[i].distance = src.data()[i];
        }
        return;
      }
      if (src_.type() == ImageType::TYPE_8UC1) {
        const ImageUInt8& src = dynamic_cast<const ImageUInt8&>(src_);
        resize(src.rows(), src.cols());
        for (size_t i = 0; i < _data.size(); ++i) {
          const int c       = 127 - src.data()[i];
          _data[i].distance = c * c;
        }
        return;
      }
    }

    if (channel == Cost) {
      if (src_.type() == ImageType::TYPE_32FC1) {
        const ImageFloat& src = dynamic_cast<const ImageFloat&>(src_);
        resize(src.rows(), src.cols());
        for (size_t i = 0; i < _data.size(); ++i) {
          _data[i].cost = src.data()[i];
        }
        return;
      }
      if (src_.type() == ImageType::TYPE_8UC1) {
        const ImageUInt8& src = dynamic_cast<const ImageUInt8&>(src_);
        resize(src.rows(), src.cols());
        for (size_t i = 0; i < _data.size(); ++i) {
          const int c   = 255 - src.data()[i];
          _data[i].cost = c;
        }
        return;
      }
    }
  }
} // namespace srrg2_core
