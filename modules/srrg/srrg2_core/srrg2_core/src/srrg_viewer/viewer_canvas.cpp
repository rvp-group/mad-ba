#include "viewer_canvas.h"
#include "viewer_core/packets.h"

namespace srrg2_core {

#define DEFINE_METHOD(METHOD_NAME_, PARAM_TYPE_)                   \
  void ViewerCanvas::putPoints(const PARAM_TYPE_& points_) {       \
    if (!_setup())                                                 \
      return;                                                      \
    _serializer.putPacket(Packet##PARAM_TYPE_(&points_));                       \
  }

  ViewerCanvas::~ViewerCanvas() {
  }

  // --------------------------------------------------------------------------
  // --------------------- vector point cloud stuff ---------------------------
  // --------------------------------------------------------------------------
  DEFINE_METHOD(putPoints, PointIntensityDescriptor2fVectorCloud)
  DEFINE_METHOD(putPoints, PointIntensityDescriptor3fVectorCloud)

  DEFINE_METHOD(putPoints, Point2fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormal2fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormalColor2fVectorCloud)

  DEFINE_METHOD(putPoints, Point3fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormal3fVectorCloud)
  // DEFINE_METHOD(putPoints, PointNormalCurvature3fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormalColor3fVectorCloud)

  DEFINE_METHOD(putPoints, Point4fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormal4fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormalColor4fVectorCloud)

  DEFINE_METHOD(putPoints, Point2fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormal2fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormalColor2fMatrixCloud)

  DEFINE_METHOD(putPoints, Point3fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormal3fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormalColor3fMatrixCloud)

  DEFINE_METHOD(putPoints, Point4fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormal4fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormalColor4fMatrixCloud)

  void ViewerCanvas::putPolygon(const PointNormalColor3fVectorCloud& points_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketPolygonPointNormalColor3fVectorCloud(&points_));
  }

  void ViewerCanvas::putPolygonWireframe(const PointNormalColor3fVectorCloud& points_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketPolygonWireframePointNormalColor3fVectorCloud(&points_));
  }

  // --------------------------------------------------------------------------
  // --------------------------- everything else ------------------------------
  // --------------------------------------------------------------------------
  void
  ViewerCanvas::putPoints(const int& size_, const Vector3f* points_, const Vector3f* normals_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketPayloadPoints(size_, points_, normals_));
  }

  void ViewerCanvas::putLine(const int& size_, const Vector3f* points_, const Vector3f* normals_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketPayloadLines(size_, points_, normals_));
  }

  void
  ViewerCanvas::putSegment(const int& size_, const Vector3f* points_, const Vector3f* normals_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketPayloadSegments(size_, points_, normals_));
  }

  void ViewerCanvas::putSphere(const float& radius_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectSphere(radius_));
  }

  void ViewerCanvas::putPlane(const Vector2f& size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectPlane(size_));
  }

  void ViewerCanvas::putBox(const Vector3f& size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectBox(size_));
  }

  void ViewerCanvas::putBoxWireframe(const Vector3f& size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectBoxWireframe(size_));
  }

  void ViewerCanvas::putPyramid(const Vector2f& size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectPyramid(size_));
  }

  void ViewerCanvas::putPyramidWireframe(const Vector2f& size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectPyramidWireframe(size_));
  }

  void ViewerCanvas::putEllipsoid(const Vector3f& size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectEllipsoid(size_));
  }

  void ViewerCanvas::putCone(const Vector2f& size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectCone(size_));
  }

  void ViewerCanvas::putDisk(const float& radius_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectDisk(radius_));
  }

  void ViewerCanvas::putCylinder(const Vector2f& size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectCylinder(size_));
  }

  void ViewerCanvas::putArrow2D(const Vector3f& size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectArrow2D(size_));
  }

  void ViewerCanvas::putReferenceSystem(const float& rf_linewidth_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectReferenceFrame(rf_linewidth_));
  }

  void ViewerCanvas::putText(const std::string& text_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketObjectText(text_));
  }

  void ViewerCanvas::multMatrix(const Matrix4f& m) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketTransformMultMatrix(m));
  }

  void ViewerCanvas::scale(const Vector3f& scales) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketTransformMultScale(scales));
  }

  void ViewerCanvas::rotate(const Matrix3f& rotation_matrix) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketTransformMultRotation(rotation_matrix));
  }

  void ViewerCanvas::translate(const Vector3f& translation) {
    if (!_setup())
      return;
    // ia create a packet object
    _serializer.putPacket(PacketTransformMultTraslation(translation));
  }

  void ViewerCanvas::pushMatrix() {
    if (!_setup())
      return;
    _serializer.putPacket(PacketCommandPushMatrix());
  }

  void ViewerCanvas::popMatrix() {
    if (!_setup())
      return;
    _serializer.putPacket(PacketCommandPopMatrix());
  }

  void ViewerCanvas::setPointSize(const float& point_size_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketAttributePointSize(point_size_));
  }

  void ViewerCanvas::setLineWidth(const float& line_width_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketAttributeLineWidth(line_width_));
  }

  void ViewerCanvas::setColor(const Vector3f& color_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketAttributeColorRGB(color_));
  }

  void ViewerCanvas::setColor(const Vector4f& color_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketAttributeColorRGBA(color_));
  }

  void ViewerCanvas::pushPointSize() {
    if (!_setup())
      return;
    _serializer.putPacket(PacketCommandPushPointSize());
  }

  void ViewerCanvas::pushLineWidth() {
    if (!_setup())
      return;
    _serializer.putPacket(PacketCommandPushLineWidth());
  }

  void ViewerCanvas::pushColor() {
    if (!_setup())
      return;
    _serializer.putPacket(PacketCommandPushColor());
  }

  void ViewerCanvas::popAttribute() {
    if (!_setup())
      return;
    _serializer.putPacket(PacketCommandPopAttribute());
  }

  void ViewerCanvas::createList(const std::string& list) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketCreateList(list));
  }

  void ViewerCanvas::beginList(const std::string& list) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketBeginList(list));
  }

  void ViewerCanvas::endList() {
    if (!_setup())
      return;
    _serializer.putPacket(PacketEndList());
  }

  void ViewerCanvas::destroyList(const std::string& list) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketDestroyList(list));
  }

  void ViewerCanvas::callList(const std::string& list) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketCallList(list));
  }
   
  void ViewerCanvas::putVisualMatchables(const VisualMatchablefVector& matchables_) {
    if (!_setup())
      return;
    _serializer.putPacket(PacketVisualMatchablefVector(&matchables_));
  }

  // ds FIXME
  void ViewerCanvas::putImage(const cv::Mat& image_) {
    if (!_setup()) {
      return;
    }
    _serializer.putPacket(PacketCvMat(image_));
  }

  // ia TODO workaround
  void ViewerCanvas::putPoints(const PointIntensity3fVectorCloud& points_) {
    if (!_setup()) {
      return;
    }

    // ia copy everything in a normal vector cloud
    Point3fVectorCloud copy;
    copy.reserve(points_.size());

    for (const auto& p : points_) {
      Point3f copy_p;
      copy_p.coordinates() = p.coordinates();
      copy.emplace_back(copy_p);
    }

    _serializer.putPacket(PacketPoint3fVectorCloud(&copy));
  }

  // ia TODO workaround
  void ViewerCanvas::putPoints(const PointNormalCurvatureColor3fVectorCloud& points_) {
    if (!_setup()) {
      return;
    }

    // ia copy everything in a normal vector cloud
    PointNormalColor3fVectorCloud copy;
    copy.reserve(points_.size());

    for (const auto& p : points_) {
      PointNormalColor3f copy_p;
      copy_p.coordinates() = p.coordinates();
      copy_p.normal()      = p.normal();
      copy_p.color()       = p.color();
      copy.emplace_back(copy_p);
    }

    // ia packet
    _serializer.putPacket(PacketPointNormalColor3fVectorCloud(&copy));
  }
  void ViewerCanvas::putPoints(const PointNormalCurvature3fVectorCloud& points_) {
    if (!_setup()) {
      return;
    }

    // ia copy everything in a normal vector cloud
    PointNormalColor3fVectorCloud copy;
    copy.reserve(points_.size());
    for (const auto& p : points_) {
      PointNormalColor3f copy_p;
      Vector3f color       = Vector3f::Zero();
      copy_p.coordinates() = p.coordinates();
      copy_p.normal()      = p.normal();
      if (p.curvature() < 0.25) {
        color = ColorPalette::color3fWhite();
      } else if (p.curvature() < 0.75) {
        color = ColorPalette::color3fLightGray();
      } else {
        color = ColorPalette::color3fBlack();
      }
      copy_p.color() = color;
      copy.emplace_back(copy_p);
    }

    _serializer.putPacket(PacketPointNormalColor3fVectorCloud(&copy));
  }

  // ia TODO workaround
  void ViewerCanvas::putPoints(const PointIntensity3fMatrixCloud& points_) {
    if (!_setup()) {
      return;
    }

    // ia copy everything in a normal vector cloud
    Point3fVectorCloud copy;
    copy.reserve(points_.size());

    for (auto it = points_.begin(); it != points_.end(); ++it) {
      if (it->status == POINT_STATUS::Invalid) {
        continue;
      }
      Point3f copy_p;
      copy_p.coordinates() = it->coordinates();
      copy.emplace_back(copy_p);
    }

    _serializer.putPacket(PacketPoint3fVectorCloud(&copy));
  }

  // ia TODO workaround
  void ViewerCanvas::putPoints(const PointNormalCurvatureColor3fMatrixCloud& points_) {
    if (!_setup()) {
      return;
    }
    // ia copy everything in a normal vector cloud
    PointNormalColor3fVectorCloud copy;
    copy.reserve(points_.size());

    for (auto it = points_.begin(); it != points_.end(); ++it) {
      PointNormalColor3f copy_p;
      copy_p.coordinates() = it->coordinates();
      copy_p.normal()      = it->normal();
      copy_p.color()       = it->color();
      copy.emplace_back(copy_p);
    }
    _serializer.putPacket(PacketPointNormalColor3fVectorCloud(&copy));
  }

  void ViewerCanvas::putPoints(const PointNormalCurvature3fMatrixCloud& points_) {
    if (!_setup()) {
      return;
    }

    // ia copy everything in a normal vector cloud
    PointNormalColor3fVectorCloud copy;
    copy.reserve(points_.size());
    for (const auto& p : points_) {
      if (p.status == POINT_STATUS::Invalid) {
        continue;
      }
      PointNormalColor3f copy_p;
      Vector3f color       = Vector3f::Zero();
      copy_p.coordinates() = p.coordinates();
      copy_p.normal()      = p.normal();
      if (p.curvature() < 0.25) {
        color = ColorPalette::color3fWhite();
      } else if (p.curvature() < 0.75) {
        color = ColorPalette::color3fLightGray();
      } else {
        color = ColorPalette::color3fBlack();
      }
      copy_p.color() = color;
      copy.emplace_back(copy_p);
    }

    _serializer.putPacket(PacketPointNormalColor3fVectorCloud(&copy));
  }

  void ViewerCanvas::flush() {
    if (!_setup())
      return;

    // ia send an end epoch
    _serializer.putPacket(PacketInfoEndEpoch());

    // ia throw the buffer somewhere
    param_buffer_sink_ptr->putBuffer(_current_buffer);
    _current_buffer = 0;
  }

  bool ViewerCanvas::_setup() {
    // ia if nobody is attached to this canvas, simply do nothing (bench mode)
    if (!param_buffer_sink_ptr.value())
      return false;

    // ia check connection to the viewport (if its up again maybe)
    param_buffer_sink_ptr.value()->checkConnection(); // ia this check is heavy :(

    if (!param_buffer_sink_ptr->isActive())
      return false;

    if (!param_buffer_manager_ptr.value())
      throw std::runtime_error("ViewerCanvas::_setup|please set a valid BufferManager");

    // ia check if I have a buffer otherwise get one
    if (!_current_buffer) {
      _current_buffer = param_buffer_manager_ptr->getBuffer();
      _serializer.setBuffer(_current_buffer);
    }

    return true;
  }

} // namespace srrg2_core
