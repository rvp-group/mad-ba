#include "qgl_packet_processor.h"
#include "gl_helpers.h"
// I will also include this shit here otherwise bad things will happen in my IDE
// (hey Emacs! how are you?)
#include <srrg_viewer/viewer_core/packet_processor.h>
#include <srrg_viewer/viewer_core/packets.h>

namespace srrg2_qgl_viewport {

  using namespace srrg2_core;
  static constexpr int max_gl_lists = 128;

  QGLPacketProcessor::CallbackBase::~CallbackBase() {
  }

#define INSTALL_HANDLER(__PACKET_TYPE__, __METHOD_TYPE__)     \
  _callbacks[__PACKET_TYPE__ ::PacketType] = CallbackBasePtr( \
    new Callback_<__PACKET_TYPE__, (&QGLPacketProcessor::__METHOD_TYPE__)>(*this));

#define INSTALL_CMD_HANDLER(__PACKET_NUM__, __METHOD_TYPE__) \
  _callbacks[__PACKET_NUM__] = CallbackBasePtr(              \
    new CallbackCmd_<__PACKET_NUM__, (&QGLPacketProcessor::__METHOD_TYPE__)>(*this));

  QGLPacketProcessor::QGLPacketProcessor(CvMatVector* cv_vector_ = nullptr) :
    PacketProcessorBase::PacketProcessorBase() {
    assert(cv_vector_ && "QGLPacketProcessor::QGLPacketProcessor|invalid cv vector");
    _cv_mat_vector        = cv_vector_;
    GLuint start_list_key = glGenLists(max_gl_lists);
    for (int i = 0; i < max_gl_lists; ++i) {
      _gl_list_keys.push_back(i + start_list_key);
    }
    std::cerr << "creating gl_lists, start: " << start_list_key << std::endl;
    _callbacks.resize(256);

    INSTALL_HANDLER(PacketPayloadPoints, _drawPoints);
    INSTALL_HANDLER(PacketPayloadLines, _drawLines);
    INSTALL_HANDLER(PacketPayloadSegments, _drawSegments);

    INSTALL_HANDLER(PacketAttributeColorRGBA, _setAttributeColorRGBA);
    INSTALL_HANDLER(PacketAttributeColorRGB, _setAttributeColorRGB);
    INSTALL_HANDLER(PacketAttributePointSize, _setAttributePointSize);
    INSTALL_HANDLER(PacketAttributeLineWidth, _setAttributeLineWidth);
    INSTALL_CMD_HANDLER(PACKET_TYPE_PUSH_MATRIX, _pushMatrix);
    INSTALL_CMD_HANDLER(PACKET_TYPE_POP_MATRIX, _popMatrix);
    INSTALL_CMD_HANDLER(PACKET_TYPE_PUSH_COLOR, _pushColor);
    INSTALL_CMD_HANDLER(PACKET_TYPE_PUSH_POINT_SIZE, _pushPointSize);
    INSTALL_CMD_HANDLER(PACKET_TYPE_PUSH_LINE_WIDTH, _pushLineWidth);
    INSTALL_CMD_HANDLER(PACKET_TYPE_POP_ATTRIBUTE, _popAttribute);
    INSTALL_HANDLER(PacketCreateList, _createList);
    INSTALL_HANDLER(PacketBeginList, _beginList);
    INSTALL_HANDLER(PacketCallList, _callList);
    INSTALL_HANDLER(PacketDestroyList, _destroyList);
    INSTALL_CMD_HANDLER(PACKET_TYPE_END_LIST, _endList);
    INSTALL_HANDLER(PacketObjectPlane, _drawPlane);
    INSTALL_HANDLER(PacketObjectBox, _drawBox);
    INSTALL_HANDLER(PacketObjectBoxWireframe, _drawBoxWireframe);
    INSTALL_HANDLER(PacketObjectSphere, _drawSphere);
    INSTALL_HANDLER(PacketObjectDisk, _drawDisk);
    INSTALL_HANDLER(PacketObjectPyramid, _drawPyramid);
    INSTALL_HANDLER(PacketObjectPyramidWireframe, _drawPyramidWireframe);
    INSTALL_HANDLER(PacketObjectEllipsoid, _drawEllipsoid);
    INSTALL_HANDLER(PacketObjectCone, _drawCone);
    INSTALL_HANDLER(PacketObjectCylinder, _drawCylinder);
    INSTALL_HANDLER(PacketObjectArrow2D, _drawArrow2D);
    INSTALL_HANDLER(PacketObjectReferenceFrame, _drawReferenceFrame);
    INSTALL_HANDLER(PacketObjectText, _drawText);
    INSTALL_HANDLER(PacketTransformMultTraslation, _multTranslation);
    INSTALL_HANDLER(PacketTransformMultScale, _multScale);
    INSTALL_HANDLER(PacketTransformMultRotation, _multRotation);
    INSTALL_HANDLER(PacketTransformMultMatrix, _multMatrix);
    INSTALL_HANDLER(PacketPoint2fVectorCloud, _drawPointVector2f);
    INSTALL_HANDLER(PacketPointNormal2fVectorCloud, _drawPointVector2f);
    INSTALL_HANDLER(PacketPointNormalColor2fVectorCloud, _drawPointVector2f);
    INSTALL_HANDLER(PacketPoint3fVectorCloud, _drawPointVector3f);
    INSTALL_HANDLER(PacketPointNormal3fVectorCloud, _drawPointVector3f);
    INSTALL_HANDLER(PacketPointNormalColor3fVectorCloud, _drawPointVector3f);
    INSTALL_HANDLER(PacketPointNormalCurvature3fVectorCloud, _drawPointVector3f);
    INSTALL_HANDLER(PacketPoint4fVectorCloud, _drawPointVector4f);
    INSTALL_HANDLER(PacketPointNormal4fVectorCloud, _drawPointVector4f);
    INSTALL_HANDLER(PacketPointNormalColor4fVectorCloud, _drawPointVector4f);
    INSTALL_HANDLER(PacketPoint2fMatrixCloud, _drawPointMatrix2f);
    INSTALL_HANDLER(PacketPointNormal2fMatrixCloud, _drawPointMatrix2f);
    INSTALL_HANDLER(PacketPointNormalColor2fMatrixCloud, _drawPointMatrix2f);
    INSTALL_HANDLER(PacketPoint3fMatrixCloud, _drawPointMatrix3f);
    INSTALL_HANDLER(PacketPointNormal3fMatrixCloud, _drawPointMatrix3f);
    INSTALL_HANDLER(PacketPointNormalColor3fMatrixCloud, _drawPointMatrix3f);
    INSTALL_HANDLER(PacketPoint4fMatrixCloud, _drawPointMatrix4f);
    INSTALL_HANDLER(PacketPointNormal4fMatrixCloud, _drawPointMatrix4f);
    INSTALL_HANDLER(PacketPointNormalColor4fMatrixCloud, _drawPointMatrix4f);
    INSTALL_HANDLER(PacketPointIntensityDescriptor2fVectorCloud, _drawPointVector2f);
    INSTALL_HANDLER(PacketPointIntensityDescriptor3fVectorCloud, _drawPointVector3f);
    INSTALL_HANDLER(PacketPolygonPointNormalColor3fVectorCloud, _drawPolygon);
    INSTALL_HANDLER(PacketVisualMatchablefVector, _drawVisualMatchables);
    INSTALL_HANDLER(PacketCvMat, _drawCvMat);
  }

#undef INSTALL_HANDLER

  QGLPacketProcessor::~QGLPacketProcessor() {
  }

  QGLPacketProcessor::GLListInfo* QGLPacketProcessor::glListByNum(unsigned int num) {
    for (auto& it : _gl_lists) {
      if (it.second.listNum() == num)
        return &it.second;
    }
    return nullptr;
  }

  void QGLPacketProcessor::processPacket(PacketBase* packet_) {
    if (packet_->type == PACKET_TYPE_END_EPOCH)
      return;
    CallbackBasePtr& cb = _callbacks[packet_->type];
    if (!cb) {
      std::printf("QGLPacketProcessor::processPacket|unknown packet [%02X]\n", packet_->type);
      throw std::runtime_error("QGLPacketProcessor::processPacket|maybe you forgot to define the "
                               "function to process it?");
    }
    (*cb)(packet_);
  }

  void QGLPacketProcessor::_drawPoints(PacketPayloadPoints* packet_) {
    if (!_process_payloads)
      return;
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_points; ++i) {
      const srrg2_core::Vector3f& p = packet_->points[i];
      glVertex3f(p.x(), p.y(), p.z());
    }
    glEnd();

    // ia normals
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_normals; ++i) {
      const srrg2_core::Vector3f& p = packet_->points[i];
      const srrg2_core::Vector3f n  = packet_->normals[i] * _normal_scale;
      glVertex3f(p.x(), p.y(), p.z());
      glVertex3f(p.x() + n.x(), p.y() + n.y(), p.z() + n.z());
      glNormal3f(n.x(), n.y(), n.z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawLines(PacketPayloadLines* packet_) {
    if (!_process_payloads)
      return;
    glBegin(GL_LINES);
    srrg2_core::Vector3f* prev_p = 0;
    for (size_t i = 0; i < packet_->num_points; ++i) {
      srrg2_core::Vector3f& p = packet_->points[i];

      if (prev_p) {
        glVertex3f(prev_p->x(), prev_p->y(), prev_p->z());
        glVertex3f(p.x(), p.y(), p.z());
      }

      prev_p = &p;
    }

    // ia normals
    for (size_t i = 0; i < packet_->num_normals; ++i) {
      const srrg2_core::Vector3f& p = packet_->points[i];
      const srrg2_core::Vector3f n  = packet_->normals[i] * _normal_scale;
      glVertex3f(p.x(), p.y(), p.z());
      glVertex3f(p.x() + n.x(), p.y() + n.y(), p.z() + n.z());
      glNormal3f(n.x(), n.y(), n.z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawSegments(PacketPayloadSegments* packet_) {
    if (!_process_payloads)
      return;

    glBegin(GL_LINES);
    // ia segments
    for (size_t i = 0; i < packet_->num_points;) {
      const srrg2_core::Vector3f& p0 = packet_->points[i++];
      const srrg2_core::Vector3f& p1 = packet_->points[i++];

      glVertex3f(p0.x(), p0.y(), p0.z());
      glVertex3f(p1.x(), p1.y(), p1.z());
    }

    // ia normals
    for (size_t i = 0; i < packet_->num_normals; ++i) {
      const srrg2_core::Vector3f& p = packet_->points[i];
      const srrg2_core::Vector3f n  = packet_->normals[i] * _normal_scale;
      glVertex3f(p.x(), p.y(), p.z());
      glVertex3f(p.x() + n.x(), p.y() + n.y(), p.z() + n.z());
      glNormal3f(n.x(), n.y(), n.z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_setAttributeColorRGB(PacketAttributeColorRGB* packet_color_rgb_) {
    glColor3f(packet_color_rgb_->data[0], packet_color_rgb_->data[1], packet_color_rgb_->data[2]);
  }

  void QGLPacketProcessor::_setAttributeColorRGBA(PacketAttributeColorRGBA* packet_color_rgba_) {
    glColor4f(packet_color_rgba_->data[0],
              packet_color_rgba_->data[1],
              packet_color_rgba_->data[2],
              packet_color_rgba_->data[3]);
  }

  void QGLPacketProcessor::_setAttributePointSize(PacketAttributePointSize* packet_point_size_) {
    glPointSize(packet_point_size_->data);
  }

  void QGLPacketProcessor::_setAttributeLineWidth(PacketAttributeLineWidth* packet_line_width_) {
    glLineWidth(packet_line_width_->data);
  }

  void QGLPacketProcessor::_pushMatrix() {
    glPushMatrix();
  }

  void QGLPacketProcessor::_popMatrix() {
    glPopMatrix();
  }

  void QGLPacketProcessor::_pushColor() {
    glPushAttrib(GL_CURRENT_BIT);
  }

  void QGLPacketProcessor::_pushPointSize() {
    glPushAttrib(GL_POINT_BIT);
  }

  void QGLPacketProcessor::_pushLineWidth() {
    glPushAttrib(GL_LINE_BIT);
  }

  void QGLPacketProcessor::_popAttribute() {
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPlane(PacketObjectPlane* packet_) {
    if (!_process_objects)
      return;

    drawPlane(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawBox(PacketObjectBox* packet_) {
    if (!_process_objects)
      return;
    drawBox(packet_->data[0], packet_->data[1], packet_->data[2]);
  }
  void QGLPacketProcessor::_drawBoxWireframe(PacketObjectBoxWireframe* packet_) {
    if (!_process_objects)
      return;
    drawBoxWireframe(packet_->data[0], packet_->data[1], packet_->data[2]);
  }

  void QGLPacketProcessor::_drawSphere(PacketObjectSphere* packet_) {
    if (!_process_objects)
      return;
    drawSphere(packet_->data);
  }

  void QGLPacketProcessor::_drawPyramid(PacketObjectPyramid* packet_) {
    if (!_process_objects)
      return;
    drawPyramid(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawPyramidWireframe(PacketObjectPyramidWireframe* packet_) {
    if (!_process_objects)
      return;
    drawPyramidWireframe(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawReferenceFrame(PacketObjectReferenceFrame* packet_) {
    if (!_process_objects)
      return;
    drawReferenceSystem(packet_->data);
  }

  void QGLPacketProcessor::_drawText(srrg2_core::PacketObjectText* packet_) {
    if (!_process_objects)
      return;
    drawText(packet_->data);
  }

  void QGLPacketProcessor::_drawEllipsoid(PacketObjectEllipsoid* packet_) {
    if (!_process_objects)
      return;
    drawEllipsoid(packet_->data[0], packet_->data[1], packet_->data[2]);
  }

  void QGLPacketProcessor::_drawCone(PacketObjectCone* packet_) {
    if (!_process_objects)
      return;
    drawCone(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawDisk(PacketObjectDisk* packet_) {
    if (!_process_objects)
      return;
    drawDisk(packet_->data);
  }

  void QGLPacketProcessor::_drawCylinder(PacketObjectCylinder* packet_) {
    if (!_process_objects)
      return;
    drawCylinder(packet_->data[0], packet_->data[1]);
  }

  void QGLPacketProcessor::_drawArrow2D(PacketObjectArrow2D* packet_) {
    if (!_process_objects)
      return;
    drawArrow2D(packet_->data[0], packet_->data[1], packet_->data[2]);
  }

  void QGLPacketProcessor::_multTranslation(PacketTransformMultTraslation* packet_) {
    Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
    iso.translation()     = packet_->data;
    glMultMatrix(iso);
  }

  void QGLPacketProcessor::_multScale(PacketTransformMultScale* packet_) {
  }

  void QGLPacketProcessor::_multRotation(PacketTransformMultRotation* packet_) {
    Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
    iso.linear()          = packet_->data;
    glMultMatrix(iso);
  }

  void QGLPacketProcessor::_multMatrix(PacketTransformMultMatrix* packet_) {
    glMultMatrixf(packet_->data.data());
  }

  void QGLPacketProcessor::_drawPointVector2f(srrg2_core::PacketPoint2fVectorCloud* packet_) {
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point2f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector2f(srrg2_core::PacketPointNormal2fVectorCloud* packet_) {
    // ia points
    if (!_process_point_vectors)
      return;

    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal2f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glNormal3f(p.normal().x(), p.normal().y(), 0.0);
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal2f& p = packet_->data_vector->at(i);
      const Vector2f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glVertex3f(p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), 0.0);
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointVector2f(srrg2_core::PacketPointNormalColor2fVectorCloud* packet_) {
    // ia points
    if (!_process_point_vectors)
      return;
    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor2f& p = packet_->data_vector->at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glNormal3f(p.normal().x(), p.normal().y(), 0.0);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor2f& p = packet_->data_vector->at(i);
      const Vector2f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glVertex3f(p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), 0.0);
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPointVector3f(srrg2_core::PacketPoint3fVectorCloud* packet_) {
    if (!_process_point_vectors)
      return;

    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point3f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector3f(srrg2_core::PacketPointNormal3fVectorCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal3f& p = packet_->data_vector->at(i);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal3f& p = packet_->data_vector->at(i);
      const Vector3f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector3f(
    srrg2_core::PacketPointNormalCurvature3fVectorCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalCurvature3f& p = packet_->data_vector->at(i);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalCurvature3f& p = packet_->data_vector->at(i);
      const Vector3f n                = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointVector3f(srrg2_core::PacketPointNormalColor3fVectorCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor3f& p = packet_->data_vector->at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;
    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor3f& p = packet_->data_vector->at(i);
      const Vector3f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPointVector4f(srrg2_core::PacketPoint4fVectorCloud* packet_) {
    if (!_process_point_vectors)
      return;

    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point4f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector4f(srrg2_core::PacketPointNormal4fVectorCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal4f& p = packet_->data_vector->at(i);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;

    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormal4f& p = packet_->data_vector->at(i);
      const Vector4f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointVector4f(srrg2_core::PacketPointNormalColor4fVectorCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor4f& p = packet_->data_vector->at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const PointNormalColor4f& p = packet_->data_vector->at(i);
      const Vector4f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
    glPopAttrib();
  }

  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  void QGLPacketProcessor::_drawPointMatrix2f(srrg2_core::PacketPoint2fMatrixCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const Point2f& p = vector_data.at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointMatrix2f(srrg2_core::PacketPointNormal2fMatrixCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal2f& p = vector_data.at(i);
      glNormal3f(p.normal().x(), p.normal().y(), 0.0);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal2f& p = vector_data.at(i);
      const Vector2f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glVertex3f(p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), 0.0);
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointMatrix2f(srrg2_core::PacketPointNormalColor2fMatrixCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();
    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor2f& p = vector_data.at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glNormal3f(p.normal().x(), p.normal().y(), 0.0);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor2f& p = vector_data.at(i);
      const Vector2f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
      glVertex3f(p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), 0.0);
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPointMatrix3f(srrg2_core::PacketPoint3fMatrixCloud* packet_) {
    if (!_process_point_vectors)
      return;

    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const Point3f& p = vector_data.at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointMatrix3f(srrg2_core::PacketPointNormal3fMatrixCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal3f& p = vector_data.at(i);

      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal3f& p = vector_data.at(i);
      const Vector3f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointMatrix3f(srrg2_core::PacketPointNormalColor3fMatrixCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor3f& p = vector_data.at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor3f& p = vector_data.at(i);
      const Vector3f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPointMatrix4f(srrg2_core::PacketPoint4fMatrixCloud* packet_) {
    if (!_process_point_vectors)
      return;

    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const Point4f& p = vector_data.at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointMatrix4f(srrg2_core::PacketPointNormal4fMatrixCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal4f& p = vector_data.at(i);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();

    // ia normals
    if (!_show_normal)
      return;
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormal4f& p = vector_data.at(i);
      const Vector4f n       = p.normal() * _normal_scale;

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawPointMatrix4f(srrg2_core::PacketPointNormalColor4fMatrixCloud* packet_) {
    if (!_process_point_vectors)
      return;

    // ia points
    const size_t& num_points = packet_->data_matrix->size();
    const auto& vector_data  = packet_->data_matrix->data();

    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor4f& p = vector_data.at(i);
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
    glPopAttrib();

    // ia normals
    if (!_show_normal)
      return;

    glPushAttrib(GL_COLOR);
    glBegin(GL_LINES);
    for (size_t i = 0; i < num_points; ++i) {
      const PointNormalColor4f& p = vector_data.at(i);
      const Vector4f n            = p.normal() * _normal_scale;
      const Vector3f& p_color     = p.color();
      glColor3f(p_color[0], p_color[1], p_color[2]);

      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
      glVertex3f(
        p.coordinates().x() + n.x(), p.coordinates().y() + n.y(), p.coordinates().z() + n.z());
    }
    glEnd();
    glPopAttrib();
  }

  void QGLPacketProcessor::_drawPolygonWireframe(
    srrg2_core::PacketPolygonWireframePointNormalColor3fVectorCloud* packet_) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_POLYGON);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const auto& p = packet_->data_vector->at(i);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
  void QGLPacketProcessor::_drawPolygon(
    srrg2_core::PacketPolygonPointNormalColor3fVectorCloud* packet_) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_POLYGON);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const auto& p = packet_->data_vector->at(i);
      glNormal3f(p.normal().x(), p.normal().y(), p.normal().z());
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void QGLPacketProcessor::_drawPointVector2f(
    srrg2_core::PacketPointIntensityDescriptor2fVectorCloud* packet_) {
    // ia points
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point2f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), 0.0);
    }
    glEnd();
  }
  void QGLPacketProcessor::_drawPointVector3f(
    srrg2_core::PacketPointIntensityDescriptor3fVectorCloud* packet_) {
    glBegin(GL_POINTS);
    for (size_t i = 0; i < packet_->num_elements; ++i) {
      const Point3f& p = packet_->data_vector->at(i);
      glVertex3f(p.coordinates().x(), p.coordinates().y(), p.coordinates().z());
    }
    glEnd();
  }

  void
  QGLPacketProcessor::_drawVisualMatchables(srrg2_core::PacketVisualMatchablefVector* packet_) {
    // ia group matchables by type
    std::vector<size_t> point_index_vector;
    std::vector<size_t> line_index_vector;
    std::vector<size_t> plane_index_vector;

    point_index_vector.reserve(packet_->num_elements);
    line_index_vector.reserve(packet_->num_elements);
    plane_index_vector.reserve(packet_->num_elements);

    for (size_t m = 0; m < packet_->num_elements; ++m) {
      const auto& matchable = packet_->data_vector->at(m);
      switch (matchable.type()) {
        case MatchableBase::Point:
          point_index_vector.emplace_back(m);
          break;
        case MatchableBase::Line:
          line_index_vector.emplace_back(m);
          break;
        case MatchableBase::Plane:
          plane_index_vector.emplace_back(m);
          break;
        default:
          break;
      }
    }

    // ia start visualizing points
    glPushAttrib(GL_CURRENT_BIT);
    glBegin(GL_POINTS);
    for (const size_t& p : point_index_vector) {
      const auto& m = packet_->data_vector->at(p);
      glColor3f(m.color().x(), m.color().y(), m.color().z());
      glVertex3f(m.origin().x(), m.origin().y(), m.origin().z());
    }
    glEnd();
    glPopAttrib();

    // ia visualizing lines
    glPushAttrib(GL_CURRENT_BIT);
    glBegin(GL_LINES);
    for (const size_t& p : line_index_vector) {
      const auto& m = packet_->data_vector->at(p);
      assert(m.support().size() &&
             "QGLPacketProcessor::_drawVisualMatchables|ERROR, no line support");
      const auto& p_start = m.support()[0].coordinates();
      const auto& p_end   = m.support()[1].coordinates();
      glColor3f(m.color().x(), m.color().y(), m.color().z());
      glVertex3f(p_start.x(), p_start.y(), p_start.z());
      glVertex3f(p_end.x(), p_end.y(), p_end.z());
    }
    glEnd();
    glPopAttrib();

    // ia visualizing planes
    glPushAttrib(GL_CURRENT_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for (const size_t& p : plane_index_vector) {
      const auto& m = packet_->data_vector->at(p);
      assert(m.support().size() &&
             "QGLPacketProcessor::_drawVisualMatchables|ERROR, no plane support");
      glBegin(GL_POLYGON);
      glColor3f(m.color().x(), m.color().y(), m.color().z());
      for (const auto& support_point : m.support()) {
        glVertex3f(support_point.coordinates().x(),
                   support_point.coordinates().y(),
                   support_point.coordinates().z());
      }
      glEnd();
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPopAttrib();
  }

  void QGLPacketProcessor::_createList(srrg2_core::PacketCreateList* packet) {
    const std::string& list_name = packet->data;
    if (_gl_lists.count(list_name)) {
      std::cerr << "gl_list named[" << list_name << "] exists already" << std::endl;
      return;
    }
    if (_gl_list_keys.empty()) {
      throw std::runtime_error("gl_list named[" + list_name + "] no lists available");
    }
    auto list_num = _gl_list_keys.front();
    _gl_list_keys.pop_front();
    _gl_lists[list_name] = GLListInfo(list_num);
  }

  void QGLPacketProcessor::_beginList(srrg2_core::PacketBeginList* packet) {
    const std::string& list_name = packet->data;
    auto it                      = _gl_lists.find(list_name);
    if (it == _gl_lists.end()) {
      throw std::runtime_error("no gl_list named[" + list_name + "] found");
    }
    auto list_num = it->second.listNum();
    glNewList(list_num, GL_COMPILE);
  }

  void QGLPacketProcessor::_endList() {
    glEndList();
  }

  void QGLPacketProcessor::_callList(srrg2_core::PacketCallList* packet) {
    const std::string& list_name = packet->data;
    auto it                      = _gl_lists.find(list_name);
    if (it == _gl_lists.end()) {
      throw std::runtime_error("no gl_list named[" + list_name + "] found");
    }
    auto list_num = it->second.listNum();
    if (!it->second.shown) {
      return;
    }
    if (glIsList(list_num) != GL_TRUE) {
      throw std::runtime_error("no gl_list named[" + list_name + "] in gl context");
    }
    glCallList(list_num);
  }

  void QGLPacketProcessor::_destroyList(srrg2_core::PacketDestroyList* packet) {
    const std::string& list_name = packet->data;
    auto it                      = _gl_lists.find(list_name);
    if (it == _gl_lists.end()) {
      throw std::runtime_error("no gl_list named[" + list_name + "] found");
    }
    auto list_num = it->second.listNum();
    if (glIsList(list_num) != GL_TRUE) {
      throw std::runtime_error("no gl_list named[" + list_name + "] in gl context");
    }
    _gl_list_keys.push_back(list_num);
    _gl_lists.erase(it);
  }

  void QGLPacketProcessor::_drawCvMat(srrg2_core::PacketCvMat* image_packet) {
    assert(image_packet && &image_packet->data && _cv_mat_vector &&
           "QGLPacketProcessor::processPacket|invalid cv something");
    _cv_mat_vector->push_back(image_packet->data);
  }

} // namespace srrg2_qgl_viewport
