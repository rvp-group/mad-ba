#pragma once
#include <list>
#include <srrg_viewer/viewer_core/packet_processor.h>
#include <srrg_viewer/viewer_core/packets.h>

namespace srrg2_qgl_viewport {

  class QGLPacketProcessor : public srrg2_core::PacketProcessorBase {
  public:
    using CvMatVector = std::vector<cv::Mat>;

    QGLPacketProcessor(CvMatVector* cv_vector_);
    virtual ~QGLPacketProcessor();

    void processPacket(srrg2_core::PacketBase* packet_) override;

    struct GLListInfo {
      GLListInfo(unsigned int gl_list_num_ = 0) : _gl_list_num(gl_list_num_) {
      }
      bool shown = true;
      inline unsigned int listNum() const {
        return _gl_list_num;
      }

    protected:
      unsigned int _gl_list_num;
    };
    std::map<std::string, GLListInfo>& glLists() {
      return _gl_lists;
    }
    const std::map<std::string, GLListInfo>& glLists() const {
      return _gl_lists;
    }
    GLListInfo* glListByNum(unsigned int num);

  protected:
    // ia all a=of those must be static and in construction we populate an
    // unordered map ia with [packet_type|static-function-that-handles-it].
    // otherwise we generate a super fragmented asm ia here is hidden all the
    // trash. if you need to do something you must add it here
    void _drawPoints(srrg2_core::PacketPayloadPoints* packet_);
    void _drawLines(srrg2_core::PacketPayloadLines* packet_);
    void _drawSegments(srrg2_core::PacketPayloadSegments* packet_);

    void _drawPointVector2f(srrg2_core::PacketPoint2fVectorCloud* packet_);
    void _drawPointVector2f(srrg2_core::PacketPointNormal2fVectorCloud* packet_);
    void _drawPointVector2f(srrg2_core::PacketPointNormalColor2fVectorCloud* packet_);

    void _drawPointVector3f(srrg2_core::PacketPoint3fVectorCloud* packet_);
    void _drawPointVector3f(srrg2_core::PacketPointNormal3fVectorCloud* packet_);
    void _drawPointVector3f(srrg2_core::PacketPointNormalColor3fVectorCloud* packet_);
    void _drawPointVector3f(srrg2_core::PacketPointNormalCurvature3fVectorCloud* packet_);

    void _drawPointVector4f(srrg2_core::PacketPoint4fVectorCloud* packet_);
    void _drawPointVector4f(srrg2_core::PacketPointNormal4fVectorCloud* packet_);
    void _drawPointVector4f(srrg2_core::PacketPointNormalColor4fVectorCloud* packet_);

    void _drawPointMatrix2f(srrg2_core::PacketPoint2fMatrixCloud* packet_);
    void _drawPointMatrix2f(srrg2_core::PacketPointNormal2fMatrixCloud* packet_);
    void _drawPointMatrix2f(srrg2_core::PacketPointNormalColor2fMatrixCloud* packet_);

    void _drawPointMatrix3f(srrg2_core::PacketPoint3fMatrixCloud* packet_);
    void _drawPointMatrix3f(srrg2_core::PacketPointNormal3fMatrixCloud* packet_);
    void _drawPointMatrix3f(srrg2_core::PacketPointNormalColor3fMatrixCloud* packet_);

    void _drawPointMatrix4f(srrg2_core::PacketPoint4fMatrixCloud* packet_);
    void _drawPointMatrix4f(srrg2_core::PacketPointNormal4fMatrixCloud* packet_);
    void _drawPointMatrix4f(srrg2_core::PacketPointNormalColor4fMatrixCloud* packet_);

    void _drawPointVector2f(srrg2_core::PacketPointIntensityDescriptor2fVectorCloud* packet_);
    void _drawPointVector3f(srrg2_core::PacketPointIntensityDescriptor3fVectorCloud* packet_);

    void _drawVisualMatchables(srrg2_core::PacketVisualMatchablefVector* packet_);

    void
    _drawPolygonWireframe(srrg2_core::PacketPolygonWireframePointNormalColor3fVectorCloud* packet_);
    void _drawPolygon(srrg2_core::PacketPolygonPointNormalColor3fVectorCloud* packet_);

    void _setAttributeColorRGB(srrg2_core::PacketAttributeColorRGB* packet_);
    void _setAttributeColorRGBA(srrg2_core::PacketAttributeColorRGBA* packet_);
    void _setAttributePointSize(srrg2_core::PacketAttributePointSize* packet_);
    void _setAttributeLineWidth(srrg2_core::PacketAttributeLineWidth* packet_);

    void _drawPlane(srrg2_core::PacketObjectPlane* packet_);
    void _drawBox(srrg2_core::PacketObjectBox* packet_);
    void _drawBoxWireframe(srrg2_core::PacketObjectBoxWireframe* packet_);
    void _drawSphere(srrg2_core::PacketObjectSphere* packet_);
    void _drawPyramid(srrg2_core::PacketObjectPyramid* packet_);
    void _drawPyramidWireframe(srrg2_core::PacketObjectPyramidWireframe* packet_);
    void _drawEllipsoid(srrg2_core::PacketObjectEllipsoid* packet_);
    void _drawCone(srrg2_core::PacketObjectCone* packet_);
    void _drawDisk(srrg2_core::PacketObjectDisk* packet_);
    void _drawCylinder(srrg2_core::PacketObjectCylinder* packet_);
    void _drawArrow2D(srrg2_core::PacketObjectArrow2D* packet_);
    void _drawReferenceFrame(srrg2_core::PacketObjectReferenceFrame* packet_);
    void _drawText(srrg2_core::PacketObjectText* packet_);

    void _multTranslation(srrg2_core::PacketTransformMultTraslation* packet_);
    void _multScale(srrg2_core::PacketTransformMultScale* packet_);
    void _multRotation(srrg2_core::PacketTransformMultRotation* packet_);
    void _multMatrix(srrg2_core::PacketTransformMultMatrix* packet_);

    void _createList(srrg2_core::PacketCreateList* packet);
    void _beginList(srrg2_core::PacketBeginList* packet);
    void _endList();
    void _callList(srrg2_core::PacketCallList* packet);
    void _destroyList(srrg2_core::PacketDestroyList* packet);

    void _pushMatrix();
    void _popMatrix();
    void _pushColor();
    void _pushPointSize();
    void _pushLineWidth();
    void _popAttribute();
    void _drawCvMat(srrg2_core::PacketCvMat* image_packet);

  protected:
    //! @brief vector of cv::mat. at this stage we just copy the cv::Mat and then its rendering will
    //!        happen outsite the glLoop to avoid strange behaviours and deadlocks. all of this
    //!        comes at the cost of another copy of the payload (1 when creating the packet in the
    //!        SLAM thread + 1 when processing the packet in the VIEWER thread)
    CvMatVector* _cv_mat_vector = nullptr;

    struct CallbackBase {
      CallbackBase(QGLPacketProcessor& processor_) : _processor(processor_){};
      virtual void operator()(srrg2_core::PacketBase* packet) = 0;
      virtual ~CallbackBase();
      QGLPacketProcessor& _processor;
    };
    using CallbackBasePtr = std::unique_ptr<CallbackBase>;
    using CallbackVector  = std::vector<CallbackBasePtr>;

    template <typename Packet_, void (QGLPacketProcessor::*Method_)(Packet_*)>
    struct Callback_ : public CallbackBase {
      Callback_(QGLPacketProcessor& processor_) : CallbackBase(processor_) {
      }
      static inline constexpr unsigned char type() {
        return Packet_::PacketType;
      }
      virtual void operator()(srrg2_core::PacketBase* packet_) override {
        (_processor.*Method_)(reinterpret_cast<Packet_*>(packet_));
      }
    };

    template <uint8_t PacketNum_, void (QGLPacketProcessor::*Method_)()>
    struct CallbackCmd_ : public CallbackBase {
      CallbackCmd_(QGLPacketProcessor& processor_) : CallbackBase(processor_) {
      }
      static inline constexpr unsigned char type() {
        return PacketNum_;
      }
      virtual void operator()(srrg2_core::PacketBase* packet_) override {
        (_processor.*Method_)();
      }
    };

  public:
    std::map<std::string, GLListInfo> _gl_lists;
    std::list<unsigned int> _gl_list_keys;
    CallbackVector _callbacks;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_qgl_viewport
