#pragma once
#include "srrg_pcl/point_types.h"
#include "viewer_core/buffer_manager.h"
#include "viewer_core/buffer_sink.h"
#include "viewer_core/packet_serializer.h"
#include "srrg_matchable/visual_matchable.h"
#include "srrg_pcl/point_types.h"

namespace srrg2_core {

  class ViewerCanvas : public Configurable {
  public:
    PARAM(PropertyConfigurable_<BufferSinkBase>,
          buffer_sink_ptr,
          "the place where to put buffers to be rendered",
          0,
          0);
    PARAM(PropertyConfigurable_<BufferManager>,
          buffer_manager_ptr,
          "buffer manager that gives us free buffer where we can write",
          0,
          0);

    ~ViewerCanvas();
    bool _setup();

    //! @brief payloads
    void putPoints(const int& size_, const Vector3f* points_, const Vector3f* normals_ = nullptr);
    void putLine(const int& size_, const Vector3f* points_, const Vector3f* normals_ = nullptr);
    void putSegment(const int& size_, const Vector3f* points_, const Vector3f* normals_ = nullptr);

    //! @brief vector of points - missing double and int
    void putPoints(const Point2fVectorCloud& points_);
    void putPoints(const PointNormal2fVectorCloud& points_);
    void putPoints(const PointNormalColor2fVectorCloud& points_);
    void putPoints(const PointIntensityDescriptor2fVectorCloud& points_);

    void putPoints(const Point3fVectorCloud& points_);
    void putPoints(const PointNormal3fVectorCloud& points_);
    // ia TODO WORKAROUND WITH HIDDEN COPY
    void putPoints(const PointNormalCurvature3fVectorCloud& points_);
    void putPoints(const PointNormalColor3fVectorCloud& points_);
    void putPoints(const PointIntensityDescriptor3fVectorCloud& points_);
    // ia TODO WORKAROUND WITH HIDDEN COPY
    void putPoints(const PointIntensity3fVectorCloud& points_);
    // ia TODO WORKAROUND WITH HIDDEN COPY
    void putPoints(const PointNormalCurvatureColor3fVectorCloud& points_);

    void putPoints(const Point4fVectorCloud& points_);
    void putPoints(const PointNormal4fVectorCloud& points_);
    void putPoints(const PointNormalColor4fVectorCloud& points_);

    //! @brief matrix of points - missing double and int
    void putPoints(const Point2fMatrixCloud& points_);
    void putPoints(const PointNormal2fMatrixCloud& points_);
    void putPoints(const PointNormalColor2fMatrixCloud& points_);

    void putPoints(const Point3fMatrixCloud& points_);
    void putPoints(const PointNormal3fMatrixCloud& points_);
    void putPoints(const PointNormalColor3fMatrixCloud& points_);
    // ia TODO WORKAROUND WITH HIDDEN COPY
    void putPoints(const PointIntensity3fMatrixCloud& points_);
    // ia TODO WORKAROUND WITH HIDDEN COPY
    void putPoints(const PointNormalCurvatureColor3fMatrixCloud& points_);
    // ia TODO WORKAROUND WITH HIDDEN COPY
    void putPoints(const PointNormalCurvature3fMatrixCloud& points_);

    void putPoints(const Point4fMatrixCloud& points_);
    void putPoints(const PointNormal4fMatrixCloud& points_);
    void putPoints(const PointNormalColor4fMatrixCloud& points_);

    void putVisualMatchables(const VisualMatchablefVector& matchables_);

    void putPolygon(const PointNormalColor3fVectorCloud& points_);
    void putPolygonWireframe(const PointNormalColor3fVectorCloud& points_);

    //! @brief objects
    void putPlane(const Vector2f& size_);
    void putBox(const Vector3f& size_);
    void putBoxWireframe(const Vector3f& size_);
    void putSphere(const float& radius_);
    void putPyramid(const Vector2f& size_);
    void putPyramidWireframe(const Vector2f& size_);
    void putEllipsoid(const Vector3f& size_);
    void putCone(const Vector2f& size_);
    void putDisk(const float& radius_);
    void putCylinder(const Vector2f& size_);
    void putArrow2D(const Vector3f& size_);
    void putReferenceSystem(const float& rf_linewidth);

    void putText(const std::string& text_);

    void multMatrix(const Matrix4f& m);
    void scale(const Vector3f& scales);
    void rotate(const Matrix3f& rotation_matrix);
    void translate(const Vector3f& translation);

    void createList(const std::string& list);
    void beginList(const std::string& list);
    void endList();
    void destroyList(const std::string& list);
    void callList(const std::string& list);
    
    //! @brief commands
    void pushMatrix();
    void popMatrix();
    void pushColor();
    void pushPointSize();
    void pushLineWidth();
    void popAttribute();

    //! @brief attributes
    void setPointSize(const float& point_size_);
    void setLineWidth(const float& line_with_);

    //! @brief color handling
    void setColor(const Vector3f& color_);
    void setColor(const Vector4f& color_);

    //    //! clear active canvas
    //     void clearBuffer(const Vector3f& background_color_,
    //                             const float alpha_ = 0);

    // ds FIXME
    //! @brief opencv raster image drawing
    void putImage(const cv::Mat& image_);

    void flush();

  protected:
    BufferMemory* _current_buffer = nullptr;
    PacketSerializer _serializer;

  public:
    friend class ViewerManagerShared;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  using ViewerCanvasPtr = std::shared_ptr<ViewerCanvas>;
} // namespace srrg2_core
