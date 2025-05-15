#pragma once
#include <QGLViewer/qglviewer.h>
#include <QtGui/QKeyEvent>
#include <QtWidgets/QWidget>
#include <QtWidgets/qapplication.h>

#include <srrg_viewer/viewer_core/viewport.h>

#include "qgl_packet_processor.h"

namespace srrg2_qgl_viewport {

  using QApplicationPtr = std::shared_ptr<QApplication>;

  class QGLViewport : public QGLViewer, public srrg2_core::ViewportBase {
  public:
    //! @brief custom camera
    class StandardCamera : public qglviewer::Camera {
    public:
      StandardCamera() : _standard(true) {
      }

      inline qreal zNear() const {
        if (_standard) {
          return qreal(_z_near);
        } else {
          return Camera::zNear();
        }
      }

      inline qreal zFar() const {
        if (_standard) {
          return qreal(_z_far);
        } else {
          return Camera::zFar();
        }
      }

      inline bool standard() const {
        return _standard;
      }
      inline void setStandard(bool s) {
        _standard = s;
      }
      inline void setZNear(const float& z_near_) {
        _z_near = z_near_;
      }
      inline void setZFar(const float& z_far_) {
        _z_far = z_far_;
      }

    protected:
      bool _standard;
      float _z_near = 0.001f;
      float _z_far  = 10000.0f;
    };

    enum BackgroundMode { Dark = 0, Light = 1 };

    QGLViewport(QWidget* parent = 0);
    virtual ~QGLViewport();

    //! @brief override of the base viewport function
    bool isActive() const override {
      return QGLViewer::isVisible();
    }

    //! @brief override of the base viewport function
    void update() override;

    //! @brief calls the init function and sets up the viewport
    void init() override;

    //! @brief override of the QGLViewer draw function, renders the buffer
    void draw() override;

    //! @brief callback invoked by the application on new key event.
    //!        It saves the last event in a member variable
    void keyPressEvent(QKeyEvent* event_) override;

    //! @brief inline set-get methods that will be mostly replaced by
    //! srrg2_parameters
    inline StandardCamera* camera() const {
      return _camera;
    }

    inline void setQGLServer(QApplication* qapp_ptr_) {
      _qapp_ptr = qapp_ptr_;
    }

    //! @brief help string
    QString helpString() const override;

  protected:
    void toggleShownList(unsigned int num);

    //! @brief working variables
    float _point_size;
    float _line_width;

    bool _show_normals;
    bool _lighting;

    //! @brief keypress handling
    QKeyEvent _last_key_event;
    bool _last_key_event_processed;

    //! @brief new camera
    StandardCamera* _camera = nullptr;

    //! @brief pointer to the qapplication
    QApplication* _qapp_ptr = nullptr;

    //! @brief light or dark background
    BackgroundMode _background;

    //! @brief deferred opencv rendering
    QGLPacketProcessor::CvMatVector _cv_mat_vector;

    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace srrg2_qgl_viewport
