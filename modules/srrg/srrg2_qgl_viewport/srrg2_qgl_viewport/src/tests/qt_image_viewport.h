#pragma once
#include <QGLViewer/qglviewer.h>
#include <QtGui/QKeyEvent>
#include <QtGui/QPainter>
#include <QtPrintSupport/QPrintDialog>
#include <QtPrintSupport/QPrinter>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>
#include <QtWidgets/qapplication.h>

#include "qgl_viewport.h"
#include <opencv2/opencv.hpp>

namespace srrg2_qgl_viewport {

  class QtImageViewport : public QWidget {
    // ia magic macro
    //    Q_OBJECT

  public:
    using BaseType = QWidget;
    using ThisType = QtImageViewport;

    explicit QtImageViewport(QWidget* parent_ = nullptr);
    virtual ~QtImageViewport();

    inline QSize sizeHint() const {
      return _qimage.size();
    }
    inline QSize minimumSizeHint() const {
      return _qimage.size();
    }

  public slots:
    void showCvImage(const cv::Mat& cv_image_);

  protected:
    //    QLabel* _qlabel = nullptr;

    void paintEvent(QPaintEvent* /*event*/) {
      // Display the image
      QPainter painter(this);
      painter.drawImage(QPoint(0, 0), _qimage);
      painter.end();
    }
    QImage _qimage;
  };

  class CvGlViewport : public srrg2_qgl_viewport::QGLViewport {
  public:
    CvGlViewport(QWidget* parent_ = nullptr) {
    }
  };

} /* namespace srrg2_qgl_viewport */
