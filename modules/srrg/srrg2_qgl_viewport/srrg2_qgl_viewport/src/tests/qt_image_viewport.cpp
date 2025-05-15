
#include "qt_image_viewport.h"

namespace srrg2_qgl_viewport {

  QtImageViewport::QtImageViewport(QWidget* parent_) : QtImageViewport::BaseType(parent_) {
    //    _qlabel->setBackgroundRole(QPalette::Base);
    //    _qlabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    //    _qlabel->setScaledContents(true);
    //    _qlabel->setVisible(true);

    resize(640, 480);
  }

  QtImageViewport::~QtImageViewport() {
    //    if (_qlabel) {
    //      delete _qlabel;
    //    }
  }

  void QtImageViewport::showCvImage(const cv::Mat& cv_image_) {
    //    std::cerr << "QtImageViewport::setCvImage|cv image size = [" << cv_image_.cols << " x "
    //              << cv_image_.rows << "]\n";

    _qimage = QImage((uchar*) cv_image_.data,
                     cv_image_.cols,
                     cv_image_.rows,
                     cv_image_.step,
                     QImage::Format_RGB888);
    //    std::cerr << "QtImageViewport::setCvImage|qt image size = [" << _qimage.width() << " x "
    //              << _qimage.height() << "]\n";

    //    _qimage.save("my_image.png");
    //    _qlabel->setPixmap(QPixmap::fromImage(_qimage));
    //    _qlabel->adjustSize();
    this->setFixedSize(_qimage.size());
    repaint();
  }

} /* namespace srrg2_qgl_viewport */
