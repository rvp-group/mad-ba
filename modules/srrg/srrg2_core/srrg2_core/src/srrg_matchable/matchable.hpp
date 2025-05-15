namespace srrg2_core {

  template <typename Scalar_>
  void Matchable_<Scalar_>::setDirection(
    const Matchable_<Scalar_>::VectorType& direction_) {
    Scalar d = std::sqrt(direction_.x() * direction_.x() +
                         direction_.y() * direction_.y());

    const Scalar& dirx = direction_.x();
    const Scalar& diry = direction_.y();
    const Scalar& dirz = direction_.z();

    if (d > std::numeric_limits<Scalar>::min()) {
      _rotation << dirx, diry / d, dirx * dirz / d, diry, -dirx / d,
        diry * dirz / d, dirz, 0, -d;
    } else {
      _rotation << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    }
  }

  template <typename Scalar_>
  void Matchable_<Scalar_>::resetOmegaFromType() {
    _activation_matrix.setZero();
    switch (_type) {
      case Type::Point:
        _activation_matrix.setIdentity();
        break;
      case Type::Line:
        _activation_matrix.diagonal()[0] = _epsilon;
        _activation_matrix.diagonal()[1] = 1;
        _activation_matrix.diagonal()[2] = 1;
        break;
      case Type::Plane:
        _activation_matrix.diagonal()[0] = 1;
        _activation_matrix.diagonal()[1] = _epsilon;
        _activation_matrix.diagonal()[2] = _epsilon;
        break;
      case Type::Surfel:
        throw std::runtime_error(
          "Matchable_::resetOmegaFromType|surfels not managed");
      default:
        break;
    }
  }

} // namespace srrg2_core
