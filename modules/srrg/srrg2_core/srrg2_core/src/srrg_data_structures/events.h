#pragma once
#include "srrg_geometry/geometry_defs.h"
#include "srrg_property/property_container.h"
#include "srrg_property/property_eigen.h"
#include "srrg_property/property_serializable.h"
#include "srrg_property/property_vector.h"
#include <list>

namespace srrg2_core {

  //! @class abstract base event (construction only allowed by subclasses)
  class BaseEvent : public PropertyContainerIdentifiable {
  protected:
    BaseEvent() : SETUP_PROPERTY(_time_seconds, 0), SETUP_PROPERTY(_identifier, "") {
    }

    BaseEvent(const double& time_seconds_, const std::string& identifier_link_) :
      SETUP_PROPERTY(_time_seconds, time_seconds_),
      SETUP_PROPERTY(_identifier, identifier_link_) {
    }

    BaseEvent(const BaseEvent& event_) :
      SETUP_PROPERTY(_time_seconds, event_._time_seconds.value()),
      SETUP_PROPERTY(_identifier, event_._identifier.value()) {
    }

  public:
    bool operator<(const BaseEvent& other_) {
      return _time_seconds.value() < other_._time_seconds.value();
    }

    bool operator>(const BaseEvent& other_) {
      return _time_seconds.value() > other_._time_seconds.value();
    }

    void operator=(const BaseEvent& other_) {
      _time_seconds.value() = other_._time_seconds.value();
      _identifier.value()   = other_._identifier.value();
    }

    void setTimeSeconds(const double& time_seconds_) {
      _time_seconds.setValue(time_seconds_);
    }
    const double& timeSeconds() const {
      return _time_seconds.value();
    }

    const std::string& identifier() const {
      return _identifier.value();
    }

  protected:
    PropertyDouble _time_seconds;
    PropertyString _identifier;
  };

  typedef std::shared_ptr<BaseEvent> BaseEventPtr;
  typedef std::vector<BaseEventPtr> BaseEventPtrVector;

  //! @class joint event (construction only allowed by subclasses)
  class JointEvent : public BaseEvent {
  public:
    JointEvent() :
      SETUP_PROPERTY(_position, 0),
      SETUP_PROPERTY(_velocity, 0),
      SETUP_PROPERTY(_effort, 0) {
    }

    JointEvent(const double& time_seconds_,
               const std::string& identifier_joint_,
               const double& position_,
               const double& velocity_ = 0,
               const double& effort_   = 0) :
      BaseEvent(time_seconds_, identifier_joint_),
      SETUP_PROPERTY(_position, position_),
      SETUP_PROPERTY(_velocity, velocity_),
      SETUP_PROPERTY(_effort, effort_) {
    }

    JointEvent(const JointEvent& event_) :
      BaseEvent(event_._time_seconds.value(), event_._identifier.value()),
      SETUP_PROPERTY(_position, event_._position.value()),
      SETUP_PROPERTY(_velocity, event_._effort.value()),
      SETUP_PROPERTY(_effort, event_._effort.value()) {
    }

    void operator=(const JointEvent& other_) {
      _time_seconds.value() = other_._time_seconds.value();
      _identifier.value()   = other_._identifier.value();
      _position.value()     = other_._position.value();
      _velocity.value()     = other_._velocity.value();
      _effort.value()       = other_._effort.value();
    }

    bool operator==(const JointEvent& other_) {
      return (_position.value() == other_._position.value() &&
              _velocity.value() == other_._velocity.value() &&
              _effort.value() == other_._effort.value());
    }

    bool operator!=(const JointEvent& other_) {
      return !(*this == other_);
    }

    const double& position() const {
      return _position.value();
    }

    const double& velocity() const {
      return _velocity.value();
    }

    const double& effort() const {
      return _effort.value();
    }

  protected:
    PropertyDouble _position;
    PropertyDouble _velocity;
    PropertyDouble _effort;
  };

  typedef std::shared_ptr<JointEvent> JointEventPtr;
  typedef PropertySerializableVector_<JointEvent> PropertyJointEventVector;

  //! @class transform event (construction only allowed by subclasses)
  class TransformEvent : public BaseEvent {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TransformEvent() :
      SETUP_PROPERTY(_transform, Isometry3f::Identity()),
      SETUP_PROPERTY(_identifier_parent, "") {
    }

    TransformEvent(const double& time_seconds_,
                   const std::string& identifier_,
                   const Isometry3f& transform_,
                   const std::string& identifier_parent_ = "") :
      BaseEvent(time_seconds_, identifier_),
      SETUP_PROPERTY(_transform, transform_),
      SETUP_PROPERTY(_identifier_parent, identifier_parent_) {
    }

    TransformEvent(const TransformEvent& event_) :
      BaseEvent(event_._time_seconds.value(), event_._identifier.value()),
      SETUP_PROPERTY(_transform, event_._transform.value()),
      SETUP_PROPERTY(_identifier_parent, event_._identifier_parent.value()) {
    }

    void operator=(const TransformEvent& other_) {
      _time_seconds.value()      = other_._time_seconds.value();
      _identifier.value()        = other_._identifier.value();
      _transform.value()         = other_._transform.value();
      _identifier_parent.value() = other_._identifier_parent.value();
    }

    bool operator==(const TransformEvent& other_) const {
      return _transform.value().isApprox(other_._transform.value(), 1e-8);
    }

    bool operator!=(const TransformEvent& other_) const {
      return !(*this == other_);
    }

    const Isometry3f& transform() const {
      return _transform.value();
    }

    const std::string& identifierParent() const {
      return _identifier_parent.value();
    }

  protected:
    //! @brief this in parent's frame
    PropertyEigen_<Isometry3f> _transform;

    //! @brief parent links identifier in case we have additional information about the parent link
    //! (consistency check)
    PropertyString _identifier_parent;
  };

  using TransformEventPtr            = std::shared_ptr<TransformEvent>;
  using PropertyTransformEventVector = PropertySerializableVector_<TransformEvent>;

} // namespace srrg2_core
