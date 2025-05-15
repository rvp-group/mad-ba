#include "link.h"

namespace srrg2_core {

  Link::~Link() {
    _children.clear();
    _events.clear();
  }

  Link::InterpolationStatus Link::sample(const double& time_seconds_) {
    if (time_seconds_ < 1e-8) {
      _pose_in_parent = _events.rbegin()->second;
      return Link::InterpolationStatus::Ok;
    }
    // ds search two events for interpolating the specified time
    TransformQueue::const_iterator event_bot = _events.cend();
    TransformQueue::const_iterator event_top = _events.cend();

    // ds if interpolation is successful
    if (_setInterpolationData(time_seconds_, event_bot, event_top) == InterpolationStatus::Ok) {
      if (event_bot == _events.cend() && event_top == _events.cend()) {
        return InterpolationStatus::NoData;
      }

      const double& t_top = event_top->first;
      const double& t_bot = event_bot->first;

      const Isometry3f& transform_top = event_top->second;
      const Isometry3f& transform_bot = event_bot->second;

      if ((t_top - t_bot) < 1e-8 || transform_top.isApprox(transform_bot)) {
        _pose_in_parent = event_top->second;
        return _sampled_status;
      }
      // ds interpolate between the two events at the specified time
      const double normalized_time = (time_seconds_ - t_bot) / (t_top - t_bot);

      // ds interpolate translation - TODO check
      const Vector3f& translation_top = transform_top.translation();
      const Vector3f& translation_bot = transform_bot.translation();
      _pose_in_parent.translation() =
        translation_top + normalized_time * (translation_top - translation_bot);

      // tg interpolate rotation
      const Quaternionf q_bot(transform_bot.linear());
      const Quaternionf q_top(transform_top.linear());
      _pose_in_parent.linear() = q_bot.slerp(normalized_time, q_top).toRotationMatrix();
    }

    return _sampled_status;
  }

  void Link::setParent(Link* link_) {
    if (!link_) {
      return;
    }

    // ds set parent
    _parent            = link_;
    _identifier_parent = _parent->_identifier;

    // ds add ourself to the parents children
    //    _parent->_children.insert(this);
  }

  bool Link::addEvent(TransformEventPtr event_) {
    if (!event_) {
      return false;
    }

    // ds if it worked and the identifiers match - keep the event - otherwise ignore it
    if (event_->identifier() == _identifier) {
      if (!_identifier_parent.length() && event_->identifierParent().length()) {
        _identifier_parent = event_->identifierParent();
      }
      if (event_->identifierParent() != _identifier_parent) {
        std::cerr << RED << "Link::addEvent|link" << _identifier << " claims to have two fathers ["
                  << _identifier_parent << ", " << event_->identifierParent() << "]" << RESET
                  << std::endl;
        throw std::runtime_error("mismatching parents");
      }
      // ds in case we have at least two events in the link (minimum for interpolation)
      if (_events.size() > 1) {
        TransformQueue::reverse_iterator event_last = _events.rbegin();
        TransformQueue::iterator event_first        = _events.begin();

        if ((event_last->first - event_first->first) >= _time_window) {
          _events.erase(event_first);
        }

        std::pair<TransformQueue::iterator, bool> insert_result =
          _events.insert(std::make_pair(event_->timeSeconds(), event_->transform()));

        if (!insert_result.second) {
          throw std::runtime_error("Link::addEvent|Failed to insert new event, two different "
                                   "transform are present with the same timestamp");
        }
      } else {
        // ds add always first event
        _events[event_->timeSeconds()] = event_->transform();
      }
      return true;
    } else {
      return false;
    }
  }

  const std::string Link::toString() const {
    if (parent()) {
      return "[(" + parent()->identifier() + ") -> (" + _identifier + ":" + std::to_string(_level) +
             "," + std::to_string(_children.size()) + ")]";
    } else {
      return "[(world) -> (" + _identifier + ":" + std::to_string(_level) + "," +
             std::to_string(_children.size()) + ")]";
    }
  }

  std::ostream& operator<<(std::ostream& ostream_, const Link& joint_) {
    ostream_ << joint_.toString();
    return ostream_;
  }

  std::ostream& operator<<(std::ostream& ostream_, const Link* joint_) {
    return operator<<(ostream_, *joint_);
  }

  /* template <typename BearingType_, typename PositionType_> */
  /* bool LinkWithJoint_<BearingType_, PositionType_>::addEvent(BaseEventPtr event_) { */

  /*   //ds attempt a downcast */
  /*   JointEventPtr event = std::dynamic_pointer_cast<JointEvent>(event_); */

  /*   //ds if it worked - keep the event - otherwise ignore it */
  /*   if (event) { */
  /*     _events.push_back(event_); */
  /*     return true; */
  /*   } else { */
  /*     return false; */
  /*   } */
  /* } */

  Link::InterpolationStatus
  Link::_setInterpolationData(const double& time_seconds_,
                              TransformQueue::const_iterator& event_bottom_,
                              TransformQueue::const_iterator& event_top_) {
    // ds we require at least 2 events to interpolate - check for failure
    if (_events.size() < 2) {
      _sampled_status = InterpolationStatus::NoData;
      return _sampled_status;
    }

    event_top_ = _events.lower_bound(time_seconds_);
    if (event_top_ == _events.end()) {
      _sampled_status = InterpolationStatus::ExtrapolationTop;
      return _sampled_status;
    }

    if (std::fabs(event_top_->first - time_seconds_) < 1e-8) {
      event_bottom_   = event_top_;
      _sampled_status = InterpolationStatus::Ok;
      return _sampled_status;
    }

    event_bottom_ = event_top_;
    --event_bottom_;
    if (event_bottom_ == _events.end()) {
      _sampled_status = InterpolationStatus::ExtrapolationBottom;
      return _sampled_status;
    }

    if (event_bottom_->first == event_top_->first) {
      _sampled_status = InterpolationStatus::TreeError;
      return _sampled_status;
    }

    // ds done
    _sampled_status = InterpolationStatus::Ok;
    return _sampled_status;
  }

  /*   Link::InterpolationStatus LinkWithJointPrismatic::sample(const double& time_seconds_) { */

  /*     //ds search two events for interpolating the specified time */
  /*     double time_seconds_bot = 0; */
  /*     double time_seconds_top = 0; */
  /*     BaseEventPtr event_bot; */
  /*     BaseEventPtr event_top; */

  /*     //ds if interpolation is successful */
  /*     if (_setInterpolationData(time_seconds_, time_seconds_bot, time_seconds_top, event_bot,
   * event_top) == InterpolationStatus::Ok) { */
  /*       if (!event_bot || !event_top) { */
  /*         return InterpolationStatus::NoData; */
  /*       } */

  /*       //ds current positions to interpolate - downcasting */
  /*       const double t_bot = std::dynamic_pointer_cast<JointEvent>(event_bot)->position(); */
  /*       const double t_top = std::dynamic_pointer_cast<JointEvent>(event_top)->position(); */

  /*       //ds interpolate between the two events at the specified time */
  /*       const double normalized_time = (time_seconds_ - time_seconds_bot) / (time_seconds_top -
   * time_seconds_bot); */

  /*       //ds position in specified direction */
  /*       _sampled_position = t_top + normalized_time * (t_top - t_bot); */

  /*       //ds update pose in parent - shift in direction */
  /*       _pose_in_parent.translation() += _sampled_position * _bearing; */
  /*     } */
  /*     return _sampled_status; */
  /*   } */

  /*   Link::InterpolationStatus LinkWithJointRotational::sample(const double& time_seconds_) { */

  /*     //ds search two events for interpolating the specified time */
  /*     double time_seconds_bot = 0; */
  /*     double time_seconds_top = 0; */
  /*     BaseEventPtr event_bot; */
  /*     BaseEventPtr event_top; */

  /*     //ds if interpolation is successful */
  /*     if (_setInterpolationData(time_seconds_, time_seconds_bot, time_seconds_top, event_bot,
   * event_top) == InterpolationStatus::Ok) { */
  /*       if (!event_bot || !event_top) { */
  /*         return InterpolationStatus::NoData; */
  /*       } */

  /*       //ds current positions to interpolate - downcasting */
  /*       const Quaternionf q_bot =
   * Quaternionf(Eigen::AngleAxisf(std::dynamic_pointer_cast<JointEvent>(event_bot)->position() *
   * M_PI, _bearing)); */
  /*       const Quaternionf q_top =
   * Quaternionf(Eigen::AngleAxisf(std::dynamic_pointer_cast<JointEvent>(event_top)->position() *
   * M_PI, _bearing)); */

  /*       //ds interpolate between the two events at the specified time */
  /*       const double normalized_time = (time_seconds_ - time_seconds_bot) / (time_seconds_top -
   * time_seconds_bot); */

  /*       //ds spherical interpolation between orientations */
  /*       _sampled_position = q_bot.slerp(normalized_time, q_top); */

  /*       //ds update pose in parent - rotate */
  /*       _pose_in_parent.rotate(_sampled_position); */
  /*     } */
  /*     return _sampled_status; */
  /*   } */
} // namespace srrg2_core
