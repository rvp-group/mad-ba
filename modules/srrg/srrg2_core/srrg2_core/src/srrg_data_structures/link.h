#pragma once
#include <srrg_system_utils/shell_colors.h>

#include "events.h"

namespace srrg2_core {

  //! @brief friend access required to arrange and configure joints properly
  class Platform;

  // ds readability
  // ia never use shared pointer or goddamn enable_shared_from this.
  // ia it's not our cup of tea.
  class Link;
  using LinkPtr       = std::shared_ptr<Link>;
  using LinkVector    = std::vector<Link*>;
  using LinkSet       = std::set<Link*>;
  using LinkList      = std::list<Link*>;
  using StringLinkMap = std::map<std::string, Link*>;

  // Transform event container
  using TransformQueue = std::map<double,
                                  Isometry3f,
                                  std::less<double>,
                                  Eigen::aligned_allocator<std::pair<const double, Isometry3f>>>;

  //! @class basic link class (i.e. transforms connecting other links) bless C++ for multiple
  //! inheritance
  class Link : public PropertyContainerIdentifiable {
    // ds exported types
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum InterpolationStatus {   // ds why not move to exceptions?
      Ok                  = 0x0, // ds interpolation successful
      ExtrapolationBottom = 0x1, // ds missing past information for interpolation
      ExtrapolationTop    = 0x2, // ds missing future information for interpolation
      NoData              = 0x4, // ds no events available for interpolation
      TreeError           = 0x8  // ds invalid joint model
    };

    // ds construction
    // ia TODO protectet link contruction. only platform can create goddamn link, since without
    // platform they are useless
  public:
    Link(const std::string& identifier_, const std::string& identifier_parent_ = "") :
      _identifier(identifier_),
      _identifier_parent(identifier_parent_) {
      _children.clear();
      _events.clear();
    }

    Link(const std::string& identifier_,
         const Isometry3f& pose_in_parent_,
         const std::string& identifier_parent_ = "") :
      _identifier(identifier_),
      _identifier_parent(identifier_parent_),
      _pose_in_parent(pose_in_parent_) {
      _children.clear();
      _events.clear();
    }

    Link(const Link& other_) :
      _identifier(other_._identifier),
      _identifier_parent(other_._identifier_parent),
      _pose_in_parent(other_._pose_in_parent),
      _children(other_._children),
      _level(other_._level),
      _sampled_status(other_._sampled_status),
      _events(other_._events) {
    }

    virtual ~Link();

    //! @brief disable default construction
    Link() = delete;

    // ds interface
  public:
    //! @brief this method has no effect on a basic link, but has an effect for a link with a joint
    virtual InterpolationStatus sample(const double& time_seconds_);

    void setParent(Link* link_);

    void addChild(Link* link_) {
      _children.insert(link_);
    }

    void setIdentifierParent(const std::string& identifier_parent_) {
      _identifier_parent = identifier_parent_;
    }

    //! @brief accepts only TransformEvents
    virtual bool addEvent(TransformEventPtr event_);

    inline Link* parent() const {
      return _parent;
    }

    //! @brief print joint information
    const std::string toString() const;
    friend std::ostream& operator<<(std::ostream& ostream_, const Link& link_);
    friend std::ostream& operator<<(std::ostream& ostream_, const Link* link_);

    const Isometry3f poseInParent() const {
      return _pose_in_parent;
    }

    const int32_t level() const {
      return _level;
    }

    const size_t numberOfChildren() const {
      return _children.size();
    }

    const size_t numberOfEvents() const {
      return _events.size();
    }

    const std::string& identifier() const {
      return _identifier;
    }

    const std::string& identifierParent() const {
      return _identifier_parent;
    }

    //! helpers
  protected:
    Link::InterpolationStatus _setInterpolationData(const double& time_seconds_,
                                                    TransformQueue::const_iterator& event_bottom_,
                                                    TransformQueue::const_iterator& event_top_);
    //! attributes
  protected:
    //! @brief this links identifier - corresponds to the event identifier
    std::string _identifier = "";

    //! parent link (zero for world) - not required on construction (might arrive later)
    Link* _parent = nullptr;

    //! @brief parent identifier
    std::string _identifier_parent = "";

    //! @brief time window for the transform queue (TODO make some config or set/get accessor)
    double _time_window = 5.0;

    //! @brief this link in parent frame
    Isometry3f _pose_in_parent = Isometry3f::Identity();

    //! @brief child links
    LinkSet _children;

    //! @brief level in the model (tree): -1 not set
    int32_t _level = -1;

    //! @brief workspace (last sampled state)
    InterpolationStatus _sampled_status = InterpolationStatus::NoData;

    //! @brief event history
    TransformQueue _events;

    //! @brief friend access required to arrange and configure joints without requiring accessors
    friend Platform;
  };

  /* //! @class link with joint, contains relative pose information as well as a mutable heading */
  /*   template <typename BearingType_, typename PositionType_> */
  /*   class LinkWithJoint_: public Link { */
  /*   public: */

  /*     EIGEN_MAKE_ALIGNED_OPERATOR_NEW; */

  /*     using BearingType  = BearingType_; */
  /*     using PositionType = PositionType_; */

  /*     LinkWithJoint_(const std::string& identifier_, const BearingType& bearing_):
   * Link(identifier_), _bearing(bearing_) {} */

  /*     LinkWithJoint_(const std::string& identifier_, const BearingType& bearing_, const
   * Isometry3f& pose_in_parent_): Link(identifier_, */
  /*                                                                                                                          pose_in_parent_),
   */
  /*                                                                                                                     _bearing(bearing_)
   * {} */

  /* //! interface */
  /*   public: */

  /*     //! @brief accepts only JointEvents */
  /*     virtual bool addEvent(BaseEventPtr event_) override; */

  /*     virtual InterpolationStatus sample(const double& time_seconds_) = 0; */

  /*     const BearingType& bearing() const { */
  /*       return _bearing; */
  /*     } */

  /*     const PositionType& sampledPosition() const { */
  /*       return _sampled_position; */
  /*     } */

  /*   protected: */

  /*     //! @brief bearing configuration of the joint (e.g. a direction for prismatic, an axis for
   * rotational) */
  /*     BearingType _bearing; */

  /*     //! @brief last sampled joint position in specified bearing direction */
  /*     PositionType _sampled_position; */

  /*   }; */

  /*   class LinkWithJointPrismatic: public LinkWithJoint_<Vector3f, double> { */

  /* //! construction */
  /*   public: */
  /*     EIGEN_MAKE_ALIGNED_OPERATOR_NEW; */

  /*     LinkWithJointPrismatic(const std::string& identfier_, const Vector3f& direction_): */
  /*       LinkWithJoint_(identfier_, */
  /*                      direction_.normalized()) {} */

  /*     LinkWithJointPrismatic(const std::string& identfier_, const Vector3f& direction_, const
   * Isometry3f& pose_in_parent_): */
  /*       LinkWithJoint_(identfier_, */
  /*                      direction_.normalized(), */
  /*                      pose_in_parent_) {} */

  /*     virtual InterpolationStatus sample(const double& time_seconds_) override; */
  /*   }; */

  /*   class LinkWithJointRotational: public LinkWithJoint_<Vector3f, Quaternionf> { */

  /* //! construction */
  /*   public: */
  /*     EIGEN_MAKE_ALIGNED_OPERATOR_NEW; */

  /*     LinkWithJointRotational(const std::string& identfier_, const Vector3f& axis_): */
  /*       LinkWithJoint_(identfier_, */
  /*                      axis_.normalized()) {} */

  /*     LinkWithJointRotational(const std::string& identfier_, const Vector3f& axis_, const
   * Isometry3f& pose_in_parent_): */
  /*       LinkWithJoint_(identfier_, */
  /*                      axis_.normalized(), */
  /*                      pose_in_parent_) {} */

  /*     virtual InterpolationStatus sample(const double& time_seconds_) override; */
  /*   }; */

} // namespace srrg2_core
