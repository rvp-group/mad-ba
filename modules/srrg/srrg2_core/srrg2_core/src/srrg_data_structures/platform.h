#pragma once
#include "link.h"
#include "srrg_geometry/geometry3d.h"

namespace srrg2_core {
  class Platform {
    //! object life
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Platform();
    ~Platform();

    //! interface
  public:
    //! @brief most generic access to the platform
    //! @param[in] message_ generic SRRG message (a set of events or joints, a message of type
    //! TransformEvents, ..)
    bool add(PropertyContainerBasePtr message_);

    //! @brief
    //! @param[out] transform_ target w.r.t. the root frame
    //! @param[in] target_
    //! @param[in, optional] time_ (default value will give back the last transform)
    //! throws std::out_of_range
    //! @returns true if no interpolation errors occur
    bool
    getTransform(Isometry3f& transform_, const std::string& target_, const double& time_ = 0) const;

    inline bool getTransform(Isometry2f& transform_,
                             const std::string& target_,
                             const double& time_t = 0) const {
      Isometry3f transform = Isometry3f::Identity();
      bool res             = this->getTransform(transform, target_, time_t);
      transform_           = srrg2_core::geometry3d::get2dFrom3dPose(transform);
      return res;
    }
    //! @brief
    //! @param[out] transform_ target w.r.t. reference
    //! @param[in] target_
    //! @param[in] reference_
    //! @param[in, optional] time_ (default value will give back the last transform)
    //! throws std::out_of_range
    //! @returns true if no interpolation errors occur
    bool getTransform(Isometry3f& transform_,
                      const std::string& target_,
                      const std::string& reference_,
                      const double& time_ = 0) const;

    inline bool getTransform(Isometry2f& transform_,
                             const std::string& target_,
                             const std::string& reference_,
                             const double& time_t = 0) const {
      Isometry3f transform = Isometry3f::Identity();
      bool res             = this->getTransform(transform, target_, reference_, time_t);
      transform_           = srrg2_core::geometry3d::get2dFrom3dPose(transform);
      return res;
    }

    //! @brief add generic joint event to model - optimally resulting in being assigned to the
    //! correct joint
    //! @brief param[in] joint_event_ generic polymorphic event, whose dynamic type is evaluated by
    //! its targeted joint
    bool addEvent(BaseEventPtr link_event_);

    //! @brief aims to integrate a new joint in the model
    //! @param[in] link_ joint to add to the model
    //! @returns true on success, false otherwise
    bool addLink(Link* link_);

    //! @brief configures joints - called automatically if not already done (e.g. in getTransform)
    bool setup();

    //! @returns
    bool isWellFormed() const;

    //! @brief write current model to stream
    friend std::ostream& operator<<(std::ostream& os, const Platform& platform_);
    friend std::ostream& operator<<(std::ostream& os, const Platform* platform_);

    //! @brief returns the links map
    inline const StringLinkMap& links() const {
      return _identifier_to_link_map;
    }
    //! @brief number of links in the platform
    inline const size_t size() const {
      return _identifier_to_link_map.size();
    }

    //! helpers
  protected:
    Link* findCommonRoot(Link* target, Link* reference) const;
    
    //! @brief recursively traverses the tree to the root (potentially interpolating between
    //! changing joints)
    bool
    _getTransformInRoot(Isometry3f& transform_, Link* current_, const double& time_seconds_) const;

    bool _getTransformInAncestor(Isometry3f& transform_,
                                           Link* current_,
                                           Link* ancestor_,
                                           const double& time_seconds_) const;
    
    //! @brief print warning/errors reguards the interpolation status
    void _printInterpolationStatus(const std::string& target_,
                                   const std::string& reference_,
                                   const Link::InterpolationStatus& sampled_status_) const;

    //! @brief set joint levels of current model
    //! @returns number of updated joints
    size_t _updateChildLevel(Link* joint_);

    //! @brief set the root for the current model
    void _setRoot();

    //! @brief update the hierarchy of the kinematic model
    void _updateHierarchy();

    //! attributes
  protected:
    //! @brief root joint
    Link* _root = nullptr;

    //! @brief all joints added (not necessarily linked in case of conflics)
    StringLinkMap _identifier_to_link_map;

    //! @brief current state
    bool _is_set_up = false;

    //! @brief number of correctly connected joint (last setup() call)
    size_t _number_of_correctly_connected_joints = 0;
  };

  //! @brief useful typedefs
  // using StringPlatformMap = std::map<std::string, Platform*>;
  using PlatformPtr          = std::shared_ptr<Platform>;
  using StringPlatformPtrMap = std::map<std::string, PlatformPtr>;

  // generic object that uses a platform
  class PlatformUser {
  public:
    virtual void setPlatform(PlatformPtr platform_);
    virtual PlatformPtr platform();
    virtual ~PlatformUser();

  protected:
    PlatformPtr _platform;
  };

  using PlatformUserPtr = std::shared_ptr<PlatformUser>;

} // namespace srrg2_core
