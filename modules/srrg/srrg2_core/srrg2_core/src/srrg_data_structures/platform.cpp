#include "platform.h"
#include "srrg_system_utils/system_utils.h"

#define DEBUG(var) \
  if (var)         \
  std::cerr

static bool platform_debug = false;
namespace srrg2_core {

  Platform::Platform() {
    //    std::cerr << "Platform::Platform|constructed" << std::endl;
  }

  Platform::~Platform() {
    //    std::cerr << "Platform::~Platform|destroying" << std::endl;
    for (auto item : _identifier_to_link_map) {
      delete item.second;
    }
    _identifier_to_link_map.clear();
    //    std::cerr << "Platform::~Platform|destroyed" << std::endl;
  }

  bool Platform::getTransform(Isometry3f& transform_,
                              const std::string& target_,
                              const double& time_seconds_) const {
    transform_.setIdentity();
    // ds check if the platform is set up
    if (!_is_set_up) {
      std::cerr << YELLOW << std::endl;
      std::cerr << "Platform::getTransform|WARNING platform is not set up yet\n";
      std::cerr << "Number of correctly connected joints : "
                << _number_of_correctly_connected_joints << std::endl;
      std::cerr << RESET << std::endl;
      return false;
    }

    // ia cleaning the identifier from strange tokens (/ and \)
    const std::string target_identifier          = removeStringTokens(target_);
    StringLinkMap::const_iterator target_link_it = _identifier_to_link_map.find(target_identifier);

    // ds check if the target link is not available in the model
    if (target_link_it == _identifier_to_link_map.end()) {
      std::cerr << FG_YELLOW("Platform::getTransform|WARNING could not find target ")
                << FG_BLUE(target_identifier) << std::endl;
      return false;
    }

    Link* target_link = target_link_it->second;
    // ds compute transform bottom up
    bool result = _getTransformInRoot(transform_, target_link, time_seconds_);
    if (!result) {
      _printInterpolationStatus(
        target_identifier, _root->identifier(), target_link->_sampled_status);
    }
    return result;
  }

    
  bool Platform::getTransform(Isometry3f& transform_,
                              const std::string& target_,
                              const std::string& reference_,
                              const double& time_seconds_) const {
    transform_.setIdentity();
    // ds check if the platform is set up
    if (!_is_set_up) {
      std::cerr << YELLOW << std::endl;
      std::cerr << "Platform::getTransform|WARNING platform is not set up\n";
      std::cerr << "Number of correctly connected joints : "
                << _number_of_correctly_connected_joints << std::endl;
      std::cerr << RESET << std::endl;
      return false;
    }

    // ia cleaning the identifier from strange tokens (/ and \)
    const std::string target_identifier          = removeStringTokens(target_);
    const std::string reference_identifier       = removeStringTokens(reference_);
    StringLinkMap::const_iterator target_link_it = _identifier_to_link_map.find(target_identifier);
    StringLinkMap::const_iterator reference_link_it =
      _identifier_to_link_map.find(reference_identifier);
    // ds check if the target links are not available in the model
    if (target_link_it == _identifier_to_link_map.end() ||
        reference_link_it == _identifier_to_link_map.end()) {
      std::cerr << "target   : " << target_identifier << std::endl;
      std::cerr << "reference: " << reference_identifier << std::endl;
      std::cerr << FG_YELLOW("Platform::getTransform|WARNING could not find target or "
                             "reference\n");
      return false;
    }

    // ds compute both transforms bottom up (TODO precompute paths in setup?)
    Link* target_link = target_link_it->second;
    Link* reference_link = reference_link_it->second;

    Link* common_ancestor = findCommonRoot(target_link, reference_link);
    if (! common_ancestor) {
      std::cerr << FG_YELLOW("Platform::getTransform|WARNING could not find a common ancestor between\n" << target_identifier << " and " << reference_identifier);
      return false;
    }
                                           
    Isometry3f target_in_ancestor;
    target_in_ancestor.setIdentity();
    bool target_result = _getTransformInAncestor(target_in_ancestor, target_link, common_ancestor, time_seconds_);
    if (!target_result) {
      _printInterpolationStatus(
        target_identifier, reference_identifier, target_link->_sampled_status);
      return target_result;
    }

    Isometry3f reference_in_ancestor;
    reference_in_ancestor.setIdentity();
    bool reference_result = _getTransformInAncestor(reference_in_ancestor, reference_link, common_ancestor, time_seconds_);
    if (!reference_result) {
      std::cerr << "reference" << std::endl;
      _printInterpolationStatus(
        target_identifier, reference_identifier, reference_link->_sampled_status);
      return reference_result;
    }

    transform_ = reference_in_ancestor.inverse() * target_in_ancestor;
    return true;
  }

  Link* Platform::findCommonRoot(Link* target, Link* reference) const {
    Link* t=target;
    Link* r=reference;
    while(1) {
      if (! t || ! r)
        break;
      if (t==r) {
        return t;
      }
      
      if (t->level()>r->level()) {
        t=t->parent();
      } else if(r->level()>t->level()) {
        r=r->parent();
      } else {
        r=r->parent();
        t=t->parent();
      } 
    }
    
    t=target;
    r=reference;
    while (1) {
      if (! t)
        return 0;
      if (t==r)
        return t;
      t=t->parent();
    }
  }

  

  bool Platform::add(PropertyContainerBasePtr message_) {
    if (!message_) {
      //      std::cerr << "Platform::add|ignoring empty message" << std::endl;
      return false;
    }

    // ia try to get a transform event vector
    PropertyTransformEventVector* transform_event_vector =
      message_->property<PropertyTransformEventVector>("events");
    if (!transform_event_vector) {
      return false;
    }

    for (std::size_t i = 0; i < transform_event_vector->size(); ++i) {
      TransformEventPtr transform_event(new TransformEvent(transform_event_vector->value(i)));
      addEvent(transform_event);
    }

    // ds success if still here
    return true;
  }

  bool Platform::addEvent(BaseEventPtr link_event_) {
    // ds filter invalid events which we cannot associate with a link
    if (link_event_->identifier().empty()) {
      std::cerr << FG_YELLOW("Platform::addEvent|WARNING: received disconnected "
                             "BaseEvent (identfier not set)")
                << std::endl;
      return false;
    }

    // ds check if corresponding link is available
    StringLinkMap::iterator element = _identifier_to_link_map.find(link_event_->identifier());
    Link* link                      = 0;
    // ds if not available
    TransformEventPtr tf_event = std::dynamic_pointer_cast<TransformEvent>(link_event_);

    if (element == _identifier_to_link_map.end()) {
      DEBUG(platform_debug) << "Platform::addEvent|"
                            << FG_YELLOW("WARNING: link not present: '" << link_event_->identifier()
                                                                        << "'\n");

      // ds check if we have a transform event - ONLY FOR THESE WE CAN ADD
      // BASE LINKS
      if (tf_event) {
        // ds setup a new tf link
        link =
          new Link(link_event_->identifier(), tf_event->transform(), tf_event->identifierParent());
        DEBUG(platform_debug) << "Platform::addEvent|" << GREEN << "adding new Link: '"
                              << link_event_->identifier() << "' with parent: '"
                              << tf_event->identifierParent() << "'" << RESET << std::endl;
        addLink(link);
      } else {
        // ds nothing we can do
        return false;
      }
    } else {
      // ds retrieve targeted link
      link = element->second;
      DEBUG(platform_debug) << "Platform::addEvent|"
                            << FG_BLUE("WARNING: link present: '"
                                       << link->identifier() << "' with parent: '"
                                       << link->identifierParent() << "'\n");

      if (link->identifierParent() != tf_event->identifierParent()) {
        if (link->identifierParent() == "") {
          DEBUG(platform_debug) << "Platform::addEvent|"
                                << FG_GREEN("found root " << tf_event->identifierParent() << " for "
                                                          << link->identifier())
                                << std::endl;
          link->setIdentifierParent(tf_event->identifierParent());
          setup();
        } else {
          std::cerr << "Platform::addEvent|"
                    << FG_RED(link->identifier()
                              << " has different roots: " << link->identifierParent() << " "
                              << tf_event->identifierParent())
                    << std::endl;
        }
      }
    }
    // ds add event (will fail and have no effect if wrong dynamic event type)
    /* std::cerr << "Platform::addEvent|link " << link->identifier() << " num
     * events = " << link->numberOfEvents() << std::endl; */
    return link->addEvent(tf_event);
  }

  bool Platform::addLink(Link* link_) {
    if (!link_) {
      DEBUG(platform_debug) << "Platform::addLink|WARNING: received null joint object" << std::endl;
      return false;
    }

    // ds attempt insertion and check for already present identfier
    if (!_identifier_to_link_map.insert(std::make_pair(link_->identifier(), link_)).second) {
      // ds ignore request (TODO: critical or not?)
      DEBUG(platform_debug) << FG_YELLOW("Platform::addLink|WARNING: Link: '" << link_->_identifier
                                                                              << "' already exists")
                            << std::endl;
      return false;
    } else {
      // ds update the structure
      return setup();
    }
  }

  bool Platform::isWellFormed() const {
    if (!_is_set_up) {
      return false;
    }

    // ds check if empty (trivial case)
    if (!_identifier_to_link_map.size()) {
      std::cerr << FG_RED("Platform::isWellFormed|empty kinematic model") << std::endl;
      return true;
    }

    // ds check if we have loose children (TODO critical?)
    if (_number_of_correctly_connected_joints != _identifier_to_link_map.size()) {
      std::cerr << FG_RED("Platform::isWellFormed|WARNING: invalid Links: "
                          << _identifier_to_link_map.size() - _number_of_correctly_connected_joints)
                << std::endl;

      for (const auto& id_link_elem : _identifier_to_link_map) {
        std::cerr << (id_link_elem.second->parent() ? id_link_elem.second->parent()->identifier()
                                                    : "world")
                  << " -> " << id_link_elem.first << std::endl;
      }
      return false;
    }

    return true;
  }

  std::ostream& operator<<(std::ostream& ostream_, const Platform& platform_) {
    if (platform_._identifier_to_link_map.empty()) {
      ostream_ << "empty model" << std::endl;
    }

    // ds construct output string based on level in the tree - by scanning all
    // joints
    std::map<int32_t, LinkVector> joints_per_level;
    std::vector<int32_t> levels;
    for (auto it = platform_._identifier_to_link_map.begin();
         it != platform_._identifier_to_link_map.end();
         ++it) {
      Link* joint  = it->second;
      auto element = joints_per_level.find(joint->level());
      if (element != joints_per_level.end()) {
        element->second.push_back(joint);
      } else {
        if (joint->level() != -1) {
          joints_per_level.insert(std::make_pair(joint->level(), std::vector<Link*>(1, joint)));
          levels.push_back(joint->level());
        }
      }
    }
    if (levels.empty()) {
      return ostream_;
    }

    // ds sort levels
    std::sort(levels.begin(), levels.end());

    // ds print each level
    ostream_ << "-----------------------" << std::endl;
    for (const int32_t& level : levels) {
      ostream_ << "level: " << level << " ";
      for (Link* joint : joints_per_level.at(level)) {
        ostream_ << joint << " ";
      }
      ostream_ << std::endl;
    }
    ostream_ << "-----------------------";
    return ostream_;
  }

  std::ostream& operator<<(std::ostream& ostream_, const Platform* platform_) {
    return operator<<(ostream_, *platform_);
  }

  void Platform::_printInterpolationStatus(const std::string& target_,
                                           const std::string& reference_,
                                           const Link::InterpolationStatus& sampled_status) const {
    std::cerr << FG_BLUE("Platform::getTransform|" << target_ << " in " << reference_ << "[" << sampled_status << "]");
    switch (sampled_status) {
      case Link::InterpolationStatus::ExtrapolationTop:
        std::cerr << FG_YELLOW(" interpolation require extrapolation in the future") << std::endl;
        break;
      case Link::InterpolationStatus::ExtrapolationBottom:
        std::cerr << FG_YELLOW(" interpolation require extrapolation in the past") << std::endl;
        break;
      case Link::InterpolationStatus::NoData:
        std::cerr << FG_RED(" interpolation failed no data available") << std::endl;
        break;
      case Link::InterpolationStatus::TreeError:
        std::cerr << FG_BRED("transform tree error") << std::endl;
        break;
      default:
        break;
    }
  }

  bool Platform::setup() {
    _is_set_up                            = false;
    _number_of_correctly_connected_joints = 0;

    _root = 0;
    DEBUG(platform_debug) << "Platform::setup|updating hierarchy" << std::endl;
    _updateHierarchy();
    DEBUG(platform_debug) << "Platform::setup|setting root" << std::endl;
    _setRoot();

    // ds check if it didn't WORK!!!!!
    if (!_root) {
      std::cerr << FG_RED("Platform::setup|ERROR: failed to set root") << std::endl;
      return false;
    }
    // ds adjust joint levels of all joints connected to the root
    _root->_level                         = 0;
    _number_of_correctly_connected_joints = _updateChildLevel(_root);

    // ds check if we have loose children (TODO critical?)
    if (_number_of_correctly_connected_joints != _identifier_to_link_map.size()) {
      DEBUG(platform_debug) << "Platform::setup|WARNING: invalid Links: "
                            << _identifier_to_link_map.size() -
                                 _number_of_correctly_connected_joints
                            << std::endl;
    }
    DEBUG(platform_debug) << "8=========D" << this << "C==========8" << std::endl;

    // ds fine
    _is_set_up = true;
    return true;
  }

  bool Platform::_getTransformInAncestor(Isometry3f& transform_,
                                         Link* current_,
                                         Link* ancestor_,
                                         const double& time_seconds_) const {
    transform_.setIdentity();
    while (current_!=ancestor_) {
      Link::InterpolationStatus sampled_status = current_->sample(time_seconds_);
      if (sampled_status != Link::InterpolationStatus::Ok) {
        std::cerr << "interpolation error status for link: "
                  << current_->identifier()
                  << " " << sampled_status << std::endl;
        return false;
      }
      transform_= current_->poseInParent() * transform_;
      if (!current_->parent())
        return false;
      current_=current_->parent();
    }
    return true;
  }

  bool Platform::_getTransformInRoot(Isometry3f& transform_,
                                     Link* current_,
                                     const double& time_seconds_) const {
    return _getTransformInAncestor(transform_, current_, _root, time_seconds_);
  }

  void Platform::_setRoot() {
    _root = 0;

    Link* root_candidate;
    for (auto pair : _identifier_to_link_map) {
      root_candidate = pair.second;

      // ds traverse backwards towards the root
      while (root_candidate->parent()) {
        root_candidate = root_candidate->parent();
      }

      // ds check for conflicting root candidates
      if (root_candidate && _root && _root != root_candidate) {
        DEBUG(platform_debug) << "Platform::_setRoot|WARNING: multiple roots: "
                              << _root->identifier() << " " << root_candidate->identifier()
                              << std::endl;
      } else {
        _root = root_candidate;
      }
    }
  }

  void Platform::_updateHierarchy() {
    for (auto pair : _identifier_to_link_map) {
      Link* child_link = pair.second;

      // ds if the link is not connected to a parent, no hierarchy to update
      if (child_link->_identifier_parent.empty()) {
        continue;
      }

      // ds check if parent is available AND (!!!!!!!) not set already
      /* std::cerr << "processing " << pair.first << std::endl; */
      /* std::cerr << "parent should be" << child_link->_identifier_parent <<
       * std::endl; */
      StringLinkMap::iterator parent_it =
        _identifier_to_link_map.find(child_link->_identifier_parent);
      Link* parent = 0;
      // ds if parent was not found
      if (parent_it == _identifier_to_link_map.end()) {
        // ds allocate a default link (TODO discuss)
        parent = new Link(child_link->_identifier_parent);
        DEBUG(platform_debug) << "Platform::_updateHierarchy|WARNING: creating empty Link: '"
                              << parent->identifier() << "' as potential root candidate"
                              << std::endl;
        addLink(parent);
      } else {
        parent = parent_it->second;
      }

      // ds if the child is not yet connected
      if (!child_link->_parent) {
        // ds set parent for this child
        child_link->setParent(parent);
        parent->addChild(child_link);
      }
    }
  }

  size_t Platform::_updateChildLevel(Link* joint_) {
    size_t number_of_children = 1;
    for (Link* child_joint : joint_->_children) {
      child_joint->_level = joint_->_level + 1;
      number_of_children += _updateChildLevel(child_joint);
    }
    return number_of_children;
  }

  void PlatformUser::setPlatform(PlatformPtr platform_) {
    _platform = platform_;
  }

  PlatformPtr PlatformUser::platform() {
    return _platform;
  }

  PlatformUser::~PlatformUser() {
  }

} // namespace srrg2_core

#undef DEBUG
