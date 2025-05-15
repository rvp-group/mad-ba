#pragma once
#include "viewer_context_base.h"

namespace srrg2_core {

  class ViewerManagerBase;

  //! @brief a unique canvas and all its metadata
  class ViewerContextShared : public ViewerContextBase {
  public:

    //! @brief creates connection between buffer manager and
    //!        the sink/source substratum
    void setup() override;

  protected:
    //! @brief protected ctor/dtor - only ViewerManagers can create this thing
    //!        it allocates all the structure needed and configures the memory stuff
    //! @param[in] context_name - required
    //! @param[in] manager_ - pointer to the creator - required
    //! @param[in, optional] num_buffers_ - number of buffers for this context, default 5
    //! @param[in, optional] buffer_size_ - buffer size for this context, default 1MB
    ViewerContextShared() = delete;
    ViewerContextShared(const std::string& context_name_,
                        ViewerManagerBase* manager_,
                        const size_t& num_buffers_ = 5,
                        const size_t& buffer_size_ = BUFFER_SIZE_1MEGABYTE);
    virtual ~ViewerContextShared();

    //! @brief inline get methods
    inline const ViewportBaseSet& viewports() const {return _viewports;}

    //! @brief container of all vieweports connected to this context - jic
    ViewportBaseSet _viewports;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //ia teletubbies friendship
    friend class ViewerManagerShared;
  };

  using ViewerContextSharedSet = std::set<ViewerContextShared*>;
  using ViewerContextSharedVector = std::vector<ViewerContextShared*>;
  using StringViewerContextSharedUnorderedMap = std::unordered_map<std::string, ViewerContextShared*>;
  using StringViewerContextSharedMap = std::map<std::string, ViewerContextShared*>;

  using ViewerContextSharedPtr = std::shared_ptr<ViewerContextShared>;
  using ViewerContextSharedPtrSet = std::set<ViewerContextSharedPtr>;
  using ViewerContextSharedPtrVector = std::vector<ViewerContextSharedPtr>;
  using StringViewerContextSharedPtrUnorderedMap = std::unordered_map<std::string, ViewerContextSharedPtr>;
  using StringViewerContextSharedPtrMap = std::map<std::string, ViewerContextSharedPtr>;
} //ia end namespace srrg2_core
