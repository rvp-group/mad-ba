#pragma once
#include "viewer_core/viewport.h"
#include "viewer_canvas.h"


namespace srrg2_core {

  class ViewerManagerBase;

  //! @brief a unique canvas and all its metadata
  //!        base class, will be specialized in SHARED and ROS ones
  class ViewerContextBase {
  public:
    //! @brief Invalid: should never happen
    //!        Inactive: context is created but is not ready to be used -> call setup()
    //!        Ready: redy to be used and rendered
    //!        Active: at least a viewport has been attached to this context
    enum ViewerContextStatus {Invalid=0, Inactive=1, Ready=2, Active=3};
    enum ViewerContextType {Unknown=0, Shared=1, ROS=2};


    //! @brief this function will create all the connection required for the
    //!        functioning of the current context depending on its type (shared, ROS)
    //!        after this, the context is ready to be used and rendered
    virtual void setup() = 0;

    //! @brief inline get method
    inline const ViewerContextStatus& status() const {return _status;}
    inline const ViewerContextType& type() const {return _type;}

    inline const std::string& name() const {return _context_name;}
    inline const ViewerCanvasPtr& canvas() const {return _canvas;}
    inline const BufferManagerPtr& bufferManager() const {return _buffer_manager;}
    inline const BufferSinkBasePtr& sink() const {return _sink;}
    inline const BufferSourceBasePtr& source() const {return _source;}

  protected:
    //! @brief protected ctor/dtor - only ViewerManagers can create this thing
    //!        it allocates all the structure needed and configures the memory stuff
    //! @param[in] context_name - required
    //! @param[in] manager_ - pointer to the creator - required
    //! @param[in, optional] num_buffers_ - number of buffers for this context, default 5
    //! @param[in, optional] buffer_size_ - buffer size for this context, default 1MB
    ViewerContextBase() = delete;
    ViewerContextBase(const std::string& context_name_,
                      ViewerManagerBase* viewer_manager_,
                      const size_t& num_buffers_ = 5,
                      const size_t& buffer_size_ = BUFFER_SIZE_1MEGABYTE);
    virtual ~ViewerContextBase();

    //! @brief every context has a unique canvas (owned)
    ViewerCanvasPtr _canvas;

    //! @brief every context has a unique manager (owned)
    BufferManagerPtr _buffer_manager;

    //! @brief every context needs a source and a sink (owned)
    BufferSinkBasePtr _sink;
    BufferSourceBasePtr _source;

    //! @brief every context has a unique name
    const std::string _context_name;

    //! @brief ptr to the creator
    ViewerManagerBase* _viewer_manger = 0;

    //! @brief status and type - jic
    ViewerContextStatus _status = ViewerContextStatus::Invalid;
    ViewerContextType   _type   = ViewerContextType::Unknown;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //ia teletubbies friendship
    friend class ViewerManagerBase;
    friend class ViewerContextShared;
  };

  //! @brief useful typedefs
  using ViewerContextBaseSet = std::set<ViewerContextBase*>;
  using ViewerContextBaseVector = std::vector<ViewerContextBase*>;
  using StringViewerContextBaseUnorderedMap = std::unordered_map<std::string, ViewerContextBase*>;
  using StringViewerContextBaseMap = std::map<std::string, ViewerContextBase*>;

  using ViewerContextBasePtr = std::shared_ptr<ViewerContextBase>;
  using ViewerContextBasePtrSet = std::set<ViewerContextBasePtr>;
  using ViewerContextBasePtrVector = std::vector<ViewerContextBasePtr>;
  using StringViewerContextBasePtrUnorderedMap = std::unordered_map<std::string, ViewerContextBasePtr>;
  using StringViewerContextBasePtrMap = std::map<std::string, ViewerContextBasePtr>;

} //ia end namespace srrg2_core



