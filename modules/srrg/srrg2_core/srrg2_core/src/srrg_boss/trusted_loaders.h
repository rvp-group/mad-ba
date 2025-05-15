#define BOSS_TRUSTED_LOADER_H(ns)                        \
  namespace ns {                                         \
    class ns##_dummy_obj : public Serializable {         \
    public:                                              \
      virtual void serialize(ObjectData&, IdContext&);   \
      virtual void deserialize(ObjectData&, IdContext&); \
    };                                                   \
  }

#define BOSS_TRUSTED_LOADER_CPP(ns)                             \
  namespace ns {                                                \
    void ns##_dummy_obj::serialize(ObjectData&, IdContext&) {   \
    }                                                           \
    void ns##_dummy_obj::deserialize(ObjectData&, IdContext&) { \
    }                                                           \
    BOSS_REGISTER_CLASS(ns##_dummy_obj);                        \
  }

// the following MUST BE _dummy_obj_inst and not _dummy_obj_inst() (damned value-initialization
// "feature")
#define BOSS_ENSURE_LOAD(ns) ns::ns##_dummy_obj ns##_dummy_obj_inst;
