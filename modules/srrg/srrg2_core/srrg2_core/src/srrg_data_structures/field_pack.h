#pragma once

namespace srrg2_core {

  // type getter, returns the type of the ith type in a tuple
  template <typename FieldPackType_, int i>
  struct TypeAt_ {
    using FieldItemType =
      typename TypeAt_<typename FieldPackType_::FieldPackRestType, i - 1>::FieldItemType;
  };

  template <typename FieldPackType_>
  struct TypeAt_<FieldPackType_, 0> {
    using FieldItemType = typename FieldPackType_::FieldItemType;
  };

  // value getter, returns the value of the ith type in a tuple

  template <typename FieldPackType_, int i>
  struct FieldAt_ {
    static inline typename TypeAt_<FieldPackType_, i>::FieldItemType& field(FieldPackType_& pack) {
      return FieldAt_<typename FieldPackType_::FieldPackRestType, i - 1>::field(pack);
    }

    static const inline typename TypeAt_<FieldPackType_, i>::FieldItemType&
    field(const FieldPackType_& pack) {
      return FieldAt_<typename FieldPackType_::FieldPackRestType, i - 1>::field(pack);
    }
  };

  template <typename FieldPackType_>
  struct FieldAt_<FieldPackType_, 0> {
    static inline typename FieldPackType_::FieldItemType& field(FieldPackType_& pack) {
      return pack._field;
    }

    static const inline typename FieldPackType_::FieldItemType& field(const FieldPackType_& pack) {
      return pack._field;
    }
  };

  // tuple: the types are accessible with indices starting from 0
  // in the order they are declared in the template
  template <typename FieldItemType_, typename... FieldRestType_>
  struct FieldPack_ : public FieldPack_<FieldRestType_...> {
    using FieldItemType            = FieldItemType_;
    using ThisType                 = FieldPack_<FieldItemType_, FieldRestType_...>;
    using FieldPackRestType        = FieldPack_<FieldRestType_...>;
    static constexpr int NumFields = 1 + sizeof...(FieldRestType_);

    template <int i>
    static constexpr int unrolledIdx() {
      return i;
    }

    template <int i>
    static constexpr int UnrolledIdx = unrolledIdx<i>();

    template <int i>
    using TypeAt = typename TypeAt_<ThisType, i>::FieldItemType;

    template <int i>
    inline TypeAt<i>& field() {
      return FieldAt_<ThisType, i>::field(*this);
    }

    template <int i>
    inline const TypeAt<i>& field() const {
      return FieldAt_<ThisType, i>::field(*this);
    }

    FieldItemType _field;
  };

  template <typename FieldItemType_>
  struct FieldPack_<FieldItemType_> {
    using FieldItemType            = FieldItemType_;
    using ThisType                 = FieldPack_<FieldItemType_>;
    static constexpr int NumFields = 1;

    template <int i>
    static constexpr int unrolledIdx() {
      return i;
    }

    template <int i>
    static constexpr int UnrolledIdx = unrolledIdx<i>();

    template <int i>
    using TypeAt = typename TypeAt_<ThisType, i>::FieldItemType;

    template <int i>
    inline TypeAt<i>& field() {
      return FieldAt_<ThisType, i>::field(*this);
    }

    template <int i>
    inline const TypeAt<i>& field() const {
      return FieldAt_<ThisType, i>::field(*this);
    }

    FieldItemType _field;
  };

  // derived tuple: the types are accessible with indices starting from 0
  // first are the types in the derived object, then in the remaining objects

  // recursive type definition for tuple objects
  template <typename FieldPackBaseType_, typename FieldItemType_, typename... FieldRestType_>
  struct FieldPackDerived_ : public FieldPackDerived_<FieldPackBaseType_, FieldRestType_...> {
    using FieldItemType     = FieldItemType_;
    using FieldPackBaseType = FieldPackBaseType_;
    using ThisType = FieldPackDerived_<FieldPackBaseType_, FieldItemType_, FieldRestType_...>;
    using FieldPackRestType = FieldPackDerived_<FieldPackBaseType, FieldRestType_...>;

    static constexpr int NumOuterFields = 1 + sizeof...(FieldRestType_);
    static constexpr int NumFields      = NumOuterFields + FieldPackBaseType::NumFields;

    template <int i>
    static constexpr int unrolledIdx() {
      return (i < FieldPackBaseType::NumFields)
               ? NumOuterFields + FieldPackBaseType::template unrolledIdx<i>()
               : i - FieldPackBaseType::NumFields;
    }

    template <int i>
    static constexpr int UnrolledIdx = unrolledIdx<i>();

    template <int i>
    using TypeAt = typename TypeAt_<ThisType, UnrolledIdx<i>>::FieldItemType;

    template <int i>
    inline TypeAt<i>& field() {
      return FieldAt_<ThisType, UnrolledIdx<i>>::field(*this);
    }

    template <int i>
    inline const TypeAt<i>& field() const {
      return FieldAt_<ThisType, UnrolledIdx<i>>::field(*this);
    }

    FieldItemType _field;
  };

  template <typename FieldPackBaseType_, typename FieldItemType_>
  struct FieldPackDerived_<FieldPackBaseType_, FieldItemType_> : public FieldPackBaseType_ {
    using FieldItemType     = FieldItemType_;
    using FieldPackBaseType = FieldPackBaseType_;
    using ThisType          = FieldPackDerived_<FieldPackBaseType_, FieldItemType_>;
    using FieldPackRestType = FieldPackBaseType;

    static constexpr int NumOuterFields = 1;
    static constexpr int NumFields      = NumOuterFields + FieldPackBaseType::NumFields;

    template <int i>
    static constexpr int unrolledIdx() {
      return (i < FieldPackBaseType::NumFields)
               ? NumOuterFields + FieldPackBaseType::template unrolledIdx<i>()
               : i - FieldPackBaseType::NumFields;
    }

    template <int i>
    static constexpr int UnrolledIdx = unrolledIdx<i>();

    template <int i>
    using TypeAt = typename TypeAt_<ThisType, UnrolledIdx<i>>::FieldItemType;

    template <int i>
    inline TypeAt<i>& field() {
      return FieldAt_<ThisType, UnrolledIdx<i>>::field(*this);
    }

    template <int i>
    inline const TypeAt<i>& field() const {
      return FieldAt_<ThisType, UnrolledIdx<i>>::field(*this);
    }

    FieldItemType _field;
  };


} // namespace srrg2_core
