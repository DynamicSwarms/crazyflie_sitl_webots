#pragma once

#include "quadcopter_model/common/types.hpp"

namespace quadcopter {

class DynamicsBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  using DynamicsFunction =
    std::function<bool(const Ref<const Vector<>>, Ref<Vector<>>)>;

  DynamicsBase();
  virtual ~DynamicsBase();

  // public get function
  virtual DynamicsFunction getDynamicsFunction() const = 0;

 private:
};

}  // namespace quadcopter
