#pragma once

#include "quadcopter_model/common/integrator_base.hpp"

namespace quadcopter {

class IntegratorRK4 : public IntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using IntegratorBase::DynamicsFunction;
  using IntegratorBase::IntegratorBase;

  bool step(const Ref<const Vector<>> initial, const Scalar dt,
            Ref<Vector<>> final) const;
};

}  // namespace quadcopter