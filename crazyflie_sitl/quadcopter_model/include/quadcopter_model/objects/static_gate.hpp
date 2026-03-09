#pragma once

#include "quadcopter_model/objects/static_object.hpp"

namespace quadcopter {
class StaticGate : public StaticObject {
 public:
  StaticGate(const std::string& id, const std::string& prefab_id = "rpg_gate")
    : StaticObject(id, prefab_id) {}
  ~StaticGate() {}
};

}  // namespace quadcopter
