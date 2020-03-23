#ifndef LAZY_CONSTRAINT_INSTANCE_GENERATION_HPP_
#define LAZY_CONSTRAINT_INSTANCE_GENERATION_HPP_

#include "models/instance_generation_model/instance_generation_model.hpp" 

#include <ilcplex/ilocplex.h>

using namespace models;
//using namespace models::instance_generation_model;

namespace models {
  namespace instance_generation_model {
    class Instance_generation_model;
    class Lazy_constraint : public IloCplex::LazyConstraintCallbackI {
      public:
         explicit Lazy_constraint (Instance_generation_model& instance_generation_model);
      protected:
        Instance_generation_model& instance_generation_model;
    };    
  }
} 

#endif
