#ifndef QUADRATIC_MODEL_HPP_
#define QUADRATIC_MODEL_HPP_

#include <list>
#include <models/gvrp_instance.hpp>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace utils {
  namespace cplex {
//    class Quadratic_model_subcycle_constraint_callback : public IloCplex::LazyConstraintCallbackI {
//      public:
//        explicit Quadratic_model_subcycle_constraint_callback(IloEnv env, Gvrp_instance gvrp_instance); 
//        IloCplex::CallbackI* duplicateCallback() const;
//        void main(void);
//        IloExprArray lhs;
//        IloNumArray rhs;
//        IloNum eps;
//    };

    class Quadratic_model {
      public:
        explicit Quadratic_model(Gvrp_instance gvrp_instance); 
        list<list<Vertex> > run();
        Gvrp_instance gvrp_instance;
    };

  } 
}
#endif
