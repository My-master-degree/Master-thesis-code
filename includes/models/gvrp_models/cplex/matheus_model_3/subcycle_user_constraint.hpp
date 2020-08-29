#ifndef SUBCYCLE_USER_CONSTRAINT_MATHEUS_MODEL_3_CPLEX_HPP_
#define SUBCYCLE_USER_CONSTRAINT_MATHEUS_MODEL_3_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_3/user_constraint.hpp" 
#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp" 

#include <ilcplex/ilocplex.h>
#include <unordered_set>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_3 {
        class Subcycle_user_constraint : public User_constraint {
          public:
            Subcycle_user_constraint (Matheus_model_3& matheus_model_3);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
            void fracSeparationSubsets(unordered_set<int> S, unordered_set<int> remainingAFSs, const Matrix2DVal& x_vals);
            int maxNRoutes;
        };
      } 
    } 
  } 
}
#endif
