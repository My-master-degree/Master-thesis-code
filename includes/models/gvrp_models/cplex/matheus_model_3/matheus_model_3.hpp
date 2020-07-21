#ifndef _MATHEUS_MODEL_3_CPLEX_HPP_
#define _MATHEUS_MODEL_3_CPLEX_HPP_

#include "models/vertex.hpp" 
#include "models/cplex/mip_solution_info.hpp" 
#include "models/gvrp_models/gvrp_instance.hpp" 
#include "models/gvrp_models/gvrp_afs_tree.hpp" 
#include "models/gvrp_models/gvrp_solution.hpp" 
#include "models/gvrp_models/cplex/gvrp_model.hpp" 
#include "models/gvrp_models/cplex/matheus_model_3/user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/extra_constraint.hpp"

#include <map>
#include <set>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

typedef IloArray<IloNumVarArray> Matrix2DVar;
typedef IloArray<IloNumArray> Matrix2DVal;

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_3 {
        class User_constraint;
        class Lazy_constraint;
        class Preprocessing;
        class Extra_constraint;
        class Matheus_model_3 : public Gvrp_model {
          public:
            explicit Matheus_model_3(const Gvrp_instance& gvrp_instance, unsigned int time_limit); 
            ~Matheus_model_3(); 
            pair<Gvrp_solution, Mip_solution_info> run();
            map<int, list<int>> afs_dummies;
            map<int, const Vertex *> dummies;
            const Vertex * depotDummy;
            Matrix2DVar x;
            IloNumVarArray t;
            IloNumVarArray e;
            Matrix2DVal x_vals;
            list<User_constraint*> user_constraints;
            list<Preprocessing*> preprocessings;
            list<Extra_constraint*> extra_constraints;
          protected:
            void createVariables();
            void createObjectiveFunction();
            void createModel();
            virtual void extraStepsAfterModelCreation();
            void setCustomParameters();
            void fillX_vals();
            void createGvrp_solution();
            void endVals();
            void endVars();
        };
      }
    }
  }
}

#endif
