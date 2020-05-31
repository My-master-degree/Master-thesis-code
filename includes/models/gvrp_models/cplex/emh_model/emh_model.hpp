#ifndef _EMH_MODEL_CPLEX_HPP_
#define _EMH_MODEL_CPLEX_HPP_

#include "models/vertex.hpp" 
#include "models/cplex/mip_solution_info.hpp" 
#include "models/gvrp_models/gvrp_instance.hpp" 
#include "models/gvrp_models/gvrp_afs_tree.hpp" 
#include "models/gvrp_models/gvrp_solution.hpp" 
#include "models/gvrp_models/cplex/gvrp_model.hpp" 
#include "models/gvrp_models/cplex/emh_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/emh_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/emh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/emh_model/extra_constraint.hpp"

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
      namespace emh_model {
        class User_constraint;
        class Lazy_constraint;
        class Preprocessing;
        class Extra_constraint;
        class EMH_model : public Gvrp_model {
          public:
            explicit EMH_model(const Gvrp_instance& gvrp_instance, unsigned int time_limit); 
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
            void endVars();
        };
      }
    }
  }
}

#endif
