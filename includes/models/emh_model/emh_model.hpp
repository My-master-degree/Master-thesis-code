#ifndef _EMH_MODEL_HPP_
#define _EMH_MODEL_HPP_

#include "models/vertex.hpp" 
#include "models/gvrp_instance.hpp" 
#include "models/gvrp_solution.hpp" 
#include "models/mip_solution_info.hpp" 
#include "models/cplex_model.hpp" 
#include "models/emh_model/user_constraint_emh_model.hpp"
#include "models/emh_model/preprocessing_emh_model.hpp"
#include "models/emh_model/extra_constraint_emh_model.hpp"

#include <map>
#include <set>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

typedef IloArray<IloNumVarArray> Matrix2DVar;
typedef IloArray<IloNumArray> Matrix2DVal;

using namespace std;
using namespace models;

namespace models {
  namespace emh_model {
    class User_constraint_emh_model;
    class Preprocessing_emh_model;
    class Extra_constraint_emh_model;
    class EMH_model : public Cplex_model<Gvrp_instance, Gvrp_solution> {
      public:
        explicit EMH_model(Gvrp_instance& gvrp_instance, unsigned int time_limit); 
        pair<Gvrp_solution, Mip_solution_info> run();
        map<int, list<int>> afs_dummies;
        map<int, const Vertex *> all;
        map<int, const Vertex *> dummies;
        set<int> customers;
        Matrix2DVar x;
        IloNumVarArray t;
        IloNumVarArray e;
        Matrix2DVal x_vals;
        list<User_constraint_emh_model*> user_constraints;
        list<Preprocessing_emh_model*> preprocessings;
        list<Extra_constraint_emh_model*> extra_constraints;
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

#endif
