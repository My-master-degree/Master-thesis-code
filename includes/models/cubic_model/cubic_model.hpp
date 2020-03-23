#ifndef CUBIC_MODEL_HPP_
#define CUBIC_MODEL_HPP_

#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/mip_solution_info.hpp"
#include "models/cplex_model.hpp"
#include "models/cubic_model/lazy_constraint_cubic_model.hpp"
#include "models/cubic_model/user_constraint_cubic_model.hpp"
#include "models/cubic_model/preprocessing_cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"

#include <map>
#include <list>
#include <set>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

using namespace std;
using namespace models;

typedef map<int, Vertex> IDVertex;
typedef IloArray<IloArray<IloNumVarArray> > Matrix3DVar;
typedef IloArray<IloArray<IloNumArray> > Matrix3DVal;

namespace models {
  namespace cubic_model {
    class Lazy_constraint_cubic_model;
    class User_constraint_cubic_model;
    class Preprocessing_cubic_model;
    class Extra_constraint_cubic_model;
    class Cubic_model : public Cplex_model<Gvrp_instance, Gvrp_solution> {
      public:
        explicit Cubic_model(Gvrp_instance& gvrp_instance, unsigned int time_limit); 
        pair<Gvrp_solution, Mip_solution_info> run();
        unsigned int time_limit;//seconds
        unsigned int max_num_feasible_integer_sol;//0 to 2100000000
        bool VERBOSE;
        bool ALLOW_SUBCYCLE_USER_CUT;
        IloEnv env;
        IloModel model;
        IDVertex all;
        set<int> customers;
        Matrix3DVar x;
        IloNumVarArray e;
        int ub_edge_visit;
        list<User_constraint_cubic_model*> user_constraints;
        list<Preprocessing_cubic_model*> preprocessings;
        list<Extra_constraint_cubic_model*> extra_constraints;
    protected:
        Matrix3DVal x_vals;
        Lazy_constraint_cubic_model* separation_algorithm();
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
