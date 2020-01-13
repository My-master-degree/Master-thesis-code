#ifndef COMPACT_MODEL_HPP_
#define COMPACT_MODEL_HPP_

#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/mip_solution_info.hpp"
#include "utils/cplex/lazy_constraint_compact_model.hpp"
#include "utils/cplex/user_constraint_compact_model.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"

#include <map>
#include <list>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

using namespace std;
using namespace models;

typedef map<int, Vertex> IDVertex;
typedef IloArray<IloArray<IloNumVarArray> > Matrix3DVar;
typedef IloArray<IloArray<IloNumArray> > Matrix3DVal;

namespace utils {
  namespace cplex {
    class Lazy_constraint_compact_model;
    class User_constraint_compact_model;
    class Preprocessing_compact_model;
    class Extra_constraint_compact_model;
    class Compact_model {
      public:
        explicit Compact_model(Gvrp_instance& gvrp_instance, unsigned int time_limit); 
        pair<Gvrp_solution, Mip_solution_info> run();
        Gvrp_instance gvrp_instance;
        Gvrp_solution* gvrp_solution;
        unsigned int time_limit;//seconds
        unsigned int max_num_feasible_integer_sol;//0 to 2100000000
        bool VERBOSE;
        IloEnv env;
        IloModel model;
        IDVertex all;
        Matrix3DVar x;
        IloNumVarArray e;
        int ub_edge_visit;
        list<User_constraint_compact_model*> user_constraints;
        list<Preprocessing_compact_model*> preprocessings;
        list<Extra_constraint_compact_model*> extra_constraints;
    private:
        IloCplex cplex;
        Matrix3DVal x_vals;
        Lazy_constraint_compact_model* separation_algorithm();
        void createVariables();
        void createObjectiveFunction();
        void createModel();
        void setCustomParameters();
        void fillX_vals();
        void createGvrp_solution();
    };

  } 
}
#endif
