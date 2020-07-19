#ifndef _MATHEUS_MODEL_HPP_
#define _MATHEUS_MODEL_HPP_

#include "models/vertex.hpp" 
#include "models/cplex/mip_solution_info.hpp" 
#include "models/gvrp_models/gvrp_instance.hpp" 
#include "models/gvrp_models/gvrp_afs_tree.hpp" 
#include "models/gvrp_models/gvrp_solution.hpp" 
#include "models/gvrp_models/cplex/gvrp_model.hpp" 
#include "models/gvrp_models/cplex/matheus_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model/heuristic_callback.hpp"
#include "models/gvrp_models/cplex/matheus_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model/extra_constraint.hpp"

#include <vector>
#include <map>
#include <unordered_set>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

typedef IloArray<IloNumVarArray> Matrix2DVar;
typedef IloArray<IloNumArray> Matrix2DVal;
typedef IloArray<Matrix2DVar> Matrix3DVar;
typedef IloArray<Matrix2DVal> Matrix3DVal;

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model { 
        class User_constraint;
        class Heuristic_callback;
        class Lazy_constraint;
        class Preprocessing;
        class Extra_constraint;
        class Matheus_model : public Gvrp_model {
          public:
            explicit Matheus_model(const Gvrp_instance& gvrp_instance, unsigned int time_limit); 
            ~Matheus_model(); 
            pair<Gvrp_solution, Mip_solution_info> run();
            Matrix3DVar y;
            Matrix2DVar x;
            Matrix2DVar a;
            Matrix2DVar u;
            Matrix2DVar v;
            Matrix2DVar e;
            Matrix2DVar c;
            Matrix2DVar w;
            Matrix2DVar z;
            Matrix3DVal y_vals;
            Matrix2DVal x_vals;
            list<User_constraint*> user_constraints;
            list<Lazy_constraint*> lazy_constraints;
            list<Heuristic_callback*> heuristic_callbacks;
            list<Preprocessing*> preprocessings;
            list<Extra_constraint*> extra_constraints;
            map<int, int> customersC0Indexes;
            map<int, int> afssF0Indexes;
            vector<const Vertex *> c0;
            vector<const Vertex *> f0;
            vector<double> customersMinRequiredFuel;
            vector<double> customersMinRequiredTime;
            vector<vector<double>> gvrpReducedGraphDistances;
            vector<vector<double>> gvrpReducedGraphTimes;
            int nPreprocessings1;
            int nPreprocessings2;
            int nPreprocessings3;
            int nPreprocessings4;
            unsigned long int nGreedyLP;
            int nRoutesLB;
            double solLB;
            double time(int i, int f, int j);
            double time(int i, int j);
            double customersFuel(int i, int j);
            double customerToAfsFuel(int i, int f);
            double afsToCustomerFuel(int f, int i);
          protected:
            void createVariables();
            void createObjectiveFunction();
            void createModel();
            virtual void extraStepsAfterModelCreation();
            void setCustomParameters();
            void fillVals();
            void createGvrp_solution();
            void endVals();
            void endVars();
        };
      }
    }
  }
}

#endif
