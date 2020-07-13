#ifndef _MATHEUS_MODEL_2_HPP_
#define _MATHEUS_MODEL_2_HPP_

#include "models/vertex.hpp" 
#include "models/cplex/mip_solution_info.hpp" 
#include "models/gvrp_models/gvrp_instance.hpp" 
#include "models/gvrp_models/gvrp_afs_tree.hpp" 
#include "models/gvrp_models/gvrp_solution.hpp" 
#include "models/gvrp_models/cplex/gvrp_model.hpp" 
#include "models/gvrp_models/cplex/matheus_model_2/user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/heuristic_callback.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/extra_constraint.hpp"

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
      namespace matheus_model_2 { 
        class User_constraint;
        class Heuristic_callback;
        class Lazy_constraint;
        class Preprocessing;
        class Extra_constraint;
        class Matheus_model_2 : public Gvrp_model {
          public:
            explicit Matheus_model_2(const Gvrp_instance& gvrp_instance, unsigned int time_limit); 
            pair<Gvrp_solution, Mip_solution_info> run();
            Matrix3DVar y;
            Matrix2DVar x;
            IloNumVarArray t;
            IloNumVarArray e;
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
            unsigned long int nGreedyLP;
            unsigned int BPPTimeLimit;
            long int levelGreedyLPHeuristic;
            long int levelSubcycleCallback;
            int nRoutesLB;
            int nPreprocessings1;
            int nPreprocessings2;
            int nPreprocessings3;
            int nPreprocessings4;
            int nImprovedMSTNRoutesLB;
            int nBPPNRoutesLB;
            double solLB;
            double psi;
            double lambda;
            double alpha;
            double time(int i, int f, int j);
            double time(int i, int j);
            double customersFuel(int i, int j);
            double customerToAfsFuel(int i, int f);
            double afsToCustomerFuel(int f, int i);
            double M1(int i, int f, int j);
            double M2(int i, int j);
          protected:
            void createVariables();
            void createObjectiveFunction();
            void createModel();
            virtual void extraStepsAfterModelCreation();
            void setCustomParameters();
            void fillVals();
            void createGvrp_solution();
            void endVars();
        };
      }
    }
  }
}

#endif
