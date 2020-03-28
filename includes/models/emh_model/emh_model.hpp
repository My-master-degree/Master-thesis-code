#ifndef _EMH_MODEL_HPP_
#define _EMH_MODEL_HPP_

namespace emh_model {
  /*
     class EMH_model : public Cplex_model<Gvrp_instance, Gvrp_solution> {
     public:
     explicit EMH_model(Gvrp_instance& gvrp_instance, unsigned int time_limit); 
     pair<Gvrp_solution, Mip_solution_info> run();
     unsigned int time_limit;//seconds
     unsigned int max_num_feasible_integer_sol;//0 to 2100000000
     bool VERBOSE;
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
     */
}

#endif
