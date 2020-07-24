#include "utils/util.hpp"
#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/depth_node_callback.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model/subcycle_user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model/greedy_lp_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing_3.hpp"
#include "models/gvrp_models/cplex/matheus_model/invalid_edge_preprocessing_4.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"

#include <sstream>
#include <list>
#include <float.h>
#include <time.h> 
#include <string> 
#include <unordered_set>

using namespace utils;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::matheus_model;

using namespace std;

Matheus_model::Matheus_model(const Gvrp_instance& instance, unsigned int time_limit) : Gvrp_model(instance, time_limit), c0(vector<const Vertex *> (instance.customers.size() + 1)), f0(vector<const Vertex *> (instance.afss.size() + 1)), nPreprocessings1(0), nPreprocessings2(0), nPreprocessings3(0), nPreprocessings4(0), nGreedyLP(0) {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with metric distances");
  //c_0
  c0[0] = &instance.depot;
  customersC0Indexes[instance.depot.id] = 0;
  int i = 0;
  for (const Vertex& customer : instance.customers) {
    c0[i + 1] = &customer;
    customersC0Indexes[customer.id] = i + 1;
    ++i;
  }
  //f_0
  f0[0] = &instance.depot;
  afssF0Indexes[instance.depot.id] = 0;
  int f = 0;
  for (const Vertex& afs : instance.afss) {
    f0[f + 1] = &afs;
    afssF0Indexes[afs.id] = f + 1;
    ++f;
  }
  //reductions
  const auto& gvrpReducedGraphs = calculateGVRPReducedGraphs (instance, *gvrp_afs_tree);
  gvrpReducedGraphDistances = gvrpReducedGraphs.first;
  gvrpReducedGraphTimes = gvrpReducedGraphs.second;
  //set sol lb
  const auto& closestsDistances = calculateClosestsGVRPCustomers(gvrpReducedGraphDistances, c0);
  solLB = max(calculateGvrpLBByImprovedMST(c0, closestsDistances, gvrpReducedGraphDistances), calculateGvrpLB1(closestsDistances));
  //set n routes lb
  const auto& closestsTimes = calculateClosestsGVRPCustomers(gvrpReducedGraphTimes, c0);
  nRoutesLB = max(int(ceil(calculateGvrpLBByImprovedMSTTime(c0, closestsTimes, gvrpReducedGraphTimes)/instance.timeLimit)), calculateGVRP_BPP_NRoutesLB(instance, c0, closestsTimes, 1000000));
  //user constraints
//  user_constraints.push_back(new Subcycle_user_constraint(*this));
  //preprocessings
  preprocessings.push_back(new Invalid_edge_preprocessing(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_2(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_3(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_4(*this));
  //heuristic callbacks
//  heuristic_callbacks.push_back(new Greedy_lp_heuristic(*this));
  //customer min required fuel
  customersMinRequiredFuel = vector<double> (c0.size()- 1);
  for (size_t i = 1; i < c0.size(); ++i)
    customersMinRequiredFuel[i - 1] = calculateCustomerMinRequiredFuel(instance, *gvrp_afs_tree, *c0[i]);
  //customer min required time 
  customersMinRequiredTime = vector<double> (c0.size());
  for (size_t i = 1; i < c0.size(); ++i)
    customersMinRequiredTime[i] = calculateCustomerMinRequiredTime(instance, *gvrp_afs_tree, *c0[i]);
  customersMinRequiredTime[0] = 0.0;
} 

Matheus_model::~Matheus_model() {
  for (Preprocessing * preprocessing : preprocessings)
    delete preprocessing;  
  for (User_constraint * user_constraint : user_constraints)
    delete user_constraint;  
  for (Lazy_constraint * lazy_constraint : lazy_constraints)
    delete lazy_constraint;  
  for (Extra_constraint * extra_constraint : extra_constraints)
    delete extra_constraint;  
  for (Heuristic_callback * heuristic_callback : heuristic_callbacks)
    delete heuristic_callback;  
}

double Matheus_model::time (int i, int f, int j) {
  return c0[i]->serviceTime + instance.time(c0[i]->id, f0[f]->id) + (f == 0 ? instance.afss.front().serviceTime : f0[f]->serviceTime) + instance.time(f0[f]->id, c0[j]->id);
}

double Matheus_model::time(int i, int j) {
  return c0[i]->serviceTime + instance.time(c0[i]->id, c0[j]->id);
}

double Matheus_model::customersFuel(int i, int j) {
  return instance.fuel(c0[i]->id, c0[j]->id);
}

double Matheus_model::afsToCustomerFuel(int f, int i) {
  return instance.fuel(f0[f]->id, c0[i]->id);
}

double Matheus_model::customerToAfsFuel(int i, int f) {
  return instance.fuel(c0[i]->id, f0[f]->id);
}

pair<Gvrp_solution, Mip_solution_info> Matheus_model::run(){
  //setup
  stringstream output_exception;
  Mip_solution_info mipSolInfo;
  try {
//    cout<<"Creating variables"<<endl;
    createVariables();
//    cout<<"Creating objective function"<<endl;
    createObjectiveFunction();
//    cout<<"Creating model"<<endl;
    createModel();
//    cout<<"Setting parameter"<<endl;
    setCustomParameters();
//    cout<<"Solving model"<<endl;
    struct timespec start, finish;
    double elapsed;
    clock_gettime(CLOCK_MONOTONIC, &start);
    if ( !cplex.solve() ) {
//      env.error() << "Failed to optimize LP." << endl;
      mipSolInfo = Mip_solution_info(-1, cplex.getStatus(), -1, -1);
      endVars();
      //env.end();
      throw mipSolInfo;
    }
    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    cplex.exportModel("cplexcpp.lp");
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    cout<<"Getting x values"<<endl;
    fillVals();
//    cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
    endVals ();
    mipSolInfo = Mip_solution_info(cplex.getMIPRelativeGap(), cplex.getStatus(), elapsed, cplex.getObjValue());
    endVars();
//    env.end();
    return make_pair(*solution, mipSolInfo);
  } catch (IloException& e) {
    output_exception<<"Concert exception caught: " << e<<endl;
    throw output_exception.str();
  } catch (string s) {
    throw s;
  }
}

void Matheus_model::createVariables(){
  y = Matrix3DVar (env, c0.size());
  x = Matrix2DVar (env, c0.size());
  a = Matrix2DVar (env, c0.size() - 1);
  u = Matrix2DVar (env, c0.size());
  v = Matrix2DVar (env, c0.size() - 1);
  try {
    //setting names
    stringstream nameStream;
    for (size_t i = 0; i < c0.size(); ++i) {
      //x, u, c, and w vars
      x[i] = IloNumVarArray (env, c0.size(), 0, 1, IloNumVar::Int);
      u[i] = IloNumVarArray (env, c0.size(), 0, instance.timeLimit, IloNumVar::Float);
      //v and a
      if (i > 0) {
        a[i - 1] = IloNumVarArray (env, c0.size() - 1, 0, instance.vehicleFuelCapacity, IloNumVar::Float);
        v[i - 1] = IloNumVarArray (env, f0.size(), 0, instance.vehicleFuelCapacity, IloNumVar::Float);
        for (size_t f = 0; f < f0.size(); ++f) {
          nameStream<<"v["<<i - 1<<"]["<<f<<"]=edge("<<c0[i]->id<<","<<f0[f]->id<<")";
          v[i - 1][f].setName(nameStream.str().c_str());
          nameStream.clear();
          nameStream.str("");
        }
      }
      for (size_t j = 0; j < c0.size(); ++j) {
        //x
        nameStream<<"x["<<i<<"]["<<j<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
        //u
        nameStream<<"u["<<i<<"]["<<j<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
        u[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
        //a
        if (j > 0 && i > 0) {
          nameStream<<"a["<<i - 1<<"]["<<j - 1<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
          a[i - 1][j - 1].setName(nameStream.str().c_str());
          nameStream.clear();
          nameStream.str("");
        } 
      }
      //y var
      y[i] = Matrix2DVar (env, f0.size());
      for (size_t f = 0; f < f0.size(); ++f) {
        y[i][f] = IloNumVarArray(env, c0.size(), 0, 1, IloNumVar::Int);
        for (size_t j = 0; j < c0.size(); ++j) {
          nameStream<<"y["<<i<<"]["<<f<<"]["<<j<<"]=path("<<c0[i]->id<<","<<f0[f]->id<<","<<c0[j]->id<<")";
          y[i][f][j].setName(nameStream.str().c_str());
          nameStream.clear();
          nameStream.str("");
        }
      }
    }
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating variables");
  }
}

void Matheus_model::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (size_t i = 0; i < c0.size(); ++i) 
      for (size_t j = 0; j < c0.size(); ++j) {
        fo +=  instance.distances[c0[i]->id][c0[j]->id] * x[i][j];
        for (size_t f = 0; f < f0.size(); ++f)
          fo += (instance.distances[c0[i]->id][f0[f]->id] + instance.distances[f0[f]->id][c0[j]->id]) * y[i][f][j];
      }
    model = IloModel (env);
    model.add(IloMinimize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void Matheus_model::createModel() {
  try {
    //preprocessing conditions
    for (Preprocessing* preprocessing : preprocessings)
      preprocessing->add();
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint constraint;
    stringstream constraintName;
    //x_{ii} = 0, \forall v_i \in C_0
    for (size_t i = 0; i < c0.size(); ++i) 
      model.add(x[i][i] == 0);
    //y_{ifi} = 0, \forall v_i \in C_0, \forall v_f \in F
    for (size_t i = 0; i < c0.size(); ++i) 
      for (size_t f = 0; f < f0.size(); ++f)
        model.add(y[i][f][i] == 0);
    //y_{00i} = y_{i00} = 0, \forall v_i \in C_0
    for (size_t i = 0; i < c0.size(); ++i) {
      model.add(y[0][0][i] == 0);
      model.add(y[i][0][0] == 0);
    }
    //\sum_{v_j \in C_0} (x_{ij} + \sum_{v_f \in F_0} y_{ifj}) = 1, \forall v_i \in C
    for (size_t i = 1; i < c0.size(); ++i) {
      for (size_t j = 0; j < c0.size(); ++j) {
        expr += x[i][j];
        for (size_t f = 1; f < f0.size(); ++f)
          expr += y[i][f][j];
      }
      constraint = IloConstraint (expr == 1);
      constraintName<<"# of exiting edges in customer "<<c0[i]->id<<" must exactly one";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} (x_{ji} + \sum_{v_f \in F_0} y_{jfi}) = 1, \forall v_i \in C
    for (size_t i = 1; i < c0.size(); ++i) {
      for (size_t j = 0; j < c0.size(); ++j) {
        expr += x[j][i];
        for (size_t f = 1; f < f0.size(); ++f)
          expr += y[j][f][i];
      }
      constraint = IloConstraint (expr == 1);
      constraintName<<"# of entering edges in customer "<<c0[i]->id<<" must exactly one";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} (x_{0j} + \sum_{v_f \in F_0} y_{0fj}) \leqslant m 
    for (size_t j = 0; j < c0.size(); ++j) {
      expr += x[0][j];
      for (size_t f = 0; f < f0.size(); ++f)
        expr += y[0][f][j];
    }
    constraint = IloConstraint (expr <= instance.maxRoutes);
    constraintName<<"at most "<<instance.maxRoutes<<" routes must be used";
    constraint.setName(constraintName.str().c_str());
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);
    constraintName.clear();
    constraintName.str("");
    //time constraints
    for (size_t i = 0; i < c0.size(); ++i) 
      model.add(u[i][i]);
    //\sum_{v_i \in C \cup \{v_0\}} u_{ji} = \sum_{v_i \in C \cup \{v_0\}} u_{ij} + \sum_{v_i \in C \cup \{v_0\}} t_{ij} x_{ij} + \sum_{v_i \in C \cup \{v_0\}} \sum_{v_f \in F_0} t_{irj} x_{irj}, \forall v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      for (size_t i = 0; i < c0.size(); ++i) {
        expr += u[j][i] - u[i][j] - time(i, j) * x[i][j];
        for (size_t f = 1; f < f0.size(); ++f)
          expr -= time(i, f, j) * y[i][f][j];
      }
      constraint = IloConstraint (expr == 0);
      constraintName<<"time in customer "<<c0[j]->id;
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //u_{0j} = 0, \forall v_j \in C
    for (size_t j = 0; j < c0.size(); ++j) {
      constraint = IloConstraint (u[0][j] == 0);
      constraintName<<"time in depot to "<<c0[j]->id<<" must be 0";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
    }
    //u_{ij} \geqslant max(LB_j^T - t_{ij} - s_j, LB_i^T) x_{ij} + \sum_{v_f \in F} max(LB_j^T - t_{ifj} - s_j, LB_i^T) y_{ifj}, \forall v_i \in C \forall v_j \in C_0
    for (size_t i = 1; i < c0.size(); ++i) 
      for (size_t j = 0; j < c0.size(); ++j) {
//        expr = max(customersMinRequiredTime[j] - time(i, j), customersMinRequiredTime[i]) * x[i][j];
        expr = max(time(0, j) - time(i, j), time(0, i)) * x[i][j];
        for (size_t f = 1; f < f0.size(); ++f)
//          expr += max(customersMinRequiredTime[j] - time(i, f, j), customersMinRequiredTime[i]) * y[i][f][j];
          expr += max(time(0, j) - time(i, f, j), time(0, i)) * y[i][f][j];
        constraint = IloConstraint (u[i][j] >= expr);
        constraintName<<"lb time in "<<c0[i]->id<<" to "<<c0[j]->id;
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    //u_{ij} \leqslant T - t_{ij} - s_j - LB_j^T, \forall v_i \in C \forall v_j \in C_0
    for (size_t i = 1; i < c0.size(); ++i) 
      for (size_t j = 0; j < c0.size(); ++j) {
        expr = min(instance.timeLimit - time(j, 0) - time(i, j), instance.timeLimit - time(i, 0)) * x[i][j];
//        expr = min(instance.timeLimit - c0[j]->serviceTime - customersMinRequiredTime[j] - time(i, j), instance.timeLimit - customersMinRequiredTime[i]) * x[i][j];
        for (size_t f = 1; f < f0.size(); ++f)
          expr += min(instance.timeLimit - time(j, 0) - time(i, f, j), instance.timeLimit - time(i, 0)) * y[i][f][j];
//          expr += min(instance.timeLimit - c0[j]->serviceTime - customersMinRequiredTime[j] - time(i, f, j), instance.timeLimit - c0[i]->serviceTime - customersMinRequiredTime[i]) * y[i][f][j];
        constraint = IloConstraint (u[i][j] <= expr);
//        constraint = IloConstraint (u[i][j] <= max(instance.timeLimit - time(i, j) - c0[j]->serviceTime - customersMinRequiredTime[j], 0.0));
        constraintName<<"ub time in "<<c0[i]->id<<" to "<<c0[j]->id;
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    /*
    //u_{ij} \leqslant T - \sum_{v_f \in F_0} (t_{ifj} * z_{ifj}) - s_j - LB_j^T, \forall v_i \in C \forall v_j \in C_0
    for (size_t i = 1; i < c0.size(); ++i) 
      for (size_t j = 0; j < c0.size(); ++j) {
        for (size_t f = 1; f < f0.size(); ++f)
          expr -= time(i, f, j) * y[i][f][j];
        expr += instance.timeLimit - customersMinRequiredTime[j] - c0[j]->serviceTime;
        constraint = IloConstraint (u[i][j] <= expr);
        constraintName<<"ub2 time in "<<c0[i]->id<<" to "<<c0[j]->id;
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    */
    /*
    //\sum_{v_j \in C_0} u_{ij} + (t_{ij} + s_j + LB_j^T) x_{ij} + \sum_{v_f \in F_0} (t_{ifj} + s_j + LB_j^T) * z_{ifj} \leqslant T, \forall v_i \in C
    for (size_t i = 1; i < c0.size(); ++i) {
      for (size_t j = 0; j < c0.size(); ++j) {
        expr = u[i][j] + (time(i, j) + c0[j]->serviceTime + customersMinRequiredTime[j]) * x[i][j];
        for (size_t f = 1; f < f0.size(); ++f)
          expr += (time(i, f, j) + c0[j]->serviceTime + customersMinRequiredTime[j]) * y[i][f][j];
      }
      constraint = IloConstraint (expr <= instance.timeLimit);
      constraintName<<"ub time in "<<c0[i]->id;
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    */
    //energy constraints
    //\sum_{v_i \in C} a_{i, j} - a_{j, i} = \sum_{v_f \in F_0} v_{jr} + \sum_{v_i \in C} e_{ji} x_{ji} - \sum_{v_i \in C_0} \sum_{v_f \in F} (\beta - e_{fj}) y_{ifj} - (\beta - e_{0j}) x_{0j}, \forall v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      for (size_t f = 0; f < f0.size(); ++f)
        expr -= v[j - 1][f];
      for (size_t i = 0; i < c0.size(); ++i) {
        if (i > 0) 
          expr += a[i - 1][j - 1] - a[j - 1][i - 1] - customersFuel(j, i) * x[j][i];
        for (size_t f = 1; f < f0.size(); ++f)
          expr += (instance.vehicleFuelCapacity - afsToCustomerFuel(f, j)) * y[i][f][j];
      }
      expr += (instance.vehicleFuelCapacity - customersFuel(0, j)) * x[0][j];
      constraint = IloConstraint (expr == 0);
      constraintName<<"customer "<<c0[j]->id<<" energy update";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //a_{ij} >= x_{ij} * max (LB_j^E, LB_i^E - e_{ij}), \forall v_i, v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) 
      for (size_t i = 1; i < c0.size(); ++i) {
        //min_{v_f \in F_0} e_{jf} and min_{v_f \in F_0} e_{if}
        double closestAfsToJ = customerToAfsFuel(j, 0),
               closestAfsToI = customerToAfsFuel(i, 0);
        for (size_t f = 1; f < f0.size(); ++f) {
          closestAfsToJ = min(closestAfsToJ, customerToAfsFuel(j, f));
          closestAfsToI = min(closestAfsToI, customerToAfsFuel(i, f));
        }
        constraint = IloConstraint (a[i - 1][j - 1] >= x[i][j] * max(closestAfsToJ, closestAfsToI - customersFuel(i, j)));
//        constraint = IloConstraint (a[i - 1][j - 1] >= x[i][j] * max(customersMinRequiredFuel[j - 1], customersMinRequiredFuel[i - 1] - customersFuel(i, j)));
        constraintName<<"a["<<i - 1<<"]["<<j - 1<<"] lb";
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        constraintName.clear();
        constraintName.str("");
      }
    //a_{ij} <= x_{ij} * min (\beta - LB_j^E, \beta - LB_i^E - e_{ij}), \forall v_i, v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) 
      for (size_t i = 1; i < c0.size(); ++i) {
        //min_{v_f \in F_0} e_{fj} and min_{v_f \in F_0} e_{fi}
        double closestAfsToJ = afsToCustomerFuel(0, j),
               closestAfsToI = afsToCustomerFuel(0, i);
        for (size_t f = 1; f < f0.size(); ++f) {
          closestAfsToJ = min(closestAfsToJ, afsToCustomerFuel(f, j));
          closestAfsToI = min(closestAfsToI, afsToCustomerFuel(f, i));
        }
        constraint = IloConstraint (a[i - 1][j - 1] <= x[i][j] * min(instance.vehicleFuelCapacity - closestAfsToI - customersFuel(i, j), instance.vehicleFuelCapacity - closestAfsToJ));
//        constraint = IloConstraint (a[i - 1][j - 1] <= x[i][j] * min(instance.vehicleFuelCapacity - customersMinRequiredFuel[i - 1] - customersFuel(i, j), instance.vehicleFuelCapacity - customersMinRequiredFuel[j - 1]));
        constraintName<<"a["<<i - 1<<"]["<<j - 1<<"] ub";
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        constraintName.clear();
        constraintName.str("");
      }
    //v_{j0} \geqslant x_{j0} * e_{j0}, \forall v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      constraint = IloConstraint (v[j - 1][0] >= x[j][0] * customersFuel(j, 0));
      constraintName<<"v["<<j - 1<<"][0] lb";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
    }
    //v_{j0} \leqslant x_{j0} * (\beta - LB_j^E), \forall v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      constraint = IloConstraint (v[j - 1][0] <= x[j][0] * (instance.vehicleFuelCapacity - customersMinRequiredFuel[j - 1]));
      constraintName<<"v["<<j - 1<<"][0] ub";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
    }
    //v_{jr} \geqslant \sum_{v_i \in C_0} z_{jfi} * e_{jf}, \forall v_j \in C, \forall v_f \in F
    for (size_t j = 1; j < c0.size(); ++j) {
      for (size_t f = 1; f < f0.size(); ++f) {
        for (size_t i = 0; i < c0.size(); ++i) 
          expr += y[j][f][i];
        constraint = IloConstraint (v[j - 1][f] >= expr * customerToAfsFuel(j, f));
        constraintName<<"v["<<j - 1<<"]["<<f<<"] lb";
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //v_{jr} \leqslant (\beta - LB_j^E) * \sum_{v_i \in C_0} z_{jfi}, \forall v_j \in C, \forall v_f \in F
    for (size_t j = 1; j < c0.size(); ++j) {
      for (size_t f = 1; f < f0.size(); ++f) {
        for (size_t i = 0; i < c0.size(); ++i) 
          expr += y[j][f][i];
        //min_{v_f \in F_0} e_{jf}
        constraint = IloConstraint (v[j - 1][f] <= expr * (instance.vehicleFuelCapacity - customersMinRequiredFuel[j - 1]));
        constraintName<<"v["<<j - 1<<"]["<<f<<"] ub";
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //no 2 subcycles
    //x_{ij} + x_{ji} + \sum_{v_f \in F} z_{jfi} + z_{ifj} \leqslant 1, \forall v_i, v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      for (size_t i = 1; i < c0.size(); ++i) {
        expr = x[i][j] + x[j][i];
        for (size_t f = 1; f < f0.size(); ++f) 
          expr += y[j][f][i] + y[i][f][j];
        constraint = IloConstraint (expr <= 1);
        constraintName<<"no 2 subcyle between customers "<<i<<", and "<<j;
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //new inequalities
    //solution lb
    for (size_t i = 0; i < c0.size(); ++i) 
      for (size_t j = 0; j < c0.size(); ++j) {
        expr +=  instance.distances[c0[i]->id][c0[j]->id] * x[i][j];
        for (size_t f = 0; f < f0.size(); ++f)
          expr += (instance.distances[c0[i]->id][f0[f]->id] + instance.distances[f0[f]->id][c0[j]->id]) * y[i][f][j];
      }
    constraint = IloConstraint (expr >= solLB);
    constraint.setName("solution LB");
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);
    /*
    //solution fuel lb alpha 1
    for (size_t i = 0; i < c0.size(); ++i) {
      for (size_t f = 0; f < f0.size(); ++f)
        expr -= alpha * y[0][f][i];
      for (size_t j = 0; j < c0.size(); ++j) {
        expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
        for (size_t f = 0; f < f0.size(); ++f)
          expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - instance.vehicleFuelCapacity/2.0) * y[i][f][j];
      }
    }
    constraint = IloConstraint (expr >= 0);
    constraint.setName("solution fuel LB alpha 1");
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);
    //solution fuel lb alpha 2
    for (size_t i = 0; i < c0.size(); ++i) {
      for (size_t f = 0; f < f0.size(); ++f)
        expr -= alpha * y[i][f][0];
      for (size_t j = 0; j < c0.size(); ++j) {
        expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
        for (size_t f = 0; f < f0.size(); ++f)
          expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - instance.vehicleFuelCapacity/2.0) * y[i][f][j];
      }
    }
    constraint = IloConstraint (expr >= 0);
    constraint.setName("solution fuel LB alpha 2");
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);
    //solution fuel lb alpha 3
    for (size_t i_ = 1; i_ < c0.size(); ++i_) 
      for (size_t j_ = 1; j_ < c0.size(); ++j_) {
        for (size_t i = 0; i < c0.size(); ++i) {
          for (size_t j = 0; j < c0.size(); ++j) {
            expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
            for (size_t f = 0; f < f0.size(); ++f)
              expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - instance.vehicleFuelCapacity/2.0) * y[i][f][j];
          }
        }
        for (size_t f = 0; f < f0.size(); ++f)
          expr -= alpha * y[i_][f][j_];
        constraint = IloConstraint (expr >= 0);
        constraintName<<"solution fuel LB alpha 3 in customer"<<c0[i_]->id<<" and "<<c0[j_]->id;
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    //solution fuel lb lambda 1
    for (size_t i = 0; i < c0.size(); ++i) {
      expr += psi * x[i][0];
      for (size_t f = 0; f < f0.size(); ++f)
        expr -= 2 * lambda * y[0][f][i] + psi * y[0][f][i];
      for (size_t j = 0; j < c0.size(); ++j) {
        expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
        for (size_t f = 0; f < f0.size(); ++f)
          expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - psi) * y[i][f][j];
      }
    }
    constraint = IloConstraint (expr >= 0);
    constraint.setName("solution fuel LB lambda 1");
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);
    //solution fuel lb lambda 2
    for (size_t i = 0; i < c0.size(); ++i) {
      expr += psi * x[0][i];
      for (size_t f = 0; f < f0.size(); ++f)
        expr -= (2 * lambda + psi) * y[i][f][0];
      for (size_t j = 0; j < c0.size(); ++j) {
        expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
        for (size_t f = 0; f < f0.size(); ++f)
          expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - psi) * y[i][f][j];
      }
    }
    constraint = IloConstraint (expr >= 0);
    constraint.setName("solution fuel LB lambda 2");
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);
    //solution fuel lb lambda 3
    for (size_t i_ = 0; i_ < c0.size(); ++i_) 
      for (size_t j_ = 0; j_ < c0.size(); ++j_) {
        for (size_t f = 0; f < f0.size(); ++f)
          expr -= 2 * lambda * y[i_][f][j_];
        for (size_t i = 0; i < c0.size(); ++i) {
          expr += psi * x[0][i];
          for (size_t f = 0; f < f0.size(); ++f)
            expr += psi * y[0][f][i];
          for (size_t j = 0; j < c0.size(); ++j) {
            expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
            for (size_t f = 0; f < f0.size(); ++f)
              expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - psi) * y[i][f][j];
          }
        }
        constraint = IloConstraint (expr >= 0);
        constraintName<<"solution fuel LB lambda 3 in customers "<<c0[i_]->id<<" and "<<c0[j_]->id;
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    */
    //n routes LB
    for (size_t i = 0; i < c0.size(); ++i) {
      expr += x[0][i];
      for (size_t f = 0; f < f0.size(); ++f)
        expr += y[0][f][i];
    }
    constraint = IloConstraint (expr >= nRoutesLB);
    constraint.setName("nRoutes LB");
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);









  /*
    vector<vector<int>> routes_ = {
      {0, 4, 15, 12, 10, 15, 11, 15, 1, 0}, 
      {0, 7, 13, 7, 0}, 
      {0, 7, 14, 5, 7, 9, 16, 7, 19, 0}, 
      {0, 8, 17, 18, 8, 3, 2, 7, 6, 0}, 
    };
    list<list<Vertex>> routes;
    for (const vector<int>& route_ : routes_) {
      list<Vertex> route;
      for (int node : route_) 
        if (customersC0Indexes.count(node))
          route.push_back(Vertex(*c0[customersC0Indexes[node]]));
        else if (afssF0Indexes.count(node))
          route.push_back(Vertex(*f0[afssF0Indexes[node]]));
      routes.push_back(route);
    }
    double currFuel, 
           currTime;
    for (const list<Vertex>& route : routes) {
      currFuel = instance.vehicleFuelCapacity;
      currTime = 0.0;
      list<Vertex>::const_iterator curr = route.begin(), 
        prev = curr;
      for (++curr; curr != route.end(); prev = curr, ++curr) {
        auto currIndex = customersC0Indexes.find(curr->id);
        int i = customersC0Indexes[prev->id];
        //is a customer
        if (currIndex != customersC0Indexes.end()) {
          int j = currIndex->second;
          model.add(x[i][j] == 1);
          model.add(u[i][j] == currTime);
          if (i > 0 && j > 0) 
            model.add(a[i - 1][j - 1] == currFuel - customersFuel(i, j));
          else if (i > 0) 
            model.add(v[i - 1][0] == currFuel);
          currFuel -= customersFuel(i, j);
          currTime += time(i, j);
        } else {
          //is an afs 
          int f = afssF0Indexes[curr->id];
          ++curr;
          int j = customersC0Indexes[curr->id];
          model.add(y[i][f][j] == 1);
          model.add(u[i][j] == currTime);
          if (i > 0) 
            model.add(v[i - 1][f] == currFuel);
          currTime += time(i, f, j);
          currFuel = instance.vehicleFuelCapacity - afsToCustomerFuel(f, j);
        }
      }
    }
    */








    //extra constraints
    for (Extra_constraint* extra_constraint : extra_constraints) 
      extra_constraint->add();
    //init
    cplex = IloCplex(model);
    //lazy constraints
    for (Lazy_constraint* lazy_constraint : lazy_constraints)
      cplex.use(lazy_constraint);
    //user cuts
    for (User_constraint* user_constraint : user_constraints)
      cplex.use(user_constraint);
    //heuristic callback
    for (Heuristic_callback* heuristic_callback : heuristic_callbacks)
      cplex.use(heuristic_callback);
    //extra steps
    extraStepsAfterModelCreation();
    //depth node callback
    depth_node_callback = new Depth_node_callback(env);
    cplex.use(depth_node_callback);
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

void Matheus_model::extraStepsAfterModelCreation() {
  //
}

void Matheus_model::setCustomParameters(){
  try{
    setParameters();
    //for the user cut callback, although this formulation does not make use of lazy constraints, (this parameter is being defined to standarize the experiments (since the cubic formulations makes use of user constraints)
    cplex.setParam(IloCplex::Param::Preprocessing::Linear, 0);
    //for the lazy constraint callback, although this formulation does not make use of lazy constraints, (this parameter is being defined to standarize the experiments (since the cubic formulations makes use of lazy constraints)
    cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 2);
    //this parameter is being defined to standarize the experiments (since the cubic formulations makes use of lazy constraints)
    cplex.setParam(IloCplex::Param::Threads, 1);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void Matheus_model::fillVals(){
  //getresult
  Matrix2DVal u_vals = Matrix2DVal (env, c0.size());
  try{
    x_vals = Matrix2DVal (env, c0.size());
    y_vals = Matrix3DVal (env, c0.size());
    for (size_t i = 0; i < c0.size(); ++i){
      x_vals[i] = IloNumArray (env, c0.size(), 0, 1, IloNumVar::Int);
      y_vals[i] = Matrix2DVal (env, f0.size());
      u_vals[i] = IloNumArray (env, c0.size(), 0, 1, IloNumVar::Float);
      cplex.getValues(u_vals[i], u[i]);
      cplex.getValues(x_vals[i], x[i]);
      for (size_t f = 0; f < f0.size(); ++f){
        y_vals[i][f] = IloNumArray(env, c0.size(), 0, 1, IloNumVar::Int);
        cplex.getValues(y_vals[i][f], y[i][f]);
      }
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
  /*
  cout<<" ";
  for (size_t i = 0; i < c0.size(); ++i){
    cout<<" ";
    if (i <=9)
      cout<<" ";
    cout<<i;
  }
  cout<<endl;
  for (size_t i = 0; i < c0.size(); ++i){
    cout<<i<<" ";
    if (i <= 9)
      cout<<" ";
    for (size_t j = 0; j < c0.size(); ++j) {
      cout<<abs(int(u_vals[i][j] * 100 + .5)/100.0)<<"  ";
    }
    cout<<endl;
  }
  cout<<" ";
  for (size_t i = 0; i < c0.size(); ++i){
    cout<<" ";
    if (i <=9)
      cout<<" ";
    cout<<i;
  }
  cout<<endl;
  for (size_t i = 0; i < c0.size(); ++i){
    cout<<i<<" ";
    if (i <= 9)
      cout<<" ";
    for (size_t j = 0; j < c0.size(); ++j) {
      cout<<abs(x_vals[i][j])<<"  ";
    }
    cout<<endl;
  }
  for (size_t f = 0; f < f0.size(); ++f){
    cout<<"AFS: "<<f<<endl;
    cout<<" ";
    for (size_t i = 0; i < c0.size(); ++i){
      cout<<" ";
      if (i <=9)
        cout<<" ";
      cout<<i;
    }
    cout<<endl;
    for (size_t i = 0; i < c0.size(); ++i){
      cout<<i<<" ";
      if (i <= 9)
        cout<<" ";
      for (size_t j = 0; j < c0.size(); ++j)
        cout<<abs(y_vals[i][f][j])<<"  ";
      cout<<endl;
    }
  }
  for (size_t i = 0; i < c0.size(); ++i)
    cout<<i<<": "<<c0[i]->id<<endl;
    */
}

void Matheus_model::createGvrp_solution(){
  try{
    list<list<Vertex>> routes;
    list<Vertex> route;
    size_t curr;    
    bool next = false;
    //checking the depot neighboring
    while (true) {
      next = false;
      for (size_t i = 1; i < c0.size() && !next; ++i) {
        if (x_vals[0][i] > INTEGRALITY_TOL) {
          next = true;
          route.push_back(Vertex(*c0[0]));
          x_vals[0][i] = 0;
        } 
        if (!next)
          //on y[j][0][i]
          for (size_t j = 0; j < c0.size(); ++j) 
            if (y_vals[j][0][i] > INTEGRALITY_TOL) {
              next = true;
              route.push_back(Vertex(*c0[0]));
              y_vals[j][0][i] = 0;
              break;
            }
        if (!next)
          //on y[0][f][i]
          for (size_t f = 0; f < f0.size(); ++f)
            if (y_vals[0][f][i] > INTEGRALITY_TOL) {
              next = true;
              route.push_back(Vertex(*c0[0]));
              route.push_back(Vertex(*f0[f]));
              y_vals[0][f][i] = 0;
              break;
            }
        if (next) {
          route.push_back(Vertex(*c0[i]));
          curr = i;
        }
      }
      if (!next)
        break;
      //dfs
      while (curr != 0) {
        for (size_t i = 0; i < c0.size(); ++i) {
          next = false;
          if (x_vals[curr][i] > INTEGRALITY_TOL) {
            next = true;
            route.push_back(Vertex(*c0[i]));
            x_vals[curr][i] = 0;
            curr = i;
            break;
          } else {
            for (size_t f = 0; f < f0.size(); ++f)
              if (y_vals[curr][f][i] > INTEGRALITY_TOL) {
                next = true;
                route.push_back(Vertex(*f0[f]));
                route.push_back(Vertex(*c0[i]));
                y_vals[curr][f][i] = 0;
                curr = i;
                i = c0.size() - 1;
                break;
              }
          }
        }
        //edge case
        if (!next) {
          route.push_back(Vertex(*c0[0]));
          curr = 0;
        }
      }
      routes.push_back(route);
      route = list<Vertex> ();
    }
    solution = new Gvrp_solution(routes, instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void Matheus_model::endVals () {
  //end vals
  for (size_t i = 0; i < c0.size(); ++i) {
    for (size_t f = 0; f < f0.size(); ++f)
      y_vals[i][f].end();
    y_vals[i].end();
    x_vals[i].end();
  }
  y_vals.end();
  x_vals.end();
}

void Matheus_model::endVars(){
  for (size_t i = 0; i < c0.size(); ++i) {
    if (i > 0) {
      a[i - 1].end();
      v[i - 1].end();
    }
    u[i].end();
    x[i].end();
    for (size_t f = 0; f < f0.size(); ++f) 
      y[i][f].end();
    y[i].end();
  }
  x.end();
  y.end();
  a.end();
  u.end();
  v.end();
}
