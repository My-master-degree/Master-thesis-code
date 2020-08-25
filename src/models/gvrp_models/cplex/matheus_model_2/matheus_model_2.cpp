#include "utils/util.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/subcycle_user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/greedy_lp_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/invalid_edge_preprocessing_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/invalid_edge_preprocessing_4.hpp"
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
using namespace models::gvrp_models::cplex::matheus_model_2;

using namespace std;

Matheus_model_2::Matheus_model_2(const Gvrp_instance& instance, unsigned int time_limit) : Gvrp_model(instance, time_limit), c0(vector<const Vertex *> (instance.customers.size() + 1)), f0(vector<const Vertex *> (instance.afss.size() + 1)), nGreedyLP(0), BPPTimeLimit(100000000), levelSubcycleCallback(0), nPreprocessings1(0), nPreprocessings2(0), nPreprocessings3(0), nPreprocessings4(0), nImprovedMSTNRoutesLB(0), nBPPNRoutesLB(0), RELAXED(false) {
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
  nRoutesLB = max(int(ceil(calculateGvrpLBByImprovedMSTTime(c0, closestsTimes, gvrpReducedGraphTimes)/instance.timeLimit)), calculateGVRP_BPP_NRoutesLB(instance, c0, closestsTimes, BPPTimeLimit));
  //user constraints
  user_constraints.push_back(new Subcycle_user_constraint(*this));
  //preprocessings
  preprocessings.push_back(new Invalid_edge_preprocessing(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_2(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_3(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_4(*this));
  //heuristic callbacks
  heuristic_callbacks.push_back(new Greedy_lp_heuristic(*this));
  //customer min required fuel
  customersMinRequiredFuel = vector<double> (c0.size()- 1);
  for (int i = 1; i < c0.size(); ++i)
    customersMinRequiredFuel[i - 1] = calculateCustomerMinRequiredFuel(instance, *gvrp_afs_tree, *c0[i]);
  //customer min required time 
  customersMinRequiredTime = vector<double> (c0.size());
  for (int i = 1; i < c0.size(); ++i)
    customersMinRequiredTime[i] = calculateCustomerMinRequiredTime(instance, *gvrp_afs_tree, *c0[i]);
  customersMinRequiredTime[0] = 0.0;
} 

double Matheus_model_2::time (int i, int f, int j) {
  return c0[i]->serviceTime + instance.time(c0[i]->id, f0[f]->id) + (f == 0 ? instance.afss.front().serviceTime : f0[f]->serviceTime) + instance.time(f0[f]->id, c0[j]->id);
}

double Matheus_model_2::time(int i, int j) {
  return c0[i]->serviceTime + instance.time(c0[i]->id, c0[j]->id);
}

double Matheus_model_2::customersFuel(int i, int j) {
  return instance.fuel(c0[i]->id, c0[j]->id);
}

double Matheus_model_2::afsToCustomerFuel(int f, int i) {
  return instance.fuel(f0[f]->id, c0[i]->id);
}

double Matheus_model_2::customerToAfsFuel(int i, int f) {
  return instance.fuel(c0[i]->id, f0[f]->id);
}

double Matheus_model_2::M1(int i, int f, int j) {
  return instance.timeLimit + time(i, j) + time(i, f, j) - c0[i]->serviceTime - customersMinRequiredTime[i] - customersMinRequiredTime[j];
}

double Matheus_model_2::M2(int i, int j) {
  return instance.vehicleFuelCapacity + customersFuel(i, j) - customersMinRequiredFuel[i - 1] - customersMinRequiredFuel[j - 1];
}

pair<Gvrp_solution, Mip_solution_info> Matheus_model_2::run(){
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
//      env.end();
      throw mipSolInfo;
    }
    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    mipSolInfo = Mip_solution_info(cplex.getMIPRelativeGap(), cplex.getStatus(), elapsed, cplex.getObjValue());
    if (RELAXED) {
      endVars ();
      throw mipSolInfo;
    }
//    cplex.exportModel("cplexcpp.lp");
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    cout<<"Getting x values"<<endl;
    fillVals();
//    cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
    endVals ();
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

void Matheus_model_2::createVariables(){
  y = Matrix3DVar (env, c0.size());
  x = Matrix2DVar (env, c0.size());
  e = IloNumVarArray (env, c0.size() - 1, 0, instance.vehicleFuelCapacity, IloNumVar::Float);
  t = IloNumVarArray (env, c0.size() - 1, 0, instance.timeLimit, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for (int i = 1; i < c0.size(); ++i) {
      //e var
      nameStream<<"e["<<i - 1<<"]=energy of customer "<<c0[i]->id;
      e[i - 1].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //t var
      nameStream<<"t["<<i - 1<<"]=time of customer "<<c0[i]->id;
      t[i - 1].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
    }
    for (int i = 0; i < c0.size(); ++i) {
      //x vars
      x[i] = IloNumVarArray (env, c0.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
      for (int j = 0; j < c0.size(); ++j) {
        nameStream<<"x["<<i<<"]["<<j<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
      }
      //y var
      y[i] = Matrix2DVar (env, f0.size());
      for (int f = 0; f < f0.size(); ++f) {
        y[i][f] = IloNumVarArray(env, c0.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
        for (int j = 0; j < c0.size(); ++j) {
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

void Matheus_model_2::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (int i = 0; i < c0.size(); ++i) 
      for (int j = 0; j < c0.size(); ++j) {
        fo +=  instance.distances[c0[i]->id][c0[j]->id] * x[i][j];
        for (int f = 0; f < f0.size(); ++f)
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

void Matheus_model_2::createModel() {
  try {
    //preprocessing conditions
    for (Preprocessing* preprocessing : preprocessings)
      preprocessing->add();
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //x_{ii} = 0, \forall v_i \in C_0
    for (int i = 0; i < c0.size(); ++i) 
      model.add(x[i][i] == 0);
    //y_{ifi} = 0, \forall v_i \in C_0, \forall v_f \in F
    for (int i = 0; i < c0.size(); ++i) 
      for (int f = 0; f < f0.size(); ++f)
        model.add(y[i][f][i] == 0);
    //y_{00i} = y_{i00} = 0, \forall v_i \in C_0
    for (int i = 0; i < c0.size(); ++i) {
      model.add(y[0][0][i] == 0);
      model.add(y[i][0][0] == 0);
    }
    //\sum_{v_j \in C_0} (x_{ij} + \sum_{v_f \in F_0} y_{ifj}) = 1, \forall v_i \in C
    for (int i = 1; i < c0.size(); ++i) {
      for (int j = 0; j < c0.size(); ++j) {
        expr += x[i][j];
        for (int f = 0; f < f0.size(); ++f)
          expr += y[i][f][j];
      }
      c = IloConstraint (expr == 1);
      constraintName<<"customer "<<c0[i]->id<<" must be visited exactly once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} ((x_{ij} - x_{ji}) + \sum_{v_f \in F_0} (y_{ifj} - y_{jfi})) = 0, \forall v_i \in C_0 
    for (int i = 0; i < c0.size(); ++i) {
      for (int j = 0; j < c0.size(); ++j) {
        expr += x[i][j] - x[j][i];
        for (int f = 0; f < f0.size(); ++f)
          expr += y[i][f][j] - y[j][f][i];
      }
      c = IloConstraint (expr == 0);
      constraintName<<c0[i]->id<<" flow conservation ";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} (x_{0j} + \sum_{v_f \in F_0} y_{0fj}) \leqslant m 
    for (int j = 0; j < c0.size(); ++j) {
      expr += x[0][j];
      for (int f = 0; f < f0.size(); ++f)
        expr += y[0][f][j];
    }
    c = IloConstraint (expr <= instance.maxRoutes);
    constraintName<<instance.maxRoutes<<" routes must be used";
    c.setName(constraintName.str().c_str());
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    constraintName.clear();
    constraintName.str("");
    // \tau_i - \tau_j + (M1_{ifj} - t_{ifj}) * x_{ij}
    //                + (M1_{ifj} - t_{ifj} - t_{ij} - t_{ji}) * x_{ji}
    //                + (M1_{ifj} - t_{ij}) * y_{ifj}
    //                + (M1_{ifj} - t_{ij} - t_{ifj} - t_{jfi}) * y_{jfi}
    //                \leqslant M1_{ifj} - t_{ij} - t_{ifj} 
    for (int i = 1; i < c0.size(); ++i)
      for (int j = 1; j < c0.size(); ++j) 
        for (int f = 0; f < f0.size(); ++f) {
          expr = t[i - 1] - t[j - 1] + (M1(i, f, j) - time(i, f, j)) * x[i][j] 
                              + (M1(i, f, j) - time(i, f, j) - time(i, j) - time(j, i)) * x[j][i]
                             + (M1(i, f, j) - time(i, j)) * y[i][f][j] 
                              + (M1(i, f, j) - time(i, j) - time(i, f, j) - time(j, f, i)) * y[j][f][i]
                              - (M1(i, f, j) - time(i, j) - time(i, f, j));
          c = IloConstraint (expr <= 0);
          constraintName<<"time path ("<<c0[i]->id<<", "<<f0[f]->id<<", "<<c0[j]->id<<")";
          c.setName(constraintName.str().c_str());
          model.add(c);
          expr.end();
          expr = IloExpr(env);
          constraintName.clear();
          constraintName.str("");
        }
    // \tau_j \geqslant t_{0j}) * x_{0j} + \sum_{v_f \in F_0} y_{0fj} t_{0fj} \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = time(0, j) * x[0][j];
      for (int f = 0; f < f0.size(); ++f) 
        expr += time(0, f, j) * y[0][f][j];
      c = IloConstraint (t[j - 1] >= expr);
      constraintName<<"customer "<<c0[j]->id<<" time lb";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // \tau_j \leqslant T_{max} - (T_{max}^- t_{0j}) * x_{0j} - \sum_{v_f \in F_0} y_{0fj} * (T_{max} - t_{0fj}) \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = instance.timeLimit - (instance.timeLimit - time(0, j)) * x[0][j];
      for (int f = 1; f < f0.size(); ++f) 
        expr -= (instance.timeLimit - time(0, f, j)) * y[0][f][j];
      c = IloConstraint (t[j - 1] <= expr);
      constraintName<<"customer "<<c0[j]->id<<" time ub";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // \tau_j \leqslant T_{max} - t_{j0} * x_{j0} - \sum_{v_f \in F_0} - t_{jf0} * y_{jf0} \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = instance.timeLimit - time(j, 0) * x[j][0];
      for (int f = 1; f < f0.size(); ++f) 
        expr -= time(j, f, 0) * y[j][f][0];
      c = IloConstraint (t[j - 1] <= expr);
      constraintName<<"customer "<<c0[j]->id<<" time ub 2";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // \tau_j 
    for (int j = 1; j < c0.size(); ++j) {
      model.add(customersMinRequiredTime[j] <= t[j - 1] <= instance.timeLimit - customersMinRequiredTime[j] - c0[j]->serviceTime);
    }
    // e_j - e_i + M2_{ij} * x_{ij} + (M2_{ij} - e_{ij} - e_{ji}) * x_{ji} \leqslant M2_{ij} - e_{ij} \forall v_i, v_j \in C
    for (int i = 1; i < c0.size(); ++i) 
      for (int j = 1; j < c0.size(); ++j) {
        expr = e[j - 1] - e[i - 1] + M2(i, j) * x[i][j] + (M2(i, j) - customersFuel(i, j) - customersFuel(j, i)) * x[j][i];
        c = IloConstraint (expr <= M2(i, j) - customersFuel(i, j));
        constraintName<<"edge ("<<c0[i]->id<<", "<<c0[j]->id<<") energy";
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    // e_j \leqslant \beta - e_{0j} * x_{0j} - \sum_{v_i \in C_0} \sum_{v_f \in F_0} e_{fj} * y_{ifj} \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = instance.vehicleFuelCapacity - afsToCustomerFuel(0, j) * x[0][j];
      for (int i = 0; i < c0.size(); ++i) 
        for (int f = 0; f < f0.size(); ++f) 
          expr -= afsToCustomerFuel(f, j) * y[i][f][j];
      c = IloConstraint (e[j - 1] <= expr);
      constraintName<<"customer "<<c0[j]->id<<" energy ub";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // e_j \geqslant e_{j0} * x_{j0} + \sum_{v_i \in C_0} \sum_{v_f \in F_0} e_{jf} * y_{jfi} \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = customerToAfsFuel(j, 0) * x[j][0];
      for (int i = 0; i < c0.size(); ++i) 
        for (int f = 0; f < f0.size(); ++f) 
          expr += customerToAfsFuel(j, f) * y[j][f][i];
      c = IloConstraint (e[j - 1] >= expr);
      constraintName<<"customer "<<c0[j]->id<<" energy lb";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // e_j \geqslant E^{LB}_j \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      c = IloConstraint (e[j - 1] >= customersMinRequiredFuel[j - 1]);
      constraintName<<"customer "<<c0[j]->id<<" energy lb2";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    // e_j \leqslant \beta - E^{LB}_j \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      c = IloConstraint (e[j - 1] <= instance.vehicleFuelCapacity - customersMinRequiredFuel[j - 1]);
      constraintName<<"customer "<<c0[j]->id<<" energy ub2";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    //new inequalities
    //solution lb
    for (int i = 0; i < c0.size(); ++i) 
      for (int j = 0; j < c0.size(); ++j) {
        expr +=  instance.distances[c0[i]->id][c0[j]->id] * x[i][j];
        for (int f = 0; f < f0.size(); ++f)
          expr += (instance.distances[c0[i]->id][f0[f]->id] + instance.distances[f0[f]->id][c0[j]->id]) * y[i][f][j];
      }
    c = IloConstraint (expr >= solLB);
    c.setName("solution LB");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //n routes LB
    for (int i = 0; i < c0.size(); ++i) {
      expr += x[0][i];
      for (int f = 0; f < f0.size(); ++f)
        expr += y[0][f][i];
    }
    c = IloConstraint (expr >= nRoutesLB);
    c.setName("nRoutes LB");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //solution fuel lb alpha 1
    for (int i = 0; i < c0.size(); ++i) {
      for (int f = 0; f < f0.size(); ++f)
        expr -= alpha * y[0][f][i];
      for (int j = 0; j < c0.size(); ++j) {
        expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
        for (int f = 0; f < f0.size(); ++f)
          expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - instance.vehicleFuelCapacity/2.0) * y[i][f][j];
      }
    }
    c = IloConstraint (expr >= 0);
    c.setName("solution fuel LB alpha 1");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //solution fuel lb alpha 2
    for (int i = 0; i < c0.size(); ++i) {
      for (int f = 0; f < f0.size(); ++f)
        expr -= alpha * y[i][f][0];
      for (int j = 0; j < c0.size(); ++j) {
        expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
        for (int f = 0; f < f0.size(); ++f)
          expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - instance.vehicleFuelCapacity/2.0) * y[i][f][j];
      }
    }
    c = IloConstraint (expr >= 0);
    c.setName("solution fuel LB alpha 2");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //solution fuel lb alpha 3
    for (int i_ = 1; i_ < c0.size(); ++i_) 
      for (int j_ = 1; j_ < c0.size(); ++j_) {
        for (int i = 0; i < c0.size(); ++i) {
          for (int j = 0; j < c0.size(); ++j) {
            expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
            for (int f = 0; f < f0.size(); ++f)
              expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - instance.vehicleFuelCapacity/2.0) * y[i][f][j];
          }
        }
        for (int f = 0; f < f0.size(); ++f)
          expr -= alpha * y[i_][f][j_];
        c = IloConstraint (expr >= 0);
        constraintName<<"solution fuel LB alpha 3 in customer"<<c0[i_]->id<<" and "<<c0[j_]->id;
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    //solution fuel lb lambda 1
    for (int i = 0; i < c0.size(); ++i) {
      expr += psi * x[i][0];
      for (int f = 0; f < f0.size(); ++f)
        expr -= 2 * lambda * y[0][f][i] + psi * y[0][f][i];
      for (int j = 0; j < c0.size(); ++j) {
        expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
        for (int f = 0; f < f0.size(); ++f)
          expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - psi) * y[i][f][j];
      }
    }
    c = IloConstraint (expr >= 0);
    c.setName("solution fuel LB lambda 1");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //solution fuel lb lambda 2
    for (int i = 0; i < c0.size(); ++i) {
      expr += psi * x[0][i];
      for (int f = 0; f < f0.size(); ++f)
        expr -= (2 * lambda + psi) * y[i][f][0];
      for (int j = 0; j < c0.size(); ++j) {
        expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
        for (int f = 0; f < f0.size(); ++f)
          expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - psi) * y[i][f][j];
      }
    }
    c = IloConstraint (expr >= 0);
    c.setName("solution fuel LB lambda 2");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //solution fuel lb lambda 3
    for (int i_ = 0; i_ < c0.size(); ++i_) 
      for (int j_ = 0; j_ < c0.size(); ++j_) {
        for (int f = 0; f < f0.size(); ++f)
          expr -= 2 * lambda * y[i_][f][j_];
        for (int i = 0; i < c0.size(); ++i) {
          expr += psi * x[0][i];
          for (int f = 0; f < f0.size(); ++f)
            expr += psi * y[0][f][i];
          for (int j = 0; j < c0.size(); ++j) {
            expr += instance.fuel(c0[i]->id, c0[j]->id) * x[i][j];
            for (int f = 0; f < f0.size(); ++f)
              expr += (instance.fuel(c0[i]->id, f0[f]->id) + instance.fuel(f0[f]->id, c0[j]->id) - psi) * y[i][f][j];
          }
        }
        c = IloConstraint (expr >= 0);
        constraintName<<"solution fuel LB lambda 3 in customers "<<c0[i_]->id<<" and "<<c0[j_]->id;
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }






    /*
    vector<vector<int>> routes_ = {
      {0, 21, 20, 0, 13, 0},
      {0, 9, 2, 0 },
      {0, 10, 4, 1, 0 },
      {0, 11, 15, 23, 0 },
      {0, 19, 1, 12, 14, 22, 0 },
      {0, 16, 5, 8, 7, 0 },
      {0, 18, 17, 6, 2, 0 }
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
        if (currIndex != customersC0Indexes.end() && !(curr->id == instance.depot.id && curr != --route.end())) {
          int j = currIndex->second;
          model.add(x[i][j] == 1);
          currFuel -= customersFuel(i, j);
          currTime += time(i, j);
          if (j != 0) {
            model.add(t[j - 1] == currTime);
            model.add(e[j - 1] == currFuel);
          }
        } else {
          //is an afs 
          int f = afssF0Indexes[curr->id];
          ++curr;
          int j = customersC0Indexes[curr->id];
          model.add(y[i][f][j] == 1);
          currTime += time(i, f, j);
          currFuel = instance.vehicleFuelCapacity - afsToCustomerFuel(f, j);
          if (j != 0) {
            model.add(t[j - 1] == currTime);
            model.add(e[j - 1] == currFuel);
          }
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
    cplex.use(new Depth_node_callback(env));
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

void Matheus_model_2::extraStepsAfterModelCreation() {
  //
}

void Matheus_model_2::setCustomParameters(){
  try{
    setParameters();
    //for the user cut callback
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

void Matheus_model_2::fillVals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, c0.size());
    y_vals = Matrix3DVal (env, c0.size());
    for (int i = 0; i < c0.size(); ++i){
      x_vals[i] = IloNumArray (env, c0.size(), 0, 1, IloNumVar::Int);
      y_vals[i] = Matrix2DVal (env, f0.size());
      cplex.getValues(x_vals[i], x[i]);
      for (int f = 0; f < f0.size(); ++f){
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
  for (int i = 0; i < c0.size(); ++i){
    cout<<" ";
    if (i <=9)
      cout<<" ";
    cout<<i;
  }
  cout<<endl;
  for (int i = 0; i < c0.size(); ++i){
    cout<<i<<" ";
    if (i <= 9)
      cout<<" ";
    for (int j = 0; j < c0.size(); ++j) {
      cout<<abs(x_vals[i][j])<<"  ";
    }
    cout<<endl;
  }
  for (int f = 0; f < f0.size(); ++f){
    cout<<"AFS: "<<f<<endl;
    cout<<" ";
    for (int i = 0; i < c0.size(); ++i){
      cout<<" ";
      if (i <=9)
        cout<<" ";
      cout<<i;
    }
    cout<<endl;
    for (int i = 0; i < c0.size(); ++i){
      cout<<i<<" ";
      if (i <= 9)
        cout<<" ";
      for (int j = 0; j < c0.size(); ++j)
        cout<<abs(y_vals[i][f][j])<<"  ";
      cout<<endl;
    }
  }
  for (int i = 0; i < c0.size(); ++i)
    cout<<i<<": "<<c0[i]->id<<endl;
    */
}

void Matheus_model_2::createGvrp_solution(){
  try{
    list<list<Vertex>> routes;
    list<Vertex> route;
    int curr;    
    bool next = false;
    //checking the depot neighboring
    while (true) {
      next = false;
      for (int i = 1; i < c0.size() && !next; ++i) {
        if (x_vals[0][i] > INTEGRALITY_TOL) {
          next = true;
          route.push_back(Vertex(*c0[0]));
          x_vals[0][i] = 0;
        } 
        if (!next)
          //on y[j][0][i]
          for (int j = 0; j < c0.size(); ++j) 
            if (y_vals[j][0][i] > INTEGRALITY_TOL) {
              next = true;
              route.push_back(Vertex(*c0[0]));
              y_vals[j][0][i] = 0;
              break;
            }
        if (!next)
          //on y[0][f][i]
          for (int f = 0; f < f0.size(); ++f)
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
        for (int i = 0; i < c0.size(); ++i) {
          next = false;
          if (x_vals[curr][i] > INTEGRALITY_TOL) {
            next = true;
            route.push_back(Vertex(*c0[i]));
            x_vals[curr][i] = 0;
            curr = i;
            break;
          } else {
            for (int f = 0; f < f0.size(); ++f)
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

void Matheus_model_2::endVals () {
  //end vals
  for (int i = 0; i < c0.size(); ++i) {
    for (int f = 0; f < f0.size(); ++f)
      y_vals[i][f].end();
    y_vals[i].end();
    x_vals[i].end();
  }
  y_vals.end();
  x_vals.end();
}

void Matheus_model_2::endVars(){
  for (int i = 0; i < c0.size(); ++i) {
    x[i].end();
    for (int f = 0; f < f0.size(); ++f) 
      y[i][f].end();
    y[i].end();
  }
  x.end();
  y.end();
  t.end();
  e.end();
}
