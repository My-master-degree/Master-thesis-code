#include "utils/util.hpp"
#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/depth_node_callback.hpp"
#include "models/local_search_strategy_enum.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/local_searchs/all.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/heuristic_callback.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/subcycle_user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/invalid_edge_preprocessing_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/invalid_edge_preprocessing_4.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/greedy_lp_heuristic.hpp"

#include <sstream>
#include <list>
#include <time.h> 
#include <string> 
#include <unordered_set> 
#include <queue> 
#include <unordered_set>


using namespace utils;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::matheus_model_3;

using namespace std;

Matheus_model_3::Matheus_model_3(const Gvrp_instance& instance, unsigned int time_limit) : Gvrp_model(instance, time_limit), nGreedyLP(0), BPPTimeLimit(10000000), nPreprocessings1(0), nPreprocessings2(0), nPreprocessings3(0), nPreprocessings4(0), nImprovedMSTNRoutesLB(0), nBPPNRoutesLB(0), RELAXED(false) {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with metric distances");
  //populating all map and customers set
  all[instance.depot.id] = &instance.depot;
  timesLBs[instance.depot.id] = 0.0;
  for (const Vertex& customer: instance.customers) {
    all[customer.id] = &customer;
    customers.insert(customer.id);
    timesLBs[customer.id] = calculateCustomerMinRequiredTime (instance, *gvrp_afs_tree, customer) + customer.serviceTime;
    fuelsLBs[customer.id] = calculateCustomerMinRequiredFuel (instance, *gvrp_afs_tree, customer);
  }
  //creating dummies
  //get ub on the number of dummies
  Gvrp_feasible_solution_heuristic gfsh (instance);
  Gvrp_solution heuristic_solution = gfsh.run();
  All all_ls (instance, heuristic_solution, FIRST_IMPROVEMENT);
  heuristic_solution = all_ls.run();
  double routeFuelLimit = (instance.timeLimit - instance.customers.front().serviceTime) * instance.vehicleAverageSpeed * instance.vehicleFuelConsumptionRate;
  int nDummies = min(min(floor(((routeFuelLimit - 2 * lambda)/psi) + 1), floor(2 * (routeFuelLimit - alpha)/instance.vehicleFuelCapacity)) * heuristic_solution.routes.size(), double(instance.customers.size() + instance.maxRoutes));
  //insert them
  for (const Vertex& afs: instance.afss) {
    afs_dummies[afs.id].push_back(afs.id);
    all[afs.id] = &afs;
    dummies[afs.id] = &afs;
  }
  for (int f = 0; f < gvrp_afs_tree->f0.size(); ++f)
    timesLBs[gvrp_afs_tree->f0[f]->id] = gvrp_afs_tree->times[f];
  int dummy_id = all.rbegin()->second->id;
  //afs dummies
  for (const Vertex& afs: instance.afss)
    for (int i = 1; i < nDummies; ++i) {
      afs_dummies[afs.id].push_back(++dummy_id);
      all[dummy_id] = &afs;
      dummies[dummy_id] = &afs;
      timesLBs[dummy_id] = timesLBs[afs.id];
    }


  


  //reductions
  const auto& gvrpReducedGraphs = calculateGVRPReducedGraphs (instance, *gvrp_afs_tree);
  gvrpReducedGraphDistances = gvrpReducedGraphs.first;
  gvrpReducedGraphTimes = gvrpReducedGraphs.second;
  //set sol lb
  vector<const Vertex *> c0 (instance.customers.size() + 1);
  int i = 1;
  c0[0] = &instance.depot;
  for (const Vertex& customer : instance.customers) {
    c0[i] = &customer;
    ++i;
  }
  const auto& closestsDistances = calculateClosestsGVRPCustomers(gvrpReducedGraphDistances, c0);
  solLB = max(calculateGvrpLBByImprovedMST(c0, closestsDistances, gvrpReducedGraphDistances), calculateGvrpLB1(closestsDistances));
  //set n routes lb
  const auto& closestsTimes = calculateClosestsGVRPCustomers(gvrpReducedGraphTimes, c0);
  nRoutesLB = max(int(ceil(calculateGvrpLBByImprovedMSTTime(c0, closestsTimes, gvrpReducedGraphTimes)/instance.timeLimit)), calculateGVRP_BPP_NRoutesLB(instance, c0, closestsTimes, BPPTimeLimit));
  //user cuts
  user_constraints.push_back(new Subcycle_user_constraint(*this));
  //preprocessings
  preprocessings.push_back(new Invalid_edge_preprocessing(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_2(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_3(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_4(*this));
  //heuristic callbacks
  heuristic_callbacks.push_back(new Greedy_lp_heuristic(*this));

} 

double Matheus_model_3::time (int i, int j) {
  return all[i]->serviceTime + instance.time(all[i]->id, all[j]->id);
}

double Matheus_model_3::M1(int i, int j) {
  return instance.timeLimit + time(i, j) - (timesLBs[i] + timesLBs[j]);
//  return instance.timeLimit + time(i, j) - (time(i, 0) + time(j, 0));
}

double Matheus_model_3::fuel (int i, int j) {
  return instance.fuel(all[i]->id, all[j]->id);
}

double Matheus_model_3::M2 (int i, int j) {
  return instance.vehicleFuelCapacity + instance.fuel(all[i]->id, all[j]->id) - (fuelsLBs[i]  + fuelsLBs[j]);
}

Matheus_model_3::~Matheus_model_3() {
  for (Preprocessing * preprocessing : preprocessings)
    delete preprocessing;  
  for (User_constraint * user_constraint : user_constraints)
    delete user_constraint;  
  for (Heuristic_callback * heuristic_callback : heuristic_callbacks)
    delete heuristic_callback;  
}

pair<Gvrp_solution, Mip_solution_info> Matheus_model_3::run(){
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
//    cout<<"Setting parameters"<<endl;
    setCustomParameters();
//    cout<<"Solving model"<<endl;
    struct timespec start, finish;
    double elapsed;
    clock_gettime(CLOCK_MONOTONIC, &start);
    if ( !cplex.solve() ) {
//      env.error() << "Failed to optimize LP." << endl;
      mipSolInfo = Mip_solution_info(-1, cplex.getStatus(), -1, -1);
      endVars();
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
    fillX_vals();
//    cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
    endVals();
    endVars();
    return make_pair(*solution, mipSolInfo);
  } catch (IloException& e) {
    output_exception<<"Concert exception caught: " << e<<endl;
    throw output_exception.str();
  } catch (string s) {
    throw s;
  }
}

void Matheus_model_3::createVariables(){
  x = Matrix2DVar (env, all.size());
  t = IloNumVarArray (env, all.size(), 0, instance.timeLimit, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for(int customer : customers) {
      //e var
      nameStream<<"e["<<customer<<"]";
      e[customer] = IloNumVar (env, 0, instance.vehicleFuelCapacity, IloNumVar::Float);
      e[customer].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
    }
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      //t var
      nameStream<<"t["<<i<<"]";
      t[i].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //x var
      x[i] = IloNumVarArray(env, all.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        nameStream<<"x["<<i<<"]["<<j<<"]";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
      }
    }
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating variables");
  }
}

void Matheus_model_3::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        fo +=  instance.distances[p.second->id][p1.second->id] * x[i][j];
      }
    }  
    model = IloModel (env);
    model.add(IloMinimize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void Matheus_model_3::createModel() {
  try {
    for (Preprocessing* preprocessing : preprocessings)
      preprocessing->add();
    //setup
    int depot = instance.depot.id;
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //x_{ii} = 0, \forall v_i \in V'
    for (const pair<int, const Vertex *>& p : all) {
      int i = p.first;
      model.add(x[i][i] == 0);
    }
    //\sum_{v_j \in V' : v_i \neq v_j} x_{ij} = 1, \forall v_i \in C
    for (const Vertex& customer : instance.customers){
      int i = customer.id;
      for (const pair<int, const Vertex *>& p1 : all) {
        int j = p1.first;
        if (i != j)
          expr += x[i][j];
      }
      c = IloConstraint (expr == 1);
      constraintName<<"customer "<<i<<" must be visited exactly once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in V' : v_f \neq v_j} x_{fj} \leqslant 1, \forall v_f inF_0 
    for (const pair<int, const Vertex *>& p : dummies) {
      int f = p.first;
      for (const pair<int, const Vertex *>& p1 : all) {
        int j = p1.first;
        if (f != j)
          expr += x[f][j];
      }
      c = IloConstraint (expr <= 1);
      constraintName<<"dummy "<<f<<" must be used at most once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in V' : v_j \neq v_i} x_{ij} = \sum_{v_j \in V' : v_j \neq v_i} x_{ji}, \forall v_i \in V'
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        if (i != j) {
          expr += x[i][j];
          expr1 += x[j][i]; 
        }
      }
      c = IloConstraint (expr == expr1);
      constraintName<<i<<" # entering edges == "<<i<<" # exiting edges";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr1.end();
      expr = IloExpr(env);
      expr1 = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }    
    //\sum_{v_j \in V' : v_j \neq v_0} x_{0j} \leqslant m
    for (const pair<int, const Vertex *>& p : all)
      expr += x[depot][p.first];
    c = IloConstraint (expr <= instance.maxRoutes);
    constraintName<<" # routes limit == "<<instance.maxRoutes;
    c.setName(constraintName.str().c_str());
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    constraintName.clear();
    constraintName.str("");
    //times 
    //t_i - t_j + M1_{ij} x_{ij} + (M1_{ij} - t_{ij} - t_{ji}) x_{ji} \leqslant M1_{ij} - t_{ij}, \forall v_i, v_j \in V'\backslash\{v_0\}
    for (const pair<int, const Vertex *>& p1 : all) {
      int j = p1.first;
      if (j != depot) {
        for (const pair<int, const Vertex *>& p : all) {
          int i = p.first;
          if (i != depot && i != j) {
            c = IloConstraint (t[i] - t[j] + M1(i, j) * x[i][j] + (M1(i, j) - time(i, j) - time(j, i)) * x[j][i] <= M1(i, j) - time(i, j));
            constraintName<<" updating time "<<i;
            c.setName(constraintName.str().c_str());
            model.add(c);
            constraintName.clear();
            constraintName.str("");
          }
        }
      }
    }
    //LB^i_T - s_i \leqslant t[i] \leqslant \beta - LB_i^T, \forall v_i \in V'\backslash\{v_0\}
    for (const pair<int, const Vertex *>& p : all) {
      int i = p.first;
      if (i != depot) {
        c = IloConstraint (timesLBs[i] - p.second->serviceTime <= t[i] <= instance.timeLimit - timesLBs[i]);
        constraintName<<i<<" time lb and ub";
        c.setName(constraintName.str().c_str());
        model.add(c);
        constraintName.clear();
        constraintName.str("");
      }
    }
    /*
    for (int customer : customers) {
      c = IloConstraint (timesLBs[customer] - all[customer]->serviceTime <= t[customer] <= instance.timeLimit - timesLBs[customer]);
      constraintName<<customer<<" time lb and ub";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    */
    //energy
    //e_j - e_i + M2_{ij} x_{ij} + (M2_{ij} - (e_{ij} + e_{ji})) x_{ji} \leqslant M2_{ij} - e_{ij}, \forall v_i, v_j \in C
    for (int i : customers) 
      for (int j : customers) 
        if (i != j) {
          c = IloConstraint (e[j] - e[i] + M2(i, j) * x[i][j] + (M2(i, j) - (instance.fuel(j, i) + instance.fuel(i, j))) * x[j][i] <= M2(i, j) - instance.fuel(i, j));
          constraintName<<i<<" updating fuel";
          c.setName(constraintName.str().c_str());
          model.add(c);
          constraintName.clear();
          constraintName.str("");
        }
    //\beta - c_{ij} x_{ij} \geq e_i, \forall v_i \in C, \forall v_f \in V^{'}\backslash C
    for (int i : customers) 
      for (const pair<int, const Vertex *>& p : all) {
        int j = p.first;
        if (!customers.count(j)) {
          c = IloConstraint (fuel(i, j) * x[i][j] <= e[i]);
          constraintName<<i<<" fuel ub and lb";
          c.setName(constraintName.str().c_str());
          model.add(c);
          constraintName.clear();
          constraintName.str("");
        }
      }
    //e_i \geq c_{ij} x_{ij}, \forall v_i \in C, \forall v_f \in V^{'}\backslash C
    for (int i : customers) 
      for (const pair<int, const Vertex *>& p : all) {
        int j = p.first;
        if (!customers.count(j)) {
          c = IloConstraint (e[i] <= instance.vehicleFuelCapacity - fuel(j, i) * x[j][i]);
          constraintName<<i<<" fuel ub and lb";
          c.setName(constraintName.str().c_str());
          model.add(c);
          constraintName.clear();
          constraintName.str("");
        }
      }
    //\beta - LB_i^E \geq e_i, \forall v_i, \forall v_f \in F
    for (int i : customers) {
      c = IloConstraint ( fuelsLBs[i] <= e[i]);
      constraintName<<i<<" time ub and lb";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    //e_i \geq LB_i^E, \forall v_i, \forall v_f \in F
    for (int i : customers) {
      c = IloConstraint (e[i] <= instance.vehicleFuelCapacity - fuelsLBs[i] );
      constraintName<<i<<" time ub and lb";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    //no edge between dummies from the same AFS
    for (const pair<int, list<int>>& p : afs_dummies) {
      for (int dummy : p.second)
        for (int dummy1 : p.second)
          expr += x[dummy][dummy1];
      c = IloConstraint (expr == 0);
      constraintName<<" no visits between dummies from the afs "<<p.first;
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //dummy i + 1 will be used only if i is used
    for (const pair<int, list<int>>& p : afs_dummies) {
      for (list<int>::const_iterator prev = p.second.begin(), curr = next(prev); curr != p.second.end(); prev = curr, ++curr) {
        for (const pair<int, const Vertex *>& p1 : all) {
          int j = p1.first;
          expr += x[j][*prev] - x[j][*curr];
        }
        c = IloConstraint (expr >= 0);
        constraintName<<" afs "<<*curr<<" is used only if "<<*prev;
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //n routes LB
    for (const pair<int, const Vertex *>& p2 : all){
      int i = p2.first;
      expr += x[depot][i];
    }
    c = IloConstraint (expr >= nRoutesLB);
    c.setName("nRoutes LB");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //solution lb
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        expr += x[i][j] * instance.distances[p.second->id][p1.second->id];
      }
    }
    c = IloConstraint (expr >= solLB);
    c.setName("solution LB");
    model.add(c);
    expr.end();
    expr = IloExpr(env);






    /*
    list<list<int>> routesIds = {
      {0, 1, 10, 12, 3, 8, 10, 2, 0},
      {0, 7, 13, 7, 5, 7, 9, 7, 6, 0},
      {0, 10, 15, 11, 4, 0},
    };
    list<list<Vertex>> routes;
    for (const list<int>& routeIds : routesIds) {
      list<Vertex> route; 
      for (int id : routeIds) 
        route.push_back(Vertex(*all[id]));
      routes.push_back(route);
    }
    Gvrp_solution gvrp_sol (routes, instance);
    cout<<gvrp_sol<<endl;
    unordered_map<int, list<int>> afs_dummies_ = afs_dummies;
    for (const pair<int, list<int>>& p : afs_dummies_) {
      cout<<p.first<<": ";
      for (int id : p.second)
        cout<<id<<", ";
      cout<<endl;
    }
    for(list<Vertex>& route : routes) {
      int prevId = route.begin()->id;
      for (list<Vertex>::iterator curr = next(route.begin()); curr != route.end(); ++curr) {
        int currId = curr->id;
        //is an afs
        if (afs_dummies_.count(currId) && currId != 0) {
          int dummyId = afs_dummies_[currId].front();
          afs_dummies_[currId].pop_front();
          currId = dummyId;
        } 
        cout<<"("<<prevId<<", "<<currId<<")"<<endl;
        model.add(x[prevId][currId] == 1);
        prevId = currId;
      }
      cout<<endl;
    }
    */








    //init
    cplex = IloCplex(model);
    //user cuts
    for (User_constraint* user_constraint : user_constraints)
      cplex.use(user_constraint);
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

void Matheus_model_3::extraStepsAfterModelCreation() {
  //
}

void Matheus_model_3::setCustomParameters(){
  try{
    setParameters();
    cplex.setParam(IloCplex::Param::Simplex::Tolerances::Optimality, 1e-3);
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

void Matheus_model_3::fillX_vals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, all.size());
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      x_vals[i] = IloNumArray (env, all.size(), 0, 1, IloNumVar::Int);
      cplex.getValues(x_vals[i], x[i]);
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void Matheus_model_3::createGvrp_solution(){
  try{
    list<list<Vertex> > routes;
    list<Vertex> route;
    int depot = instance.depot.id;
    int curr,
        next;    
    //checking the depot neighboring
    for (const pair<int, const Vertex *>& p : all){
      //dfs
      curr = depot;
      next = p.first; 
      if (x_vals[curr][next] > INTEGRALITY_TOL && curr != next) {
        route.push_back(Vertex(*all[curr]));
        for (; next != depot; route.push_back(Vertex(*all[curr]))) {
          curr = next;
          for (const pair<int, const Vertex *>& p1 : all) {
            int j = p1.first;
            if (x_vals[curr][j] > INTEGRALITY_TOL) {
              next = j;
              break;
            }
          }
        }
        route.push_back(Vertex(*all[next]));
        routes.push_back(route);
        route = list<Vertex> ();
      }
    }
    solution = new Gvrp_solution(routes, instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void Matheus_model_3::endVals(){
  for (const pair<int, const Vertex *>& p : all) 
    x_vals[p.first].end();
  x_vals.end();
}

void Matheus_model_3::endVars(){
  for (const pair<int, const Vertex *>& p : all)
    x[p.first].end();
  x.end();
  e.end(); 
  t.end(); 
}
