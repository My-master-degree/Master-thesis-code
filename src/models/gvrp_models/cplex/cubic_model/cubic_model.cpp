#include "models/vertex.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/depth_node_callback.hpp"
#include "models/distances_enum.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_afs_tree.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/subcycle_lazy_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"

#include <list>
#include <set>
#include <vector>
#include <queue>
#include <stdlib.h>
#include <exception>
#include <sstream>
#include <time.h>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

using namespace std;
using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::cubic_model;

Cubic_model::Cubic_model(const Gvrp_instance& instance, unsigned int _time_limit): Gvrp_model(instance, time_limit) {
  if (instance.distances_enum != SYMMETRIC && instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with symmetric or metric distances");
  //fill all and customers
  for (const Vertex& customer : instance.customers) {
    all[customer.id] = &customer;
    customers.insert(customer.id);
  }
  for (const Vertex& afs : instance.afss) 
    all[afs.id] = &afs;
  all[instance.depot.id] = &instance.depot;
  //lazies
  lazy_constraints.push_back(new Subcycle_lazy_constraint(*this));
}

Cubic_model::~Cubic_model() {
  for (Lazy_constraint * lazy_constraint : lazy_constraints)
    delete lazy_constraint;
  for (User_constraint * user_constraint : user_constraints)
    delete user_constraint;
}

pair<Gvrp_solution, Mip_solution_info> Cubic_model::run(){
    throw string("Error: The compact model requires a G-VRP instance with symmetric or metric distances");
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
//      env.end();
      throw mipSolInfo;
    }
    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
//    cplex.exportModel("cplexcpp.lp");
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    cout<<"Getting x values"<<endl;
    fillX_vals();
//    cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
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

void Cubic_model::createVariables(){
  x = Matrix3DVar (env, instance.maxRoutes);
  e = IloNumVarArray (env, all.size(), 0, instance.vehicleFuelCapacity, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      nameStream<<"e["<<i<<"]";
      const string name = nameStream.str();
      e[i].setName(name.c_str());
      nameStream.clear();
      nameStream.str("");
    }
    //x var
    for (int k = 0; k < instance.maxRoutes; k++){
      x[k] = IloArray<IloNumVarArray> (env, all.size());
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        x[k][i] = IloNumVarArray(env, all.size(), 0, 1, IloNumVar::Int);
        //setting names
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          nameStream<<"x["<<k<<"]["<<i<<"]["<<j<<"]";
          const string name_x = nameStream.str();
          x[k][i][j].setName(name_x.c_str());
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

void Cubic_model::createObjectiveFunction() {
//objective function
  try{
    IloExpr fo (env);
    for (int k = 0; k < instance.maxRoutes; k++)
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          fo +=  instance.distances[i][j] * x[k][i][j];
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

void Cubic_model::createModel() {
  try {
    //preprocessing conditions
    for (Preprocessing* preprocessing : preprocessings)
      preprocessing->add();
    //setup
    int depot = instance.depot.id;
    double beta = instance.vehicleFuelCapacity;
    double T = instance.timeLimit;
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //\sum_{v_j \in V} x_{ij}^k = \sum_{v_j \in V} x_{ji}^k, \forall v_i \in V, \forall k \in M
    for (int k = 0; k < instance.maxRoutes; k++)
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          expr += x[k][i][j];
          expr1 += x[k][j][i];
        }
        c = IloConstraint (expr == expr1);
        c.setName("#entering edges == #exiting edges");
        model.add(c);
        expr1.end();
        expr.end();
        expr = IloExpr(env);
        expr1 = IloExpr(env);
      }    
    //\sum_{v_i \in V} x_{0i}^k \leqslant 1, \forall k in M
    for (int k = 0; k < instance.maxRoutes; k++){
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        expr += x[k][depot][i];
      }
      c = IloConstraint (expr <= 1);
      c.setName("route must be used at most once");
      model.add(c);
      expr.end();
      expr = IloExpr(env);
    }
    //\sum_{k \in M} \sum_{v_j \in V} x_{ij}^{k} = 1, \forall v_i \in C
    for (Vertex customer :instance.customers){
      int i = customer.id;
      for (int k = 0; k < instance.maxRoutes; k++){
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          expr += x[k][i][j];
        }
      }
      c = IloConstraint (expr == 1);
      c.setName("customer must be visited exactly once");
      model.add(c);
      expr.end();
      expr = IloExpr(env);
    }
    //\sum_{k \in M} \sum_{v_j \in V} x_{0j}^k \leqslant m
    for (int k = 0; k < instance.maxRoutes; k++)
      for (const pair<int, const Vertex *>& p : all)
        expr += x[k][depot][p.first];
    c = IloConstraint (expr <= instance.maxRoutes);
    c.setName("# routes upper bound");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //e_0 = \beta
    c = IloConstraint (e[depot] == beta);
    c.setName("e_depot = beta");
    model.add(c);
    //e_f = \beta, \forall v_f \in F
    for (Vertex afs : instance.afss)  {
      int f = afs.id;
      c = IloConstraint (e[f] == beta);
      c.setName("e_f = beta");
      model.add(c);
    }
    //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k), \forall v_j \in C,\forall v_i \in V, \forall k \in M
    for (int k = 0; k < instance.maxRoutes; k++)
      for (Vertex customer : instance.customers) {
        int j = customer.id;
        for (const pair<int, const Vertex *>& p :all) {
          int i =  p.first;
          expr = e[i] - x[k][i][j] * instance.distances[i][j] * instance.vehicleFuelConsumptionRate + beta * (1 -  x[k][i][j]);
          c = IloConstraint (e[j] <= expr);
          c.setName("updating fuel level");
          model.add(c);
          expr.end();
          expr = IloExpr (env);
        }
      }
    //e_i \geq c_{ij} x_{ij}^k, \forall v_i, \forall v_j \in V, \forall k \in M
    for (int k = 0; k < instance.maxRoutes; k++)
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          expr = instance.distances[i][j] * x[k][i][j] * instance.vehicleFuelConsumptionRate;
          c = IloConstraint (e[i] >= expr);
          c.setName("disabling infeasible edges");
          model.add(c);
          expr.end();
          expr = IloExpr(env);
        }
      }
    //x_{ij}^k c_{ij} \leq \beta, \forall v_i, \forall v_j \in V, \forall k \in M
      for (int k = 0; k < instance.maxRoutes; k++)
        for (const pair<int, const Vertex *>& p : all){
          int i = p.first;
          for (const pair<int, const Vertex *>& p1 : all){
            int j = p1.first;
            expr = instance.distances[i][j] * x[k][i][j] * instance.vehicleFuelConsumptionRate;
            c = IloConstraint (expr <= beta);
            c.setName("disabling infeasible edges 2");
            model.add(c);
            expr.end();
            expr = IloExpr(env);
          }
        }
      //\sum_{(i, j) \in E} x_{ij}^k ((c_{ij} / S) + time(v_i) )\leq T, \forall k \in M
      for (int k = 0; k < instance.maxRoutes; k++){
        for (const pair<int, const Vertex *>& p : all){
          int i = p.first;
          for (const pair<int, const Vertex *>& p1 : all){
            int j = p1.first;
            expr += x[k][i][j] * ((instance.distances[i][j] / instance.vehicleAverageSpeed) + p.second->serviceTime);
          }
        }
        c = IloConstraint (expr <= T);
        c.setName("time limit constraint");
        model.add(c);
        expr.end();
        expr = IloExpr(env);
      }
    //extra constraints
    for (Extra_constraint* extra_constraint : extra_constraints) 
      extra_constraint->add();
    //init
    cplex = IloCplex(model);
    //lazy constraints
    for (Lazy_constraint * lazy_constraint : lazy_constraints)
      cplex.use(lazy_constraint);
    //user cuts
    for (User_constraint * user_constraint : user_constraints)
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

void Cubic_model::extraStepsAfterModelCreation() {
  //
}

void Cubic_model::setCustomParameters(){
  try{
    setParameters();
    //DOUBTS:
    // Turn off the presolve reductions and set the CPLEX optimizer
    // to solve the worker LP with primal simplex method.
    cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 1);
    cplex.setParam(IloCplex::Param::Preprocessing::Symmetry, 0);
    cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal); 
    //preprocesing setting
    cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse); 
    // Turn on traditional search for use with control callbacks
//    cplex.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);
    //:DOUBTS
    //LAZY CONSTRAINTS
    //thread safe setting
    cplex.setParam(IloCplex::Param::Threads, 1);
    // Tweak some CPLEX parameters so that CPLEX has a harder time to
    // solve the model and our cut separators can actually kick in.
    cplex.setParam(IloCplex::Param::MIP::Strategy::HeuristicFreq, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::MIRCut, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::Implied, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::Gomory, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::FlowCovers, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::PathCut, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::LiftProj, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::ZeroHalfCut, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::Cliques, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::Covers, -1);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void Cubic_model::fillX_vals(){
  //getresult
  try{
    x_vals = Matrix3DVal (env, instance.maxRoutes);
    for (int k = 0; k < instance.maxRoutes; k++) { 
      x_vals[k] = IloArray<IloNumArray> (env, all.size());
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        x_vals[k][i] = IloNumArray (env, all.size(), 0, 1, IloNumVar::Int);
        cplex.getValues(x_vals[k][i], x[k][i]);
      }
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void Cubic_model::createGvrp_solution(){
  /*
  //print x vals
  for (int k = 0; k < instance.maxRoutes; k++){ {
    //columns
    cout<<" ";
    for (auto p : all){
      if (p.first <= 9)
        cout<<" ";
      cout<<" "<<p.first;
    }
    cout<<endl;
    //content
    for (auto p : all){
      cout<<p.first;
      if (p.first <= 9)
        cout<<" ";
      for (auto p1 : all){
        cout<<" "<<int(x_vals[k][p.first][p1.first])<<" ";
      }
      cout<<endl;
    }
  }
  */
  try{
    list<list<Vertex> > routes;
    //dfs
    int depot = instance.depot.id;
    int curr,
        next;    
    for (int k = 0; k < instance.maxRoutes; k++) {
      curr = depot;
      next = depot;
      //checking if the route is used
      //get remaining nodes (if exists)
      set<int> routeAfssSet;
      queue<int> routeAfss;
      list<Vertex> route;
      route.push_back(*all[curr]);
      do {
        //get new neighborhood
        for (auto it  = all.rbegin(); it != all.rend(); ++it){
          int i = it->first;
          if (x_vals[k][curr][i] > INTEGRALITY_TOL){
            if (!customers.count(i) && i != depot && !routeAfssSet.count(i)) {
              routeAfssSet.insert(i);
              routeAfss.push(i);
            }
            next = i;    
            break;
          }
        }
        route.push_back(*all[next]);
        x_vals[k][curr][next]--;
        curr = next;
      } while (curr != depot);
      //get remainig neighboring in AFSs
      //use queue here
      while (!routeAfss.empty()) {
        int afs = routeAfss.front();
        routeAfss.pop();
        //while has path
        while (true) {
          bool hasPath = false;
          //dfs
          list<Vertex> partial_route;
          curr = afs;
          next = afs;
          partial_route.push_back(*all[curr]);
          while (true) {
            //get new neighborhood
            for (auto it = all.rbegin(); it != all.rend(); ++it) {
              int i = it->first;
              if (x_vals[k][curr][i] > INTEGRALITY_TOL) {
                //check if i is an afs
                if (!customers.count(i) && i != depot)
                  routeAfss.push(i);
                hasPath = true;
                next = i;    
                break;
              }
            }
            x_vals[k][curr][next]--;
            if (next == afs)
              break;
            partial_route.push_back(*all[next]);
            curr = next;
          }
          if (!hasPath)
            break;
          //find pointer
          auto it = route.begin();
          for (; it->id != afs; it++);
          route.splice(it, partial_route);
        }
      }
      if (route.size() > 2)
        routes.push_back(route);
    }
    solution = new Gvrp_solution(routes, instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void Cubic_model::endVars(){
  for (int k = 0; k < instance.maxRoutes; k++){ 
    for (const pair<int, const Vertex *>& p : all)
      x[k][p.first].end();
    x[k].end();
  }
  x.end();
  e.end(); 
}
