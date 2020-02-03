#include "models/vertex.hpp"
#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/distances_enum.hpp"
#include "models/mip_solution_info.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/subcycle_lazy_constraint_compact_model.hpp"
#include "utils/cplex/lazy_constraint_compact_model.hpp"
#include "utils/cplex/user_constraint_compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/preprocessing_compact_model.hpp"

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
using namespace utils::cplex;

Compact_model::Compact_model(Gvrp_instance& _gvrp_instance, unsigned int _time_limit): 
  gvrp_instance(_gvrp_instance), time_limit(_time_limit), max_num_feasible_integer_sol(2100000000), VERBOSE(true) {
  ub_edge_visit = gvrp_instance.distances_enum == SYMMETRIC || gvrp_instance.distances_enum == METRIC ? 1 : gvrp_instance.customers.size() + 1; 
  //fill all and customers
  for (Vertex customer : gvrp_instance.customers) {
    all[customer.id] = customer;
    customers.insert(customer.id);
  }
  for (Vertex afs : gvrp_instance.afss) 
    all[afs.id] = afs;
  all[gvrp_instance.depot.id] = gvrp_instance.depot;
}

pair<Gvrp_solution, Mip_solution_info> Compact_model::run(){
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
    time_t start = clock();
    if ( !cplex.solve() ) {
//      env.error() << "Failed to optimize LP." << endl;
      mipSolInfo = Mip_solution_info(-1, cplex.getStatus(), -1, -1);
      endVars();
      env.end();
      throw mipSolInfo;
    }
    double total_time =  (double) (clock() - start) / (double) CLOCKS_PER_SEC;
//    cplex.exportModel("cplexcpp.lp");
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    cout<<"Getting x values"<<endl;
    fillX_vals();
//    cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
    mipSolInfo = Mip_solution_info(cplex.getMIPRelativeGap(), cplex.getStatus(), total_time, cplex.getObjValue());
    endVars();
    env.end();
    return make_pair(*gvrp_solution, mipSolInfo);
  } catch (IloException& e) {
    output_exception<<"Concert exception caught: " << e<<endl;
    throw output_exception.str();
  } catch (string s) {
    throw s;
  }
  
}

void Compact_model::createVariables(){
  x = Matrix3DVar (env, gvrp_instance.customers.size());
  e = IloNumVarArray (env, all.size(), 0, gvrp_instance.vehicleFuelCapacity, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for (pair<int, Vertex> p : all){
      int i = p.first;
      nameStream<<"e["<<i<<"]";
      const string name = nameStream.str();
      e[i].setName(name.c_str());
      nameStream.clear();
      nameStream.str("");
    }
    //x var
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++){
      x[k] = IloArray<IloNumVarArray> (env, all.size());
      for (pair<int, Vertex> p : all){
        int i = p.first;
        x[k][i] = IloNumVarArray(env, all.size(), 0, ub_edge_visit, IloNumVar::Int);
        //setting names
        for (pair<int, Vertex> p1 : all){
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

void Compact_model::createObjectiveFunction() {
//objective function
  try{
    IloExpr fo (env);
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
      for (pair<int, Vertex> p : all){
        int i = p.first;
        for (pair<int, Vertex> p1 : all){
          int j = p1.first;
          fo +=  gvrp_instance.distances[i][j] * x[k][i][j];
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

void Compact_model::createModel() {
  try {
    //preprocessing conditions
    for (Preprocessing_compact_model* preprocessing : preprocessings)
      preprocessing->add();
    //setup
    int depot = gvrp_instance.depot.id;
    double beta = gvrp_instance.vehicleFuelCapacity;
    double T = gvrp_instance.timeLimit;
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //\sum_{v_j \in V} x_{ij}^k = \sum_{v_j \in V} x_{ji}^k, \forall v_i \in V, \forall k \in M
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
      for (pair<int, Vertex> p : all){
        int i = p.first;
        for (pair<int, Vertex> p1 : all){
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
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++){
      for (pair<int, Vertex> p : all){
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
    for (Vertex customer :gvrp_instance.customers){
      int i = customer.id;
      for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++){
        for (pair<int, Vertex> p1 : all){
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
    //e_0 = \beta
    c = IloConstraint (e[depot] == beta);
    c.setName("e_depot = beta");
    model.add(c);
    //e_f = \beta, \forall v_f \in F
    for (Vertex afs : gvrp_instance.afss)  {
      int f = afs.id;
      c = IloConstraint (e[f] == beta);
      c.setName("e_f = beta");
      model.add(c);
    }
    //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k), \forall v_j \in C,\forall v_i \in V, \forall k \in M
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
      for (Vertex customer : gvrp_instance.customers) {
        int j = customer.id;
        for (pair<int, Vertex> p :all){
          int i =  p.first;
          expr = e[i] - x[k][i][j] * gvrp_instance.distances[i][j] * gvrp_instance.vehicleFuelConsumptionRate + beta * (1 -  x[k][i][j]);
          c = IloConstraint (e[j] <= expr);
          c.setName("updating fuel level");
          model.add(c);
          expr.end();
          expr = IloExpr (env);
        }
      }
    //e_i \geq c_{ij} x_{ij}^k, \forall v_i, \forall v_j \in V, \forall k \in M
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
      for (pair<int, Vertex> p : all){
        int i = p.first;
        for (pair<int, Vertex> p1 : all){
          int j = p1.first;
          expr = gvrp_instance.distances[i][j] * x[k][i][j] * gvrp_instance.vehicleFuelConsumptionRate;
          c = IloConstraint (e[i] >= expr);
          c.setName("disabling infeasible edges");
          model.add(c);
          expr.end();
          expr = IloExpr(env);
        }
      }
    //x_{ij}^k c_{ij} \leq \beta, \forall v_i, \forall v_j \in V, \forall k \in M
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
      for (pair<int, Vertex> p : all){
        int i = p.first;
        for (pair<int, Vertex> p1 : all){
          int j = p1.first;
          expr = gvrp_instance.distances[i][j] * x[k][i][j] * gvrp_instance.vehicleFuelConsumptionRate;
          c = IloConstraint (expr <= beta);
          c.setName("disabling infeasible edges 2");
          model.add(c);
          expr.end();
          expr = IloExpr(env);
        }
      }
    //\sum_{(i, j) \in E} x_{ij}^k ((c_{ij} / S) + time(v_i) )\leq T, \forall k \in M
//    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++){
//     for (pair<int, Vertex> p : all){
//       int i = p.first;
//       for (pair<int, Vertex> p1 : all){
//         int j = p1.first;
//         expr += x[k][i][j] * ((gvrp_instance.distances[i][j] / gvrp_instance.vehicleAverageSpeed) + p.second.serviceTime);
//       }
//     }
//     c = IloConstraint (expr <= T);
//     c.setName("time limit constraint");
//     model.add(c);
//     expr.end();
//     expr = IloExpr(env);
//   }
    //extra constraints
    for (Extra_constraint_compact_model* extra_constraint : extra_constraints) 
      extra_constraint->add();
    //init
    cplex = IloCplex(model);
    //\sum_{(v_i, v_j) \in \delta(S)} x_{ij}^k \geq 2, \forall k \in M, \forall S \subseteq V \backlash \{v_0\} : |C \wedge S| \geq 1 \wedge |S \wedge F| \geq 1
    //cplex.use(Subcycle_constraint(env, x, gvrp_instance, all, ub_edge_visit));
    cplex.use(separation_algorithm()); 
    //user cuts
    for (User_constraint_compact_model* user_constraint : user_constraints)
      cplex.use(user_constraint);
    //extra steps
    extraStepsAfterModelCreation();
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

void Compact_model::extraStepsAfterModelCreation() {
  //
}

Lazy_constraint_compact_model* Compact_model::separation_algorithm(){
  return new Subcycle_lazy_constraint_compact_model(*this);
}

void Compact_model::setCustomParameters(){
  try{
    if (!VERBOSE)
      cplex.setOut(env.getNullStream());
    //DOUBTS:
    // Turn off the presolve reductions and set the CPLEX optimizer
    // to solve the worker LP with primal simplex method.
    cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);
    cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal); 
    //preprocesing setting
    cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse); 
    // Turn on traditional search for use with control callbacks
    cplex.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);
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
    //time out
    cplex.setParam(IloCplex::Param::TimeLimit, time_limit);
    //num of feasible integers solution allowed
    cplex.setParam(IloCplex::Param::MIP::Limits::Solutions, max_num_feasible_integer_sol);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void Compact_model::fillX_vals(){
  //getresult
  try{
    x_vals = Matrix3DVal (env, gvrp_instance.customers.size());
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++) {
      x_vals[k] = IloArray<IloNumArray> (env, all.size());
      for (pair<int, Vertex> p : all){
        int i = p.first;
        x_vals[k][i] = IloNumArray (env, all.size(), 0, ub_edge_visit, IloNumVar::Int);
        cplex.getValues(x_vals[k][i], x[k][i]);
      }
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void Compact_model::createGvrp_solution(){
  /*
  //print x vals
  for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++) {
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
    int depot = gvrp_instance.depot.id;
    int curr,
        next;    
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++) {
      curr = depot;
      next = depot;
      //checking if the route is used
      //get remaining nodes (if exists)
      set<int> routeAfssSet;
      queue<int> routeAfss;
      list<Vertex> route;
      route.push_back(all[curr]);
      do {
        //get new neighborhood
        for (auto it  = all.rbegin(); it != all.rend(); ++it){
          int i = it->first;
          if (x_vals[k][curr][i] > 0){
            if (!customers.count(i) && i != depot && !routeAfssSet.count(i)) {
              routeAfssSet.insert(i);
              routeAfss.push(i);
            }
            next = i;    
            break;
          }
        }
        route.push_back(all[next]);
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
          partial_route.push_back(all[curr]);
          while (true) {
            //get new neighborhood
            for (auto it = all.rbegin(); it != all.rend(); ++it) {
              int i = it->first;
              if (x_vals[k][curr][i] > 0) {
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
            partial_route.push_back(all[next]);
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
    gvrp_solution = new Gvrp_solution(routes, gvrp_instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void Compact_model::endVars(){
  for (int k = 0; k < int(gvrp_instance.customers.size()); k++) {
    for (pair<int, Vertex> p : all)
      x[k][p.first].end();
    x[k].end();
  }
  x.end();
  e.end(); 
}
