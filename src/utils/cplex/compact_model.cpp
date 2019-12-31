#include "utils/cplex/compact_model.hpp"
#include "models/vertex.hpp"
#include "models/gvrp_instance.hpp"
#include "models/distances_enum.hpp"

#include <list>
#include <set>
#include <vector>
#include <queue>
#include <ilcplex/ilocplex.h>
#include <stdlib.h>
#include <exception>
ILOSTLBEGIN


using namespace std;
using namespace models;
using namespace utils::cplex;

//separation algorithm
ILOLAZYCONSTRAINTCALLBACK4(Subcycle_constraint, Matrix3DVar&, x, Gvrp_instance&, gvrp_instance, IDVertex&, all, int, ub_edge_visit) {
  int depot = gvrp_instance.depot.id;
  IloEnv env = getEnv();
  IloExpr expr(env);
  Matrix3DVal x_vals (env, gvrp_instance.customers.size());
  for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++) {
    x_vals[k] = IloArray<IloNumArray> (env, all.size());
    for (pair<int, Vertex> p : all) {
      int i = p.first;
      x_vals[k][i] = IloNumArray (env, all.size(), 0, ub_edge_visit, IloNumVar::Int);
      getValues(x_vals[k][i], x[k][i]);
    }
  }
  //for each route
  for (int k = 0; k < int(gvrp_instance.customers.size()); k++){
    set<int> vertexes;
    for (pair<int, Vertex> p : all){
      int i = p.first;
      for (pair<int, Vertex> p1 : all){
        int j = p1.first;
        if (x_vals[k][i][j] > 0){
          vertexes.insert(i);
          vertexes.insert(j);
        }
      }
    }
    //subcycle found
    if (vertexes.size() > 0 && vertexes.find(depot) == vertexes.end()){
      for (pair<int, Vertex> p : all){
        int i = p.first;
        for (int j : vertexes)
          if (vertexes.find(i) == vertexes.end())
            expr += x[k][i][j];
      }       
      expr -= 1;
      try {
        add(expr >= 0).end();
      } catch(IloException& e) {
        std::cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
        throw;
      } 
      expr.end();
      expr = IloExpr(env);
    }
    for (pair<int, Vertex> p : all){
      int i = p.first;
      x_vals[k][i].end();
    }
  }
  x_vals.end();
}


Compact_model::Compact_model(Gvrp_instance& _gvrp_instance, unsigned int _time_limit): 
  gvrp_instance(_gvrp_instance), time_limit(_time_limit), max_num_feasible_integer_sol(2100000000) {
  ub_edge_visit = gvrp_instance.distances_enum == SYMMETRIC || gvrp_instance.distances_enum == METRIC ? 1 : gvrp_instance.customers.size() + 1; 
  //fill all
  for (Vertex customer : gvrp_instance.customers)
    all[customer.id] = customer;
  for (Vertex afs : gvrp_instance.afss) 
    all[afs.id] = afs;
  all[gvrp_instance.depot.id] = gvrp_instance.depot;
}

Compact_model::Compact_model(Gvrp_instance& gvrp_instance, unsigned int time_limit, unsigned int _max_num_feasible_integer_sol): 
  Compact_model(gvrp_instance, time_limit) {
  max_num_feasible_integer_sol = _max_num_feasible_integer_sol;

}

list<list<Vertex> > Compact_model::run(){
  //setup
  try {
    cout<<"Creating variables"<<endl;
    createVariables();
    cout<<"Creating objective function"<<endl;
    createObjectiveFunction();
    cout<<"Creating model"<<endl;
    createModel();
    cout<<"Setting parameters"<<endl;
    setCustomParameters();
    if ( !cplex.solve() ) {
      env.error() << "Failed to optimize LP." << endl;
      exit(EXIT_FAILURE);
    }
    env.out() << "Solution status = " << cplex.getStatus() << endl;
    env.out() << "Solution value = " << cplex.getObjValue() << endl;
    getSolution();
    getRoutes();
    env.end();
    return routes;
  }catch (IloException& e) {
    cerr << "Concert exception caught: " << e << endl;
    exit(EXIT_FAILURE);
  }
  catch (const char * s) {
    cerr << s << endl;
    exit(EXIT_FAILURE);
  }
}

void Compact_model::createVariables(){
  x = Matrix3DVar (env, gvrp_instance.customers.size());
  e = IloNumVarArray (env, all.size(), 0, gvrp_instance.vehicleFuelCapacity, IloNumVar::Float);
  t = Matrix3DVar (env, gvrp_instance.customers.size());
  try {
    //x var
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++){
      x[k] = IloArray<IloNumVarArray> (env, all.size());
      t[k] = IloArray<IloNumVarArray> (env, all.size());
      for (pair<int, Vertex> p : all){
        int i = p.first;
        x[k][i] = IloNumVarArray(env, all.size(), 0, ub_edge_visit, IloNumVar::Int);
        t[k][i] = IloNumVarArray(env, all.size(), 0, gvrp_instance.timeLimit, IloNumVar::Float);
      }
    }
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw "Error in creating variables";
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
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw "Error in creating the objective function";
  }
}

void Compact_model::createModel() {
  try{
    int depot = gvrp_instance.depot.id;
    double beta = gvrp_instance.vehicleFuelCapacity;
    double T = gvrp_instance.timeLimit;
    //constraints
    IloExpr expr(env), expr1(env);    
    //\sum_{v_j \in V} x_{ij}^k = \sum_{v_j \in V} x_{ji}^k, \forall v_i \in V, \forall k \in M
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
      for (pair<int, Vertex> p : all){
        int i = p.first;
        for (pair<int, Vertex> p1 : all){
          int j = p1.first;
          expr += x[k][i][j];
          expr1 += x[k][j][i];
        }
        model.add(expr == expr1);
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
      model.add(expr <= 1);
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
      model.add(expr == 1);
      expr.end();
      expr = IloExpr(env);
    }
    //e_0 = \beta
    expr = e[depot];
    model.add(expr == beta);
    expr.end();
    expr = IloExpr(env);
    //e_f = \beta, \forall v_f \in F
    for (Vertex afs : gvrp_instance.afss)  {
      int f = afs.id;
      model.add(e[f] == beta);
    }
    //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k), \forall v_j \in C,\forall v_i \in V, \forall k \in M
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
      for (Vertex customer : gvrp_instance.customers) {
        int j = customer.id;
        for (pair<int, Vertex> p :all){
          int i =  p.first;
          expr = e[i] - gvrp_instance.distances[i][j] * x[k][i][j] * gvrp_instance.vehicleFuelConsumptionRate + beta * (1 - x[k][i][j]);
          model.add(e[j] <= expr);
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
          model.add(e[i] >= expr);
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
          model.add(expr <= beta);
          expr.end();
          expr = IloExpr(env);
        }
      }
    //t[k][i][j] >= t[k][i] + service_time[j] + travel_times[i][j] - T(1 - x[k][i][j]) \forall k \in M, \forall i,j \in V
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
      for (par<int, Vertex> p : all) {
        int j = p.id;
        for (pair<int, Vertex> p1 :all){
          int i = p.first;
          expr = t[k][i] + gvrp_instance.distances[i][j] * x[k][i][j] * gvrp_instance.vehicleAverageSpeed + p.serviceTime - T * (1 - x[k][i][j]);
          model.add(t[k][j] >= expr);
          expr.end();
          expr = IloExpr (env);
        }
      }
    cplex = IloCplex(model);
    //\sum_{(v_i, v_j) \in \delta(S)} x_{ij}^k \geq 2, \forall k \in M, \forall S \subseteq V \backlash \{v_0\} : |C \wedge S| \geq 1 \wedge |S \wedge F| \geq 1
    cplex.use(Subcycle_constraint(env, x, gvrp_instance, all, ub_edge_visit)); 
  } catch (IloException& e) {
    throw e;
  } catch (runtime_error& e) {
    throw e;
  }
}

void Compact_model::setCustomParameters(){
  try{
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
    throw "Error in setting parameters";
  }
}

void Compact_model::getSolution(){
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
    throw "Error in getting solution";
  }
}

void Compact_model::getRoutes(){
  //dfs
  try{
    int depot = gvrp_instance.depot.id;
    int curr,
        next;
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++) {
      curr = depot;
      next = depot;
      //checking if the route is used
      //get remaining nodes (if exists)
      list<Vertex> route;
      route.push_back(all[curr]);
      do {
        //get new neighborhood
        for (auto it  = all.rbegin(); it != all.rend(); ++it){
          int i = it->first;
          if (x_vals[k][curr][i] > 0){
            next = i;    
            break;
          }
        }
        route.push_back(all[next]);
        x_vals[k][curr][next]--;
        curr = next;
      } while (curr != depot);
      if (route.size() > 2)
        routes.push_back(route);
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw "Error in getting routes";
  }
}

