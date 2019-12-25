#include "utils/cplex/compact_model.hpp"
#include "models/vertex.hpp"
#include "models/gvrp_instance.hpp"
#include "models/distances_enum.hpp"

#include <list>
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

using namespace std;
using namespace models;
using namespace utils::cplex;

void getX_var_vals (IloArray<IloArray<IloNumArray> >& x_vals, IloEnv& env, IloArray<IloArray<IloNumVarArray> >& x_var, vector<int>& routes, map<int, Vertex>& all, int ub_afs_visit){
  IloCplex cplex(env);
  for (int k : routes) {
    x_vals[k] = IloArray<IloNumArray> (env, all.size());
    for (auto const& [i, val] : all){
      x_vals[k][i] = IloNumArray (env, all.size(), 0, ub_afs_visit, IloNumVar::Int);
      cplex.getValues(x_vals[k][i], x_var[k][i]);
    }
  }
}

typedef map<int, Vertex> IDVertex;

//    cplex.use(subcycle_constraint(env, x, routes, depot, all, ub_afs_visit)); 
//separation algorithm
ILOLAZYCONSTRAINTCALLBACK5(subcycle_constraint, IloArray<IloArray<IloNumVarArray> >&, x, vector<int>&, routes, int, depot, IDVertex&, all, int, ub_afs_visit) {
  IloEnv env = getEnv();
  IloExpr expr(env);
  IloArray<IloArray<IloNumArray> > x_vals (env, routes.size());
//  getX_var_vals(x_vals, env, x, routes, all, ub_afs_visit);
  //for each route
  for (int k : routes){
    set<int> vertexes;
//    for (auto const& [i, val] : all)
//      for (auto const& [j, val] : all)
//        if (x_vals[k][i][j] > 0){
//          vertexes.insert(i);
//          vertexes.insert(j);
//        }
    for (pair<int, Vertex> i: all)
      for (pair<int, Vertex> j : all)
        if (x_vals[k][i.first][j.first] > 0){
          vertexes.insert(i.first);
          vertexes.insert(j.first);
        }
    //subcycle found
    if (vertexes.find(depot) == vertexes.end()){
      for (auto const& [i, val] : all)
        for (int j : vertexes)
          if (vertexes.find(i) == vertexes.end())
            expr += x[k][i][j];
      add(expr >= 1);
      expr.end();
      expr = IloExpr(env);
    }
  }
}

Compact_model::Compact_model(Gvrp_instance _gvrp_instance): 
  gvrp_instance(_gvrp_instance) {
  }

list<list<Vertex> > Compact_model::run(){
  //setup
  IloEnv env;
  IloCplex cplex(env);
  map<int, Vertex> customers, 
            afss,
            all,
  vector<int> routes (gvrp_instance.customers.size());
  int depot = 0;
  double beta = gvrp_instance.vehicleFuelCapacity;
  //fill customers
  for (Vertex customer : gvrp_instance.customers){
    customers[customer.id] = customer;
    all[customer.id] = customer;
  }
  //fill afss
  for (Vertex afs : gvrp_instance.afss){ 
    afss[afs.id] = afs;
    all[afs.id] = afs;
  }
  //fill all
  all[depot] = gvrp_instance.depot;
  //fill routes
  for (unsigned int i = 0; i < customers.size(); i++)
    routes[i] = i;
  int ub_afs_visit = customers.size() + 1;
  try {
    IloModel model(env);
    IloArray<IloArray<IloNumVarArray> > x (env, customers.size());
    IloNumVarArray e(env, all.size(), 0, beta, IloNumVar::Float);
    //x var
    int x_var_ub = gvrp_instance.distances_enum == SYMMETRIC || gvrp_instance.distances_enum == METRIC ? 1 : ub_afs_visit; 
    for (int k : routes){
      x[k] = IloArray<IloNumVarArray> (env, all.size());
      for (auto const& [i, val] : all)  
        x[k][i] = IloNumVarArray(env, all.size(), 0, x_var_ub, IloNumVar::Int);
    }
    //objective function
    IloExpr fo (env);
    for (int k : routes)  
      for (auto const& [i, val] : all)  
        for (auto const& [j, val] : all)  
          fo +=  gvrp_instance.distances[i][j] * x[k][i][j];
    model.add(IloMinimize(env, fo));
    //constraints
    IloExpr expr(env);    
    //\sum_{v_j \in V} x_{ij}^k = \sum_{v_j \in V} x_{ji}^k, \forall v_i \in V, \forall k \in M
    for (int k : routes)  
      for (auto const& [i, val] : all)  
        for (auto const& [j, val1] : all)  
          expr += x[k][i][j] - x[k][j][i];
    model.add(expr == 0);
    expr.end();
    expr = IloExpr(env);
    //\sum_{v_i \in V} x_{0i}^k \leqslant 1, \forall k in M
    for (int k : routes)  
      for (auto const& [i, val] : all)  
          expr += x[k][depot][i];
    model.add(expr =< 1);
    expr.end();
    expr = IloExpr(env);
    //\sum_{k \in M} \sum_{v_j \in V} x_{ij}^{k} = 1, \forall v_i \in C
    for (auto const& [i, val] : customers){
      for (int k : routes) 
        for (auto const& [j, val1] : all)  
          expr += x[k][i][j];
      model.add(expr == 1);
      expr.end();
      expr = IloExpr(env);
    }
    //e_0 = \beta
    model.add(e[depot] == beta);
    //e_f = \beta, \forall v_f \in F
    for (auto const& [f, val] : afss)  
      model.add(e[f] == beta);
    //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k), \forall v_j \in C,\forall v_i \in V, \forall k \in M
    for (int k : routes)  
      for (auto const& [j, val] : customers)  
        for (auto const& [i, val1] : all){
          expr = e[i] - gvrp_instance.distances[i][j] * x[k][i][j] + beta * (1 - x[k][i][j]);
          model.add(e[j] <= expr);
          expr.end();
          expr = IloExpr (env);
        }
    //e_i \geq c_{ij} x_{ij}^k, \forall v_i, \forall v_j \in V, \forall k \in M
    for (int k : routes)
      for (auto const& [i, val] : customers)
        for (auto const& [j, val1] : all){
          expr = gvrp_instance.distances[i][j] * x[k][i][j];
          model.add(e[i] >= expr);
          expr.end();
          expr = IloExpr(env);
      }
    //x_{ij}^k c_{ij} \leq \beta, \forall v_i, \forall v_j \in V, \forall k \in M
    for (int k : routes)
      for (auto const& [i, val] : all)
        for (auto const& [j, val1] : all){
          expr = gvrp_instance.distances[i][j] * x[k][i][j];
          model.add(expr <= beta);
          expr.end();
          expr = IloExpr(env);
      }
    //\sum_{(v_i, v_j) \in \delta(S)} x_{ij}^k \geq 2, \forall k \in M, \forall S \subseteq V \backlash \{v_0\} : |C \wedge S| \geq 1 \wedge |S \wedge F| \geq 1
    cplex.use(subcycle_constraint(env, x, routes, depot, all, ub_afs_visit)); 
    //run model
    IloCplex cplex(model);
    if ( !cplex.solve() ) {
      env.error() << "Failed to optimize LP." << endl;
      throw(-1);
    }
    env.out() << "Solution status = " << cplex.getStatus() << endl;
    env.out() << "Solution value = " << cplex.getObjValue() << endl;
  }catch (IloException& e) {
    cerr << "Concert exception caught: " << e << endl;
  }
  catch (...) {
    cerr << "Unknown exception caught" << endl;
  }
  //getresult
  list<list<vertex> > _routes;
  IloArray<IloArray<IloNumArray> > x_vals (env, routes.size());
  getX_var_vals(x_vals, env, x, routes, all, ub_afs_visit);
  env.end();
    //dfs
  for (int k : routes){
    int curr = depot, next = depot;
    //checking if the route is used
    //get remaining nodes (if exists)
    list<vertex> route;
    route.push_back(curr);
    do {
      //get new neighborhood
      for (int i : all)
        if (curr != i && x_vals[k][curr][i] > 0){
          next = i;    
          break;
        }
      route.push_back(next);
      x_vals[k][curr][next]--;
      curr = next;
    } while (next != depot);
    _routes.push_back(route);
  }
  return _routes;
}
