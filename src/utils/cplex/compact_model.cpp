#include "utils/cplex/compact_model.hpp"
#include "models/vertex.hpp"
#include "models/gvrp_instance.hpp"

#include <list>
#include <map>
#include <set>
#include <queue>
#include <ilcplex/ilocplex.h>
#include <models/distances_enum.hpp>
ILOSTLBEGIN

using namespace std;
using namespace models;
using namespace utils::cplex;

//separation algorithm
ILOLAZYCONSTRAINTCALLBACK7(subcycle_constraint, IloArray<IloArray<IloNumVarArray> > &, x, set<int> &, routes, set<int> &, customers, set<int> &, afss, int, depot, set<int> &, all, int, ub_afs_visit) {
  IloEnv env = getEnv();
  IloExpr expr(env);

  IloArray<IloArray<IloNumArray> > x_vals (env, routes.size());
  for (int k : routes) {
    x_vals[k] = IloArray<IloNumArray> (env, all.size());
    for (int i : all){
      x_vals[k][i] = IloNumArray (env, all.size(), 0, ub_afs_visit, IloNumVar::Int);
      getValues(x_vals[k][i], x[k][i]);
    }
  }
  //for each route
  for (int k : routes){
    set<int> vertexes;
    for (int i : all)
      for (int j : all)
        if (x_vals[k][i][j] > 0){
          vertexes.insert(i);
          vertexes.insert(j);
        }
    //subcycle found
    if (vertexes.find(depot) == vertexes.end()){
      for (int i : all)
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
  list<list<Vertex> > routes;
  set<int> customers, 
            afss,
            all,
            routes;
  int depot = 0;
  double beta = beta;
  //fill customers
  for (Vertex customer : gvrp_instance.customers)
    customers.insert(customer.id);
  //fill afss
  for (Vertex afs : gvrp_instance.afss)    
    afss.insert(afs.id);
  //fill all
  all.insert(depot);
  all.insert(customers.begin(), customers.end());
  all.insert(afss.begin(), afss.end());
  //fill routes
  for (int i = 0; i < customers.size(); i++)
    routes.insert(i);
  int ub_afs_visit = customers.size() + 1;
  try {
    IloModel model(env);
    IloArray<IloArray<IloNumVarArray> > x (env, customers.size());
    IloNumVarArray e(env, all.size(), 0, beta, IloNumVar::Float);
    //x var
    int x_var_ub = gvrp_instance.distances_enum == SYMMETRIC || gvrp_instance.distances_enum == METRIC ? 1 : ub_afs_visit; 
    for (int k : routes)  
      x[k] = IloArray<IloNumVarArray> (env, all.size());
      for (int i : all)  
        x[k][i] = IloNumVarArray(env, all.size(), 0, x_var_ub, IloNumVar::Int);
    }
    //objective function
    IloExpr fo (env);
    for (int k : routes)  
      for (int i : all)  
        for (int j : all)  
          fo +=  gvrp_instance.distances[i][j] * x[k][i][j];
    model.add(IloMinimize(env, fo));
    //constraints
    IloExpr expr(env);    
    //\sum_{v_j \in V} x_{ij}^k = \sum_{v_j \in V} x_{ji}^k, \forall v_i \in V, \forall k \in M
    for (int k : routes)  
      for (int i : all)  
        for (int j : all)  
          expr += x[k][i][j] - x[k][j][i];
    model.add(expr == 0)
    expr.end();
    expr = IloExpr(env);
    //\sum_{k \in M} \sum_{v_j \in V} x_{ij}^{k} = 1, \forall v_i \in C
    for (int i : routes) {
      for (int k : customers)  
        for (int j : all)  
          expr += x[k][i][j];
      model.add(expr == 1);
      expr.end();
      expr = IloExpr(env);
    }
    //e_0 = \beta
    model.add(e[depot] == beta);
    //e_f = \beta, \forall v_f \in F
    for (int f : afss)  
      model.add(e[f] == beta);
    //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k), \forall v_j \in C,\forall v_i \in V, \forall k \in M
    for (int k : routes)  
      for (int j : customers)  
        for (int i : all){
          expr = e[i] - gvrp_instance.distances[i][j] * x[k][i][j] + beta * (1 - x[k][i][j]);
          model.add(e[j] <= expr);
          expr.end();
          expr = IloExpr (env);
        }
    //e_i \geq c_{ij} x_{ij}^k, \forall v_i, \forall v_j \in V, \forall k \in M
    for (int k : routes)
      for (int i : customers)
        for (int j : all){
          expr = gvrp_instance.distances[i][j] * x[k][i][j];
          model.add(e[i] >= expr);
          expr.end();
          expr = IloExpr(env);
      }
    //x_{ij}^k c_{ij} \leq \beta, \forall v_i, \forall v_j \in V, \forall k \in M
    for (int k : routes)
      for (int i : all)
        for (int j : all){
          expr = gvrp_instance.distances[i][j] * x[k][i][j];
          model.add(expr =< beta);
          expr.end();
          expr = IloExpr(env);
      }
    //\sum_{(v_i, v_j) \in \delta(S)} x_{ij}^k \geq 2, \forall k \in M, \forall S \subseteq V \backlash \{v_0\} : |C \wedge S| \geq 1 \wedge |S \wedge F| \geq 1
    cplex.use(subcycle_constraint(env, x, routes, customers, afss, depot)); 
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
  env.end();
  return routes;
}
