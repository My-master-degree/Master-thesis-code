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
#include <stdlib.h>
ILOSTLBEGIN

typedef map<int, Vertex> IDVertex;
typedef IloArray<IloArray<IloNumVarArray> > Matrix3DVar;
typedef IloArray<IloArray<IloNumArray> > Matrix3DVal;

using namespace std;
using namespace models;
using namespace utils::cplex;

//functions prototyps
IloModel* createModel(IloEnv& env, IloNumVarArray& e, Matrix3DVar& x,  Gvrp_instance& gvrp_instance, IDVertex& all);

void setCustomParameters(IloCplex& cplex);

//function declarations
Compact_model::Compact_model(Gvrp_instance& _gvrp_instance): 
  gvrp_instance(_gvrp_instance) {
  }

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

list<list<Vertex> > Compact_model::run(){
  //setup
  IDVertex  all;
  int depot = 0;
  double beta = gvrp_instance.vehicleFuelCapacity;
  int ub_afs_visit = gvrp_instance.customers.size() + 1;
  int ub_edge_visit = gvrp_instance.distances_enum == SYMMETRIC || gvrp_instance.distances_enum == METRIC ? 1 : ub_afs_visit; 
  IloEnv env;
  IloModel* model;
  //fill all
  for (Vertex customer : gvrp_instance.customers)
    all[customer.id] = customer;
  for (Vertex afs : gvrp_instance.afss) 
    all[afs.id] = afs;
  all[depot] = gvrp_instance.depot;
  Matrix3DVar x (env, gvrp_instance.customers.size());
  IloNumVarArray e(env, all.size(), 0, beta, IloNumVar::Float);
  try {
    //x var
    for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++){
      x[k] = IloArray<IloNumVarArray> (env, all.size());
      for (pair<int, Vertex> p : all){
        int i = p.first;
        x[k][i] = IloNumVarArray(env, all.size(), 0, ub_edge_visit, IloNumVar::Int);
      }
    }
    //create model
    model = createModel(env, e, x, gvrp_instance, all);
    IloCplex cplex(*model);
    //\sum_{(v_i, v_j) \in \delta(S)} x_{ij}^k \geq 2, \forall k \in M, \forall S \subseteq V \backlash \{v_0\} : |C \wedge S| \geq 1 \wedge |S \wedge F| \geq 1
    cplex.use(Subcycle_constraint(env, x, gvrp_instance, all, ub_edge_visit)); 
    setCustomParameters(cplex);
    if ( !cplex.solve() ) {
      env.error() << "Failed to optimize LP." << endl;
      throw(-1);
    }else{
      env.out() << "Solution status = " << cplex.getStatus() << endl;
      env.out() << "Solution value = " << cplex.getObjValue() << endl;
      //getresult
      Matrix3DVal x_vals (env, gvrp_instance.customers.size());
      for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++) {
        x_vals[k] = IloArray<IloNumArray> (env, all.size());
        for (pair<int, Vertex> p : all){
          int i = p.first;
          x_vals[k][i] = IloNumArray (env, all.size(), 0, ub_edge_visit, IloNumVar::Int);
          cplex.getValues(x_vals[k][i], x[k][i]);
        }
      }
      env.end();
      //dfs
      int curr,
          next;
      list<list<Vertex> > routes;
      for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++) {
        curr = depot;
        next = depot;
        //checking if the route is used
        //get remaining nodes (if exists)
        list<Vertex> route;
        route.push_back(all[curr]);
        do {
          //get new neighborhood
          for (pair<int, Vertex> p : all){
            int i = p.first;
            if (curr != i && x_vals[k][curr][i] > 0){
              next = i;    
              break;
            }
          }
          route.push_back(all[next]);
          x_vals[k][curr][next]--;
          curr = next;
        } while (curr != depot);
        routes.push_back(route);
      }
      //print
      for (list<Vertex> route : routes) {
        for (Vertex vertex : route)
          cout<<vertex.id<<",";
        cout<<endl;
      }

      return routes;
    }
  }catch (IloException& e) {
    cerr << "Concert exception caught: " << e << endl;
    exit(EXIT_FAILURE);
  }
  catch (...) {
    cerr << "Unknown exception caught" << endl;
    exit(EXIT_FAILURE);
  }
}


IloModel* createModel(IloEnv& env, IloNumVarArray& e,Matrix3DVar& x,  Gvrp_instance& gvrp_instance, IDVertex& all){
  int depot = gvrp_instance.depot.id;
  double beta = gvrp_instance.vehicleFuelCapacity;
  IloModel* model;
  model = new IloModel (env);
  //objective function
  IloExpr fo (env);
  for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
    for (pair<int, Vertex> p : all){
      int i = p.first;
      for (pair<int, Vertex> p1 : all){
        int j = p1.first;
        fo +=  gvrp_instance.distances[i][j] * x[k][i][j];
      }
    }  
  model->add(IloMinimize(env, fo));
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
      model->add(expr == expr1);
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
    model->add(expr <= 1);
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
    model->add(expr == 1);
    expr.end();
    expr = IloExpr(env);
  }
  //e_0 = \beta
  expr = e[depot];
  model->add(expr == beta);
  expr.end();
  expr = IloExpr(env);
  //e_f = \beta, \forall v_f \in F
  for (Vertex afs : gvrp_instance.afss)  {
    int f = afs.id;
    model->add(e[f] == beta);
  }
  //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k), \forall v_j \in C,\forall v_i \in V, \forall k \in M
  for (unsigned int k = 0; k < gvrp_instance.customers.size(); k++)
    for (Vertex customer : gvrp_instance.customers) {
      int j = customer.id;
      for (pair<int, Vertex> p :all){
        int i =  p.first;
        expr = e[i] - gvrp_instance.distances[i][j] * x[k][i][j] + beta * (1 - x[k][i][j]);
        model->add(e[j] <= expr);
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
        expr = gvrp_instance.distances[i][j] * x[k][i][j];
        model->add(e[i] >= expr);
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
        expr = gvrp_instance.distances[i][j] * x[k][i][j];
        model->add(expr <= beta);
        expr.end();
        expr = IloExpr(env);
      }
    }
  return model;
}

void setCustomParameters(IloCplex& cplex){
  //run model
  // Turn off the presolve reductions and set the CPLEX optimizer
  //    // to solve the worker LP with primal simplex method.
  //
  cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);
  cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal); 
  //thread safe setting
  cplex.setParam(IloCplex::Param::Threads, 1);
  //preprocesing setting
  cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse); 
  // Turn on traditional search for use with control callbacks
  cplex.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);
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
}
