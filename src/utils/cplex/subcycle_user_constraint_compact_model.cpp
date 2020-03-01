#include "utils/cplex/lazy_constraint_compact_model.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/subcycle_user_constraint_compact_model.hpp"

#include <set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>
/*
#include <lemon/gomory_hu.h>
#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>
*/

using namespace std;
using namespace utils::cplex;
/*
using namespace lemon;
using namespace lemon::concepts;
*/

Subcer_constraint_compact_model::Subcycle_user_constraint_compact_model (Compact_model& compact_model_) : User_constraint_compact_model (compact_model_) {}

IloCplex::CallbackI* Subcycle_user_constraint_compact_model::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint_compact_model (*this);
}

void Subcycle_user_constraint_compact_model::main() {
  if (!compact_model.ALLOW_SUBCYCLE_USER_CUT)
    return;
  int depot = compact_model.gvrp_instance.depot.id,
      i,
      j;
  IloEnv env = getEnv();
  IloExpr lhs(env);
  bool infeasibilityFound = false;
//  vector<ListGraph::Node> nodes (all.size());
  size_t k;
  //get values
  Matrix3DVal x_vals (env, compact_model.gvrp_instance.customers.size());
  for (k = 0; k < compact_model.gvrp_instance.customers.size(); k++) {
    x_vals[k] = IloArray<IloNumArray> (env, compact_model.all.size());
    for (const pair<int, Vertex>& p : compact_model.all) {
      i = p.first;
      x_vals[k][i] = IloNumArray (env, compact_model.all.size(), 0, compact_model.ub_edge_visit, IloNumVar::Float);
      getValues(x_vals[k][i], compact_model.x[k][i]);
    }
  }
  //get subcycles
  for (k = 0; k < compact_model.gvrp_instance.customers.size(); k++) {
    /*
    //gomory hu
    ListGraph graph;
    ListGraph::EdgeMap<double>  weight(graph); 
    //creating nodes
    for (const pair<int, Vertex>& p : compact_model.all) 
      nodes[p.first] = graph.addNode();
    //creating edges
    for (const pair<int, Vertex>& p : compact_model.all) {
      i = p.first;
      for (const pair<int, Vertex>& p1 : compact_model.all) {
        j = p1.first;
        weight[graph.addEdge(nodes[i], nodes[j])] = x_vals[k][i][j];
      }
    }
    GomoryHu<ListGraph, ListGraph::EdgeMap<double> > gh (graph, weight);
    gh.run();
    //get subcycles
    for (int i : compact_model.customers) {
    if (gh.minCutValue(nodes[i], nodes[0]) < 2.0) {

    }
    }
    */
    //bfs to remove all edges connected to the depot
    queue<int> q;
    q.push(depot);
    while (!q.empty()) {
      int curr = q.front();
      q.pop();
      for (pair<int, Vertex> p : compact_model.all)
        if (x_vals[k][curr][p.first] > EPS){
          x_vals[k][curr][p.first] = 0.0;
          q.push(p.first);
        }        
    }
    //bfs to remove all edges not connected to the depot
    for (Vertex customer : compact_model.gvrp_instance.customers) {
      set<int> component, customersComponent;
      //checking for neighboring
      bool hasNeighboring = false;
      for (pair<int, Vertex> p : compact_model.all)
        if (x_vals[k][customer.id][p.first] > EPS){
          hasNeighboring = true;
          break; 
        }
      //checking if bfs is needed
      if (!hasNeighboring)
        continue;
      infeasibilityFound = true;
      q.push(customer.id);
      while (!q.empty()) {
        int curr = q.front();
        q.pop();
        component.insert(curr);
        //if it is a customer
        if (compact_model.customers.count(curr))
          customersComponent.insert(curr);
        for (pair<int, Vertex> p : compact_model.all)
          if (x_vals[k][curr][p.first] > EPS){
            x_vals[k][curr][p.first] = 0.0;
            q.push(p.first);
          }
      }
      for (int customer_ : customersComponent) {
        //getting lhs
        for (pair<int, Vertex> p2 : compact_model.all) {
          int a = p2.first;
          if (!component.count(a))
            for (int b : component) 
              lhs += compact_model.x[k][a][b];
        }
        //getting rhs
        for (int b : component)
          lhs -= compact_model.x[k][b][customer_];
        try {
          add(lhs >= 0.0).end();
        } catch(IloException& e) {
          cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
          throw;
        }
        lhs.end();
        lhs = IloExpr(env);
      } 
    }
    for (pair<int, Vertex> p : compact_model.all){
      int i = p.first;
      x_vals[k][i].end();
    }
  }
  x_vals.end();
  compact_model.ALLOW_SUBCYCLE_USER_CUT = infeasibilityFound;
}
