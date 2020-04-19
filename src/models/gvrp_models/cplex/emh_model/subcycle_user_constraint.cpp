#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/emh_model/subcycle_user_constraint.hpp"

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
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models::cplex::emh_model;
/*
   using namespace lemon;
   using namespace lemon::concepts;
   */

Subcycle_user_constraint::Subcycle_user_constraint (EMH_model& emh_model_) : User_constraint (emh_model_) {}

IloCplex::CallbackI* Subcycle_user_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint (*this);
}

void Subcycle_user_constraint::main() {
  Depth const* const d = (Depth *) getNodeData();
  IloInt depth = d ? d->depth : 0;
  if (depth > 0) {
    abortCutLoop();
    return;
  }
  /*
  int depot = emh_model.instance.depot.id;
  //int i,
  //k;
  //  int j;
  IloEnv env = getEnv();
  IloExpr lhs(env);
  //  vector<ListGraph::Node> nodes (all.size());
  //get values
  Matrix2DVal x_vals (env, emh_model.all.size());
  for (const pair<int, const Vertex *>& p : emh_model.all) 
    x_vals[p.first] = IloNumArray (env, emh_model.all.size());
  //get subcycles
  //gomory hu
  ListGraph graph;
  ListGraph::EdgeMap<double> weight(graph); 
  //creating nodes
  for (const pair<int, Vertex>& p : emh_model.all) 
    nodes[p.first] = graph.addNode();
  //creating edges
  for (const pair<int, Vertex>& p : emh_model.all) {
    i = p.first;
    for (const pair<int, Vertex>& p1 : emh_model.all) {
      j = p1.first;
      weight[graph.addEdge(nodes[i], nodes[j])] = x_vals[k][i][j];
    }
  }
  GomoryHu<ListGraph, ListGraph::EdgeMap<double> > gh (graph, weight);
  gh.run();
  //get subcycles
  for (int i : emh_model.customers) {
    if (gh.minCutValue(nodes[i], nodes[0]) < 2.0) {

    }
  }
  queue<int> q;
  //bfs to get components connected to a customer 
  for (const Vertex& customer : emh_model.instance.customers) {
    set<int> component, customersComponent;
    //checking for neighboring
    bool hasNeighboring = false;
    for (const pair<int, const Vertex *>& p : emh_model.all)
      if (x_vals[customer.id][p.first] > EPS){
        hasNeighboring = true;
        break; 
      }
    //checking if bfs is needed
    if (!hasNeighboring)
      continue;
    q.push(customer.id);
    while (!q.empty()) {
      int curr = q.front();
      q.pop();
      component.insert(curr);
      //if it is a customer
      if (emh_model.customers.count(curr))
        customersComponent.insert(curr);
      for (const pair<int, const Vertex *>& p : emh_model.all)
        if (x_vals[curr][p.first] > EPS){
          x_vals[curr][p.first] = 0.0;
          q.push(p.first);
        }
    }
    for (int customer_ : customersComponent) {
      //getting lhs
      for (const pair<int, const Vertex *>& p2 : emh_model.all) {
        int a = p2.first;
        if (!component.count(a))
          for (int b : component) 
            lhs += emh_model.x[a][b];
      }
      //getting rhs
      for (int b : component)
        lhs -= emh_model.x[b][customer_];
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
  //clean
  for (const pair<int, const Vertex *>& p : emh_model.all){
    int i = p.first;
    x_vals[i].end();
  }
  x_vals.end();
  */
}
