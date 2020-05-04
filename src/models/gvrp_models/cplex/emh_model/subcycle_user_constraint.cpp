#include "models/vertex.hpp"
#include "models/dsu.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/emh_model/subcycle_user_constraint.hpp"

#include <set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>
#include <lemon/gomory_hu.h>
#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>
#include <iostream>

using namespace std;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models::cplex::emh_model;
using namespace lemon;
using namespace lemon::concepts;

Subcycle_user_constraint::Subcycle_user_constraint (EMH_model& emh_model_) : User_constraint (emh_model_) {}

IloCplex::CallbackI* Subcycle_user_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint (*this);
}

void Subcycle_user_constraint::main() {
  //node callback
  Depth const* const d = (Depth *) getNodeData();
  IloInt depth = d ? d->depth : 0;
  if (depth > 0) {
    abortCutLoop();
    return;
  }
  //setup
  DSU dsu (emh_model.all.size());
  IloEnv env = getEnv();
  IloExpr lhs(env);
  vector<ListGraph::Node> nodes (emh_model.all.size());
  ListGraph graph;
  multimap<int, int> subcomponents;
  set<int> component;
  list<set<int>> components;
  queue<int> q;
  //get values
  Matrix2DVal x_vals (env, emh_model.all.size());
  for (const pair<int, const Vertex *>& p : emh_model.all) {
    x_vals[p.first] = IloNumArray (env, emh_model.all.size(), 0, 1, IloNumVar::Float);
    getValues(x_vals[p.first], emh_model.x[p.first]);
  }
  //creating nodes
  for (const pair<int, const Vertex *>& p : emh_model.all) 
    nodes[p.first] = graph.addNode();
  //get components
  set<int> visited;
  for (int customer : emh_model.customers) {
    if (visited.count(customer))
      continue;
    ListGraph::EdgeMap<double> weight(graph); 
    //bfs
    component.insert(customer);
    q.push(customer);
    while (!q.empty()) {
      int curr = q.front();
      visited.insert(curr);
      q.pop();
      for (const pair<int, const Vertex *>& p : emh_model.all) 
        if (x_vals[curr][p.first] > EPS || x_vals[p.first][curr] > EPS) {
          if (!component.count(p.first)) {
            component.insert(p.first);
            q.push(p.first);
          }
          weight[graph.addEdge(nodes[curr], nodes[p.first])] = x_vals[curr][p.first] > EPS ? x_vals[curr][p.first] : 0;
          weight[graph.addEdge(nodes[p.first], nodes[curr])] = x_vals[p.first][curr] > EPS ? x_vals[p.first][curr] : 0;
          x_vals[p.first][curr] = 0;
          x_vals[curr][p.first] = 0;
        } 
    }
    //gh
    GomoryHu<ListGraph, ListGraph::EdgeMap<double> > gh (graph, weight);
    gh.run();
    //get subcycles
    for (int i : component) 
      for (int j : component) 
        if (gh.minCutValue(nodes[j], nodes[j]) >= 2.0) 
          dsu.join(i, j);
    component.clear();
  }
  //get subcomponents
  for (const pair<int, const Vertex *>& p : emh_model.all) 
    subcomponents.insert(make_pair(dsu.findSet(p.first), p.first));
  //store components
  multimap<int, int>::iterator it = subcomponents.begin();
  int j = it->first;
  for (; it != subcomponents.end(); it++) {
    if (it->first != j) {
      j = it->first;
      //check if component has at leat a customer
      for (int customer : emh_model.customers)
        if (component.count(customer)) {
          components.push_back(component);
          break;
        }
      component.clear();
    } 
    component.insert(it->second);
  }
  //check if component has at leat a customer
  for (int customer : emh_model.customers)
    if (component.count(customer)) {
      components.push_back(component);
      break;
    }
  //end of multimap 
  //inequallitites
  for (const set<int>& S : components) 
    if (!S.count(emh_model.instance.depot.id)) {
      //\sum_{v_i \in V'\S} \sum_{v_j \in S} x_{ij} \geqslant 1 
      //lhs
      for (const pair<int, const Vertex *>& p : emh_model.all) 
        if (!S.count(p.first))
          for (int j : S) 
            lhs += emh_model.x[p.first][j];
      lhs -= 1;
      try {
        add(lhs >= 0).end();
      } catch(IloException& e) {
        cerr << "Exception while adding user constraint" << e.getMessage() << "\n";
        throw;
      }
      lhs.end();
      lhs = IloExpr(env);
    }
  //clean
  for (const pair<int, const Vertex *>& p : emh_model.all){
    int i = p.first;
    x_vals[i].end();
  }
  x_vals.end();
}
