#include "models/cubic_model/lazy_constraint_cubic_model.hpp"
#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/subcycle_user_constraint_cubic_model.hpp"
#include "utils/dsu.hpp"

#include <set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>
#include <lemon/gomory_hu.h>
#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>

using namespace std;
using namespace models::cubic_model;
using namespace utils;
using namespace lemon;
using namespace lemon::concepts;

Subcycle_user_constraint_cubic_model::Subcycle_user_constraint_cubic_model (Cubic_model& cubic_model_) : User_constraint_cubic_model (cubic_model_) {}

IloCplex::CallbackI* Subcycle_user_constraint_cubic_model::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint_cubic_model (*this);
}

void Subcycle_user_constraint_cubic_model::main() {
  if (!cubic_model.ALLOW_SUBCYCLE_USER_CUT)
    return;
  int i;
  IloEnv env = getEnv();
  IloExpr lhs(env);
  bool infeasibilityFound = false;
  size_t k;
  queue<int> q;
  Matrix3DVal x_vals (env, cubic_model.gvrp_instance.customers.size());
  //get values
  for (k = 0; k < cubic_model.gvrp_instance.customers.size(); k++) {
    x_vals[k] = IloArray<IloNumArray> (env, cubic_model.all.size());
    for (const pair<int, Vertex>& p : cubic_model.all) {
      i = p.first;
      x_vals[k][i] = IloNumArray (env, cubic_model.all.size(), 0, cubic_model.ub_edge_visit, IloNumVar::Float);
      getValues(x_vals[k][i], cubic_model.x[k][i]);
    }
  }
  //get subcycles
  for (k = 0; k < cubic_model.gvrp_instance.customers.size(); k++) { 
    //setup
    ListGraph graph;
    ListGraph::EdgeMap<double>  weight(graph); 
    ListGraph::NodeMap<int> ids (graph);
    map<int, ListGraph::Node> nodes;
    ListGraph::Node node, next, pred;
    set<int> usedNodes,
              component,
              customers;
    map<int, list<int>> components;
    DSU dsu (cubic_model.gvrp_instance.customers.size() + cubic_model.gvrp_instance.afss.size() + 1);
    int curr;
    infeasibilityFound = false;
    //iterating over customers
    for (const Vertex& customer : cubic_model.gvrp_instance.customers) {
      if (!nodes.count(customer.id)) {
        //bfs
        node = graph.addNode();
        nodes[customer.id] = node; 
        ids[node] = customer.id;
        q.push(customer.id);
        while (!q.empty()) {
          curr = q.front();
          q.pop();
          node = nodes[curr];
          usedNodes.insert(curr);
          for (const pair<int, Vertex>& p : cubic_model.all)
            //if edge exist
            if (x_vals[k][curr][p.first] > EPS){
              //if node wasnt created
              auto it = nodes.find(p.first);
              if (it == nodes.end()) {
                next = graph.addNode();
                ids[next] = p.first;
                nodes[p.first] = next;
              } else 
                next = it->second;
              weight[graph.addEdge(node, next)] = x_vals[k][curr][p.first];
              x_vals[k][curr][p.first] = 0.0;
              q.push(p.first);
            }
        }
      }
    }
    //gomory hu
    GomoryHu<ListGraph, ListGraph::EdgeMap<double> > gh (graph, weight);
    gh.run();
    //dsu
    //get subcycles
    for (int id : usedNodes) {
      auto node = nodes[id];
      auto pred = gh.predNode(node);
      if (pred != INVALID && gh.minCutValue(node, pred) >= 2) 
        dsu.join(id, ids[pred]);
      else
        infeasibilityFound = true;
    }
    //iterating over the components, i.e., the cut < 2
    for (size_t t = 0; t < dsu.n; components[dsu.findSet(t)].push_back(t++));
    //write inequallities
    for (const pair<int, list<int>>& componentsEntry : components) {
      //getting component customers and nodes 
      for (int b : componentsEntry.second) {
        component.insert(b);
        if (cubic_model.customers.count(b))
          customers.insert(b);
      }
      //getting lhs
      for (const pair<int, Vertex>& p : cubic_model.all) {
        int a = p.first;
        if (!component.count(a)) {
          for (int b : component) 
            lhs += cubic_model.x[k][a][b];
        }
      }
      //getting rhs
      for (int a : customers)
        for (int b : component)
          lhs -= cubic_model.x[k][a][b];
      //add constraint
      try {
        add(lhs >= 0.0).end();
      } catch(IloException& e) {
        cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
        throw;
      }
      lhs.end();
      lhs = IloExpr(env);
      customers = set<int> ();
      component = set<int> ();
    } 
    for (pair<int, Vertex> p : cubic_model.all){
      int i = p.first;
      x_vals[k][i].end();
    }
  }
  x_vals.end();
  cubic_model.ALLOW_SUBCYCLE_USER_CUT = infeasibilityFound;
}
