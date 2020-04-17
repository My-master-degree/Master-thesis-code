#include "models/dsu.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/instance_generation_model/user_constraint.hpp"
#include "models/instance_generation_model/subcycle_user_constraint.hpp"
#include "models/instance_generation_model/instance_generation_model.hpp"

#include <lemon/gomory_hu.h>
#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>
#include <map>
#include <set>
#include <list>
#include <queue>
#include <ilcplex/ilocplex.h>

using namespace models;
using namespace models::cplex;
using namespace models::instance_generation_model;
using namespace lemon;
using namespace lemon::concepts;
using namespace std;

Subcycle_user_constraint::Subcycle_user_constraint (Instance_generation_model& instance_generation_model_): User_constraint(instance_generation_model_) {}

IloCplex::CallbackI* Subcycle_user_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint (*this);
}

void Subcycle_user_constraint::main() {
  Depth const* const d = (Depth *)getNodeData();
  IloInt depth = d ? d->depth : 0;
  if (depth > 0) {
    abortCutLoop();
    return;
  }
  IloEnv env = getEnv();
  IloExpr lhs(env);
  size_t i,
         j, 
         k,
         curr;
  map<unsigned int, ListGraph::Node> componentNodes;
  multimap<unsigned int, unsigned int> subcomponents;
  list<unsigned int> component;
  list<list<unsigned int>> components;
  ListGraph graph; queue<unsigned int> q;
  DSU dsu (instance_generation_model.sNodes);
  vector<bool> visited (instance_generation_model.sNodes, false);
  bool cond1, cond2;
  //get values
  try{
    instance_generation_model.x_vals = Matrix2DVal (env, instance_generation_model.sNodes);
    instance_generation_model.z_vals = IloNumArray (env, instance_generation_model.sNodes, 0, 1, IloNumVar::Float);
    getValues(instance_generation_model.z_vals, instance_generation_model.z);
    for (size_t i = 0; i < instance_generation_model.sNodes; i++) {
      instance_generation_model.x_vals[i] = IloNumArray (env, instance_generation_model.sNodes, 0, 1, IloNumVar::Float);
      getValues(instance_generation_model.x_vals[i], instance_generation_model.x[i]);
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
  //get components
  for (i = 0; i < instance_generation_model.sNodes; i++)
    //if i is a facility
    if (instance_generation_model.z_vals[i] > EPS && !visited[i]) {
      ListGraph::EdgeMap<double> weight(graph); 
      //bfs
      q.push(i);
      visited[curr] = true;
      componentNodes[i] = graph.addNode();
      while (!q.empty()) {
        curr = q.front();
        q.pop();
        //iterate over the neighboring
        for (j = 0; j < instance_generation_model.sNodes; j++) {
          cond1 = instance_generation_model.x_vals[curr][j] > EPS;
          cond2 = instance_generation_model.z_vals[j] > EPS && instance_generation_model.x_vals[j][curr] > EPS;
          if (cond1 || cond2) {
            if (!componentNodes.count(j))
              componentNodes[j] = graph.addNode();
            if (cond1)
              weight[graph.addEdge(componentNodes[curr], componentNodes[j])] = instance_generation_model.x_vals[curr][j];
            if (cond2)
              weight[graph.addEdge(componentNodes[j], componentNodes[curr])] = instance_generation_model.x_vals[j][curr];
            if (!visited[j])
              q.push(j);
            visited[j] = true;
          }
        }
      } 
      //gomory hu
      GomoryHu<ListGraph, ListGraph::EdgeMap<double> > gh (graph, weight);
      gh.run();
      //get subcomponents
      for (const pair<unsigned int, ListGraph::Node>& p : componentNodes) {
        j = p.first;
        if (dsu.pred[j] == j) 
          for (const pair<unsigned int, ListGraph::Node>& p1 : componentNodes) {
            k = p1.first;
//             if (gh.minCutValue(p.second, p1.second) >= instance_generation_model.z_vals[k])
            if (gh.minCutValue(p.second, p1.second) >= max(instance_generation_model.z_vals[k], instance_generation_model.z_vals[j]) - EPS) 
              dsu.join(j, k);
          }
      }
        //clean z_vals
      for (const pair<unsigned int, ListGraph::Node>& p : componentNodes) 
        subcomponents.insert(make_pair(dsu.findSet(p.first), p.first));
      //store components
      multimap<unsigned int, unsigned int>::iterator it = subcomponents.begin();
      j = it->first;
      for (; it != subcomponents.end(); it++) {
        if (it->first != j) {
          j = it->first;
          components.push_back(component);
          component.clear();
        } 
        component.push_back(it->second);
      }
        //end of multimap 
      components.push_back(component);
      //reset vars
      component = list<unsigned int> ();
      graph.clear();
      subcomponents.clear();
      componentNodes.clear();
      dsu.clean();
    }
  //inequallitites
  for (const list<unsigned int>& S : components) 
    for (const list<unsigned int>& T : components) 
      if (T != S){ 
        //for ech component facility        
        //\sum_{v_j \in T} \sum_{v_k \in S} y_{jk} + y_{kj} \geqslant z_{k*}, \forall k* \in S
        for (unsigned int j_ : T) {
          for (unsigned int i : S) 
            for (unsigned int j: T) 
              lhs += instance_generation_model.x[i][j] + instance_generation_model.x[j][i];
          lhs -= instance_generation_model.z[j_];
          try {
            add(lhs >= 0).end();
          } catch(IloException& e) {
            cerr << "Exception while adding user constraint" << e.getMessage() << "\n";
            throw;
          }
          lhs.end();
          lhs = IloExpr(env);
        }
      }
  //end vals
  for (i = 0; i < instance_generation_model.sNodes; i++)
    instance_generation_model.x_vals[i].end();
  instance_generation_model.x_vals.end();
  instance_generation_model.z_vals.end();
}
