#include "utils/cplex/lazy_constraint_compact_model.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/improved_subcycle_lazy_constraint_compact_model.hpp"

#include <set>
#include <queue>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils::cplex;

Improved_subcycle_lazy_constraint_compact_model::Improved_subcycle_lazy_constraint_compact_model (Compact_model& compact_model_) : Lazy_constraint_compact_model (compact_model_) {}

IloCplex::CallbackI* Improved_subcycle_lazy_constraint_compact_model::duplicateCallback() const {
  return new(getEnv()) Improved_subcycle_lazy_constraint_compact_model (*this);
}

void Improved_subcycle_lazy_constraint_compact_model::main() {
  int depot = compact_model.gvrp_instance.depot.id;
  IloEnv env = getEnv();
  IloExpr lhs(env), lhs_(env);
  //get values
  Matrix3DVal x_vals (env, compact_model.gvrp_instance.customers.size());
  for (unsigned int k = 0; k < compact_model.gvrp_instance.customers.size(); k++) {
    x_vals[k] = IloArray<IloNumArray> (env, compact_model.all.size());
    for (pair<int, Vertex> p : compact_model.all) {
      int i = p.first;
      x_vals[k][i] = IloNumArray (env, compact_model.all.size(), 0, compact_model.ub_edge_visit, IloNumVar::Int);
      getValues(x_vals[k][i], compact_model.x[k][i]);
    }
  }
  //get subcycles
  for (int k = 0; k < int(compact_model.gvrp_instance.customers.size()); k++) {
//    cout<<"=====Route "<<k<<endl;
    //bfs to remove all edges related to the depot
    queue<int> q;
//    cout<<"Depot: ";
    q.push(depot);
    while (!q.empty()) {
      int curr = q.front();
      q.pop();
//      cout<<curr<<" ";
      for (pair<int, Vertex> p : compact_model.all)
        if (x_vals[k][curr][p.first] > 0){
          x_vals[k][curr][p.first]--;
          q.push(p.first);
        }        
    }
//    cout<<endl;
    for (pair<int, Vertex> p : compact_model.all) {
      int i = p.first;
      for (pair<int, Vertex> p1 : compact_model.all) {
        int j = p1.first;
        //an edge found
        if (x_vals[k][i][j] > 0) {
          x_vals[k][i][j]--;
          //dfs
          set<int> vertexes;
          vertexes.insert(i);
          int curr = i;
          int next = j;
//         list<int> seq;
//          seq.push_back(curr);
          while (next != i) {
            curr = next;
            vertexes.insert(next);
//            seq.push_back(next);
            for (pair<int, Vertex> p2 : compact_model.all)
              if (x_vals[k][curr][p2.first] > 0) {
                next = p2.first;
                x_vals[k][curr][next]--;
                break;
              }                             
          }
//          seq.push_back(next);
          //subcycle found
          if (!vertexes.count(depot)){
//            cout<<"Seq: ";
//            for (auto v : seq)
//              cout<<v<<" ";
//            cout<<endl;
//            cout<<"Existing cutting edges: ";
//           for (int b : vertexes){ 
//             for (pair<int, Vertex> p2 : compact_model.all){
//               int a = p2.first;
//               if (!vertexes.count(a) && (x_vals[k][a][b] > 0 || x_vals[k][b][a] > 0)){
//                 cout<<"("<<b<<", "<<a<<") ";
//               }
//             }       
//             cout<<endl;
//           }
//            cout<<"New cutting edges: ";
            for (pair<int, Vertex> p2 : compact_model.all){
              int a = p2.first;
              for (int b : vertexes){ 
                if (!vertexes.count(a)){
                  lhs += compact_model.x[k][a][b];
                  //                 cout<<"("<<b<<", "<<a<<") ";
                }       
                //              cout<<endl;
              }
            }
            //getting rhs
            for (auto customer : compact_model.gvrp_instance.customers)
              if (vertexes.count(customer.id)) {
                lhs_ = lhs;
                for (int b : vertexes)
                  lhs_ -= compact_model.x[k][b][customer.id];
                //                cout<<lhs<<endl;
                try {
                  add(lhs_ >= 0.0).end();
                } catch(IloException& e) {
                  cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
                  throw;
                }
                lhs_.end();
                lhs_ = IloExpr(env);
              } 
            lhs.end();
            lhs = IloExpr(env);
          }
        }
      }
    }
    for (pair<int, Vertex> p : compact_model.all){
      int i = p.first;
      x_vals[k][i].end();
    }
  }
  x_vals.end();

}