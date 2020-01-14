#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"
#include "utils/cplex/routes_lb_constraint.hpp"
#include "models/distances_enum.hpp"

#include <ilcplex/ilocplex.h>

using namespace utils::cplex;

Routes_lb_constraint::Routes_lb_constraint (Compact_model& compact_model) : Extra_constraint_compact_model (compact_model) {}

void Routes_lb_constraint::add() {
//  IloExpr lhs;
// IloConstraint c;
// int bfsLb = 0;
// double componentTime = 0;
// //bfs
// if (compact_model.gvrp_instance.distance_enum == METRIC || compact_model.gvrp_instance.distance_enum == SYMMETRIC) {
//   set<int> visited;
//   queue<int> q;
//   //for each customer
//   for (Vertex customer : compact_model.gvrp_instance.customers) {
//     if (!visited.count(customer.id)) {
//       //bfs itself
//       q.push(customer.id);
//       visited.insert (customer.id);
//       while (!q.empty()) {
//         int curr = q.front();
//         q.pop();
//         //count serivce time
//         componentTime += compact_model.all[curr].serviceTime;
//         //get neighboring
//         for (pair<int, Vertex> p : compact_model.all) {
//           if (curr)
//         }
//       }
//     }
//   }
// }
  //time sum
}
