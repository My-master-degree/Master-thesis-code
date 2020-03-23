#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/extra_constraint_cubic_model.hpp"
#include "models/cubic_model/routes_lb_constraint.hpp"
#include "models/distances_enum.hpp"

#include <ilcplex/ilocplex.h>

using namespace models::cubic_model;

Routes_lb_constraint::Routes_lb_constraint (Cubic_model& cubic_model) : Extra_constraint_cubic_model (cubic_model) {}

void Routes_lb_constraint::add() {
//  IloExpr lhs;
// IloConstraint c;
// int bfsLb = 0;
// double componentTime = 0;
// //bfs
// if (cubic_model.instance.distance_enum == METRIC || cubic_model.instance.distance_enum == SYMMETRIC) {
//   set<int> visited;
//   queue<int> q;
//   //for each customer
//   for (Vertex customer : cubic_model.instance.customers) {
//     if (!visited.count(customer.id)) {
//       //bfs itself
//       q.push(customer.id);
//       visited.insert (customer.id);
//       while (!q.empty()) {
//         int curr = q.front();
//         q.pop();
//         //count serivce time
//         componentTime += cubic_model.all[curr].serviceTime;
//         //get neighboring
//         for (pair<int, Vertex> p : cubic_model.all) {
//           if (curr)
//         }
//       }
//     }
//   }
// }
  //time sum
}
