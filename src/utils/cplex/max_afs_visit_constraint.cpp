#include "utils/cplex/max_afs_visit_constraint.hpp"
#include "utils/cplex/extra_constraint_compact_model.hpp"

#include <ilcplex/ilocplex.h>

using namespace utils::cplex;

Max_afs_visit_constraint::Max_afs_visit_constraint (Compact_model& compact_model) : Extra_constraint_compact_model (compact_model){
}

void Max_afs_visit_constraint::add () {
  IloExpr lhs (compact_model.env), 
          rhs (compact_model.env);
  IloConstraint c;
  for (Vertex afs : compact_model.gvrp_instance.afss)
    for (unsigned int k = 0; k < compact_model.gvrp_instance.customers.size(); k++) {
      //number of visits to the AFS
      for (pair<int, Vertex> p : compact_model.all)
        lhs += compact_model.x[k][afs.id][p.first];
      //number of visited customers by route k
      for (Vertex customer : compact_model.gvrp_instance.customers)
        for (pair<int, Vertex> p : compact_model.all)
          rhs += compact_model.x[k][customer.id][p.first];
      c = IloConstraint (lhs <= rhs + 1);
      c.setName("num_max_visit_afs");
      //add constraint
      compact_model.model.add(c);
      //clean
      lhs.end();
      rhs.end();
      lhs = IloExpr(compact_model.env);
      rhs = IloExpr(compact_model.env);
    }
}
