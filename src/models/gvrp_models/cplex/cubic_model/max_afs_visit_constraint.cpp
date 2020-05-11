#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/max_afs_visit_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace models;
using namespace models::gvrp_models::cplex::cubic_model;

Max_afs_visit_constraint::Max_afs_visit_constraint (Cubic_model& cubic_model) : Extra_constraint (cubic_model){
}

void Max_afs_visit_constraint::add () {
  IloExpr lhs (cubic_model.env), 
          rhs (cubic_model.env);
  IloConstraint c;
  for (const Vertex& afs : cubic_model.instance.afss)
    for (int k = 0; k < cubic_model.instance.maxRoutes; k++) {
      //number of visits to the AFS
      for (const pair<int, const Vertex *>& p : cubic_model.all)
        lhs += cubic_model.x[k][afs.id][p.first];
      //number of visited customers by route k
      for (const Vertex& customer : cubic_model.instance.customers)
        for (const pair<int, const Vertex *>& p : cubic_model.all)
          rhs += cubic_model.x[k][customer.id][p.first];
      c = IloConstraint (lhs <= rhs + 1);
      c.setName("num_max_visit_afs");
      //add constraint
      cubic_model.model.add(c);
      //clean
      lhs.end();
      rhs.end();
      lhs = IloExpr(cubic_model.env);
      rhs = IloExpr(cubic_model.env);
    }
}
