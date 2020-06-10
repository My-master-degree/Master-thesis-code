#include "models/mlsa_models/cplex/flow_model.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/cplex_model.hpp"

#include <set> 
#include <sstream> 
#include <float.h> 

using namespace models::mlsa_models::cplex;
using namespace models::gvrp_models;
using namespace std;

Flow_model::Flow_model (const Vrp_instance& vrp_instance, double timeLimit, double vehicleFuelCapacity_) : Cplex_model (vrp_instance, timeLimit), vehicleFuelCapacity (vehicleFuelCapacity_), n(vrp_instance.customers.size() + 1) {
}

pair<Gvrp_instance, Mip_solution_info> Flow_model::run () {
  //setup
  stringstream output_exception;
  Mip_solution_info mipSolInfo;
  try {
//    cout<<"Creating variables"<<endl;
    createVariables();
//    cout<<"Creating objective function"<<endl;
    createObjectiveFunction();
//    cout<<"Creating model"<<endl;
    createModel();
//    cout<<"Setting parameters"<<endl;
    setCustomParameters();
//    cout<<"Solving model"<<endl;
    if ( !cplex.solve() ) {
//      env.error() << "Failed to optimize LP." << endl;
      mipSolInfo = Mip_solution_info(-1, cplex.getStatus(), -1, -1);
      endVars();
      env.end();
      throw mipSolInfo;
    }
//    cplex.exportModel("cplexcpp.lp");
 //   env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    cout<<"Getting values"<<endl;
    fillVals();
    //cout<<"Creating VRP instance"<<endl;
    createGvrp_instance();
    mipSolInfo = Mip_solution_info(cplex.getMIPRelativeGap(), cplex.getStatus(), cplex.getTime(), cplex.getObjValue());
    endVars();
    env.end();
    return make_pair(*solution, mipSolInfo);
  } catch (IloException& e) {
    output_exception<<"Concert exception caught: " << e<<endl;
    throw output_exception.str();
  } catch (string s) {
    throw s;
  }
}

void Flow_model::createVariables(){
  try {
    stringstream nameStream;
    x = Matrix2DVar (env, n);
    double floatVarUB = 2 * (n - 1);
    //s var
    s = IloNumVar (env, 0, n - 1, IloNumVar::Int);
    nameStream<<"s";
    s.setName(nameStream.str().c_str());
    nameStream.clear();
    nameStream.str("");
    //x var
    for (size_t i = 0; i < n; i++){
      x[i] = IloNumVarArray(env, n - 1, 0, floatVarUB, IloNumVar::Float);
      //setting names
      for (size_t j = 0; j < n - 1; j++) {
        nameStream<<"x["<<i<<"]["<<j<<"]";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
      }
      //terminal int var
      x[i].add(IloNumVar (env, 0, 1, IloNumVar::Int));
      nameStream<<"x["<<i<<"]["<<n - 1<<"]";
      x[i][n - 1].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
    }
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating variables");
  }
}

void Flow_model::createObjectiveFunction() {
//objective function
  try{
    IloExpr fo (env);
    for (size_t i = 1; i < n; i++) 
      fo +=  x[i][n - 1];
    model = IloModel (env);
    model.add(IloMaximize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void Flow_model::createModel() {
  try {
    //constraints
    IloExpr expr(env);    
    IloConstraint c;
    stringstream constraintName;
    //s
    for (size_t i = 1; i < n; i++) 
      expr += x[i][n - 1];
    c = IloConstraint (s == expr);
    c.setName("s definition");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //x_{rt} = 0
    c = IloConstraint (x[0][n - 1] == 0);
    c.setName("root can not be a leaf");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //\sum_{v_j \in V\backslash{v_t}} x_{rj} = n - 1 + s
    for (size_t i = 0; i < n - 1; i++) 
      expr += x[0][i];
    c = IloConstraint (expr == n - 1 + s);
    c.setName("amount of flow from the root");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //\sum_{v_j \in V} x_{ji} - \sum_{v_k \in V \backslash \{v_r\}} x_{ik} = 1, \forall v_i \in V' \backslash \{v_r, v_t\}
    for (size_t i = 1; i < n; i++) {
      for (size_t j = 0; j < n; j++) 
        expr += x[j][i] - x[i][j];
      constraintName<<"decrease the incoming amount of flow by 1 in "<<i;
      c = IloConstraint (expr == 1);
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in V \backslash \{v_r, v_t\}} f_{ij} + 2 * (n - 2) * f_{it} \leqslant 2 * (n - 2), \forall v_i \in V' \backslash \{v_r, v_t\} 
    for (size_t i = 1; i < n; i++) {
      for (size_t j = 0; j < n - 1; j++)
        expr += x[i][j];
      expr += 2 * (int(n) - 2) * x[i][n - 1];
      constraintName<<"if "<<i<<" is a sink";
      c = IloConstraint (expr <= 2 * (int(n) - 2));
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in V : e_{ij} \leqslant \beta/2 && e_{jr} \leqslant \beta} 1 - x_{jt} \geqslant x_{it}, \forall v_i \in V' \backslash \{v_r, v_t\} : e_{ir} > \beta/2
    for (size_t i = 1; i < n; ++i) {
      if (instance.distances[0][i] > vehicleFuelCapacity/2.0) {
        for (size_t j = 0; j < n - 1; ++j) 
          if (instance.distances[i][j + 1] <= vehicleFuelCapacity/2.0 && instance.distances[j + 1][0] <= vehicleFuelCapacity) 
            expr += 1 - x[j + 1][n - 1];
        constraintName<<"if "<<i<<" is a sink then must exists a branch node respecting the EMH constraints";
        c = IloConstraint (expr >= x[i][n - 1]);
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //preprocessings
    //removing edges with energy greater than \beta
    for (size_t i = 0; i < n; ++i)
      for (size_t j = 0; j < n - 1; ++j)
        if (instance.distances[i][j + 1] > vehicleFuelCapacity) {
          constraintName<<"edge("<<i<<", "<<j + 1<<") is an invalid edge";
          c = IloConstraint (x[i][j] == 0);
          c.setName(constraintName.str().c_str());
          model.add(c);
          expr.end();
          expr = IloExpr(env);
          constraintName.clear();
          constraintName.str("");
        }
    //init
    cplex = IloCplex(model);
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

void Flow_model::setCustomParameters(){
  try{
    setParameters();
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void Flow_model::fillVals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, n);
    for (size_t i = 0; i < n; i++) {
      x_vals[i] = IloNumArray (env, n - 1, 0, 1, IloNumVar::Float);
      IloInt t;
      x_vals[i].add(t);
      cplex.getValues(x_vals[i], x[i]);
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void Flow_model::createGvrp_instance(){
  try{
    //setup
    list<Vertex> customers,
                 afss;
    set<int> customersSet;
    double longestTime = 0, 
           afsServiceTime, 
           customerServiceTime,
           routeTime;
    int nCustomers;
    //get nodes
    for (const Vertex& v : instance.customers)
      if (x_vals[v.id][n - 1] > 0) 
        customers.push_back(v);
      else
        afss.push_back(v);
    //cout<<afss.size()<<endl;
    //create instance
    solution = new Gvrp_instance(afss, customers, instance.depot, vehicleFuelCapacity, instance.distances, instance.distances_enum, customers.size(), DBL_MAX, 1, 1);
    //set average speed
    size_t sall = customers.size() + afss.size() + 1;
    for (size_t i = 0; i < sall; ++i)
      for (size_t j = 0; j < sall; ++j)
        solution->vehicleAverageSpeed = max(solution->vehicleAverageSpeed, instance.distances[i][j]);
    //customers service time
    customerServiceTime = solution->vehicleAverageSpeed / customers.size();
    //build set of customers
    for (Vertex& customer : solution->customers) {
      customersSet.insert(customer.id);
      customer.serviceTime = customerServiceTime;
    }
    //afss service time
    afsServiceTime = solution->vehicleAverageSpeed / afss.size();
    for (Vertex& afs : solution->afss) 
      afs.serviceTime = afsServiceTime;
    //set time limit
    Gvrp_feasible_solution_heuristic gvrp_feasible_solution_heuristic (*solution);
    Gvrp_solution gvrp_solution = gvrp_feasible_solution_heuristic.run();
    //cout<<gvrp_solution<<endl;
    for (const list<Vertex>& route : gvrp_solution.routes) {
      routeTime = 0;
      nCustomers = 0;
      for (auto curr = route.begin(); curr != --route.end(); ++curr) {
        routeTime += solution->time(curr->id, next(curr)->id);
        if (customersSet.count(curr->id))
          ++nCustomers;
      }
      routeTime += nCustomers * customerServiceTime + (route.size() - nCustomers - 2) * afsServiceTime;
      longestTime = max(longestTime, routeTime);
    }
    solution->timeLimit = 2 * longestTime; 
    //cout<<*solution<<endl;
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void Flow_model::endVars(){
  for (size_t i = 0; i < n; i++)
    x[i].end();
  x.end();
}

