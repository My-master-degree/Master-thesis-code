#include "models/gvrp_instance.hpp"
#include "models/gvrp_solution.hpp"
#include "models/mip_solution_info.hpp"
#include "utils/util.hpp"
#include "utils/cplex/compact_model.hpp"
#include "SampleConfig.h"


#include <string>
#include <set>
#include <queue>
#include <map>

using namespace std;
using namespace models;
using namespace utils;
using namespace utils::cplex;

int main (int argc, char **argv)
{ 

  string instances[] = {
    "S1_20c3sU10.txt",
    "S1_20c3sU1.txt",
    "S1_20c3sU2.txt",
    "S1_20c3sU3.txt",
    "S1_20c3sU4.txt",
    "S1_20c3sU5.txt",
    "S1_20c3sU6.txt",
    "S1_20c3sU7.txt",
    "S1_20c3sU8.txt",
    "S1_20c3sU9.txt",
    "S2_20c3sC10.txt",
    "S2_20c3sC1.txt",
    "S2_20c3sC2.txt",
    "S2_20c3sC3.txt",
    "S2_20c3sC4.txt",
    "S2_20c3sC5.txt",
    "S2_20c3sC6.txt",
    "S2_20c3sC7.txt",
    "S2_20c3sC8.txt",
    "S2_20c3sC9.txt",
    "S3_S1_10i6s.txt",
    "S3_S1_2i6s.txt",
    "S3_S1_4i6s.txt",
    "S3_S1_6i6s.txt",
    "S3_S1_8i6s.txt",
    "S3_S2_10i6s.txt", 
    "S3_S2_2i6s.txt",
    "S3_S2_4i6s.txt",
    "S3_S2_6i6s.txt",
    "S3_S2_8i6s.txt",
    "S4_S1_4i10s.txt",
    "S4_S1_4i2s.txt",
    "S4_S1_4i4s.txt",
    "S4_S1_4i6s.txt",
    "S4_S1_4i8s.txt",
    "S4_S2_4i10s.txt",
    "S4_S2_4i2s.txt",
    "S4_S2_4i4s.txt",
    "S4_S2_4i6s.txt",
    "S4_S2_4i8s.txt",
    "Large_VA_Input_111c_21s.txt",
    "Large_VA_Input_111c_22s.txt",
    "Large_VA_Input_111c_24s.txt",
    "Large_VA_Input_111c_26s.txt",
    "Large_VA_Input_111c_28s.txt",
    "Large_VA_Input_200c_21s.txt",
    "Large_VA_Input_300c_21s.txt",
    "Large_VA_Input_350c_21s.txt",
    "Large_VA_Input_400c_21s.txt",
    "Large_VA_Input_450c_21s.txt",
    "Large_VA_Input_500c_21s.txt",
  };



  for (auto instance : instances){
//    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_PATH + string("instances/") + instance);
//  create new instance
//   int numCustomersAtDepot = 2,
//     numCustomersAtAfs = 2;
// set<int> firstLevelAfss;
// set<int> usedCustomers;
// set<int> usedAfss;
// //bfs for the first level
// int curr = gvrp_instance.depot.id;
// for (auto afs: gvrp_instance.afss)
//   //if afs is a valid afs
//   if (gvrp_instance.distances[curr][afs.id] * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity)
//     firstLevelAfss.insert(afs.id);          
// //getting customers closer to depot
// int i = 0;
// for (auto it = gvrp_instance.customers.begin(); it != gvrp_instance.customers.end() && i < numCustomersAtDepot ; ++it){
//   Vertex* customer = &(*it);
//   if (gvrp_instance.distances[customer->id][gvrp_instance.depot.id] * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity / 2.0){
//     usedCustomers.insert(customer->id);
//     i++;
//   }
// }
// //getting customers closer to afss
// i = 0;
// for (auto it = gvrp_instance.customers.begin(); it != gvrp_instance.customers.end() && i < numCustomersAtAfs; ++it){
//   Vertex* customer = &(*it);
//   for (auto afs : firstLevelAfss){      
//     if (gvrp_instance.distances[customer->id][afs] * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity / 2.0 && gvrp_instance.distances[customer->id][gvrp_instance.depot.id] * gvrp_instance.vehicleFuelConsumptionRate > gvrp_instance.vehicleFuelCapacity / 2.0){
//       usedCustomers.insert(customer->id);
//       usedAfss.insert(afs);
//       i++;
//       break;
//     }
//   }
// }
// //checking
// if (int(usedCustomers.size()) < numCustomersAtDepot + numCustomersAtAfs)
//   continue;
// //removing customers
// for (auto it = gvrp_instance.customers.begin(); it != gvrp_instance.customers.end();){
//   if (!usedCustomers.count(it->id))
//     it = gvrp_instance.customers.erase(it);
//   else
//     it++;
// }
// //removing afss 
// for (auto it = gvrp_instance.afss.begin(); it != gvrp_instance.afss.end();){
//   if (!usedAfss.count(it->id))
//     it = gvrp_instance.afss.erase(it);
//   else
//     it++;
// }
// //print distances
// set<int> all = usedCustomers;
// all.insert(usedAfss.begin(), usedAfss.end());
// all.insert(gvrp_instance.depot.id);
// for (auto id : all)
//   cout<<"  "<<id<<" ";
// cout<<endl;
// for (auto id : all){
//   cout<<id;
//   for (auto id1 : all){
//     double fuel = gvrp_instance.distances[id][id1] * gvrp_instance.vehicleFuelConsumptionRate;
//     fuel = int(fuel * 100.0)/100.0;
//     cout<<" "<<fuel;
//   }
//   cout<<endl;
// }
// //update distance matrix
// for (int j = gvrp_instance.distances.size() - 1; j >= 0; j--)
//   if (j != gvrp_instance.depot.id && !usedAfss.count(j) && !usedCustomers.count(j))
//     gvrp_instance.distances.erase(gvrp_instance.distances.begin() + j);    
//   else
//     for (int k = gvrp_instance.distances[j].size() - 1; k >= 0; k--)
//       if (k != gvrp_instance.depot.id && !usedAfss.count(k) && !usedCustomers.count(k))
//         gvrp_instance.distances[j].erase(gvrp_instance.distances[j].begin() + k);
// //print distances
// for (int i = 0; i < gvrp_instance.distances.size(); i++)
//   cout<<"  "<<i<<" ";
// cout<<endl;
// for (int i = 0; i < gvrp_instance.distances.size(); i++){
//   cout<<i;
//   for (int j = 0; j < gvrp_instance.distances[i].size(); j++){
//     double fuel = gvrp_instance.distances[i][j] * gvrp_instance.vehicleFuelConsumptionRate;
//     fuel = int(fuel * 100.0)/100.0;
//     cout<<" "<<fuel;
//   }
//   cout<<endl;
// }
// //update ids
// int id = 1;
// for (auto it = gvrp_instance.afss.begin(); it != gvrp_instance.afss.end(); it++){
//   it->id = id++; 
// }
// for (auto it = gvrp_instance.customers.begin(); it != gvrp_instance.customers.end(); it++){
//   it->id = id++; 
// }
    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_PATH + string("instances/") + instance);
    unsigned int time_limit = 360;
    cout<<gvrp_instance<<endl;
    Compact_model compact_model(gvrp_instance, time_limit, 1);
    try{
    pair<Gvrp_solution, Mip_solution_info > sol = compact_model.run();    
    cout<<sol.first;
    }catch (const char * s){
      cout<<s;
    }
    break;
  }
  return 0;
}
