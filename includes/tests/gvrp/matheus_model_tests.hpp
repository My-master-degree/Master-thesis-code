#ifndef MATHEUS_MODEL_TESTS_HPP_
#define MATHEUS_MODEL_TESTS_HPP_

#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"

#include <string>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model;

namespace tests {
  namespace gvrp {
    class Matheus_model_tests {
      public:
        explicit Matheus_model_tests (bool VERBOSE, unsigned int execution_time, unsigned int nIntSol);
        void run ();
      private:
        bool VERBOSE;
        unsigned int execution_time;
        unsigned int nIntSol;
        void execute_model(Matheus_model& matheus_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo);
        void openResultFile (ofstream& file, string fileName);
        void closeResultFile (ofstream& file);
    };
  }
}

#endif
