#ifndef MATHEUS_MODEL_3_TESTS_HPP_
#define MATHEUS_MODEL_3_TESTS_HPP_

#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"

#include <string>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_3;

namespace tests {
  namespace gvrp {
    class Matheus_model_3_tests {
      public:
        explicit Matheus_model_3_tests (bool VERBOSE, unsigned int execution_time, unsigned int nIntSol);
        void run ();
      private:
        bool VERBOSE;
        unsigned int execution_time;
        unsigned int nIntSol;
        void execute_model(Matheus_model_3& matheus_model_3, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo);
        void openResultFile (ofstream& file, string fileName);
        void closeResultFile (ofstream& file);
    };
  }
}

#endif
