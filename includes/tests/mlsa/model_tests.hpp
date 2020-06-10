#ifndef _MLSA_FLOW_MODEL_TESTS_HPP_
#define _MLSA_FLOW_MODEL_TESTS_HPP_

#include "models/mlsa_models/cplex/flow_model.hpp"

#include <string>

using namespace std;
using namespace models::mlsa_models::cplex;

namespace tests {
  namespace mlsa_flow {
    class Model_tests {
      public:
        explicit Model_tests (bool VERBOSE, unsigned int execution_time, unsigned int nIntSol);
        void run ();
      private:
        bool VERBOSE;
        unsigned int execution_time;
        unsigned int nIntSol;
        void execute_model(Flow_model& flow_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo);
        void openResultFile (ofstream& file, string fileName);
        void closeResultFile (ofstream& file);
    };
  }
}

#endif
