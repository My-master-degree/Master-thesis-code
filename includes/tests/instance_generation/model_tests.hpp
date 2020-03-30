#ifndef INSTANCE_GENERATION_TESTS_HPP_
#define INSTANCE_GENERATION_TESTS_HPP_

#include "models/instance_generation_model/instance_generation_model.hpp"

#include <string>

using namespace std;
using namespace models::instance_generation_model;

namespace tests {
  namespace instance_generation {
    class Model_tests {
      public:
        explicit Model_tests (bool VERBOSE, unsigned int execution_time, unsigned int nIntSol);
        void run ();
      private:
        bool VERBOSE;
        unsigned int execution_time;
        unsigned int nIntSol;
        void execute_model(Instance_generation_model& instance_generation_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo);
        void openResultFile (ofstream& file, string fileName);
        void closeResultFile (ofstream& file);
    };
  }
}

#endif
