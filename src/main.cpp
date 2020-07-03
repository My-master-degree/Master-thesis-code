#include "tests/gvrp/cubic_model_tests.hpp"
#include "tests/gvrp/emh_model_tests.hpp"
#include "tests/gvrp/kk_model_tests.hpp"
#include "tests/gvrp/lh_model_tests.hpp"
#include "tests/gvrp/matheus_model_tests.hpp"
#include "tests/gvrp/matheus_model_2_tests.hpp"
#include "tests/gvrp/local_searchs_tests.hpp"
#include "tests/mlsa/model_tests.hpp"
#include "tests/bpp/bpp_model_tests.hpp"

#include <string>

using namespace std;
using namespace tests::bpp;
using namespace tests::gvrp;
using namespace tests::mlsa_flow;

int main (int argc, char **argv)
{ 
  unsigned int execution_time = 120,
               nIntSol = -1;
  bool VERBOSE = false;  
  //getting params
  for (int i = 0; i < argc; i++)
    if (strcmp(argv[i], "-time") == 0)
      execution_time = stoi(argv[++i]);
    else if (strcmp(argv[i], "-verbose") == 0) {
      if (strcmp(argv[++i], "true") == 0 ||  strcmp(argv[i], "1") == 0) {
        VERBOSE = true; 
      }
    } else if (strcmp(argv[i], "-nIntSol") == 0) {
      nIntSol = stoi(argv[++i]);
      if (nIntSol <= 0)
        nIntSol = 2100000000;
    }
  //experiments
  /*
  //cubic model
  Cubic_model_tests cubic_model_tests (VERBOSE, execution_time, nIntSol);
  cubic_model_tests.run();
  */
    //instance generation model
  Model_tests model_tests (VERBOSE, execution_time, nIntSol);
  model_tests.run();
  /*
    //local searchs
  Local_searchs_tests local_searchs_tests;
  local_searchs_tests.run();
    //EMH model
  EMH_model_tests emh_model_tests (VERBOSE, execution_time, nIntSol);
  emh_model_tests.run();
    //KK model
  KK_model_tests kk_model_tests (VERBOSE, execution_time, nIntSol);
  kk_model_tests.run();
    //LH model
  LH_model_tests lh_model_tests (VERBOSE, execution_time, nIntSol);
  lh_model_tests.run();
    //Matheus model
  Matheus_model_tests matheus_model_tests (VERBOSE, execution_time, nIntSol);
  matheus_model_tests.run();
    //Matheus model 2
  Matheus_model_2_tests matheus_model_2_tests (VERBOSE, execution_time, nIntSol);
  matheus_model_2_tests.run();
    //BBP model
  BPP_model_tests bpp_model_tests (VERBOSE, execution_time, nIntSol);
  bpp_model_tests.run();
  */


  return 0;
}
