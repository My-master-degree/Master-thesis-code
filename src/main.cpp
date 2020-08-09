#include "tests/gvrp/cubic_model_tests.hpp"
#include "tests/gvrp/emh_model_tests.hpp"
#include "tests/gvrp/kk_model_tests.hpp"
#include "tests/gvrp/lh_model_tests.hpp"
#include "tests/gvrp/matheus_model_tests.hpp"
#include "tests/gvrp/matheus_model_2_tests.hpp"
#include "tests/gvrp/matheus_model_3_tests.hpp"
#include "tests/gvrp/matheus_model_4_tests.hpp"
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
    //instance generation model
  Model_tests model_tests (VERBOSE, execution_time, nIntSol);
  model_tests.run();
    //local searchs
  Local_searchs_tests local_searchs_tests;
  local_searchs_tests.run();
    //EMH model
  cout<<"============    EMH MODEL    =============="<<endl;
  EMH_model_tests emh_model_tests (VERBOSE, execution_time, nIntSol);
  emh_model_tests.run();
    //KK model
  cout<<"============    KK MODEL    =============="<<endl;
  KK_model_tests kk_model_tests (VERBOSE, execution_time, nIntSol);
  kk_model_tests.run();
    //LH model
  cout<<"============    LH MODEL    =============="<<endl;
  LH_model_tests lh_model_tests (VERBOSE, execution_time, nIntSol);
  lh_model_tests.run();
    //Matheus model
  cout<<"============    MATHEUS MODEL    =============="<<endl;
  Matheus_model_tests matheus_model_tests (VERBOSE, execution_time, nIntSol);
  matheus_model_tests.run();
    //Matheus model 2
  cout<<"============    MATHEUS 2 MODEL    =============="<<endl;
  Matheus_model_2_tests matheus_model_2_tests (VERBOSE, execution_time, nIntSol);
  matheus_model_2_tests.run();
  //cubic model
  cout<<"============    CUBIC MODEL    =============="<<endl;
  Cubic_model_tests cubic_model_tests (VERBOSE, execution_time, nIntSol);
  cubic_model_tests.run();
    //Matheus model 3
  cout<<"============    MATHEUS 3 MODEL    =============="<<endl;
  Matheus_model_3_tests matheus_model_3_tests (VERBOSE, execution_time, nIntSol);
  matheus_model_3_tests.run();
  */
    //Matheus model 4 
  cout<<"============    MATHEUS 4 MODEL    =============="<<endl;
  Matheus_model_4_tests matheus_model_4_tests (VERBOSE, execution_time, nIntSol);
  matheus_model_4_tests.run();
  /*
    //BBP model
  BPP_model_tests bpp_model_tests (VERBOSE, execution_time, nIntSol);
  bpp_model_tests.run();
  */


  return 0;
}
