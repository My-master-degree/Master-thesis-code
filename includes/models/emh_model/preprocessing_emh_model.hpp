#ifndef PREPROCESSING_EMH_MODEL_HPP_
#define PREPROCESSING_EMH_MODEL_HPP_

namespace models {
  namespace emh_model {
    class EMH_model;
    class Preprocessing_emh_model {
      protected:
        EMH_model& emh_model;
      public:
        explicit Preprocessing_emh_model (EMH_model& emh_model);
        virtual void add () = 0;
    };
  }
}

#endif
