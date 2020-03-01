#ifndef PREPROCESSING_CUBIC_MODEL_HPP_
#define PREPROCESSING_CUBIC_MODEL_HPP_

namespace models {
  namespace cubic_model {
    class Cubic_model;
    class Preprocessing_cubic_model {
      protected:
        Cubic_model& cubic_model;
      public:
        explicit Preprocessing_cubic_model (Cubic_model& cubic_model);
        virtual void add () = 0;
    };
  }
}

#endif
