#include "../include/dp.hpp"


namespace cleaner{
    dp::dp(world const& w, double epsilon, double gamma) : w(w), epsilon(epsilon), gamma(gamma){}

    void dp::solve(){
      this->init();

      do{
        this->copy();
        this->backup();
      }while( distance() > this->epsilon );
    }

    double dp::getValueAt(int s) const{
      return this->vf.at(s);
    }

    void dp::backup(){
      for(int s=0; s<this->w.getNumStates(); ++s){
        this->vf[s] = MIN;
        for(int a=0; a<action::END; a++){

          double value = w.reward(w.getState(s), static_cast<action>(a));
          for(int ss=0; ss<this->w.getNumStates(); ++ss)
            value += this->gamma * w.probability(w.getState(s), static_cast<action>(a), w.getState(ss)) * this->vf_copy[ss];

          this->vf[s] = std::max(this->vf[s], value);
        }
      }
    }

    void dp::init(){
      for(int s=0; s<this->w.getNumStates(); ++s){
        this->vf.emplace(s, 0.0);
      }
    }

    void dp::copy(){
      for(int s=0; s<this->w.getNumStates(); ++s){
        this->vf_copy[s] = this->vf[s];
      }
    }

    double dp::distance(){
      double error = 0;
      for(int s=0; s<this->w.getNumStates(); ++s){
        error = std::max( error, std::abs(this->vf[s] - this->vf_copy[s]) );
      }

      //TODO
      std::cout << error << std::endl;
      return error;
    }
}
