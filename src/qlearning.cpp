#include "../include/qlearning.hpp"


namespace cleaner{
    qlearning::qlearning(world const& w, double epsilon, double learning_rate, double gamma, int episodes) : w(w), epsilon(epsilon), learning_rate(learning_rate), gamma(gamma), episodes(episodes){
    }

    qlearning::~qlearning(){
    }

    void qlearning::plots(){
      std::cout << this->getValueAt(0) << std::endl;
  }

    void qlearning::solve(){
      double r;
      int s, a, ss;
      this->init();

      do{
        this->t1 = 0.5;
        this->t2 = 0.5;
        s=0;
        for(int i=0; i<100; i++){
          a = greedy(s);
          w.execute(s, static_cast<action>(a), ss, r);
          this->backup(s,a,ss,r);
          s = ss;
        }

        this->plots();
      }while( ++this->episode < this->episodes );
    }

    double qlearning::getValueAt(int s){
      double value = MIN;
      for(int a=0; a<action::END; ++a){
        value = std::max(value, this->qf[s][a]);
      } return value;
    }

    int qlearning::greedy(int s){
      int agreedy;
      double value = MIN;
      double rd = rand() / ((double) RAND_MAX);

      if( rd > this->epsilon ) {
        for(int a=0; a<action::END; ++a){
          if( value < this->qf[s][a] ){
            agreedy = a;
            value = this->qf[s][a];
          }
        }
      }

      else {
        agreedy = rand() % 7;
      }

      return agreedy;
    }

    void qlearning::backup(int s, int a, int ss, double r){
      //this->qf[s][a] = this->qf[s][a] + this->learning_rate * (r + this->gamma * this->getValueAt(ss) - this->qf[s][a]);

      int width = w.getWidth();
      // calculate phi of state St
      phi1 = w.getState(s)->getPose() / width + w.getState(s)->getPose() % width;
      phi2 = w.getState(s)->getBattery;
      // calculate phi of state St+1
      switch(a) {
        case action::LEFT :
          phi1_ = phi1 - 1;
          phi2_ = phi2 - 1;
        break;
        case action::RIGHT :
          phi1_ = phi1 + 1;
          phi2_ = phi2 - 1;
        break;
        case action::UP :
          phi1_ = phi1 - 1;
          phi2_ = phi2 - 1;
        break;
        case action::DOWN :
          phi1_ = phi1 + 1;
          phi2_ = phi2 - 1;
        break;
        case action::CHARGE :
          phi1_ = phi1;
          phi2_ = phi2 + 1;
        break;
        case action::WAIT :
          phi1_ = phi1;
          phi2_ = phi2;
        break;
        case action::CLEAN :
          phi1_ = phi1;
          phi2_ = phi2;
        break;
      }
      // calculate Gt - phi(St) * theta
      fac = alpha * (r + gamma * (this->t1 * (phi1_-phi1) + this->t2 * (phi2_-phi2)));
      // calculate new theta
      dt1 = fac * phi1;
      dt2 = fac * phi2;
      this->t1 += dt1;
      this->t2 += dt2;
      
      this->qf[s][a] = this->t1 * phi1_+ this->t2 * phi2_;
    }

    void qlearning::init(){
      for(int s=0; s<this->w.getNumStates(); ++s){
        this->qf.emplace(s,  std::unordered_map<int, double>());
        for(int a=0; a<action::END; ++a){
          this->qf.at(s).emplace(a, 0.0);
        }
      }
    }
}
