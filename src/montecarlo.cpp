#include "../include/montecarlo.hpp"


namespace cleaner{
    montecarlo::montecarlo(world const& w, double epsilon, double gamma, double alpha, int episodes) : w(w), epsilon(epsilon), gamma(gamma), episodes(episodes){
    }

    montecarlo::~montecarlo(){
    }

    void montecarlo::plots(){
      std::cout << this->getValueAt(0) << std::endl;
  }

    void montecarlo::solve(){
      this->init();
      this->t1 = 0.5;
      this->t2 = 0.5;
      do{
        this->setEpisode();
        this->backup();

        this->plots();
      }while( ++this->cepisode < this->episodes );
    }

    double montecarlo::getValueAt(int s){
      double value = MIN;
      for(int a=0; a<action::END; ++a){
        value = std::max(value, this->qf[s][a]);
      } return value;
    }

    action montecarlo::greedy(int s){
      action agreedy;
      double value = MIN;
      for(int a=0; a<action::END; ++a){
        if( value < this->qf[s][a] ){
          agreedy = static_cast<action>(a);
          value = this->qf[s][a];
        }
      } return agreedy;
    }

    double montecarlo::getReturn(int pose){
      double r = 0;
      for(int i=pose; i<100; i++){
        r += pow( this->gamma, i-pose ) * std::get<2>(this->episode[i]);
      }

      return r;
    }


    void montecarlo::setEpisode(){
      action a;
      double r;
      this->episode.clear();
      int s, ss;

      for(s=0; s<this->w.getNumStates(); ++s){
        for(int a=0; a<action::END; ++a){
          this->pf[s][a] = -1;
        }
      }

      s = 0;
      double rd = rand() / ((double) RAND_MAX);
      for(int i=0; i<100; i++){
        if( rd > this->epsilon ) {
          a = greedy(s);
        }else {
          a = static_cast<action>(rand() % 7);
        }

        w.execute(s, a, ss, r);

        this->episode.push_back(std::make_tuple(s, a, r));

        if(this->pf[s][a] == -1){
          this->pf[s][a] = i;
        }

        s = ss;
      }
    }


    void montecarlo::backup(){
      int s, a;
      int width;
      double old, cumul;
      double dt1, dt2, dt3;
      double phi1, phi2, phi3;
      double fac;

      for(s=0; s<this->w.getNumStates(); ++s){
        for(a=0; a<action::END; ++a){
          if( this->pf[s][a] > -1 ){
            old = this->qf[s][a];
            cumul = this->getReturn(this->pf[s][a]);
            /*
            this->jf[s][a].second ++;
            this->jf[s][a].first += cumul;
            this->qf[s][a] = this->jf[s][a].first / this->jf[s][a].second;
            */
            width = w.getWidth();
            // calculate phi of state s
            phi1 = w.getState(s)->getPose() / width + w.getState(s)->getPose() % width;
            phi2 = w.getState(s)->getBattery;
            // calculate Gt - phi(St) * theta
            fac = alpha * (cumul - this->t1 * phi1 + this->t2 * ph2);
            // calculate new theta
            dt1 = fac * phi1;
            dt2 = fac * phi2;
            this->t1 += dt1;
            this->t2 += dt2;

            //calculate Q-table
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
            this->qf[s][a] = this->t1 * phi1_+ this->t2 * phi2_;

          }
        }
      }
    }

    void montecarlo::init(){
      for(int s=0; s<this->w.getNumStates(); ++s){
        this->pf.emplace(s,  std::unordered_map<int, int>());
        this->qf.emplace(s,  std::unordered_map<int, double>());
        this->jf.emplace(s,  std::unordered_map<int, std::pair<double, int>>());
        for(int a=0; a<action::END; ++a){
          this->pf.at(s).emplace(a, -1);
          this->qf.at(s).emplace(a, 0.0);
          this->jf.at(s).emplace(a, std::pair<double, int>(0.0, 0));
        }
      }
    }

}
