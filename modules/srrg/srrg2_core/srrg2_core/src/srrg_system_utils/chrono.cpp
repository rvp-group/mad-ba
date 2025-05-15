#include <iostream>
#include "chrono.h"
#include <srrg_system_utils/system_utils.h>

namespace srrg2_core {
  using namespace std;
  Chrono::Chrono(const std::string& tag_,
                 Chrono::ChronoMap* time_map_,
                 bool verbose_) {
    _tag=tag_;
    _time_map=time_map_;
    _verbose=verbose_;
    _time = getTime();
  }

  Chrono::Chrono(const std::string& tag_,
                 double& accumulator_,
                 bool verbose_) {
    _tag=tag_;
    _accumulator=&accumulator_;
    _verbose=verbose_;
    _time = getTime();
  }
    
  Chrono::~Chrono() {
    double t_start=_time;
    double _duration= getTime() - _time;
    if (_verbose)
      std::cerr << _tag << " " << _duration << endl;
    if (_accumulator) {
      *_accumulator+=_duration;
    }
    if (! _time_map) {
      return;
    }
    auto it=_time_map->find(_tag);
    if (it==_time_map->end()) {
      auto result=_time_map->insert(std::make_pair(_tag, std::list<std::pair<double, double> >()));
      it=result.first;
    }
    it->second.push_back(std::make_pair(t_start, _duration));
  }

  double Chrono::duration() const {return getTime()-_time;}

  void Chrono::printReport(const ChronoMap& chrono_map){
    cerr << "TOTAL_TIME" << endl;
    for (const auto& m_it: chrono_map) {
      double duration=0;
      for (const auto& e_it: m_it.second) {
        duration+=e_it.second;
      }
      int n_events=m_it.second.size();
      if (n_events) {
        cerr << m_it.first << duration<< ", #events:" << n_events << " avg_time: " << duration/n_events << endl;
      }
    }
  }

}
