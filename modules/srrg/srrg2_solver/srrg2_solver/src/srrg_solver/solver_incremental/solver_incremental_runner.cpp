#include <string>
#include <iostream>
#include "solver_incremental_runner.h"

namespace srrg2_solver {
  using  namespace std;


  void SolverIncrementalRunner::setup() {
    if (param_benchmark_file.value().length()>0) {
      gt_graph=FactorGraph::read(param_benchmark_file.value());
      _evaluator.setGroundTruth(*gt_graph);
    }
    
    // create a graph
    graph.reset(new FactorGraph);

    // extract from the solver the path filename, and set it to the reader
    // to read the epochs
    _reader.setPathType(param_solver.value()->param_gauge_type.value());
    if (! param_input_file.value().length()) {
      throw std::runtime_error("no input file set");
    }
    _reader.setFilePath(param_input_file.value());
    _reader.setGraph(graph);
    std::set<VariableBase::Id> relaxed;
    for (auto& v: param_relaxed.value())
      relaxed.insert(v);
    
    _reader.relaxed()=relaxed;

    // bind the solver to the graph
    param_solver->setGraph(graph);

  }

  bool SolverIncrementalRunner::readEpoch(std::set<VariableBase::Id>& new_vars,
                                               std::set<VariableBase::Id>& new_factors){
    if (param_end_epoch_tag.value()){
      return _reader.readEpochTag(new_vars, new_factors);
    } else {
      return _reader.readEpoch(new_vars, new_factors);
    }
  }

  
  void SolverIncrementalRunner::compute() {
    setup();
    std::set<VariableBase::Id> new_vars;
    std::set<FactorBase::Id> new_factors;
    while (readEpoch(new_vars, new_factors)) {
      if(!new_factors.size())
        continue;
      cerr << "epoch!" << endl;
      startEpochCallback(new_vars, new_factors);
      param_solver->compute(new_factors, param_end_epoch_tag.value());
      endEpochCallback();
      preemptLocal();
    }
    if (new_factors.size()) {
      param_solver->compute(new_factors, true);
      preemptLocal();
    }
    endDataCallback();
  }

  void SolverIncrementalRunner::reset() {
    param_solver->reset();
    setup();
  }

  void SolverIncrementalRunner::startEpochCallback(std::set<VariableBase::Id>& new_vars,
                                                        std::set<VariableBase::Id>& new_factors) {
    if (new_vars.empty())
      return;
    int length=80;
    int epoch=*new_vars.begin();
    int num_vars=new_vars.size();
    int num_factors=new_factors.size();
    std::string body(" EPOCH ");
    body += to_string(epoch) + "  VARS: " +to_string(num_vars) + "(" + to_string(graph->variables().size()) + ") FACTORS: " + to_string(num_factors) + "(" + to_string(graph->factors().size()) + ")  ";
    if (body.length()>(size_t) length)
      length=body.length()+2;
    int start=(length-body.length())/2;
    char buf[1024];
    for(int i=0; i<length; ++i) {
      buf[i]='*';
    }
    buf[length]=0;
    for (size_t k=0; k<body.length(); ++k)
      buf[k+start] = body[k];
    cerr << buf << endl;
  }

  void SolverIncrementalRunner::endDataCallback(){
    Chrono::printReport(param_solver->resourceUsage());
    cerr << "Finalizing... (might take long)" << endl;
    if (param_do_finalize.value()) {
      param_solver->finalize();
    }
    cerr << "done" << endl;
    FactorGraphView final_view;
    // final_view.addFactors(*solver->graph(), solver->finalizedFactors());
    // final_view.write(std::string("staropt_")+output_file.value());

  }
  void SolverIncrementalRunner::endEpochCallback(){
    if (gt_graph) {
      FactorGraphView view;
      param_solver->computeSurroundingView(view);
      _evaluator.compute(view);
    }
  }
  
  SolverIncrementalRunner::~SolverIncrementalRunner(){}

}
