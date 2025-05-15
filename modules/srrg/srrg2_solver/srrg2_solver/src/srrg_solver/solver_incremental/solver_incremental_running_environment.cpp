#include <string>
#include <iostream>
#include "solver_incremental_running_environment.h"

namespace srrg2_solver {
  using  namespace std;

  SolverIncrementalRunningEnvironment::SolverIncrementalRunningEnvironment(ParseCommandLine& cmd_line_):
    cmd_line(cmd_line_),
    input_file(&cmd_line, "i",    "input-file",             "file where to read the input ", ""),
    config_file(&cmd_line, "c",    "config",             "file where to read the slover configuration ", "splitter.config"),
    solver_name(&cmd_line,
                "n",
                "star-solver",
                "name of the solver in the star",
                "star_solver"),
    output_file(&cmd_line, "o",    "output-file",           "file where to save the output", ""),
    do_tag(&cmd_line, "t", "tag", "accumulates the measurements till end epoch"),
    do_finalize(&cmd_line, "f", "finalize", "runs a global low level opt when done at sparse level"),
    benchmark_file(&cmd_line, "bench",    "benchmark-file",             "file where to read the gt for evaluation ", "")
  {}

  void SolverIncrementalRunningEnvironment::setup(std::set<VariableBase::Id>& relaxed) {
    if (benchmark_file.isSet()) {
      gt_graph=FactorGraph::read(benchmark_file.value());
      _evaluator.setGroundTruth(*gt_graph);
    }
    
    // load config file
    _manager.read(config_file.value());
    
    // retrieve the solver 
    solver=_manager.getByName<SolverIncrementalBase>(solver_name.value());
    if (!solver) {
      throw std::runtime_error(std::string(environ[0]) + "|ERROR, cannot find solver with name [ " +
                               solver_name.value() + " ] in configuration file [ " +
                               config_file.value() + " ]");
    }

    // create a graph
    graph.reset(new FactorGraph);

    // extract from the solver the path filename, and set it to the reader
    // to read the epochs
    reader.setPathType(solver->param_gauge_type.value());
    if (! input_file.isSet()) {
      throw std::runtime_error("no input file set");
    }
    reader.setFilePath(input_file.value());
    reader.setGraph(graph);
    reader.relaxed()=relaxed;

    // bind the solver to the graph
    solver->setGraph(graph);

  }

  bool SolverIncrementalRunningEnvironment::readEpoch(std::set<VariableBase::Id>& new_vars,
                                               std::set<VariableBase::Id>& new_factors){
    if (do_tag.isSet()){
      return reader.readEpochTag(new_vars, new_factors);
    } else {
      return reader.readEpoch(new_vars, new_factors);
    }
  }

  
  void SolverIncrementalRunningEnvironment::run() {
    std::set<VariableBase::Id> new_vars;
    std::set<FactorBase::Id> new_factors;
    while (readEpoch(new_vars, new_factors)) {
      if(!new_factors.size())
        continue;
      cerr << "epoch!" << endl;
      startEpochCallback(new_vars, new_factors);
      solver->compute(new_factors, do_tag.isSet());
      endEpochCallback();
    }
    if (new_factors.size()) {
      solver->compute(new_factors, true);
    }
    endDataCallback();
  }

  void SolverIncrementalRunningEnvironment::startEpochCallback(std::set<VariableBase::Id>& new_vars,
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

  void SolverIncrementalRunningEnvironment::endDataCallback(){
    Chrono::printReport(solver->resourceUsage());
    if (! output_file.isSet()) {
      cerr << "NO OUTPUT SELECTED, GOODIES NOT SAVED" << endl;
    }

    cerr << "Finalizing... (might take long)" << endl;
    solver->finalize(); 
    if (do_finalize.isSet()) {
      solver->finalize();
    }
    cerr << "done" << endl;
    FactorGraphView final_view;
    final_view.addFactors(*solver->graph(), solver->finalizedFactors());
    final_view.write(std::string("staropt_")+output_file.value());

  }
  void SolverIncrementalRunningEnvironment::endEpochCallback(){
    if (gt_graph) {
      FactorGraphView view;
      solver->computeSurroundingView(view);
      _evaluator.compute(view);
    }
  }
  
  SolverIncrementalRunningEnvironment::~SolverIncrementalRunningEnvironment(){}

}
