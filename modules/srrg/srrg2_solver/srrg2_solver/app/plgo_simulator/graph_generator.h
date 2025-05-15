#pragma once
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <iomanip>
#include <memory>
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "plgo_simulator.h"
#include "sim_record.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  struct GraphGeneratorBase {
  
    PLGOSimulator& sim;
    FactorGraphPtr graph;
    bool has_odom=false;
    int landmark_min_measurements=3;
    GraphGeneratorBase(PLGOSimulator& sim_): sim(sim_){}
    SimRecordList records;


    std::map<std::string, int> constant_ids;
    
    using LandmarkFactorList = std::list<FactorBase*>;
    using LandmarkFactorVectorList = std::vector<LandmarkFactorList>;

    LandmarkFactorVectorList pending_landmark_factors;
    std::vector <VariableBase*> landmark_vars;
    std::vector<int> landmarks_num_seen;

    int landmark_idx_offset;
    int image_sizes_idx_offset;
    int projection_matrix_idx_offset;
    void compute();
    virtual ~GraphGeneratorBase();
  protected:
    virtual VariableBase* createProjectionMatrixVariable() {return 0;}
    virtual VariableBase* createImageSizesVariable() { return 0; };
    virtual VariableBase* createLandmarkSensorOffsetVariable() { return 0;};
    virtual VariableBase* createPoseVariable(int idx) = 0;
    virtual FactorBase*   createPoseFactor(int idx_from, int idx_to) = 0;
    virtual VariableBase* createLandmarkVariable(int idx) = 0;
    virtual FactorBase*   createLandmarkFactor(int idx_from, int idx_to, int idx_offset) = 0;

  };

  using GraphGeneratorPtr = std::shared_ptr<GraphGeneratorBase>;

  struct GraphGeneratorSE3 : public GraphGeneratorBase {
    GraphGeneratorSE3(PLGOSimulator& sim_): GraphGeneratorBase(sim_){}
    // to be specialized in derived classes to provide the right flavor
    VariableBase* createLandmarkSensorOffsetVariable() override ;
    VariableBase* createPoseVariable(int idx) override ;
    FactorBase*   createPoseFactor(int idx_from, int idx_to) override ;
    VariableBase* createLandmarkVariable(int idx) override ;
    FactorBase*   createLandmarkFactor(int idx_from, int idx_to, int idx_offset) override ;
  };

  struct GraphGeneratorSim3 : public GraphGeneratorBase {
    GraphGeneratorSim3(PLGOSimulator& sim_): GraphGeneratorBase(sim_){}
    VariableBase* createPoseVariable(int idx) override ;
    FactorBase*   createPoseFactor(int idx_from, int idx_to) override ;
    VariableBase* createLandmarkVariable(int idx) override ;
    FactorBase*   createLandmarkFactor(int idx_from, int idx_to, int idx_offset) override ;
  };

  struct GraphGeneratorSE3OmniBA : public GraphGeneratorSE3 {
    GraphGeneratorSE3OmniBA(PLGOSimulator& sim_): GraphGeneratorSE3(sim_){}
    FactorBase*   createLandmarkFactor(int idx_from, int idx_to, int idx_offset) override ;
  };

  struct GraphGeneratorSE3PinholeBA : public GraphGeneratorSE3 {
    GraphGeneratorSE3PinholeBA(PLGOSimulator& sim_): GraphGeneratorSE3(sim_){}
    VariableBase* createProjectionMatrixVariable() override;
    VariableBase* createImageSizesVariable() override;
    FactorBase*   createLandmarkFactor(int idx_from, int idx_to, int idx_offset) override ;
  };

  struct GraphGeneratorSE3RectifiedStereo : public GraphGeneratorSE3PinholeBA {
    GraphGeneratorSE3RectifiedStereo(PLGOSimulator& sim_): GraphGeneratorSE3PinholeBA(sim_){}
    VariableBase* createImageSizesVariable() override;
    FactorBase*   createLandmarkFactor(int idx_from, int idx_to, int idx_offset) override ;
  };

}
