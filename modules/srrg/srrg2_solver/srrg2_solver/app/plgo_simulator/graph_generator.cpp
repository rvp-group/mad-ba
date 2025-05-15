#include "graph_generator.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_point_offset_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/variable_sim3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/sim3_pose_pose_error_factor_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_pinhole_ba_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_rectified_stereo_error_factor.h"

namespace srrg2_solver {
  using namespace std;

  GraphGeneratorBase::~GraphGeneratorBase() {}
  
  void GraphGeneratorBase::compute() {
    
    landmarks_num_seen.resize(sim.landmarks.size());
    std::fill(landmarks_num_seen.begin(), landmarks_num_seen.end(), 0);
    landmark_vars.resize(sim.landmarks.size());
    std::fill(landmark_vars.begin(), landmark_vars.end(), nullptr);
    pending_landmark_factors.resize(sim.landmarks.size());

    SimRecordPtr  record (new SimRecord);
    record->epoch=-1;
    
    // cerr << "generating graph for incremental optimization" << endl;
    
    graph=FactorGraphPtr (new FactorGraph);
    // cerr << "adding sensor offset fixed var" << endl;
    // add the offset
    
    landmark_idx_offset=sim.poses.size();
    VariableBase* offset_var = createLandmarkSensorOffsetVariable();
    if (offset_var) {
      offset_var->setGraphId(landmark_idx_offset);
      graph->addVariable(VariableBasePtr(offset_var));
      constant_ids["sensor_offset"]=offset_var->graphId();
      landmark_idx_offset++;
      record->vars.push_back(offset_var->graphId());
      cerr << "offset_id: " << offset_var->graphId() << endl;
    }
    
    VariableBase* proj_var = createProjectionMatrixVariable();
    if (proj_var){
      proj_var->setGraphId(landmark_idx_offset);
      graph->addVariable(VariableBasePtr(proj_var));
      constant_ids["proj"]=proj_var->graphId();
      landmark_idx_offset++;
      record->vars.push_back(proj_var->graphId());
      cerr << "proj_id: " << proj_var->graphId() << endl;
    }
    VariableBase* image_sizes_var=createImageSizesVariable();
    if (image_sizes_var) {
      image_sizes_var->setGraphId(landmark_idx_offset);
      graph->addVariable(VariableBasePtr(image_sizes_var));
      landmark_idx_offset++;
      constant_ids["image_sizes"]=image_sizes_var->graphId();
      record->vars.push_back(image_sizes_var->graphId());
      cerr << "sizes_id: " << image_sizes_var->graphId() << endl;
    }
    records.push_back(record);
    // process the poses sequentially,
    // for each pose:
    //     add a new variable
    //     if needed add an odometry factor
    //     update the landmark seen


    size_t la_idx=0;
    size_t pa_idx=0;

    VariableBase* pose_prev=0;
    for (int idx_from=0; idx_from< (int) sim.poses.size(); ++idx_from) {
      
      // cerr << "processing pose: " << idx_from << endl;
      SimRecordPtr  record (new SimRecord);
      record->epoch=idx_from;
      
      VariableBase* pose_var=createPoseVariable(idx_from);
      graph->addVariable(VariableBasePtr(pose_var));

      if (! pose_prev)
        pose_var->setStatus(VariableBase::Fixed);
      record->vars.push_back(pose_var->graphId());

      // odometry
      if (has_odom && pose_prev) {
        // cerr << "\tmaking odometry: " << pose_prev->graphId() << " -> " << pose_var->graphId() << endl;
        FactorBase* odom_factor=createPoseFactor(pose_prev->graphId(), pose_var->graphId());
        graph->addFactor(FactorBasePtr(odom_factor));

        record->factors.push_back(odom_factor->graphId());
      }
      pose_prev=pose_var;

      // cerr << "scanning landmarks " << endl;      //landmarks
      while(la_idx<sim.landmark_associations.size()) {
        const IntPair& l_ass=sim.landmark_associations[la_idx];
        if (l_ass.first!=idx_from)
          break;
        
        // the first time a landmark is seen we make the variable, otherwise
        // we retrieve it
        int idx_to=l_ass.second;
        ++la_idx;
        
        // cerr << "\tlandmark tmeasurement "<< idx_from << " -> " << landmark_idx_offset+idx_to << " added to pending list" << endl;

        // cerr << "\t\tlandmark:  " << landmark_idx_offset+idx_to ;
        
        VariableBase* landmark_var=landmark_vars[idx_to];
        if (!landmark_var) {
          landmark_var=createLandmarkVariable(idx_to);
          landmark_vars[idx_to]=landmark_var;
        }
        
        //we add the factor to the pending list
        FactorBase* landmark_factor=createLandmarkFactor(idx_from, idx_to, offset_var->graphId());
        if (! landmark_factor)
          continue;
        
        pending_landmark_factors[idx_to].push_back(landmark_factor);
        
        bool has_variable=graph->variable(landmark_var->graphId());
        if (! has_variable &&
            (int) pending_landmark_factors[idx_to].size()<landmark_min_measurements) {
          // cerr << "\t\tskip adding (mm)  " <<  endl;
          continue;
        }
        if (! has_variable) {
          // cerr << "\t\tlandmark added" << endl;
          graph->addVariable(VariableBasePtr(landmark_var));
          record->vars.push_back(landmark_var->graphId());
        }
        // cerr << "\t\tflush: [ ";
        for (auto this_factor: pending_landmark_factors[idx_to]) {
          
          // cerr << "("    << this_factor->variableId(0)
          //               << " -> " << this_factor->variableId(1) << ") ";
          graph->addFactor(FactorBasePtr(this_factor));
          record->factors.push_back(this_factor->graphId());
        }
        // cerr << "]" << endl;
        pending_landmark_factors[idx_to].clear();
      }

      //poses
      while(pa_idx<sim.pose_associations.size()) {
        const IntPair& p_ass=sim.pose_associations[pa_idx];
        if (p_ass.first!=idx_from)
          break;
        int idx_to=p_ass.second;
        ++pa_idx;
        FactorBase* pose_factor=createPoseFactor(idx_from, idx_to);
        graph->addFactor(FactorBasePtr(pose_factor));
        // cerr << "\tpose tmeasurement "<< idx_from << " -> " << idx_to << " added" << endl;
        record->factors.push_back(pose_factor->graphId());
      }
      records.push_back(record);
    }
  }

  //======================== SE3 generator ========================

  VariableBase* GraphGeneratorSE3::createLandmarkSensorOffsetVariable() {
    VariableSE3QuaternionRightAD* offset_var=new VariableSE3QuaternionRightAD;
    offset_var->setEstimate(sim.landmark_sensor_offset);
    offset_var->setGraphId(sim.poses.size());
    offset_var->setStatus(VariableBase::Fixed); // lock the offset
    return offset_var;
  }

  VariableBase* GraphGeneratorSE3::createPoseVariable(int idx) {
    VariableSE3QuaternionRightAD* pose_var=new VariableSE3QuaternionRightAD;
    pose_var->setGraphId(idx);
    pose_var->setEstimate(sim.poses[idx]);
    return pose_var;
  }

  FactorBase* GraphGeneratorSE3::createPoseFactor(int idx_from, int idx_to) {
    const VariableSE3QuaternionRightAD* pose_from=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_from));
    const VariableSE3QuaternionRightAD* pose_to=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_to));
    assert(pose_from && pose_to && "bookkeeping error, no vars in graph");
    SE3PosePoseGeodesicErrorFactor* pose_factor=new SE3PosePoseGeodesicErrorFactor();
    pose_factor->setVariableId(0, pose_from->graphId());
    pose_factor->setVariableId(1, pose_to->graphId());
    pose_factor->setMeasurement(pose_from->estimate().inverse()
                                *pose_to->estimate());
    return pose_factor;
  }

  VariableBase* GraphGeneratorSE3::createLandmarkVariable(int idx) {
    if (landmark_vars[idx]!=nullptr) {
      return landmark_vars[idx];
    }
    VariablePoint3AD* landmark_var=new VariablePoint3AD;
    landmark_var->setGraphId(landmark_idx_offset+idx);
    landmark_var->setEstimate(sim.landmarks[idx]);
    return landmark_var;
  }

  FactorBase* GraphGeneratorSE3::createLandmarkFactor(int idx_from, int idx_to, int idx_offset) {
    const VariableSE3QuaternionRightAD* pose_var=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_from));
    const VariableSE3QuaternionRightAD* offset_var=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_offset));
    const VariablePoint3AD* landmark_var=dynamic_cast<const VariablePoint3AD*>(landmark_vars[idx_to]);
    SE3PosePointOffsetErrorFactor* landmark_factor=new SE3PosePointOffsetErrorFactor();
    Isometry3f sensor_pose=pose_var->estimate()*offset_var->estimate();
    landmark_factor->setMeasurement(sensor_pose.inverse()*sim.landmarks[idx_to]);
    landmark_factor->setVariableId(0, pose_var->graphId());
    landmark_factor->setVariableId(1, landmark_var->graphId());
    landmark_factor->setVariableId(2, offset_var->graphId());
    return landmark_factor;
  }

    //======================== Sim generator ========================

  VariableBase* GraphGeneratorSim3::createPoseVariable(int idx) {
    VariableSim3QuaternionRightAD* pose_var=new VariableSim3QuaternionRightAD;
    pose_var->setGraphId(idx);
    Similiarity_<float,3> v;
    v.linear()=sim.poses[idx].linear();
    v.translation()=sim.poses[idx].translation();
    v.inverseScaling()=1.f;
    pose_var->setEstimate(v);
    return pose_var;
  }

  FactorBase* GraphGeneratorSim3::createPoseFactor(int idx_from, int idx_to) {
    const VariableSim3QuaternionRightAD* pose_from=dynamic_cast<const VariableSim3QuaternionRightAD*>(graph->variable(idx_from));
    const VariableSim3QuaternionRightAD* pose_to=dynamic_cast<const VariableSim3QuaternionRightAD*>(graph->variable(idx_to));
    assert(pose_from && pose_to && "bookkeeping error, no vars in graph");
    Sim3PosePoseErrorFactorAD* pose_factor=new Sim3PosePoseErrorFactorAD();
    pose_factor->setVariableId(0, pose_from->graphId());
    pose_factor->setVariableId(1, pose_to->graphId());
    pose_factor->setMeasurement(pose_from->estimate().inverse()
                                *pose_to->estimate());
    return pose_factor;
  }

  VariableBase* GraphGeneratorSim3::createLandmarkVariable(int idx) {
    throw std::runtime_error("generator does not support landmarks");
    return 0;
  }

  FactorBase* GraphGeneratorSim3::createLandmarkFactor(int idx_from, int idx_to, int idx_offset) {
    throw std::runtime_error("generator does not support landmarks");
    return 0;
  }

  //=============================== SE3_Omni_BA ===============================

  FactorBase* GraphGeneratorSE3OmniBA::createLandmarkFactor(int idx_from, int idx_to, int idx_offset) {
    const VariableSE3QuaternionRightAD* pose_var=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_from));
    const VariableSE3QuaternionRightAD* offset_var=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_offset));
    const VariablePoint3AD* landmark_var=dynamic_cast<const VariablePoint3AD*>(landmark_vars[idx_to]);
    SE3PosePointOmniBAErrorFactor* landmark_factor=new SE3PosePointOmniBAErrorFactor();
    Isometry3f sensor_pose=pose_var->estimate()*offset_var->estimate();
    Vector3f point_in_sensor=sensor_pose.inverse()*sim.landmarks[idx_to];
    float n = point_in_sensor.norm();
    if (n<1e-3)
      return 0;
    landmark_factor->setMeasurement(point_in_sensor/n);
    landmark_factor->setVariableId(0, pose_var->graphId());
    landmark_factor->setVariableId(1, landmark_var->graphId());
    landmark_factor->setVariableId(2, offset_var->graphId());
    return landmark_factor;
  }

    //=============================== SE3_Pinhole_BA ===============================

  VariableBase* GraphGeneratorSE3PinholeBA::createProjectionMatrixVariable()  {
    VariableMatrix3_4* projection_matrix_var=new VariableMatrix3_4;
    Eigen::Matrix3f camera_matrix;
    camera_matrix <<
      100, 0, 320,
      0, 100, 240,
      0, 0, 1;
    auto est=projection_matrix_var->estimate();
    int off_id=constant_ids["sensor_offset"];
    const VariableSE3QuaternionRightAD* offset_var=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(off_id));
    auto inv_offset=offset_var->estimate().inverse();
    est.block<3,3>(0,0)=camera_matrix*inv_offset.linear();
    est.block<3,1>(0,3)=camera_matrix*inv_offset.translation();
    projection_matrix_var->setEstimate(est);
    projection_matrix_var->setStatus(VariableBase::Fixed);
    return projection_matrix_var;
  }

  VariableBase* GraphGeneratorSE3PinholeBA::createImageSizesVariable()  {
    VariablePoint2* image_sizes=new VariablePoint2;
    image_sizes->setEstimate(Eigen::Vector2f(640,480));
    image_sizes->setStatus(VariableBase::Fixed);
    return image_sizes;
  }

  
  FactorBase* GraphGeneratorSE3PinholeBA::createLandmarkFactor(int idx_from, int idx_to, int idx_offset) {

    auto v = graph->variable(idx_from);
    const VariableSE3QuaternionRight* pose_var=dynamic_cast<const VariableSE3QuaternionRight*>(v);

    if (! pose_var) {
      cerr << "num vars: " << graph->variables().size() << endl;
      cerr << "type: " << v->className() << endl;
      cerr << "no pose [" << idx_from << "]" << endl; exit(0);
    }

    int idx_proj=constant_ids["proj"];
    VariableMatrix3_4* proj_var=dynamic_cast<VariableMatrix3_4*>(graph->variable(idx_proj));
    if (! proj_var) {
      cerr << "no proj" << endl; exit(0);
    }
    int idx_sizes=constant_ids["image_sizes"];
    VariablePoint2* sizes_var=dynamic_cast<VariablePoint2*>(graph->variable(idx_sizes));
    if (! sizes_var) {
      cerr << "no sizes"<< endl; exit(0);
    }

    const VariablePoint3AD* landmark_var=dynamic_cast<const VariablePoint3AD*>(landmark_vars[idx_to]);


    if (! landmark_var) {
      cerr << "no landmark"<< endl; exit(0);
    }

    Vector2f image_coords;
    bool valid=SE3PosePointPinholeBAErrorFactor::computePrediction(image_coords,
                                                                   *pose_var,
                                                                   *landmark_var,
                                                                   *proj_var,
                                                                   *sizes_var);
    if (! valid) {
      return 0;
    }
    SE3PosePointPinholeBAErrorFactor* landmark_factor=new SE3PosePointPinholeBAErrorFactor();

    
    landmark_factor->setVariableId(0, pose_var->graphId());
    landmark_factor->setVariableId(1, landmark_var->graphId());
    landmark_factor->setVariableId(2, proj_var->graphId());
    landmark_factor->setVariableId(3, sizes_var->graphId());
    landmark_factor->setMeasurement(image_coords);
    return landmark_factor;
  }


  //=============================== SE3_Stereo ===============================

  VariableBase* GraphGeneratorSE3RectifiedStereo::createImageSizesVariable()  {
    VariablePoint3* image_sizes=new VariablePoint3;
    image_sizes->setEstimate(Eigen::Vector3f(640,480,100));
    image_sizes->setStatus(VariableBase::Fixed);
    return image_sizes;
  }

  
  FactorBase* GraphGeneratorSE3RectifiedStereo::createLandmarkFactor(int idx_from, int idx_to, int idx_offset) {

    auto v = graph->variable(idx_from);
    const VariableSE3QuaternionRight* pose_var=dynamic_cast<const VariableSE3QuaternionRight*>(v);

    if (! pose_var) {
      cerr << "num vars: " << graph->variables().size() << endl;
      cerr << "type: " << v->className() << endl;
      cerr << "no pose [" << idx_from << "]" << endl; exit(0);
    }

    int idx_proj=constant_ids["proj"];
    VariableMatrix3_4* proj_var=dynamic_cast<VariableMatrix3_4*>(graph->variable(idx_proj));
    if (! proj_var) {
      cerr << "no proj" << endl; exit(0);
    }
    int idx_sizes=constant_ids["image_sizes"];
    VariablePoint3* sizes_var=dynamic_cast<VariablePoint3*>(graph->variable(idx_sizes));
    if (! sizes_var) {
      cerr << "no sizes"<< endl; exit(0);
    }

    const VariablePoint3AD* landmark_var=dynamic_cast<const VariablePoint3AD*>(landmark_vars[idx_to]);


    if (! landmark_var) {
      cerr << "no landmark"<< endl; exit(0);
    }

    Vector3f image_coords;
    bool valid=SE3PosePointRectifiedStereoErrorFactor::computePrediction(image_coords,
                                                                         *pose_var,
                                                                         *landmark_var,
                                                                         *proj_var,
                                                                         *sizes_var);
    if (! valid) {
      return 0;
    }
    SE3PosePointRectifiedStereoErrorFactor* landmark_factor=
      new SE3PosePointRectifiedStereoErrorFactor();
    
    landmark_factor->setVariableId(0, pose_var->graphId());
    landmark_factor->setVariableId(1, landmark_var->graphId());
    landmark_factor->setVariableId(2, proj_var->graphId());
    landmark_factor->setVariableId(3, sizes_var->graphId());
    landmark_factor->setMeasurement(image_coords);
    return landmark_factor;
  }


}
