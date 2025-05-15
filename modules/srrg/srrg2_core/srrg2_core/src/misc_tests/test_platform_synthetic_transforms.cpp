#include <iostream>

#include "../srrg_data_structures/platform.h"
#include "srrg_geometry/geometry3d.h"

using namespace srrg2_core;

int32_t main() {

  //ds compute example transform
  Vector6d v; v << 1, 0, 0, 0.5, -0.5, 0.5;
  const Isometry3f example_transform(geometry3d::v2t(v));
  std::cerr << "---------------------------------------------------------- test transforms" << std::endl;

  //ds instanciate joints
  LinkWithJointPrismatic* j0 = new LinkWithJointPrismatic("j0", Vector3f(1, 0, 0), example_transform); //ds prismatic base
  Link* j1 = new Link("j1", example_transform); //ds fixed link
  Link* j2 = new Link("j2", example_transform); //ds fixed link
  Link* j3 = new Link("j3", example_transform); //ds fixed link
  Link* j4 = new Link("j4", example_transform); //ds fixed link
  Link* j5 = new Link("j5", example_transform); //ds fixed link
  Link* j6 = new Link("j6", example_transform); //ds fixed link
  Link* j7 = new Link("j7", example_transform); //ds fixed link
  Link* j8 = new Link("j8", example_transform); //ds fixed link

  //ds connect the joints
  j1->setParent(j0);
  j2->setParent(j0);
  j3->setParent(j0);
  j4->setParent(j1);
  j5->setParent(j2);
  j6->setParent(j3);
  j7->setParent(j5);
  j8->setParent(j5);
  std::cerr << j1 << std::endl;
  std::cerr << j5 << std::endl;
  std::cerr << j8 << std::endl;

  //ds allocate an empty platform
  Platform platform;
  std::cerr << "---------------------------------------------------------- test model filling" << std::endl;

  //ds add joints to model
  platform.addLink(j0);
  platform.addLink(j1);
  platform.addLink(j2);
  platform.addLink(j3);
  platform.addLink(j4);
  platform.addLink(j5);
  platform.addLink(j6);
  platform.addLink(j7);
  platform.addLink(j8);

  //ds try duplicate additions
  platform.addLink(j8);
  platform.addLink(j4);
  std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;

  //ds retrieve transforms of last state
  std::cerr << "---------------------------------------------------------- test model construction" << std::endl;
  Isometry3f j0_in_world; 
  if(platform.getTransform(j0_in_world,"j0")){
    std::cerr << "[j0 in world]: " << geometry3d::t2v(j0_in_world).transpose() << std::endl;
    std::cerr << j1 << std::endl;
    std::cerr << j5 << std::endl;
    std::cerr << j8 << std::endl;
  }
  //ds change root joint j0 -> j9
  Link* j9 = new Link("j9", example_transform);
  j0->setParent(j9);
  platform.addLink(j9);
  std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;
  std::cerr << platform << std::endl;

  //ds retrieve transforms of last state
  if(platform.getTransform(j0_in_world,"j0")){
    std::cerr << "[j0 in world]: " << geometry3d::t2v(j0_in_world).transpose() << std::endl;
    std::cerr << platform << std::endl;
    std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;
  }
  //ds try inserting another root joint
  std::cerr << "---------------------------------------------------------- test model disturbance" << std::endl;
  Link* j10 = new Link("j10", example_transform);
  platform.addLink(j10);
  std::cerr << "is well formed: " << platform.isWellFormed() << std::endl;
  std::cerr << platform << std::endl;

  //ds retrieve transforms of last state
  Isometry3f j5_in_j7 ;
  if(platform.getTransform(j5_in_j7,"j5", "j7"))
    std::cerr << "[j5 in j7]: " << geometry3d::t2v(j5_in_j7).transpose() << std::endl;

  //ds retrieve transforms at time: 1s
  if(platform.getTransform(j0_in_world,"j0", 1.0))
    std::cerr << "[j0 in world] at t=1s: " << geometry3d::t2v(j0_in_world).transpose() << std::endl;
  if(platform.getTransform(j5_in_j7,"j5", "j7", 1.0))
    std::cerr << "[j5 in j7] at t=1s: " << geometry3d::t2v(j5_in_j7).transpose() << std::endl;

  //ds instanciate events for joint j0
  std::cerr << "---------------------------------------------------------- test joint events in model" << std::endl;
  JointEventPtr joint_event_t0(new JointEvent(0, "j0", 0));
  JointEventPtr joint_event_t1(new JointEvent(5, "j0", 100));

  //ds include an invalid event
  TransformEventPtr invalid_event_t2(new TransformEvent(2, "j0", Isometry3f::Identity()));
  std::cerr << "number of added events for joint j0: " << j0->numberOfEvents() << std::endl;
  platform.addEvent(joint_event_t0);
  platform.addEvent(joint_event_t1);
  platform.addEvent(invalid_event_t2);
  std::cerr << "number of added events for joint j0: " << j0->numberOfEvents() << std::endl;

  //ds retrieve transforms at time: 1s
  if(platform.getTransform(j0_in_world,"j0", 1.0))
    std::cerr << "[j0 in world] at t=1s: " << geometry3d::t2v(j0_in_world).transpose() << std::endl;
  if(platform.getTransform(j5_in_j7,"j5", "j7", 1.0))
    std::cerr << "[j5 in j7] at t=1s: " << geometry3d::t2v(j5_in_j7).transpose() << std::endl;

  //ds add a transform event for link 1
  std::cerr << "number of added events for joint j1: " << j1->numberOfEvents() << std::endl;
  Isometry3f transform_update_t1(Isometry3f::Identity());
  transform_update_t1.translation() = Vector3f(10, 5, 25);
  TransformEventPtr transform_event_t1(new TransformEvent(1, "j1", transform_update_t1, "j0")); //ds provide additional parent information (optional)
  Isometry3f transform_update_t2(Isometry3f::Identity());
  transform_update_t2.translation() = Vector3f(10, 5, 0);
  TransformEventPtr transform_event_t2(new TransformEvent(2, "j1", transform_update_t2, "j0")); //ds provide additional parent information (optional)
  TransformEventPtr transform_event_t3(new TransformEvent(3, "j1", transform_update_t2, "j0")); //ds an event which contains now new information, to check double insertion
  TransformEventPtr transform_event_j9_t0(new TransformEvent(0, "j9", example_transform));
  TransformEventPtr transform_event_j9_t10(new TransformEvent(10, "j9", example_transform));
  platform.addEvent(transform_event_t1);
  platform.addEvent(transform_event_t2);
  platform.addEvent(transform_event_t3);
  platform.addEvent(transform_event_j9_t0);
  platform.addEvent(transform_event_j9_t10);
  std::cerr << "number of added events for joint j1: " << j1->numberOfEvents() << std::endl;
  std::cerr << "number of added events for joint j9: " << j9->numberOfEvents() << std::endl;

  //ds retrieve transforms at time: 1s
  /* Isometry3f j1_in_world = platform.getTransform("j1", 1.0); */
  /* std::cerr << "[j1 in world] at t = 1s: " << geometry3d::t2v(j1_in_world).transpose() << " status: " << platform.sampledStatus() << std::endl; */
  /* j1_in_world = platform.getTransform("j1", 2.5); */
  /* std::cerr << "[j1 in world] at t = 2.5s: " << geometry3d::t2v(j1_in_world).transpose() << " status: " << platform.sampledStatus() << std::endl; */
  /* j1_in_world = platform.getTransform("j1", 3.5); */
  /* std::cerr << "[j1 in world] at t = 3.5s: " << geometry3d::t2v(j1_in_world).transpose() << " status: " << platform.sampledStatus() << std::endl; */
  return 0;
}
