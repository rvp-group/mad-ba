#include <iostream>

#include "../srrg_data_structures/platform.h"
#include "srrg_geometry/geometry3d.h"

using namespace srrg2_core;

int32_t main() {

  //ds check events
  JointEventPtr event_t0(new JointEvent(0, "j0", 0));
  JointEventPtr event_t1(new JointEvent(1, "j0", 100));
  JointEventPtr event_t2(new JointEvent(2, "j0", 103));
  if (event_t0 < event_t1) {
    std::cerr << "event t=0 is earlier than event t=1" << std::endl;
  } else {
    std::cerr << "event t=1 is earlier than event t=0" << std::endl;
  }
  if (event_t2 > event_t1) {
    std::cerr << "event t=2 is older than event t=1" << std::endl;
  } else {
    std::cerr << "event t=1 is older than event t=2" << std::endl;
  }

  std::cerr << "copying event_t1 into event_t3: fields" << std::endl;
  JointEventPtr event_t3 = event_t1;
  std::cerr << "event_t3: " << event_t3->timeSeconds() << " = " << event_t1->timeSeconds() << std::endl;
  std::cerr << "event_t3: " << event_t3->position() << " = " << event_t1->position() << std::endl;
  std::cerr << "copying event_t2 into event_t3: fields" << std::endl;
  event_t3 = event_t2;
  std::cerr << "event_t3: " << event_t3->timeSeconds() << " = " << event_t2->timeSeconds() << std::endl;
  std::cerr << "event_t3: " << event_t3->position() << " = " << event_t2->position() << std::endl;

  LinkWithJointPrismatic* prismatic_joint = new LinkWithJointPrismatic("jp0", Vector3f(1, 0, 0));
  prismatic_joint->addEvent(event_t0);
  prismatic_joint->addEvent(event_t1);
  prismatic_joint->addEvent(event_t2);
  std::cerr << "number of events in joint jp0: " << prismatic_joint->numberOfEvents() << std::endl;

  std::cerr << "sampling" << std::endl;
  prismatic_joint->sample(0);
  std::cerr << "sampled distance (LinkWithJointPrismatic) at t=0s: " << prismatic_joint->sampledPosition() << std::endl;
  prismatic_joint->sample(1.5);
  std::cerr << "sampled distance (LinkWithJointPrismatic) at t=1.5s: " << prismatic_joint->sampledPosition() << std::endl;
  prismatic_joint->sample(0.5);
  std::cerr << "sampled distance (LinkWithJointPrismatic) at t=0.5s: " << prismatic_joint->sampledPosition() << std::endl;

  LinkWithJointRotational* rotational_joint = new LinkWithJointRotational("jr0", Vector3f(0, 0, 1));
  JointEventPtr event_rotational_t0(new JointEvent(0, "jr0", 0));
  JointEventPtr event_rotational_t1(new JointEvent(1, "jr0", 1));

  rotational_joint->addEvent(event_rotational_t0);
  rotational_joint->addEvent(event_rotational_t1);
  std::cerr << "number of events in joint jp0: " << rotational_joint->numberOfEvents() << std::endl;

  rotational_joint->sample(0.5);
  std::cerr << "sampled orientation (LinkWithJointRotational) at t=0.5s: "
            << rotational_joint->sampledPosition().w()
            << " " << rotational_joint->sampledPosition().x()
            << " " << rotational_joint->sampledPosition().y()
            << " " << rotational_joint->sampledPosition().z()
            << std::endl;

  rotational_joint->sample(0.75);
  std::cerr << "sampled orientation (LinkWithJointRotational) at t=0.75s: "
            << rotational_joint->sampledPosition().w()
            << " " << rotational_joint->sampledPosition().x()
            << " " << rotational_joint->sampledPosition().y()
            << " " << rotational_joint->sampledPosition().z()
            << std::endl;

  delete prismatic_joint;
  delete rotational_joint;
  return 0;
}
