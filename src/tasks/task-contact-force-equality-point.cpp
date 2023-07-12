//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include "tsid/tasks/task-contact-force-equality-point.hpp"

namespace tsid {
namespace tasks {

using namespace tsid::math;
using namespace std;

TaskContactForceEqualityPoint::TaskContactForceEqualityPoint(
    const std::string& name, RobotWrapper& robot, const double dt,
    contacts::ContactPoint& contact)
    : TaskContactForce(name, robot),
      m_contact(&contact),
      m_constraint(name, 3, 3),
      m_ref(3, 3),
      m_fext(3, 3) {
  m_forceIntegralError = Vector::Zero(3);
  m_dt = dt;
  m_leak_rate = 0.05;
  m_contact_name = m_contact->name();
  m_use_fext = false; // [Sol] Don't use f_ext by default
}

int TaskContactForceEqualityPoint::dim() const { return 3; }

const Vector& TaskContactForceEqualityPoint::Kp() const { return m_Kp; }
const Vector& TaskContactForceEqualityPoint::Kd() const { return m_Kd; }
const Vector& TaskContactForceEqualityPoint::Ki() const { return m_Ki; }
const double& TaskContactForceEqualityPoint::getLeakRate() const {
  return m_leak_rate;
}

void TaskContactForceEqualityPoint::Kp(ConstRefVector Kp) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kp.size() == 3,
                                 "The size of the Kp vector needs to equal 3");
  m_Kp = Kp;
}

void TaskContactForceEqualityPoint::Kd(ConstRefVector Kd) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kd.size() == 3,
                                 "The size of the Kd vector needs to equal 3");
  m_Kd = Kd;
}

void TaskContactForceEqualityPoint::Ki(ConstRefVector Ki) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Ki.size() == 3,
                                 "The size of the Ki vector needs to equal 3");
  m_Ki = Ki;
}

void TaskContactForceEqualityPoint::setLeakRate(double leak) { m_leak_rate = leak; }

const std::string& TaskContactForceEqualityPoint::getAssociatedContactName() {
  return m_contact_name;
}

const contacts::ContactPoint& TaskContactForceEqualityPoint::getAssociatedContact() {
  return *m_contact;
}

void TaskContactForceEqualityPoint::setAssociatedContact(
    contacts::ContactPoint& contact) {
  m_contact = &contact;
  m_contact_name = m_contact->name();
}

void TaskContactForceEqualityPoint::setReference(TrajectorySample& ref) {
  m_ref = ref;
}

const TaskContactForceEqualityPoint::TrajectorySample&
TaskContactForceEqualityPoint::getReference() const {
  return m_ref;
}

void TaskContactForceEqualityPoint::setExternalForce(TrajectorySample& f_ext) {
  m_fext = f_ext;
}

const TaskContactForceEqualityPoint::TrajectorySample&
TaskContactForceEqualityPoint::getExternalForce() const {
  return m_fext;
}

const ConstraintBase& TaskContactForceEqualityPoint::compute(
    const double t, ConstRefVector q, ConstRefVector v, Data& data,
    const std::vector<std::shared_ptr<ContactLevel> >* contacts) {
  bool contactFound = false;
  if (m_contact_name != "") {
    // look if the associated contact is in the list of contact
    for (auto cl : *contacts) {
      if (m_contact_name == cl->contact.name()) {
        contactFound = true;
        break;
      }
    }
  } else {
    std::cout << "[TaskContactForceEqualityPoint] ERROR: Contact name empty"
              << std::endl;
    return m_constraint;
  }
  if (!contactFound) {
    std::cout << "[TaskContactForceEqualityPoint] ERROR: Contact name not in the "
                 "list of contact in the formulation pb"
              << std::endl;
    return m_constraint;
  }
  return compute(t, q, v, data);
}

const ConstraintBase& TaskContactForceEqualityPoint::compute(const double,
                                                        ConstRefVector,
                                                        ConstRefVector,
                                                        Data& /*data*/) {
  auto& M = m_constraint.matrix();
  M = m_contact->getForceGeneratorMatrix();  // 3x3 for point contact
  //forceGeneratorMatrix is 3x3 identity for ContactPoint

  Vector3 f_ref;

  if(m_use_fext) {
    // Complex force equality with PID tracking of external (measured by sensor) component
    Vector forceError = m_ref.getValue() - m_fext.getValue();
    f_ref =
        m_ref.getValue() + m_Kp.cwiseProduct(forceError) +
        m_Kd.cwiseProduct(m_ref.getDerivative() - m_fext.getDerivative()) +
        m_Ki.cwiseProduct(m_forceIntegralError);
    m_forceIntegralError +=
        (forceError - m_leak_rate * m_forceIntegralError) * m_dt;
  } else { 
    // Simple direct force equality without external (measured by sensor) component
    f_ref = m_ref.getValue();
  }

  m_constraint.vector() = f_ref;

  return m_constraint;
}

const ConstraintBase& TaskContactForceEqualityPoint::getConstraint() const {
  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid
