//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
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

#include "tsid/math/utils.hpp"
#include "tsid/contacts/contact-two-frames-mirror.hpp"

#include <pinocchio/spatial/skew.hpp>

using namespace tsid;
using namespace contacts;
using namespace math;
using namespace trajectories;
using namespace tasks;

ContactTwoFramesMirror::ContactTwoFramesMirror(const std::string & name,
                     RobotWrapper & robot,
                     const std::string & frameName1,
                     const std::string & frameName2,
                     const std::string & frameNameMiddle,
                     ConstRefVector contactNormal,
                     const double frictionCoefficient,
                     const double minNormalForce,
                     const double maxNormalForce):
  ContactBase(name, robot),
  m_motionTask(name, robot, frameName1, frameName2, frameNameMiddle), // Actual motion task with type TaskTwoFramesMirror
  m_dummyMotionTask(name, robot, frameName1), // Only to fit the ContactBase class returns, type TaskSE3Equality, seems to be needed only by TaskCopEquality
  m_forceInequality(name, 6, 6),
  m_forceRegTask(name, 6, 6),
  //m_contactNormal(contactNormal),
  m_mu(frictionCoefficient),
  m_fMin(minNormalForce),
  m_fMax(maxNormalForce)
{
  m_weightForceRegTask << 1, 1, 1, 1, 1, 1;
  m_forceGenMat.resize(6, 6);
  m_fRef = Vector6::Zero();

  updateForceGeneratorMatrix();
  updateForceInequalityConstraints();
  updateForceRegularizationTask();

  math::Vector motion_mask(6);  
  motion_mask << 0., 0., 0., 1., 1., 1.; 
  m_motionTask.setMask(motion_mask);
}

void ContactTwoFramesMirror::useLocalFrame(bool local_frame)
{
  m_dummyMotionTask.useLocalFrame(local_frame);
}

void ContactTwoFramesMirror::updateForceInequalityConstraints()
{
  // Force "gluing" two frames together can be arbitrary in sign/direction
  Matrix B = Matrix::Identity(6, 6);
  //Vector lb = -1e10*Vector::Ones(3); 
  //Vector ub = 1e10*Vector::Ones(3);

  Vector lb = m_fMin*Vector::Ones(6); 
  Vector ub = m_fMax*Vector::Ones(6);

  m_forceInequality.setMatrix(B);
  m_forceInequality.setLowerBound(lb);
  m_forceInequality.setUpperBound(ub);
}

double ContactTwoFramesMirror::getNormalForce(ConstRefVector f) const
{
  //assert(f.size()==n_force());
  //return m_contactNormal.dot(f);
}

const Matrix3x & ContactTwoFramesMirror::getContactPoints() const
{
  //return m_contactPoints;
}

void ContactTwoFramesMirror::setRegularizationTaskWeightVector(ConstRefVector & w)
{
  m_weightForceRegTask = w;
  updateForceRegularizationTask();
}

void ContactTwoFramesMirror::updateForceRegularizationTask()
{
  typedef Eigen::Matrix<double,6,6> Matrix6;
  Matrix6 A = Matrix6::Zero();
  A.diagonal() = m_weightForceRegTask;
  m_forceRegTask.setMatrix(A);
  m_forceRegTask.setVector(A*m_fRef);
}

void ContactTwoFramesMirror:: updateForceGeneratorMatrix()
{
  m_forceGenMat.setIdentity();
}

//unsigned int ContactTwoFramesMirror::n_motion() const { return m_motionTask.dim(); }
unsigned int ContactTwoFramesMirror::n_motion() const { return 3; }
unsigned int ContactTwoFramesMirror::n_force() const { return 3; }

const Vector & ContactTwoFramesMirror::Kp()
{
  return m_Kp;
}

const Vector & ContactTwoFramesMirror::Kd()
{
  return m_Kd;
}

void ContactTwoFramesMirror::Kp(ConstRefVector Kp)
{
  m_motionTask.Kp(Kp);
}

void ContactTwoFramesMirror::Kd(ConstRefVector Kd)
{
  m_motionTask.Kd(Kd);
}

bool ContactTwoFramesMirror::setContactNormal(ConstRefVector contactNormal)
{
  /*assert(contactNormal.size()==3);
  if(contactNormal.size()!=3)
    return false;
  m_contactNormal = contactNormal;
  updateForceInequalityConstraints();*/
  return true;
}

bool ContactTwoFramesMirror::setFrictionCoefficient(const double frictionCoefficient)
{
  /*
  assert(frictionCoefficient>0.0);
  if(frictionCoefficient<=0.0)
    return false;
  m_mu = frictionCoefficient;
  updateForceInequalityConstraints();
  */
  return true;
}

bool ContactTwoFramesMirror::setMinNormalForce(const double minNormalForce)
{
  /*
  assert(minNormalForce>0.0 && minNormalForce<=m_fMax);
  if(minNormalForce<=0.0 || minNormalForce>m_fMax)
    return false;
  m_fMin = minNormalForce;
  Vector & lb = m_forceInequality.lowerBound();
  lb(lb.size()-1) = m_fMin;
  */
  return true;
}

bool ContactTwoFramesMirror::setMaxNormalForce(const double maxNormalForce)
{
  /*
  assert(maxNormalForce>=m_fMin);
  if(maxNormalForce<m_fMin)
    return false;
  m_fMax = maxNormalForce;
  Vector & ub = m_forceInequality.upperBound();
  ub(ub.size()-1) = m_fMax;
  */
  return true;
}

void ContactTwoFramesMirror::setForceReference(ConstRefVector & f_ref)
{
  m_fRef = f_ref;
  updateForceRegularizationTask();
}

void ContactTwoFramesMirror::setReference(const SE3 & ref)
{
  m_dummyMotionTask.setReference(ref);
}

const ConstraintBase & ContactTwoFramesMirror::computeMotionTask(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
{
  return m_motionTask.compute(t, q, v, data);
}

const ConstraintInequality & ContactTwoFramesMirror::computeForceTask(const double,
                                                         ConstRefVector ,
                                                         ConstRefVector ,
                                                         const Data & )
{
  return m_forceInequality;
}

const Matrix & ContactTwoFramesMirror::getForceGeneratorMatrix()
{
  return m_forceGenMat;
}

const ConstraintEquality & ContactTwoFramesMirror::
computeForceRegularizationTask(const double ,
			       ConstRefVector ,
			       ConstRefVector ,
			       const Data & )
{
  return m_forceRegTask;
}

double ContactTwoFramesMirror::getMinNormalForce() const { return m_fMin; }
double ContactTwoFramesMirror::getMaxNormalForce() const { return m_fMax; }

const TaskSE3Equality & ContactTwoFramesMirror::getMotionTask() const { 
  std::cout << "Warning! Returning emtpy motion task from ContactTwoFramesMirror::m_dummyMotionTask" << std::endl;
  return m_dummyMotionTask; 
}

const ConstraintBase & ContactTwoFramesMirror::getMotionConstraint() const { 
  return m_motionTask.getConstraint(); 
}

const ConstraintInequality & ContactTwoFramesMirror::getForceConstraint() const { return m_forceInequality; }

const ConstraintEquality & ContactTwoFramesMirror::getForceRegularizationTask() const { return m_forceRegTask; }
