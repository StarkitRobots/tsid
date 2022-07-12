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
#include "tsid/contacts/contact-two-frames.hpp"

#include <pinocchio/spatial/skew.hpp>

using namespace tsid;
using namespace contacts;
using namespace math;
using namespace trajectories;
using namespace tasks;

ContactTwoFrames::ContactTwoFrames(const std::string & name,
                     RobotWrapper & robot,
                     const std::string & frameName1,
                     const std::string & frameName2,
                     ConstRefVector contactNormal,
                     const double frictionCoefficient,
                     const double minNormalForce,
                     const double maxNormalForce):
  ContactBase(name, robot),
  m_motionTask(name, robot, frameName1, frameName2), // Actual motion task with type TaskFramesEquality
  m_dummyMotionTask(name, robot, frameName1), // Only to fit the ContactBase class returns, type TaskSE3Equality
  m_forceInequality(name, 3, 3),
  m_forceRegTask(name, 3, 3),
  m_contactNormal(contactNormal),
  m_mu(frictionCoefficient),
  m_fMin(minNormalForce),
  m_fMax(maxNormalForce)
{
  m_weightForceRegTask << 1, 1, 1;
  m_forceGenMat.resize(3, 3);
  m_fRef = Vector3::Zero();
  m_contactPoints.resize(3,1);
  m_contactPoints.setZero();
  updateForceGeneratorMatrix();
  updateForceInequalityConstraints();
  updateForceRegularizationTask();

  math::Vector motion_mask(6);
  motion_mask << 1., 1., 1., 0., 0., 0.;
  m_motionTask.setMask(motion_mask);
}

void ContactTwoFrames::useLocalFrame(bool local_frame)
{
  m_dummyMotionTask.useLocalFrame(local_frame);
}

void ContactTwoFrames::updateForceInequalityConstraints()
{
  /*Vector3 t1, t2;
  const int n_in = 4*1 + 1;
  const int n_var = 3*1;
  Matrix B = Matrix::Zero(n_in, n_var);
  Vector lb = -1e10*Vector::Ones(n_in);
  Vector ub =  Vector::Zero(n_in);
  t1 = m_contactNormal.cross(Vector3::UnitX());
  if(t1.norm()<1e-5)
    t1 = m_contactNormal.cross(Vector3::UnitY());
  t2 = m_contactNormal.cross(t1);
  t1.normalize();
  t2.normalize();

  B.block<1,3>(0,0) = (-t1 - m_mu*m_contactNormal).transpose();
  B.block<1,3>(1,0) = (t1 - m_mu*m_contactNormal).transpose();
  B.block<1,3>(2,0) = (-t2 - m_mu*m_contactNormal).transpose();
  B.block<1,3>(3,0) = (t2 - m_mu*m_contactNormal).transpose();

  B.block<1,3>(n_in-1,0) = m_contactNormal.transpose();
  ub(n_in-1)    = m_fMax;
  lb(n_in-1)    = m_fMin;*/

  Matrix B = Matrix::Identity(3, 3);
  Vector lb = -100*Vector::Ones(3); 
  Vector ub = 100*Vector::Ones(3);

  m_forceInequality.setMatrix(B);
  m_forceInequality.setLowerBound(lb);
  m_forceInequality.setUpperBound(ub);
}

double ContactTwoFrames::getNormalForce(ConstRefVector f) const
{
  assert(f.size()==n_force());
  return m_contactNormal.dot(f);
}

const Matrix3x & ContactTwoFrames::getContactPoints() const
{
  return m_contactPoints;
}

void ContactTwoFrames::setRegularizationTaskWeightVector(ConstRefVector & w)
{
  //m_weightForceRegTask = w;
  updateForceRegularizationTask();
}

void ContactTwoFrames::updateForceRegularizationTask()
{
  typedef Eigen::Matrix<double,3,3> Matrix3;
  Matrix3 A = Matrix3::Zero();
  A.diagonal() = m_weightForceRegTask;
  m_forceRegTask.setMatrix(A);
  m_forceRegTask.setVector(A*m_fRef);
}

void ContactTwoFrames:: updateForceGeneratorMatrix()
{
  m_forceGenMat.setIdentity();
}

unsigned int ContactTwoFrames::n_motion() const { return m_motionTask.dim(); }
unsigned int ContactTwoFrames::n_force() const { return 3; }

const Vector & ContactTwoFrames::Kp()
{
  m_Kp3 = m_motionTask.Kp().head<3>();
  return m_Kp3;
}

const Vector & ContactTwoFrames::Kd()
{
  m_Kd3 = m_motionTask.Kd().head<3>();
  return m_Kd3;
}

void ContactTwoFrames::Kp(ConstRefVector Kp)
{
  assert(Kp.size()==3);
  Vector6 Kp6;
  Kp6.head<3>() = Kp;
  m_motionTask.Kp(Kp6);
}

void ContactTwoFrames::Kd(ConstRefVector Kd)
{
  assert(Kd.size()==3);
  Vector6 Kd6;
  Kd6.head<3>() = Kd;
  m_motionTask.Kd(Kd6);
}

bool ContactTwoFrames::setContactNormal(ConstRefVector contactNormal)
{
  /*assert(contactNormal.size()==3);
  if(contactNormal.size()!=3)
    return false;
  m_contactNormal = contactNormal;
  updateForceInequalityConstraints();*/
  return true;
}

bool ContactTwoFrames::setFrictionCoefficient(const double frictionCoefficient)
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

bool ContactTwoFrames::setMinNormalForce(const double minNormalForce)
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

bool ContactTwoFrames::setMaxNormalForce(const double maxNormalForce)
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

void ContactTwoFrames::setForceReference(ConstRefVector & f_ref)
{
  m_fRef = f_ref;
  updateForceRegularizationTask();
}

void ContactTwoFrames::setReference(const SE3 & ref)
{
  m_dummyMotionTask.setReference(ref);
}

const ConstraintBase & ContactTwoFrames::computeMotionTask(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
{
  return m_motionTask.compute(t, q, v, data);
}

const ConstraintInequality & ContactTwoFrames::computeForceTask(const double,
                                                         ConstRefVector ,
                                                         ConstRefVector ,
                                                         const Data & )
{
  return m_forceInequality;
}

const Matrix & ContactTwoFrames::getForceGeneratorMatrix()
{
  return m_forceGenMat;
}

const ConstraintEquality & ContactTwoFrames::
computeForceRegularizationTask(const double ,
			       ConstRefVector ,
			       ConstRefVector ,
			       const Data & )
{
  return m_forceRegTask;
}

double ContactTwoFrames::getMinNormalForce() const { return m_fMin; }
double ContactTwoFrames::getMaxNormalForce() const { return m_fMax; }

const TaskSE3Equality & ContactTwoFrames::getMotionTask() const { 
  std::cout << "Returning ContactTwoFrames::m_dummyMotionTask" << std::endl;
  return m_dummyMotionTask; 
}

const ConstraintBase & ContactTwoFrames::getMotionConstraint() const { 
  std::cout << "Returning ContactTwoFrames::getMotionConstraint" << std::endl;
  return m_motionTask.getConstraint(); 
}

const ConstraintInequality & ContactTwoFrames::getForceConstraint() const { return m_forceInequality; }

const ConstraintEquality & ContactTwoFrames::getForceRegularizationTask() const { return m_forceRegTask; }
