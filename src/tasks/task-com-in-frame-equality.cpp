//
// Copyright (c) 2017-2020 CNRS, Inria
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

#include "tsid/tasks/task-com-in-frame-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskComInFrameEquality::TaskComInFrameEquality(const std::string & name,
                                     RobotWrapper & robot, const std::string & frameName):
      TaskMotion(name, robot),
      m_constraint(name, 3, robot.nv()),
      m_frame_name(frameName)
    {
      assert(m_robot.model().existFrame(frameName));
      m_frame_id = m_robot.model().getFrameId(frameName);
      m_Kp.setZero(3);
      m_Kd.setZero(3);
      m_p_error_vec.setZero(3);
      m_v_error_vec.setZero(3);
      m_p_com.setZero(3);
      m_v_com.setZero(3);
      m_a_des_vec.setZero(3);
      m_ref.resize(3);
      m_mask.resize(3);
      m_mask.fill(1.);
      setMask(m_mask);
    }


    void TaskComInFrameEquality::setMask(math::ConstRefVector mask)
    {
      assert(mask.size() == 3);
      TaskMotion::setMask(mask);
      int n = dim();
      m_constraint.resize(n, m_robot.nv());
      m_p_error_masked_vec.resize(n);
      m_v_error_masked_vec.resize(n);
      m_drift_masked.resize(n);
      m_a_des_masked.resize(n);
    }

    int TaskComInFrameEquality::dim() const
    {
      return int(m_mask.sum());
    }

    const Vector3 & TaskComInFrameEquality::Kp(){ return m_Kp; }

    const Vector3 & TaskComInFrameEquality::Kd(){ return m_Kd; }

    void TaskComInFrameEquality::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==3);
      m_Kp = Kp;
    }

    void TaskComInFrameEquality::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==3);
      m_Kd = Kd;
    }

    void TaskComInFrameEquality::setReference(const TrajectorySample & ref)
    {
      m_ref = ref;
    }

    const TrajectorySample & TaskComInFrameEquality::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskComInFrameEquality::getDesiredAcceleration() const
    {
      return m_a_des_masked;
    }

    Vector TaskComInFrameEquality::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv - m_drift_masked;
    }

    const Vector & TaskComInFrameEquality::position_error() const
    {
      return m_p_error_masked_vec;
    }

    const Vector & TaskComInFrameEquality::velocity_error() const
    {
      return m_v_error_masked_vec;
    }

    const Vector & TaskComInFrameEquality::position() const
    {
      return m_p_com;
    }

    const Vector & TaskComInFrameEquality::velocity() const
    {
      return m_v_com;
    }

    const Vector & TaskComInFrameEquality::position_ref() const
    {
      return m_ref.getValue();
    }

    const Vector & TaskComInFrameEquality::velocity_ref() const
    {
      return m_ref.getDerivative();
    }

    const ConstraintBase & TaskComInFrameEquality::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskComInFrameEquality::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector ,
                                                    Data & data)
    {


      SE3 oMi;
      Motion v_frame_local, v_frame_world;
      Motion drift_frame_local;
      Motion drift_frame_world;
      Vector3 drift_com;
      m_robot.com(data, m_p_com, m_v_com, drift_com);
      m_robot.framePosition(data, m_frame_id, oMi);
      m_robot.frameVelocity(data, m_frame_id, v_frame_local);
      m_robot.frameClassicAcceleration(data, m_frame_id, drift_frame_local); 
      m_robot.frameJacobianLocal(data, m_frame_id, m_Jframe);     

      // Transformation from local to world of frame
      m_wMl.rotation(oMi.rotation());

      v_frame_world =  m_wMl.act(v_frame_local);  // vel in world-oriented frame

      // Compute errors
      m_p_error = m_p_com - oMi.translation() - m_ref.getValue();
      m_v_error = m_v_com - v_frame_world.linear() - m_ref.getDerivative();

      // Compute drift
      drift_frame_world = m_wMl.act(drift_frame_local);
      m_drift = drift_com - drift_frame_world.linear();
      
      // Compute desired acceleration
      m_a_des = - m_Kp.cwiseProduct(m_p_error)
                - m_Kd.cwiseProduct(m_v_error)
                + m_ref.getSecondDerivative();

      m_p_error_vec = m_p_error;
      m_v_error_vec = m_v_error;
      m_a_des_vec = m_a_des;
#ifndef NDEBUG
//      std::cout<<m_name<<" errors: "<<m_p_error.norm()<<" "
//        <<m_v_error.norm()<<std::endl;
#endif

      // Get CoM jacobian
      const Matrix3x & Jcom = m_robot.Jcom(data);

      // Get frame jacobian in world frame
      m_Jframe_rotated.noalias() = m_wMl.toActionMatrix() * m_Jframe; // Use an explicit temporary `m_J_rotated` here to avoid allocations.
      m_Jframe = m_Jframe_rotated;      

      int idx = 0;
      for (int i = 0; i < 3; i++) {
        if (m_mask(i) != 1.) continue;

        m_constraint.matrix().row(idx) = Jcom.row(i) - m_Jframe.row(i);
        m_constraint.vector().row(idx) = (m_a_des - m_drift).row(i);

        m_a_des_masked(idx)            = m_a_des(i);
        m_drift_masked(idx)            = m_drift(i);
        m_p_error_masked_vec(idx)      = m_p_error_vec(i);
        m_v_error_masked_vec(idx)      = m_v_error_vec(i);

        idx += 1;
      }

      return m_constraint;
    }

  }
}
