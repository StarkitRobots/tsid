//
// Copyright (c) 2017-2020 CNRS, NYU, MPI Tübingen, Inria
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
#include "tsid/tasks/task-frames-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace tasks
  {
    using namespace std;
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskFramesEquality::TaskFramesEquality(const std::string & name,
                                     RobotWrapper & robot,
                                     const std::string & frameName1,
                                     const std::string & frameName2):
      TaskMotion(name, robot),
      m_frame_name1(frameName1),
      m_frame_name2(frameName2),
      m_constraint(name, 6, robot.nv()),
      m_ref(12, 6)
    {
      assert(m_robot.model().existFrame(frameName1));
      assert(m_robot.model().existFrame(frameName2));
      m_frame_id1 = m_robot.model().getFrameId(frameName1);
      m_frame_id2 = m_robot.model().getFrameId(frameName2);

      m_v_ref.setZero();
      m_a_ref.setZero();
      m_M_ref.setIdentity();
      m_wMl.setIdentity();
      m_p_error_vec.setZero(6);
      m_v_error_vec.setZero(6);
      m_p.resize(12);
      m_v.resize(6);
      m_p_ref.resize(12);
      m_v_ref_vec.resize(6);
      m_Kp.setZero(6);
      m_Kd.setZero(6);
      m_a_des.setZero(6);
      m_J1.setZero(6, robot.nv());
      m_J2.setZero(6, robot.nv());
      //m_J_rotated.setZero(6, robot.nv());

      m_mask.resize(6);
      m_mask.fill(1.);
      setMask(m_mask);

      m_local_frame = true;
    }

    void TaskFramesEquality::setMask(math::ConstRefVector mask)
    {
      TaskMotion::setMask(mask);
      int n = dim();
      m_constraint.resize(n, (unsigned int)m_J1.cols());
      m_p_error_masked_vec.resize(n);
      m_v_error_masked_vec.resize(n);
      m_drift_masked.resize(n);
      m_a_des_masked.resize(n);
    }

    int TaskFramesEquality::dim() const
    {
      return (int)m_mask.sum();
    }

    const Vector & TaskFramesEquality::Kp() const { return m_Kp; }

    const Vector & TaskFramesEquality::Kd() const { return m_Kd; }

    void TaskFramesEquality::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==6);
      m_Kp = Kp;
    }

    void TaskFramesEquality::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==6);
      m_Kd = Kd;
    }

    void TaskFramesEquality::setReference(TrajectorySample & ref)
    {
      m_ref = ref;
TSID_DISABLE_WARNING_PUSH
TSID_DISABLE_WARNING_DEPRECATED
      assert(ref.pos.size() == 12);
      m_M_ref.translation( ref.pos.head<3>());
      m_M_ref.rotation(MapMatrix3(&ref.pos(3), 3, 3));
TSID_DISABLE_WARNING_POP
      m_v_ref = Motion(ref.getDerivative());
      m_a_ref = Motion(ref.getSecondDerivative());
    }

    void TaskFramesEquality::setReference(const SE3 & ref)
    {
      TrajectorySample s(12, 6);
TSID_DISABLE_WARNING_PUSH
TSID_DISABLE_WARNING_DEPRECATED
      tsid::math::SE3ToVector(ref, s.pos);
TSID_DISABLE_WARNING_POP
      setReference(s);
    }


    const TrajectorySample & TaskFramesEquality::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskFramesEquality::position_error() const
    {
      return m_p_error_masked_vec;
    }

    const Vector & TaskFramesEquality::velocity_error() const
    {
      return m_v_error_masked_vec;
    }
  
    /*const Vector & TaskFramesEquality::position() const
    {
      return m_p;
    }

    const Vector & TaskFramesEquality::velocity() const
    {
      return m_v;
    }

    const Vector & TaskFramesEquality::position_ref() const
    {
      return m_p_ref;
    }

    const Vector & TaskFramesEquality::velocity_ref() const
    {
      return m_v_ref_vec;
    }*/

    const Vector & TaskFramesEquality::getDesiredAcceleration() const
    {
      return m_a_des_masked;
    }

    Vector TaskFramesEquality::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv + m_drift_masked;
    }

    Index TaskFramesEquality::frame_id1() const
    {
      return m_frame_id1;
    }

    Index TaskFramesEquality::frame_id2() const
    {
      return m_frame_id2;
    }    

    const ConstraintBase & TaskFramesEquality::getConstraint() const
    {
      return m_constraint;
    }

    void TaskFramesEquality::useLocalFrame(bool local_frame)
    {
      m_local_frame = local_frame;
    }

    const ConstraintBase & TaskFramesEquality::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector ,
                                                    Data & data)
    {
      SE3 oMi1, oMi2;
      Motion v_frame1, v_frame2;
      Motion m_drift1, m_drift2;
      m_robot.framePosition(data, m_frame_id1, oMi1);
      m_robot.framePosition(data, m_frame_id2, oMi2);
      m_robot.frameVelocity(data, m_frame_id1, v_frame1);
      m_robot.frameVelocity(data, m_frame_id2, v_frame2);      
      m_robot.frameClassicAcceleration(data, m_frame_id1, m_drift1);
      m_robot.frameClassicAcceleration(data, m_frame_id2, m_drift2);

      // @todo Since Jacobian computation is cheaper in world frame
      // we could do all computations in world frame
      m_robot.frameJacobianLocal(data, m_frame_id1, m_J1);
      m_robot.frameJacobianLocal(data, m_frame_id2, m_J2);

      // ==== [Sol]: Exchanging ref to frame2 ====
      //errorInSE3(oMi, m_M_ref, m_p_error);          // pos err in local frame
      errorInSE3(oMi1, oMi2, m_p_error);          // pos err in local frame

      // Always working in local frame
      m_p_error_vec = m_p_error.toVector();
      //m_v_error =  m_wMl.actInv(m_v_ref) - v_frame;  // vel err in local frame
      m_v_error =  v_frame2 - v_frame1;  // vel err in local frame

      // desired acc in local frame
      m_a_des = m_Kp.cwiseProduct(m_p_error_vec)
                + m_Kd.cwiseProduct(m_v_error.toVector());
                //+ m_wMl.actInv(m_a_ref).toVector();

      m_v_error_vec = m_v_error.toVector();
      //m_v_ref_vec = m_v_ref.toVector();
      //m_v = v_frame.toVector();

      m_drift = m_drift1 - m_drift2; // ACtually dunno what to do with drift

      int idx = 0;
      for (int i = 0; i < 6; i++) {
        if (m_mask(i) != 1.) continue;

        m_constraint.matrix().row(idx) = m_J1.row(i) - m_J2.row(i);
        m_constraint.vector().row(idx) = (m_a_des - m_drift.toVector()).row(i);
        m_a_des_masked(idx)            = m_a_des(i);
        m_drift_masked(idx)            = m_drift.toVector()(i);
        m_p_error_masked_vec(idx)      = m_p_error_vec(i);
        m_v_error_masked_vec(idx)      = m_v_error_vec(i);

        idx += 1;
      }

      return m_constraint;
    }
  }
}