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
#include "tsid/tasks/task-two-frames-mirror.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace tasks
  {
    using namespace std;
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskTwoFramesMirror::TaskTwoFramesMirror(const std::string & name,
                                     RobotWrapper & robot,
                                     const std::string & frameName1,
                                     const std::string & frameName2,
                                     const std::string & frameNameMiddle):
      TaskMotion(name, robot),
      m_frame_name1(frameName1),
      m_frame_name2(frameName2),
      m_frame_name_middle(frameNameMiddle),
      m_constraint(name, 6, robot.nv())
    {
      assert(m_robot.model().existFrame(frameName1));
      assert(m_robot.model().existFrame(frameName2));
      assert(m_robot.model().existFrame(frameNameMiddle));
      m_frame_id1 = m_robot.model().getFrameId(frameName1);
      m_frame_id2 = m_robot.model().getFrameId(frameName2);
      m_frame_id_middle = m_robot.model().getFrameId(frameNameMiddle);

      m_v_ref.setZero();
      m_a_ref.setZero();
      m_wMl1.setIdentity();
      m_wMl2.setIdentity();
      m_wMlM.setIdentity();
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
      m_JM.setZero(6, robot.nv());
      m_J1_rotated.setZero(6, robot.nv());
      m_J2_rotated.setZero(6, robot.nv());
      m_JM_rotated.setZero(6, robot.nv());

      m_mask.resize(6);
      m_mask.fill(1.);
      setMask(m_mask);

    }

    void TaskTwoFramesMirror::setMask(math::ConstRefVector mask)
    {
      TaskMotion::setMask(mask);
      int n = dim();
      m_constraint.resize(n, (unsigned int)m_J1.cols());
      m_p_error_masked_vec.resize(n);
      m_v_error_masked_vec.resize(n);
      m_drift_masked.resize(n);
      m_a_des_masked.resize(n);
    }

    int TaskTwoFramesMirror::dim() const
    {
      return (int)m_mask.sum();
    }

    const Vector & TaskTwoFramesMirror::Kp() const { return m_Kp; }

    const Vector & TaskTwoFramesMirror::Kd() const { return m_Kd; }

    void TaskTwoFramesMirror::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==6);
      m_Kp = Kp;
    }

    void TaskTwoFramesMirror::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==6);
      m_Kd = Kd;
    }

    const Vector & TaskTwoFramesMirror::position_error() const
    {
      return m_p_error_masked_vec;
    }

    const Vector & TaskTwoFramesMirror::velocity_error() const
    {
      return m_v_error_masked_vec;
    }

    const Vector & TaskTwoFramesMirror::getDesiredAcceleration() const
    {
      return m_a_des_masked;
    }

    Vector TaskTwoFramesMirror::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv + m_drift_masked;
    }

    Index TaskTwoFramesMirror::frame_id1() const
    {
      return m_frame_id1;
    }

    Index TaskTwoFramesMirror::frame_id2() const
    {
      return m_frame_id2;
    }    

    const ConstraintBase & TaskTwoFramesMirror::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskTwoFramesMirror::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector ,
                                                    Data & data)
    {

      // Frame f1 and f2 mirrored by fm: f1(q) - fm(q) = fm(q) - f2(q)
      //                                 f1(q) + f2(q) - 2*fm(q) = 0
      //                                df(q) = J*dq
      //                                J1*dq + J2*dq - 2Jm*dq = 0
      //                                J1*ddq + J1dot*dq + J2*ddq + J2dot*dq - 2Jmddq - 2Jmdot*dq = 0
      //                                (J1 + J2 - 2JM) ddq = -J1dot*dq - J2dot*dq + 2JMdot*dq
      //                                                       drift1      drift2      driftM
      //
      // Calculating task with matrix form of: [J1 + J2 - 2*JM   0   0] y = [-J1dot*v - J2dot*v + 2*JMdot*v]

      SE3 oMi1, oMi2, oMiM;
      SE3 M_1toM, M_Mto2;
      Motion v_frame1, v_frame2, v_frameM;
      Motion m_drift1, m_drift2, m_driftM;
      m_robot.framePosition(data, m_frame_id1, oMi1);
      m_robot.framePosition(data, m_frame_id2, oMi2);
      m_robot.framePosition(data, m_frame_id_middle, oMiM);
      m_robot.frameVelocity(data, m_frame_id1, v_frame1);
      m_robot.frameVelocity(data, m_frame_id2, v_frame2);
      m_robot.frameVelocity(data, m_frame_id_middle, v_frameM);
      m_robot.frameClassicAcceleration(data, m_frame_id1, m_drift1);
      m_robot.frameClassicAcceleration(data, m_frame_id2, m_drift2);
      m_robot.frameClassicAcceleration(data, m_frame_id_middle, m_driftM);

      // Transformations from local to world
      m_wMl1.rotation(oMi1.rotation());   
      m_wMl2.rotation(oMi2.rotation());
      m_wMlM.rotation(oMiM.rotation());

      m_robot.frameJacobianLocal(data, m_frame_id1, m_J1);
      m_robot.frameJacobianLocal(data, m_frame_id2, m_J2);
      m_robot.frameJacobianLocal(data, m_frame_id_middle, m_JM);

      M_1toM = oMiM.inverse() * oMi1;
      M_Mto2 = oMi2.inverse() * oMiM;
      errorInSE3(M_Mto2, M_1toM, m_p_error);

      m_p_error_vec = m_p_error.toVector();

      m_v_error = - m_wMl2.act(v_frame2) -  m_wMl1.act(v_frame1) + 2.0 * m_wMlM.act(v_frameM);  // vel err in world frame

      // desired acc in world frame
      m_a_des = m_Kp.cwiseProduct(m_p_error_vec)
                + m_Kd.cwiseProduct(m_v_error.toVector());

      m_v_error_vec = m_v_error.toVector();

      m_drift = m_wMl1.act(m_drift1) + m_wMl2.act(m_drift2) - 2.0 * m_wMlM.act(m_driftM);

      m_J1_rotated.noalias() = m_wMl1.toActionMatrix() * m_J1;
      m_J1 = m_J1_rotated;      

      m_J2_rotated.noalias() = m_wMl2.toActionMatrix() * m_J2;
      m_J2 = m_J2_rotated;    

      m_JM_rotated.noalias() = m_wMlM.toActionMatrix() * m_JM;
      m_JM = m_JM_rotated;          

      int idx = 0;
      for (int i = 0; i < 6; i++) {
        if (m_mask(i) != 1.) continue;

        m_constraint.matrix().row(idx) = m_J1.row(i) + m_J2.row(i) - 2.0*m_JM.row(i);
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
