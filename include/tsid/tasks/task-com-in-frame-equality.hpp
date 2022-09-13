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

#ifndef __invdyn_task_com_in_frame_equality_hpp__
#define __invdyn_task_com_in_frame_equality_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tsid
{
  namespace tasks
  {

    class TaskComInFrameEquality : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Vector3 Vector3;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data::Matrix6x Matrix6x;
      typedef pinocchio::Motion Motion;
      typedef pinocchio::SE3 SE3;      

      TaskComInFrameEquality(const std::string & name,
                      RobotWrapper & robot, const std::string & frameName);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(const TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const Vector & getDesiredAcceleration() const;
      Vector getAcceleration(ConstRefVector dv) const;
      virtual void setMask(math::ConstRefVector mask);

      const Vector & position_error() const;
      const Vector & velocity_error() const;
      const Vector & position() const;
      const Vector & velocity() const;
      const Vector & position_ref() const;
      const Vector & velocity_ref() const;

      const Vector3 & Kp();
      const Vector3 & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      Vector3 m_Kp;
      Vector3 m_Kd;
      Vector3 m_p_error, m_v_error;
      Vector m_p_error_masked_vec, m_v_error_masked_vec;
      Vector3 m_a_des;
      Vector m_a_des_vec, m_a_des_masked;
      Vector3 m_drift;
      Vector m_drift_masked;
      Vector m_p_com, m_v_com;
      Vector m_p_error_vec, m_v_error_vec;
      TrajectorySample m_ref;
      ConstraintEquality m_constraint;
      std::string m_frame_name;
      Index m_frame_id;
      Matrix6x m_Jframe;
      Matrix6x m_Jframe_rotated;   
      SE3 m_wMl;   
    };

  }
}

#endif // ifndef __invdyn_task_com_equality_hpp__
