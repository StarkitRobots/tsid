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

#include <tsid/tasks/task-joint-mimic.hpp>
#include "tsid/robots/robot-wrapper.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    #define DIM 1

    TaskJointMimic::TaskJointMimic( const std::string & name,
                                    RobotWrapper & robot,
                                    const std::string & slaveJointName,
                                    const std::string & masterJointName,
                                    const float direction):
      TaskMotion(name, robot),
      m_ref(robot.nq_actuated(), robot.na()),
      //m_constraint(name, robot.na(), robot.nv())
      //m_constraint(name, 1, 1)
      m_constraint(name, DIM, robot.nv())
      
    {
      assert(m_robot.model().existJoint(slaveJointName));
      assert(m_robot.model().existJoint(masterJointName));

      m_q_master = pinocchio::neutral(robot.model());
      m_q_slave = pinocchio::neutral(robot.model());
      m_Kp.setZero(DIM);
      m_Kd.setZero(DIM);
      m_p_error.resize(DIM);   
      m_v_error.resize(DIM);    
      m_jointIndexSlave = robot.model().getJointId(slaveJointName);
      m_jointIndexMaster  = robot.model().getJointId(masterJointName);  
      m_direction = direction;
      //Vector m = Vector::Ones(robot.na());
      //m[m_jointIndex1+6-2] = 1;
      //m[m_jointIndex2+6-2] = 1;
      //setMask(m);
      Matrix S = Matrix::Zero(DIM, m_robot.nv());
      S(0,m_robot.nv()-m_robot.na()+(m_jointIndexSlave-2)) = 1.0; // Slave joint
      //S(1,m_robot.nv()-m_robot.na()+(m_jointIndexMaster-2)) = 1.0; // Master joint
      m_constraint.setMatrix(S);  
      std::cout << "m_jointIndexSlave=" << m_jointIndexSlave << std::endl;      
      std::cout << "m_jointIndexMaster=" << m_jointIndexMaster << std::endl;
      std::cout << "S(mimic)=" << std::endl << S << std::endl;
    }

    const Vector & TaskJointMimic::mask() const
    {
      return m_mask;
    }

    void TaskJointMimic::mask(const Vector & m)
    {
      std::cerr<<"The method TaskJointMimic::mask is deprecated. Use TaskJointMimic::setMask instead.\n";
      return setMask(m);
    }

    void TaskJointMimic::setMask(ConstRefVector m)
    {
      // assert(m.size()==m_robot.na());
      // m_mask = m;
      // const Vector::Index dim = static_cast<Vector::Index>(m.sum());
      // Matrix S = Matrix::Zero(dim, m_robot.nv());
      // m_activeAxes.resize(dim);
      // unsigned int j=0;
      // for(unsigned int i=0; i<m.size(); i++)
      //   if(m(i)!=0.0)
      //   {
      //     assert(m(i)==1.0);
      //     S(j,m_robot.nv()-m_robot.na()+i) = 1.0;
      //     m_activeAxes(j) = i;
      //     j++;
      //   }
      // m_constraint.resize((unsigned int)dim, m_robot.nv());
      // m_constraint.setMatrix(S);
      // std::cout << "S(mimic)=" << std::endl << S << std::endl;      
    }

    int TaskJointMimic::dim() const
    {
      //return (int)m_mask.sum();
      return DIM;
    }

    const Vector & TaskJointMimic::Kp(){ return m_Kp; }

    const Vector & TaskJointMimic::Kd(){ return m_Kd; }

    void TaskJointMimic::Kp(ConstRefVector Kp)
    {
      //assert(Kp.size()==m_robot.na());
      //assert(Kp.size()==1);
      m_Kp = Kp;
    }

    void TaskJointMimic::Kd(ConstRefVector Kd)
    {
      //assert(Kd.size()==m_robot.na());
      //assert(Kd.size()==1);
      m_Kd = Kd;
    }

    void TaskJointMimic::setReference(const TrajectorySample & ref)
    {
      // assert(ref.getValue().size()==m_robot.nq_actuated());
      // assert(ref.getDerivative().size()==m_robot.na());
      // assert(ref.getSecondDerivative().size()==m_robot.na());
      // m_ref = ref;
    }

    const TrajectorySample & TaskJointMimic::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskJointMimic::getDesiredAcceleration() const
    {
      return m_a_des;
    }

    Vector TaskJointMimic::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv;
    }

    const Vector & TaskJointMimic::position_error() const
    {
      return m_p_error;
    }

    const Vector & TaskJointMimic::velocity_error() const
    {
      return m_v_error;
    }

    const Vector & TaskJointMimic::position() const
    {
      return m_p;
    }

    const Vector & TaskJointMimic::velocity() const
    {
      return m_v;
    }

    const Vector & TaskJointMimic::position_ref() const
    {
      return m_ref.getValue();
    }

    const Vector & TaskJointMimic::velocity_ref() const
    {
      return m_ref.getDerivative();
    }

    const ConstraintBase & TaskJointMimic::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskJointMimic::compute(const double ,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & )
    {
      if(0) {
        m_ref_q_augmented.tail(m_robot.nq_actuated()) = m_ref.getValue();

        // Compute errors
        m_p_error = pinocchio::difference(m_robot.model(), m_ref_q_augmented, q);
        m_p_error = m_p_error.tail(m_robot.na());

        m_v = v.tail(m_robot.na());
        m_v_error = m_v - m_ref.getDerivative();
        m_a_des = - m_Kp.cwiseProduct(m_p_error)
                  - m_Kd.cwiseProduct(m_v_error)
                  + m_ref.getSecondDerivative();

        for(unsigned int i=0; i<m_activeAxes.size(); i++)
          m_constraint.vector()(i) = m_a_des(m_activeAxes(i));
      }

      if(0) {
        //Opposite accelerations test
        // int nun = m_robot.nv()-m_robot.na();
        // m_p_error[1] = q[nun+m_jointIndex1-2] - q[nun+m_jointIndex2-2]; // TODO: 6 is not right for fixed-base robots
        // m_p_error[0] = q[nun+m_jointIndex2-2] - q[nun+m_jointIndex1-2];
        // //std::cout << m_jointIndex1 <<", "<< m_jointIndex2 << std::endl;
        // //std::cout << "TaskJointMimic:" << q << std::endl;

        // m_v_error[1] = v[nun+m_jointIndex1-2] - v[nun+m_jointIndex2-2];
        // m_v_error[0] = v[nun+m_jointIndex2-2] - v[nun+m_jointIndex1-2];
        // m_a_des = - m_Kp.cwiseProduct(m_p_error)
        //           - m_Kd.cwiseProduct(m_v_error);

        // //std::cout << "m_a_des=" << m_a_des.transpose() << std::endl;         
        // m_constraint.vector() = m_a_des;                        
      }

      if(1) {
        // Slave -> master single joint mimic acceleration (slave joint is following the master. Master will not follow the slave)
        int nun = m_robot.nv()-m_robot.na();
        // m_q_master = q;
        // m_q_master[nun+m_jointIndexSlave-1] = m_direction * q[nun+m_jointIndexMaster-1];
        // m_p_error_all = pinocchio::difference(m_robot.model(), m_q_master, q);
        // //m_p_error[0] = m_p_error_all[nun+m_jointIndexSlave-2];
        m_p_error[0] = q[nun+m_jointIndexSlave-1] - m_direction * q[nun+m_jointIndexMaster-1];
        m_v_error[0] = v[nun+m_jointIndexSlave-2] - m_direction * v[nun+m_jointIndexMaster-2];
        
        m_a_des = - m_Kp.cwiseProduct(m_p_error)
                  - m_Kd.cwiseProduct(m_v_error);
        
        //std::cout << "m_p_error=" << m_p_error << std::endl;
        //std::cout << m_direction << ": q_slave=" << q[nun+m_jointIndexSlave-1] << ", q_master=" << q[nun+m_jointIndexMaster-1] <<", m_a_des=" << m_a_des << std::endl;         
        //std::cout << q << std::endl;
        m_constraint.vector() = m_a_des;        
      }

      if(0) {
        // Same rank joint mimic - both joints will try to reach the position computed as middle point between them 
        int nun = m_robot.nv()-m_robot.na();
        double p_average = (q[nun+m_jointIndexSlave-1] + q[nun+m_jointIndexMaster-1]) / 2;
        double v_average = (v[nun+m_jointIndexSlave-2] + v[nun+m_jointIndexMaster-2]) / 2;
        m_p_error[0] = q[nun+m_jointIndexSlave-1] - p_average;
        m_p_error[1] = q[nun+m_jointIndexMaster-1] - p_average;
        m_v_error[0] = v[nun+m_jointIndexSlave-2] - v_average;
        m_v_error[1] = v[nun+m_jointIndexMaster-2] - v_average;
        
        m_a_des = - m_Kp.cwiseProduct(m_p_error)
                  - m_Kd.cwiseProduct(m_v_error);
        
        //std::cout << "m_a_des=" << m_a_des.transpose() << std::endl;
        //std::cout << m_direction << ": q_slave=" << q[nun+m_jointIndexSlave-1] << ", q_master=" << q[nun+m_jointIndexMaster-1] <<", m_a_des=" << m_a_des << std::endl;         
        //std::cout << q << std::endl;
        m_constraint.vector() = m_a_des;        
      }      
      
      return m_constraint;
    }

  }
}
