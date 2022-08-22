//
// Copyright (c) 2018 CNRS
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

#ifndef __tsid_python_task_joint_hpp__
#define __tsid_python_task_joint_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-joint-mimic.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename TaskMimic>
    struct TaskJointMimicPythonVisitor
    : public boost::python::def_visitor< TaskJointMimicPythonVisitor<TaskMimic> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, const std::string &, const std::string &, const float &> ((bp::arg("name"), bp::arg("robot"), bp::arg("slaveJpintName"), bp::arg("masterJointName"), bp::arg("sign")), "Default Constructor"))
        .add_property("dim", &TaskMimic::dim, "return dimension size")
        .def("setReference", &TaskJointMimicPythonVisitor::setReference, bp::arg("ref"))
        .add_property("getDesiredAcceleration", bp::make_function(&TaskJointMimicPythonVisitor::getDesiredAcceleration, bp::return_value_policy<bp::copy_const_reference>()), "Return Acc_desired")
        .add_property("mask", bp::make_function(&TaskJointMimicPythonVisitor::getmask, bp::return_value_policy<bp::copy_const_reference>()), "Return mask")
        .def("setMask", &TaskJointMimicPythonVisitor::setmask, bp::arg("mask"))
        .def("getAcceleration", &TaskJointMimicPythonVisitor::getAcceleration, bp::arg("dv"))
        .add_property("position_error", bp::make_function(&TaskJointMimicPythonVisitor::position_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_error", bp::make_function(&TaskJointMimicPythonVisitor::velocity_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position", bp::make_function(&TaskJointMimicPythonVisitor::position, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity", bp::make_function(&TaskJointMimicPythonVisitor::velocity, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position_ref", bp::make_function(&TaskJointMimicPythonVisitor::position_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_ref", bp::make_function(&TaskJointMimicPythonVisitor::velocity_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kp", bp::make_function(&TaskJointMimicPythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&TaskJointMimicPythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskJointMimicPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &TaskJointMimicPythonVisitor::setKd, bp::arg("Kd"))
        .def("compute", &TaskJointMimicPythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskJointMimicPythonVisitor::getConstraint)
        .add_property("name", &TaskJointMimicPythonVisitor::name)
        ;
      }
      static std::string name(TaskMimic & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintEquality compute(TaskMimic & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static math::ConstraintEquality getConstraint(const TaskMimic & self){
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static void setReference(TaskMimic & self, const trajectories::TrajectorySample & ref){
        self.setReference(ref);
      }
      static const Eigen::VectorXd & getDesiredAcceleration(const TaskMimic & self){
        return self.getDesiredAcceleration();
      }
      static const Eigen::VectorXd & getmask(const TaskMimic & self){
        return self.getMask();
      }
      static void setmask (TaskMimic & self, const Eigen::VectorXd mask){
        return self.setMask(mask);
      }
      static Eigen::VectorXd getAcceleration (TaskMimic & self, const Eigen::VectorXd dv){
        return self.getAcceleration(dv);
      }
      static const Eigen::VectorXd & position_error(const TaskMimic & self){
        return self.position_error();
      }
      static const Eigen::VectorXd & velocity_error(const TaskMimic & self){
        return self.velocity_error();
      }
      static const Eigen::VectorXd & position (const TaskMimic & self){
        return self.position();
      }
      static const Eigen::VectorXd & velocity (const TaskMimic & self){
        return self.velocity();
      }
      static const Eigen::VectorXd & position_ref (const TaskMimic & self){
        return self.position_ref();
      }
      static const Eigen::VectorXd & velocity_ref (const TaskMimic & self){
        return self.velocity_ref();
      }     
      static const Eigen::VectorXd & Kp (TaskMimic & self){
        return self.Kp();
      }  
      static const Eigen::VectorXd & Kd (TaskMimic & self){
        return self.Kd();
      }    
      static void setKp (TaskMimic & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (TaskMimic & self, const::Eigen::VectorXd Kv){
        return self.Kd(Kv);
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "TaskMimic info.";
        bp::class_<TaskMimic>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskJointMimicPythonVisitor<TaskMimic>());

      //  bp::register_ptr_to_python< boost::shared_ptr<math::ConstraintBase> >();
      }
    };
  }
}


#endif // ifndef __tsid_python_task_joint_hpp__
