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

#ifndef __tsid_python_task_com_in_frame_hpp__
#define __tsid_python_task_com_in_frame_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-com-in-frame-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename TaskCOMInFrame>
    struct TaskCOMInFrameEqualityPythonVisitor
    : public boost::python::def_visitor< TaskCOMInFrameEqualityPythonVisitor<TaskCOMInFrame> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, std::string> ((bp::arg("name"), bp::arg("robot"), bp::arg("framename")), "Default Constructor"))
        .add_property("dim", &TaskCOMInFrame::dim, "return dimension size")
        .def("setReference", &TaskCOMInFrameEqualityPythonVisitor::setReference, bp::arg("ref"))
        .add_property("getDesiredAcceleration", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::getDesiredAcceleration, bp::return_value_policy<bp::copy_const_reference>()), "Return Acc_desired")
        .def("getAcceleration", &TaskCOMInFrameEqualityPythonVisitor::getAcceleration, bp::arg("dv"))
        .add_property("position_error", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::position_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_error", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::velocity_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::position, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::velocity, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position_ref", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::position_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_ref", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::velocity_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kp", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskCOMInFrameEqualityPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &TaskCOMInFrameEqualityPythonVisitor::setKd, bp::arg("Kd"))
        .def("compute", &TaskCOMInFrameEqualityPythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskCOMInFrameEqualityPythonVisitor::getConstraint)
        .add_property("name", &TaskCOMInFrameEqualityPythonVisitor::name)
        .add_property("mask", bp::make_function(&TaskCOMInFrameEqualityPythonVisitor::getmask, bp::return_value_policy<bp::copy_const_reference>()), "Return mask")
        .def("setMask", &TaskCOMInFrameEqualityPythonVisitor::setmask, bp::arg("mask"))
        ;
      }
      static std::string name(TaskCOMInFrame & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintEquality compute(TaskCOMInFrame & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static math::ConstraintEquality getConstraint(const TaskCOMInFrame & self){
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static void setReference(TaskCOMInFrame & self, const trajectories::TrajectorySample & ref){
        self.setReference(ref);
      }
      static const Eigen::VectorXd & getDesiredAcceleration(const TaskCOMInFrame & self){
        return self.getDesiredAcceleration();
      }
      static Eigen::VectorXd getAcceleration (TaskCOMInFrame & self, const Eigen::VectorXd dv){
        return self.getAcceleration(dv);
      }
      static const Eigen::VectorXd & position_error(const TaskCOMInFrame & self){
        return self.position_error();
      }
      static const Eigen::VectorXd & velocity_error(const TaskCOMInFrame & self){
        return self.velocity_error();
      }
      static const Eigen::VectorXd & position (const TaskCOMInFrame & self){
        return self.position();
      }
      static const Eigen::VectorXd & velocity (const TaskCOMInFrame & self){
        return self.velocity();
      }
      static const Eigen::VectorXd & position_ref (const TaskCOMInFrame & self){
        return self.position_ref();
      }
      static const Eigen::VectorXd & velocity_ref (const TaskCOMInFrame & self){
        return self.velocity_ref();
      }     
      static const Eigen::Vector3d & Kp (TaskCOMInFrame & self){
        return self.Kp();
      }  
      static const Eigen::Vector3d & Kd (TaskCOMInFrame & self){
        return self.Kd();
      }    
      static void setKp (TaskCOMInFrame & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (TaskCOMInFrame & self, const::Eigen::VectorXd Kv){
        return self.Kd(Kv);
      }
      static const Eigen::VectorXd & getmask(const TaskCOMInFrame & self){
        return self.getMask();
      }
      static void setmask (TaskCOMInFrame & self, const Eigen::VectorXd mask){
        return self.setMask(mask);
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "TaskCOMInFrameEqualityPythonVisitor info.";
        bp::class_<TaskCOMInFrame>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskCOMInFrameEqualityPythonVisitor<TaskCOMInFrame>());

        bp::register_ptr_to_python< boost::shared_ptr<math::ConstraintBase> >();
      }
    };
  }
}


#endif // ifndef __tsid_python_task_com_hpp__
