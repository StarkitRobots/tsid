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

#ifndef __tsid_python_task_frames_mirror_hpp__
#define __tsid_python_task_frames_mirror_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-two-frames-mirror.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid
{
  namespace python
  {
    namespace bp = boost::python;

    template<typename TaskFramesMirror>
    struct TaskTwoFramesMirrorPythonVisitor
    : public boost::python::def_visitor< TaskTwoFramesMirrorPythonVisitor<TaskFramesMirror> >
    {

      template<class PyClass>


      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, std::string, std::string, std::string> ((bp::arg("name"), bp::arg("robot"), bp::arg("framename1"), bp::arg("framename2"), bp::arg("framename_mirror")), "Default Constructor"))
        .add_property("dim", &TaskFramesMirror::dim, "return dimension size")
        .add_property("getDesiredAcceleration", bp::make_function(&TaskTwoFramesMirrorPythonVisitor::getDesiredAcceleration, bp::return_value_policy<bp::copy_const_reference>()), "Return Acc_desired")
        .def("getAcceleration", &TaskTwoFramesMirrorPythonVisitor::getAcceleration, bp::arg("dv"))
        .add_property("position_error", bp::make_function(&TaskTwoFramesMirrorPythonVisitor::position_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_error", bp::make_function(&TaskTwoFramesMirrorPythonVisitor::velocity_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kp", bp::make_function(&TaskTwoFramesMirrorPythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&TaskTwoFramesMirrorPythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskTwoFramesMirrorPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &TaskTwoFramesMirrorPythonVisitor::setKd, bp::arg("Kd"))
        .add_property("mask", bp::make_function(&TaskTwoFramesMirrorPythonVisitor::getMask, bp::return_value_policy<bp::copy_const_reference>()), "Return mask")
        .def("setMask", &TaskTwoFramesMirrorPythonVisitor::setMask, bp::arg("mask"))
        .def("compute", &TaskTwoFramesMirrorPythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskTwoFramesMirrorPythonVisitor::getConstraint)
        .add_property("frame_id1", &TaskFramesMirror::frame_id1, "frame id 1 return")
        .add_property("frame_id2", &TaskFramesMirror::frame_id2, "frame id 2 return")
        .add_property("name", &TaskTwoFramesMirrorPythonVisitor::name)
        ;
      }
       static std::string name(TaskFramesMirror & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintEquality compute(TaskFramesMirror & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static math::ConstraintEquality getConstraint(const TaskFramesMirror & self){
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static const Eigen::VectorXd & getDesiredAcceleration(const TaskFramesMirror & self){
        return self.getDesiredAcceleration();
      }
      static Eigen::VectorXd getAcceleration (TaskFramesMirror & self, const Eigen::VectorXd dv){
        return self.getAcceleration(dv);
      }
      static const Eigen::VectorXd & position_error(const TaskFramesMirror & self){
        return self.position_error();
      }
      static const Eigen::VectorXd & velocity_error(const TaskFramesMirror & self){
        return self.velocity_error();
      }
      static const Eigen::VectorXd & Kp (TaskFramesMirror & self){
        return self.Kp();
      }
      static const Eigen::VectorXd & Kd (TaskFramesMirror & self){
        return self.Kd();
      }
      static void setKp (TaskFramesMirror & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (TaskFramesMirror & self, const::Eigen::VectorXd Kv){
        return self.Kd(Kv);
      }
      static void getMask (TaskFramesMirror & self) {
        self.getMask();
      }
      static void setMask (TaskFramesMirror & self, const::Eigen::VectorXd mask) {
        self.setMask(mask);
      }
      static Eigen::VectorXd frame_id1 (TaskFramesMirror & self){
        return self.frame_id1();
      }
      static Eigen::VectorXd frame_id2 (TaskFramesMirror & self){
        return self.frame_id2();
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "TaskFramesMirror info.";
        bp::class_<TaskFramesMirror>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskTwoFramesMirrorPythonVisitor<TaskFramesMirror>());

      //  bp::register_ptr_to_python< boost::shared_ptr<math::ConstraintBase> >();
      }
    };
  }
}


#endif // ifndef __tsid_python_task_frames_hpp__
