//
// Copyright (c) 2021 LAAS-CNRS, University of Trento
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

#ifndef __tsid_python_task_contact_force_equality_point_hpp__
#define __tsid_python_task_contact_force_equality_point_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-contact-force-equality-point.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename TaskContactForceEqualityPoint>
struct TaskContactForceEqualityPointPythonVisitor
    : public boost::python::def_visitor<
          TaskContactForceEqualityPointPythonVisitor<TaskContactForceEqualityPoint> > {
  template <class PyClass>

  void visit(PyClass &cl) const {
    cl.def(
          bp::init<std::string, robots::RobotWrapper &, double,
                   contacts::ContactPoint &>((bp::arg("name"), bp::arg("robot"),
                                             bp::arg("dt"), bp::arg("contact")),
                                            "Default Constructor"))
        .add_property("dim", &TaskContactForceEqualityPoint::dim,
                      "return dimension size")
        .def("setReference",
             &TaskContactForceEqualityPointPythonVisitor::setReference,
             bp::arg("ref"))
        .def("setExternalForce",
             &TaskContactForceEqualityPointPythonVisitor::setExternalForce,
             bp::arg("f_ext"))
        .def("compute", &TaskContactForceEqualityPointPythonVisitor::compute,
             bp::args("t", "q", "v", "data"))
        .def("getConstraint",
             &TaskContactForceEqualityPointPythonVisitor::getConstraint)
        .add_property("name", &TaskContactForceEqualityPointPythonVisitor::name)
        .add_property("Kp",
                      bp::make_function(
                          &TaskContactForceEqualityPointPythonVisitor::Kp,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd",
                      bp::make_function(
                          &TaskContactForceEqualityPointPythonVisitor::Kd,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Ki",
                      bp::make_function(
                          &TaskContactForceEqualityPointPythonVisitor::Kd,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("getLeakRate",
                      bp::make_function(
                          &TaskContactForceEqualityPointPythonVisitor::getLeakRate,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskContactForceEqualityPointPythonVisitor::setKp,
             bp::arg("Kp"))
        .def("setKd", &TaskContactForceEqualityPointPythonVisitor::setKd,
             bp::arg("Kd"))
        .def("setKi", &TaskContactForceEqualityPointPythonVisitor::setKi,
             bp::arg("Ki"))
        .def("setLeakRate", &TaskContactForceEqualityPointPythonVisitor::setLeakRate,
             bp::arg("leak"));
  }
  static std::string name(TaskContactForceEqualityPoint &self) {
    std::string name = self.name();
    return name;
  }
  static math::ConstraintEquality compute(TaskContactForceEqualityPoint &self,
                                          const double t,
                                          const Eigen::VectorXd &q,
                                          const Eigen::VectorXd &v,
                                          pinocchio::Data &data) {
    self.compute(t, q, v, data);
    math::ConstraintEquality cons(self.getConstraint().name(),
                                  self.getConstraint().matrix(),
                                  self.getConstraint().vector());
    return cons;
  }
  static math::ConstraintEquality getConstraint(
      const TaskContactForceEqualityPoint &self) {
    math::ConstraintEquality cons(self.getConstraint().name(),
                                  self.getConstraint().matrix(),
                                  self.getConstraint().vector());
    return cons;
  }
  static void setReference(TaskContactForceEqualityPoint &self,
                           trajectories::TrajectorySample &ref) {
    self.setReference(ref);
  }
  static void setExternalForce(TaskContactForceEqualityPoint &self,
                               trajectories::TrajectorySample &f_ext) {
    self.setExternalForce(f_ext);
  }
  static const Eigen::VectorXd &Kp(TaskContactForceEqualityPoint &self) {
    return self.Kp();
  }
  static const Eigen::VectorXd &Kd(TaskContactForceEqualityPoint &self) {
    return self.Kd();
  }
  static const Eigen::VectorXd &Ki(TaskContactForceEqualityPoint &self) {
    return self.Ki();
  }
  static const double &getLeakRate(TaskContactForceEqualityPoint &self) {
    return self.getLeakRate();
  }
  static void setKp(TaskContactForceEqualityPoint &self,
                    const ::Eigen::VectorXd Kp) {
    return self.Kp(Kp);
  }
  static void setKd(TaskContactForceEqualityPoint &self,
                    const ::Eigen::VectorXd Kd) {
    return self.Kd(Kd);
  }
  static void setKi(TaskContactForceEqualityPoint &self,
                    const ::Eigen::VectorXd Ki) {
    return self.Ki(Ki);
  }
  static void setLeakRate(TaskContactForceEqualityPoint &self, const double leak) {
    return self.setLeakRate(leak);
  }
  static void expose(const std::string &class_name) {
    std::string doc = "TaskContactForceEqualityPointPythonVisitor info.";
    bp::class_<TaskContactForceEqualityPoint>(class_name.c_str(), doc.c_str(),
                                         bp::no_init)
        .def(TaskContactForceEqualityPointPythonVisitor<TaskContactForceEqualityPoint>());
  }
};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_task_contact_force_equality_hpp__
