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

#ifndef __tsid_python_task_actuation_equality_hpp__
#define __tsid_python_task_actuation_equality_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-actuation-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
//#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-base.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename Task>
    struct TaskActuationEqualityPythonVisitor
    : public boost::python::def_visitor< TaskActuationEqualityPythonVisitor<Task> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &> ((bp::arg("name"), bp::arg("robot")), "Default Constructor"))
        .add_property("dim", &Task::dim, "return dimension size")
        .add_property("mask", bp::make_function(&TaskActuationEqualityPythonVisitor::getmask, bp::return_value_policy<bp::copy_const_reference>()), "Return mask")
        .def("setMask", &TaskActuationEqualityPythonVisitor::setmask, bp::arg("mask"))
        .def("setReference", &TaskActuationEqualityPythonVisitor::setReference, bp::args("ref"))
        .def("setWeightVector", &TaskActuationEqualityPythonVisitor::setWeightVector, bp::args("wights"))
        .add_property("getReference", bp::make_function(&TaskActuationEqualityPythonVisitor::getReference, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("name", &TaskActuationEqualityPythonVisitor::name)
        ;
      }
      static std::string name(Task & self){
        std::string name = self.name();
        return name;
      }
      static const Eigen::VectorXd & getmask(const Task & self){
        return self.mask();
      }
      static void setmask (Task & self, const Eigen::VectorXd mask){
        return self.mask(mask);
      }
      static const Eigen::VectorXd & getReference (const Task & self){
        return self.getReference();
      }    
      static void setReference (Task & self, const Eigen::VectorXd ref){
        return self.setReference(ref);
      }
      static void setWeightVector (Task & self, const Eigen::VectorXd weights){
        return self.setWeightVector(weights);
      }      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Task info.";
        bp::class_<Task>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskActuationEqualityPythonVisitor<Task>());
      }
    };
  }
}


#endif // ifndef __tsid_python_task_actuation_equality_hpp__
