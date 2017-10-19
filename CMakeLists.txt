CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

PROJECT(copra)

option(EIGEN_QLD "Add the quadprog qp solver into the lib" ON)
option(EIGEN_QUADPROG "Add the qld qp solver into the lib" ON)

if((NOT ${EIGEN_QLD}) AND (NOT ${EIGEN_QUADPROG}))
	message(FATAL_ERROR "The lib needs at least one qp solver to work properly")
endif()

set(COMPILER_DEFS "")
if(${EIGEN_QLD})
	set(COMPILER_DEFS ${COMPILER_DEFS} "EIGEN_QLD")
endif()
if(${EIGEN_QUADPROG})
	set(COMPILER_DEFS ${COMPILER_DEFS} "EIGEN_QUADPROG")
endif()

declare_PID_Package(
	AUTHOR 			Vincent Samy
	INSTITUTION		LIRMM
	MAIL			vincent.samy@lirmm.fr
	ADDRESS 		git@github.com:vsamy/copra.git
	PUBLIC_ADDRESS	https://github.com/vsamy/copra.git
	YEAR 			2017 
	LICENSE 		CeCILL-C
	DESCRIPTION 	"This a C++ implementation of a Time Invariant Linear Model Predictive Controller (LMPC) done in C++14 with python bindings"
)

set_PID_Package_Version(1 1 0)

#now finding packages
declare_PID_Package_Dependency(PACKAGE eigen EXTERNAL VERSION 3.3.4)
declare_PID_Package_Dependency(PACKAGE eigen-qp NATIVE VERSION 0.1)
declare_PID_Package_Dependency(PACKAGE boost EXTERNAL VERSION 1.64.0)
declare_PID_Package_Dependency(PACKAGE pygen-converter NATIVE VERSION 0.2)

build_PID_Package()

