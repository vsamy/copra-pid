declare_PID_Component(
    SHARED_LIB 
    NAME copra
    DIRECTORY copra
    CXX_STANDARD 14
    INTERNAL DEFINITIONS ${COMPILER_DEFS}
)

declare_PID_Component_Dependency(
    COMPONENT copra
    EXPORT EXTERNAL eigen 
    PACKAGE eigen
)

if(${EIGEN_QUADPROG})
    declare_PID_Component_Dependency(
        COMPONENT copra
        EXPORT NATIVE eigen-quadprog
        PACKAGE eigen-qp
    )
endif()

if(${EIGEN_QLD})
    declare_PID_Component_Dependency(
        COMPONENT copra
        EXPORT NATIVE eigen-qld
        PACKAGE eigen-qp
    )
endif()

#declare the python module

declare_PID_Component(
    MODULE_LIB
    NAME pyCopra
    DIRECTORY pyCopra
    CXX_STANDARD 14
    INTERNAL COMPILER_OPTIONS -Wno-deprecated-declarations
             DEFINITIONS ${COMPILER_DEFS}
)

declare_PID_Component_Dependency(
    COMPONENT pyCopra 
    NATIVE copra
)

declare_PID_Component_Dependency(
    COMPONENT pyCopra
    EXPORT NATIVE pygen
    PACKAGE pygen-converter
)

get_PID_Platform_Info(PYTHON PY_VERSION)

if(PY_VERSION VERSION_LESS 3.0)
    #using python2 to manage python wrappers
    declare_PID_Component_Dependency(
        COMPONENT pyCopra 
        EXTERNAL boost-python 
        PACKAGE boost
    )
else()
    #using python3 to manage python wrappers
    declare_PID_Component_Dependency(
        COMPONENT pyCopra 
        EXTERNAL boost-python3 
        PACKAGE boost
    )
endif()
