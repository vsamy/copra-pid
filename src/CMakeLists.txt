declare_PID_Component(
    SHARED_LIB 
    NAME copra
    DIRECTORY copra
    CXX_STANDARD 14
)

declare_PID_Component_Dependency(
    COMPONENT copra
    EXPORT EXTERNAL eigen 
    PACKAGE eigen
)

declare_PID_Component_Dependency(
    COMPONENT copra
    EXPORT NATIVE eigen-quadprog
    PACKAGE eigen-qp
)

declare_PID_Component_Dependency(
    COMPONENT copra
    EXPORT NATIVE eigen-qld
    PACKAGE eigen-qp
)

#declare the python module

declare_PID_Component(
    MODULE_LIB
    NAME pyCopra
    DIRECTORY pyCopra
    CXX_STANDARD 14
    INTERNAL COMPILER_OPTIONS -Wno-deprecated-declarations
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
