
declare_PID_Component(
    TEST_APPLICATION 
    NAME test-copra
    DIRECTORY copra
)

declare_PID_Component_Dependency(
    COMPONENT test-copra
    NATIVE copra
)

declare_PID_Component_Dependency(
    COMPONENT test-copra
    EXTERNAL boost-utest 
    PACKAGE boost
)



run_PID_Test(
    NAME testing-copra
    COMPONENT test-copra
)

run_PID_Test(
    NAME test-python-copra-wrapper
    PYTHON ARGUMENTS pyCopra/TestCopraBindings.py
)
