
// header
#include <copra/SolverInterface.h>

// stl
#include <iostream>
#include <utility>

namespace copra {

/*
 * SolverInterface
 */

int SolverInterface::SI_iter() const
{
    return 0;
}

void SolverInterface::SI_printLevel(int /* pl */)
{
    std::cout << "No printLevel() function for this qp" << std::endl;
}

void SolverInterface::SI_feasibilityTolerance(double /* tol */)
{
    std::cout << "No tol(double) function for this qp" << std::endl;
}

bool SolverInterface::SI_warmStart() const
{
    std::cout << "No warmStart() function for this qp" << std::endl;
    return false;
}

void SolverInterface::SI_warmStart(bool /* w */)
{
    std::cout << "No warmStart(bool) function for this qp" << std::endl;
}

} // namespace pc