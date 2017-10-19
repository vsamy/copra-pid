# This file is part of pyCopra.

# pyCopra is free software: you can redistribute it and/or
# modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# pyCopra is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with pyCopra.  If not, see
# <http://www.gnu.org/licenses/>.

import unittest
import pyCopra
import numpy as np
from sys import float_info


class TestPyCopra(unittest.TestCase):
    def setUp(self):
        self.timestep = 0.005
        self.mass = 5
        self.nbStep = 300
        self.A = np.identity(2)
        self.A[0, 1] = self.timestep
        self.B = np.array([[0.5 * self.timestep * self.timestep / self.mass], [self.timestep / self.mass]])
        self.c = np.array([(-9.81/2.) * self.timestep**2, -9.81 * self.timestep])
        self.x0 = np.array([0., -5.])
        self.wu = np.array([1e-4])
        self.wx = np.array([10., 10000.])

        # Costs
        self.xd = np.zeros((2,))
        self.ud = np.zeros((1,))
        self.M = np.identity(2)
        self.N = np.array([[1.]])

        # Inequality constraints
        self.Gineq = np.array([[1.]])
        self.hineq = np.array([200.])
        self.Eineq = np.array([[0., 1.]])
        self.fineq = np.zeros((1,))

        # Bound constraints
        self.uLower = np.array([-float('Inf')])
        self.uUpper = np.array([200.])
        self.xLower = np.array([-float('Inf'), -float('Inf')])
        self.xUpper = np.array([float('Inf'), 0.])

        # Equality constraints
        self.x0Eq = np.zeros((2,))
        self.xdEq = np.zeros((2,))
        self.Geq = np.array([[1.]])
        self.heq = np.array([200.])
        self.Eeq = np.zeros((2, 2))
        self.Eeq[0, 0] = 1.
        self.feq = self.x0Eq

    def test_lmpc_ineq(self):
        print "test_lmpc_ineq"
        ps = pyCopra.PreviewSystem()
        ps.system(self.A, self.B, self.c, self.x0, self.nbStep)

        controller = pyCopra.LMPC(ps)
        xCost = pyCopra.TargetCost(self.M, -self.xd)
        uCost = pyCopra.ControlCost(self.N, -self.ud)
        trajConstr = pyCopra.TrajectoryConstraint(self.Eineq, self.fineq)
        contConstr = pyCopra.ControlConstraint(self.Gineq, self.hineq)
        xCost.weights(self.wx)
        uCost.weights(self.wu)

        controller.add_cost(xCost)
        controller.add_cost(uCost)
        controller.add_constraint(trajConstr)
        controller.add_constraint(contConstr)

        self.assertTrue(controller.solve())
        control = controller.control()
        fullTraj = controller.trajectory()
        fTLen = len(fullTraj) / 2
        posTraj = [0.] * fTLen
        velTraj = [0.] * fTLen
        for i in xrange(fTLen):
            posTraj[i] = fullTraj[2 * i]
            velTraj[i] = fullTraj[2 * i + 1]

        self.assertAlmostEqual(self.xd[1], velTraj[-1], places=3)
        self.assertLessEqual(max(posTraj), self.x0[0])
        self.assertLessEqual(np.amax(control), self.hineq[0])

        print "Test lmpc with inequalities"
        print controller.solve_time(), "s"
        print controller.solve_and_build_time(), "s"
        print

    def test_lmpc_mixed(self):
        print "test_lmpc_mixed"
        ps = pyCopra.PreviewSystem()
        ps.system(self.A, self.B, self.c, self.x0, self.nbStep)

        controller = pyCopra.LMPC(ps)
        xCost = pyCopra.TargetCost(self.M, -self.xd)
        uCost = pyCopra.ControlCost(self.N, -self.ud)
        mixedConstr = pyCopra.MixedConstraint(self.Eineq, self.Gineq, self.hineq)
        xCost.weights(self.wx)
        uCost.weights(self.wu)

        controller.add_cost(xCost)
        controller.add_cost(uCost)
        controller.add_constraint(mixedConstr)

        self.assertTrue(controller.solve())
        control = controller.control()
        fullTraj = controller.trajectory()
        fTLen = len(fullTraj) / 2
        posTraj = [0.] * fTLen
        velTraj = [0.] * fTLen
        for i in xrange(fTLen):
            posTraj[i] = fullTraj[2 * i]
            velTraj[i] = fullTraj[2 * i + 1]

        self.assertAlmostEqual(self.xd[1], velTraj[-1], places=3)
        self.assertLessEqual(max(posTraj), self.x0[0])
        for i in xrange(self.Eineq.shape[0]):
            res = 0
            for j in xrange(self.Eineq.shape[1]):
                res += self.Eineq[i, j] * fullTraj[i * self.Eineq.shape[1] + j]
            for j in xrange(self.Gineq.shape[1]):
                res += self.Gineq[i, j] * control[i * self.Gineq.shape[1] + j]
            self.assertLessEqual(res, self.hineq[0])

        print "Test lmpc with inequalities"
        print controller.solve_time(), "s"
        print controller.solve_and_build_time(), "s"
        print

    def test_lmpc_bound(self):
        print "test_lmpc_bound"
        ps = pyCopra.PreviewSystem()
        ps.system(self.A, self.B, self.c, self.x0, self.nbStep)

        controller = pyCopra.LMPC(ps)
        xCost = pyCopra.TargetCost(self.M, -self.xd)
        uCost = pyCopra.ControlCost(self.N, -self.ud)
        trajConstr = pyCopra.TrajectoryBoundConstraint(self.xLower, self.xUpper)
        contConstr = pyCopra.ControlBoundConstraint(self.uLower, self.uUpper)
        xCost.weights(self.wx)
        uCost.weights(self.wu)

        controller.add_cost(xCost)
        controller.add_cost(uCost)
        controller.add_constraint(trajConstr)
        controller.add_constraint(contConstr)

        self.assertTrue(controller.solve())
        control = controller.control()
        fullTraj = controller.trajectory()
        fTLen = len(fullTraj) / 2
        posTraj = [0.] * fTLen
        velTraj = [0.] * fTLen
        for i in xrange(fTLen):
            posTraj[i] = fullTraj[2 * i]
            velTraj[i] = fullTraj[2 * i + 1]

        self.assertAlmostEqual(self.xd[1], velTraj[-1], places=3)
        self.assertLessEqual(max(posTraj), self.x0[0])
        self.assertLessEqual(max(velTraj), self.xUpper[1] + 1e-6)
        self.assertLessEqual(max(control), self.uUpper[0] + 1e-6)

        print "Test lmpc with bounds"
        print controller.solve_time(), "s"
        print controller.solve_and_build_time(), "s"
        print

    def test_lmpc_eq(self):
        print "test_lmpc_eq"
        ps = pyCopra.PreviewSystem()
        ps.system(self.A, self.B, self.c, self.x0Eq, self.nbStep)

        controller = pyCopra.LMPC(ps)
        xCost = pyCopra.TargetCost(self.M, -self.xdEq)
        uCost = pyCopra.ControlCost(self.N, -self.ud)
        trajConstr = pyCopra.TrajectoryConstraint(self.Eeq, self.feq, False)
        xCost.weights(self.wx)
        uCost.weights(self.wu)

        controller.add_cost(xCost)
        controller.add_cost(uCost)
        controller.add_constraint(trajConstr)

        self.assertTrue(controller.solve())
        control = controller.control()
        fullTraj = controller.trajectory()
        fTLen = len(fullTraj) / 2
        posTraj = [0.] * fTLen
        velTraj = [0.] * fTLen
        for i in xrange(fTLen):
            posTraj[i] = fullTraj[2 * i]
            velTraj[i] = fullTraj[2 * i + 1]

        self.assertAlmostEqual(self.xd[1], velTraj[-1], places=3)
        self.assertLessEqual(max(posTraj), self.x0[0] + 1e-6)
        self.assertLessEqual(max(velTraj), self.feq[0] + 1e-6)

        print "Test lmpc with equalities"
        print controller.solve_time(), "s"
        print controller.solve_and_build_time(), "s"
        print

    def test_constructors_initialisations(self):
        print "test_constructors_initialisations"
        ps = pyCopra.PreviewSystem()
        ps.system(self.A, self.B, self.c, self.x0, self.nbStep)

        controller = pyCopra.LMPC(ps)

        pyCopra.LMPC()
        if hasattr(pyCopra.SolverFlag, 'QuadProgDense'):
            pyCopra.LMPC(pyCopra.SolverFlag.QuadProgDense)
            pyCopra.LMPC(ps, pyCopra.SolverFlag.QuadProgDense)
        elif hasattr(pyCopra.SolverFlag, 'QLD'):
            pyCopra.LMPC(pyCopra.SolverFlag.QLD)
            pyCopra.LMPC(ps, pyCopra.SolverFlag.QLD)
        else:
            pyCopra.LMPC(pyCopra.SolverFlag.Default)
            pyCopra.LMPC(ps, pyCopra.SolverFlag.Default)

        controller.initialize_controller(ps)

    @unittest.expectedFailure
    def test_fail_construct_trajectory(self):
        print "test_fail_construct_trajectory"
        constr = pyCopra.TrajectoryConstraint()

    @unittest.expectedFailure
    def test_fail_construct_control(self):
        print "test_fail_construct_control"
        constr = pyCopra.ControlConstraint()

    @unittest.expectedFailure
    def test_fail_construct_control(self):
        print "test_fail_construct_control"
        constr = pyCopra.TrajectoryBoundConstraint()

    @unittest.expectedFailure
    def test_fail_construct_control(self):
        print "test_fail_construct_control"
        constr = pyCopra.ControlBoundConstraint()

    def test_constraint_and_cost_deletion(self):
        print "Testing 'test_constraint_deletion'."
        print "In order to see the outputs, the pyCopra must be installed under Debug mode."

        ps = pyCopra.PreviewSystem()
        ps.system(self.A, self.B, self.c, self.x0, self.nbStep)

        controller = pyCopra.LMPC(ps)
        trajConstr = pyCopra.TrajectoryConstraint(self.Eineq, self.fineq)
        contConstr = pyCopra.ControlConstraint(self.Gineq, self.hineq)
        trajEqConstr = pyCopra.TrajectoryConstraint(self.Eeq, self.feq, False)
        contEqConstr = pyCopra.ControlConstraint(self.Geq, self.heq, False)
        trajBdConstr = pyCopra.TrajectoryBoundConstraint(self.xLower, self.xUpper)
        contBdConstr = pyCopra.ControlBoundConstraint(self.uLower, self.uUpper)
        targetCost = pyCopra.TargetCost(self.M, -self.xd)
        trajectoryCost = pyCopra.TrajectoryCost(self.M, -self.xd)
        controlCost = pyCopra.ControlCost(self.N, -self.ud)
        M_mixed = np.ones((1, 2))
        mixedCost = pyCopra.MixedCost(M_mixed, self.N, -self.ud)
        
        controller.add_constraint(trajConstr)
        controller.add_constraint(contConstr)
        controller.add_constraint(trajEqConstr)
        controller.add_constraint(contEqConstr)
        controller.add_constraint(trajBdConstr)
        controller.add_constraint(contBdConstr)
        controller.add_cost(targetCost)
        controller.add_cost(trajectoryCost)
        controller.add_cost(controlCost)
        controller.add_cost(mixedCost)

        del trajConstr

        targetCost.weights(self.wx)
        controlCost.weights(self.wu)

        del trajEqConstr
        del contEqConstr
        del trajBdConstr
        del contBdConstr
        del trajectoryCost
        del mixedCost

        self.assertFalse(controller.solve())
        self.assertTrue(controller.solve()) # Has kept the contConstr only

    def test_preview_systeme_still_exist(self):
        print "test_preview_systeme_still_exist"
        ps = pyCopra.PreviewSystem()
        ps.system(self.A, self.B, self.c, self.x0, self.nbStep)

        controller = pyCopra.LMPC(ps)
        del ps
        trajConstr = pyCopra.TrajectoryConstraint(self.Eineq, self.fineq)
        contConstr = pyCopra.ControlConstraint(self.Gineq, self.hineq)
        targetCost = pyCopra.TargetCost(self.M, -self.xd)
        controlCost = pyCopra.ControlCost(self.N, -self.ud)
        targetCost.weights(self.wx)
        controlCost.weights(self.wu)

        controller.add_constraint(trajConstr)
        controller.add_constraint(contConstr)
        controller.add_cost(targetCost)
        controller.add_cost(controlCost)

        self.assertTrue(controller.solve())
        control = controller.control()
        fullTraj = controller.trajectory()
        fTLen = len(fullTraj) / 2
        posTraj = [0.] * fTLen
        velTraj = [0.] * fTLen
        for i in xrange(fTLen):
            posTraj[i] = fullTraj[2 * i]
            velTraj[i] = fullTraj[2 * i + 1]

        self.assertAlmostEqual(self.xd[1], velTraj[-1], places=3)
        self.assertLessEqual(max(posTraj), self.x0[0])
        self.assertLessEqual(np.amax(control), self.hineq[0])

    def test_throw_handler(self):
        print "test_throw_handler"
        ps = pyCopra.PreviewSystem()
        ps.system(self.A, self.B, self.c, self.x0, self.nbStep)

        controller = pyCopra.LMPC(ps)
        # Test trajectory constraint throws
        with self.assertRaises(RuntimeError):
            constr = pyCopra.TrajectoryConstraint(np.identity(5), np.ones((2,)))
            controller.add_constraint(constr)

        # Test control constraint throws
        with self.assertRaises(RuntimeError):
            constr = pyCopra.ControlConstraint(np.identity(5), np.ones((2,)))
            controller.add_constraint(constr)

        # Test mixed constraint throws
        with self.assertRaises(RuntimeError):
            constr = pyCopra.MixedConstraint(np.identity(5), np.identity(5), np.ones((2,)))
            controller.add_constraint(constr)

        # Test trajectory bound constraint throws
        with self.assertRaises(RuntimeError):
            constr = pyCopra.TrajectoryBoundConstraint(np.ones((3,)), np.ones((2,)))
            controller.add_constraint(constr)

        # Test control bound constraint throws
        with self.assertRaises(RuntimeError):
            constr = pyCopra.ControlBoundConstraint(np.ones((3,)), np.ones((2,)))
            controller.add_constraint(constr)

    def test_dynamic_walk(self):
        print "test_dynamic_walk"
        A = np.array([
            [      1,      0,      0,0.11699999999999999,      0,      0],
            [      0,      1,      0,      0,0.11699999999999999,      0],
            [      0,      0,      1,      0,      0,0.11699999999999999],
            [      0,      0,      0,      1,      0,      0],
            [      0,      0,      0,      0,      1,      0],
            [      0,      0,      0,      0,      0,      1]])
        B = np.array([
	        [0.006844499999999999,      0,      0],
            [      0,0.006844499999999999,      0],
            [      0,      0,0.006844499999999999],
            [0.11699999999999999,      0,      0],
            [      0,0.11699999999999999,      0],
            [      0,      0,0.11699999999999999]])

        c = np.array([0., 0., 0., 0., 0., 0.])
        x_init = np.array([1.5842778860957882,0.3422260214935311,2.289067474385933, 0,0,0])
        x_goal = np.array([1.627772868473883,0.4156386515475985,2.3984423755527136, 0.06745225960685897,0.3882830795737303,0.06845759848745198])
        nb_steps = 10
        G = np.array([
            [     -1,9.946646523934742,-4.870790074510924,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [-18.826459196882055,3.4468275392859393,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [-1.374181960437557,-8.028252906078723,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [-9.936224732113594,5.000580301294253,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [8.750597187695343,-4.7538557382857105,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [18.65430148414319,      1,-5.084871935334947,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [2.1775137248880574e-15,-5.443784312220143e-16,      1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,     -1,9.946646523934742,-4.870790074510924,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,-18.826459196882055,3.4468275392859393,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,-1.374181960437557,-8.028252906078723,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,-9.936224732113594,5.000580301294253,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,8.750597187695343,-4.7538557382857105,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,18.65430148414319,      1,-5.084871935334947,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,2.1775137248880574e-15,-5.443784312220143e-16,      1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,     -1,9.946646523934742,-4.870790074510924,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,-18.826459196882055,3.4468275392859393,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,-1.374181960437557,-8.028252906078723,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,-9.936224732113594,5.000580301294253,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,8.750597187695343,-4.7538557382857105,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,18.65430148414319,      1,-5.084871935334947,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,2.1775137248880574e-15,-5.443784312220143e-16,      1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,     -1,9.946646523934742,-4.870790074510924,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,-18.826459196882055,3.4468275392859393,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,-1.374181960437557,-8.028252906078723,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,-9.936224732113594,5.000580301294253,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,8.750597187695343,-4.7538557382857105,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,18.65430148414319,      1,-5.084871935334947,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,2.1775137248880574e-15,-5.443784312220143e-16,      1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,     -1,9.946646523934742,-4.870790074510924,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-18.826459196882055,3.4468275392859393,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-1.374181960437557,-8.028252906078723,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-9.936224732113594,5.000580301294253,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,8.750597187695343,-4.7538557382857105,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,18.65430148414319,      1,-5.084871935334947,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,2.1775137248880574e-15,-5.443784312220143e-16,      1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,     -1,9.946646523934742,-4.870790074510924,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-18.826459196882055,3.4468275392859393,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-1.374181960437557,-8.028252906078723,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-9.936224732113594,5.000580301294253,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,8.750597187695343,-4.7538557382857105,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,18.65430148414319,      1,-5.084871935334947,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,2.1775137248880574e-15,-5.443784312220143e-16,      1,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,8.750597218241072,-4.753855754313641,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-1.3741819771739483,-8.028252929943818,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-18.82645925631406,3.4468275193254927,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,1.1824397341134247,7.4638136184143935,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,14.006645137157978,-2.2569159229140494,     -1,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,2.1775137248880578e-15,-5.443784312220144e-16,      1,      0,      0,      0,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,8.750597218241072,-4.753855754313641,     -1,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-1.3741819771739483,-8.028252929943818,     -1,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-18.82645925631406,3.4468275193254927,     -1,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,1.1824397341134247,7.4638136184143935,     -1,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,14.006645137157978,-2.2569159229140494,     -1,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,2.1775137248880578e-15,-5.443784312220144e-16,      1,      0,      0,      0,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,8.750597218241072,-4.753855754313641,     -1,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-1.3741819771739483,-8.028252929943818,     -1,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-18.82645925631406,3.4468275193254927,     -1,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,1.1824397341134247,7.4638136184143935,     -1,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,14.006645137157978,-2.2569159229140494,     -1,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,2.1775137248880578e-15,-5.443784312220144e-16,      1,      0,      0,      0],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,8.750597218241072,-4.753855754313641,     -1],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-1.3741819771739483,-8.028252929943818,     -1],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,-18.82645925631406,3.4468275193254927,     -1],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,1.1824397341134247,7.4638136184143935,     -1],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,14.006645137157978,-2.2569159229140494,     -1],
            [      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,2.1775137248880578e-15,-5.443784312220144e-16,      1]])

        h = np.array([47.76613348420254,9.80665,9.80665, 9.80665,9.806649999999998,49.86555936465245, 9.806649999999994,47.76613348420254,9.80665, 9.80665,9.80665,9.806649999999998, 49.86555936465245,9.806649999999994,47.76613348420254, 9.80665,9.80665,9.80665, 9.806649999999998,49.86555936465245,9.806649999999994, 47.76613348420254,9.80665,9.80665, 9.80665,9.806649999999998,49.86555936465245, 9.806649999999994,47.76613348420254,9.80665, 9.80665,9.80665,9.806649999999998, 49.86555936465245,9.806649999999994,47.76613348420254, 9.80665,9.80665,9.80665, 9.806649999999998,49.86555936465245,9.806649999999994, 9.806650000000007,9.806650000000008,9.80665, 9.806650000000001,9.806650000000007,9.806649999999996, 9.806650000000007,9.806650000000008,9.80665, 9.806650000000001,9.806650000000007,9.806649999999996, 9.806650000000007,9.806650000000008,9.80665, 9.806650000000001,9.806650000000007,9.806649999999996, 9.806650000000007,9.806650000000008,9.80665, 9.806650000000001,9.806650000000007,9.806649999999996])

        ps = pyCopra.PreviewSystem()
        ps.system(A, B, c, x_init, nb_steps)

        controller = pyCopra.LMPC(ps)
        contConstr = pyCopra.ControlConstraint(G, h)
        M_cost = np.identity(6)
        targetCost = pyCopra.TargetCost(M_cost, -x_goal)

        controller.add_constraint(contConstr)
        controller.add_cost(targetCost)

        controller.solve()
        print controller.solve_time(), "s"

nb_steps = 10

if __name__ == '__main__':
    unittest.main()
