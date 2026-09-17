from dataclasses import dataclass, field

import numpy as np

from giskardpy.motion_statechart.context import MotionStatechartContext
from cramph.exceptions import PlotterNotConfiguredError
from giskardpy.motion_statechart.exceptions import WorldStateArrayReplacedError
from giskardpy.motion_statechart.graph_node import DebugExpression, MotionStatechartNode
from giskardpy.qp.constraint_collection import ConstraintCollection
from cramph.statechart import Statechart
from giskardpy.motion_statechart.plotters.debug_expression_trajectory_plotter import (
    DebugExpressionTrajectoryPlotter,
)
from giskardpy.qp.exceptions import EmptyProblemException
from giskardpy.qp.qp_controller import QPController
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.world_description.world_state_trajectory_plotter import (
    WorldStateTrajectoryPlotter,
)
from cramph.executor import StatechartExecutor


@dataclass
class Executor(StatechartExecutor):
    """
    Represents the main execution entity that manages motion statecharts, collision
    scenes, and control cycles for the robot's operations.
    """

    context: MotionStatechartContext

    trajectory_plotter: WorldStateTrajectoryPlotter | None = field(default=None)
    """
    The trajectory plotter used to plot the robot's trajectory.
    """

    debug_expression_plotter: DebugExpressionTrajectoryPlotter | None = field(
        default=None
    )
    """
    Records and plots how the debug expressions evolved during the motion.
    """

    # %% init False
    statechart: Statechart | None = field(init=False, default=None)
    """
    The motion statechart describing the robot's motion logic, set by :meth:`compile`.
    """

    qp_controller: QPController | None = field(default=None, init=False)
    """
    Optional quadratic programming controller used for motion control.
    """

    _compiled_world_state_data: np.ndarray | None = field(default=None, init=False)
    """
    The world state array the motion statechart was compiled against.

    The compiled updaters read it through a memory view, so it must stay the very same
    array for as long as they are in use.
    """

    def _after_compile(self):
        self._compiled_world_state_data = self.context.world.state._data
        self._compile_qp_controller(self.context.qp_controller_config)
        if self.trajectory_plotter is not None:
            self.trajectory_plotter.reset(self.context.world.state, self.time)
        if self.debug_expression_plotter is not None:
            self.debug_expression_plotter.reset(
                DebugExpression.collect_from(self.statechart)
            )
            self.debug_expression_plotter.debug_expression_trajectory.append(self.time)
        self.context.collision_manager.update_collision_matrix()

    def _before_tick(self):
        self._raise_if_world_state_array_was_replaced()
        if self.context.requires_collision_checking:
            self.context.collision_manager.compute_collisions()

    def _after_tick(self):
        if self.debug_expression_plotter is not None:
            self.debug_expression_plotter.debug_expression_trajectory.append(self.time)
        if self.qp_controller is None:
            return
        next_cmd = self.qp_controller.compute_command(
            world_state=self.context.world.state._data,
            life_cycle_state=self.statechart.life_cycle_state.data,
            float_variables=self.context.float_variable_data.data,
        )
        self.context.world.apply_control_commands(
            next_cmd,
            self.qp_controller.config.control_dt,
            self.qp_controller.config.max_derivative,
        )
        if self.trajectory_plotter is not None:
            self.trajectory_plotter.world_state_trajectory.append(
                self.context.world.state, self.time
            )

    def _after_run(self):
        self.set_velocity_acceleration_jerk_to_zero()

    def _raise_if_world_state_array_was_replaced(self):
        """
        Ensures the world still holds the state array the motion statechart compiled
        against.

        :raises WorldStateArrayReplacedError: If the world replaced its state array,
            which leaves the compiled updaters reading a detached copy of the state.
        """
        if self._compiled_world_state_data is None:
            return
        if self.context.world.state._data is self._compiled_world_state_data:
            return
        raise WorldStateArrayReplacedError(
            compiled_degrees_of_freedom=self._compiled_world_state_data.shape[1],
            current_degrees_of_freedom=self.context.world.state._data.shape[1],
        )

    def set_velocity_acceleration_jerk_to_zero(self):
        """
        Clear all commanded derivatives of the world state.
        """
        self.context.world.state.velocities[:] = 0
        self.context.world.state.accelerations[:] = 0
        self.context.world.state.jerks[:] = 0

    def _compile_qp_controller(self, controller_config: QPControllerConfig):
        ordered_dofs = sorted(
            self.context.world.active_degrees_of_freedom,
            key=lambda dof: self.context.world.state._index[dof.id],
        )
        constraint_collection = self._combine_constraint_collections_of_nodes()
        if len(constraint_collection._constraints) == 0:
            self.qp_controller = None
            # to not build controller, if there are no constraints
            return
        self.qp_controller = QPController(
            config=controller_config,
            degrees_of_freedom=ordered_dofs,
            constraint_collection=constraint_collection,
            world_state_symbols=self.context.world.state.get_variables(),
            life_cycle_variables=self.statechart.life_cycle_state.life_cycle_symbols(),
            float_variables=self.context.float_variable_data.variables,
        )
        if self.qp_controller.has_not_free_variables():
            raise EmptyProblemException()

    def _combine_constraint_collections_of_nodes(self) -> ConstraintCollection:
        """
        :return: The constraint collections of all motion nodes, merged into one, with
            each node's constraints prefixed by its
            :attr:`~cramph.node.StatechartNode.unique_name`.
        """
        combined_constraint_collection = ConstraintCollection()
        for node in self.statechart.get_nodes_by_type(MotionStatechartNode):
            combined_constraint_collection.merge(
                name_prefix=node.unique_name, other=node.constraint_collection
            )
        return combined_constraint_collection

    def plot_debug_expressions(self, file_name: str = "./debug_expressions.pdf"):
        """
        Plot the recorded debug expressions to the given PDF file.
        """
        if self.debug_expression_plotter is None:
            raise PlotterNotConfiguredError("debug expression plotter")
        self.debug_expression_plotter.plot(file_name)
