from __future__ import annotations

import numpy as np

from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.data_types import LifeCycleValues
from cramph.executor import StatechartExecutor
from cramph.statechart import Statechart
from cramph.world_modification_nodes import MoveBranch
from giskardpy.motion_control import MotionControl
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.world_entity import Body

# %% helpers


def _add_box_next_to_the_robot(world: World) -> Body:
    """
    :return: A box fixed to the root of `world`, half a metre in front of the origin.
    """
    box = Body(name=PrefixedName("box"))
    with world.modify_world():
        world.add_connection(
            FixedConnection(
                parent=world.root,
                child=box,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=0.5
                ),
            )
        )
    return box


# %% a motion built before the branch moved


def test_a_motion_built_before_a_body_was_moved_moves_it_with_its_new_parent(
    cylinder_bot_world: World,
):
    world = cylinder_bot_world
    box = _add_box_next_to_the_robot(world)
    robot = world.get_kinematic_structure_entity_by_name("bot")
    executor = StatechartExecutor(
        context=StatechartContext(world=world), extensions=[MotionControl()]
    )
    statechart = Statechart(context=executor.context)
    goal_pose = Pose.from_xyz_rpy(x=1.5, reference_frame=world.root)
    statechart.add_node(
        carry := Sequence(
            [
                CartesianPose(
                    root_link=world.root,
                    tip_link=robot,
                    goal_pose=Pose.from_xyz_rpy(x=0.2, reference_frame=world.root),
                    name="approach",
                ),
                MoveBranch(body=box, new_parent=robot),
                CartesianPose(
                    root_link=world.root,
                    tip_link=box,
                    goal_pose=goal_pose,
                    name="carry",
                ),
            ]
        )
    )
    statechart.add_node(EndMotion.when_true(carry))

    executor.compile(statechart)
    executor.tick_until_end()

    assert carry.life_cycle_state == LifeCycleValues.SUCCEEDED
    assert np.allclose(
        world.compute_forward_kinematics(world.root, box), goal_pose, atol=0.02
    )


def test_building_the_nodes_again_registers_no_further_variables(
    cylinder_bot_world: World,
):
    world = cylinder_bot_world
    box = _add_box_next_to_the_robot(world)
    robot = world.get_kinematic_structure_entity_by_name("bot")
    executor = StatechartExecutor(
        context=StatechartContext(world=world), extensions=[MotionControl()]
    )
    statechart = Statechart(context=executor.context)
    statechart.add_node(
        CartesianPose(
            root_link=world.root,
            tip_link=robot,
            goal_pose=Pose.from_xyz_rpy(x=0.2, reference_frame=world.root),
        )
    )
    executor.compile(statechart)
    variable_count = len(executor.context.float_variable_data.data)

    world.move_branch(box, robot)
    executor.tick()

    assert len(executor.context.float_variable_data.data) == variable_count


# %% standing still while the chart compiles again


def test_motion_control_stops_the_commanded_motion_before_the_chart_compiles_again(
    cylinder_bot_world: World,
):
    motion_control = MotionControl()
    executor = StatechartExecutor(
        context=StatechartContext(world=cylinder_bot_world),
        extensions=[motion_control],
    )
    cylinder_bot_world.state.velocities[:] = 1.0

    motion_control.before_recompile(executor)

    assert np.all(cylinder_bot_world.state.velocities == 0)
