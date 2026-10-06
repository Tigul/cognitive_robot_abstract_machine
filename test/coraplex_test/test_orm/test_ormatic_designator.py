import pytest

from krrood.ormatic.data_access_objects.helper import to_dao
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.execution_environment import simulated_robot
from coraplex.orm.ormatic_interface import *  # type: ignore
from coraplex.robot_plans.actions.composite.transporting import TransportAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from coraplex.plans.plan_execution import PlanExecutor
from cramph.composites import Sequence
from cramph.node import StatechartNode
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk


@pytest.fixture()
def simple_plan(immutable_model_world):
    world, robot_view, context = immutable_model_world

    plan = Sequence(
        [
            NavigateAction(
                Pose.from_xyz_quaternion(
                    1.6, 1.9, 0, 0, 0, 0, 1, reference_frame=world.root
                )
            ),
            MoveTorsoAction(TorsoState.HIGH),
            ParkArmsAction(Arms.BOTH),
        ]
    )
    return plan


def _stored_and_loaded(session, plan: StatechartNode) -> StatechartNode:
    """
    :return: `plan`, written to the database and read back.
    """
    dao = to_dao(plan)
    session.add(dao)
    session.commit()
    database_id = dao.database_id
    session.expunge_all()
    return session.get(type(dao), database_id).from_dao()


def _executed(plan: StatechartNode, context: Context) -> None:
    """
    Execute `plan` in simulation.
    """
    executor = PlanExecutor(context)
    with simulated_robot:
        executor.compile(plan)
        executor.execute()


def test_a_performed_plan_is_read_back_with_its_steps(
    coraplex_testing_session, immutable_model_world, simple_plan
):
    world, robot_view, context = immutable_model_world
    _executed(simple_plan, context)

    recreated_plan = _stored_and_loaded(coraplex_testing_session, simple_plan)

    assert type(recreated_plan) is Sequence
    assert [type(step) for step in recreated_plan.nodes] == [
        type(step) for step in simple_plan.nodes
    ]
    assert recreated_plan.nodes[1].torso_state == simple_plan.nodes[1].torso_state


@pytest.fixture
def complex_plan(mutable_model_world):
    world, robot_view, context = mutable_model_world

    plan = TransportAction(
        object_designator=world.get_semantic_annotations_by_type(Milk)[0],
        target_location=Pose.from_xyz_quaternion(
            2.4, 2.8, 1, 0, 0, 0, 1, reference_frame=world.root
        ),
        arm=Arms.LEFT,
        grasp_description=GraspDescription(
            ApproachDirection.LEFT,
            VerticalAlignment.NoAlignment,
            robot_view.left_arm.end_effector,
        ),
    )

    return plan


def test_a_performed_transport_is_read_back(
    coraplex_testing_session, mutable_model_world, complex_plan
):
    world, robot_view, context = mutable_model_world
    _executed(complex_plan, context)

    recreated_plan = _stored_and_loaded(coraplex_testing_session, complex_plan)

    assert type(recreated_plan) is TransportAction
    assert recreated_plan.arm == complex_plan.arm
