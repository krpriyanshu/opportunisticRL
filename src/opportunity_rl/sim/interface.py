"""The frozen `SimulationInterface` from docs/rl-formulation.md's
Interfaces section, as an actual `typing.Protocol` -- until now this
existed only as a spec in that doc. Defining it in code lets mypy check
that any backend (this milestone's FastSimulator, and a future
ROS2/Gazebo-backed implementation) actually satisfies it structurally,
rather than each backend's conformance being asserted by hand.

Per docs/rl-formulation.md's own governance: this file is the frozen
interface; if implementation ever reveals a real problem with it, the
fix is a deliberate, noted edit here (and in rl-formulation.md) -- not a
silent divergence in one backend.
"""

from __future__ import annotations

from typing import Protocol

from opportunity_rl.sim.state import EnvironmentState, RobotState, TargetState


class SimulationInterface(Protocol):
    def reset(self, seed: int | None = None) -> None: ...
    def step(self, dt: float) -> None: ...
    def get_robot_state(self) -> RobotState: ...
    def get_target_state(self) -> TargetState: ...
    def get_environment_state(self) -> EnvironmentState: ...
    def apply_action(self, action: int) -> None: ...


# The action space from docs/rl-formulation.md's action table. These live
# here, alongside SimulationInterface.apply_action, because they are part
# of that frozen contract -- not an implementation detail of any one
# backend. (They were previously defined in fast_simulator.py, which made
# the algorithms layer import a concrete backend just to name an action.)
WAIT, APPROACH, ATTEMPT_GRASP, ABORT = 0, 1, 2, 3
ACTIONS: tuple[int, ...] = (WAIT, APPROACH, ATTEMPT_GRASP, ABORT)
