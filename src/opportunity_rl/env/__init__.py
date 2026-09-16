"""Milestone M5: the Gymnasium-facing RL environment layer.

`DynamicGraspEnv` (env.py) wraps a `SimulationInterface` backend behind
the standard Gymnasium API, using the fixed observation schema
(observation.py) and config-parameterized reward (reward.py) frozen in
docs/rl-formulation.md. This is the layer the RL algorithms (M6+) train
against; it never imports ROS2 or Gazebo, only the `SimulationInterface`
Protocol -- architecture.md's single most important structural boundary.
"""
