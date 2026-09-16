"""Milestone M6+: RL algorithms. B2 (standard reactive RL) is a small,
self-contained PPO (ppo.py) depending only on torch -- per
docs/rl-formulation.md's Framework decision, which explicitly permits "a
small custom PPO" over pulling in Stable-Baselines3, keeping the
dependency footprint minimal and the algorithm legible/inspectable. The
`RLAlgorithm` Protocol (interface.py) is the frozen train/evaluate/
checkpoint contract from rl-formulation.md's Interfaces section.
"""
