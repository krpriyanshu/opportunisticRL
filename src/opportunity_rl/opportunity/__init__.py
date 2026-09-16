"""Milestone M8: the opportunity mechanism. `estimator.py` implements the
deterministic/analytic `OpportunityEstimator` from
docs/rl-formulation.md's Future opportunity representation section --
representation (2), a future-opportunity vector [O(t+d1), ..., O(t+dN)],
computed by extrapolating the CURRENT (and short-past) relative state
forward. It never reads the simulator's true future trajectory
(confound register item #7 / rl-formulation.md's binding leakage rule);
extrapolation from current+past state is the only permitted input.
"""
