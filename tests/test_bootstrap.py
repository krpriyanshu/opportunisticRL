"""M0 smoke test: the package installs, imports, and exposes a version.

This is deliberately the only test at M0 -- there is no RL, environment,
or robotics code yet to test. It exists to validate the packaging/tooling
bootstrap itself (Milestone M0's Definition of Done), not any research
logic.
"""

import re

import opportunity_rl


def test_package_imports() -> None:
    assert opportunity_rl is not None


def test_version_is_a_valid_semver_like_string() -> None:
    assert re.match(r"^\d+\.\d+\.\d+", opportunity_rl.__version__)
