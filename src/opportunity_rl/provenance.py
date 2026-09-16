"""Run provenance for experiment metadata (rl-formulation.md Section 29).

Every reported experiment records the commit it ran at, so a number in a
write-up can be traced back to the exact code that produced it. That is
only worth anything if the recorded value is either a real commit or an
honest admission that there wasn't one -- a plausible-looking placeholder
is worse than nothing, because it looks like provenance while carrying
none.

This module exists because a previous inline implementation did exactly
that: it ran `git rev-parse HEAD` without checking the return code, and
on a repository with no commits yet that command exits 128 while printing
the literal string "HEAD" to stdout. Every metadata file written before
this fix therefore recorded `"git_commit": "HEAD"`.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

UNKNOWN_COMMIT = "unknown"
_SHA_RE = re.compile(r"^[0-9a-f]{40}$")


def git_commit(repo_root: Path | str, timeout_s: float = 5.0) -> str:
    """The full SHA of `repo_root`'s current HEAD, or `UNKNOWN_COMMIT`.

    Returns UNKNOWN_COMMIT -- never a placeholder that could be mistaken
    for a revision -- when git is missing, the directory is not a
    repository, or the repository has no commits yet."""
    try:
        result = subprocess.run(
            ["git", "-C", str(repo_root), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.SubprocessError):
        return UNKNOWN_COMMIT

    if result.returncode != 0:
        return UNKNOWN_COMMIT
    commit = result.stdout.strip()
    # Belt and braces: only accept something that is actually SHA-shaped,
    # so no future git version's stdout quirk can slip a non-revision
    # through on a zero exit code.
    return commit if _SHA_RE.match(commit) else UNKNOWN_COMMIT


def git_is_dirty(repo_root: Path | str, timeout_s: float = 5.0) -> bool | None:
    """True if `repo_root` has uncommitted tracked changes, None if that
    can't be determined. A run recorded against a commit whose tree was
    already modified is not reproducible from that commit, so the metadata
    says so rather than letting the commit hash imply more than it can."""
    try:
        result = subprocess.run(
            ["git", "-C", str(repo_root), "status", "--porcelain", "--untracked-files=no"],
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.SubprocessError):
        return None

    if result.returncode != 0:
        return None
    return bool(result.stdout.strip())
