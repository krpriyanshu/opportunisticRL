"""Run-provenance tests.

The failure these guard against is specific and was live in every recorded
metadata file: `git rev-parse HEAD` in a repository with no commits exits
128 *and prints the literal string "HEAD" to stdout*. Code that ignored
the return code recorded `"git_commit": "HEAD"` -- a value that looks like
provenance and carries none.
"""

import subprocess

from opportunity_rl.provenance import UNKNOWN_COMMIT, git_commit, git_is_dirty


def _git(*args, cwd):
    subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True)


def _init_repo(path):
    path.mkdir(parents=True, exist_ok=True)
    _git("init", "-q", cwd=path)
    _git("config", "user.email", "test@example.com", cwd=path)
    _git("config", "user.name", "test", cwd=path)
    return path


def test_repository_with_no_commits_reports_unknown_not_the_word_head(tmp_path):
    repo = _init_repo(tmp_path / "empty")
    assert git_commit(repo) == UNKNOWN_COMMIT


def test_returns_the_real_sha_once_a_commit_exists(tmp_path):
    repo = _init_repo(tmp_path / "withcommit")
    (repo / "f.txt").write_text("hello")
    _git("add", "f.txt", cwd=repo)
    _git("commit", "-qm", "initial", cwd=repo)

    expected = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()

    commit = git_commit(repo)
    assert commit == expected
    assert len(commit) == 40
    assert commit != UNKNOWN_COMMIT


def test_non_repository_reports_unknown(tmp_path):
    plain = tmp_path / "not-a-repo"
    plain.mkdir()
    assert git_commit(plain) == UNKNOWN_COMMIT


def test_dirty_flag_tracks_uncommitted_tracked_changes(tmp_path):
    repo = _init_repo(tmp_path / "dirty")
    (repo / "f.txt").write_text("hello")
    _git("add", "f.txt", cwd=repo)
    _git("commit", "-qm", "initial", cwd=repo)
    assert git_is_dirty(repo) is False

    (repo / "f.txt").write_text("modified")
    assert git_is_dirty(repo) is True


def test_dirty_flag_is_none_when_undeterminable(tmp_path):
    plain = tmp_path / "not-a-repo"
    plain.mkdir()
    assert git_is_dirty(plain) is None
