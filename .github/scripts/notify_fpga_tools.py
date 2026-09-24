#!/usr/bin/env python3
"""Tell fpgas.online-fpga-tools that this repository's main has moved.

Dispatching a workflow is fire-and-forget by design: GitHub answers 204 without
promising a run. That is how this notification managed to report success for a
day while doing nothing, so the dispatch here is followed by a check that a run
actually started.

This calls the workflow-dispatch endpoint rather than the repository-dispatch
one. Both start CI over there; only repository_dispatch also demands
`contents: write`, which on that repository means the power to rewrite
`debian/` and publish arbitrary packages to the fleet. `actions: write` is the
smaller grant that does the same job, and it needs nothing added on their side
because the target workflow already offers `workflow_dispatch`.
"""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timedelta, timezone

API = "https://api.github.com"
TARGET = "fpgas-online/fpgas.online-fpga-tools"

# The bump-verify-promote job: it resolves this repository's head itself, so it
# needs no inputs from us. It declares none, and sending any would be a 422.
WORKFLOW = "daily.yml"

# How long to wait for the dispatch to show up as a run. GitHub queues the run
# before it starts, so this is queue latency, not build time.
DEADLINE = timedelta(seconds=120)
POLL_INTERVAL = 10


def api(path: str, token: str, method: str = "GET", body: dict | None = None):
    request = urllib.request.Request(
        f"{API}/{path}",
        method=method,
        data=json.dumps(body).encode() if body is not None else None,
        headers={
            "Authorization": f"Bearer {token}",
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
            "Content-Type": "application/json",
        },
    )
    with urllib.request.urlopen(request) as response:
        payload = response.read()
    return json.loads(payload) if payload else None


def fail(message: str) -> None:
    print(f"::error::{message}")
    sys.exit(1)


def main() -> None:
    token = os.environ.get("GH_TOKEN", "")
    sha = os.environ["SHA"]

    if not token:
        fail(
            "FPGA_TOOLS_DISPATCH_TOKEN is not set, so a push to main does not "
            f"reach {TARGET}. Create a fine-grained token scoped to that one "
            "repository with Actions: write, and add it as the repository "
            "secret FPGA_TOOLS_DISPATCH_TOKEN."
        )

    # Ask rather than assume: a dispatch names the ref it runs on, and getting
    # it wrong is a 404 that reads like a missing workflow. Metadata: read is
    # granted to every fine-grained token, so this costs no extra permission.
    try:
        branch = api(f"repos/{TARGET}", token)["default_branch"]
    except urllib.error.HTTPError as error:
        fail(
            f"cannot read {TARGET}: HTTP {error.code}. The token is missing, "
            f"expired, or not scoped to that repository. "
            f"{error.read().decode(errors='replace').strip()}"
        )

    started = datetime.now(timezone.utc)
    try:
        api(
            f"repos/{TARGET}/actions/workflows/{WORKFLOW}/dispatches",
            token,
            method="POST",
            body={"ref": branch},
        )
    except urllib.error.HTTPError as error:
        detail = error.read().decode(errors="replace").strip()
        if error.code == 404:
            fail(
                f"{TARGET} has no {WORKFLOW} on {branch}, or the token cannot "
                f"see it. If that workflow was renamed, update WORKFLOW in "
                f"this script. {detail}"
            )
        if error.code == 403:
            fail(
                f"{TARGET} refused to run {WORKFLOW}: it is disabled, it no "
                f"longer declares `on: workflow_dispatch`, or the token lacks "
                f"Actions: write. {detail}"
            )
        fail(f"dispatching {WORKFLOW} to {TARGET} failed with HTTP {error.code}: {detail}")
    print(f"dispatched {WORKFLOW} on {branch} for {sha}")

    # 204 only means GitHub accepted the request, not that a run exists.
    while datetime.now(timezone.utc) - started < DEADLINE:
        time.sleep(POLL_INTERVAL)
        runs = api(
            f"repos/{TARGET}/actions/workflows/{WORKFLOW}/runs"
            "?event=workflow_dispatch&per_page=20",
            token,
        )
        for run in runs.get("workflow_runs", []):
            created = datetime.fromisoformat(run["created_at"].replace("Z", "+00:00"))
            if created >= started.replace(microsecond=0):
                print(f"{TARGET} started {run['name']}: {run['html_url']}")
                return
        print("no run yet; waiting")

    fail(
        f"{TARGET} accepted the request to run {WORKFLOW} but started no run "
        f"within {int(DEADLINE.total_seconds())}s, so this notification was "
        "discarded. Check whether that workflow is disabled."
    )


if __name__ == "__main__":
    main()
