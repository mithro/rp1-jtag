#!/usr/bin/env python3
"""Tell fpgas.online-fpga-tools that this repository's main has moved.

A repository_dispatch is fire-and-forget by design: GitHub answers 204 whether
or not anything over there listens for the event type. That is how this
notification managed to report success for a day while doing nothing, so the
dispatch here is followed by a check that a run actually started.

Needs a token with `contents: write` (to dispatch) and `actions: read` (to see
the run) on fpgas-online/fpgas.online-fpga-tools.
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
EVENT_TYPE = "rp1jtag-updated"

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
    source = os.environ["GITHUB_REPOSITORY"]

    if not token:
        fail(
            "FPGA_TOOLS_DISPATCH_TOKEN is not set, so a push to main does not "
            f"reach {TARGET}. Create a fine-grained token scoped to that one "
            "repository with Contents: write and Actions: read, and add it as "
            "the repository secret FPGA_TOOLS_DISPATCH_TOKEN."
        )

    started = datetime.now(timezone.utc)
    try:
        api(
            f"repos/{TARGET}/dispatches",
            token,
            method="POST",
            body={
                "event_type": EVENT_TYPE,
                "client_payload": {"sha": sha, "repository": source},
            },
        )
    except urllib.error.HTTPError as error:
        fail(
            f"dispatching {EVENT_TYPE} to {TARGET} failed with HTTP "
            f"{error.code}: {error.read().decode(errors='replace').strip()}"
        )
    print(f"dispatched {EVENT_TYPE} for {sha}")

    # 204 only means GitHub accepted the event, not that it matched a trigger.
    while datetime.now(timezone.utc) - started < DEADLINE:
        time.sleep(POLL_INTERVAL)
        runs = api(
            f"repos/{TARGET}/actions/runs?event=repository_dispatch&per_page=20",
            token,
        )
        for run in runs.get("workflow_runs", []):
            created = datetime.fromisoformat(run["created_at"].replace("Z", "+00:00"))
            if created >= started.replace(microsecond=0):
                print(f"{TARGET} started {run['name']}: {run['html_url']}")
                return
        print("no run yet; waiting")

    fail(
        f"{TARGET} accepted the {EVENT_TYPE} event but started no run within "
        f"{int(DEADLINE.total_seconds())}s. Its workflows most likely have no "
        f"`on: repository_dispatch: types: [{EVENT_TYPE}]` trigger, so this "
        "notification is being discarded."
    )


if __name__ == "__main__":
    main()
