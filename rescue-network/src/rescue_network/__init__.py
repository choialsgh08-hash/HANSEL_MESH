"""Disaster rescue-request mesh network package.

The same package powers three node roles selected via ``NODE_ROLE``:

* ``field``    — victim-facing web form + local SQLite + forwarder (Phase 1/2)
* ``relay``    — BATMAN-adv only, no application (Phase 5)
* ``receiver`` — rescue-team receive API + dashboard (Phase 2)

Phase 1 implements only the local ``field`` web application.
"""

__version__ = "0.1.0"
