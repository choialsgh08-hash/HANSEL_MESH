"""Application configuration via Pydantic Settings.

All settings are read from environment variables (optionally an ``.env`` file).
Secrets (``RESCUE_SHARED_TOKEN``) are never logged; see ``__repr__`` handling
provided by ``SecretStr``.
"""

from __future__ import annotations

from enum import Enum
from functools import lru_cache
from pathlib import Path

from pydantic import SecretStr
from pydantic_settings import BaseSettings, SettingsConfigDict


class NodeRole(str, Enum):
    """Role a node plays in the mesh."""

    FIELD = "field"
    RELAY = "relay"
    RECEIVER = "receiver"


class Settings(BaseSettings):
    """Runtime configuration shared by every node role.

    Only the fields needed by Phase 1 (``field`` web app + local storage) are
    exercised yet; the remaining fields are defined so later phases (forwarder,
    receiver, network scripts) can reuse the same object without a schema churn.
    """

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        case_sensitive=False,
    )

    # ---- identity / role --------------------------------------------------
    node_role: NodeRole = NodeRole.FIELD
    node_id: str = "node-01"

    # ---- victim AP (configured by scripts/configure-ap.sh; here for reference)
    ap_interface: str = "wlan0"
    ap_address: str = "192.168.10.1/24"

    # ---- delivery (Phase 2) ----------------------------------------------
    # The receiver is reached over the EXISTING HANSEL B.A.T.M.A.N. mesh
    # (bat0, 192.168.50.0/24). rescue-network does not set up its own mesh;
    # bring the mesh up with the repo's scripts/*mesh*.sh, then point this at
    # the receiver node's bat0 IP (base node = 192.168.50.1 by convention).
    receiver_url: str = "http://192.168.50.1:8080"
    rescue_shared_token: SecretStr = SecretStr("change-me")

    # ---- enhanced auth (Phase 6) -----------------------------------------
    # When true, require an HMAC signature instead of the plain shared token.
    require_signature: bool = False
    # Accepted clock skew for signature timestamps (replay window).
    signature_max_skew_seconds: float = 300.0

    # ---- web server -------------------------------------------------------
    web_host: str = "0.0.0.0"
    web_port: int = 80
    # Captive portal (Phase 6, opt-in): answer OS probes + redirect stray URLs
    # to the rescue form so phones pop it automatically.
    captive_portal: bool = False

    # ---- forwarder / delivery (Phase 2) ----------------------------------
    # Seconds the forwarder sleeps between polling passes.
    forwarder_poll_interval_seconds: float = 2.0
    # Per-request HTTP timeout when posting to the receiver.
    delivery_timeout_seconds: float = 10.0
    # Max requests claimed per polling pass.
    forwarder_batch_size: int = 20
    # A request stuck in "sending" longer than this is recovered to "pending"
    # (guards against a forwarder that crashed mid-attempt).
    stale_sending_seconds: float = 120.0

    # ---- monitoring / alerting (Phase 6) ---------------------------------
    # The forwarder warns if the oldest pending request is older than this.
    alert_pending_age_seconds: float = 600.0
    # Optional webhook URL to POST alerts to (empty = log only).
    alert_webhook: str = ""

    # ---- storage ----------------------------------------------------------
    data_dir: Path = Path("./data")

    @property
    def field_db_path(self) -> Path:
        """Absolute path to the field node's SQLite database file."""
        return self.data_dir / "field.db"

    @property
    def receiver_db_path(self) -> Path:
        """Absolute path to the receiver node's SQLite database file."""
        return self.data_dir / "receiver.db"

    @property
    def field_database_url(self) -> str:
        """SQLAlchemy URL for the field node's DB (web server + forwarder)."""
        return f"sqlite:///{self.field_db_path.as_posix()}"

    @property
    def receiver_database_url(self) -> str:
        """SQLAlchemy URL for the receiver node's DB."""
        return f"sqlite:///{self.receiver_db_path.as_posix()}"

    @property
    def receiver_receive_url(self) -> str:
        """Full URL the forwarder posts rescue requests to."""
        return self.receiver_url.rstrip("/") + "/api/rescue/receive"


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    """Return a cached ``Settings`` instance.

    Cached so the whole process shares one config object. Tests that need a
    custom config call ``get_settings.cache_clear()`` after setting env vars.
    """
    return Settings()
