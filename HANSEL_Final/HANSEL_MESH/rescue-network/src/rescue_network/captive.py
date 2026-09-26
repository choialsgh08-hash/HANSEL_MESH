"""Captive-portal routes for the field app (Phase 6, opt-in).

When a phone joins the AP, the OS probes a known URL to detect a paywall. With
DNS wild-carded to the AP (``configure-ap.sh`` with ``AP_CAPTIVE=1``) those
probes hit us; here we answer them so the phone pops the rescue form
automatically instead of the user having to type the address.

Enabled only when ``CAPTIVE_PORTAL=1`` so normal dev/tests are unaffected.
"""

from __future__ import annotations

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, RedirectResponse

# Apple expects the literal "Success" page when online; returning anything else
# makes iOS show the "Log In" sheet, which opens this page.
_PORTAL_HTML = (
    "<!DOCTYPE html><html lang='ko'><head><meta charset='utf-8'>"
    "<meta http-equiv='refresh' content='0; url=/'></head>"
    "<body>구조 요청 페이지로 이동합니다… <a href='/'>여기를 누르세요</a>.</body></html>"
)

# OS probe URLs that should pop the portal.
_APPLE_PATHS = ("/hotspot-detect.html", "/library/test/success.html")
_REDIRECT_PATHS = (
    "/generate_204",  # Android
    "/gen_204",
    "/redirect",  # Windows
    "/connecttest.txt",
    "/ncsi.txt",
)


def register(app: FastAPI, redirect_to: str = "/") -> None:
    """Attach probe endpoints + a GET catch-all that redirects to the form.

    Must be called AFTER the real routes are registered so they keep priority;
    the ``/{path:path}`` catch-all is added last and only matches leftovers.
    """

    def portal() -> HTMLResponse:
        return HTMLResponse(_PORTAL_HTML)

    def redirect() -> RedirectResponse:
        return RedirectResponse(redirect_to, status_code=302)

    for path in _APPLE_PATHS:
        app.add_api_route(path, portal, methods=["GET"], include_in_schema=False)
    for path in _REDIRECT_PATHS:
        app.add_api_route(path, redirect, methods=["GET"], include_in_schema=False)

    def catch_all(path: str) -> RedirectResponse:
        return RedirectResponse(redirect_to, status_code=302)

    app.add_api_route("/{path:path}", catch_all, methods=["GET"], include_in_schema=False)
