"""Run against local synthetic radar + map servers, never against hardware."""
import argparse
from pathlib import Path

from playwright.sync_api import sync_playwright, expect


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:18083/")
    parser.add_argument("--output", type=Path, default=Path("/tmp/hansel-browser"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path="/usr/bin/google-chrome", headless=True)
        for name, viewport in [("desktop", {"width": 1440, "height": 950}),
                               ("mobile", {"width": 390, "height": 844})]:
            page = browser.new_page(viewport=viewport)
            errors = []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.goto(args.url)
            expect(page.locator("#x")).not_to_have_text("--")
            page.wait_for_timeout(1600)
            assert page.locator("#mode-banner").is_visible()
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
            for selector in ("#zoom-in", "#zoom-out", "#fit-view", "#follow"):
                page.locator(selector).click()
            assert page.locator("#follow").get_attribute("aria-pressed") == "true"
            radar = page.frame_locator("#radar-frame")
            radar.locator("#radar-main-canvas").wait_for()
            assert radar.locator("#radar-status").inner_text() not in ("FAULT", "STALE", "HTTP LOST")
            blocked = radar.locator("#radar-main-canvas").evaluate("""() =>
              panel.deriveOperatorGuide({cells: [
                {forward_m:.2,lateral_m:-.15,probability:.95},
                {forward_m:.2,lateral_m:.15,probability:.95}
              ]}, {status:'blocked_near_field', points:[[0,0]]})
            """)
            assert blocked["points"] == []
            for context, selector in [(page, "#map"), (radar, "#radar-main-canvas")]:
                pixels = context.locator(selector).evaluate("""c => {
                  const a = c.getContext('2d').getImageData(0,0,c.width,c.height).data;
                  const colors = new Set();
                  for(let i=0;i<a.length;i+=40) colors.add(`${a[i]},${a[i+1]},${a[i+2]}`);
                  return colors.size;
                }""")
                assert pixels > 8, (selector, pixels)
            with page.expect_download() as download:
                page.locator("#export").click()
            saved = args.output / f"{name}.json"
            download.value.save_as(saved)
            page.locator("#file").set_input_files(saved)
            page.locator("#live").wait_for(state="visible")
            page.locator("#live").click()
            expect(page.locator("#x")).not_to_have_text("--")
            page.locator("#fit-view").click()
            page.wait_for_timeout(700)
            page.screenshot(path=str(args.output / f"{name}.png"), full_page=True)
            page.locator(".map-details summary").click()
            assert page.locator("#pose-age").is_visible()
            assert not errors, errors
            print(f"PASS {name}: layout, canvases, controls, snapshot round-trip")
            page.close()
        browser.close()


if __name__ == "__main__":
    main()
