"""Opt-in browser checks for panel splitters, table widths and persistence."""

import os
from pathlib import Path

import pytest

URL = os.environ.get("COLLISION_E2E_URL")
pytestmark = pytest.mark.skipif(
    not URL, reason="Set COLLISION_E2E_URL for layout acceptance tests"
)


def test_resizable_panels_and_matrix(tmp_path):
    playwright = pytest.importorskip("playwright.sync_api")
    root = Path(__file__).resolve().parent.parent
    errors = []
    with playwright.sync_playwright() as p:
        browser = p.chromium.launch(
            headless=True,
            args=[
                "--no-sandbox",
                "--use-angle=swiftshader",
                "--enable-unsafe-swiftshader",
            ],
        )
        page = browser.new_page(viewport={"width": 1600, "height": 1000})
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.goto(URL)
        page.locator("#model-path").fill(
            str(root / "examples/mabi_mobile/urdf/mabi_mobile.urdf")
        )
        with page.expect_response(
            lambda r: r.url.endswith("/api/load") and r.request.method == "POST"
        ) as response:
            page.locator("#load").click()
        revision = response.value.json()["model"]["revision"]
        page.wait_for_function(
            "revision => window.collisionStudio?.ready && window.collisionStudio.session.model.revision === revision",
            arg=revision,
        )

        def drag(selector, dx=0, dy=0):
            box = page.locator(selector).bounding_box()
            x, y = box["x"] + box["width"] / 2, box["y"] + box["height"] / 2
            page.mouse.move(x, y)
            page.mouse.down()
            page.mouse.move(x + dx, y + dy, steps=10)
            page.mouse.up()

        def box(selector):
            return page.locator(selector).first.bounding_box()

        initial_view, initial_matrix = box("#viewport"), box(".matrix-panel")
        drag("#view-matrix-divider", dy=-100)
        assert box("#viewport")["height"] == pytest.approx(
            initial_view["height"] - 100, abs=3
        )
        assert box(".matrix-panel")["height"] == pytest.approx(
            initial_matrix["height"] + 100, abs=3
        )
        assert box("#viewport canvas")["height"] == pytest.approx(
            box("#viewport")["height"], abs=2
        )
        initial_width = box(".workspace")["width"]
        drag("#left-divider", dx=-50)
        assert box(".workspace")["width"] == pytest.approx(initial_width + 50, abs=3)
        drag("#right-divider", dx=50)
        assert box(".workspace")["width"] == pytest.approx(initial_width + 100, abs=3)
        assert box("#viewport canvas")["width"] == pytest.approx(
            box("#viewport")["width"], abs=2
        )
        table_width = box("#matrix table")["width"]
        cell_width = box("#matrix td[data-id]")["width"]
        assert table_width < box(".workspace")["width"] * 0.7  # No stretch-to-fill.
        page.locator("#matrix-cell-size").focus()
        page.keyboard.press("Home")
        for _ in range(18):
            page.keyboard.press("ArrowRight")
        assert box("#matrix table")["width"] > table_width + 60
        assert box("#matrix td[data-id]")["width"] > cell_width + 8
        label_width = box("#matrix thead th:first-child")["width"]
        drag(".matrix-column-resizer", dx=70)
        assert box("#matrix thead th:first-child")["width"] == pytest.approx(
            label_width + 70, abs=3
        )
        geometry = {
            selector: box(selector)
            for selector in ["#viewport", ".workspace", "#matrix table"]
        }
        # Layout choices survive a reload but do not change analysis parameters.
        page.reload()
        page.wait_for_function("window.collisionStudio?.ready")
        for selector, expected in geometry.items():
            actual = box(selector)
            assert actual["width"] == pytest.approx(expected["width"], abs=3)
            assert actual["height"] == pytest.approx(expected["height"], abs=3)
        assert page.locator("#matrix-cell-size").input_value() == "38"
        page.locator("#view-matrix-divider").focus()
        h = box("#viewport")["height"]
        page.keyboard.press("ArrowDown")
        assert box("#viewport")["height"] == pytest.approx(h + 5, abs=2)
        page.locator("#view-matrix-divider").dblclick()
        assert box("#viewport")["height"] == pytest.approx(
            initial_view["height"], abs=3
        )
        # A narrow window preserves scrollable matrix contents and usable panels.
        page.set_viewport_size({"width": 1000, "height": 800})
        page.wait_for_function(
            """() => {
            const viewport = document.getElementById('viewport').getBoundingClientRect();
            const canvas = document.querySelector('#viewport canvas').getBoundingClientRect();
            return Math.abs(viewport.width-canvas.width) < 2 && Math.abs(viewport.height-canvas.height) < 2;
        }""",
            timeout=3000,
        )
        assert box("#viewport canvas")["width"] == pytest.approx(
            box("#viewport")["width"], abs=2
        )
        assert page.locator("#matrix").evaluate(
            "(node) => node.scrollWidth > node.clientWidth"
        )
        assert not errors, errors
        browser.close()
