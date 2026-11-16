"""Opt-in real-browser acceptance test against an already running local server."""

import os
from pathlib import Path

import pytest

URL = os.environ.get("COLLISION_E2E_URL")
pytestmark = pytest.mark.skipif(
    not URL, reason="Set COLLISION_E2E_URL to opt into browser acceptance tests"
)


def test_full_browser_workflow(tmp_path):
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
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.goto(URL)
        page.locator("#model-path").fill(
            str(root / "examples/mabi_mobile/urdf/mabi_mobile.urdf")
        )
        with page.expect_response(
            lambda response: (
                response.url.endswith("/api/load") and response.request.method == "POST"
            )
        ) as loaded:
            page.locator("#load").click()
        revision = loaded.value.json()["model"]["revision"]
        page.wait_for_function(
            "revision => window.collisionStudio?.ready && window.collisionStudio.session.model.revision === revision",
            arg=revision,
        )
        assert page.locator("#matrix td[data-id]").count() == 42
        # Real pointer interaction with a visible collision body, not a slider.
        before = page.evaluate(
            "Object.fromEntries(Object.entries(window.collisionStudio.robot.joints).map(([n,j])=>[n,j.angle]))"
        )
        point = page.evaluate("window.collisionStudio.projectObject(10)")
        page.mouse.move(point["x"], point["y"])
        page.mouse.down()
        page.mouse.move(point["x"] + 45, point["y"] + 45, steps=15)
        page.mouse.up()
        page.wait_for_timeout(250)
        after = page.evaluate(
            "Object.fromEntries(Object.entries(window.collisionStudio.robot.joints).map(([n,j])=>[n,j.angle]))"
        )
        assert any(
            abs(after[k] - v) > 0.0001
            for k, v in before.items()
            if isinstance(v, (int, float))
        )
        # Select a matrix cell without altering its checkbox.
        cell = page.locator("#matrix td[data-id]").first
        cell.click(position={"x": 2, "y": 2})
        page.wait_for_selector("#current-distance")
        page.wait_for_function(
            "document.getElementById('current-distance').textContent.includes(' m')"
        )
        assert page.locator("#matrix td.selected").count() == 2
        page.locator("#samples").fill("200")
        page.locator("#samples").press("Tab")
        page.wait_for_function(
            "window.collisionStudio.session.settings.samples === 200"
        )
        page.locator("#analyze").click()
        page.wait_for_function("window.collisionStudio.session.stale === false")
        page.locator("#top-k").fill("2")
        page.locator("#preview-topk").click()
        page.wait_for_function(
            "document.getElementById('topk-report').textContent.includes('→')"
        )
        page.once("dialog", lambda dialog: dialog.accept())
        page.locator("#apply-topk").click()
        page.wait_for_function(
            "window.collisionStudio.session.pairs.filter(p=>p.enabled).length === 2"
        )
        page.locator("#undo").click()
        page.wait_for_function(
            "window.collisionStudio.session.pairs.filter(p=>p.enabled).length > 2"
        )
        page.locator("#benchmark-samples").fill("10")
        page.locator("#benchmark").click()
        page.wait_for_function("window.collisionStudio.session.benchmark !== null")
        assert page.locator("#benchmark-report").text_content().find("加速") >= 0
        # Per-pair override forces explicit acknowledgement for original OCS2 INFO.
        checkbox = page.locator("#matrix td.selected input").first
        if checkbox.is_checked():
            checkbox.click()
            page.wait_for_function(
                'window.collisionStudio.session.pairs.find(p=>p.id === document.querySelector("td.selected").dataset.id).enabled'
            )
        page.locator("#pair-distance").fill("0.08")
        page.locator("#pair-distance").press("Tab")
        page.wait_for_function("window.collisionStudio.session.stale === true")
        output = tmp_path / "config"
        dialogs = []

        def handle_dialog(dialog):
            dialogs.append(dialog.message)
            dialog.accept(str(output) if dialog.type == "prompt" else None)

        page.on("dialog", handle_dialog)
        page.locator("#export").click()
        page.wait_for_function(
            "document.getElementById('toast').textContent.includes('已生成')"
        )
        assert (output / "self_collision.yaml").is_file()
        assert any("INFO" in message and "全局" in message for message in dialogs)
        # Screenshot is a local artifact for visual QA; never committed.
        (root / "test-results").mkdir(exist_ok=True)
        page.screenshot(path=str(root / "test-results/studio.png"))
        # A collision-only model must also keep mimic geometry aligned while editing.
        page.locator("#model-path").fill(str(root / "tests/fixtures/sample.urdf"))
        with page.expect_response(
            lambda response: (
                response.url.endswith("/api/load") and response.request.method == "POST"
            )
        ) as loaded:
            page.locator("#load").click()
        revision = loaded.value.json()["model"]["revision"]
        page.wait_for_function(
            "revision => window.collisionStudio.ready && window.collisionStudio.session.model.revision === revision",
            arg=revision,
        )
        page.locator('input[type="number"][data-joint-value="extension"]').fill("0.4")
        page.locator('input[type="number"][data-joint-value="extension"]').press("Tab")
        page.wait_for_function(
            "Math.abs(window.collisionStudio.robot.joints.finger_mimic.angle - 0.02) < 1e-8"
        )
        response = page.request.post(
            URL + "/api/pose", data={"pose": {"extension": 0.4, "swing": 0.0}}
        )
        assert response.ok
        delta = page.evaluate(
            """frames => {
            window.collisionStudio.robot.updateMatrixWorld(true);
            let error=0;
            for(const [name, rows] of Object.entries(frames)) {
                const actual=window.collisionStudio.robot.links[name].matrixWorld.elements;
                rows.forEach((row,i)=>row.forEach((value,j)=>error=Math.max(error,Math.abs(value-actual[j*4+i]))));
            }
            return error;
        }""",
            response.json()["frames"],
        )
        assert delta < 1e-5
        assert not errors, errors
        browser.close()
