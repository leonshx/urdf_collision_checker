"""Opt-in acceptance tests for hover names and clearing view selection."""

import os
from pathlib import Path

import pytest

URL = os.environ.get("COLLISION_E2E_URL")
pytestmark = pytest.mark.skipif(
    not URL, reason="Set COLLISION_E2E_URL for view interaction tests"
)


@pytest.fixture
def page():
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
            lambda r: r.url.endswith("/api/load") and r.request.method == "POST"
        ) as loaded:
            page.locator("#load").click()
        revision = loaded.value.json()["model"]["revision"]
        page.wait_for_function(
            "r => window.collisionStudio?.ready && window.collisionStudio.session.model.revision === r",
            arg=revision,
        )
        yield page
        assert not errors, errors
        browser.close()


def hover_object(page, index, name):
    point = page.evaluate("index => window.collisionStudio.projectObject(index)", index)
    page.mouse.move(point["x"], point["y"])
    page.wait_for_function(
        'name => { const tip=document.getElementById("part-tooltip"); return !tip.hidden && tip.dataset.link === name; }',
        arg=name,
    )
    return point


def blank_point(page):
    box = page.locator("#viewport").bounding_box()
    return dict(x=box["x"] + 25, y=box["y"] + box["height"] * 0.35)


def test_hover_names_and_visibility(page):
    point = hover_object(page, 6, "ELBOW")
    tooltip = page.locator("#part-tooltip")
    assert tooltip.text_content() == "LINK 部件ELBOW"
    assert tooltip.evaluate("(node) => getComputedStyle(node).pointerEvents") == "none"
    page.mouse.move(point["x"] + 2, point["y"] + 2)
    bounds = tooltip.bounding_box()
    view = page.locator("#viewport").bounding_box()
    assert (
        view["x"] <= bounds["x"]
        and bounds["x"] + bounds["width"] <= view["x"] + view["width"]
    )
    assert (
        view["y"] <= bounds["y"]
        and bounds["y"] + bounds["height"] <= view["y"] + view["height"]
    )
    hover_object(page, 0, "arm_base")  # Fixed/root components also report names.
    page.mouse.move(20, 20)
    assert tooltip.is_hidden()
    hover_object(page, 6, "ELBOW")
    page.mouse.down()
    assert tooltip.is_hidden()
    page.mouse.up()
    page.locator("#show-collision").uncheck()
    page.mouse.move(point["x"], point["y"])
    assert tooltip.is_hidden()  # Invisible collision meshes cannot be picked.
    page.locator("#show-visual").check()
    hover_object(page, 6, "ELBOW")
    page.mouse.move(**blank_point(page))
    assert tooltip.is_hidden()


def test_blank_click_clears_only_view_selection(page):
    cell = page.locator("#matrix td[data-id]").first
    cell.click(position={"x": 2, "y": 2})
    page.wait_for_function('document.querySelectorAll("td.selected").length === 2')
    before = page.request.get(URL + "/api/state").json()["session"]
    # Clicking a toolbar or part retains selection; moving the camera does too.
    page.locator("#fit").click()
    assert page.locator("td.selected").count() == 2
    point = hover_object(page, 0, "arm_base")
    page.mouse.click(point["x"], point["y"])
    assert page.locator("td.selected").count() == 2
    blank = blank_point(page)
    page.mouse.move(**blank)
    page.mouse.down()
    page.mouse.move(blank["x"] + 65, blank["y"] + 30, steps=10)
    page.mouse.up()
    assert page.locator("td.selected").count() == 2
    page.mouse.click(blank["x"], blank["y"])
    page.wait_for_function('document.querySelectorAll("td.selected").length === 0')
    assert page.locator("#pair-detail").text_content() == "点击矩阵中的一格查看详情。"
    base_opacity = float(page.locator("#opacity").input_value())
    opacities = page.evaluate("""() => {
        const values=[];
        window.collisionStudio.robot.traverse(node => {
            if(node.isMesh) for(const material of Array.isArray(node.material)?node.material:[node.material]) values.push(material.opacity);
        });
        return values;
    }""")
    assert opacities and all(abs(value - base_opacity) < 1e-6 for value in opacities)
    after = page.request.get(URL + "/api/state").json()["session"]
    assert after["pairs"] == before["pairs"]
    assert after["settings"] == before["settings"]
    assert after["model"]["pose"] == before["model"]["pose"]


def test_pending_checkbox_survives_pair_highlighting(page):
    # A view-only rerender must preserve edits whose server response is pending.
    pending = []
    page.route("**/api/pairs", lambda route: pending.append(route))
    cells = page.locator("#matrix td[data-id]")
    edited_id = cells.first.get_attribute("data-id")
    edited = page.locator(f'#matrix td[data-id="{edited_id}"] input')
    assert not edited.first.is_checked()
    edited.first.check()
    page.wait_for_function("true")  # Pump the intercepted request callback.
    assert len(pending) == 1
    other = page.locator(f'#matrix td[data-id]:not([data-id="{edited_id}"])').first
    other.click(position={"x": 2, "y": 2})
    assert edited.count() == 2
    assert all(box.is_checked() for box in edited.all())
    assert page.locator("td.selected").count() == 2
    pending.pop().continue_()
    page.wait_for_function(
        "id => window.collisionStudio.session.pairs.find(p => p.id === id).manual === false",
        arg=edited_id,
    )
    assert all(box.is_checked() for box in edited.all())
    # Highlight the edited pair and then its symmetric cell: neither edits it.
    for index in (0, 1):
        page.locator(f'#matrix td[data-id="{edited_id}"]').nth(index).click(
            position={"x": 2, "y": 2}
        )
        assert all(box.is_checked() for box in edited.all())
    page.unroute("**/api/pairs")
    edited.first.uncheck()
    page.wait_for_function(
        "id => window.collisionStudio.session.pairs.find(p => p.id === id).manual === true",
        arg=edited_id,
    )
    assert all(not box.is_checked() for box in edited.all())
