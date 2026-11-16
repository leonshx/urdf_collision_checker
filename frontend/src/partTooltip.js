// Reuse the drag controller's visible-mesh hit result, avoiding a second raycast.
export function setupPartTooltip(canvas, viewport, getState) {
  const tooltip = document.createElement("div");
  tooltip.id = "part-tooltip";
  tooltip.setAttribute("role", "tooltip");
  tooltip.hidden = true;
  const caption = document.createElement("span");
  caption.textContent = "LINK 部件";
  const name = document.createElement("strong");
  tooltip.append(caption, name);
  document.body.append(tooltip);

  function hide() {
    tooltip.hidden = true;
    delete tooltip.dataset.link;
    canvas.removeAttribute("aria-describedby");
  }
  function move(event) {
    const state = getState(),
      link = state.drag?.hovered;
    if (
      !state.ready ||
      !link ||
      event.buttons ||
      state.drag.manipulatingLink ||
      document.body.classList.contains("resizing-layout")
    ) {
      hide();
      return;
    }
    name.textContent = link.name;
    tooltip.dataset.link = link.name;
    const bounds = viewport.getBoundingClientRect();
    tooltip.style.maxWidth = `${Math.max(40, Math.min(360, bounds.width - 16))}px`;
    tooltip.style.maxHeight = `${Math.max(40, bounds.height - 16)}px`;
    tooltip.hidden = false;
    canvas.setAttribute("aria-describedby", tooltip.id);
    const left = Math.max(
      bounds.left + 8,
      Math.min(event.clientX + 14, bounds.right - tooltip.offsetWidth - 8),
    );
    let top = event.clientY + 14;
    if (top + tooltip.offsetHeight > bounds.bottom - 8)
      top = event.clientY - tooltip.offsetHeight - 14;
    tooltip.style.left = `${left}px`;
    tooltip.style.top = `${Math.max(bounds.top + 8, top)}px`;
  }
  // Called after each controller is created so its mousemove raycast runs first.
  function bind(signal) {
    canvas.addEventListener("mousemove", move, { signal });
  }
  canvas.addEventListener("pointerleave", hide);
  canvas.addEventListener("pointerdown", hide, { capture: true });
  viewport.addEventListener("pointerover", (event) => {
    if (event.target !== canvas) hide();
  });
  window.addEventListener("blur", hide);
  window.addEventListener("resize", hide);
  window.addEventListener("scroll", hide, { capture: true });
  return { bind, hide };
}
