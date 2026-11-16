// Local layout preferences are independent of robot projects and analysis state.
const STORAGE_KEY = "collision-studio.layout.v1";
const MIN_VIEW_HEIGHT = 180;
const MIN_MATRIX_HEIGHT = 300;
const clamp = (value, lower, upper) => Math.min(upper, Math.max(lower, value));

export function setupLayout() {
  const root = document.querySelector("main");
  const workspace = root.querySelector(".workspace");
  const slider = document.getElementById("matrix-cell-size");
  const defaults = () => ({
    left: innerWidth <= 1100 ? 215 : 258,
    right: innerWidth <= 1100 ? 270 : 310,
    vertical: 0.57,
    cell: 26,
    label: 150,
  });
  const values = defaults();
  let lastTap = null;
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY));
    for (const key of Object.keys(values))
      if (typeof saved?.[key] === "number" && Number.isFinite(saved[key]))
        values[key] = saved[key];
  } catch {
    /* A disabled or invalid localStorage must not prevent robot loading. */
  }

  function limits(kind) {
    if (kind === "left")
      return [160, Math.max(160, root.clientWidth - values.right - 316)];
    if (kind === "right")
      return [220, Math.max(220, root.clientWidth - values.left - 316)];
    if (kind === "vertical") {
      const height = Math.max(
        MIN_VIEW_HEIGHT + MIN_MATRIX_HEIGHT,
        workspace.clientHeight - 8,
      );
      return [MIN_VIEW_HEIGHT / height, 1 - MIN_MATRIX_HEIGHT / height];
    }
    if (kind === "label") return [90, 400];
    return [20, 48];
  }
  function refresh() {
    for (const handle of root.querySelectorAll("[data-resize]")) {
      const kind = handle.dataset.resize,
        [min, max] = limits(kind);
      handle.setAttribute(
        "aria-valuemin",
        kind === "vertical" ? Math.round(min * 100) : Math.round(min),
      );
      handle.setAttribute(
        "aria-valuemax",
        kind === "vertical" ? Math.round(max * 100) : Math.round(max),
      );
      handle.setAttribute(
        "aria-valuenow",
        kind === "vertical"
          ? Math.round(values[kind] * 100)
          : Math.round(values[kind]),
      );
    }
  }
  function apply() {
    // Clamp both sidebars on window resize while reserving useful viewport width.
    values.left = clamp(
      values.left,
      160,
      Math.max(160, root.clientWidth - 536),
    );
    for (const kind of ["right", "left", "vertical", "cell", "label"])
      values[kind] = clamp(values[kind], ...limits(kind));
    root.style.setProperty("--left-panel-width", `${values.left}px`);
    root.style.setProperty("--right-panel-width", `${values.right}px`);
    root.style.setProperty("--matrix-cell-size", `${values.cell}px`);
    root.style.setProperty("--matrix-label-width", `${values.label}px`);
    workspace.style.gridTemplateRows = `minmax(${MIN_VIEW_HEIGHT}px, ${values.vertical}fr) 8px minmax(${MIN_MATRIX_HEIGHT}px, ${1 - values.vertical}fr)`;
    slider.value = values.cell;
    document.getElementById("matrix-cell-size-value").textContent =
      `${values.cell} px`;
    refresh();
  }
  function save() {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(values));
    } catch {
      /* Optional preferences only. */
    }
  }
  function set(kind, value) {
    values[kind] = clamp(value, ...limits(kind));
    apply();
  }
  function reset(kind) {
    set(kind, defaults()[kind]);
    save();
  }

  root.addEventListener("pointerdown", (event) => {
    const handle = event.target.closest("[data-resize]");
    if (!handle || event.button !== 0) return;
    event.preventDefault();
    const kind = handle.dataset.resize,
      start = values[kind];
    const origin = kind === "vertical" ? event.clientY : event.clientX;
    const span = Math.max(
      MIN_VIEW_HEIGHT + MIN_MATRIX_HEIGHT,
      workspace.clientHeight - 8,
    );
    let movement = 0;
    root.setPointerCapture(event.pointerId);
    document.body.classList.add("resizing-layout");
    document.body.style.setProperty(
      "--resize-cursor",
      kind === "vertical" ? "row-resize" : "col-resize",
    );
    const move = (e) => {
      const delta = (kind === "vertical" ? e.clientY : e.clientX) - origin;
      movement = Math.max(movement, Math.abs(delta));
      set(
        kind,
        start +
          (kind === "vertical"
            ? delta / span
            : kind === "right"
              ? -delta
              : delta),
      );
    };
    const finish = (endEvent) => {
      root.removeEventListener("pointermove", move);
      root.removeEventListener("pointerup", finish);
      root.removeEventListener("pointercancel", finish);
      root.removeEventListener("lostpointercapture", finish);
      if (root.hasPointerCapture(event.pointerId))
        root.releasePointerCapture(event.pointerId);
      document.body.classList.remove("resizing-layout");
      // Root pointer capture keeps drags alive across matrix rerenders, but
      // retargets browser click events. Recognize a double tap explicitly.
      if (endEvent.type === "pointerup" && movement < 3) {
        const now = performance.now();
        if (lastTap?.kind === kind && now - lastTap.time < 450) {
          reset(kind);
          lastTap = null;
        } else lastTap = { kind, time: now };
      } else lastTap = null;
      save();
    };
    root.addEventListener("pointermove", move);
    root.addEventListener("pointerup", finish);
    root.addEventListener("pointercancel", finish);
    root.addEventListener("lostpointercapture", finish);
  });
  root.addEventListener("dblclick", (event) => {
    const handle = event.target.closest("[data-resize]");
    if (handle) reset(handle.dataset.resize);
  });
  root.addEventListener("keydown", (event) => {
    const handle = event.target.closest("[data-resize]");
    if (!handle) return;
    const kind = handle.dataset.resize;
    if (event.key === "Home") {
      event.preventDefault();
      reset(kind);
      return;
    }
    const keys =
      kind === "vertical"
        ? ["ArrowUp", "ArrowDown"]
        : ["ArrowLeft", "ArrowRight"];
    if (!keys.includes(event.key)) return;
    event.preventDefault();
    const step = (event.shiftKey ? 20 : 5) * (event.key === keys[0] ? -1 : 1);
    set(
      kind,
      values[kind] +
        (kind === "vertical"
          ? step /
            Math.max(
              MIN_VIEW_HEIGHT + MIN_MATRIX_HEIGHT,
              workspace.clientHeight - 8,
            )
          : kind === "right"
            ? -step
            : step),
    );
    save();
  });
  slider.addEventListener("input", () => {
    set("cell", Number(slider.value));
    save();
  });
  window.addEventListener("resize", apply);
  apply();
  return { refresh };
}
