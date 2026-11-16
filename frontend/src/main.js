import "./style.css";
import { setupLayout } from "./layout.js";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import URDFLoader from "urdf-loader";
import { URDFAdapter } from "./vendor/robot_viewer/adapters/URDFAdapter.js";
import { PointerJointDragControls } from "./vendor/robot_viewer/utils/JointDragControls.js";

const $ = (id) => document.getElementById(id);
const layout = setupLayout();
const esc = (value) =>
  String(value ?? "").replace(
    /[&<>"']/g,
    (c) =>
      ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[
        c
      ],
  );
const state = {
  session: null,
  selected: null,
  revision: null,
  robot: null,
  drag: null,
  pose: {},
  priorPose: null,
  ready: false,
  viewError: null,
  job: null,
};
let toastTimer,
  poseTimer,
  poseSequence = 0,
  matrixPairs = [],
  browseParent = ".",
  pendingTopk = null;
function toast(message, error = false) {
  $("toast").textContent = message;
  $("toast").className = error ? "error" : "";
  $("toast").style.display = "block";
  clearTimeout(toastTimer);
  toastTimer = setTimeout(
    () => ($("toast").style.display = "none"),
    error ? 9000 : 5000,
  );
}
async function api(path, body, method = "POST") {
  const response = await fetch(`/api${path}`, {
    method: body === undefined ? "GET" : method,
    headers: body === undefined ? {} : { "Content-Type": "application/json" },
    body:
      body === undefined
        ? undefined
        : JSON.stringify({ ...body, revision: state.session?.model.revision }),
  });
  const value = await response.json();
  if (!response.ok) {
    const e = new Error(
      typeof value.detail === "string"
        ? value.detail
        : JSON.stringify(value.detail),
    );
    e.code = value.code;
    throw e;
  }
  return value;
}
function on(id, action, event = "click") {
  $(id).addEventListener(event, async (e) => {
    try {
      await action(e);
    } catch (err) {
      toast(err.message, true);
    }
  });
}
let changes = Promise.resolve();
function mutate(path, body, method = "POST") {
  const task = changes
    .catch(() => {})
    .then(async () => {
      const s = await api(path, body, method);
      await accept(s);
      return s;
    });
  changes = task;
  return task;
}
async function settings(patch) {
  return mutate("/settings", { settings: patch }, "PATCH");
}
const pair = () => state.session?.pairs.find((p) => p.id === state.selected);
function busy() {
  return ["running", "cancelling"].includes(state.job?.status);
}

// Z-up in both render and compute; no hidden root conversion.
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 1000);
camera.up.set(0, 0, 1);
camera.position.set(3, -4, 2.5);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
$("viewport").appendChild(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.target.set(0, 0, 0.6);
scene.add(new THREE.HemisphereLight(0xe1f3ff, 0x485957, 2.2));
const light = new THREE.DirectionalLight(0xffecd5, 3);
light.position.set(3, -2, 5);
scene.add(light);
const grid = new THREE.GridHelper(20, 80, 0x45606c, 0x2a3d46);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(0.35));
new ResizeObserver(() => {
  const r = $("viewport").getBoundingClientRect();
  renderer.setSize(r.width, r.height);
  camera.aspect = r.width / r.height;
  camera.updateProjectionMatrix();
}).observe($("viewport"));
renderer.setAnimationLoop(() => {
  orbit.update();
  renderer.render(scene, camera);
});

function fit(selected = false) {
  if (!state.robot) return;
  state.robot.updateMatrixWorld(true);
  let box = new THREE.Box3();
  if (selected && pair()) {
    for (const n of [pair().link1, pair().link2]) {
      for (const object of state.session.model.objects.filter(
        (o) => o.link === n,
      )) {
        box.union(
          new THREE.Box3().setFromObject(
            state.robot.colliders[`__cc_object_${object.index}`],
          ),
        );
      }
    }
  } else box.setFromObject(state.robot);
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3()),
    size = Math.max(0.3, box.getSize(new THREE.Vector3()).length());
  orbit.target.copy(center);
  camera.position
    .copy(center)
    .add(
      new THREE.Vector3(1, -1.3, 0.8).normalize().multiplyScalar(size * 1.5),
    );
  camera.near = Math.max(0.001, size / 1000);
  camera.far = Math.max(100, size * 100);
  camera.updateProjectionMatrix();
  orbit.update();
}
function applyPose(values) {
  const model = state.session?.model,
    resolved = { ...state.pose, ...values };
  if (model) {
    const joints = new Map(model.joints.map((j) => [j.name, j]));
    for (const j of model.joints) {
      if (j.type === "fixed") {
        resolved[j.name] = 0;
        continue;
      }
      if (j.mimic) continue;
      const bounds = model.valid_bounds[j.name];
      resolved[j.name] = Math.min(
        bounds?.upper ?? Infinity,
        Math.max(
          bounds?.lower ?? -Infinity,
          Number(resolved[j.name] ?? model.default_pose[j.name] ?? 0),
        ),
      );
    }
    const visit = (name) => {
      const j = joints.get(name);
      if (j.mimic)
        resolved[name] =
          visit(j.mimic.joint) * j.mimic.multiplier + j.mimic.offset;
      return resolved[name];
    };
    for (const j of model.joints) visit(j.name);
  }
  state.pose = resolved;
  if (!state.robot) return;
  // The server supplies mimic values too; setting URDF drivers also updates them.
  state.robot.setJointValues(resolved);
  state.robot.updateMatrixWorld(true);
  document.querySelectorAll("[data-joint-value]").forEach((el) => {
    if (document.activeElement !== el)
      el.value = resolved[el.dataset.jointValue] ?? 0;
  });
}
function updateAppearance() {
  if (!state.robot) return;
  const selected = pair();
  state.robot.traverse((node) => {
    if (node.isURDFCollider) node.visible = $("show-collision").checked;
    if (node.isURDFVisual) node.visible = $("show-visual").checked;
  });
  state.robot.traverse((mesh) => {
    if (!mesh.isMesh) return;
    let node = mesh,
      link = null,
      collider = false;
    while (node) {
      if (node.isURDFCollider) collider = true;
      if (node.isURDFLink) {
        link = node.name;
        break;
      }
      node = node.parent;
    }
    const active = selected && [selected.link1, selected.link2].includes(link);
    const color = selected
      ? link === selected.link1
        ? 0x56e2bd
        : link === selected.link2
          ? 0xedab66
          : 0x617986
      : 0x91b7be;
    const materials = Array.isArray(mesh.material)
      ? mesh.material
      : [mesh.material];
    materials.forEach((mat) => {
      if (collider) mat.color.setHex(color);
      else if (mat.userData.baseColor) mat.color.copy(mat.userData.baseColor);
      mat.transparent = true;
      mat.opacity =
        Number($("opacity").value) * (selected && !active ? 0.18 : 1);
      mat.depthWrite = mat.opacity > 0.5;
    });
  });
}
async function loadView(model) {
  state.ready = false;
  state.viewError = null;
  state.pose = {};
  state.priorPose = null;
  clearTimeout(poseTimer);
  poseSequence++;
  if (state.drag) state.drag.dispose();
  if (state.robot) {
    scene.remove(state.robot);
    state.robot.traverse((n) => {
      if (n.isMesh) {
        n.geometry.dispose();
        (Array.isArray(n.material) ? n.material : [n.material]).forEach((m) =>
          m.dispose(),
        );
      }
    });
  }
  state.drag = null;
  state.robot = null;
  const manager = new THREE.LoadingManager(),
    errors = [];
  // Collada's texture loader prepends its base path even to absolute image URLs.
  manager.setURLModifier((url) => {
    const index = url.lastIndexOf("/api/assets/");
    return index >= 0 ? url.slice(index) : url;
  });
  let done;
  const loaded = new Promise((resolve) => (done = resolve));
  manager.onLoad = done;
  manager.onError = (url) => errors.push(url);
  manager.itemStart("root-model");
  const loader = new URDFLoader(manager);
  loader.parseCollision = true;
  const defaultMeshLoader = loader.loadMeshCb;
  loader.loadMeshCb = (path, mgr, callback) => {
    if (/\.obj$/i.test(path)) {
      new OBJLoader(mgr).load(
        path,
        (obj) => callback(obj),
        undefined,
        (error) => {
          errors.push(path);
          callback(null, error);
        },
      );
    } else if (/\.(stl|dae)$/i.test(path))
      defaultMeshLoader(path, mgr, callback);
    else {
      errors.push(`不支持的显示 mesh 格式：${path}`);
      callback(null, new Error("unsupported mesh"));
    }
  };
  const robot = loader.parse(model.xml);
  manager.itemEnd("root-model");
  await loaded;
  if (state.revision !== model.revision) return;
  state.robot = robot;
  scene.add(robot);
  if (errors.length) throw new Error(`几何显示加载失败：${errors.join(", ")}`);
  if (
    Object.keys(robot.links).sort().join("\n") !==
    [...model.links].sort().join("\n")
  )
    throw new Error("前后端 link 集合不一致");
  for (const object of model.objects) {
    if (!robot.colliders[`__cc_object_${object.index}`])
      throw new Error(`未找到后端几何映射 ${object.name}`);
  }
  robot.traverse((n) => {
    if (n.isMesh) {
      n.material = Array.isArray(n.material)
        ? n.material.map((m) => m.clone())
        : n.material.clone();
      (Array.isArray(n.material) ? n.material : [n.material]).forEach((m) => {
        m.userData.baseColor = m.color.clone();
        m.side = THREE.DoubleSide;
      });
    }
  });
  const unified = URDFAdapter.convert(robot, model.xml);
  for (const j of model.joints) {
    if (j.type === "continuous") {
      const joint = unified.joints.get(j.name);
      if (joint?.limits) {
        joint.limits.lower = -Infinity;
        joint.limits.upper = Infinity;
      }
    }
  }
  state.drag = new PointerJointDragControls(
    scene,
    camera,
    renderer.domElement,
    unified,
  );
  state.drag.onDragStart = () => {
    orbit.enabled = false;
  };
  state.drag.onDragEnd = () => {
    orbit.enabled = true;
    syncPose();
  };
  state.drag.onUpdateJoint = (j, value) => {
    const info = model.joints.find((x) => x.name === j.name);
    if (info?.mimic) return;
    const r = state.session.settings.ranges[j.name];
    if (r?.locked) return;
    applyPose({ ...state.pose, [j.name]: value });
    schedulePose();
  };
  // Joint dragging takes precedence over OrbitControls' pointer handler.
  renderer.domElement.addEventListener(
    "pointerdown",
    (e) => {
      if (e.button === 0) {
        state.drag._mouseMove(e);
        state.drag._mouseDown(e);
      }
    },
    { capture: true, signal: viewListenerSignal() },
  );
  applyPose(model.pose);
  const response = await api("/pose", { pose: state.pose });
  applyPose(response.pose);
  robot.updateMatrixWorld(true);
  for (const [name, rows] of Object.entries(response.frames)) {
    const actual = robot.links[name].matrixWorld.elements;
    if (
      rows.some((row, i) =>
        row.some((v, j) => Math.abs(v - actual[j * 4 + i]) > 1e-5),
      )
    )
      throw new Error(`前后端位形不一致：${name}`);
  }
  for (const object of model.objects) {
    const actual =
        robot.colliders[`__cc_object_${object.index}`].matrixWorld.elements,
      rows = response.geometry_frames[object.index];
    if (
      rows.some((row, i) =>
        row.some((v, j) => Math.abs(v - actual[j * 4 + i]) > 1e-5),
      )
    )
      throw new Error(`collision 原点不一致：${object.name}`);
  }
  state.ready = true;
  $("empty-view").style.display = "none";
  updateAppearance();
  fit();
}
let viewController;
function viewListenerSignal() {
  viewController?.abort();
  viewController = new AbortController();
  return viewController.signal;
}
function schedulePose() {
  clearTimeout(poseTimer);
  poseTimer = setTimeout(
    () => syncPose().catch((e) => toast(e.message, true)),
    90,
  );
}
async function syncPose() {
  if (!state.robot) return;
  const sequence = ++poseSequence;
  const response = await api("/pose", {
    pose: state.pose,
    pair_id: state.selected,
  });
  if (sequence !== poseSequence) return;
  applyPose(response.pose);
  if (response.result) {
    $("current-distance").textContent =
      `${response.result.collision ? "碰撞" : "分离"} · ${response.result.distance.toFixed(5)} m`;
  }
  return response;
}

async function accept(session) {
  state.session = session;
  if (!session) return;
  if (state.selected && !session.pairs.some((p) => p.id === state.selected))
    state.selected = null;
  render();
  if (state.revision !== session.model.revision) {
    state.revision = session.model.revision;
    state.selected = null;
    try {
      await loadView(session.model);
    } catch (e) {
      state.ready = false;
      state.viewError = e.message;
      toast(e.message, true);
      $("view-status").textContent = e.message;
    }
    render();
  }
  updateAppearance();
}
function render() {
  const s = state.session;
  if (!s) return;
  $("model-label").textContent =
    `${s.model.name} / ${s.model.objects.length} 几何体`;
  $("view-status").textContent = state.ready
    ? `${s.model.name} · Z UP · ${s.model.joints.filter((j) => j.type !== "fixed" && !j.mimic).length} 独立关节`
    : state.viewError || "正在校验显示模型";
  const map = {
    samples: "samples",
    seed: "seed",
    always: "always_threshold",
    "default-distance": "default_min_distance",
    mu: "mu",
    delta: "delta",
  };
  for (const [id, key] of Object.entries(map))
    if (document.activeElement !== $(id)) $(id).value = s.settings[key];
  $("global-enabled").checked = s.settings.enabled;
  $("rules").innerHTML = Object.entries(s.settings.rules)
    .map(
      ([r, v]) =>
        `<label><input type="checkbox" data-rule="${r}" ${v ? "checked" : ""}> ${r}</label>`,
    )
    .join("");
  $("rules")
    .querySelectorAll("input")
    .forEach(
      (el) =>
        (el.onchange = () =>
          settings({ rules: { [el.dataset.rule]: el.checked } }).catch((e) =>
            toast(e.message, true),
          )),
    );
  $("stale").hidden = !s.stale;
  $("stale").textContent = "统计待更新，请重新采样后使用 Top-K。";
  $("analyze").disabled = !state.ready || busy();
  $("benchmark").disabled = !state.ready || busy();
  $("cancel").disabled = !busy();
  $("preview-topk").disabled = s.stale;
  $("apply-topk").disabled = s.stale;
  $("undo").disabled = !s.can_undo;
  $("link-count").textContent =
    `${s.model.collision_links.filter((n) => !s.settings.excluded.includes(n)).length} / ${s.model.collision_links.length}`;
  $("pair-count").textContent =
    `${s.pairs.filter((p) => p.enabled).length} / ${s.pairs.length} 对`;
  $("footer-status").textContent =
    `${s.stale ? "统计待更新" : "采样完成"} · ${s.pairs.filter((p) => p.enabled).reduce((n, p) => n + p.objects.length, 0)} 个启用 object 对${s.elapsed_seconds !== null ? ` · 生成 ${s.elapsed_seconds.toFixed(2)} s` : ""}`;
  renderLinks();
  renderMatrix();
  renderDetail();
  renderJoints();
  renderBenchmark();
}
function renderLinks() {
  const s = state.session;
  if (!s) return;
  const search = $("link-search").value.toLowerCase();
  const parents = new Map(s.model.joints.map((j) => [j.child, j.parent]));
  const depth = (name) => {
    let d = 0;
    while (parents.has(name)) {
      name = parents.get(name);
      d++;
    }
    return d;
  };
  $("links").innerHTML = s.model.links
    .filter((n) => n.toLowerCase().includes(search))
    .map((name) => {
      const count = s.model.objects.filter((o) => o.link === name).length;
      return `<div class="link-row ${count ? "" : "no-geometry"}" style="padding-left:${4 + depth(name) * 6}px"><label><input type="checkbox" data-link="${esc(name)}" ${s.settings.excluded.includes(name) ? "" : "checked"} ${count ? "" : "disabled"}>${esc(name)}</label><span class="tag">${count}</span></div>`;
    })
    .join("");
  $("links")
    .querySelectorAll("input")
    .forEach(
      (el) =>
        (el.onchange = () => {
          const excluded = new Set(s.settings.excluded);
          el.checked
            ? excluded.delete(el.dataset.link)
            : excluded.add(el.dataset.link);
          settings({ excluded: [...excluded] }).catch((e) =>
            toast(e.message, true),
          );
        }),
    );
}
function filterPair(p) {
  const search = $("matrix-search").value.toLowerCase(),
    reason = $("reason-filter").value;
  return (
    (!search ||
      [p.link1, p.link2].some((n) => n.toLowerCase().includes(search))) &&
    (!reason || (reason === "enabled" ? p.enabled : p.reason === reason))
  );
}
function renderMatrix() {
  const s = state.session;
  if (!s) return;
  const search = $("matrix-search").value.toLowerCase();
  const links = s.model.collision_links.filter(
    (n) =>
      !search ||
      n.toLowerCase().includes(search) ||
      s.pairs.some((p) => filterPair(p) && [p.link1, p.link2].includes(n)),
  );
  const map = new Map();
  s.pairs.forEach((p) => {
    map.set(JSON.stringify([p.link1, p.link2]), p);
    map.set(JSON.stringify([p.link2, p.link1]), p);
  });
  matrixPairs = s.pairs.filter(filterPair);
  const scroll = $("matrix"),
    x = scroll.scrollLeft,
    y = scroll.scrollTop;
  scroll.style.setProperty("--matrix-columns", links.length);
  scroll.innerHTML = `<table><colgroup><col class="matrix-row-label"><col class="matrix-column" span="${links.length || 1}"></colgroup><thead><tr><th>LINK / LINK<div class="matrix-column-resizer" data-resize="label" role="separator" aria-label="调整矩阵行名列宽" aria-orientation="vertical" tabindex="0" title="拖动调整行名列宽，双击恢复默认"></div></th>${links.map((n) => `<th><div title="${esc(n)}">${esc(n)}</div></th>`).join("")}</tr></thead><tbody>${links
    .map(
      (a) =>
        `<tr><th>${esc(a)}</th>${links
          .map((b) => {
            if (a === b) return '<td class="diagonal">·</td>';
            const p = map.get(JSON.stringify([a, b]));
            return `<td data-id="${esc(p.id)}" class="${p.reason || ""} ${p.enabled ? "" : "disabled"} ${state.selected === p.id ? "selected" : ""} ${filterPair(p) ? "" : "filtered"}" title="${esc(a)} ↔ ${esc(b)} / ${p.reason || "启用"}${p.stats ? ` / 碰撞 ${(p.stats.collision_rate * 100).toFixed(1)}% / 最近 ${p.stats.min_distance.toFixed(4)}m` : ""}"><input type="checkbox" aria-label="禁用 ${esc(a)} 与 ${esc(b)}" ${p.enabled ? "" : "checked"}></td>`;
          })
          .join("")}</tr>`,
    )
    .join("")}</tbody></table>`;
  scroll.scrollLeft = x;
  scroll.scrollTop = y;
  layout.refresh();
  scroll.querySelectorAll("td[data-id]").forEach((td) => {
    td.onclick = async (e) => {
      try {
        state.selected = td.dataset.id;
        if (e.target.matches("input")) {
          await mutate(
            "/pairs",
            {
              ids: [state.selected],
              patch: { manual: !e.target.checked, lock: null },
            },
            "PATCH",
          );
        } else {
          renderMatrix();
          renderDetail();
          updateAppearance();
        }
        await syncPose();
      } catch (err) {
        toast(err.message, true);
      }
    };
  });
}
const pct = (v) => `${(v * 100).toFixed(2)}%`;
function renderDetail() {
  const p = pair();
  if (!p) {
    $("pair-detail").textContent = "点击矩阵中的一格查看详情。";
    return;
  }
  const stat = p.stats;
  $("pair-detail").innerHTML =
    `<strong><i class="chip-a"></i> ${esc(p.link1)}<br><i class="chip-b"></i> ${esc(p.link2)}</strong><dl><dt>状态 / 原因</dt><dd>${p.reason || "启用"}</dd><dt>Object 对</dt><dd>${p.objects.length}</dd>${stat ? `<dt>碰撞率</dt><dd>${pct(stat.collision_rate)}</dd><dt>接近率</dt><dd>${pct(stat.near_rate)}</dd><dt>最小距离</dt><dd>${stat.min_distance.toFixed(5)} m</dd>` : ""}</dl><div id="current-distance" class="tag">等待实时距离</div><div class="row"><button id="witness-collision" ${stat?.collision_pose ? "" : "disabled"}>碰撞姿态</button><button id="witness-closest" ${stat?.closest_pose ? "" : "disabled"}>最近姿态</button></div><label>锁定<select id="pair-lock"><option value="">不锁定</option><option value="keep">必须保留</option><option value="exclude">必须排除</option></select></label><label>逐对 min_distance / m<input id="pair-distance" type="number" min="0" step="0.01" placeholder="继承 ${state.session.settings.default_min_distance}" value="${p.min_distance ?? ""}"></label><div>有效安全距离：${p.effective_min_distance} m</div><details><summary>Object 索引与名称</summary>${p.objects.map(([a, b]) => `<div>${a}: ${esc(state.session.model.objects[a].name)} ↔ ${b}: ${esc(state.session.model.objects[b].name)}</div>`).join("")}</details>`;
  $("pair-lock").value = p.lock || "";
  on(
    "pair-lock",
    () =>
      mutate(
        "/pairs",
        { ids: [p.id], patch: { lock: $("pair-lock").value || null } },
        "PATCH",
      ),
    "change",
  );
  on(
    "pair-distance",
    () =>
      mutate(
        "/pairs",
        {
          ids: [p.id],
          patch: {
            min_distance:
              $("pair-distance").value === ""
                ? null
                : Number($("pair-distance").value),
          },
        },
        "PATCH",
      ),
    "change",
  );
  const witness = async (pose) => {
    state.priorPose ??= { ...state.pose };
    applyPose(pose);
    await syncPose();
  };
  on("witness-collision", () => witness(stat.collision_pose));
  on("witness-closest", () => witness(stat.closest_pose));
}
function renderJoints() {
  const s = state.session;
  $("joints").innerHTML = s.model.joints
    .filter((j) => j.type !== "fixed")
    .map((j, index) => {
      const r = s.settings.ranges[j.name] || {},
        lo = r.lower ?? j.lower ?? -Math.PI,
        hi = r.upper ?? j.upper ?? Math.PI,
        v = state.pose[j.name] ?? s.model.pose[j.name] ?? 0;
      return `<div class="joint" data-joint-index="${index}"><div class="joint-name">${esc(j.name)} ${j.mimic ? "· mimic" : ""}</div><div class="row"><input type="range" data-joint-value="${esc(j.name)}" min="${lo}" max="${hi}" step="0.001" value="${v}" ${j.mimic || r.locked ? "disabled" : ""}><input class="joint-value" type="number" data-joint-value="${esc(j.name)}" min="${j.type === "continuous" ? "" : (j.lower ?? "")}" max="${j.type === "continuous" ? "" : (j.upper ?? "")}" step="0.01" value="${v}" ${j.mimic || r.locked ? "disabled" : ""}></div>${j.mimic ? "" : `<details><summary>采样范围 / 锁定</summary><div class="two-cols"><label>下限<input class="range-lo" type="number" step="0.01" value="${r.lower ?? j.lower ?? ""}"></label><label>上限<input class="range-hi" type="number" step="0.01" value="${r.upper ?? j.upper ?? ""}"></label></div><label><input class="range-lock" type="checkbox" ${r.locked ? "checked" : ""}>锁定在当前值</label></details>`}</div>`;
    })
    .join("");
  const joints = s.model.joints.filter((j) => j.type !== "fixed");
  $("joints")
    .querySelectorAll(".joint")
    .forEach((el) => {
      const j = joints[Number(el.dataset.jointIndex)];
      el.querySelectorAll("[data-joint-value]").forEach(
        (input) =>
          (input.oninput = () => {
            applyPose({ ...state.pose, [j.name]: Number(input.value) });
            schedulePose();
          }),
      );
      const updateRange = () => {
        const ranges = { ...s.settings.ranges },
          locked = el.querySelector(".range-lock").checked;
        const r = { locked, value: state.pose[j.name] ?? 0 };
        for (const [key, cls] of [
          ["lower", ".range-lo"],
          ["upper", ".range-hi"],
        ]) {
          const value = el.querySelector(cls).value;
          if (value !== "") r[key] = Number(value);
        }
        ranges[j.name] = r;
        settings({ ranges }).catch((e) => toast(e.message, true));
      };
      el.querySelectorAll(".range-lo,.range-hi,.range-lock").forEach(
        (input) => (input.onchange = updateRange),
      );
    });
}
function renderBenchmark() {
  const b = state.session?.benchmark;
  if (!b) {
    $("benchmark-report").textContent = "碰撞 / 距离 · 含运动学更新 · 全量遍历";
    return;
  }
  $("benchmark-report").innerHTML =
    `<table class="report-table"><thead><tr><th>平均 / P95 ms</th><th>全部</th><th>选中</th></tr></thead><tbody>${["collision", "distance"].map((mode) => `<tr><th>${mode === "collision" ? "碰撞" : "距离"}</th>${["all", "selected"].map((k) => `<td>${b.results[k][mode].mean_ms.toFixed(3)} / ${b.results[k][mode].p95_ms.toFixed(3)}</td>`).join("")}</tr>`).join("")}</tbody></table><div>碰撞加速 ${b.speedup.collision.toFixed(2)}× · 距离加速 ${b.speedup.distance.toFixed(2)}×<br>${b.results.all.link_pairs} → ${b.results.selected.link_pairs} link 对<br>${b.results.all.object_pairs} → ${b.results.selected.object_pairs} object 对 · ${b.samples} 位形</div>`;
}

on("load", async () => {
  await accept(
    await api("/load", {
      path: $("model-path").value,
      packages: JSON.parse($("packages").value),
      xacro_args: JSON.parse($("xacro-args").value),
    }),
  );
  if (state.ready) toast("模型已加载");
});
on("link-search", renderLinks, "input");
on("matrix-search", renderMatrix, "input");
on("reason-filter", renderMatrix, "change");
on("all-links", () => settings({ excluded: [] }));
on("no-links", () =>
  settings({ excluded: [...state.session.model.collision_links] }),
);
for (const id of ["show-collision", "show-visual", "opacity"])
  on(id, updateAppearance, "input");
on("fit", () => fit());
on("focus", () => fit(true));
for (const [id, key] of Object.entries({
  samples: "samples",
  seed: "seed",
  always: "always_threshold",
  "default-distance": "default_min_distance",
  mu: "mu",
  delta: "delta",
}))
  on(id, () => settings({ [key]: Number($(id).value) }), "change");
on(
  "global-enabled",
  () => settings({ enabled: $("global-enabled").checked }),
  "change",
);
on("enable-filtered", () =>
  mutate(
    "/pairs",
    { ids: matrixPairs.map((p) => p.id), patch: { manual: true, lock: null } },
    "PATCH",
  ),
);
on("disable-filtered", () =>
  mutate(
    "/pairs",
    { ids: matrixPairs.map((p) => p.id), patch: { manual: false, lock: null } },
    "PATCH",
  ),
);
on("reset-selection", () => mutate("/reset-selection", {}));
on("undo", () => mutate("/undo", {}));
async function start(kind) {
  await changes;
  const job = await api("/jobs", {
    kind,
    samples: Number($("benchmark-samples").value),
  });
  updateJob(job);
}
on("analyze", () => start("analysis"));
on("benchmark", () => start("benchmark"));
on("cancel", async () => updateJob(await api("/jobs/cancel", {})));
function showTopk(r) {
  pendingTopk = r;
  $("topk-report").textContent =
    `${r.before} → ${r.after} 对${r.exceeds_k ? "（锁定项超过 K）" : ""}；碰撞事件保留 ${r.collision_event_retention === null ? "无碰撞样本" : pct(r.collision_event_retention)}`;
}
on("preview-topk", async () => {
  await changes;
  showTopk(await api("/topk", { k: Number($("top-k").value) }));
});
on("apply-topk", async () => {
  await changes;
  const preview = await api("/topk", { k: Number($("top-k").value) });
  showTopk(preview);
  if (
    !confirm(
      `将保留 ${preview.after} 个 link 对，禁用 ${preview.removed.length} 对。应用筛选？`,
    )
  )
    return;
  showTopk(await api("/topk", { k: Number($("top-k").value), apply: true }));
  await refresh();
});
on("reset-pose", async () => {
  applyPose(state.session.model.default_pose);
  await syncPose();
});
on("restore-pose", async () => {
  if (state.priorPose) {
    applyPose(state.priorPose);
    state.priorPose = null;
    await syncPose();
  }
});
on("save-project", async () => {
  await changes;
  await syncPose();
  const path = prompt(
    "工程保存路径（引用原模型，不复制资源）",
    "robot.collision-project.json",
  );
  if (path) toast(`已保存：${(await api("/project/save", { path })).path}`);
});
on("open-project", async () => {
  const path = prompt("工程文件路径");
  if (!path) return;
  const response = await api("/project/open", { path });
  await accept(response.session);
  if (response.warning) toast(response.warning, true);
  else toast("工程已恢复");
});
on("export", async () => {
  await changes;
  if (!state.session) throw new Error("请先加载模型");
  if (
    state.session.stale &&
    !confirm("统计尚未更新。仍按当前人工选择集合导出？")
  )
    return;
  const directory = prompt("配置输出目录", "config");
  if (!directory) return;
  let output;
  try {
    output = await api("/export", { directory });
  } catch (e) {
    if (e.code !== "INFO_GLOBAL_DISTANCE_CONFIRMATION") throw e;
    if (
      !confirm(
        `${e.message}\nINFO 将使用全局距离，YAML 保留逐对距离。继续导出？`,
      )
    )
      return;
    output = await api("/export", {
      directory,
      acknowledge_info_global_distance: true,
    });
  }
  toast(
    `已生成 ${output.link_pairs} 个 link 对 / ${output.object_pairs} 个 object 对\n${output.files.join("\n")}`,
  );
});
async function browse(path) {
  const result = await api(`/browse?path=${encodeURIComponent(path)}`);
  browseParent = result.parent;
  $("browse-path").value = result.path;
  $("browse-entries").innerHTML = result.entries
    .map(
      (p, index) =>
        `<button data-index="${index}">${p.directory ? "▸" : "◇"} ${esc(p.name)}</button>`,
    )
    .join("");
  $("browse-entries")
    .querySelectorAll("button")
    .forEach(
      (button) =>
        (button.onclick = async () => {
          try {
            const p = result.entries[Number(button.dataset.index)];
            if (p.directory) await browse(p.path);
            else {
              $("model-path").value = p.path;
              $("browser").close();
            }
          } catch (e) {
            toast(e.message, true);
          }
        }),
    );
}
on("browse", async () => {
  await browse(".");
  $("browser").showModal();
});
on("browse-up", () => browse(browseParent));
on("browse-go", () => browse($("browse-path").value));
on("browse-close", () => $("browser").close());
async function refresh() {
  const response = await api("/state");
  await accept(response.session);
  if (response.job) updateJob(response.job);
}
function updateJob(job) {
  const old = state.job;
  state.job = job;
  if (!job) return;
  $("progress").value = job.progress;
  $("job-status").textContent = job.message;
  for (const id of ["analyze", "benchmark"])
    $(id).disabled = !state.ready || busy();
  $("cancel").disabled = !busy();
  if (
    !["running", "cancelling"].includes(job.status) &&
    (!old || old.status !== job.status || old.id !== job.id)
  ) {
    if (job.status === "error") toast(job.message, true);
    refresh().catch((e) => toast(e.message, true));
  }
}
function connect() {
  const ws = new WebSocket(
    `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`,
  );
  ws.onmessage = (e) => updateJob(JSON.parse(e.data));
  ws.onclose = () => setTimeout(connect, 3000);
  ws.onerror = () => ws.close();
}
refresh().catch((e) => toast(e.message, true));
connect();
// Exposed read-only diagnostics for end-to-end rendering/kinematics checks.
window.collisionStudio = {
  get session() {
    return state.session;
  },
  get robot() {
    return state.robot;
  },
  get ready() {
    return state.ready;
  },
  projectObject(index) {
    const collider = state.robot?.colliders[`__cc_object_${index}`];
    if (!collider) return null;
    state.robot.updateMatrixWorld(true);
    const point = new THREE.Box3()
      .setFromObject(collider)
      .getCenter(new THREE.Vector3())
      .project(camera);
    const rect = renderer.domElement.getBoundingClientRect();
    return {
      x: rect.left + ((point.x + 1) * rect.width) / 2,
      y: rect.top + ((1 - point.y) * rect.height) / 2,
    };
  },
};
