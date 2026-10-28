// Three.js pool scene with Talos. Everything inside `root` is in ROS world
// coordinates (x forward, y left, z up); root turns them into three's y-up.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { toCreasedNormals } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import { vehicle } from '../engine/talos.js';
import { rpyQuat, qrot } from '../engine/math.js';

const BASE = import.meta.env.BASE_URL;

function tileTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 512;
  const g = c.getContext('2d');
  g.fillStyle = '#e9f4f5';
  g.fillRect(0, 0, 512, 512);
  g.strokeStyle = '#b9d6db';
  g.lineWidth = 3;
  for (let i = 0; i <= 512; i += 64) {
    g.beginPath(); g.moveTo(i, 0); g.lineTo(i, 512); g.stroke();
    g.beginPath(); g.moveTo(0, i); g.lineTo(512, i); g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}

const v3 = (p) => new THREE.Vector3(p[0], p[1], p[2]);
const q4 = (q) => new THREE.Quaternion(q[1], q[2], q[3], q[0]);

export function createScene(canvas, { floorDepth = -3, onPick, cameraPos = [-1.9, -2.2, -0.35], target = [0.35, 0, -1.05], interactive = true, autoRotate = false, shiftRight = 0 } = {}) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  const scene = new THREE.Scene();
  // Soft studio reflections so the metal and plastic read as materials, not flat fills.
  const pmrem = new THREE.PMREMGenerator(renderer);
  const envMap = pmrem.fromScene(new RoomEnvironment(), 0.04).texture; // used by Talos's materials only
  scene.fog = new THREE.Fog(0xcfe7ec, 7, 22);
  const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 80);
  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2;
  scene.add(root);
  const toThree = (p) => v3(p).applyEuler(root.rotation);
  camera.position.copy(toThree(cameraPos));
  const controls = new OrbitControls(camera, canvas);
  controls.target.copy(toThree(target));
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI * 0.58;
  controls.minDistance = 1.2;
  controls.maxDistance = 14;
  controls.enablePan = interactive;
  controls.autoRotate = autoRotate;
  controls.autoRotateSpeed = 0.4;
  controls.update();

  scene.add(new THREE.HemisphereLight(0xf2fbff, 0x7aa9b3, 2.1));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(3, 8, 4);
  scene.add(sun);

  // Pool floor.
  const tex = tileTexture();
  tex.repeat.set(16, 16);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(32, 32), new THREE.MeshLambertMaterial({ map: tex }));
  floor.position.z = floorDepth;
  root.add(floor);
  // Water surface hint.
  const surface = new THREE.Mesh(new THREE.PlaneGeometry(32, 32), new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.08, side: THREE.DoubleSide }));
  root.add(surface);

  // Talos: model frame is origin; the group sits at base_link.
  const talos = new THREE.Group();
  root.add(talos);
  const body = new THREE.Group();
  body.position.copy(v3(vehicle.base_link).multiplyScalar(-1));
  talos.add(body);
  const loader = new GLTFLoader();
  loader.setMeshoptDecoder(MeshoptDecoder);
  const ready = new Promise((resolve) => {
    loader.load(`${BASE}models/talos.glb`, (g) => {
      g.scene.traverse((o) => {
        if (o.isMesh) {
          const m = o.material;
          // CAD meshes need hard edges kept: split normals wherever faces meet at more than 35 degrees.
          o.geometry = toCreasedNormals(o.geometry, (35 * Math.PI) / 180);
          o.material = new THREE.MeshStandardMaterial({ color: m.color, metalness: 0.3, roughness: 0.5, envMap, envMapIntensity: 0.45, side: m.side }); // all opaque: the model has no internals to see through the acrylic
        }
      });
      body.add(g.scene);
      resolve();
    }, undefined, () => resolve());
  });

  // Thrust arrows, one per thruster, in the CAD frame.
  const arrows = vehicle.thrusters.map((t) => {
    const dir = qrot(rpyQuat(t.pose[3], t.pose[4], t.pose[5]), [1, 0, 0]);
    const a = new THREE.ArrowHelper(v3(dir), v3(t.pose), 0.2, 0xd9480f, 0.06, 0.04);
    a.userData.dir = v3(dir);
    a.line.material.linewidth = 2;
    body.add(a);
    return a;
  });

  // Screen-space lines of constant pixel width (WebGL's own lines are 1 px and vanish edge-on or in fog).
  const lineMats = [];
  function line(color, { dashed = false, width = 3, opacity = 1 } = {}) {
    const mat = new LineMaterial({ color, linewidth: width, dashed, dashSize: 0.08, gapSize: 0.06, transparent: opacity < 1, opacity, fog: false, worldUnits: false });
    lineMats.push(mat);
    let l = null, visible = true;
    return {
      set(points) {
        if (l) { root.remove(l); l.geometry.dispose(); l = null; }
        if (points.length < 2) return;
        const geo = new LineGeometry();
        geo.setPositions(points.flatMap((p) => [p[0], p[1], p[2]]));
        l = new Line2(geo, mat);
        l.frustumCulled = false;
        if (dashed) l.computeLineDistances();
        l.visible = visible;
        root.add(l);
      },
      show(on) { visible = on; if (l) l.visible = on; },
    };
  }
  const prediction = line(0x13996a, { width: 3.5 });
  const planned = line(0xeb6834, { dashed: true, width: 2.5 });
  const trail = line(0x2a78d6, { width: 3 });
  // Dots along the horizon, every 5 stages.
  const dotGeo = new THREE.SphereGeometry(0.025, 12, 8);
  const predDots = Array.from({ length: 31 }, () => {
    const m = new THREE.Mesh(dotGeo, new THREE.MeshBasicMaterial({ color: 0x13996a }));
    m.visible = false;
    root.add(m);
    return m;
  });
  // Reference marker: a small orange frame at the profile pose.
  const refMarker = new THREE.Group();
  const ringGeo = new THREE.TorusGeometry(0.12, 0.012, 8, 40);
  const ring = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({ color: 0xeb6834 }));
  refMarker.add(ring);
  const nose = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.32, 0xeb6834, 0.07, 0.05);
  refMarker.add(nose);
  root.add(refMarker);
  // Goal marker on the floor.
  const goal = new THREE.Mesh(new THREE.RingGeometry(0.16, 0.22, 40), new THREE.MeshBasicMaterial({ color: 0xd9480f, transparent: true, opacity: 0.85 }));
  goal.position.z = floorDepth + 0.004;
  goal.visible = false;
  root.add(goal);
  const goalStem = line(0xd9480f, { dashed: true, width: 1.5, opacity: 0.6 });
  const sketch = line(0xd9480f, { width: 4 }); // a path being drawn by hand

  // Picking on the floor plane.
  const ray = new THREE.Raycaster();
  let down = null;
  canvas.addEventListener('pointerdown', (e) => (down = { x: e.clientX, y: e.clientY, t: performance.now() }));
  let drawing = false; // draw mode: drags trace a path instead of orbiting
  canvas.addEventListener('pointerup', (e) => {
    if (!down || !onPick || drawing) return;
    const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y);
    if (moved > 6 || performance.now() - down.t > 600) return;
    const r = canvas.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
    const hit = ray.intersectObject(floor)[0];
    if (hit) {
      const p = root.worldToLocal(hit.point.clone());
      onPick([p.x, p.y, p.z]);
    }
  });

  // Where a screen point lands on the horizontal plane at depth z (ROS world frame).
  const plane = new THREE.Plane();
  function pointOnDepth(clientX, clientY, z) {
    const r = canvas.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2(((clientX - r.left) / r.width) * 2 - 1, -((clientY - r.top) / r.height) * 2 + 1), camera);
    plane.set(new THREE.Vector3(0, 1, 0), -z); // ROS z is three's y (root is rotated)
    const hit = new THREE.Vector3();
    if (!ray.ray.intersectPlane(plane, hit)) return null;
    const p = root.worldToLocal(hit);
    if (Math.abs(p.x) > 15 || Math.abs(p.y) > 15) return null; // near the horizon the ray lands far outside the pool
    return [p.x, p.y, p.z];
  }

  let fitted = false;
  function resize() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    if (!fitted) {
      // Narrow screens: back the camera off so the robot fits under the copy.
      fitted = true;
      if (w < 700) camera.position.sub(controls.target).multiplyScalar(1.9).add(controls.target);
    }
    renderer.setSize(w, h, false);
    lineMats.forEach((m) => m.resolution.set(w, h));
    camera.aspect = w / h;
    if (shiftRight && w > 900) camera.setViewOffset(w, h, -w * shiftRight, 0, w, h);
    else camera.clearViewOffset();
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(canvas);
  resize();
  let follow = null, visible = true;
  new IntersectionObserver(([en]) => (visible = en.isIntersecting)).observe(canvas);
  renderer.setAnimationLoop(() => {
    if (!visible) return;
    if (follow) {
      const t = toThree(follow);
      controls.target.lerp(t, 0.04);
    }
    controls.update();
    renderer.render(scene, camera);
  });

  return {
    ready,
    pointOnDepth,
    setDrawing(on) {
      drawing = on;
      controls.enabled = !on;
      canvas.style.cursor = on ? 'crosshair' : '';
    },
    setSketch(points) { sketch.set(points); },
    THREE,
    root,
    camera,
    controls,
    setRobot(position, orientation) {
      talos.position.copy(v3(position));
      talos.quaternion.copy(q4(orientation));
    },
    setThrust(cmd, max = 24) {
      arrows.forEach((a, i) => {
        const f = cmd[i] || 0;
        const len = Math.max(0.001, (Math.abs(f) / max) * 0.45);
        a.setDirection(f >= 0 ? a.userData.dir : a.userData.dir.clone().negate());
        a.setLength(len + 0.06, 0.06, 0.04);
        a.visible = Math.abs(f) > 0.2;
      });
    },
    setPrediction(points) {
      prediction.set(points);
      predDots.forEach((d, i) => {
        const k = i * 5;
        d.visible = k < points.length && i > 0;
        if (d.visible) d.position.copy(v3(points[k]));
      });
    },
    setPlanned(points) { planned.set(points); },
    setTrail(points) { trail.set(points); },
    setReference(position, orientation) {
      refMarker.position.copy(v3(position));
      refMarker.quaternion.copy(q4(orientation));
    },
    setGoal(p) {
      goal.visible = true;
      goal.position.x = p[0];
      goal.position.y = p[1];
      goalStem.set([[p[0], p[1], floorDepth], p]);
    },
    follow(p) { follow = p; },
    showPrediction(on) { prediction.show(on); predDots.forEach((d) => (d.visible = d.visible && on)); },
  };
}
