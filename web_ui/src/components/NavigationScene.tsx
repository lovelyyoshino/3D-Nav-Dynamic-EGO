import { useCallback, useEffect, useRef } from "react";
import * as THREE from "three";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";
import type { NavigationMode, OccupancyCell } from "../rosbridge/client";
import {
  Point3,
  type PlannedSegment,
  sanitizePoint,
  selectVoxelTargetFromHits,
  splitPolylineAtProgress,
} from "../utils/trajectory";
import {
  type Bounds,
  boundsCenter,
  boundsDiagonal,
  boundsSpan,
  containsMapPoint,
  mapPlanningPlaneZ,
  roundScenePoint,
} from "../utils/bounds";
import {
  cellColor,
  createGoalLabel,
  createPointLabel,
  disposeObject,
  mapPalette,
  placeholderDash,
  type ZRange,
} from "../utils/palette";

const dragThresholdPixels = 6;

type SceneProps = {
  mode: NavigationMode;
  startPoint: Point3 | null;
  goals: Point3[];
  routePoints: Point3[];
  plannedSegments: PlannedSegment[];
  progress: number;
  occupancyCells: OccupancyCell[];
  mapBounds: Bounds;
  robotPosition: Point3;
  hasLivePose: boolean;
  onScenePoint: (point: Point3) => void;
  onSceneMiss: () => void;
};

type SceneInteraction = {
  pointerId: number;
  startX: number;
  startY: number;
  lastX: number;
  lastY: number;
  dragging: boolean;
  dragMode: "orbit" | "pan";
};

type SceneTestProbe = {
  projectVoxel: (point: Point3) => { x: number; y: number } | null;
  projectPoint: (point: Point3) => { x: number; y: number } | null;
  cameraTarget: () => Point3;
  routePoints: () => Point3[];
};

declare global {
  interface Window {
    __nav3dSceneProbe?: SceneTestProbe;
  }
}
export function NavigationScene({
  mode,
  startPoint,
  goals,
  routePoints,
  plannedSegments,
  progress,
  occupancyCells,
  mapBounds,
  robotPosition,
  hasLivePose,
  onScenePoint,
  onSceneMiss,
}: SceneProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const occupancyGroupRef = useRef<THREE.Group | null>(null);
  const occupancyPickMeshRef = useRef<THREE.InstancedMesh | null>(null);
  const goalGroupRef = useRef<THREE.Group | null>(null);
  const robotRef = useRef<THREE.Group | null>(null);
  const pathGroupRef = useRef<THREE.Group | null>(null);
  const mapFrameRef = useRef<THREE.Group | null>(null);
  const raycasterRef = useRef(new THREE.Raycaster());
  const pointerRef = useRef(new THREE.Vector2());
  const interactionRef = useRef<SceneInteraction | null>(null);
  const cameraTargetRef = useRef(new THREE.Vector3(0, 0, 0));
  const orbitRef = useRef({
    distance: 24,
    yaw: -0.78,
    pitch: 0.55,
    minDistance: 5,
    maxDistance: 80,
  });

  const renderScene = useCallback(() => {
    const renderer = rendererRef.current;
    const scene = sceneRef.current;
    const camera = cameraRef.current;
    if (!renderer || !scene || !camera) {
      return;
    }
    renderer.render(scene, camera);
  }, []);
  const applyCameraView = useCallback((nextMode: NavigationMode) => {
    const camera = cameraRef.current;
    if (!camera) {
      return;
    }
    const target = cameraTargetRef.current;
    if (nextMode === "2d") {
      camera.position.set(target.x, target.y, target.z + orbitRef.current.distance);
      camera.lookAt(target);
      renderScene();
      return;
    }
    const orbit = orbitRef.current;
    const horizontalDistance = orbit.distance * Math.cos(orbit.pitch);
    camera.position.set(
      target.x + horizontalDistance * Math.cos(orbit.yaw),
      target.y + horizontalDistance * Math.sin(orbit.yaw),
      target.z + orbit.distance * Math.sin(orbit.pitch),
    );
    camera.lookAt(target);
    renderScene();
  }, [renderScene]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return undefined;
    }
    const renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      preserveDrawingBuffer: true,
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.25));
    renderer.setClearColor(mapPalette.background);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(mapPalette.background);
    const camera = new THREE.PerspectiveCamera(48, 1, 0.05, 500);
    camera.up.set(0, 0, 1);
    const ambient = new THREE.AmbientLight(0xffffff, 1.0);
    // Two soft directionals only — Basic material ignores them, but the robot
    // chassis (Lambert-default Basic) and ground accents still pick up tone.
    const keyLight = new THREE.DirectionalLight(0xb0d6ff, 0.7);
    keyLight.position.set(6, -7, 11);
    const fillLight = new THREE.DirectionalLight(0xc9b3ff, 0.35);
    fillLight.position.set(-8, 6, 9);
    scene.add(ambient, keyLight, fillLight);
    const mapFrame = new THREE.Group();
    const occupancyGroup = new THREE.Group();
    const goalGroup = new THREE.Group();
    const pathGroup = new THREE.Group();
    const robotGroup = new THREE.Group();
    scene.add(mapFrame, occupancyGroup, goalGroup, pathGroup, robotGroup);

    const robotBase = new THREE.Mesh(
      new THREE.CylinderGeometry(0.28, 0.28, 0.14, 28),
      new THREE.MeshBasicMaterial({ color: mapPalette.robotBase }),
    );
    robotBase.rotation.x = Math.PI / 2;
    const robotHeading = new THREE.Mesh(
      new THREE.ConeGeometry(0.17, 0.42, 24),
      new THREE.MeshBasicMaterial({ color: mapPalette.robotAccent }),
    );
    robotHeading.rotation.z = -Math.PI / 2;
    robotHeading.position.x = 0.34;
    const robotRing = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(0.82, 0.82, 0.04)),
      new THREE.LineBasicMaterial({ color: mapPalette.robotAccent, transparent: true, opacity: 0.75 }),
    );
    robotRing.position.z = -0.08;
    robotGroup.add(robotBase, robotHeading, robotRing);

    sceneRef.current = scene;
    cameraRef.current = camera;
    rendererRef.current = renderer;
    mapFrameRef.current = mapFrame;
    occupancyGroupRef.current = occupancyGroup;
    goalGroupRef.current = goalGroup;
    pathGroupRef.current = pathGroup;
    robotRef.current = robotGroup;

    const resize = () => {
      const rect = canvas.getBoundingClientRect();
      const width = Math.max(1, rect.width);
      const height = Math.max(1, rect.height);
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderScene();
    };
    resize();
    renderScene();
    window.addEventListener("resize", resize);
    return () => {
      window.removeEventListener("resize", resize);
      disposeObject(scene);
      renderer.dispose();
    };
  }, [renderScene]);
  useEffect(() => {
    const group = mapFrameRef.current;
    if (!group) return;
    group.children.forEach((child) => disposeObject(child));
    group.clear();
    const center = boundsCenter(mapBounds);
    const span = boundsSpan(mapBounds);
    const diagonal = boundsDiagonal(mapBounds);
    const floorZ = Math.min(0, mapBounds.min.z);
    const xySpan = Math.max(span.x, span.y);
    const tallMapDistanceScale = span.z > xySpan * 1.5 ? 1.8 : 1.25;
    cameraTargetRef.current.set(center.x, center.y, center.z);
    orbitRef.current.distance = diagonal * tallMapDistanceScale;
    orbitRef.current.minDistance = Math.max(3, diagonal * 0.18);
    orbitRef.current.maxDistance = diagonal * 4;

    const gridSize = Math.max(span.x, span.y) + 2;
    const cellResolution = occupancyCells.find((c) => Number.isFinite(c.size) && c.size > 0)?.size ?? 0.5;
    const divisions = Math.max(8, Math.min(80, Math.round(gridSize / Math.max(0.25, cellResolution))));
    const grid = new THREE.GridHelper(gridSize, divisions, mapPalette.gridPrimary, mapPalette.gridSecondary);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(center.x, center.y, floorZ - 0.01);
    const gridMaterial = grid.material as THREE.LineBasicMaterial | THREE.LineBasicMaterial[];
    const tuneGrid = (m: THREE.LineBasicMaterial) => { m.transparent = true; m.opacity = mode === "3d" ? 0.55 : 0.7; };
    if (Array.isArray(gridMaterial)) gridMaterial.forEach(tuneGrid); else tuneGrid(gridMaterial);
    group.add(grid);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(gridSize, gridSize),
      new THREE.MeshBasicMaterial({ color: mapPalette.floor, transparent: true, opacity: mode === "3d" ? 0.55 : 0.32 }),
    );
    floor.position.set(center.x, center.y, floorZ - 0.025);
    group.add(floor);

    const frame = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(span.x, span.y, span.z)),
      new THREE.LineBasicMaterial({ color: mapPalette.frame, transparent: true, opacity: mode === "3d" ? 0.55 : 0.7 }),
    );
    frame.position.set(center.x, center.y, center.z);
    group.add(frame);

    // No bulky axis bars — the voxel cubes + grid floor already convey scale
    // and orientation. Operator-requested removal: the chunky X/Y boxes were
    // overpowering on small maps and visually competing with the trajectory.
    applyCameraView(mode);
    renderScene();
  }, [applyCameraView, mapBounds, mode, occupancyCells, renderScene]);
  useEffect(() => {
    const group = occupancyGroupRef.current;
    if (!group) return;
    group.children.forEach((child) => disposeObject(child));
    group.clear();
    occupancyPickMeshRef.current = null;
    if (occupancyCells.length === 0) return;

    const positions = new Float32Array(occupancyCells.length * 3);
    const colors = new Float32Array(occupancyCells.length * 3);
    const pointGeometry = new THREE.BufferGeometry();
    // Z range for voxel hue ramp; only computed across global voxels so the
    // ramp tracks vertical structure independent of local/grid layers.
    let zMin = Number.POSITIVE_INFINITY;
    let zMax = Number.NEGATIVE_INFINITY;
    occupancyCells.forEach((cell) => {
      if (cell.layer === "grid" || cell.layer === "local") return;
      const height = cell.height ?? cell.size;
      const centerZ = cell.height ? cell.z + height / 2 : cell.z;
      if (centerZ < zMin) zMin = centerZ;
      if (centerZ > zMax) zMax = centerZ;
    });
    const zRange = Number.isFinite(zMin) && Number.isFinite(zMax) && zMax > zMin
      ? { min: zMin, max: zMax }
      : undefined;
    occupancyCells.forEach((cell, index) => {
      const height = cell.height ?? cell.size;
      const centerZ = cell.height ? cell.z + height / 2 : cell.z;
      const color = cellColor(cell, zRange);
      positions[index * 3] = cell.x;
      positions[index * 3 + 1] = cell.y;
      positions[index * 3 + 2] = centerZ;
      colors[index * 3] = color.r;
      colors[index * 3 + 1] = color.g;
      colors[index * 3 + 2] = color.b;
    });
    pointGeometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    pointGeometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    const voxelCount = occupancyCells.length;
    const isDenseMap = voxelCount > 400;
    // Adaptive point-cloud accent: tiny boost on sparse maps, skip entirely on dense ones.
    // Dense maps already show clearly via InstancedMesh cubes; redundant Points blur them.
    const pointSize = isDenseMap ? 0 : (mode === "3d" ? 7.0 : 4.5);
    const pointCloud = new THREE.Points(
      pointGeometry,
      new THREE.PointsMaterial({
        size: pointSize,
        sizeAttenuation: false,
        vertexColors: true,
        transparent: true,
        opacity: isDenseMap ? 0 : 0.85,
        blending: THREE.NormalBlending,
        depthTest: false,
        depthWrite: false,
      }),
    );

    const geometry = new THREE.BoxGeometry(1, 1, 1);
    // CRITICAL: do NOT set vertexColors=true on InstancedMesh materials —
    // BoxGeometry has no per-vertex color attribute, so the shader compiles
    // with USE_COLOR but reads zero vertexColor → multiplies the white base
    // color by zero → solid voxels render visibly black on the deep navy
    // substrate. InstancedMesh's `setColorAt` injects per-instance color via
    // `instanceColor` attribute (USE_INSTANCING_COLOR define), which is
    // independent of `vertexColors`. Leaving vertexColors=false lets the
    // instance color be the sole color signal — this is exactly the path
    // three.js examples for InstancedMesh use.
    const solidMaterial = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: mode !== "3d",
      opacity: mode === "3d" ? 1.0 : 0.94,
      depthWrite: mode === "3d",
    });
    const wireMaterial = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: mode === "3d" ? 0.28 : 0.42,
      wireframe: true,
      depthWrite: false,
    });
    const solid = new THREE.InstancedMesh(geometry, solidMaterial, occupancyCells.length);
    const wire = new THREE.InstancedMesh(geometry, wireMaterial, occupancyCells.length);
    const pick = new THREE.InstancedMesh(
      geometry.clone(),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0, depthWrite: false }),
      occupancyCells.length,
    );
    solid.frustumCulled = false;
    wire.frustumCulled = false;
    pick.frustumCulled = false;
    pointCloud.frustumCulled = false;

    const dummy = new THREE.Object3D();
    const debugFirstFive: Array<{ z: number; layer: string; hex: string; rgb: { r: number; g: number; b: number } }> = [];
    occupancyCells.forEach((cell, index) => {
      const height = cell.height ?? cell.size;
      const centerZ = cell.height ? cell.z + height / 2 : cell.z;
      dummy.position.set(cell.x, cell.y, centerZ);
      dummy.rotation.set(0, 0, 0);
      dummy.scale.set(cell.size * 0.96, cell.size * 0.96, height * 0.96);
      dummy.updateMatrix();
      solid.setMatrixAt(index, dummy.matrix);
      wire.setMatrixAt(index, dummy.matrix);
      pick.setMatrixAt(index, dummy.matrix);
      const color = cellColor(cell, zRange);
      if (index < 5) {
        debugFirstFive.push({
          z: centerZ,
          layer: cell.layer ?? "global",
          hex: color.getHexString(),
          rgb: { r: color.r, g: color.g, b: color.b },
        });
      }
      // No additional lerp here — palette.ts already produces the final
      // brightness target. Mixing in another color in linear-space dragged
      // the voxels visibly darker against #050912 (root cause of the
      // "still black" regression).
      solid.setColorAt(index, color);
      wire.setColorAt(index, color);
    });

    solid.instanceMatrix.needsUpdate = true;
    wire.instanceMatrix.needsUpdate = true;
    pick.instanceMatrix.needsUpdate = true;
    if (solid.instanceColor) solid.instanceColor.needsUpdate = true;
    if (wire.instanceColor) wire.instanceColor.needsUpdate = true;

    occupancyPickMeshRef.current = pick;

    // Per-voxel edge accents — merged-geometry fast path.
    //
    // v3.8 perf fix: dense OctoMap loads (e.g. building2_9 = 62,901 voxels)
    // were dying at single-digit FPS because the previous implementation
    // cloned a `LineSegments` per voxel into `edgesGroup` — THREE renders
    // each `LineSegments` as a separate draw call, so 62k clones = 62k draw
    // calls per frame.
    //
    // New approach: pre-build one BoxEdges template, transform a copy per
    // voxel into shared CPU buffers, then use `mergeGeometries` to fuse
    // them all into a single BufferGeometry. The whole map renders as ONE
    // draw call — pale-grey edges restored on dense maps without the
    // per-voxel clone cost. Sparse maps benefit just as much; no need for
    // a count-based branch any more.
    const baseEdgeGeometry = new THREE.EdgesGeometry(geometry);
    const perVoxelEdges: THREE.BufferGeometry[] = [];
    occupancyCells.forEach((cell) => {
      const height = cell.height ?? cell.size;
      const centerZ = cell.height ? cell.z + height / 2 : cell.z;
      const local = baseEdgeGeometry.clone();
      const matrix = new THREE.Matrix4()
        .compose(
          new THREE.Vector3(cell.x, cell.y, centerZ),
          new THREE.Quaternion(),
          new THREE.Vector3(cell.size * 0.96, cell.size * 0.96, height * 0.96),
        );
      local.applyMatrix4(matrix);
      perVoxelEdges.push(local);
    });
    const mergedEdges = perVoxelEdges.length > 0 ? mergeGeometries(perVoxelEdges, false) : null;
    perVoxelEdges.forEach((g) => g.dispose());
    baseEdgeGeometry.dispose();

    let edgesMesh: THREE.LineSegments | null = null;
    if (mergedEdges) {
      const edgeMaterial = new THREE.LineBasicMaterial({
        color: mapPalette.voxelEdge,
        transparent: true,
        // Pale-grey edges at low opacity preserve voxel "border感" without
        // hiding the bright trajectory routed between voxels.
        opacity: mode === "3d" ? 0.32 : 0.55,
        depthWrite: false,
      });
      edgesMesh = new THREE.LineSegments(mergedEdges, edgeMaterial);
      edgesMesh.frustumCulled = false;
    }

    if (import.meta.env.DEV) {
      (window as unknown as { __nav3dVoxelDebug?: typeof debugFirstFive }).__nav3dVoxelDebug = debugFirstFive;
      (window as unknown as { __nav3dVoxelLayers?: string[] }).__nav3dVoxelLayers = Array.from(
        new Set(occupancyCells.map((c) => c.layer ?? "global")),
      );
      (window as unknown as { __nav3dVoxelZRange?: ZRange | null }).__nav3dVoxelZRange = zRange ?? null;
      (window as unknown as { __nav3dEdgeMerged?: boolean }).__nav3dEdgeMerged = mergedEdges !== null;
    }
    // Render order: solid → wire (highlight) → edges (single merged mesh)
    // → points → pick (raycast).
    if (edgesMesh) {
      group.add(solid, pointCloud, wire, edgesMesh, pick);
    } else {
      group.add(solid, pointCloud, wire, pick);
    }
    renderScene();
  }, [mode, occupancyCells, renderScene]);

  useEffect(() => {
    applyCameraView(mode);
  }, [applyCameraView, mode]);
  useEffect(() => {
    const group = goalGroupRef.current;
    const pathGroup = pathGroupRef.current;
    if (!group || !pathGroup) return;
    group.children.forEach((child) => disposeObject(child));
    group.clear();
    pathGroup.children.forEach((child) => disposeObject(child));
    pathGroup.clear();

    if (startPoint) {
      const marker = new THREE.Mesh(
        new THREE.CylinderGeometry(0.24, 0.24, 0.28, 28),
        new THREE.MeshBasicMaterial({ color: mapPalette.start }),
      );
      marker.rotation.x = Math.PI / 2;
      marker.position.set(startPoint.x, startPoint.y, startPoint.z + 0.14);
      group.add(marker);
      const ring = new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.BoxGeometry(0.78, 0.78, 0.06)),
        new THREE.LineBasicMaterial({ color: mapPalette.start, transparent: true, opacity: 0.95 }),
      );
      ring.position.set(startPoint.x, startPoint.y, startPoint.z + 0.04);
      group.add(ring);
      const label = createPointLabel("S", "#2BD9A8");
      label.position.set(startPoint.x, startPoint.y, startPoint.z + 0.78);
      group.add(label);
    }

    goals.forEach((goal, index) => {
      const marker = new THREE.Mesh(
        new THREE.BoxGeometry(0.32, 0.32, 0.32),
        new THREE.MeshBasicMaterial({ color: mapPalette.robotBase }),
      );
      marker.position.set(goal.x, goal.y, goal.z + 0.16);
      group.add(marker);
      const ring = new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.BoxGeometry(0.72, 0.72, 0.06)),
        new THREE.LineBasicMaterial({ color: mapPalette.goal, transparent: true, opacity: 0.95 }),
      );
      ring.position.set(goal.x, goal.y, goal.z + 0.04);
      group.add(ring);
      const label = createGoalLabel(String(index + 1));
      label.position.set(goal.x, goal.y, goal.z + 0.72);
      group.add(label);
    });
    if (routePoints.length >= 2) {
      const { traveled, remaining, local } = splitPolylineAtProgress(routePoints, progress);
      // Black underlay + bright green primary — operator-requested duotone.
      // The underlay sits 1px below in z so the eye reads a crisp black
      // outline around every green stroke regardless of voxel hue beneath.
      const drawDuoLine = (
        pts: typeof traveled,
        primary: number,
        underlay: number,
        primaryOpacity: number,
        underlayOpacity: number,
        lift: number,
      ) => {
        if (pts.length < 2) return;
        const underlayLine = new THREE.Line(
          new THREE.BufferGeometry().setFromPoints(pts.map((p) => new THREE.Vector3(p.x, p.y, p.z + lift - 0.005))),
          new THREE.LineBasicMaterial({ color: underlay, transparent: true, opacity: underlayOpacity, depthWrite: false }),
        );
        const primaryLine = new THREE.Line(
          new THREE.BufferGeometry().setFromPoints(pts.map((p) => new THREE.Vector3(p.x, p.y, p.z + lift))),
          new THREE.LineBasicMaterial({ color: primary, transparent: true, opacity: primaryOpacity, depthWrite: false }),
        );
        pathGroup.add(underlayLine, primaryLine);
      };

      // Split the flattened polyline into "planned" and "placeholder" sub-
      // sequences by walking the per-segment kinds. Placeholder legs render
      // as a dashed warm-grey line (LineDashedMaterial requires
      // computeLineDistances). Planned legs use the bright duotone above.
      // We pick the global progress sentinel only when the *first* leg is
      // planned — operators look at the green path for "已走" feedback, and
      // the placeholder preview should never be tinted by progress.
      const drawDashedLine = (pts: Point3[], color: number, opacity: number, lift: number) => {
        if (pts.length < 2) return;
        const geometry = new THREE.BufferGeometry().setFromPoints(
          pts.map((p) => new THREE.Vector3(p.x, p.y, p.z + lift)),
        );
        const line = new THREE.Line(
          geometry,
          new THREE.LineDashedMaterial({
            color,
            transparent: true,
            opacity,
            depthWrite: false,
            dashSize: placeholderDash.dashSize,
            gapSize: placeholderDash.gapSize,
          }),
        );
        line.computeLineDistances();
        pathGroup.add(line);
      };

      if (plannedSegments.length === 0) {
        // Single-goal flow without explicit segment authorship — just draw
        // the duotone trajectory split by progress.
        drawDuoLine(traveled, mapPalette.pathTrail, mapPalette.pathUnderlay, 0.42, 0.55, 0.1);
        drawDuoLine(remaining, mapPalette.path, mapPalette.pathUnderlay, 0.98, 0.7, 0.12);
        drawDuoLine(local, mapPalette.localPath, mapPalette.pathUnderlay, 1.0, 0.65, 0.16);
      } else {
        // Walk segments and emit per-kind line primitives. We also splice
        // each "planned" segment with global progress so 已走/全局/局部 split
        // visuals survive — but only on planned legs (placeholder cannot
        // carry progress because the bridge has not curved it yet).
        const totalLength = routePoints.reduce((sum, p, i) => {
          if (i === 0) return 0;
          const prev = routePoints[i - 1];
          return sum + Math.hypot(p.x - prev.x, p.y - prev.y, p.z - prev.z);
        }, 0);
        let cursorMeters = 0;
        plannedSegments.forEach((seg) => {
          const segPoints = seg.points;
          if (segPoints.length < 2) return;
          let segLen = 0;
          for (let i = 1; i < segPoints.length; i += 1) {
            const a = segPoints[i - 1];
            const b = segPoints[i];
            segLen += Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z);
          }
          if (seg.kind === "placeholder") {
            drawDashedLine(segPoints, mapPalette.pathPlaceholder, 0.55, 0.12);
            cursorMeters += segLen;
            return;
          }
          // Map global progress to local progress within this leg.
          const legStartGlobal = totalLength > 0 ? cursorMeters / totalLength : 0;
          const legEndGlobal = totalLength > 0 ? (cursorMeters + segLen) / totalLength : 1;
          let localProgress = 0;
          if (progress <= legStartGlobal) {
            localProgress = 0;
          } else if (progress >= legEndGlobal) {
            localProgress = 1;
          } else if (legEndGlobal > legStartGlobal) {
            localProgress = (progress - legStartGlobal) / (legEndGlobal - legStartGlobal);
          }
          const split = splitPolylineAtProgress(segPoints, localProgress);
          drawDuoLine(split.traveled, mapPalette.pathTrail, mapPalette.pathUnderlay, 0.42, 0.55, 0.1);
          drawDuoLine(split.remaining, mapPalette.path, mapPalette.pathUnderlay, 0.98, 0.7, 0.12);
          drawDuoLine(split.local, mapPalette.localPath, mapPalette.pathUnderlay, 1.0, 0.65, 0.16);
          cursorMeters += segLen;
        });
        // Suppress unused warning for `traveled / remaining / local` vars in
        // the segment-kind path while preserving them for the no-segment fallback.
        void traveled;
        void remaining;
        void local;
      }
    }
    renderScene();
  }, [goals, mode, plannedSegments, progress, renderScene, routePoints, startPoint]);

  useEffect(() => {
    const robot = robotRef.current;
    if (!robot) return;
    // Only show the robot once the bridge is feeding live odom — never park it
    // at (0,0,0) just because state has not arrived yet.
    robot.visible = hasLivePose;
    if (hasLivePose) {
      robot.position.set(robotPosition.x, robotPosition.y, robotPosition.z + 0.18);
    }
    renderScene();
  }, [hasLivePose, renderScene, robotPosition]);
  const getSceneVoxelPoint = useCallback((event: React.PointerEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    const camera = cameraRef.current;
    const pickMesh = occupancyPickMeshRef.current;
    if (!canvas || !camera || !pickMesh) return null;
    const rect = canvas.getBoundingClientRect();
    pointerRef.current.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointerRef.current.y = -(((event.clientY - rect.top) / rect.height) * 2 - 1);
    raycasterRef.current.setFromCamera(pointerRef.current, camera);
    const hits = raycasterRef.current.intersectObject(pickMesh, false);
    const voxelPoint = selectVoxelTargetFromHits(occupancyCells, hits);
    if (voxelPoint) return voxelPoint;
    if (mode === "2d" && occupancyCells.some((c) => c.layer === "grid")) return null;
    const ray = raycasterRef.current.ray;
    const planningZ = mapPlanningPlaneZ(mode, mapBounds);
    if (Math.abs(ray.direction.z) < 1e-6) return null;
    const distanceToPlane = (planningZ - ray.origin.z) / ray.direction.z;
    if (distanceToPlane < 0) return null;
    const planePoint = ray.origin.clone().add(ray.direction.clone().multiplyScalar(distanceToPlane));
    const selected = roundScenePoint(sanitizePoint({ x: planePoint.x, y: planePoint.y, z: planningZ }, mode === "2d"));
    return containsMapPoint(mapBounds, selected) ? selected : null;
  }, [mapBounds, mode, occupancyCells]);

  const handlePointerDown = useCallback((event: React.PointerEvent<HTMLCanvasElement>) => {
    event.currentTarget.setPointerCapture(event.pointerId);
    interactionRef.current = {
      pointerId: event.pointerId,
      startX: event.clientX,
      startY: event.clientY,
      lastX: event.clientX,
      lastY: event.clientY,
      dragging: false,
      dragMode: mode === "2d" || event.ctrlKey ? "pan" : "orbit",
    };
  }, [mode]);

  const handlePointerMove = useCallback((event: React.PointerEvent<HTMLCanvasElement>) => {
    const interaction = interactionRef.current;
    if (!interaction || interaction.pointerId !== event.pointerId) return;
    const totalDx = event.clientX - interaction.startX;
    const totalDy = event.clientY - interaction.startY;
    if (!interaction.dragging && Math.hypot(totalDx, totalDy) >= dragThresholdPixels) interaction.dragging = true;
    if (!interaction.dragging) return;
    const dx = event.clientX - interaction.lastX;
    const dy = event.clientY - interaction.lastY;
    interaction.lastX = event.clientX;
    interaction.lastY = event.clientY;
    if (interaction.dragMode === "pan") {
      const span = boundsSpan(mapBounds);
      const scale = Math.max(span.x, span.y, span.z) / 900;
      cameraTargetRef.current.x -= dx * scale;
      cameraTargetRef.current.y += dy * scale;
      if (mode === "3d") cameraTargetRef.current.z += dy * scale * 0.35;
    } else {
      const orbit = orbitRef.current;
      orbit.yaw -= dx * 0.008;
      orbit.pitch = Math.max(0.18, Math.min(1.2, orbit.pitch + dy * 0.006));
    }
    applyCameraView(mode);
  }, [applyCameraView, mapBounds, mode]);

  const handlePointerUp = useCallback((event: React.PointerEvent<HTMLCanvasElement>) => {
    const interaction = interactionRef.current;
    if (!interaction || interaction.pointerId !== event.pointerId) return;
    interactionRef.current = null;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
    if (interaction.dragging) return;
    const point = getSceneVoxelPoint(event);
    if (point) onScenePoint(point);
    else onSceneMiss();
  }, [getSceneVoxelPoint, onSceneMiss, onScenePoint]);

  const handlePointerCancel = useCallback((event: React.PointerEvent<HTMLCanvasElement>) => {
    interactionRef.current = null;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
  }, []);

  const handleWheel = useCallback((event: React.WheelEvent<HTMLCanvasElement>) => {
    event.preventDefault();
    const orbit = orbitRef.current;
    const deltaPixels =
      event.deltaMode === WheelEvent.DOM_DELTA_LINE
        ? event.deltaY * 16
        : event.deltaMode === WheelEvent.DOM_DELTA_PAGE
          ? event.deltaY * window.innerHeight
          : event.deltaY;
    const clampedDelta = Math.max(-240, Math.min(240, deltaPixels));
    const zoomFactor = Math.exp(clampedDelta * 0.0012);
    orbit.distance = Math.max(orbit.minDistance, Math.min(orbit.maxDistance, orbit.distance * zoomFactor));
    applyCameraView(mode);
  }, [applyCameraView, mode]);
  useEffect(() => {
    if (!import.meta.env.DEV) return undefined;
    window.__nav3dSceneProbe = {
      projectVoxel: (point) => {
        const canvas = canvasRef.current;
        const camera = cameraRef.current;
        if (!canvas || !camera) return null;
        const nearestCell = occupancyCells.reduce<OccupancyCell | null>((nearest, cell) => {
          if (!nearest) return cell;
          const nd = (nearest.x - point.x) ** 2 + (nearest.y - point.y) ** 2 + (nearest.z - point.z) ** 2;
          const cd = (cell.x - point.x) ** 2 + (cell.y - point.y) ** 2 + (cell.z - point.z) ** 2;
          return cd < nd ? cell : nearest;
        }, null);
        if (!nearestCell) return null;
        const rect = canvas.getBoundingClientRect();
        const height = nearestCell.height ?? nearestCell.size;
        const centerZ = nearestCell.height ? nearestCell.z + height / 2 : nearestCell.z;
        const projected = new THREE.Vector3(nearestCell.x, nearestCell.y, centerZ).project(camera);
        if (projected.z < -1 || projected.z > 1 || projected.x < -1 || projected.x > 1 || projected.y < -1 || projected.y > 1) return null;
        return {
          x: rect.left + ((projected.x + 1) / 2) * rect.width,
          y: rect.top + ((1 - projected.y) / 2) * rect.height,
        };
      },
      projectPoint: (point) => {
        const canvas = canvasRef.current;
        const camera = cameraRef.current;
        if (!canvas || !camera || !containsMapPoint(mapBounds, point)) return null;
        const rect = canvas.getBoundingClientRect();
        const planningZ = mapPlanningPlaneZ(mode, mapBounds);
        const projected = new THREE.Vector3(point.x, point.y, planningZ).project(camera);
        if (projected.z < -1 || projected.z > 1 || projected.x < -1 || projected.x > 1 || projected.y < -1 || projected.y > 1) return null;
        return {
          x: rect.left + ((projected.x + 1) / 2) * rect.width,
          y: rect.top + ((1 - projected.y) / 2) * rect.height,
        };
      },
      cameraTarget: () => ({
        x: cameraTargetRef.current.x,
        y: cameraTargetRef.current.y,
        z: cameraTargetRef.current.z,
      }),
      routePoints: () => routePoints.map((p) => ({ ...p })),
    };
    return () => { delete window.__nav3dSceneProbe; };
  }, [occupancyCells, mapBounds, mode, routePoints]);
  return (
    <canvas
      ref={canvasRef}
      className="scene-canvas"
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerCancel={handlePointerCancel}
      onWheel={handleWheel}
      role="img"
      aria-label="三维 OctoMap 场景"
      tabIndex={0}
    />
  );
}












