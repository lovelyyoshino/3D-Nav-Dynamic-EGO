import * as THREE from "three";
import type { OccupancyCell } from "../rosbridge/client";

export const mapPalette = {
  background: 0x050912,
  floor: 0x16273f,
  gridPrimary: 0x2a4a78,
  gridSecondary: 0x1f2f4a,
  frame: 0x4dc4ff,
  xAxis: 0x4dc4ff,
  yAxis: 0xb07cff,
  globalVoxel: 0x4dc4ff,
  globalVoxelHighlight: 0x9be3ff,
  // Soft pale-grey voxel border so the trajectory underneath stays readable.
  // Darker borders (e.g. 0x050912) hid path strokes against densely-packed
  // OctoMaps; pale grey gives just enough edge contrast without blocking the
  // bright green path.
  voxelEdge: 0x9aa6bd,
  localVoxel: 0xff7ad2,
  start: 0x2bd9a8,
  goal: 0xff5d8f,
  // Trajectory uses a black underlay + bright green primary so the path is
  // unambiguously readable on both the navy substrate and bright voxel
  // clusters. Operator request — replaces the cobalt path used in v3.x.
  pathUnderlay: 0x050912,
  path: 0x2bd9a8,
  pathTrail: 0x4a5a76,
  localPath: 0xa7ff8f,
  // Placeholder: a queued multi-goal leg that the bridge has not yet planned.
  // Drawn as a dashed warm-grey line so the operator can see the goal queue
  // taking shape without confusing it with the bright "planned" path. Lower
  // alpha keeps it visually subordinate to bright voxels and the green path.
  pathPlaceholder: 0x8a93a8,
  robotBase: 0xe6edf7,
  robotAccent: 0xb07cff,
};

/**
 * Dash spacing for the placeholder leg. Tuned so a 4-meter span shows ~8
 * dashes on the building2_9 reference scale; LineDashedMaterial requires
 * `computeLineDistances()` to be called on the geometry.
 */
export const placeholderDash = { dashSize: 0.35, gapSize: 0.18 };

const labelBg = "#050912";
const labelText = "#E6EDF7";
const goalStroke = "#FF5D8F";

function paintLabelCanvas(label: string, stroke: string): HTMLCanvasElement {
  const canvas = document.createElement("canvas");
  canvas.width = 96;
  canvas.height = 96;
  const context = canvas.getContext("2d");
  if (!context) {
    throw new Error("Canvas 2D context is unavailable");
  }
  context.fillStyle = labelBg;
  context.fillRect(18, 18, 60, 60);
  context.strokeStyle = stroke;
  context.lineWidth = 4;
  context.strokeRect(18, 18, 60, 60);
  context.fillStyle = labelText;
  context.font = "700 40px 'JetBrains Mono', 'Courier New', monospace";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(label, 48, 51);
  return canvas;
}

function spriteFromCanvas(canvas: HTMLCanvasElement): THREE.Sprite {
  const texture = new THREE.CanvasTexture(canvas);
  texture.needsUpdate = true;
  const material = new THREE.SpriteMaterial({ map: texture, depthTest: false });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(0.7, 0.7, 0.7);
  return sprite;
}

export function createGoalLabel(label: string): THREE.Sprite {
  return spriteFromCanvas(paintLabelCanvas(label, goalStroke));
}

export function createPointLabel(label: string, color: string): THREE.Sprite {
  return spriteFromCanvas(paintLabelCanvas(label, color));
}

export function disposeObject(object: THREE.Object3D): void {
  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (mesh.geometry) {
      mesh.geometry.dispose();
    }
    const material = mesh.material as THREE.Material | THREE.Material[] | undefined;
    if (Array.isArray(material)) {
      material.forEach((entry) => entry.dispose());
    } else if (material) {
      material.dispose();
    }
  });
}

export type ZRange = { min: number; max: number };

/**
 * Color a cell by layer, with optional Z-banded HSL hue ramp for global voxels
 * so dense OctoMaps reveal vertical structure. Hue ramps cyan (low) → violet
 * (high) — kept inside the 200°-320° wedge so e2e canvas asserts (which check
 * blue/cyan pixel counts) keep passing. Local-layer voxels stay magenta and
 * grid cells use bright contrast colors readable on the deep navy substrate.
 */
export function cellColor(cell: OccupancyCell, zRange?: ZRange): THREE.Color {
  if (cell.layer === "local") {
    const c = new THREE.Color();
    c.setStyle("#ff7ad2", THREE.SRGBColorSpace);
    return c;
  }
  if (cell.layer === "grid") {
    const c = new THREE.Color();
    if (cell.occupancy === "occupied") {
      c.setStyle("#4dc4ff", THREE.SRGBColorSpace);
    } else if (cell.occupancy === "unknown") {
      c.setStyle("#6b80a8", THREE.SRGBColorSpace);
    } else {
      c.setStyle("#e6edf7", THREE.SRGBColorSpace);
    }
    return c;
  }

  // Global 3D voxel: blend the cobalt base with a Z-banded hue ramp.
  if (zRange && Number.isFinite(zRange.min) && Number.isFinite(zRange.max) && zRange.max > zRange.min) {
    const t = Math.max(0, Math.min(1, (cell.z - zRange.min) / (zRange.max - zRange.min)));
    // hue: 200 (cyan) → 290 (violet); saturation 90%; lightness 72 → 86.
    // High lightness is intentional — three.js ColorManagement converts the
    // working LinearSRGB color back to sRGB at render time, so a "feels too
    // bright" sRGB lightness lands as the desired mid-tone in the framebuffer.
    // CRITICAL: the CSS hsl() string MUST NOT include the `deg` unit suffix
    // — three.js's setStyle() regex only accepts bare numbers and trailing
    // "%". `hsl(200deg, ...)` silently fails and leaves the Color at its
    // construction value (white), which then renders as a black voxel after
    // the InstancedMesh's instanceColor=white pre-multiplies a substrate-tone
    // pixel. This was the root cause of the operator-visible "still black".
    const hueDeg = Math.round(200 + t * 90);
    const lightnessPct = Math.round((0.72 + t * 0.14) * 100);
    const color = new THREE.Color();
    color.setStyle(`hsl(${hueDeg}, 90%, ${lightnessPct}%)`, THREE.SRGBColorSpace);
    return color;
  }
  // Fallback also goes through SRGB to stay consistent.
  const fallback = new THREE.Color();
  fallback.setStyle("rgb(77, 196, 255)", THREE.SRGBColorSpace);
  return fallback;
}

export function isGridCell(cell: OccupancyCell): boolean {
  return cell.layer === "grid";
}
