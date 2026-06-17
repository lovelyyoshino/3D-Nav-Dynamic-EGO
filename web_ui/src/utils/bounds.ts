import type { OccupancyCell } from "../rosbridge/client";
import type { NavigationMode } from "../rosbridge/client";
import { Point3, sanitizePoint } from "./trajectory";

export type Bounds = {
  min: Point3;
  max: Point3;
};

export const fallbackBounds: Bounds = {
  min: { x: -10, y: -10, z: 0 },
  max: { x: 10, y: 10, z: 3 },
};

export function boundsFromCells(cells: OccupancyCell[]): Bounds {
  if (cells.length === 0) {
    return fallbackBounds;
  }

  const bounds: Bounds = {
    min: { x: Number.POSITIVE_INFINITY, y: Number.POSITIVE_INFINITY, z: Number.POSITIVE_INFINITY },
    max: { x: Number.NEGATIVE_INFINITY, y: Number.NEGATIVE_INFINITY, z: Number.NEGATIVE_INFINITY },
  };

  cells.forEach((cell) => {
    const height = cell.height ?? cell.size;
    const centerZ = cell.height ? cell.z + height / 2 : cell.z;
    bounds.min.x = Math.min(bounds.min.x, cell.x - cell.size / 2);
    bounds.min.y = Math.min(bounds.min.y, cell.y - cell.size / 2);
    bounds.min.z = Math.min(bounds.min.z, centerZ - height / 2);
    bounds.max.x = Math.max(bounds.max.x, cell.x + cell.size / 2);
    bounds.max.y = Math.max(bounds.max.y, cell.y + cell.size / 2);
    bounds.max.z = Math.max(bounds.max.z, centerZ + height / 2);
  });

  return bounds;
}

export function boundsCenter(bounds: Bounds): Point3 {
  return {
    x: (bounds.min.x + bounds.max.x) / 2,
    y: (bounds.min.y + bounds.max.y) / 2,
    z: (bounds.min.z + bounds.max.z) / 2,
  };
}

export function boundsSpan(bounds: Bounds): Point3 {
  return {
    x: Math.max(1, bounds.max.x - bounds.min.x),
    y: Math.max(1, bounds.max.y - bounds.min.y),
    z: Math.max(1, bounds.max.z - bounds.min.z),
  };
}

export function boundsDiagonal(bounds: Bounds): number {
  const span = boundsSpan(bounds);
  return Math.max(6, Math.hypot(span.x, span.y, span.z));
}

export function clickPlanningPlaneZ(mode: NavigationMode, bounds: Bounds): number | null {
  if (mode === "2d") {
    return 0;
  }
  const span = boundsSpan(bounds);
  if (span.z <= 4) {
    return null;
  }
  return Math.max(0, bounds.min.z + 1.5);
}

export function mapPlanningPlaneZ(mode: NavigationMode, bounds: Bounds): number {
  return clickPlanningPlaneZ(mode, bounds) ?? Math.min(0, bounds.min.z);
}

export function containsMapPoint(bounds: Bounds, point: Point3): boolean {
  return (
    point.x >= bounds.min.x &&
    point.x <= bounds.max.x &&
    point.y >= bounds.min.y &&
    point.y <= bounds.max.y
  );
}

export function roundScenePoint(point: Point3): Point3 {
  return {
    x: Math.round(point.x * 100) / 100,
    y: Math.round(point.y * 100) / 100,
    z: Math.round(point.z * 100) / 100,
  };
}

export function projectPointForMode(point: Point3, mode: NavigationMode): Point3 {
  return mode === "2d" ? { ...point, z: 0 } : point;
}

export function projectPointsForMode(points: Point3[], mode: NavigationMode): Point3[] {
  return mode === "2d" ? points.map((point) => ({ ...point, z: 0 })) : points;
}

export { sanitizePoint };
