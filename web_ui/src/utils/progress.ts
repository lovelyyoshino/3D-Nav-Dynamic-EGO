import type { Point3 } from "./trajectory";
import { clampProgress, getPathLength } from "./trajectory";

/**
 * Project a point (typically the live robot pose) orthogonally onto a polyline
 * and return the normalized arc-length progress in [0, 1] of the foot of
 * perpendicular. When the point lies past either end, progress saturates at
 * 0 or 1. Returns null when the polyline is degenerate.
 *
 * The implementation iterates segments once (O(N)) and picks the segment with
 * the smallest perpendicular distance; ties prefer the later segment so a
 * robot that legitimately passes a waypoint never snaps backward. The optional
 * `hintIndex` biases search forward — useful when called repeatedly on a
 * monotonically advancing pose to keep snapping stable across loops in the
 * route.
 */
export function projectPointToPolyline(
  points: Point3[],
  point: Point3 | null | undefined,
  hintIndex?: number,
): number | null {
  if (!point || points.length < 2) {
    return null;
  }
  const totalLength = getPathLength(points);
  if (totalLength === 0) {
    return null;
  }

  const start = Number.isInteger(hintIndex) && (hintIndex as number) >= 0 ? (hintIndex as number) : 0;
  let bestSegment = -1;
  let bestSegmentT = 0;
  let bestDistanceSq = Number.POSITIVE_INFINITY;

  for (let pass = 0; pass < 2; pass += 1) {
    const begin = pass === 0 ? start : 0;
    const end = pass === 0 ? points.length - 1 : start;
    if (end <= 0) continue;
    for (let index = begin; index < end; index += 1) {
      const a = points[index];
      const b = points[index + 1];
      const ax = b.x - a.x;
      const ay = b.y - a.y;
      const az = b.z - a.z;
      const segLenSq = ax * ax + ay * ay + az * az;
      if (segLenSq <= 1e-12) continue;
      const dx = point.x - a.x;
      const dy = point.y - a.y;
      const dz = point.z - a.z;
      const tRaw = (dx * ax + dy * ay + dz * az) / segLenSq;
      const t = Math.max(0, Math.min(1, tRaw));
      const cx = a.x + ax * t - point.x;
      const cy = a.y + ay * t - point.y;
      const cz = a.z + az * t - point.z;
      const distSq = cx * cx + cy * cy + cz * cz;
      // Prefer later segments on near-tie so monotonic progress wins.
      if (distSq < bestDistanceSq - 1e-9 || (distSq < bestDistanceSq + 1e-9 && index > bestSegment)) {
        bestSegment = index;
        bestSegmentT = t;
        bestDistanceSq = distSq;
      }
    }
    if (bestSegment >= 0) break;
  }

  if (bestSegment < 0) return null;

  let traveled = 0;
  for (let index = 0; index < bestSegment; index += 1) {
    const a = points[index];
    const b = points[index + 1];
    traveled += Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z);
  }
  const segA = points[bestSegment];
  const segB = points[bestSegment + 1];
  const segmentLength = Math.hypot(segB.x - segA.x, segB.y - segA.y, segB.z - segA.z);
  traveled += segmentLength * bestSegmentT;
  return clampProgress(traveled / totalLength);
}
