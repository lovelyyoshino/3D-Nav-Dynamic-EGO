export type Point3 = {
  x: number;
  y: number;
  z: number;
};

export type VoxelTarget = Point3 & {
  size?: number;
  height?: number;
  occupancy?: "occupied" | "free" | "unknown";
};

export type VoxelRayHit = {
  instanceId?: number | null;
  distance?: number;
};

export type TrajectoryAdvance = {
  position: Point3;
  progress: number;
  complete: boolean;
};

function distance(a: Point3, b: Point3): number {
  return Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z);
}

export function getPathLength(points: Point3[]): number {
  return points.reduce((length, point, index) => {
    if (index === 0) {
      return length;
    }
    return length + distance(points[index - 1], point);
  }, 0);
}

export function clampProgress(progress: number): number {
  if (!Number.isFinite(progress)) {
    return 0;
  }
  return Math.min(1, Math.max(0, progress));
}

export function samplePolyline(points: Point3[], progress: number): Point3 {
  if (points.length === 0) {
    return { x: 0, y: 0, z: 0 };
  }

  if (points.length === 1) {
    return { ...points[0] };
  }

  const clampedProgress = clampProgress(progress);
  const totalLength = getPathLength(points);
  if (totalLength === 0) {
    return { ...points[0] };
  }

  let remaining = totalLength * clampedProgress;
  for (let index = 1; index < points.length; index += 1) {
    const start = points[index - 1];
    const end = points[index];
    const segmentLength = distance(start, end);
    if (segmentLength === 0) {
      continue;
    }

    if (remaining <= segmentLength) {
      const t = remaining / segmentLength;
      return {
        x: start.x + (end.x - start.x) * t,
        y: start.y + (end.y - start.y) * t,
        z: start.z + (end.z - start.z) * t,
      };
    }

    remaining -= segmentLength;
  }

  const last = points[points.length - 1];
  return { ...last };
}

export function advanceAlongPolyline(
  points: Point3[],
  currentProgress: number,
  distanceMeters: number,
): TrajectoryAdvance {
  const totalLength = getPathLength(points);
  if (totalLength === 0) {
    const progress = points.length === 0 ? 0 : 1;
    return {
      position: samplePolyline(points, progress),
      progress,
      complete: progress >= 1,
    };
  }

  const distanceProgress =
    Number.isFinite(distanceMeters) && distanceMeters > 0 ? distanceMeters / totalLength : 0;
  const progress = clampProgress(clampProgress(currentProgress) + distanceProgress);
  return {
    position: samplePolyline(points, progress),
    progress,
    complete: progress >= 1,
  };
}

export function formatCoordinate(value: number): string {
  return value.toFixed(2);
}

export function sanitizePoint(point: Point3, lockZ: boolean): Point3 {
  return {
    x: Number.isFinite(point.x) ? point.x : 0,
    y: Number.isFinite(point.y) ? point.y : 0,
    z: lockZ || !Number.isFinite(point.z) ? 0 : point.z,
  };
}

function voxelVisualCenter(cell: VoxelTarget): Point3 {
  const height = cell.height ?? cell.size ?? 0;
  return sanitizePoint(
    {
      x: cell.x,
      y: cell.y,
      z: cell.height ? cell.z + height / 2 : cell.z,
    },
    false,
  );
}

function voxelTopSurface(cell: VoxelTarget): Point3 {
  const height = cell.height ?? cell.size ?? 0;
  return sanitizePoint(
    {
      x: cell.x,
      y: cell.y,
      z: cell.height ? cell.z + height : cell.z + height / 2,
    },
    false,
  );
}

export function selectVoxelTarget(cells: VoxelTarget[], instanceId: number | null | undefined): Point3 | null {
  if (instanceId === null || instanceId === undefined || !Number.isInteger(instanceId)) {
    return null;
  }
  const cell = cells[instanceId];
  if (!cell) {
    return null;
  }
  if (cell.occupancy && cell.occupancy !== "free") {
    return null;
  }
  return voxelTopSurface(cell);
}

export function selectVoxelTargetFromHits(cells: VoxelTarget[], hits: VoxelRayHit[]): Point3 | null {
  let selected: { point: Point3; z: number; distance: number } | null = null;
  const seen = new Set<number>();

  for (const hit of hits) {
    const instanceId = hit.instanceId;
    if (instanceId === null || instanceId === undefined || !Number.isInteger(instanceId)) {
      continue;
    }
    if (seen.has(instanceId)) {
      continue;
    }
    seen.add(instanceId);

    const point = selectVoxelTarget(cells, instanceId);
    if (!point) {
      continue;
    }
    const distanceValue = Number(hit.distance);
    const distanceMeters = Number.isFinite(distanceValue) ? distanceValue : Number.POSITIVE_INFINITY;
    if (
      !selected ||
      point.z > selected.z + 1e-9 ||
      (Math.abs(point.z - selected.z) <= 1e-9 && distanceMeters < selected.distance)
    ) {
      selected = { point, z: point.z, distance: distanceMeters };
    }
  }

  return selected?.point ?? null;
}

export type TrajectorySplit = {
  traveled: Point3[];
  remaining: Point3[];
  local: Point3[];
};

/**
 * One leg of a multi-goal cruise: who authored it (`goalIndex`), where it
 * starts/ends, and the polyline the bridge planner returned for it. The
 * flatten of all segments is what `splitPolylineAtProgress` and the e2e
 * `routePoints` probe operate on, so per-leg authorship stays additive — no
 * leg loses its trajectory just because a later goal triggered a replan.
 *
 * `kind` distinguishes web-side preview (`placeholder`, drawn as a dashed
 * grey straight line between previous endpoint and the new goal) from a leg
 * that bridge has already planned (`planned`, drawn as the bright duotone
 * trajectory). Without this tag the operator cannot tell whether a goal has
 * actually been dispatched to the bridge — every leg looks the same.
 */
export type PlannedSegmentKind = "placeholder" | "planned";

export type PlannedSegment = {
  goalIndex: number;
  start: Point3;
  goal: Point3;
  points: Point3[];
  kind: PlannedSegmentKind;
};

/**
 * Concatenate all leg polylines into one flat polyline, dropping the
 * duplicated joint vertex between adjacent legs (segment B starts at
 * segment A's end). The result matches what `appendTrajectorySegment` used
 * to produce, so the e2e `getSceneRoutePoints` shape is preserved.
 */
export function flattenSegments(segments: PlannedSegment[]): Point3[] {
  const flat: Point3[] = [];
  segments.forEach((segment) => {
    if (segment.points.length === 0) return;
    if (flat.length === 0) {
      flat.push(...segment.points);
      return;
    }
    const last = flat[flat.length - 1];
    const first = segment.points[0];
    const sameJoint =
      Math.hypot(last.x - first.x, last.y - first.y, last.z - first.z) < 1e-3;
    flat.push(...(sameJoint ? segment.points.slice(1) : segment.points));
  });
  return flat;
}

/**
 * Total length of every concatenated leg in the segment list (joint dedup
 * does not affect arc length because the dropped vertex is identical).
 */
export function segmentsTotalLength(segments: PlannedSegment[]): number {
  return segments.reduce((sum, seg) => sum + getPathLength(seg.points), 0);
}

/**
 * Cumulative arc-length proportion at the **end** of leg `legIndex`
 * (i.e. the goal of that leg). Used by the cruise watcher to decide when to
 * dispatch the next leg's start+goal pair to the bridge.
 */
export function progressAtSegmentEnd(segments: PlannedSegment[], legIndex: number): number {
  const total = segmentsTotalLength(segments);
  if (total <= 0 || legIndex < 0 || legIndex >= segments.length) {
    return 1;
  }
  let acc = 0;
  for (let i = 0; i <= legIndex; i += 1) {
    acc += getPathLength(segments[i].points);
  }
  return Math.min(1, acc / total);
}

/**
 * Split a polyline at the given normalized progress (0..1) into:
 * - traveled: the portion behind the robot (fades out in the scene),
 * - remaining: the portion ahead of the robot (global path),
 * - local: the first `localHorizon` meters of the remaining path (local planning window).
 * The split point is inserted into both traveled and remaining so the lines stay connected.
 */
export function splitPolylineAtProgress(
  points: Point3[],
  progress: number,
  localHorizon = 2.5,
): TrajectorySplit {
  if (points.length < 2) {
    return { traveled: [], remaining: [...points], local: [] };
  }

  const totalLength = getPathLength(points);
  if (totalLength === 0) {
    return { traveled: [], remaining: [...points], local: [] };
  }

  const clamped = clampProgress(progress);
  if (clamped >= 1) {
    return { traveled: points.slice(), remaining: [], local: [] };
  }

  const target = totalLength * clamped;
  const split = samplePolyline(points, clamped);
  const traveled: Point3[] = [points[0]];
  const remaining: Point3[] = [];

  let traveledLength = 0;
  let inserted = false;
  for (let index = 1; index < points.length; index += 1) {
    const start = points[index - 1];
    const end = points[index];
    const segment = distance(start, end);
    if (!inserted && traveledLength + segment >= target) {
      traveled.push(split);
      remaining.push(split);
      inserted = true;
    }
    if (inserted) {
      remaining.push(end);
    } else {
      traveled.push(end);
    }
    traveledLength += segment;
  }

  if (!inserted) {
    // progress at/after the end: everything is traveled
    return { traveled: points.slice(), remaining: [], local: [] };
  }

  // Local window: first localHorizon meters of the remaining path.
  const local: Point3[] = [];
  if (remaining.length >= 2 && localHorizon > 0) {
    local.push(remaining[0]);
    let acc = 0;
    for (let index = 1; index < remaining.length; index += 1) {
      const start = remaining[index - 1];
      const end = remaining[index];
      const segment = distance(start, end);
      if (acc + segment >= localHorizon) {
        const ratio = (localHorizon - acc) / segment;
        local.push({
          x: start.x + (end.x - start.x) * ratio,
          y: start.y + (end.y - start.y) * ratio,
          z: start.z + (end.z - start.z) * ratio,
        });
        break;
      }
      local.push(end);
      acc += segment;
    }
  }

  return { traveled, remaining, local };
}
