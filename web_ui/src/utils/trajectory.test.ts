import { describe, expect, it } from "vitest";
import {
  advanceAlongPolyline,
  getPathLength,
  selectVoxelTargetFromHits,
  selectVoxelTarget,
  samplePolyline,
  splitPolylineAtProgress,
} from "./trajectory";

describe("trajectory utilities", () => {
  it("samples along each segment and clamps progress to the path bounds", () => {
    const goals = [
      { x: 0, y: 0, z: 0 },
      { x: 3, y: 0, z: 0 },
      { x: 3, y: 4, z: 0 },
    ];

    expect(getPathLength(goals)).toBe(7);
    expect(samplePolyline(goals, -1)).toEqual({ x: 0, y: 0, z: 0 });
    expect(samplePolyline(goals, 0.5)).toEqual({ x: 3, y: 0.5, z: 0 });
    expect(samplePolyline(goals, 2)).toEqual({ x: 3, y: 4, z: 0 });
  });

  it("advances mock pose along the local trajectory by distance", () => {
    const trajectory = [
      { x: 0, y: 0, z: 0 },
      { x: 3, y: 0, z: 0 },
      { x: 3, y: 4, z: 0 },
    ];

    const first = advanceAlongPolyline(trajectory, 0, 2);
    expect(first.progress).toBeCloseTo(2 / 7);
    expect(first.position).toEqual({ x: 2, y: 0, z: 0 });
    expect(first.complete).toBe(false);

    const second = advanceAlongPolyline(trajectory, 2 / 7, 3);
    expect(second.progress).toBeCloseTo(5 / 7);
    expect(second.position.x).toBeCloseTo(3);
    expect(second.position.y).toBeCloseTo(2);
    expect(second.position.z).toBeCloseTo(0);
    expect(second.complete).toBe(false);

    expect(advanceAlongPolyline(trajectory, 5 / 7, 10)).toEqual({
      progress: 1,
      position: { x: 3, y: 4, z: 0 },
      complete: true,
    });
  });

  it("selects the voxel top surface hit by the scene ray instead of an occupied center or floor point", () => {
    const cells = [
      { x: 0, y: 0, z: 0, size: 1 },
      { x: 2, y: 1, z: 3, size: 0.5 },
    ];

    expect(selectVoxelTarget(cells, 1)).toEqual({ x: 2, y: 1, z: 3.25 });
    expect(selectVoxelTarget(cells, 99)).toBeNull();
    expect(selectVoxelTarget([{ x: 1, y: 2, z: 0, size: 1, occupancy: "occupied" }], 0)).toBeNull();
    expect(selectVoxelTarget([{ x: 1, y: 2, z: 0, size: 1, occupancy: "unknown" }], 0)).toBeNull();
    expect(selectVoxelTarget([{ x: 1, y: 2, z: 0, size: 1, occupancy: "free" }], 0)).toEqual({
      x: 1,
      y: 2,
      z: 0.5,
    });
  });

  it("selects the top of column-style cells that store a base z and height", () => {
    const cells = [{ x: 2, y: 1, z: 3, size: 0.5, height: 2 }];

    expect(selectVoxelTarget(cells, 0)).toEqual({ x: 2, y: 1, z: 5 });
  });

  it("prefers the highest voxel when stacked scene ray hits include lower cells first", () => {
    const cells = [
      { x: 0, y: 0, z: 0, size: 0.5 },
      { x: 0, y: 0, z: 4, size: 0.5 },
      { x: 1, y: 0, z: 2, size: 0.5 },
    ];

    expect(
      selectVoxelTargetFromHits(cells, [
        { instanceId: 0, distance: 1 },
        { instanceId: 2, distance: 0.5 },
        { instanceId: 1, distance: 3 },
      ]),
    ).toEqual({ x: 0, y: 0, z: 4.25 });
  });

  it("splits a polyline into traveled, remaining and local segments at progress", () => {
    const route = [
      { x: 0, y: 0, z: 0 },
      { x: 4, y: 0, z: 0 },
      { x: 8, y: 0, z: 0 },
    ];

    const split = splitPolylineAtProgress(route, 0.5, 2.5);
    // progress 0.5 of length 8 => split point at x=4
    expect(split.traveled[split.traveled.length - 1]).toEqual({ x: 4, y: 0, z: 0 });
    expect(split.remaining[0]).toEqual({ x: 4, y: 0, z: 0 });
    expect(getPathLength(split.traveled)).toBeCloseTo(4);
    expect(getPathLength(split.remaining)).toBeCloseTo(4);
    // local window is first 2.5 m of remaining
    expect(getPathLength(split.local)).toBeCloseTo(2.5);
  });

  it("treats a fully traveled route as all traveled with no remaining path", () => {
    const route = [
      { x: 0, y: 0, z: 0 },
      { x: 4, y: 0, z: 0 },
    ];

    const split = splitPolylineAtProgress(route, 1, 2.5);
    expect(split.remaining).toEqual([]);
    expect(split.local).toEqual([]);
    expect(getPathLength(split.traveled)).toBeCloseTo(4);
  });

});
