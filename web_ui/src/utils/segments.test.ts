import { describe, expect, it } from "vitest";
import {
  flattenSegments,
  progressAtSegmentEnd,
  segmentsTotalLength,
  type PlannedSegment,
} from "./trajectory";

const seg = (
  goalIndex: number,
  points: number[][],
  kind: PlannedSegment["kind"] = "planned",
): PlannedSegment => ({
  goalIndex,
  start: { x: points[0][0], y: points[0][1], z: points[0][2] },
  goal: {
    x: points[points.length - 1][0],
    y: points[points.length - 1][1],
    z: points[points.length - 1][2],
  },
  points: points.map(([x, y, z]) => ({ x, y, z })),
  kind,
});

describe("PlannedSegment helpers", () => {
  it("flattenSegments dedupes the joint between adjacent legs", () => {
    const segments = [
      seg(0, [
        [0, 0, 0],
        [2, 0, 0],
        [4, 0, 0],
      ]),
      seg(1, [
        [4, 0, 0],
        [4, 3, 0],
      ]),
    ];
    expect(flattenSegments(segments)).toEqual([
      { x: 0, y: 0, z: 0 },
      { x: 2, y: 0, z: 0 },
      { x: 4, y: 0, z: 0 },
      { x: 4, y: 3, z: 0 },
    ]);
  });

  it("flattenSegments keeps a non-coincident joint as two distinct vertices", () => {
    const segments = [
      seg(0, [
        [0, 0, 0],
        [2, 0, 0],
      ]),
      seg(1, [
        [3, 0, 0],
        [3, 1, 0],
      ]),
    ];
    expect(flattenSegments(segments)).toHaveLength(4);
  });

  it("flattenSegments skips empty leg points", () => {
    const segments: PlannedSegment[] = [
      seg(0, [
        [0, 0, 0],
        [1, 0, 0],
      ]),
      { goalIndex: 1, start: { x: 1, y: 0, z: 0 }, goal: { x: 2, y: 0, z: 0 }, points: [], kind: "planned" },
      seg(2, [
        [1, 0, 0],
        [2, 0, 0],
      ]),
    ];
    expect(flattenSegments(segments)).toEqual([
      { x: 0, y: 0, z: 0 },
      { x: 1, y: 0, z: 0 },
      { x: 2, y: 0, z: 0 },
    ]);
  });

  it("placeholder and planned legs flatten into the same polyline shape", () => {
    // The "kind" tag is purely visual; flattenSegments / total-length /
    // progress math must stay agnostic so the cruise watcher and progress
    // projection keep working before the bridge replaces the placeholder.
    const segments = [
      seg(
        0,
        [
          [0, 0, 0],
          [2, 0, 0],
        ],
        "placeholder",
      ),
      seg(
        1,
        [
          [2, 0, 0],
          [2, 3, 0],
        ],
        "planned",
      ),
    ];
    expect(flattenSegments(segments)).toEqual([
      { x: 0, y: 0, z: 0 },
      { x: 2, y: 0, z: 0 },
      { x: 2, y: 3, z: 0 },
    ]);
    expect(segmentsTotalLength(segments)).toBeCloseTo(5, 6);
    expect(progressAtSegmentEnd(segments, 0)).toBeCloseTo(2 / 5, 4);
  });

  it("segmentsTotalLength sums leg lengths", () => {
    const segments = [
      seg(0, [
        [0, 0, 0],
        [3, 0, 0],
      ]),
      seg(1, [
        [3, 0, 0],
        [3, 4, 0],
      ]),
    ];
    expect(segmentsTotalLength(segments)).toBeCloseTo(7, 6);
  });

  it("progressAtSegmentEnd returns cumulative ratio at leg end", () => {
    const segments = [
      seg(0, [
        [0, 0, 0],
        [3, 0, 0],
      ]),
      seg(1, [
        [3, 0, 0],
        [3, 4, 0],
      ]),
    ];
    expect(progressAtSegmentEnd(segments, 0)).toBeCloseTo(3 / 7, 4);
    expect(progressAtSegmentEnd(segments, 1)).toBeCloseTo(1, 6);
  });

  it("progressAtSegmentEnd saturates at 1 for out-of-range index", () => {
    const segments = [
      seg(0, [
        [0, 0, 0],
        [1, 0, 0],
      ]),
    ];
    expect(progressAtSegmentEnd(segments, 5)).toBe(1);
    expect(progressAtSegmentEnd([], 0)).toBe(1);
  });
});
