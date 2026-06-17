import { describe, expect, it } from "vitest";
import { projectPointToPolyline } from "./progress";

const polyline = [
  { x: 0, y: 0, z: 0 },
  { x: 4, y: 0, z: 0 },
  { x: 4, y: 3, z: 0 },
];

describe("projectPointToPolyline", () => {
  it("returns 0 at the polyline start", () => {
    expect(projectPointToPolyline(polyline, { x: 0, y: 0, z: 0 })).toBeCloseTo(0, 6);
  });

  it("returns 1 at the polyline end", () => {
    expect(projectPointToPolyline(polyline, { x: 4, y: 3, z: 0 })).toBeCloseTo(1, 6);
  });

  it("snaps an off-path point to the closest perpendicular", () => {
    // Closest foot is (2, 0, 0) on segment 0 — half the first leg.
    const result = projectPointToPolyline(polyline, { x: 2, y: 1, z: 0 });
    expect(result).toBeCloseTo(2 / 7, 4);
  });

  it("saturates progress past the end without overshoot", () => {
    expect(projectPointToPolyline(polyline, { x: 10, y: 6, z: 0 })).toBeCloseTo(1, 6);
  });

  it("clamps progress before the start without going negative", () => {
    const result = projectPointToPolyline(polyline, { x: -5, y: -2, z: 0 });
    expect(result).toBeGreaterThanOrEqual(0);
    expect(result).toBeLessThanOrEqual(0.05);
  });

  it("returns null for a degenerate polyline", () => {
    expect(projectPointToPolyline([{ x: 0, y: 0, z: 0 }], { x: 1, y: 1, z: 0 })).toBeNull();
    expect(projectPointToPolyline([], { x: 0, y: 0, z: 0 })).toBeNull();
  });

  it("returns null when point is missing", () => {
    expect(projectPointToPolyline(polyline, null)).toBeNull();
  });

  it("prefers the later segment on near-equal distances (monotonic progress)", () => {
    // Equidistant from corner; resolve to later segment.
    const result = projectPointToPolyline(polyline, { x: 4, y: 0, z: 0 });
    expect(result).toBeCloseTo(4 / 7, 4);
  });
});
