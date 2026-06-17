import { describe, expect, it } from "vitest";
import { expectedMapTopic, mapSourceLabel, selectDisplayedMap, mapPresets, findMapPreset } from "./mapSource";
import type { OccupancyCell } from "../rosbridge/client";

const voxelCells: OccupancyCell[] = [{ x: 1, y: 2, z: 3, size: 0.2, layer: "global" }];
const gridCells: OccupancyCell[] = [
  { x: 1, y: 2, z: 0.1, size: 0.2, layer: "grid", occupancy: "free" },
];

describe("map source selection", () => {
  it("uses only 3D MarkerArray voxels in 3D mode", () => {
    expect(selectDisplayedMap({ mode: "3d", voxelCells, gridCells })).toEqual({
      source: "ros-voxels",
      cells: voxelCells,
    });
  });

  it("does not fall back to 2D OccupancyGrid in 3D mode", () => {
    expect(selectDisplayedMap({ mode: "3d", voxelCells: [], gridCells })).toEqual({
      source: "ros-empty",
      cells: [],
    });
  });

  it("uses only 2D OccupancyGrid in 2D mode", () => {
    expect(selectDisplayedMap({ mode: "2d", voxelCells, gridCells })).toEqual({
      source: "ros-grid",
      cells: gridCells,
    });
  });

  it("labels the expected map topic by current mode", () => {
    expect(mapSourceLabel("ros-empty", "3d")).toBe("等待 ROS 3D OctoMap");
    expect(mapSourceLabel("ros-empty", "2d")).toBe("等待 ROS 2D 栅格");
    expect(expectedMapTopic("3d")).toBe("/nav3d/planning_occupied_markers");
    expect(expectedMapTopic("2d")).toBe("/nav3d/occupied_grid");
  });
});

describe("map presets", () => {
  it("exposes curated maps with unique ids and pcd paths", () => {
    expect(mapPresets.length).toBeGreaterThanOrEqual(2);
    const ids = mapPresets.map((preset) => preset.id);
    expect(new Set(ids).size).toBe(ids.length);
    mapPresets.forEach((preset) => {
      expect(preset.path).toMatch(/\.pcd$/);
      expect(preset.label.length).toBeGreaterThan(0);
    });
  });

  it("finds a preset by id and returns undefined for unknown ids", () => {
    expect(findMapPreset(mapPresets[0].id)?.path).toBe(mapPresets[0].path);
    expect(findMapPreset("does-not-exist")).toBeUndefined();
  });
});
