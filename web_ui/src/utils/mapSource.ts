import type { NavigationMode, OccupancyCell } from "../rosbridge/client";

export type MapSource = "ros-voxels" | "ros-grid" | "ros-empty";

export type DisplayedMap = {
  source: MapSource;
  cells: OccupancyCell[];
};

/**
 * Cheap content-fingerprint comparison for two voxel-cell arrays.
 *
 * The bridge republishes the FULL OctoMap (62k+ cells for building2_9) at
 * 1 Hz plus on every local_pointcloud update. Reference equality on the
 * parsed array always fails (each ROS callback produces a fresh array), so
 * we'd re-upload the entire mesh to the GPU on every tick.
 *
 * Comparison strategy:
 *   - same length + same global-layer count
 *   - first / middle / last cell XYZ + layer match
 *
 * Returns true when the arrays are *probably* identical and the caller can
 * keep the prior reference. Designed for use inside a setState updater.
 */
export function cellsLikelyUnchanged(
  prev: OccupancyCell[],
  next: OccupancyCell[],
): boolean {
  if (prev === next) return true;
  if (prev.length !== next.length) return false;
  if (prev.length === 0) return true;
  let prevGlobal = 0;
  let nextGlobal = 0;
  for (let i = 0; i < prev.length; i++) {
    if ((prev[i].layer ?? "global") !== "local") prevGlobal++;
  }
  for (let i = 0; i < next.length; i++) {
    if ((next[i].layer ?? "global") !== "local") nextGlobal++;
  }
  if (prevGlobal !== nextGlobal) return false;
  const samples = [0, Math.floor(prev.length / 2), prev.length - 1];
  for (const idx of samples) {
    const a = prev[idx];
    const b = next[idx];
    if (a.x !== b.x || a.y !== b.y || a.z !== b.z) return false;
    if ((a.layer ?? "global") !== (b.layer ?? "global")) return false;
  }
  return true;
}

export function selectDisplayedMap({
  mode,
  voxelCells,
  gridCells,
}: {
  mode: NavigationMode;
  voxelCells: OccupancyCell[];
  gridCells: OccupancyCell[];
}): DisplayedMap {
  if (mode === "3d") {
    return voxelCells.length > 0
      ? { source: "ros-voxels", cells: voxelCells }
      : { source: "ros-empty", cells: [] };
  }

  return gridCells.length > 0
    ? { source: "ros-grid", cells: gridCells }
    : { source: "ros-empty", cells: [] };
}

export function mapSourceLabel(source: MapSource, mode: NavigationMode): string {
  if (source === "ros-voxels") {
    return "ROS 3D OctoMap";
  }
  if (source === "ros-grid") {
    return "ROS 2D 栅格";
  }
  return mode === "3d" ? "等待 ROS 3D OctoMap" : "等待 ROS 2D 栅格";
}

export function expectedMapTopic(mode: NavigationMode): string {
  return mode === "3d" ? "/nav3d/planning_occupied_markers" : "/nav3d/occupied_grid";
}

export type MapPreset = {
  id: string;
  label: string;
  path: string;
  hint: string;
};

/**
 * Curated PCD maps the operator can switch between. `path` is sent on
 * /nav3d/load_pcd_path so the bridge reloads the map. Paths are relative to the
 * repo root, matching the bridge launch working directory.
 */
export const mapPresets: MapPreset[] = [
  {
    id: "building2_9",
    label: "Building 2-9",
    path: "reference/OctoPlanner3D-ROS2/octomap/pcd_files/building2_9.pcd",
    hint: "多层建筑参考地图",
  },
  {
    id: "plaza3_10",
    label: "Plaza 3-10",
    path: "reference/OctoPlanner3D-ROS2/octomap/pcd_files/plaza3_10.pcd",
    hint: "开阔广场参考地图",
  },
  {
    id: "mock_corridor",
    label: "Mock Corridor",
    path: "tools/testdata/mock_corridor_map.pcd",
    hint: "走廊测试地图",
  },
  {
    id: "mock_stairs",
    label: "Mock Stairs",
    path: "tools/testdata/mock_stairs_map.pcd",
    hint: "楼梯测试地图",
  },
];

export function findMapPreset(id: string): MapPreset | undefined {
  return mapPresets.find((preset) => preset.id === id);
}
