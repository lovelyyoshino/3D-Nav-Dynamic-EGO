import { describe, expect, it } from "vitest";
import { getBridgeSupervisorStatusForTest } from "./bridgeSupervisor";

describe("nav3d bridge supervisor", () => {
  it("keeps browser-triggered launch disabled unless explicitly allowed", () => {
    const status = getBridgeSupervisorStatusForTest();

    expect(status.available).toBe(false);
    expect(status.running).toBe(false);
    expect(status.command.join(" ")).toContain("nav3d_bridge.launch.py");
    expect(status.command.join(" ")).toContain(
      "pcd_path:=/media/bigdisk/3D-Octomap-Ego-Nav/reference/OctoPlanner3D-ROS2/octomap/pcd_files/building2_9.pcd",
    );
    expect(status.command.join(" ")).not.toContain("rosbridge:=true");
    expect(status.command.join(" ")).toContain("rosbridge_port:=9090");
  });
});
