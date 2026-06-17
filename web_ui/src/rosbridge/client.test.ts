import { describe, expect, it } from "vitest";
import {
  buildGoalPublishMessage,
  buildPcdPathPublishMessage,
  buildPosePublishMessage,
  createRosbridgeClient,
  parseOccupancyGridCells,
  parseOccupancyVoxelCells,
  parsePosePoint,
  parseStatusText,
  parseTrajectoryPoints,
  type RosbridgeSocket,
} from "./client";

class FakeSocket implements RosbridgeSocket {
  onopen: ((event?: Event) => void) | null = null;
  onclose: ((event?: CloseEvent) => void) | null = null;
  onerror: ((event?: Event) => void) | null = null;
  onmessage: ((event: MessageEvent<string> | { data: string }) => void) | null = null;
  readonly sent: string[] = [];
  closed = false;

  send(data: string): void {
    this.sent.push(data);
  }

  close(): void {
    this.closed = true;
    this.onclose?.();
  }
}

describe("rosbridge client", () => {
  it("builds a PoseStamped publish message and locks z in 2D mode", () => {
    const message = buildGoalPublishMessage({
      goal: { x: 1.25, y: -2.5, z: 3.5 },
      mode: "2d",
      topic: "/nav3d/goal",
      frameId: "map",
    });

    expect(message).toMatchObject({
      op: "publish",
      topic: "/nav3d/goal",
      msg: {
        header: { frame_id: "map" },
        pose: {
          position: { x: 1.25, y: -2.5, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      },
    });
  });

  it("parses nav_msgs/Path-like trajectory messages", () => {
    const points = parseTrajectoryPoints({
      op: "publish",
      topic: "/nav3d/trajectory",
      msg: {
        poses: [
          { pose: { position: { x: 0, y: 0, z: 0 } } },
          { pose: { position: { x: 1.5, y: 2.5, z: 0.75 } } },
        ],
      },
    });

    expect(points).toEqual([
      { x: 0, y: 0, z: 0 },
      { x: 1.5, y: 2.5, z: 0.75 },
    ]);
  });

  it("forwards empty trajectory messages so the UI can clear stale paths", () => {
    const sockets: FakeSocket[] = [];
    const trajectories: unknown[] = [];
    const client = createRosbridgeClient({
      url: "ws://localhost:9090",
      socketFactory: () => {
        const socket = new FakeSocket();
        sockets.push(socket);
        return socket;
      },
      onTrajectory: (points) => trajectories.push(points),
    });

    client.connect();
    const socket = sockets[0];
    socket.onopen?.();
    socket.onmessage?.({
      data: JSON.stringify({
        op: "publish",
        topic: "/nav3d/trajectory",
        msg: { poses: [] },
      }),
    });

    expect(trajectories).toEqual([[]]);
  });

  it("parses std_msgs/String status messages", () => {
    expect(
      parseStatusText({
        op: "publish",
        topic: "/nav3d/status",
        msg: { data: "plan_success poses=18 attempts=1" },
      }),
    ).toBe("plan_success poses=18 attempts=1");

    expect(parseStatusText({ op: "publish", topic: "/nav3d/status", msg: {} })).toBeNull();
  });

  it("parses PoseStamped-like current pose messages", () => {
    expect(
      parsePosePoint({
        op: "publish",
        topic: "/nav3d/current_pose",
        msg: { pose: { position: { x: 0.4, y: -1.2, z: 0.3 } } },
      }),
    ).toEqual({ x: 0.4, y: -1.2, z: 0.3 });
  });

  it("parses nav_msgs/OccupancyGrid cells into occupied, free, and unknown states", () => {
    const cells = parseOccupancyGridCells({
      op: "publish",
      topic: "/nav3d/occupied_grid",
      msg: {
        info: {
          resolution: 0.5,
          width: 3,
          height: 2,
          origin: {
            position: { x: -1, y: 2, z: 0 },
          },
        },
        data: [0, 100, -1, 51, 25, 0],
      },
    });

    expect(cells).toEqual([
      { x: -0.75, y: 2.25, z: 0.25, size: 0.5, layer: "grid", occupancy: "free" },
      { x: -0.25, y: 2.25, z: 0.25, size: 0.5, layer: "grid", occupancy: "occupied" },
      { x: 0.25, y: 2.25, z: 0.25, size: 0.5, layer: "grid", occupancy: "unknown" },
      { x: -0.75, y: 2.75, z: 0.25, size: 0.5, layer: "grid", occupancy: "occupied" },
      { x: -0.25, y: 2.75, z: 0.25, size: 0.5, layer: "grid", occupancy: "free" },
      { x: 0.25, y: 2.75, z: 0.25, size: 0.5, layer: "grid", occupancy: "free" },
    ]);
  });

  it("parses visualization_msgs/MarkerArray cube-list voxels", () => {
    const cells = parseOccupancyVoxelCells({
      op: "publish",
      topic: "/nav3d/planning_occupied_markers",
      msg: {
        markers: [
          { action: 3, ns: "cleanup" },
          {
            action: 0,
            type: 6,
            ns: "nav3d_planning_occupied_voxels",
            scale: { x: 0.5, y: 0.5, z: 0.5 },
            points: [
              { x: 1, y: 2, z: 3 },
              { x: -1, y: -2, z: 0.25 },
            ],
          },
          {
            action: 0,
            type: 6,
            ns: "nav3d_planning_occupied_voxels",
            scale: { x: 0.25, y: 0.25, z: 0.25 },
            points: [{ x: 4, y: 5, z: 6 }],
          },
          {
            action: 0,
            type: 6,
            ns: "nav3d_local_occupied_voxels",
            scale: { x: 0.2, y: 0.2, z: 0.2 },
            points: [{ x: 7, y: 8, z: 9 }],
          },
        ],
      },
    });

    expect(cells).toEqual([
      { x: 1, y: 2, z: 3, size: 0.5, layer: "global" },
      { x: -1, y: -2, z: 0.25, size: 0.5, layer: "global" },
      { x: 4, y: 5, z: 6, size: 0.25, layer: "global" },
      { x: 7, y: 8, z: 9, size: 0.2, layer: "local" },
    ]);
  });

  it("builds a map reload message for passing a PCD path into planning", () => {
    expect(buildPcdPathPublishMessage("  /tmp/maps/building2_9.pcd  ", "/nav3d/load_pcd_path")).toEqual({
      op: "publish",
      topic: "/nav3d/load_pcd_path",
      msg: { data: "/tmp/maps/building2_9.pcd" },
    });
  });

  it("publishes start, goals, and PCD path, then subscribes to trajectory, status, current pose, and map topics", () => {
    const sockets: FakeSocket[] = [];
    const statuses: string[] = [];
    const trajectories: unknown[] = [];
    const textStatuses: string[] = [];
    const poses: unknown[] = [];
    const occupancyCells: unknown[] = [];
    const occupancyVoxels: unknown[] = [];
    const client = createRosbridgeClient({
      url: "ws://localhost:9090",
      socketFactory: () => {
        const socket = new FakeSocket();
        sockets.push(socket);
        return socket;
      },
      onStatus: (status) => statuses.push(status),
      onTextStatus: (status) => textStatuses.push(status),
      onTrajectory: (points) => trajectories.push(points),
      onCurrentPose: (point) => poses.push(point),
      onOccupancyGrid: (cells) => occupancyCells.push(cells),
      onOccupancyVoxels: (cells) => occupancyVoxels.push(cells),
    });

    client.connect();
    const socket = sockets[0];
    socket.onopen?.();
    client.publishStart({ x: -1, y: 0.5, z: 0.25 }, "3d");
    client.publishGoal({ x: 2, y: 1, z: 0.5 }, "3d");
    client.publishPcdPath("/tmp/maps/building2_9.pcd");
    socket.onmessage?.({
      data: JSON.stringify({
        op: "publish",
        topic: "/nav3d/trajectory",
        msg: { poses: [{ pose: { position: { x: 2, y: 1, z: 0.5 } } }] },
      }),
    });
    socket.onmessage?.({
      data: JSON.stringify({
        op: "publish",
        topic: "/nav3d/status",
        msg: { data: "safety_replan_success poses=6 attempts=1" },
      }),
    });
    socket.onmessage?.({
      data: JSON.stringify({
        op: "publish",
        topic: "/nav3d/current_pose",
        msg: { pose: { position: { x: 0.25, y: 0.5, z: 0 } } },
      }),
    });
    socket.onmessage?.({
      data: JSON.stringify({
        op: "publish",
        topic: "/nav3d/occupied_grid",
        msg: {
          info: {
            resolution: 1,
            width: 2,
            height: 1,
            origin: { position: { x: 10, y: -4, z: 0 } },
          },
          data: [100, 0],
        },
      }),
    });
    socket.onmessage?.({
      data: JSON.stringify({
        op: "publish",
        topic: "/nav3d/planning_occupied_markers",
        msg: {
          markers: [
            {
              action: 0,
              type: 6,
                  ns: "nav3d_planning_occupied_voxels",
              scale: { x: 1, y: 1, z: 1 },
              points: [{ x: 3, y: 4, z: 5 }],
            },
          ],
        },
      }),
    });

    expect(statuses).toEqual(["connecting", "connected"]);
    expect(socket.sent.map((entry: string) => JSON.parse(entry))).toEqual([
      { op: "subscribe", topic: "/nav3d/trajectory", type: "nav_msgs/Path" },
      { op: "subscribe", topic: "/nav3d/status", type: "std_msgs/String" },
      { op: "subscribe", topic: "/nav3d/current_pose", type: "geometry_msgs/PoseStamped" },
      { op: "subscribe", topic: "/nav3d/occupied_grid", type: "nav_msgs/OccupancyGrid" },
      { op: "subscribe", topic: "/nav3d/planning_occupied_markers", type: "visualization_msgs/MarkerArray" },
      buildPosePublishMessage({ x: -1, y: 0.5, z: 0.25 }, "/nav3d/start", "map"),
      buildGoalPublishMessage({
        goal: { x: 2, y: 1, z: 0.5 },
        mode: "3d",
        topic: "/nav3d/goal",
        frameId: "map",
      }),
      buildPcdPathPublishMessage("/tmp/maps/building2_9.pcd", "/nav3d/load_pcd_path"),
    ]);
    expect(trajectories).toEqual([[{ x: 2, y: 1, z: 0.5 }]]);
    expect(textStatuses).toEqual(["safety_replan_success poses=6 attempts=1"]);
    expect(poses).toEqual([{ x: 0.25, y: 0.5, z: 0 }]);
    expect(occupancyCells).toEqual([
      [
        { x: 10.5, y: -3.5, z: 0.5, size: 1, layer: "grid", occupancy: "occupied" },
        { x: 11.5, y: -3.5, z: 0.5, size: 1, layer: "grid", occupancy: "free" },
      ],
    ]);
    expect(occupancyVoxels).toEqual([[{ x: 3, y: 4, z: 5, size: 1, layer: "global" }]]);
  });

  it("reassembles rosbridge fragments before parsing large MarkerArray voxel maps", () => {
    const sockets: FakeSocket[] = [];
    const occupancyVoxels: unknown[] = [];
    const statuses: string[] = [];
    const client = createRosbridgeClient({
      url: "ws://localhost:9090",
      socketFactory: () => {
        const socket = new FakeSocket();
        sockets.push(socket);
        return socket;
      },
      onStatus: (status) => statuses.push(status),
      onOccupancyVoxels: (cells) => occupancyVoxels.push(cells),
    });

    client.connect();
    const socket = sockets[0];
    socket.onopen?.();

    const publishMessage = JSON.stringify({
      op: "publish",
      topic: "/nav3d/planning_occupied_markers",
      msg: {
        markers: [
          { action: 3, ns: "cleanup" },
          {
            action: 0,
            type: 6,
            ns: "nav3d_planning_occupied_voxels",
            scale: { x: 0.5, y: 0.5, z: 0.5 },
            points: [
              { x: 1.1, y: 2.1, z: 3.1 },
              { x: 4.1, y: 5.1, z: 6.1 },
            ],
          },
        ],
      },
    });
    const chunks = [
      publishMessage.slice(0, 37),
      publishMessage.slice(37, 91),
      publishMessage.slice(91),
    ];

    chunks.forEach((data, num) => {
      socket.onmessage?.({
        data: JSON.stringify({
          op: "fragment",
          id: "marker-array-1",
          data,
          num,
          total: chunks.length,
        }),
      });
      if (num < chunks.length - 1) {
        expect(occupancyVoxels).toEqual([]);
      }
    });

    expect(statuses).toEqual(["connecting", "connected"]);
    expect(occupancyVoxels).toEqual([
      [
        { x: 1.1, y: 2.1, z: 3.1, size: 0.5, layer: "global" },
        { x: 4.1, y: 5.1, z: 6.1, size: 0.5, layer: "global" },
      ],
    ]);
  });

  it("ignores stale socket callbacks after disconnect", () => {
    const sockets: FakeSocket[] = [];
    const statuses: string[] = [];
    const textStatuses: string[] = [];
    const trajectories: unknown[] = [];
    const client = createRosbridgeClient({
      url: "ws://localhost:9090",
      socketFactory: () => {
        const socket = new FakeSocket();
        sockets.push(socket);
        return socket;
      },
      onStatus: (status) => statuses.push(status),
      onTextStatus: (status) => textStatuses.push(status),
      onTrajectory: (points) => trajectories.push(points),
    });

    client.connect();
    const staleSocket = sockets[0];
    staleSocket.onopen?.();
    client.disconnect();

    staleSocket.onmessage?.({
      data: JSON.stringify({
        op: "publish",
        topic: "/nav3d/status",
        msg: { data: "plan_success poses=17 attempts=1 partial=true" },
      }),
    });
    staleSocket.onmessage?.({
      data: JSON.stringify({
        op: "publish",
        topic: "/nav3d/trajectory",
        msg: { poses: [{ pose: { position: { x: 0.25, y: 0.25, z: 0.25 } } }] },
      }),
    });

    expect(statuses).toEqual(["connecting", "connected", "closed"]);
    expect(textStatuses).toEqual([]);
    expect(trajectories).toEqual([]);
  });
});
