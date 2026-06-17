import { expect, test } from "@playwright/test";

type CanvasStats = {
  sampled: number;
  opaque: number;
  bright: number;
  green: number;
  blue: number;
  cyan: number;
  buckets: number;
  meanLuminance: number;
};

type RosbridgePublish = {
  topic?: string;
  msg?: {
    pose?: {
      position?: {
        x?: number;
        y?: number;
        z?: number;
      };
    };
  };
};

type RosbridgePoint = { x: number; y: number; z: number };

type LiveExternalSimObservation = {
  cmdVelMessages: number;
  nonzeroCmdVelMessages: number;
  currentPoseMessages: number;
  finalGoalError: number | null;
  statuses: string[];
};

type LiveLocalReplanObservation = {
  initialTrajectoryPoses: number;
  replanTrajectoryPoses: number;
  initialTrajectory: RosbridgePoint[];
  replanTrajectory: RosbridgePoint[];
  maxTrajectoryDelta: number;
  currentPoint: RosbridgePoint;
  obstaclePoint: RosbridgePoint;
  statuses: string[];
};

type ConfiguredLiveClickOptions = {
  start: RosbridgePoint;
  goal: RosbridgePoint;
  minTrajectoryPoses: number;
  expectedPartial: string | null;
  expectedBounds: string[];
};

type SceneProbe = {
  projectVoxel: (point: { x: number; y: number; z: number }) => { x: number; y: number } | null;
  projectPoint: (point: { x: number; y: number; z: number }) => { x: number; y: number } | null;
  cameraTarget?: () => RosbridgePoint;
  routePoints?: () => RosbridgePoint[];
};

function liveReferenceSoakCycles(): number {
  const parsed = Number(process.env.NAV3D_E2E_LIVE_REFERENCE_ROSBRIDGE_SOAK_CYCLES ?? "12");
  if (!Number.isFinite(parsed)) {
    return 12;
  }
  return Math.max(1, Math.floor(parsed));
}

function parsePointEnv(name: string, fallback: RosbridgePoint): RosbridgePoint {
  const raw = process.env[name];
  if (!raw) {
    return fallback;
  }
  const parts = raw.split(",").map((part) => Number(part.trim()));
  if (parts.length !== 3 || parts.some((part) => !Number.isFinite(part))) {
    throw new Error(`${name} must be formatted as x,y,z`);
  }
  return { x: parts[0], y: parts[1], z: parts[2] };
}

function configuredLiveClickOptions(): ConfiguredLiveClickOptions {
  const minTrajectoryPoses = Number(process.env.NAV3D_E2E_CONFIG_MIN_TRAJECTORY_POSES ?? "2");
  const expectedBounds = (process.env.NAV3D_E2E_CONFIG_EXPECT_BOUNDS ?? "")
    .split("|")
    .map((part) => part.trim())
    .filter((part) => part.length > 0);
  return {
    start: parsePointEnv("NAV3D_E2E_CONFIG_START", { x: -2, y: 0, z: 0 }),
    goal: parsePointEnv("NAV3D_E2E_CONFIG_GOAL", { x: 2, y: 0, z: 0 }),
    minTrajectoryPoses: Number.isFinite(minTrajectoryPoses) ? Math.max(2, Math.floor(minTrajectoryPoses)) : 2,
    expectedPartial: process.env.NAV3D_E2E_CONFIG_EXPECT_PARTIAL ?? null,
    expectedBounds,
  };
}

async function getCanvasStats(page: import("@playwright/test").Page): Promise<CanvasStats | null> {
  await page.locator(".scene-canvas").waitFor({ state: "visible" });
  await page.waitForTimeout(300);
  return await page.locator(".scene-canvas").evaluate((canvasElement) => {
    const canvas = canvasElement as HTMLCanvasElement;
    const sampleCanvas = document.createElement("canvas");
    sampleCanvas.width = Math.max(1, Math.floor(canvas.width / 6));
    sampleCanvas.height = Math.max(1, Math.floor(canvas.height / 6));
    const context = sampleCanvas.getContext("2d", { willReadFrequently: true });
    if (!context) {
      return null;
    }
    context.drawImage(canvas, 0, 0, sampleCanvas.width, sampleCanvas.height);
    const pixels = context.getImageData(0, 0, sampleCanvas.width, sampleCanvas.height).data;
    let opaque = 0;
    let bright = 0;
    let greenSamples = 0;
    let blueSamples = 0;
    let cyan = 0;
    let luminance = 0;
    const buckets = new Set<string>();

    for (let index = 0; index < pixels.length; index += 4) {
      const red = pixels[index];
      const green = pixels[index + 1];
      const blue = pixels[index + 2];
      const alpha = pixels[index + 3];
      luminance += 0.2126 * red + 0.7152 * green + 0.0722 * blue;
      if (alpha > 32) {
        opaque += 1;
      }
      if (red + green + blue > 110) {
        bright += 1;
      }
      if (green > 110 && red < 80) {
        greenSamples += 1;
      }
      if (blue > 150 && green > 100 && red < 180) {
        blueSamples += 1;
      }
      if (green > 100 && blue > 100 && red < 90) {
        cyan += 1;
      }
      buckets.add(`${red >> 5}-${green >> 5}-${blue >> 5}-${alpha >> 6}`);
    }

    return {
      sampled: pixels.length / 4,
      opaque,
      bright,
      green: greenSamples,
      blue: blueSamples,
      cyan,
      buckets: buckets.size,
      meanLuminance: luminance / (pixels.length / 4),
    };
  });
}

async function expectCanvasHasNonWhitePixels(page: import("@playwright/test").Page) {
  const stats = await getCanvasStats(page);

  expect(stats).not.toBeNull();
  expect(stats?.opaque).toBeGreaterThan(500);
  expect(stats?.bright).toBeGreaterThan(120);
  expect(stats?.green).toBeGreaterThan(8);
  expect(stats?.cyan).toBeGreaterThan(1);
  expect(stats?.buckets).toBeGreaterThan(10);
  return stats as CanvasStats;
}

async function expectCanvasHasRenderedLiveMap(page: import("@playwright/test").Page) {
  const stats = await getCanvasStats(page);

  expect(stats).not.toBeNull();
  expect(stats?.opaque).toBeGreaterThan(500);
  expect(stats?.buckets).toBeGreaterThan(6);
  expect(stats?.meanLuminance).toBeGreaterThan(2);
  return stats as CanvasStats;
}

async function expectCanvasHasCoolOctomap(page: import("@playwright/test").Page) {
  const stats = await expectCanvasHasRenderedLiveMap(page);
  expect(stats.blue).toBeGreaterThan(8);
  return stats;
}

async function installRosbridgeMock(page: import("@playwright/test").Page) {
  await page.addInitScript(() => {
    class FakeRosbridgeSocket {
      static sockets: FakeRosbridgeSocket[] = [];

      url: string;
      sent: unknown[] = [];
      lastStart = { x: 0, y: 0, z: 0 };
      readyState = 0;
      onopen: ((event?: Event) => void) | null = null;
      onclose: ((event?: Event) => void) | null = null;
      onerror: ((event?: Event) => void) | null = null;
      onmessage: ((event: { data: string }) => void) | null = null;

      constructor(url: string) {
        this.url = url;
        FakeRosbridgeSocket.sockets.push(this);
        window.setTimeout(() => {
          this.readyState = 1;
          this.onopen?.();
          this.publish("/nav3d/planning_occupied_markers", {
            markers: [
              {
                action: 0,
                type: 6,
                ns: "nav3d_planning_occupied_voxels",
                scale: { x: 0.5, y: 0.5, z: 0.5 },
                points: [
                  { x: -1, y: 0.5, z: 0.25 },
                  { x: 4, y: 1.5, z: 0.25 },
                ],
              },
            ],
          });
        }, 20);
      }

      send(data: string) {
        const message = JSON.parse(data);
        this.sent.push(message);
        if (message.op === "publish" && message.topic === "/nav3d/start") {
          this.lastStart = message.msg.pose.position;
        }
        if (message.op === "publish" && message.topic === "/nav3d/goal") {
          const goal = message.msg.pose.position;
          const start = this.lastStart;
          const midpoint = {
            x: (start.x + goal.x) / 2,
            y: (start.y + goal.y) / 2,
            z: (start.z + goal.z) / 2,
          };
          window.setTimeout(() => {
            this.publish("/nav3d/status", { data: "plan_success poses=3 attempts=1" });
            this.publish("/nav3d/planning_occupied_markers", {
              markers: [
                {
                  action: 0,
                  type: 6,
                  ns: "nav3d_planning_occupied_voxels",
                  scale: { x: 0.5, y: 0.5, z: 0.5 },
                  points: [
                    { x: -1, y: 0.5, z: 0.25 },
                    { x: 4, y: 1.5, z: 0.25 },
                  ],
                },
              ],
            });
            this.publish("/nav3d/trajectory", {
              poses: [
                { pose: { position: start } },
                { pose: { position: midpoint } },
                { pose: { position: goal } },
              ],
            });
          }, 20);
        }
      }

      close() {
        this.readyState = 3;
        this.onclose?.();
      }

      publish(topic: string, msg: unknown) {
        this.onmessage?.({ data: JSON.stringify({ op: "publish", topic, msg }) });
      }
    }

    Object.defineProperty(window, "WebSocket", {
      writable: true,
      value: FakeRosbridgeSocket,
    });
    Object.defineProperty(window, "__nav3dRosbridgeMessages", {
      writable: true,
      value: () => FakeRosbridgeSocket.sockets.flatMap((socket) => socket.sent),
    });
  });
}

async function installClosedRosbridgeMock(page: import("@playwright/test").Page) {
  await page.addInitScript(() => {
    class ClosedRosbridgeSocket {
      readyState = 0;
      onopen: ((event?: Event) => void) | null = null;
      onclose: ((event?: Event) => void) | null = null;
      onerror: ((event?: Event) => void) | null = null;
      onmessage: ((event: { data: string }) => void) | null = null;

      constructor() {
        window.setTimeout(() => {
          this.readyState = 3;
          this.onclose?.();
        }, 20);
      }

      send() {}

      close() {
        this.readyState = 3;
        this.onclose?.();
      }
    }

    Object.defineProperty(window, "WebSocket", {
      writable: true,
      value: ClosedRosbridgeSocket,
    });
  });
}

async function installRosbridgeGridOnlyMock(page: import("@playwright/test").Page) {
  await page.addInitScript(() => {
    class FakeRosbridgeSocket {
      static sockets: FakeRosbridgeSocket[] = [];

      url: string;
      sent: unknown[] = [];
      readyState = 0;
      onopen: ((event?: Event) => void) | null = null;
      onclose: ((event?: Event) => void) | null = null;
      onerror: ((event?: Event) => void) | null = null;
      onmessage: ((event: { data: string }) => void) | null = null;

      constructor(url: string) {
        this.url = url;
        FakeRosbridgeSocket.sockets.push(this);
        window.setTimeout(() => {
          this.readyState = 1;
          this.onopen?.();
          this.publish("/nav3d/occupied_grid", {
            info: {
              resolution: 0.5,
              width: 2,
              height: 2,
              origin: {
                position: { x: -0.5, y: -0.5, z: 0 },
              },
            },
            data: [0, 100, -1, 0],
          });
        }, 20);
      }

      send(data: string) {
        this.sent.push(JSON.parse(data));
      }

      close() {
        this.readyState = 3;
        this.onclose?.();
      }

      publish(topic: string, msg: unknown) {
        this.onmessage?.({ data: JSON.stringify({ op: "publish", topic, msg }) });
      }
    }

    Object.defineProperty(window, "WebSocket", {
      writable: true,
      value: FakeRosbridgeSocket,
    });
  });
}

async function installRosbridgePlanFailureMock(page: import("@playwright/test").Page) {
  await page.addInitScript(() => {
    class FailingRosbridgeSocket {
      static sockets: FailingRosbridgeSocket[] = [];

      url: string;
      sent: unknown[] = [];
      readyState = 0;
      onopen: ((event?: Event) => void) | null = null;
      onclose: ((event?: Event) => void) | null = null;
      onerror: ((event?: Event) => void) | null = null;
      onmessage: ((event: { data: string }) => void) | null = null;

      constructor(url: string) {
        this.url = url;
        FailingRosbridgeSocket.sockets.push(this);
        window.setTimeout(() => {
          this.readyState = 1;
          this.onopen?.();
          this.publish("/nav3d/planning_occupied_markers", {
            markers: [
              {
                action: 0,
                type: 6,
                ns: "nav3d_planning_occupied_voxels",
                scale: { x: 0.5, y: 0.5, z: 0.5 },
                points: [
                  { x: -1, y: 0.5, z: 0.25 },
                  { x: 4, y: 1.5, z: 0.25 },
                ],
              },
            ],
          });
        }, 20);
      }

      send(data: string) {
        const message = JSON.parse(data);
        this.sent.push(message);
        if (message.op === "publish" && message.topic === "/nav3d/goal") {
          window.setTimeout(() => {
            this.publish("/nav3d/status", { data: "plan_failed reason=goal_outside_active_map attempts=1" });
          }, 20);
        }
      }

      close() {
        this.readyState = 3;
        this.onclose?.();
      }

      publish(topic: string, msg: unknown) {
        this.onmessage?.({ data: JSON.stringify({ op: "publish", topic, msg }) });
      }
    }

    Object.defineProperty(window, "WebSocket", {
      writable: true,
      value: FailingRosbridgeSocket,
    });
    Object.defineProperty(window, "__nav3dRosbridgeMessages", {
      writable: true,
      value: () => FailingRosbridgeSocket.sockets.flatMap((socket) => socket.sent),
    });
  });
}

async function installRosbridgeHighVoxelMock(page: import("@playwright/test").Page) {
  await page.addInitScript(() => {
    class HighVoxelRosbridgeSocket {
      static sockets: HighVoxelRosbridgeSocket[] = [];

      url: string;
      sent: unknown[] = [];
      readyState = 0;
      onopen: ((event?: Event) => void) | null = null;
      onclose: ((event?: Event) => void) | null = null;
      onerror: ((event?: Event) => void) | null = null;
      onmessage: ((event: { data: string }) => void) | null = null;

      constructor(url: string) {
        this.url = url;
        HighVoxelRosbridgeSocket.sockets.push(this);
        window.setTimeout(() => {
          this.readyState = 1;
          this.onopen?.();
          this.publish("/nav3d/planning_occupied_markers", {
            markers: [
              {
                action: 0,
                type: 6,
                ns: "nav3d_planning_occupied_voxels",
                scale: { x: 0.5, y: 0.5, z: 0.5 },
                points: [
                  { x: -1, y: 0, z: 0.25 },
                  { x: 0, y: 0, z: 6.25 },
                ],
              },
            ],
          });
        }, 20);
      }

      send(data: string) {
        this.sent.push(JSON.parse(data));
      }

      close() {
        this.readyState = 3;
        this.onclose?.();
      }

      publish(topic: string, msg: unknown) {
        this.onmessage?.({ data: JSON.stringify({ op: "publish", topic, msg }) });
      }
    }

    Object.defineProperty(window, "WebSocket", {
      writable: true,
      value: HighVoxelRosbridgeSocket,
    });
    Object.defineProperty(window, "__nav3dRosbridgeMessages", {
      writable: true,
      value: () => HighVoxelRosbridgeSocket.sockets.flatMap((socket) => socket.sent),
    });
  });
}

async function projectVoxel(
  page: import("@playwright/test").Page,
  point: { x: number; y: number; z: number },
) {
  const projected = await page.waitForFunction(
    (voxelPoint) => {
      const probe = (window as unknown as { __nav3dSceneProbe?: SceneProbe }).__nav3dSceneProbe;
      return probe?.projectVoxel(voxelPoint) ?? null;
    },
    point,
    { timeout: 5000 },
  );
  const screenPoint = await projected.jsonValue();
  expect(screenPoint).not.toBeNull();
  return screenPoint;
}

async function clickProjectedVoxel(
  page: import("@playwright/test").Page,
  point: { x: number; y: number; z: number },
) {
  const screenPoint = await projectVoxel(page, point);
  if (!screenPoint) {
    return;
  }
  await page.mouse.click(screenPoint.x, screenPoint.y);
}

async function projectScenePoint(
  page: import("@playwright/test").Page,
  point: { x: number; y: number; z: number },
) {
  const projected = await page.waitForFunction(
    (scenePoint) => {
      return window.__nav3dSceneProbe?.projectPoint?.(scenePoint) ?? null;
    },
    point,
    { timeout: 5000 },
  );
  const screenPoint = await projected.jsonValue();
  expect(screenPoint).not.toBeNull();
  return screenPoint;
}

async function clickScenePoint(
  page: import("@playwright/test").Page,
  point: { x: number; y: number; z: number },
) {
  const screenPoint = await projectScenePoint(page, point);
  if (!screenPoint) {
    return;
  }
  await page.mouse.click(screenPoint.x, screenPoint.y);
}

function maxPointDelta(left: RosbridgePoint[], right: RosbridgePoint[]): number {
  const count = Math.min(left.length, right.length);
  let maxDelta = 0;
  for (let index = 0; index < count; index += 1) {
    const dx = left[index].x - right[index].x;
    const dy = left[index].y - right[index].y;
    const dz = left[index].z - right[index].z;
    maxDelta = Math.max(maxDelta, Math.sqrt(dx * dx + dy * dy + dz * dz));
  }
  return maxDelta;
}

async function getSceneRoutePoints(page: import("@playwright/test").Page): Promise<RosbridgePoint[]> {
  const points = await page.waitForFunction(
    () => window.__nav3dSceneProbe?.routePoints?.() ?? null,
    undefined,
    { timeout: 5000 },
  );
  return (await points.jsonValue()) as RosbridgePoint[];
}

async function getSceneCameraTarget(page: import("@playwright/test").Page): Promise<RosbridgePoint> {
  const target = await page.waitForFunction(
    () => window.__nav3dSceneProbe?.cameraTarget?.() ?? null,
    undefined,
    { timeout: 5000 },
  );
  return (await target.jsonValue()) as RosbridgePoint;
}

async function publishLiveLocalPointcloud(
  page: import("@playwright/test").Page,
  points: RosbridgePoint[],
  currentPose: RosbridgePoint = { x: 100, y: 0, z: 0 },
) {
  await page.evaluate(async ({ cloudPoints, posePoint }) => {
    const byteLength = cloudPoints.length * 12;
    const buffer = new ArrayBuffer(byteLength);
    const view = new DataView(buffer);
    cloudPoints.forEach((point, index) => {
      const offset = index * 12;
      view.setFloat32(offset, point.x, true);
      view.setFloat32(offset + 4, point.y, true);
      view.setFloat32(offset + 8, point.z, true);
    });

    let binary = "";
    const bytes = new Uint8Array(buffer);
    bytes.forEach((byte) => {
      binary += String.fromCharCode(byte);
    });

    const socket = new WebSocket("ws://localhost:9090");
    await new Promise<void>((resolve, reject) => {
      socket.onopen = () => resolve();
      socket.onerror = () => reject(new Error("local pointcloud rosbridge socket failed to open"));
    });

    socket.send(
      JSON.stringify({
        op: "publish",
        topic: "/nav3d/current_pose",
        type: "geometry_msgs/PoseStamped",
        msg: {
          header: { frame_id: "map" },
          pose: {
            position: posePoint,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        },
      }),
    );
    socket.send(
      JSON.stringify({
        op: "publish",
        topic: "/nav3d/local_pointcloud",
        type: "sensor_msgs/PointCloud2",
        msg: {
          header: { frame_id: "map" },
          height: 1,
          width: cloudPoints.length,
          fields: [
            { name: "x", offset: 0, datatype: 7, count: 1 },
            { name: "y", offset: 4, datatype: 7, count: 1 },
            { name: "z", offset: 8, datatype: 7, count: 1 },
          ],
          is_bigendian: false,
          point_step: 12,
          row_step: byteLength,
          data: btoa(binary),
          is_dense: true,
        },
      }),
    );
    await new Promise((resolve) => window.setTimeout(resolve, 200));
    socket.close();
  }, { cloudPoints: points, posePoint: currentPose });
}

async function watchLiveLocalReplan(page: import("@playwright/test").Page): Promise<LiveLocalReplanObservation> {
  return await page.evaluate(async () => {
    type RosbridgePayload = {
      topic?: string;
      msg?: {
        data?: string;
        poses?: Array<{ pose?: { position?: { x?: number; y?: number; z?: number } } }>;
      };
    };

    type Point = { x: number; y: number; z: number };

    const currentIndex = 0;
    const obstacleIndex = 24;
    const minTrajectoryPoses = 50;
    const minTrajectoryDelta = 0.02;
    const statuses: string[] = [];
    let initialTrajectory: Point[] | null = null;
    let replanTrajectory: Point[] | null = null;
    let currentPoint: Point | null = null;
    let obstaclePoint: Point | null = null;
    let obstaclePublished = false;

    const socket = new WebSocket("ws://localhost:9090");
    await new Promise<void>((resolve, reject) => {
      const timeout = window.setTimeout(() => reject(new Error("local replan observer socket open timeout")), 5000);
      socket.onopen = () => {
        window.clearTimeout(timeout);
        resolve();
      };
      socket.onerror = () => {
        window.clearTimeout(timeout);
        reject(new Error("local replan observer socket failed to open"));
      };
    });

    const trajectoryPoints = (payload: RosbridgePayload): Point[] => {
      return (
        payload.msg?.poses
          ?.map((poseStamped) => poseStamped.pose?.position)
          .filter((position): position is { x: number; y: number; z: number } => {
            return (
              typeof position?.x === "number" &&
              typeof position.y === "number" &&
              typeof position.z === "number"
            );
          })
          .map((position) => ({ x: position.x, y: position.y, z: position.z })) ?? []
      );
    };

    const trajectoryDelta = (left: Point[], right: Point[]): number => {
      const count = Math.min(left.length, right.length);
      let maxDelta = 0;
      for (let index = 0; index < count; index += 1) {
        const dx = left[index].x - right[index].x;
        const dy = left[index].y - right[index].y;
        const dz = left[index].z - right[index].z;
        maxDelta = Math.max(maxDelta, Math.sqrt(dx * dx + dy * dy + dz * dz));
      }
      return maxDelta;
    };

    const pointCloudPayload = (points: Point[]) => {
      const byteLength = points.length * 12;
      const buffer = new ArrayBuffer(byteLength);
      const view = new DataView(buffer);
      points.forEach((point, index) => {
        const offset = index * 12;
        view.setFloat32(offset, point.x, true);
        view.setFloat32(offset + 4, point.y, true);
        view.setFloat32(offset + 8, point.z, true);
      });

      let binary = "";
      const bytes = new Uint8Array(buffer);
      bytes.forEach((byte) => {
        binary += String.fromCharCode(byte);
      });
      return {
        header: { frame_id: "map" },
        height: 1,
        width: points.length,
        fields: [
          { name: "x", offset: 0, datatype: 7, count: 1 },
          { name: "y", offset: 4, datatype: 7, count: 1 },
          { name: "z", offset: 8, datatype: 7, count: 1 },
        ],
        is_bigendian: false,
        point_step: 12,
        row_step: byteLength,
        data: btoa(binary),
        is_dense: true,
      };
    };

    const publishPose = (point: Point) => {
      socket.send(
        JSON.stringify({
          op: "publish",
          topic: "/nav3d/current_pose",
          type: "geometry_msgs/PoseStamped",
          msg: {
            header: { frame_id: "map" },
            pose: {
              position: point,
              orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
          },
        }),
      );
    };

    const publishLocalCloud = (points: Point[]) => {
      socket.send(
        JSON.stringify({
          op: "publish",
          topic: "/nav3d/local_pointcloud",
          type: "sensor_msgs/PointCloud2",
          msg: pointCloudPayload(points),
        }),
      );
    };

    for (const message of [
      { op: "subscribe", topic: "/nav3d/status", type: "std_msgs/String" },
      { op: "subscribe", topic: "/nav3d/trajectory", type: "nav_msgs/Path" },
    ]) {
      socket.send(JSON.stringify(message));
    }

    try {
      return await new Promise<LiveLocalReplanObservation>((resolve, reject) => {
        const timeout = window.setTimeout(
          () =>
            reject(
              new Error(
                `local replan observer timeout statuses=${statuses.join("|")} initial=${initialTrajectory?.length ?? 0} replan=${replanTrajectory?.length ?? 0}`,
              ),
            ),
          20_000,
        );

        const maybeResolve = () => {
          if (!initialTrajectory || !replanTrajectory || !currentPoint || !obstaclePoint) {
            return;
          }
          if (
            !statuses.some((status) => status.includes("safety_replan_needed")) ||
            !statuses.some((status) => status.includes("safety_replan_success"))
          ) {
            return;
          }
          const maxTrajectoryDelta = trajectoryDelta(initialTrajectory, replanTrajectory);
          if (maxTrajectoryDelta < minTrajectoryDelta) {
            return;
          }
          window.clearTimeout(timeout);
          resolve({
            initialTrajectoryPoses: initialTrajectory.length,
            replanTrajectoryPoses: replanTrajectory.length,
            initialTrajectory,
            replanTrajectory,
            maxTrajectoryDelta,
            currentPoint,
            obstaclePoint,
            statuses,
          });
        };

        socket.onmessage = (event) => {
          const payload = JSON.parse(event.data) as RosbridgePayload;
          if (payload.topic === "/nav3d/status" && typeof payload.msg?.data === "string") {
            statuses.push(payload.msg.data);
            maybeResolve();
            return;
          }
          if (payload.topic !== "/nav3d/trajectory") {
            return;
          }

          const points = trajectoryPoints(payload);
          if (points.length < minTrajectoryPoses) {
            return;
          }
          if (!initialTrajectory) {
            initialTrajectory = points;
            currentPoint = points[Math.min(currentIndex, points.length - 1)];
            obstaclePoint = points[Math.min(obstacleIndex, points.length - 1)];
            if (obstacleIndex <= currentIndex || !currentPoint || !obstaclePoint) {
              window.clearTimeout(timeout);
              reject(new Error("local replan observer could not choose current/obstacle points"));
              return;
            }
            window.setTimeout(() => {
              if (!currentPoint || !obstaclePoint) {
                return;
              }
              publishPose(currentPoint);
              publishPose(currentPoint);
              publishPose(currentPoint);
              publishLocalCloud([obstaclePoint]);
              obstaclePublished = true;
            }, 150);
            return;
          }

          if (obstaclePublished) {
            replanTrajectory = points;
            maybeResolve();
          }
        };
      });
    } finally {
      socket.close();
    }
  });
}

async function watchLiveExternalCmdVelSim(
  page: import("@playwright/test").Page,
  goal: RosbridgePoint,
): Promise<LiveExternalSimObservation> {
  return await page.evaluate(async (expectedGoal) => {
    type RosbridgePayload = {
      topic?: string;
      msg?: {
        data?: string;
        linear?: { x?: number; y?: number; z?: number };
        angular?: { z?: number };
        pose?: { position?: { x?: number; y?: number; z?: number } };
      };
    };

    const socket = new WebSocket("ws://localhost:9090");
    const statuses: string[] = [];
    let cmdVelMessages = 0;
    let nonzeroCmdVelMessages = 0;
    let currentPoseMessages = 0;
    let finalGoalError: number | null = null;

    await new Promise<void>((resolve, reject) => {
      const timeout = window.setTimeout(() => reject(new Error("external simulator observer socket open timeout")), 5000);
      socket.onopen = () => {
        window.clearTimeout(timeout);
        resolve();
      };
      socket.onerror = () => {
        window.clearTimeout(timeout);
        reject(new Error("external simulator observer socket failed to open"));
      };
    });

    for (const message of [
      { op: "subscribe", topic: "/nav3d/status", type: "std_msgs/String" },
      { op: "subscribe", topic: "/nav3d/current_pose", type: "geometry_msgs/PoseStamped" },
      { op: "subscribe", topic: "/cmd_vel", type: "geometry_msgs/Twist" },
    ]) {
      socket.send(JSON.stringify(message));
    }

    try {
      await new Promise<void>((resolve, reject) => {
        const timeout = window.setTimeout(
          () =>
            reject(
              new Error(
                `external simulator observer timeout statuses=${statuses.join("|")} cmd_vel=${cmdVelMessages} current_pose=${currentPoseMessages} final_goal_error=${finalGoalError ?? "none"}`,
              ),
            ),
          90_000,
        );

        socket.onmessage = (event) => {
          const payload = JSON.parse(event.data) as RosbridgePayload;
          const msg = payload.msg;
          if (!msg) {
            return;
          }
          if (payload.topic === "/nav3d/status" && typeof msg.data === "string") {
            statuses.push(msg.data);
          }
          if (payload.topic === "/cmd_vel") {
            cmdVelMessages += 1;
            if (
              Math.abs(msg.linear?.x ?? 0) > 1e-6 ||
              Math.abs(msg.linear?.y ?? 0) > 1e-6 ||
              Math.abs(msg.linear?.z ?? 0) > 1e-6 ||
              Math.abs(msg.angular?.z ?? 0) > 1e-6
            ) {
              nonzeroCmdVelMessages += 1;
            }
          }
          if (payload.topic === "/nav3d/current_pose") {
            currentPoseMessages += 1;
            const position = msg.pose?.position;
            if (position) {
              const dx = (position.x ?? 0) - expectedGoal.x;
              const dy = (position.y ?? 0) - expectedGoal.y;
              const dz = (position.z ?? 0) - expectedGoal.z;
              finalGoalError = Math.sqrt(dx * dx + dy * dy + dz * dz);
            }
          }
          if (
            statuses.some((status) => status.includes("tracking_goal_reached")) &&
            cmdVelMessages >= 20 &&
            nonzeroCmdVelMessages >= 20 &&
            currentPoseMessages >= 20 &&
            finalGoalError !== null &&
            finalGoalError <= 0.35
          ) {
            window.clearTimeout(timeout);
            resolve();
          }
        };
      });
    } finally {
      socket.close();
    }

    return {
      cmdVelMessages,
      nonzeroCmdVelMessages,
      currentPoseMessages,
      finalGoalError,
      statuses,
    };
  }, goal);
}

async function expectRosbridgeAutoConnected(page: import("@playwright/test").Page) {
  await expect(page.getByLabel("导航状态").getByText("已连接")).toBeVisible({ timeout: 8000 });
}

async function fillCoordinateForm(
  page: import("@playwright/test").Page,
  point: { x: number; y: number; z: number },
) {
  await page.getByRole("spinbutton", { name: "X" }).fill(point.x.toFixed(2));
  await page.getByRole("spinbutton", { name: "Y" }).fill(point.y.toFixed(2));
  await page.getByRole("spinbutton", { name: "Z" }).fill(point.z.toFixed(2));
}

/**
 * Press 启动 and wait for the bridge mock to deliver plan_success +
 * /nav3d/trajectory. After the placeholder-kind redesign, 添加目标 only
 * authors a local placeholder leg — the bridge is silent until the operator
 * presses 启动 (which is when publishPlanRequest fires `/nav3d/goal`).
 */
async function clickPlayAndWaitForBridgeTrajectory(page: import("@playwright/test").Page) {
  await page.getByRole("button", { name: "启动", exact: true }).click();
  await expect(page.getByText("plan_success poses=3 attempts=1")).toBeVisible({ timeout: 8000 });
}

test("starts in ROS-only mode without drawing an offline reference route", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only interaction coverage");
  await installClosedRosbridgeMock(page);
  await page.goto("/");

  await expect(page.getByRole("heading", { name: "机器人控制" })).toBeVisible();
  await expect(page.getByRole("heading", { name: "三维 OctoMap 场景" })).toBeVisible();
  await expect(page.getByRole("heading", { name: "地图层状态" })).toBeVisible();
  await expect(page.getByText("3D 体素层")).toBeVisible();
  await expect(page.getByText("2D 栅格层")).toBeVisible();
  await expect(page.getByText("等待 ROS 3D OctoMap").first()).toBeVisible();
  await expect(page.getByText("0 体素")).toBeVisible();
  await expect(page.getByText("0 规划点")).toBeVisible();
  await expect(page.getByText(/先连接 ROSBridge|等待 bridge 规划/).first()).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeDisabled();
});

test("uses 3D voxels only in 3D mode and 2D grid only in 2D mode", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only map source coverage");
  await installRosbridgeGridOnlyMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("等待 ROS 3D OctoMap").first()).toBeVisible();
  await expect(
    page.getByText("当前为 3D 模式，已收到另一种地图 topic，正在等待 /nav3d/planning_occupied_markers。"),
  ).toBeVisible();
  await expect(page.getByText("0 体素")).toBeVisible();
  await expect(page.getByText("2D 栅格层")).toBeVisible();

  await page.getByRole("button", { name: "2D" }).click();

  await expect(page.getByText("ROS 2D 栅格").first()).toBeVisible();
  await expect(page.getByRole("heading", { name: "二维栅格地图" })).toBeVisible();
  await expect(page.getByText("4 栅格")).toBeVisible();
  await expectCanvasHasRenderedLiveMap(page);
});

test("does not record an explicit start and goal before ROSBridge is connected", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only start and goal coverage");
  await installClosedRosbridgeMock(page);
  await page.goto("/");

  await page.getByRole("button", { name: "起点" }).click();
  await page.getByRole("spinbutton", { name: "X" }).fill("-1.00");
  await page.getByRole("spinbutton", { name: "Y" }).fill("0.50");
  await page.getByRole("spinbutton", { name: "Z" }).fill("0.25");
  await page.getByRole("button", { name: "设置起点" }).click();

  await expect(page.getByText("起点：未设")).toBeVisible();
  await expect(page.locator(".pick-status")).toHaveText("请先连接 ROSBridge");

  await page.getByRole("button", { name: "目标", exact: true }).click();
  await page.getByRole("spinbutton", { name: "X" }).fill("4.00");
  await page.getByRole("spinbutton", { name: "Y" }).fill("1.50");
  await page.getByRole("spinbutton", { name: "Z" }).fill("0.00");
  await page.getByRole("button", { name: "添加目标" }).click();

  await expect(page.getByText("目标：0")).toBeVisible();
  await expect(page.getByText("0 规划点")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeDisabled();
});

test("does not queue ROSBridge goals before the bridge is connected", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only connection guard coverage");
  await installClosedRosbridgeMock(page);
  await page.goto("/");

  await expect(page.getByRole("heading", { name: "ROSBridge 连接" })).toBeVisible();
  await page.getByRole("button", { name: "目标", exact: true }).click();
  await page.getByRole("button", { name: "添加目标" }).click();

  await expect(page.getByText("目标：0")).toBeVisible();
  await expect(page.locator(".pick-status")).toHaveText("请先连接 ROSBridge");
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeDisabled();
});

test("auto-connects to an already running ROSBridge and renders the bridge trajectory", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only rosbridge auto-connect coverage");
  await installRosbridgeMock(page);
  await page.goto("/");

  await expect(page.getByLabel("导航状态").getByText("已连接")).toBeVisible();
  await page.getByRole("button", { name: "起点" }).click();
  await page.getByRole("spinbutton", { name: "X" }).fill("-1.00");
  await page.getByRole("spinbutton", { name: "Y" }).fill("0.50");
  await page.getByRole("spinbutton", { name: "Z" }).fill("0.25");
  await page.getByRole("button", { name: "设置起点" }).click();

  await page.getByRole("spinbutton", { name: "X" }).fill("4.00");
  await page.getByRole("spinbutton", { name: "Y" }).fill("1.50");
  await page.getByRole("spinbutton", { name: "Z" }).fill("0.25");
  await page.getByRole("button", { name: "添加目标" }).click();

  // No preview line authored on add — but 启动 is enabled now since the
  // operator has both a start and a queued goal; pressing it dispatches
  // leg 0 to bridge.
  await expect(page.getByText("0 规划点")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();

  // Press 启动 to dispatch leg 0 to the bridge; mock returns 3-pose curve.
  await clickPlayAndWaitForBridgeTrajectory(page);
  await expect(page.getByText("3 规划点")).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
});

test("publishes explicit ROSBridge start and goal, then renders the bridge trajectory", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only rosbridge flow coverage");
  await installRosbridgeMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await page.getByRole("spinbutton", { name: "X" }).fill("1.50");
  await page.getByRole("spinbutton", { name: "Y" }).fill("0.00");
  await page.getByRole("spinbutton", { name: "Z" }).fill("0.00");
  await page.getByRole("button", { name: "添加目标" }).click();
  await expect(page.getByText("起点：已设")).toBeVisible();

  await page.getByRole("button", { name: "清空当前规划" }).click();
  await page.getByRole("button", { name: "起点" }).click();
  await page.getByRole("spinbutton", { name: "X" }).fill("-1.00");
  await page.getByRole("spinbutton", { name: "Y" }).fill("0.50");
  await page.getByRole("spinbutton", { name: "Z" }).fill("0.25");
  await page.getByRole("button", { name: "设置起点" }).click();

  await page.getByRole("spinbutton", { name: "X" }).fill("4.00");
  await page.getByRole("spinbutton", { name: "Y" }).fill("1.50");
  await page.getByRole("spinbutton", { name: "Z" }).fill("0.25");
  await page.getByRole("button", { name: "添加目标" }).click();

  // No preview line on add — 启动 dispatches leg 0 to bridge.
  await expect(page.getByText("0 规划点")).toBeVisible();
  await clickPlayAndWaitForBridgeTrajectory(page);
  await expect(page.getByText("3 规划点")).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();

  const publishes = await page.evaluate(() => {
    return (window as unknown as { __nav3dRosbridgeMessages: () => RosbridgePublish[] }).__nav3dRosbridgeMessages()
      .filter((message) => message.topic === "/nav3d/start" || message.topic === "/nav3d/goal")
      .map((message) => ({
        topic: message.topic,
        position: message.msg?.pose?.position,
      }));
  });

  // Build sequence: first 起点 form-submit (1.5,0,0) auto-promotes to start
  // (which clears any prior chain and republishes start) → 清空当前规划 →
  // 设置起点 (-1,0.5,0.25) → publishStart fires → 添加目标 (4,1.5,0.25) is
  // local-only placeholder → 启动 → publishPlanRequest fires both start+goal.
  expect(publishes).toEqual([
    { topic: "/nav3d/start", position: { x: 1.5, y: 0, z: 0 } },
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.25 } },
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.25 } },
    { topic: "/nav3d/goal", position: { x: 4, y: 1.5, z: 0.25 } },
  ]);
});

test("shows failed planner status without enabling playback when ROSBridge returns no trajectory", async ({
  page,
}, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only rosbridge failure coverage");
  await installRosbridgePlanFailureMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();

  await page.getByRole("button", { name: "起点" }).click();
  await clickScenePoint(page, { x: -1, y: 0.5, z: 0.25 });
  await expect(page.getByText("起点：已设")).toBeVisible();

  await clickScenePoint(page, { x: 4, y: 1.5, z: 0.25 });

  // No preview line authored — operator can press 启动 (enabled because
  // start+goal are set) and the bridge will reject the goal with plan_failed.
  await expect(page.getByText("0 规划点")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  await page.getByRole("button", { name: "启动", exact: true }).click();

  await expect(page.getByText("plan_failed reason=goal_outside_active_map attempts=1")).toBeVisible();
  await expect(page.getByText("失败")).toBeVisible();
  // The placeholder leg keeps drawing a 2-vertex preview, so the trajectory
  // source label stays "/nav3d/trajectory" rather than the legacy
  // "等待 bridge 规划" hint that only fired when no route existed at all.
  await expect(
    page.getByText("当前目标被 planner 拒绝，未发布新的 /nav3d/trajectory；请检查 bridge 加载的 PCD 与页面地图/坐标是否一致。"),
  ).toBeVisible();

  const publishes = await page.evaluate(() => {
    return (window as unknown as { __nav3dRosbridgeMessages: () => RosbridgePublish[] }).__nav3dRosbridgeMessages()
      .filter((message) => message.topic === "/nav3d/start" || message.topic === "/nav3d/goal")
      .map((message) => ({
        topic: message.topic,
        position: message.msg?.pose?.position,
      }));
  });

  expect(publishes).toEqual([
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.5 } },
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.5 } },
    { topic: "/nav3d/goal", position: { x: 4, y: 1.5, z: 0.5 } },
  ]);
});

test("publishes clicked ROSBridge start and goal, then renders the bridge trajectory", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only rosbridge click flow coverage");
  await installRosbridgeMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();

  await page.getByRole("button", { name: "起点" }).click();
  await clickProjectedVoxel(page, { x: -1, y: 0.5, z: 0.25 });
  await expect(page.getByText("起点：已设")).toBeVisible();
  await expect(page.locator('[aria-label="地图与目标"]').getByText(/-1\.00,\s*0\.50,\s*0\.50/)).toBeVisible();

  await clickProjectedVoxel(page, { x: 4, y: 1.5, z: 0.25 });
  await expect(page.getByText("0 规划点")).toBeVisible();
  await clickPlayAndWaitForBridgeTrajectory(page);

  await expect(page.getByText("1 起点")).toBeVisible();
  await expect(page.getByText("1 目标")).toBeVisible();
  await expect(page.getByText("3 规划点")).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();

  const publishes = await page.evaluate(() => {
    return (window as unknown as { __nav3dRosbridgeMessages: () => RosbridgePublish[] }).__nav3dRosbridgeMessages()
      .filter((message) => message.topic === "/nav3d/start" || message.topic === "/nav3d/goal")
      .map((message) => ({
        topic: message.topic,
        position: message.msg?.pose?.position,
      }));
  });

  expect(publishes).toEqual([
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.5 } },
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.5 } },
    { topic: "/nav3d/goal", position: { x: 4, y: 1.5, z: 0.5 } },
  ]);
});

test("keeps the previous trajectory when appending a second ROSBridge goal", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only multi-goal trajectory coverage");
  await installRosbridgeMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await page.getByRole("button", { name: "起点" }).click();
  await fillCoordinateForm(page, { x: -1, y: 0.5, z: 0.25 });
  await page.getByRole("button", { name: "设置起点" }).click();

  // Two queued goals — bridge silent until 启动, no preview line drawn.
  await fillCoordinateForm(page, { x: 4, y: 1.5, z: 0.25 });
  await page.getByRole("button", { name: "添加目标" }).click();
  await expect(page.getByText("0 规划点")).toBeVisible();

  await fillCoordinateForm(page, { x: 5, y: -1, z: 0.25 });
  await page.getByRole("button", { name: "添加目标" }).click();
  await expect(page.getByText("2 目标")).toBeVisible();
  // Two queued goals — still no preview, route stays empty until 启动.
  await expect(page.getByText("0 规划点")).toBeVisible();

  // 启动 dispatches leg 0; the cruise watcher would dispatch leg 1 once the
  // robot reaches goal 0 — without odom we just verify leg 0 was published.
  await clickPlayAndWaitForBridgeTrajectory(page);

  const publishes = await page.evaluate(() => {
    return (window as unknown as { __nav3dRosbridgeMessages: () => RosbridgePublish[] }).__nav3dRosbridgeMessages()
      .filter((message) => message.topic === "/nav3d/start" || message.topic === "/nav3d/goal")
      .map((message) => ({
        topic: message.topic,
        position: message.msg?.pose?.position,
      }));
  });

  // Build sequence: publishStart on 设置起点 (goes through publishStart twice
  // because handleConnectBridge also publishes the maxSpeed pre-roll path),
  // then 启动 publishes (start = -1,0.5,0.25, goal = 4,1.5,0.25) for leg 0.
  expect(publishes).toEqual([
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.25 } },
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.25 } },
    { topic: "/nav3d/goal", position: { x: 4, y: 1.5, z: 0.25 } },
  ]);
});

test("keeps clicked goals ordered as start to first goal to second goal", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only clicked multi-goal coverage");
  await installRosbridgeMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await expectCanvasHasCoolOctomap(page);

  await page.getByRole("button", { name: "起点" }).click();
  await clickScenePoint(page, { x: -1, y: 0.5, z: 0 });
  await expect(page.getByText("起点：已设")).toBeVisible();

  await clickScenePoint(page, { x: 4, y: 1.5, z: 0 });
  await clickScenePoint(page, { x: 3, y: 0.5, z: 0 });
  await expect(page.getByText("2 目标")).toBeVisible();
  // Two queued goals — no preview line until 启动.
  await expect(page.getByText("0 规划点")).toBeVisible();

  await clickPlayAndWaitForBridgeTrajectory(page);

  const publishes = await page.evaluate(() => {
    return (window as unknown as { __nav3dRosbridgeMessages: () => RosbridgePublish[] }).__nav3dRosbridgeMessages()
      .filter((message) => message.topic === "/nav3d/start" || message.topic === "/nav3d/goal")
      .map((message) => ({
        topic: message.topic,
        position: message.msg?.pose?.position,
      }));
  });

  // Click 1 sets start (publishStart x2 for the latch path) → click 2 + 3
  // append placeholders only → 启动 publishes (start = -1,0.5,0.5, goal =
  // 4,1.5,0.5) for leg 0.
  expect(publishes).toEqual([
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.5 } },
    { topic: "/nav3d/start", position: { x: -1, y: 0.5, z: 0.5 } },
    { topic: "/nav3d/goal", position: { x: 4, y: 1.5, z: 0.5 } },
  ]);
});

test("keeps the original 3D trajectory after switching through 2D view", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only 2D/3D trajectory projection coverage");
  await installRosbridgeMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await page.getByRole("button", { name: "起点" }).click();
  await fillCoordinateForm(page, { x: -1, y: 0.5, z: 0.25 });
  await page.getByRole("button", { name: "设置起点" }).click();

  await fillCoordinateForm(page, { x: 4, y: 1.5, z: 0.25 });
  await page.getByRole("button", { name: "添加目标" }).click();
  await expect(page.getByText("0 规划点")).toBeVisible();
  // 启动 to materialise the 3-pose bridge curve we want to verify projects.
  await clickPlayAndWaitForBridgeTrajectory(page);
  await expect(page.getByText("3 规划点")).toBeVisible();

  expect(await getSceneRoutePoints(page)).toEqual([
    { x: -1, y: 0.5, z: 0.25 },
    { x: 1.5, y: 1, z: 0.25 },
    { x: 4, y: 1.5, z: 0.25 },
  ]);

  await page.getByRole("button", { name: "2D" }).click();
  expect(await getSceneRoutePoints(page)).toEqual([
    { x: -1, y: 0.5, z: 0 },
    { x: 1.5, y: 1, z: 0 },
    { x: 4, y: 1.5, z: 0 },
  ]);

  await page.getByRole("button", { name: "3D" }).click();
  expect(await getSceneRoutePoints(page)).toEqual([
    { x: -1, y: 0.5, z: 0.25 },
    { x: 1.5, y: 1, z: 0.25 },
    { x: 4, y: 1.5, z: 0.25 },
  ]);
});

test("publishes the clicked ROSBridge voxel top surface instead of its occupied center", async ({
  page,
}, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only high voxel click coverage");
  await installRosbridgeHighVoxelMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();

  await page.getByRole("button", { name: "起点" }).click();
  await clickProjectedVoxel(page, { x: 0, y: 0, z: 6.25 });

  await expect(page.getByText("起点：已设")).toBeVisible();
  await expect(page.locator('[aria-label="地图与目标"]').getByText(/0\.00,\s*0\.00,\s*6\.50/)).toBeVisible();

  const publishes = await page.evaluate(() => {
    return (window as unknown as { __nav3dRosbridgeMessages: () => RosbridgePublish[] }).__nav3dRosbridgeMessages()
      .filter((message) => message.topic === "/nav3d/start")
      .map((message) => ({
        topic: message.topic,
        position: message.msg?.pose?.position,
      }));
  });

  expect(publishes).toEqual([{ topic: "/nav3d/start", position: { x: 0, y: 0, z: 6.5 } }]);
});

test("publishes ROSBridge planning-plane start and goal clicks", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only planning-plane click flow coverage");
  await installRosbridgeMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();

  await page.getByRole("button", { name: "起点" }).click();
  await clickScenePoint(page, { x: 0, y: 1, z: 0 });
  await expect(page.getByText("起点：已设")).toBeVisible();
  await expect(page.locator('[aria-label="地图与目标"]').getByText(/0\.00,\s*1\.00,\s*0\.00/)).toBeVisible();

  await clickScenePoint(page, { x: 3, y: 1, z: 0 });
  await expect(page.getByText("0 规划点")).toBeVisible();
  await clickPlayAndWaitForBridgeTrajectory(page);

  await expect(page.getByText("1 起点")).toBeVisible();
  await expect(page.getByText("1 目标")).toBeVisible();
  await expect(page.getByText("3 规划点")).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();

  const publishes = await page.evaluate(() => {
    return (window as unknown as { __nav3dRosbridgeMessages: () => RosbridgePublish[] }).__nav3dRosbridgeMessages()
      .filter((message) => message.topic === "/nav3d/start" || message.topic === "/nav3d/goal")
      .map((message) => ({
        topic: message.topic,
        position: message.msg?.pose?.position,
      }));
  });

  expect(publishes).toEqual([
    { topic: "/nav3d/start", position: { x: 0, y: 1, z: 0 } },
    { topic: "/nav3d/start", position: { x: 0, y: 1, z: 0 } },
    { topic: "/nav3d/goal", position: { x: 3, y: 1, z: 0 } },
  ]);
});

test("shows the full active ROS map bounds after live occupancy arrives", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only active map bounds coverage");
  await installRosbridgeMock(page);
  await page.goto("/");

  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await expect(page.getByText("2 体素")).toBeVisible();
  await expect(page.getByText("X -1.25..4.25")).toBeVisible();
  await expect(page.getByText("Y 0.25..1.75")).toBeVisible();
  await expect(page.getByText("Z 0.00..0.50")).toBeVisible();
});

test("renders a live ROSBridge trajectory from the running ROS2 bridge", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only live rosbridge coverage");
  test.skip(
    process.env.NAV3D_E2E_LIVE_ROSBRIDGE !== "1",
    "requires nav3d_bridge.launch.py running with rosbridge:=true rosbridge_port:=9090",
  );

  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();

  await page.getByRole("button", { name: "起点" }).click();
  await clickProjectedVoxel(page, { x: 2, y: 0, z: 0 });
  await expect(page.getByText("起点：已设")).toBeVisible();

  await clickProjectedVoxel(page, { x: 0, y: 0, z: 0 });

  const status = page.getByText(/plan_success poses=\d+/);
  await expect(status).toBeVisible({ timeout: 8000 });
  const statusText = await status.textContent();
  const poseCount = Number(statusText?.match(/poses=(\d+)/)?.[1] ?? 0);
  expect(poseCount).toBeGreaterThan(1);
  await expect(page.getByText("1 起点")).toBeVisible();
  await expect(page.getByText("1 目标")).toBeVisible();
  await expect(page.getByText(`${poseCount} 规划点`)).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  await expectCanvasHasRenderedLiveMap(page);
});

test("renders a full live mock corridor trajectory from clicked start and goal", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only live mock corridor coverage");
  test.skip(
    process.env.NAV3D_E2E_LIVE_MOCK_CORRIDOR_ROSBRIDGE !== "1",
    "requires tools/testdata/mock_corridor_map.pcd bridge running with rosbridge:=true rosbridge_port:=9090",
  );

  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await expect(page.getByText("8 体素")).toBeVisible({ timeout: 8000 });
  await expect(page.getByText("X -3.00..3.50")).toBeVisible();
  await expect(page.getByText("Y -1.00..1.50")).toBeVisible();
  await expect(page.getByText("Z 0.00..0.50")).toBeVisible();

  await page.getByRole("button", { name: "起点" }).click();
  await clickScenePoint(page, { x: -2, y: 0, z: 0 });
  await expect(page.getByText("起点：已设")).toBeVisible();

  await clickScenePoint(page, { x: 2, y: 0, z: 0 });

  const status = page.getByText(/plan_success poses=\d+/);
  await expect(status).toBeVisible({ timeout: 8000 });
  const statusText = await status.textContent();
  expect(statusText).toContain("partial=false");
  expect(statusText).toContain("planned_goal=2,0,0");
  expect(statusText).toContain("requested_goal=2,0,0");
  const poseCount = Number(statusText?.match(/poses=(\d+)/)?.[1] ?? 0);
  expect(poseCount).toBeGreaterThan(20);
  await expect(page.getByText(`${poseCount} 规划点`)).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
  await expect(page.getByText("partial=false", { exact: true })).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  await expectCanvasHasRenderedLiveMap(page);
});

test("renders a configured live ROSBridge trajectory from clicked start and goal", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only configured live rosbridge click coverage");
  test.skip(
    process.env.NAV3D_E2E_LIVE_CONFIGURED_ROSBRIDGE !== "1",
    "requires a configured bridge running with rosbridge:=true rosbridge_port:=9090",
  );

  const options = configuredLiveClickOptions();

  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await expectCanvasHasRenderedLiveMap(page);
  for (const expectedBound of options.expectedBounds) {
    await expect(page.getByText(expectedBound)).toBeVisible({ timeout: 8000 });
  }

  await page.getByRole("button", { name: "起点" }).click();
  await clickScenePoint(page, options.start);
  await expect(page.getByText("起点：已设")).toBeVisible();

  await clickScenePoint(page, options.goal);

  const status = page.getByText(/plan_success poses=\d+/);
  await expect(status).toBeVisible({ timeout: 12_000 });
  const statusText = await status.textContent();
  if (options.expectedPartial !== null) {
    expect(statusText).toContain(`partial=${options.expectedPartial}`);
  }
  const poseCount = Number(statusText?.match(/poses=(\d+)/)?.[1] ?? 0);
  expect(poseCount).toBeGreaterThanOrEqual(options.minTrajectoryPoses);
  await expect(page.getByText("1 起点")).toBeVisible();
  await expect(page.getByText("1 目标")).toBeVisible();
  await expect(page.getByText(`${poseCount} 规划点`)).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  if (options.expectedPartial !== null) {
    await expect(page.getByText(`partial=${options.expectedPartial}`, { exact: true })).toBeVisible();
  }
  await expectCanvasHasRenderedLiveMap(page);
});

test("renders a live reference PCD trajectory from coordinate start and goal", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only live reference rosbridge coverage");
  test.skip(
    process.env.NAV3D_E2E_LIVE_REFERENCE_ROSBRIDGE !== "1",
    "requires building2_9.pcd bridge running with rosbridge:=true rosbridge_port:=9090",
  );

  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await expect(page.getByText("9666 体素")).toBeVisible({ timeout: 8000 });

  await page.getByRole("button", { name: "起点" }).click();
  await page.getByRole("spinbutton", { name: "X" }).fill("-13.00");
  await page.getByRole("spinbutton", { name: "Y" }).fill("8.00");
  await page.getByRole("spinbutton", { name: "Z" }).fill("1.00");
  await page.getByRole("button", { name: "设置起点" }).click();

  await page.getByRole("spinbutton", { name: "X" }).fill("13.00");
  await page.getByRole("spinbutton", { name: "Y" }).fill("8.00");
  await page.getByRole("spinbutton", { name: "Z" }).fill("1.00");
  await page.getByRole("button", { name: "添加目标" }).click();

  const status = page.getByText(/plan_success poses=\d+/);
  await expect(status).toBeVisible({ timeout: 12_000 });
  const statusText = await status.textContent();
  expect(statusText).toContain("partial=false");
  const poseCount = Number(statusText?.match(/poses=(\d+)/)?.[1] ?? 0);
  expect(poseCount).toBeGreaterThan(50);
  await expect(page.getByText("1 起点")).toBeVisible();
  await expect(page.getByText("1 目标")).toBeVisible();
  await expect(page.getByText(`${poseCount} 规划点`)).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  await expectCanvasHasRenderedLiveMap(page);
});

test("drives the external cmd_vel pose simulator from browser start and goal", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only live external simulator coverage");
  test.skip(
    process.env.NAV3D_E2E_LIVE_EXTERNAL_CMD_VEL_SIM !== "1",
    "requires building2_9.pcd bridge with rosbridge:=true controller_command_frame:=map cmd_vel_pose_sim:=true",
  );
  test.setTimeout(100_000);

  const start = { x: -13, y: 8, z: 1 };
  const goal = { x: 13, y: 8, z: 1 };

  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await expect(page.getByText("9666 体素")).toBeVisible({ timeout: 8000 });

  await page.getByRole("button", { name: "起点" }).click();
  await fillCoordinateForm(page, start);
  await page.getByRole("button", { name: "设置起点" }).click();
  await expect(
    page.locator('[aria-label="机器人控制"] .state-grid dd').filter({ hasText: /-\d+\.\d{2},\s*8\.00,\s*1\.00/ }),
  ).toBeVisible({ timeout: 8000 });
  const observationPromise = watchLiveExternalCmdVelSim(page, goal);

  await fillCoordinateForm(page, goal);
  await page.getByRole("button", { name: "添加目标" }).click();

  const status = page.getByText(/plan_success poses=\d+/);
  await expect(status).toBeVisible({ timeout: 12_000 });
  const statusText = await status.textContent();
  expect(statusText).toContain("partial=false");
  const poseCount = Number(statusText?.match(/poses=(\d+)/)?.[1] ?? 0);
  expect(poseCount).toBeGreaterThan(200);
  await expect(page.getByText(`${poseCount} 规划点`)).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  await expect(page.locator(".bridge-status")).toContainText("plan_success", { timeout: 12_000 });
  await expect
    .poll(
      async () => {
        const text = await page.locator('[aria-label="机器人控制"] .state-grid dd').filter({ hasText: /,/ }).last().textContent();
        return Number(text?.split(",")[0] ?? Number.NaN);
      },
      { timeout: 35_000 },
    )
    .toBeGreaterThan(-10);

  const observation = await observationPromise;
  expect(observation.statuses.some((statusMessage) => statusMessage.includes("tracking_goal_reached"))).toBe(true);
  expect(observation.cmdVelMessages).toBeGreaterThanOrEqual(20);
  expect(observation.nonzeroCmdVelMessages).toBeGreaterThanOrEqual(20);
  expect(observation.currentPoseMessages).toBeGreaterThanOrEqual(20);
  expect(observation.finalGoalError).not.toBeNull();
  expect(observation.finalGoalError ?? Number.POSITIVE_INFINITY).toBeLessThanOrEqual(0.35);
  await expect(page.locator(".bridge-status")).toContainText("tracking_goal_reached", { timeout: 5000 });
  await expect
    .poll(
      async () => {
        const text = await page.locator('[aria-label="机器人控制"] .state-grid dd').filter({ hasText: /,/ }).last().textContent();
        const [xText, yText, zText] = text?.split(",").map((part) => part.trim()) ?? [];
        const x = Number(xText);
        const y = Number(yText);
        const z = Number(zText);
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
          return Number.POSITIVE_INFINITY;
        }
        return Math.sqrt((x - goal.x) ** 2 + (y - goal.y) ** 2 + (z - goal.z) ** 2);
      },
      { timeout: 5000 },
    )
    .toBeLessThanOrEqual(0.35);
});

test("drives the external cmd_vel pose simulator from clicked start and goal", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only live external simulator click coverage");
  test.skip(
    process.env.NAV3D_E2E_LIVE_EXTERNAL_CMD_VEL_SIM !== "1",
    "requires building2_9.pcd bridge with rosbridge:=true controller_command_frame:=map cmd_vel_pose_sim:=true",
  );
  test.setTimeout(100_000);

  const start = { x: -13, y: 8, z: 1 };
  const goal = { x: 13, y: 8, z: 1 };

  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await expect(page.getByText("9666 体素")).toBeVisible({ timeout: 8000 });

  await page.getByRole("button", { name: "起点" }).click();
  await clickScenePoint(page, start);
  await expect(page.getByText("起点：已设")).toBeVisible();
  await expect(
    page.locator('[aria-label="机器人控制"] .state-grid dd').filter({ hasText: /-\d+\.\d{2},\s*8\.00,\s*1\.00/ }),
  ).toBeVisible({ timeout: 8000 });
  const observationPromise = watchLiveExternalCmdVelSim(page, goal);

  await clickScenePoint(page, goal);

  const status = page.getByText(/plan_success poses=\d+/);
  await expect(status).toBeVisible({ timeout: 12_000 });
  const statusText = await status.textContent();
  expect(statusText).toContain("partial=false");
  const poseCount = Number(statusText?.match(/poses=(\d+)/)?.[1] ?? 0);
  expect(poseCount).toBeGreaterThan(50);
  await expect(page.getByText(`${poseCount} 规划点`)).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  await expect(page.locator(".bridge-status")).toContainText("plan_success", { timeout: 12_000 });
  await expect
    .poll(
      async () => {
        const text = await page.locator('[aria-label="机器人控制"] .state-grid dd').filter({ hasText: /,/ }).last().textContent();
        return Number(text?.split(",")[0] ?? Number.NaN);
      },
      { timeout: 35_000 },
    )
    .toBeGreaterThan(-10);

  const observation = await observationPromise;
  expect(observation.statuses.some((statusMessage) => statusMessage.includes("tracking_goal_reached"))).toBe(true);
  expect(observation.cmdVelMessages).toBeGreaterThanOrEqual(20);
  expect(observation.nonzeroCmdVelMessages).toBeGreaterThanOrEqual(20);
  expect(observation.currentPoseMessages).toBeGreaterThanOrEqual(20);
  expect(observation.finalGoalError).not.toBeNull();
  expect(observation.finalGoalError ?? Number.POSITIVE_INFINITY).toBeLessThanOrEqual(0.35);
  await expect(page.locator(".bridge-status")).toContainText("tracking_goal_reached", { timeout: 5000 });
  await expect
    .poll(
      async () => {
        const text = await page.locator('[aria-label="机器人控制"] .state-grid dd').filter({ hasText: /,/ }).last().textContent();
        const [xText, yText, zText] = text?.split(",").map((part) => part.trim()) ?? [];
        const x = Number(xText);
        const y = Number(yText);
        const z = Number(zText);
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
          return Number.POSITIVE_INFINITY;
        }
        return Math.sqrt((x - goal.x) ** 2 + (y - goal.y) ** 2 + (z - goal.z) ** 2);
      },
      { timeout: 5000 },
    )
    .toBeLessThanOrEqual(0.35);
});

test("renders a live reference PCD trajectory from planning-plane start and goal clicks", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only live reference planning-plane click coverage");
  test.skip(
    process.env.NAV3D_E2E_LIVE_REFERENCE_ROSBRIDGE !== "1",
    "requires building2_9.pcd bridge running with rosbridge:=true rosbridge_port:=9090",
  );

  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await expect(page.getByText("9666 体素")).toBeVisible({ timeout: 8000 });

  await page.getByRole("button", { name: "起点" }).click();
  await clickScenePoint(page, { x: -13, y: 8, z: 1 });
  await expect(page.getByText("起点：已设")).toBeVisible();
  const startText = await page.locator('[aria-label="地图与目标"] dd').filter({ hasText: /,/ }).first().textContent();
  expect(startText).toMatch(/-?\d+\.\d{2},\s*-?\d+\.\d{2},\s*1\.00/);

  await clickScenePoint(page, { x: 13, y: 8, z: 1 });

  const status = page.getByText(/plan_success poses=\d+/);
  await expect(status).toBeVisible({ timeout: 12_000 });
  const statusText = await status.textContent();
  expect(statusText).toContain("partial=false");
  const poseCount = Number(statusText?.match(/poses=(\d+)/)?.[1] ?? 0);
  expect(poseCount).toBeGreaterThan(50);
  await expect(page.getByText("1 起点")).toBeVisible();
  await expect(page.getByText("1 目标")).toBeVisible();
  await expect(page.getByText(`${poseCount} 规划点`)).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  await expectCanvasHasRenderedLiveMap(page);
});

test("soaks live reference PCD ROSBridge planning in one browser session", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only live reference rosbridge soak");
  test.skip(
    process.env.NAV3D_E2E_LIVE_REFERENCE_ROSBRIDGE_SOAK !== "1",
    "requires building2_9.pcd bridge running with rosbridge:=true rosbridge_port:=9090",
  );
  test.setTimeout(180_000);

  const consoleErrors: string[] = [];
  const pageErrors: string[] = [];
  page.on("console", (message) => {
    if (message.type() === "error") {
      consoleErrors.push(message.text());
    }
  });
  page.on("pageerror", (error) => {
    pageErrors.push(error.message);
  });

  const cycles = liveReferenceSoakCycles();
  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await expect(page.getByText("9666 体素")).toBeVisible({ timeout: 8000 });
  await expectCanvasHasRenderedLiveMap(page);

  for (let cycle = 0; cycle < cycles; cycle += 1) {
    if (cycle > 0) {
      await page.getByRole("button", { name: "清空目标" }).click();
      await expect(page.getByText("0 规划点")).toBeVisible();
      await expect(page.getByText("目标：0")).toBeVisible();
    }

    await page.getByRole("button", { name: "起点" }).click();
    await fillCoordinateForm(page, { x: -13, y: 8, z: 1 });
    await page.getByRole("button", { name: "设置起点" }).click();
    await expect(page.getByText("起点：已设")).toBeVisible();

    await fillCoordinateForm(page, { x: 13, y: 8, z: 1 });
    await page.getByRole("button", { name: "添加目标" }).click();

    const status = page.getByText(/plan_success poses=\d+.*partial=false/);
    await expect(status).toBeVisible({ timeout: 12_000 });
    const statusText = await status.textContent();
    const poseCount = Number(statusText?.match(/poses=(\d+)/)?.[1] ?? 0);
    expect(poseCount, `cycle ${cycle + 1} pose count`).toBeGreaterThan(50);
    await expect(page.getByText(`${poseCount} 规划点`)).toBeVisible();
    await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
    await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  }

  await expectCanvasHasRenderedLiveMap(page);
  expect(consoleErrors).toEqual([]);
  expect(pageErrors).toEqual([]);
});

test("renders a live reference PCD trajectory from clicked start and goal", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only live reference click rosbridge coverage");
  test.skip(
    process.env.NAV3D_E2E_LIVE_REFERENCE_ROSBRIDGE !== "1",
    "requires building2_9.pcd bridge running with rosbridge:=true rosbridge_port:=9090",
  );

  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await expect(page.getByText("9666 体素")).toBeVisible({ timeout: 8000 });

  await page.getByRole("button", { name: "起点" }).click();
  await clickProjectedVoxel(page, { x: -13, y: 8, z: 1 });
  await expect(page.getByText("起点：已设")).toBeVisible();

  await clickProjectedVoxel(page, { x: 13, y: 8, z: 1 });

  const status = page.getByText(/plan_success poses=\d+/);
  await expect(status).toBeVisible({ timeout: 12_000 });
  const statusText = await status.textContent();
  expect(statusText).toContain("partial=true");
  const poseCount = Number(statusText?.match(/poses=(\d+)/)?.[1] ?? 0);
  expect(poseCount).toBeGreaterThan(50);
  await expect(page.getByText("1 起点")).toBeVisible();
  await expect(page.getByText("1 目标")).toBeVisible();
  await expect(page.getByText(`${poseCount} 规划点`)).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
  await expect(page.getByText("partial=true", { exact: true })).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();
  await expectCanvasHasRenderedLiveMap(page);
});

test("renders a live local PointCloud2 replan trajectory from ROSBridge", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only live reference local replan coverage");
  test.skip(
    process.env.NAV3D_E2E_LIVE_REFERENCE_LOCAL_REPLAN !== "1",
    "requires building2_9.pcd bridge running with rosbridge:=true rosbridge_port:=9090 local_grid_enabled:=true safety_enabled:=true",
  );

  const start = { x: -13, y: 8, z: 1 };
  const goal = { x: 13, y: 8, z: 1 };

  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();
  await publishLiveLocalPointcloud(page, []);
  await expect(page.getByText("9666 体素")).toBeVisible({ timeout: 8000 });

  const observationPromise = watchLiveLocalReplan(page);

  await page.getByRole("button", { name: "起点" }).click();
  await fillCoordinateForm(page, start);
  await page.getByRole("button", { name: "设置起点" }).click();
  await fillCoordinateForm(page, goal);
  await page.getByRole("button", { name: "添加目标" }).click();

  await expect(page.getByText(/plan_success poses=\d+.*partial=false/)).toBeVisible({ timeout: 12_000 });
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();

  const observation = await observationPromise;
  expect(observation.initialTrajectoryPoses).toBeGreaterThan(200);
  expect(observation.replanTrajectoryPoses).toBeGreaterThan(200);
  expect(observation.maxTrajectoryDelta).toBeGreaterThan(0.02);
  expect(observation.statuses.some((status) => status.includes("safety_replan_needed"))).toBe(true);
  expect(observation.statuses.some((status) => status.includes("safety_replan_success"))).toBe(true);

  await expect(page.locator(".bridge-status")).toContainText("safety_replan_success", { timeout: 5000 });
  await expect(page.getByText(`${observation.replanTrajectoryPoses} 规划点`)).toBeVisible();
  await expect(page.getByText("/nav3d/trajectory")).toBeVisible();
  await expect(page.getByRole("button", { name: "启动", exact: true })).toBeEnabled();

  const renderedRoute = await getSceneRoutePoints(page);
  expect(renderedRoute.length).toBe(observation.replanTrajectoryPoses);
  expect(maxPointDelta(renderedRoute, observation.initialTrajectory)).toBeGreaterThan(0.02);
  expect(maxPointDelta(renderedRoute, observation.replanTrajectory)).toBeLessThan(0.001);

  await publishLiveLocalPointcloud(page, []);
});

test("dragging the scene orbits without adding a waypoint", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only scene interaction coverage");
  await installRosbridgeMock(page);
  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();

  const canvas = page.locator(".scene-canvas");
  await canvas.waitFor({ state: "visible" });
  const box = await canvas.boundingBox();
  expect(box).not.toBeNull();

  if (!box) {
    return;
  }

  await page.mouse.move(box.x + box.width * 0.35, box.y + box.height * 0.45);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.7, box.y + box.height * 0.35, { steps: 8 });
  await page.mouse.up();

  await expect(page.getByText("目标：0")).toBeVisible();

  await page.mouse.click(box.x + box.width * 0.02, box.y + box.height * 0.02);
  await expect(page.getByText("未命中地图范围：请点击 OctoMap 地图内的位置")).toBeVisible();
  await expect(page.getByText("目标：0")).toBeVisible();

  await page.mouse.click(box.x + box.width * 0.52, box.y + box.height * 0.52);
  await expect(page.getByText("起点：已设")).toBeVisible();
  await expect(page.getByText("目标：0")).toBeVisible();
  await expect(page.getByText(/^已设置起点/)).toBeVisible();

  await clickProjectedVoxel(page, { x: 4, y: 1.5, z: 0.25 });
  await expect(page.getByText("目标：1")).toBeVisible();
  await expect(page.getByText(/^已选择目标/)).toBeVisible();
});

test("ctrl left dragging pans the 3D scene without adding a waypoint", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only ctrl-drag interaction coverage");
  await installRosbridgeMock(page);
  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();

  const canvas = page.locator(".scene-canvas");
  await canvas.waitFor({ state: "visible" });
  const box = await canvas.boundingBox();
  expect(box).not.toBeNull();

  if (!box) {
    return;
  }

  const before = await getSceneCameraTarget(page);
  await page.keyboard.down("Control");
  await page.mouse.move(box.x + box.width * 0.5, box.y + box.height * 0.5);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.62, box.y + box.height * 0.62, { steps: 8 });
  await page.mouse.up();
  await page.keyboard.up("Control");

  const after = await getSceneCameraTarget(page);
  expect(Math.hypot(after.x - before.x, after.y - before.y, after.z - before.z)).toBeGreaterThan(0.05);
  await expect(page.getByText("目标：0")).toBeVisible();
});

test("wheel zoom keeps the OctoMap scene visible", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only wheel interaction coverage");
  await installRosbridgeMock(page);
  await page.goto("/");
  await expectRosbridgeAutoConnected(page);
  await expect(page.getByText("ROS 3D OctoMap").first()).toBeVisible();

  const before = await expectCanvasHasRenderedLiveMap(page);
  const canvas = page.locator(".scene-canvas");
  const box = await canvas.boundingBox();
  expect(box).not.toBeNull();

  if (!box) {
    return;
  }

  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await page.mouse.wheel(0, -900);
  await page.waitForTimeout(350);

  const after = await expectCanvasHasRenderedLiveMap(page);
  expect(after.meanLuminance).toBeGreaterThan(before.meanLuminance * 0.45);
});

test("shows rosbridge connection controls on desktop", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "desktop", "desktop-only connection coverage");
  await installClosedRosbridgeMock(page);
  await page.goto("/");

  await expect(page.getByRole("heading", { name: "ROSBridge 连接" })).toBeVisible();
  await expect(page.getByLabel("地址")).toBeVisible();
  await expect(page.getByRole("button", { name: "检查/重连" })).toBeEnabled();
  await expect(page.getByRole("button", { name: "连接", exact: true })).toBeEnabled();
  await expect(page.locator(".status-cell", { hasText: "ROS：" }).getByText(/连接中|已连接|已断开|异常|未连接/)).toBeVisible();
  await expect(page.locator(".status-cell", { hasText: "Bridge：" }).getByText(/未启用|可启动|运行中/)).toBeVisible();
});

test("keeps controls and scene reachable on mobile", async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== "mobile", "mobile-only responsive coverage");
  await installRosbridgeMock(page);
  await page.goto("/");

  await expect(page.getByRole("button", { name: "2D" })).toBeVisible();
  await expect(page.getByRole("button", { name: "3D" })).toBeVisible();
  await expect(page.getByRole("heading", { name: "机器人控制" })).toBeVisible();

  await page.locator(".scene-canvas").scrollIntoViewIfNeeded();
  await expect(page.locator(".scene-canvas")).toBeVisible();
  await expectCanvasHasRenderedLiveMap(page);
});
