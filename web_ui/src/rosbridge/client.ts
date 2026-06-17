import { Point3, sanitizePoint } from "../utils/trajectory";

export type NavigationMode = "2d" | "3d";
export type RosbridgeStatus = "idle" | "connecting" | "connected" | "closed" | "error";

export type OccupancyCell = Point3 & {
  size: number;
  height?: number;
  layer?: "global" | "local" | "grid";
  occupancy?: "occupied" | "free" | "unknown";
};

export type RosbridgeSocket = {
  onopen: ((event?: Event) => void) | null;
  onclose: ((event?: CloseEvent) => void) | null;
  onerror: ((event?: Event) => void) | null;
  onmessage: ((event: MessageEvent<string> | { data: string }) => void) | null;
  send: (data: string) => void;
  close: () => void;
};

export type RosbridgeGoalOptions = {
  goal: Point3;
  mode: NavigationMode;
  topic: string;
  frameId: string;
};

export type RosbridgeClientOptions = {
  url: string;
  startTopic?: string;
  goalTopic?: string;
  mapLoadTopic?: string;
  trajectoryTopic?: string;
  statusTopic?: string;
  currentPoseTopic?: string;
  occupancyGridTopic?: string;
  occupancyVoxelsTopic?: string;
  maxSpeedTopic?: string;
  frameId?: string;
  socketFactory?: (url: string) => RosbridgeSocket;
  onStatus?: (status: RosbridgeStatus) => void;
  onTextStatus?: (status: string) => void;
  onTrajectory?: (points: Point3[]) => void;
  onCurrentPose?: (point: Point3) => void;
  onOccupancyGrid?: (cells: OccupancyCell[]) => void;
  onOccupancyVoxels?: (cells: OccupancyCell[]) => void;
};

export type RosbridgeClient = {
  connect: () => void;
  disconnect: () => void;
  publishStart: (start: Point3, mode: NavigationMode) => void;
  publishGoal: (goal: Point3, mode: NavigationMode) => void;
  publishPcdPath: (path: string) => void;
  publishMaxSpeed: (value: number) => void;
};

type RosbridgeMessage = {
  op?: string;
  topic?: string;
  msg?: unknown;
  id?: unknown;
  data?: unknown;
  num?: unknown;
  total?: unknown;
};

type FragmentBuffer = {
  total: number;
  chunks: string[];
  received: Set<number>;
};

type PoseLike = {
  pose?: {
    position?: Partial<Point3>;
  };
};

const defaultGoalTopic = "/nav3d/goal";
const defaultStartTopic = "/nav3d/start";
const defaultMapLoadTopic = "/nav3d/load_pcd_path";
const defaultTrajectoryTopic = "/nav3d/trajectory";
const defaultStatusTopic = "/nav3d/status";
const defaultCurrentPoseTopic = "/nav3d/current_pose";
const defaultOccupancyGridTopic = "/nav3d/occupied_grid";
const defaultOccupancyVoxelsTopic = "/nav3d/planning_occupied_markers";
const defaultMaxSpeedTopic = "/nav3d/max_speed";
const defaultFrameId = "map";

function buildPoseStampedMessage(point: Point3, frameId: string) {
  return {
    header: {
      frame_id: frameId,
    },
    pose: {
      position: point,
      orientation: {
        x: 0,
        y: 0,
        z: 0,
        w: 1,
      },
    },
  };
}

export function buildPosePublishMessage(point: Point3, topic: string, frameId: string) {
  return {
    op: "publish",
    topic,
    msg: buildPoseStampedMessage(point, frameId),
  };
}

export function buildGoalPublishMessage({ goal, mode, topic, frameId }: RosbridgeGoalOptions) {
  const point = sanitizePoint(goal, mode === "2d");
  return buildPosePublishMessage(point, topic, frameId);
}

export function buildPcdPathPublishMessage(path: string, topic: string) {
  return {
    op: "publish",
    topic,
    msg: {
      data: path.trim(),
    },
  };
}

export function buildFloat32PublishMessage(value: number, topic: string) {
  const safeValue = Number.isFinite(value) ? value : 0;
  return {
    op: "publish",
    topic,
    msg: {
      data: safeValue,
    },
  };
}

export function parseTrajectoryPoints(message: RosbridgeMessage): Point3[] {
  const msg = message.msg;
  if (!msg || typeof msg !== "object") {
    return [];
  }

  const poses = (msg as { poses?: PoseLike[] }).poses;
  if (Array.isArray(poses)) {
    return poses
      .map((entry) => entry.pose?.position)
      .filter((position): position is Partial<Point3> => Boolean(position))
      .map((position) =>
        sanitizePoint(
          {
            x: Number(position.x),
            y: Number(position.y),
            z: Number(position.z),
          },
          false,
        ),
      );
  }

  const points = (msg as { points?: Partial<Point3>[] }).points;
  if (Array.isArray(points)) {
    return points.map((point) =>
      sanitizePoint(
        {
          x: Number(point.x),
          y: Number(point.y),
          z: Number(point.z),
        },
        false,
      ),
    );
  }

  return [];
}

export function parseStatusText(message: RosbridgeMessage): string | null {
  const msg = message.msg;
  if (!msg || typeof msg !== "object") {
    return null;
  }

  const data = (msg as { data?: unknown }).data;
  return typeof data === "string" ? data : null;
}

export function parsePosePoint(message: RosbridgeMessage): Point3 | null {
  const msg = message.msg;
  if (!msg || typeof msg !== "object") {
    return null;
  }

  const position = (msg as PoseLike).pose?.position;
  if (!position) {
    return null;
  }

  return sanitizePoint(
    {
      x: Number(position.x),
      y: Number(position.y),
      z: Number(position.z),
    },
    false,
  );
}

export function parseOccupancyGridCells(message: RosbridgeMessage): OccupancyCell[] {
  const msg = message.msg;
  if (!msg || typeof msg !== "object") {
    return [];
  }

  const grid = msg as {
    info?: {
      resolution?: unknown;
      width?: unknown;
      height?: unknown;
      origin?: {
        position?: Partial<Point3>;
      };
    };
    data?: unknown[];
  };
  const resolution = Number(grid.info?.resolution);
  const width = Number(grid.info?.width);
  const height = Number(grid.info?.height);
  if (!Number.isFinite(resolution) || resolution <= 0 || !Number.isInteger(width) || !Number.isInteger(height)) {
    return [];
  }
  if (width <= 0 || height <= 0 || !Array.isArray(grid.data)) {
    return [];
  }

  const origin = grid.info?.origin?.position ?? {};
  const originX = Number.isFinite(Number(origin.x)) ? Number(origin.x) : 0;
  const originY = Number.isFinite(Number(origin.y)) ? Number(origin.y) : 0;
  const originZ = Number.isFinite(Number(origin.z)) ? Number(origin.z) : 0;
  const cells: OccupancyCell[] = [];
  const totalCells = Math.min(grid.data.length, width * height);
  for (let index = 0; index < totalCells; index += 1) {
    const occupancy = Number(grid.data[index]);
    if (!Number.isFinite(occupancy)) {
      continue;
    }
    const xIndex = index % width;
    const yIndex = Math.floor(index / width);
    cells.push({
      x: originX + (xIndex + 0.5) * resolution,
      y: originY + (yIndex + 0.5) * resolution,
      z: originZ + resolution / 2,
      size: resolution,
      layer: "grid",
      occupancy: occupancy < 0 ? "unknown" : occupancy >= 50 ? "occupied" : "free",
    });
  }
  return cells;
}

function markerSize(marker: { scale?: Partial<Point3> }): number {
  const scale = marker.scale ?? {};
  const xScale = Number(scale.x);
  if (Number.isFinite(xScale) && xScale > 0) {
    return xScale;
  }
  const values = [Number(scale.x), Number(scale.y), Number(scale.z)].filter(
    (value) => Number.isFinite(value) && value > 0,
  );
  if (values.length === 0) {
    return 1;
  }
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

function markerLayer(ns: unknown): OccupancyCell["layer"] {
  const namespace = typeof ns === "string" ? ns.toLowerCase() : "";
  return namespace.includes("local") ? "local" : "global";
}

export function parseOccupancyVoxelCells(message: RosbridgeMessage): OccupancyCell[] {
  const msg = message.msg;
  if (!msg || typeof msg !== "object") {
    return [];
  }

  const markers = (msg as { markers?: unknown[] }).markers;
  if (!Array.isArray(markers)) {
    return [];
  }

  const cells: OccupancyCell[] = [];
  markers.forEach((entry) => {
    if (!entry || typeof entry !== "object") {
      return;
    }
    const marker = entry as {
      action?: unknown;
      ns?: unknown;
      type?: unknown;
      pose?: { position?: Partial<Point3> };
      points?: Partial<Point3>[];
      scale?: Partial<Point3>;
    };
    const action = Number(marker.action);
    if (action === 2 || action === 3) {
      return;
    }

    const type = Number(marker.type);
    const size = markerSize(marker);
    const layer = markerLayer(marker.ns);
    if (type === 6 && Array.isArray(marker.points)) {
      marker.points.forEach((point) => {
        const x = Number(point.x);
        const y = Number(point.y);
        const z = Number(point.z);
        if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
          cells.push({ x, y, z, size, layer });
        }
      });
      return;
    }

    if (type === 1 && marker.pose?.position) {
      const x = Number(marker.pose.position.x);
      const y = Number(marker.pose.position.y);
      const z = Number(marker.pose.position.z);
      if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
        cells.push({ x, y, z, size, layer });
      }
    }
  });
  return cells;
}

function appendRosbridgeFragment(
  message: RosbridgeMessage,
  buffers: Map<string, FragmentBuffer>,
): RosbridgeMessage | null {
  const id = typeof message.id === "string" || typeof message.id === "number" ? String(message.id) : "";
  const data = typeof message.data === "string" ? message.data : "";
  const num = Number(message.num);
  const total = Number(message.total);
  if (
    id.length === 0 ||
    data.length === 0 ||
    !Number.isInteger(num) ||
    !Number.isInteger(total) ||
    total <= 0 ||
    num < 0 ||
    num >= total
  ) {
    return null;
  }

  const current =
    buffers.get(id) ??
    {
      total,
      chunks: new Array<string>(total),
      received: new Set<number>(),
    };
  if (current.total !== total) {
    buffers.set(id, {
      total,
      chunks: new Array<string>(total),
      received: new Set<number>(),
    });
    return appendRosbridgeFragment(message, buffers);
  }

  current.chunks[num] = data;
  current.received.add(num);
  buffers.set(id, current);
  if (current.received.size !== total) {
    return null;
  }

  for (let index = 0; index < total; index += 1) {
    if (!current.received.has(index)) {
      return null;
    }
  }

  buffers.delete(id);
  return JSON.parse(current.chunks.join("")) as RosbridgeMessage;
}

export function createRosbridgeClient({
  url,
  startTopic = defaultStartTopic,
  goalTopic = defaultGoalTopic,
  mapLoadTopic = defaultMapLoadTopic,
  trajectoryTopic = defaultTrajectoryTopic,
  statusTopic = defaultStatusTopic,
  currentPoseTopic = defaultCurrentPoseTopic,
  occupancyGridTopic = defaultOccupancyGridTopic,
  occupancyVoxelsTopic = defaultOccupancyVoxelsTopic,
  maxSpeedTopic = defaultMaxSpeedTopic,
  frameId = defaultFrameId,
  socketFactory = (socketUrl) => new WebSocket(socketUrl) as unknown as RosbridgeSocket,
  onStatus,
  onTextStatus,
  onTrajectory,
  onCurrentPose,
  onOccupancyGrid,
  onOccupancyVoxels,
}: RosbridgeClientOptions): RosbridgeClient {
  let socket: RosbridgeSocket | null = null;
  let connected = false;
  let generation = 0;
  const fragmentBuffers = new Map<string, FragmentBuffer>();

  const setStatus = (status: RosbridgeStatus) => onStatus?.(status);

  const sendJson = (message: unknown) => {
    if (!socket || !connected) {
      return;
    }
    socket.send(JSON.stringify(message));
  };

  return {
    connect: () => {
      if (socket) {
        return;
      }

      generation += 1;
      const socketGeneration = generation;
      setStatus("connecting");
      const nextSocket = socketFactory(url);
      socket = nextSocket;
      const isCurrentSocket = () => socket === nextSocket && socketGeneration === generation;
      nextSocket.onopen = () => {
        if (!isCurrentSocket()) {
          return;
        }
        connected = true;
        setStatus("connected");
        sendJson({
          op: "subscribe",
          topic: trajectoryTopic,
          type: "nav_msgs/Path",
        });
        sendJson({
          op: "subscribe",
          topic: statusTopic,
          type: "std_msgs/String",
        });
        sendJson({
          op: "subscribe",
          topic: currentPoseTopic,
          type: "geometry_msgs/PoseStamped",
        });
        sendJson({
          op: "subscribe",
          topic: occupancyGridTopic,
          type: "nav_msgs/OccupancyGrid",
        });
        sendJson({
          op: "subscribe",
          topic: occupancyVoxelsTopic,
          type: "visualization_msgs/MarkerArray",
        });
      };
      nextSocket.onclose = () => {
        if (!isCurrentSocket()) {
          return;
        }
        connected = false;
        socket = null;
        fragmentBuffers.clear();
        setStatus("closed");
      };
      nextSocket.onerror = () => {
        if (!isCurrentSocket()) {
          return;
        }
        connected = false;
        setStatus("error");
      };
      nextSocket.onmessage = (event) => {
        if (!isCurrentSocket()) {
          return;
        }
        try {
          const envelope = JSON.parse(event.data) as RosbridgeMessage;
          const message =
            envelope.op === "fragment"
              ? appendRosbridgeFragment(envelope, fragmentBuffers)
              : envelope;
          if (!message) {
            return;
          }
          if (message.op === "publish" && message.topic === trajectoryTopic) {
            const points = parseTrajectoryPoints(message);
            onTrajectory?.(points);
          }
          if (message.op === "publish" && message.topic === statusTopic) {
            const status = parseStatusText(message);
            if (status) {
              onTextStatus?.(status);
            }
          }
          if (message.op === "publish" && message.topic === currentPoseTopic) {
            const point = parsePosePoint(message);
            if (point) {
              onCurrentPose?.(point);
            }
          }
          if (message.op === "publish" && message.topic === occupancyGridTopic) {
            onOccupancyGrid?.(parseOccupancyGridCells(message));
          }
          if (message.op === "publish" && message.topic === occupancyVoxelsTopic) {
            onOccupancyVoxels?.(parseOccupancyVoxelCells(message));
          }
        } catch {
          setStatus("error");
        }
      };
    },
    disconnect: () => {
      const currentSocket = socket;
      generation += 1;
      socket = null;
      connected = false;
      fragmentBuffers.clear();
      currentSocket?.close();
      setStatus("closed");
    },
    publishStart: (start, mode) => {
      sendJson(buildPosePublishMessage(sanitizePoint(start, mode === "2d"), startTopic, frameId));
    },
    publishGoal: (goal, mode) => {
      sendJson(
        buildGoalPublishMessage({
          goal,
          mode,
          topic: goalTopic,
          frameId,
        }),
      );
    },
    publishPcdPath: (path) => {
      const trimmed = path.trim();
      if (trimmed.length === 0) {
        return;
      }
      sendJson(buildPcdPathPublishMessage(trimmed, mapLoadTopic));
    },
    publishMaxSpeed: (value) => {
      sendJson(buildFloat32PublishMessage(value, maxSpeedTopic));
    },
  };
}
