import { FormEvent, useCallback, useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import {
  createRosbridgeClient,
  type NavigationMode,
  type OccupancyCell,
  type RosbridgeClient,
  type RosbridgeStatus,
} from "./rosbridge/client";
import { expectedMapTopic, mapSourceLabel, selectDisplayedMap, mapPresets, findMapPreset, cellsLikelyUnchanged } from "./utils/mapSource";
import {
  Point3,
  type PlannedSegment,
  flattenSegments,
  formatCoordinate,
  getPathLength,
  progressAtSegmentEnd,
  selectVoxelTargetFromHits,
  sanitizePoint,
  splitPolylineAtProgress,
} from "./utils/trajectory";
import { projectPointToPolyline } from "./utils/progress";
import {
  type Bounds,
  boundsCenter,
  boundsDiagonal,
  boundsFromCells,
  boundsSpan,
  containsMapPoint,
  mapPlanningPlaneZ,
  projectPointForMode,
  projectPointsForMode,
  roundScenePoint,
} from "./utils/bounds";
import {
  cellColor,
  createGoalLabel,
  createPointLabel,
  disposeObject,
  isGridCell,
  mapPalette,
} from "./utils/palette";
import { MiniMap } from "./components/MiniMap";
import { NavigationScene } from "./components/NavigationScene";
import { GoalManager } from "./components/GoalManager";

type PlaybackState = "idle" | "playing" | "paused" | "complete";
type PointRole = "start" | "goal";
type BridgeLaunchStatus = {
  available: boolean;
  running: boolean;
  pid: number | null;
  command: string[];
  logPath: string;
  message: string;
};
type PlanStatusKind = "idle" | "full" | "partial" | "failed";
type PickStatus =
  | { kind: "idle"; text: string }
  | { kind: "hit"; text: string }
  | { kind: "miss"; text: string };
type PendingPlanRequest = {
  append: boolean;
  baseRoute: Point3[];
  goalIndex: number;
};

type CruiseState = {
  // Index of the leg currently being driven; -1 means cruise has not started.
  legIndex: number;
  // Highest legIndex for which we have already dispatched the next leg's
  // start+goal pair. Prevents double-publish when progress oscillates near a
  // segment endpoint.
  dispatchedFor: number;
};

const initialForm = { x: "2.00", y: "1.50", z: "0.00" };
const defaultRosbridgeUrl = "ws://localhost:9090";

function playbackLabel(playback: PlaybackState): string {
  switch (playback) {
    case "playing":
      return "运行中";
    case "paused":
      return "暂停";
    case "complete":
      return "完成";
    case "idle":
    default:
      return "待机";
  }
}

function bridgeStatusLabel(status: RosbridgeStatus): string {
  switch (status) {
    case "connecting":
      return "连接中";
    case "connected":
      return "已连接";
    case "closed":
      return "已断开";
    case "error":
      return "异常";
    case "idle":
    default:
      return "未连接";
  }
}

function bridgeLaunchLabel(status: BridgeLaunchStatus | null): string {
  if (!status) {
    return "未检查";
  }
  if (status.running) {
    return `运行中 pid=${status.pid ?? "unknown"}`;
  }
  return status.available ? "可连接" : "未启用";
}

function trajectorySourceLabel(points: Point3[], bridgeStatus: RosbridgeStatus): string {
  if (points.length > 0) {
    return "/nav3d/trajectory";
  }
  return bridgeStatus === "connected" ? "等待 bridge 规划" : "先连接 ROSBridge";
}

function parsePlanStatusKind(status: string): PlanStatusKind | null {
  if (status.includes("plan_failed")) {
    return "failed";
  }
  if (!status.includes("plan_success")) {
    return null;
  }
  return status.includes("partial=true") ? "partial" : "full";
}

async function requestBridgeLaunch(): Promise<BridgeLaunchStatus | null> {
  try {
    const statusResponse = await fetch("/api/nav3d/bridge/status");
    if (!statusResponse.ok) {
      return null;
    }
    return (await statusResponse.json()) as BridgeLaunchStatus;
  } catch {
    return null;
  }
}

function pointsNearlyEqual(left: Point3, right: Point3): boolean {
  return Math.hypot(left.x - right.x, left.y - right.y, left.z - right.z) < 1e-3;
}

function routeContainsSegment(route: Point3[], segment: Point3[]): boolean {
  if (route.length === 0 || segment.length === 0 || segment.length > route.length) {
    return false;
  }
  for (let startIndex = 0; startIndex <= route.length - segment.length; startIndex += 1) {
    const contained = segment.every((point, segmentIndex) =>
      pointsNearlyEqual(route[startIndex + segmentIndex], point),
    );
    if (contained) {
      return true;
    }
  }
  return false;
}

function cloneRoute(points: Point3[]): Point3[] {
  return points.map((point) => ({ ...point }));
}

export default function App() {
  const [mode, setMode] = useState<NavigationMode>("3d");
  const [pointRole, setPointRole] = useState<PointRole>("goal");
  const [startPoint, setStartPoint] = useState<Point3 | null>(null);
  const [goals, setGoals] = useState<Point3[]>([]);
  const [form, setForm] = useState(initialForm);
  const [playback, setPlayback] = useState<PlaybackState>("idle");
  const [progress, setProgress] = useState(0);
  const [speed, setSpeed] = useState(1.2);
  const [bridgeUrl, setBridgeUrl] = useState(defaultRosbridgeUrl);
  const [pcdPath, setPcdPath] = useState("");
  const [selectedMapId, setSelectedMapId] = useState(mapPresets[0]?.id ?? "");
  const [bridgeStatus, setBridgeStatus] = useState<RosbridgeStatus>("idle");
  const [bridgeMessage, setBridgeMessage] = useState("未连接");
  const [planStatusKind, setPlanStatusKind] = useState<PlanStatusKind>("idle");
  const [bridgeLaunchStatus, setBridgeLaunchStatus] = useState<BridgeLaunchStatus | null>(null);
  const [liveRobotPosition, setLiveRobotPosition] = useState<Point3 | null>(null);
  const [plannedSegments, setPlannedSegments] = useState<PlannedSegment[]>([]);
  const [bridgeGridCells, setBridgeGridCells] = useState<OccupancyCell[]>([]);
  const [bridgeVoxelCells, setBridgeVoxelCells] = useState<OccupancyCell[]>([]);
  const [pickStatus, setPickStatus] = useState<PickStatus>({
    kind: "idle",
    text: "点击 OctoMap 体素选择起点或目标",
  });
  const lastFrameRef = useRef<number | null>(null);
  const bridgeClientRef = useRef<RosbridgeClient | null>(null);
  const plannedSegmentsRef = useRef<PlannedSegment[]>([]);
  const plannedTrajectoryRef = useRef<Point3[]>([]);
  const startPointRef = useRef<Point3 | null>(null);
  const displayedRobotPositionRef = useRef<Point3>({ x: 0, y: 0, z: 0 });
  const modeRef = useRef<NavigationMode>(mode);
  const pendingPlanRef = useRef<PendingPlanRequest | null>(null);
  const queuedGoalsRef = useRef<Point3[]>([]);
  // Cruise dispatcher state — only non-null after 启动 is pressed; cleared on
  // any CRUD that invalidates the chain or on 复位/清空.
  const cruisingRef = useRef<CruiseState | null>(null);
  // Wall-clock when the operator last pressed 启动. Used by the no-odom
  // watchdog (effect below) to surface a guidance message when no
  // /nav3d/current_pose ever arrives — without odom progress stays at 0,
  // cruise never advances, and the operator otherwise has no signal.
  const cruiseStartedAtRef = useRef<number | null>(null);
  const cruiseWatchdogTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  // Manual reset latch — when "复位" is clicked we force progress=0 even if a
  // late /nav3d/current_pose arrives that would project to a non-zero point.
  // Cleared on the next play action.
  const manualResetRef = useRef(false);
  // Debounce handle for max-speed publishes — slider drag emits ~60 onChange
  // events per second; we coalesce into one publish per ~120 ms.
  const speedPublishHandleRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  // Editing index for in-place goal edits; null = "添加目标" mode.
  const [editingGoalIndex, setEditingGoalIndex] = useState<number | null>(null);

  const plannedTrajectory = useMemo(() => flattenSegments(plannedSegments), [plannedSegments]);

  const displayedMap = useMemo(
    () => selectDisplayedMap({ mode, voxelCells: bridgeVoxelCells, gridCells: bridgeGridCells }),
    [bridgeGridCells, bridgeVoxelCells, mode],
  );
  const mapSource = displayedMap.source;
  const displayedOccupancyCells = displayedMap.cells;
  const expectedMapTopicName = expectedMapTopic(mode);
  const hasOnlyNonDisplayedMap =
    displayedOccupancyCells.length === 0 &&
    ((mode === "3d" && bridgeGridCells.length > 0) || (mode === "2d" && bridgeVoxelCells.length > 0));

  const displayedStartPoint = useMemo(
    () => (startPoint ? projectPointForMode(startPoint, mode) : null),
    [mode, startPoint],
  );
  const displayedGoals = useMemo(() => projectPointsForMode(goals, mode), [goals, mode]);
  const displayedRoutePoints = useMemo(
    () => projectPointsForMode(plannedTrajectory, mode),
    [mode, plannedTrajectory],
  );
  const pathLength = useMemo(() => getPathLength(displayedRoutePoints), [displayedRoutePoints]);
  // Robot position is driven by /nav3d/current_pose only — no browser-side
  // playback animation. While the bridge has not delivered a pose yet, fall
  // back to the operator's start point so the HUD coordinates stay sensible.
  const liveOrStartPoint = useMemo<Point3 | null>(() => {
    if (liveRobotPosition) return liveRobotPosition;
    return displayedStartPoint;
  }, [displayedStartPoint, liveRobotPosition]);
  const displayedRobotPosition = projectPointForMode(
    liveOrStartPoint ?? { x: 0, y: 0, z: 0 },
    mode,
  );
  const hasLivePose = liveOrStartPoint !== null;

  useEffect(() => {
    modeRef.current = mode;
  }, [mode]);

  useEffect(() => {
    displayedRobotPositionRef.current = displayedRobotPosition;
  }, [displayedRobotPosition]);

  const mapBounds = useMemo(() => {
    return boundsFromCells(displayedOccupancyCells);
  }, [displayedOccupancyCells]);

  const setStartPointValue = useCallback((point: Point3 | null) => {
    startPointRef.current = point;
    setStartPoint(point);
  }, []);

  const setPlannedSegmentsValue = useCallback((segments: PlannedSegment[]) => {
    plannedSegmentsRef.current = segments;
    plannedTrajectoryRef.current = flattenSegments(segments);
    setPlannedSegments(segments);
  }, []);

  const clearPlannedSegments = useCallback(() => {
    setPlannedSegmentsValue([]);
  }, [setPlannedSegmentsValue]);

  const replaceLastSegmentPoints = useCallback(
    (points: Point3[]) => {
      const current = plannedSegmentsRef.current;
      if (current.length === 0) return false;
      const lastIndex = current.length - 1;
      const next = current.map((seg, idx) =>
        idx === lastIndex ? { ...seg, points, kind: "planned" as const } : seg,
      );
      setPlannedSegmentsValue(next);
      return true;
    },
    [setPlannedSegmentsValue],
  );

  const upsertCruiseLegPoints = useCallback(
    (legIndex: number, points: Point3[]): boolean => {
      const current = plannedSegmentsRef.current;
      const idx = current.findIndex((seg) => seg.goalIndex === legIndex);
      if (idx < 0) return false;
      const next = current.map((seg, i) =>
        i === idx ? { ...seg, points, kind: "planned" as const } : seg,
      );
      setPlannedSegmentsValue(next);
      return true;
    },
    [setPlannedSegmentsValue],
  );

  const upsertSegmentByGoalIndex = useCallback(
    (goalIndex: number, segment: PlannedSegment) => {
      const current = plannedSegmentsRef.current;
      // Replace if the goal already has a segment, otherwise append.
      const existing = current.findIndex((seg) => seg.goalIndex === goalIndex);
      if (existing >= 0) {
        const next = current.map((seg, idx) => (idx === existing ? segment : seg));
        setPlannedSegmentsValue(next);
        return;
      }
      // Drop any segments that came *after* this one (stale chain) before
      // appending — happens when a CRUD action shrinks the goal list.
      const trimmed = current.filter((seg) => seg.goalIndex < goalIndex);
      setPlannedSegmentsValue([...trimmed, segment]);
    },
    [setPlannedSegmentsValue],
  );

  const publishPlanRequest = useCallback(
    (goal: Point3, baseRoute: Point3[], goalIndex: number) => {
      const currentMode = modeRef.current;
      const is2dMode = currentMode === "2d";
      const planningBaseRoute = projectPointsForMode(baseRoute, currentMode);
      const routeEnd = planningBaseRoute.length >= 2 ? planningBaseRoute[planningBaseRoute.length - 1] : null;
      const startCandidate = routeEnd ?? startPointRef.current;
      if (!startCandidate) {
        // No start point set yet — refuse to silently fall back to (0,0,0) or robot pose.
        setBridgeMessage("请先设置起点：先点 [起点] 再选择目标");
        setPickStatus({ kind: "miss", text: "请先设置起点" });
        return;
      }
      const nextStart = sanitizePoint(startCandidate, is2dMode);
      const nextGoal = sanitizePoint(goal, is2dMode);
      pendingPlanRef.current = {
        append: routeEnd !== null,
        baseRoute: cloneRoute(planningBaseRoute),
        goalIndex,
      };
      bridgeClientRef.current?.publishStart(nextStart, currentMode);
      bridgeClientRef.current?.publishGoal(nextGoal, currentMode);
      setBridgeMessage(
        `start_goal_published leg=${goalIndex + 1} start=${formatCoordinate(nextStart.x)}, ${formatCoordinate(nextStart.y)}, ${formatCoordinate(nextStart.z)} goal=${formatCoordinate(nextGoal.x)}, ${formatCoordinate(nextGoal.y)}, ${formatCoordinate(nextGoal.z)}`,
      );
    },
    [setStartPointValue],
  );

  const publishStart = useCallback(
    (point: Point3) => {
      if (bridgeStatus !== "connected") {
        setBridgeMessage("请先连接 ROSBridge；起点只有发送到 bridge 后才有效");
        setPickStatus({
          kind: "miss",
          text: "请先连接 ROSBridge",
        });
        return null;
      }
      const nextStart = sanitizePoint(point, mode === "2d");
      setStartPointValue(nextStart);
      setGoals([]);
      clearPlannedSegments();
      setPlanStatusKind("idle");
      setPlayback("paused");
      setProgress(0);
      lastFrameRef.current = null;
      pendingPlanRef.current = null;
      queuedGoalsRef.current = [];
      cruisingRef.current = null;
      bridgeClientRef.current?.publishStart(nextStart, mode);
      setBridgeMessage(
        `start_published ${formatCoordinate(nextStart.x)}, ${formatCoordinate(nextStart.y)}, ${formatCoordinate(nextStart.z)}`,
      );
      return nextStart;
    },
    [bridgeStatus, clearPlannedSegments, mode, setStartPointValue],
  );

  const appendGoal = useCallback(
    (point: Point3): boolean => {
      if (!startPointRef.current) {
        setBridgeMessage("请先设置起点：选择 [起点] 标签并在场景上点一下，或填入 X/Y/Z 后按 [设置起点]");
        setPickStatus({ kind: "miss", text: "请先设置起点" });
        setPointRole("start");
        return false;
      }
      // Multi-goal authoring is local-only — we collect every goal in `goals`
      // but DO NOT call publishStart/publishGoal yet, AND we do NOT draw any
      // preview connecting line. The bridge only ever sees one (start, goal)
      // pair at a time, dispatched leg-by-leg by the cruise watcher when the
      // operator hits 启动. Until then the only visible artifacts are the
      // goal markers themselves — no straight-line interpolation is drawn so
      // the operator never confuses queued waypoints with a planned path.
      const nextGoal = sanitizePoint(point, mode === "2d");
      const goalIndex = goals.length;
      setGoals((current) => [...current, nextGoal]);
      setPlanStatusKind("idle");
      setPlayback("paused");
      lastFrameRef.current = null;
      setBridgeMessage(
        `goal_queued index=${goalIndex + 1} goal=${formatCoordinate(nextGoal.x)}, ${formatCoordinate(nextGoal.y)}, ${formatCoordinate(nextGoal.z)}（启动后才会逐段下发到 bridge 进行真实路径规划）`,
      );
      return true;
    },
    [goals.length, mode],
  );

  const handleScenePoint = useCallback(
    (point: Point3) => {
      if (displayedOccupancyCells.length === 0) {
        setPickStatus({
          kind: "miss",
          text: `等待 ${expectedMapTopicName}`,
        });
        setBridgeMessage(`等待 ROS 发布 ${expectedMapTopicName}`);
        return;
      }
      const selected = sanitizePoint(point, mode === "2d");
      setForm({
        x: formatCoordinate(selected.x),
        y: formatCoordinate(selected.y),
        z: formatCoordinate(selected.z),
      });
      // First click always becomes the start when none is set yet, regardless of role,
      // so the operator never has to manually toggle the role for the very first pick.
      if (pointRole === "start" || !startPointRef.current) {
        const nextStart = publishStart(point);
        if (!nextStart) {
          return;
        }
        setPickStatus({
          kind: "hit",
          text: `已设置起点 ${formatCoordinate(nextStart.x)}, ${formatCoordinate(nextStart.y)}, ${formatCoordinate(nextStart.z)}`,
        });
        setPointRole("goal");
        return;
      }
      if (!appendGoal(point)) {
        return;
      }
      setPickStatus({
        kind: "hit",
        text: `已选择目标 ${formatCoordinate(selected.x)}, ${formatCoordinate(selected.y)}, ${formatCoordinate(selected.z)}`,
      });
    },
    [appendGoal, displayedOccupancyCells.length, expectedMapTopicName, mode, pointRole, publishStart],
  );

  const handleSceneMiss = useCallback(() => {
    if (displayedOccupancyCells.length === 0) {
      setPickStatus({
        kind: "miss",
        text: `等待 ${expectedMapTopicName}`,
      });
      return;
    }
    setPickStatus({
      kind: "miss",
      text: "未命中地图范围：请点击 OctoMap 地图内的位置",
    });
  }, [displayedOccupancyCells.length, expectedMapTopicName]);

  const handleSubmit = (event: FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    const point = {
      x: Number(form.x),
      y: Number(form.y),
      z: Number(form.z),
    };
    if (editingGoalIndex !== null) {
      // Patch in place + replan from start through the updated goal list,
      // reusing the existing pendingPlan/queue chain.
      const sanitized = sanitizePoint(point, mode === "2d");
      const nextGoals = goals.map((existing, index) => (index === editingGoalIndex ? sanitized : existing));
      setGoals(nextGoals);
      setEditingGoalIndex(null);
      setForm(initialForm);
      setBridgeMessage(
        `goal_updated index=${editingGoalIndex + 1} goal=${formatCoordinate(sanitized.x)}, ${formatCoordinate(sanitized.y)}, ${formatCoordinate(sanitized.z)}`,
      );
      replanFromGoals(nextGoals);
      return;
    }
    if (pointRole === "start" || !startPointRef.current) {
      const nextStart = publishStart(point);
      if (!nextStart) {
        return;
      }
      setPickStatus({
        kind: "hit",
        text: `已设置起点 ${formatCoordinate(nextStart.x)}, ${formatCoordinate(nextStart.y)}, ${formatCoordinate(nextStart.z)}`,
      });
      setPointRole("goal");
      return;
    }
    appendGoal(point);
  };

  const handlePlay = () => {
    if (!startPointRef.current) {
      setBridgeMessage("先设置起点");
      return;
    }
    if (goals.length === 0) {
      setBridgeMessage("先添加至少一个目标点");
      return;
    }
    if (bridgeStatus !== "connected") {
      setBridgeMessage("请先连接 ROSBridge");
      return;
    }
    // Leg-by-leg dispatch: arm cruise on leg 0, publish (start, goal[0]) once.
    // Bridge will plan, return /nav3d/trajectory for this single segment, and
    // store it as plannedSegments[0]. The cruise watcher (progress effect)
    // detects arrival at goal[0] and dispatches (goal[0], goal[1]) — and so
    // on through the full goals[] queue.
    cruisingRef.current = { legIndex: 0, dispatchedFor: 0 };
    pendingPlanRef.current = null;
    queuedGoalsRef.current = [];
    // Keep the placeholder chain in place — the bridge will replace each leg
    // in-place via upsertSegmentByGoalIndex(kind: "planned"), so the operator
    // sees a smooth grey-dashed → bright-green transition per leg instead of
    // a flicker to empty.
    setProgress(0);
    lastFrameRef.current = null;
    publishPlanRequest(goals[0], [], 0);
    cruiseStartedAtRef.current = Date.now();
    setBridgeMessage(
      `cruise_start leg=1/${goals.length} goal=${formatCoordinate(goals[0].x)}, ${formatCoordinate(goals[0].y)}, ${formatCoordinate(goals[0].z)}（机器人到达后自动转下一段）`,
    );
    manualResetRef.current = false;
    setPlayback("playing");
  };

  const handlePause = () => {
    setPlayback((current) => (current === "playing" ? "paused" : current));
  };

  const handleReset = () => {
    manualResetRef.current = true;
    cruisingRef.current = null;
    setPlayback("idle");
    setProgress(0);
    lastFrameRef.current = null;
  };

  const handleClearGoals = () => {
    setStartPointValue(null);
    setGoals([]);
    clearPlannedSegments();
    setPlanStatusKind("idle");
    setPlayback("idle");
    setProgress(0);
    lastFrameRef.current = null;
    pendingPlanRef.current = null;
    queuedGoalsRef.current = [];
    cruisingRef.current = null;
    setEditingGoalIndex(null);
    setForm(initialForm);
  };

  const handleUndoLastGoal = () => {
    if (goals.length === 0) {
      return;
    }
    const removedIndex = goals.length - 1;
    setGoals((current) => current.slice(0, -1));
    setPlannedSegmentsValue(plannedSegmentsRef.current.filter((seg) => seg.goalIndex !== removedIndex));
    setPlanStatusKind("idle");
    setPlayback("paused");
    setProgress(0);
    lastFrameRef.current = null;
    pendingPlanRef.current = null;
    queuedGoalsRef.current = [];
    cruisingRef.current = null;
    setEditingGoalIndex(null);
    setBridgeMessage(`undo_last_goal remaining=${goals.length - 1}`);
  };

  const replanFromGoals = useCallback(
    (nextGoals: Point3[]) => {
      // Send the first leg from the start point and queue the rest. The
      // /nav3d/trajectory callback drains the queue one segment at a time so
      // the publish-sequence shape (start+goal per leg) stays linear.
      if (!startPointRef.current || nextGoals.length === 0) {
        clearPlannedSegments();
        setPlanStatusKind("idle");
        pendingPlanRef.current = null;
        queuedGoalsRef.current = [];
        cruisingRef.current = null;
        return;
      }
      if (bridgeStatus !== "connected") {
        clearPlannedSegments();
        setPlanStatusKind("idle");
        pendingPlanRef.current = null;
        queuedGoalsRef.current = [];
        cruisingRef.current = null;
        setBridgeMessage("请先连接 ROSBridge；目标已更新但未重新规划");
        return;
      }
      // No placeholder chain — until the bridge returns a real trajectory we
      // do not draw any preview line. Goals show as markers only.
      clearPlannedSegments();
      setPlanStatusKind("idle");
      setProgress(0);
      lastFrameRef.current = null;
      pendingPlanRef.current = null;
      queuedGoalsRef.current = nextGoals.slice(1);
      cruisingRef.current = null;
      publishPlanRequest(nextGoals[0], [], 0);
    },
    [bridgeStatus, clearPlannedSegments, publishPlanRequest],
  );

  const handleEditGoal = useCallback(
    (index: number) => {
      const goal = goals[index];
      if (!goal) return;
      setEditingGoalIndex(index);
      setPointRole("goal");
      setForm({
        x: formatCoordinate(goal.x),
        y: formatCoordinate(goal.y),
        z: formatCoordinate(goal.z),
      });
      setBridgeMessage(`editing_goal index=${index + 1}（修改 X/Y/Z 后按 [更新目标]）`);
    },
    [goals],
  );

  const handleCancelEdit = useCallback(() => {
    setEditingGoalIndex(null);
    setForm(initialForm);
    setBridgeMessage("已取消编辑");
  }, []);

  const handleDeleteGoal = useCallback(
    (index: number) => {
      if (index < 0 || index >= goals.length) return;
      const cruise = cruisingRef.current;
      if (cruise && index < cruise.legIndex) {
        // Cannot rewrite history — already-traversed leg.
        setBridgeMessage(`无法删除已完成的目标 ${index + 1}（机器人已经离开该段）`);
        return;
      }
      const nextGoals = goals.filter((_, i) => i !== index);
      setGoals(nextGoals);
      if (editingGoalIndex === index) {
        setEditingGoalIndex(null);
        setForm(initialForm);
      } else if (editingGoalIndex !== null && editingGoalIndex > index) {
        setEditingGoalIndex(editingGoalIndex - 1);
      }
      setBridgeMessage(`delete_goal index=${index + 1} remaining=${nextGoals.length}`);
      replanFromGoals(nextGoals);
    },
    [editingGoalIndex, goals, replanFromGoals],
  );

  // Publish max-speed (std_msgs/Float32) on slider change, debounced so a drag
  // does not hammer the bridge with 60 Hz writes. The slider only governs
  // /cmd_vel saturation downstream — it never animates the browser-side robot.
  const queueSpeedPublish = useCallback((value: number) => {
    if (speedPublishHandleRef.current !== null) {
      clearTimeout(speedPublishHandleRef.current);
    }
    speedPublishHandleRef.current = setTimeout(() => {
      bridgeClientRef.current?.publishMaxSpeed(value);
      speedPublishHandleRef.current = null;
    }, 120);
  }, []);

  const handleSpeedChange = useCallback(
    (event: React.ChangeEvent<HTMLInputElement>) => {
      const next = Number(event.target.value);
      if (!Number.isFinite(next)) return;
      setSpeed(next);
      queueSpeedPublish(next);
    },
    [queueSpeedPublish],
  );

  // Sync the current slider value to bridge once a fresh connection comes up
  // so the controller does not run with a stale internal default.
  useEffect(() => {
    if (bridgeStatus !== "connected") return;
    bridgeClientRef.current?.publishMaxSpeed(speed);
  }, [bridgeStatus, speed]);

  useEffect(() => {
    return () => {
      if (speedPublishHandleRef.current !== null) {
        clearTimeout(speedPublishHandleRef.current);
        speedPublishHandleRef.current = null;
      }
    };
  }, []);

  const handleBridgeStatus = useCallback((status: RosbridgeStatus) => {
    setBridgeStatus(status);
    setBridgeMessage(bridgeStatusLabel(status));
  }, []);

  const handleConnectBridge = useCallback(() => {
    bridgeClientRef.current?.disconnect();
    const client = createRosbridgeClient({
      url: bridgeUrl,
      onStatus: handleBridgeStatus,
      onTextStatus: (status) => {
        const kind = parsePlanStatusKind(status);
        if (kind) {
          setPlanStatusKind(kind);
          if (kind === "failed") {
            pendingPlanRef.current = null;
            queuedGoalsRef.current = [];
            cruisingRef.current = null;
          }
        }
        setBridgeMessage(status);
      },
      onTrajectory: (points) => {
        const segment = points.map((point) => sanitizePoint(point, modeRef.current === "2d"));
        const pending = pendingPlanRef.current;
        if (segment.length === 0) {
          if (pending) {
            // Bridge declined this leg — drop pending + queue, keep prior segments.
            pendingPlanRef.current = null;
            queuedGoalsRef.current = [];
          }
          setPlayback((current) => (current === "playing" ? "playing" : "paused"));
          lastFrameRef.current = null;
          return;
        }
        if (pending) {
          if (routeContainsSegment(pending.baseRoute, segment)) {
            return;
          }
          const goalEnd = sanitizePoint(segment[segment.length - 1], modeRef.current === "2d");
          const startCandidate =
            pending.baseRoute.length > 0
              ? pending.baseRoute[pending.baseRoute.length - 1]
              : (startPointRef.current ?? segment[0]);
          upsertSegmentByGoalIndex(pending.goalIndex, {
            goalIndex: pending.goalIndex,
            start: sanitizePoint(startCandidate, modeRef.current === "2d"),
            goal: goalEnd,
            points: segment,
            kind: "planned",
          });
          pendingPlanRef.current = null;
          setPlayback((current) => (current === "complete" ? "paused" : current));
          lastFrameRef.current = null;
          const nextQueuedGoal = queuedGoalsRef.current.shift();
          if (nextQueuedGoal) {
            const nextGoalIndex = pending.goalIndex + 1;
            publishPlanRequest(nextQueuedGoal, plannedTrajectoryRef.current, nextGoalIndex);
          }
          return;
        }
        // No pending request — this is an unauthored bridge replan (safety
        // replan, latched-topic republish, or external trigger).
        const cruise = cruisingRef.current;
        if (cruise) {
          // Update only the active leg so earlier legs in the multi-goal
          // chain are preserved.
          upsertCruiseLegPoints(cruise.legIndex, segment);
          lastFrameRef.current = null;
          return;
        }
        // Not cruising. If we already have multiple authored segments, an
        // unsolicited single-segment payload would otherwise overwrite the
        // last leg — which manifests as the operator-visible "trajectories
        // collapse to one straight line after adding more goals". Refuse the
        // overwrite and keep the prior segments intact. This includes the
        // common case where bridge republishes /nav3d/trajectory because
        // it's a Transient Local / latched topic.
        const existingSegments = plannedSegmentsRef.current;
        if (existingSegments.length >= 2) {
          lastFrameRef.current = null;
          return;
        }
        const replaced = replaceLastSegmentPoints(segment);
        if (!replaced) {
          // Nothing to replace yet — accept this as the first segment of a
          // single-goal flow (matches the e2e mock that publishes a trajectory
          // immediately on the first goal publish).
          const goalEnd = sanitizePoint(segment[segment.length - 1], modeRef.current === "2d");
          const startCandidate = startPointRef.current ?? segment[0];
          upsertSegmentByGoalIndex(0, {
            goalIndex: 0,
            start: sanitizePoint(startCandidate, modeRef.current === "2d"),
            goal: goalEnd,
            points: segment,
            kind: "planned",
          });
        }
        lastFrameRef.current = null;
      },
      onCurrentPose: (point) => {
        setLiveRobotPosition(sanitizePoint(point, false));
      },
      onOccupancyGrid: (cells) => {
        setBridgeGridCells(cells);
      },
      onOccupancyVoxels: (cells) => {
        // v3.8 perf: bridge republishes the FULL voxel set (62k+ cells for
        // building2_9) every 1s + on every local_pointcloud_updated. Without
        // stabilization the reference flips on every callback → all
        // downstream useMemo / useEffect chains in NavigationScene re-fire,
        // tearing down and rebuilding the merged-edges geometry every
        // second. We compare a cheap fingerprint (counts split by layer +
        // first / last cell coord) and only commit a new array when the
        // map content actually changed. This costs O(N) once per callback
        // but saves O(N) GPU upload work on the no-op path.
        setBridgeVoxelCells((prev) => {
          if (cellsLikelyUnchanged(prev, cells)) {
            return prev;
          }
          return cells;
        });
      },
    });
    bridgeClientRef.current = client;
    client.connect();
  }, [
    bridgeUrl,
    handleBridgeStatus,
    publishPlanRequest,
    replaceLastSegmentPoints,
    upsertCruiseLegPoints,
    upsertSegmentByGoalIndex,
  ]);

  const handleStartBridge = useCallback(async () => {
    const status = await requestBridgeLaunch();
    setBridgeLaunchStatus(status);
    if (status?.message) {
      setBridgeMessage(status.message);
    }
    handleConnectBridge();
  }, [handleConnectBridge]);

  const handleDisconnectBridge = useCallback(() => {
    bridgeClientRef.current?.disconnect();
    bridgeClientRef.current = null;
    handleBridgeStatus("closed");
    setLiveRobotPosition(null);
    clearPlannedSegments();
    setPlanStatusKind("idle");
    setBridgeGridCells([]);
    setBridgeVoxelCells([]);
    pendingPlanRef.current = null;
    queuedGoalsRef.current = [];
    cruisingRef.current = null;
  }, [clearPlannedSegments, handleBridgeStatus]);

  const handleApplyMapPath = useCallback(() => {
    const trimmed = pcdPath.trim();
    if (trimmed.length === 0) {
      return;
    }
    // Switching maps invalidates the current plan; clear points and trajectory.
    setStartPointValue(null);
    setGoals([]);
    clearPlannedSegments();
    setPlanStatusKind("idle");
    setPlayback("idle");
    setProgress(0);
    lastFrameRef.current = null;
    pendingPlanRef.current = null;
    queuedGoalsRef.current = [];
    cruisingRef.current = null;
    if (bridgeStatus !== "connected") {
      setBridgeMessage(`map_pending ${trimmed}（连接 ROSBridge 后自动加载）`);
      return;
    }
    bridgeClientRef.current?.publishPcdPath(trimmed);
    setBridgeMessage(`map_load_requested ${trimmed}`);
  }, [bridgeStatus, clearPlannedSegments, pcdPath, setStartPointValue]);

  const handlePublishPcdPath = handleApplyMapPath;

  const handlePickMapPreset = useCallback((presetId: string) => {
    const preset = findMapPreset(presetId);
    if (!preset) {
      return;
    }
    setSelectedMapId(presetId);
    setPcdPath(preset.path);
  }, []);

  useEffect(() => {
    if (mode === "2d") {
      setForm((current) => ({ ...current, z: "0.00" }));
      pendingPlanRef.current = null;
      queuedGoalsRef.current = [];
    }
  }, [mode]);

  useEffect(() => {
    return () => {
      bridgeClientRef.current?.disconnect();
    };
  }, []);

  useEffect(() => {
    if (bridgeStatus === "connecting" || bridgeStatus === "connected") {
      return;
    }
    void handleStartBridge();
  }, [bridgeStatus, handleStartBridge]);

  // Drive `progress` from the live robot pose by projecting it orthogonally
  // onto the planned polyline. When the bridge has not yet sent a pose, leave
  // progress at its previous value (no auto-advance, no jump back to 0).
  useEffect(() => {
    if (manualResetRef.current) {
      // Operator explicitly reset; keep progress=0 until they hit 启动 again.
      return;
    }
    if (!liveRobotPosition || displayedRoutePoints.length < 2 || pathLength === 0) {
      return;
    }
    const projected = projectPointToPolyline(displayedRoutePoints, liveRobotPosition);
    if (projected === null) {
      return;
    }
    setProgress((current) => (Math.abs(current - projected) < 1e-4 ? current : projected));
    if (projected >= 1 - 1e-3) {
      setPlayback("complete");
    } else if (projected > 0) {
      setPlayback((current) => (current === "idle" || current === "complete" ? "playing" : current));
    }
  }, [displayedRoutePoints, liveRobotPosition, pathLength]);

  // Cruise dispatcher — once 启动 has armed cruisingRef, watch `progress` and
  // publish (start = goal[i], goal = goal[i+1]) as soon as the robot crosses
  // the end of leg i. Guarded by `dispatchedFor` so this fires exactly once
  // per leg-boundary even if `progress` re-projects on each pose update.
  useEffect(() => {
    const cruise = cruisingRef.current;
    if (!cruise) return;
    if (bridgeStatus !== "connected") return;
    if (plannedSegments.length === 0) return;
    if (cruise.legIndex >= goals.length - 1) return;
    if (cruise.dispatchedFor > cruise.legIndex) return;
    const legEndProgress = progressAtSegmentEnd(plannedSegments, cruise.legIndex);
    // 0.96 hysteresis — robot is "at the goal" when within ~4% of total path
    // before the leg endpoint, accounting for tracking error / discretization.
    if (progress < legEndProgress - 0.04) return;
    const nextLeg = cruise.legIndex + 1;
    const nextGoal = goals[nextLeg];
    if (!nextGoal) return;
    cruisingRef.current = { legIndex: nextLeg, dispatchedFor: nextLeg };
    pendingPlanRef.current = null;
    publishPlanRequest(nextGoal, plannedTrajectoryRef.current, nextLeg);
    setBridgeMessage(
      `cruise_advance leg=${nextLeg + 1}/${goals.length} goal=${formatCoordinate(nextGoal.x)}, ${formatCoordinate(nextGoal.y)}, ${formatCoordinate(nextGoal.z)}`,
    );
  }, [bridgeStatus, goals, plannedSegments, progress, publishPlanRequest]);

  // No-odom watchdog — if 启动 was pressed but /nav3d/current_pose never
  // arrives within 8 s, surface a guidance message. Without odom the cruise
  // dispatcher cannot advance because progress stays at 0, and the operator
  // otherwise gets no signal that anything is wrong.
  useEffect(() => {
    if (cruiseWatchdogTimerRef.current !== null) {
      clearTimeout(cruiseWatchdogTimerRef.current);
      cruiseWatchdogTimerRef.current = null;
    }
    if (playback !== "playing") return;
    if (!cruisingRef.current) return;
    if (liveRobotPosition) return;
    cruiseWatchdogTimerRef.current = setTimeout(() => {
      // Re-check at fire time — operator may have already paused/reset/etc.
      if (!cruisingRef.current) return;
      if (liveRobotPosition) return;
      setBridgeMessage(
        "未收到 /nav3d/current_pose：启动 cmd_vel_pose_sim 或真实控制器后才会推进段切换（当前仍在原地）",
      );
    }, 8000);
    return () => {
      if (cruiseWatchdogTimerRef.current !== null) {
        clearTimeout(cruiseWatchdogTimerRef.current);
        cruiseWatchdogTimerRef.current = null;
      }
    };
  }, [playback, liveRobotPosition]);

  const canPlay = displayedRoutePoints.length >= 2 && pathLength > 0;
  // 启动 should be available as soon as the operator has a start point AND
  // at least one queued goal — even before the bridge has returned any
  // trajectory. The web side draws no preview line for queued goals, so
  // requiring `canPlay` (which depends on routePoints.length >= 2) would
  // leave the button permanently disabled and there would be no way to
  // dispatch the first leg to bridge.
  const canStartControl =
    bridgeStatus === "connected" && startPoint !== null && goals.length > 0;
  const canSendGoal = bridgeStatus === "connected";
  const trajectorySource = trajectorySourceLabel(displayedRoutePoints, bridgeStatus);
  // v3.8 fix: tighten the emergency-stop detection regex.
  //
  // The bridge emits both genuine `safety_emergency_stop` /
  // `safety_replan_emergency_stop` (real braking events) AND the preventive
  // `safety_replan_attempt_before_emergency_stop` status that fires whenever
  // the planner notices a future collision and tries a recovery replan
  // before braking. Matching `emergency_stop` substring lit up the red
  // banner constantly during normal travel-mode runs (every replan attempt
  // around an obstacle pulses the banner), making the UI look "broken" /
  // "stuck" / "always emergency-stopped" when it's actually doing the right
  // thing. Only trigger the overlay when the bridge has actually published
  // a stopping status; preventive replan attempts get logged in the status
  // strip but don't escalate to the full-screen banner.
  const isEmergencyStop = /\b(safety_emergency_stop|safety_replan_emergency_stop|navigate_emergency_stop|plan_emergency_stop|map_reload_stop_tracking)\b/.test(
    bridgeMessage,
  );
  const progressMeters = pathLength * progress;
  const nextGoal =
    displayedRoutePoints.length === 0
      ? null
      : displayedRoutePoints[
          Math.min(displayedRoutePoints.length - 1, Math.ceil(progress * Math.max(0, displayedRoutePoints.length - 1)))
        ];
  const mapSpan = boundsSpan(mapBounds);
  const mapResolution =
    displayedOccupancyCells[0]?.size ?? 0;
  const layerCounts = useMemo(() => {
    return {
      global: bridgeVoxelCells.filter((cell) => cell.layer !== "local").length,
      local: bridgeVoxelCells.filter((cell) => cell.layer === "local").length,
      grid: bridgeGridCells.length,
    };
  }, [bridgeGridCells.length, bridgeVoxelCells.length]);
  const pathSplit = useMemo(
    () => splitPolylineAtProgress(displayedRoutePoints, progress),
    [displayedRoutePoints, progress],
  );
  const remainingLength = useMemo(() => getPathLength(pathSplit.remaining), [pathSplit.remaining]);
  const localLength = useMemo(() => getPathLength(pathSplit.local), [pathSplit.local]);
  const traveledLength = Math.max(0, pathLength - remainingLength);

  return (
    <main className="app-shell">
      <nav className="rail" aria-label="主控制栏">
        <div className="rail-brand" title="3D-Octomap-Ego-Nav">N3</div>
        <button
          type="button"
          className={`rail-orb ${bridgeStatus}`}
          title={`ROS：${bridgeStatusLabel(bridgeStatus)}（点击检查/重连）`}
          aria-label={`ROSBridge ${bridgeStatusLabel(bridgeStatus)}`}
          onClick={handleStartBridge}
        >
          <span className="orb-dot" aria-hidden="true" />
        </button>
        <div className="rail-toggle" role="group" aria-label="导航模式">
          <button
            type="button"
            aria-pressed={mode === "2d"}
            className={mode === "2d" ? "active" : ""}
            onClick={() => setMode("2d")}
          >
            2D
          </button>
          <button
            type="button"
            aria-pressed={mode === "3d"}
            className={mode === "3d" ? "active" : ""}
            onClick={() => setMode("3d")}
          >
            3D
          </button>
        </div>
        <div className="rail-spacer" />
        <button
          type="button"
          className="rail-icon-btn"
          title="清空当前规划"
          aria-label="清空当前规划"
          onClick={handleClearGoals}
          disabled={goals.length === 0 && !startPoint}
        >
          ⌫
        </button>
      </nav>

      <section className="viewport" aria-label="三维导航场景">
        <NavigationScene
          mode={mode}
          startPoint={displayedStartPoint}
          goals={displayedGoals}
          routePoints={displayedRoutePoints}
          plannedSegments={plannedSegments}
          progress={progress}
          occupancyCells={displayedOccupancyCells}
          mapBounds={mapBounds}
          robotPosition={displayedRobotPosition}
          hasLivePose={hasLivePose}
          onScenePoint={handleScenePoint}
          onSceneMiss={handleSceneMiss}
        />

        <div className="glass hud-top" role="group" aria-label="场景与运行控制">
          <div className="hud-scene-title">
            <p>{mapSourceLabel(mapSource, mode)}</p>
            <h2>{mode === "3d" ? "三维 OctoMap 场景" : "二维栅格地图"}</h2>
          </div>
          <span
            className={`plan-chip ${planStatusKind}`}
            title="规划结果"
          >
            {planStatusKind === "partial"
              ? "部分"
              : planStatusKind === "failed"
                ? "失败"
                : planStatusKind === "full"
                  ? "完整"
                  : "待规划"}
          </span>
          <div className="hud-run">
            <button
              type="button"
              className="command-button command-primary"
              onClick={handlePlay}
              disabled={!canStartControl || playback === "playing"}
              title="把当前最新目标重新发布到 /nav3d/goal，让下游 controller / 仿真接管 /nav3d/trajectory 跟踪；浏览器侧只做视觉指示"
            >
              启动
            </button>
            <button
              type="button"
              className="command-button"
              onClick={handlePause}
              disabled={playback !== "playing"}
            >
              暂停
            </button>
            <button type="button" className="command-button" onClick={handleReset}>
              复位
            </button>
          </div>
        </div>

        <div className="glass hud-minimap" aria-label="OctoMap 投影">
          <p className="hud-title">OctoMap 投影 · {playbackLabel(playback)}</p>
          <MiniMap
            cells={displayedOccupancyCells}
            startPoint={displayedStartPoint}
            goals={displayedGoals}
            routePoints={displayedRoutePoints}
            robotPosition={displayedRobotPosition}
            bounds={mapBounds}
          />
        </div>

        <div className="glass hud-legend">
          <p className={`pick-status pick-${pickStatus.kind}`}>{pickStatus.text}</p>
          {hasOnlyNonDisplayedMap ? (
            <p className="inline-warning">
              当前为 {mode.toUpperCase()} 模式，已收到另一种地图 topic，正在等待 {expectedMapTopicName}。
            </p>
          ) : null}
          <div className="path-legend" aria-label="路径图例">
            <div className="legend-row">
              <span className="legend-swatch global" aria-hidden="true" />
              <strong>全局路径</strong>
              <span>{remainingLength.toFixed(2)} m</span>
            </div>
            <div className="legend-row">
              <span className="legend-swatch local" aria-hidden="true" />
              <strong>局部路径</strong>
              <span>{localLength.toFixed(2)} m</span>
            </div>
            <div className="legend-row">
              <span className="legend-swatch trail" aria-hidden="true" />
              <strong>已走轨迹</strong>
              <span>{traveledLength.toFixed(2)} m</span>
            </div>
          </div>
          <div className="scene-meta" aria-label="场景统计">
            <span>{displayedOccupancyCells.length} {mode === "3d" ? "体素" : "栅格"}</span>
            <span>{startPoint ? "1 起点" : "0 起点"}</span>
            <span>{goals.length} 目标</span>
            <span>{displayedRoutePoints.length} 规划点</span>
          </div>
        </div>
      </section>

      <aside className="dock" aria-label="机器人控制">
        <div className="dock-head">
          <div>
            <p>NAV3D · COMMAND DECK</p>
            <h1>机器人控制</h1>
          </div>
          <span className="plan-chip">{mode.toUpperCase()}</span>
        </div>

        <div className="dock-statusbar" aria-label="导航状态">
          <span className="status-cell">
            <span>ROS：</span>
            <strong>{bridgeStatusLabel(bridgeStatus)}</strong>
          </span>
          <span className="status-cell">
            <span>Bridge：</span>
            <strong>{bridgeLaunchLabel(bridgeLaunchStatus)}</strong>
          </span>
          <span className="status-cell">起点：{startPoint ? "已设" : "未设"}</span>
          <span className="status-cell">目标：{goals.length}</span>
          <span className="status-cell">
            {mode === "3d" ? "体素" : "栅格"}：{displayedOccupancyCells.length}
          </span>
          <span className="status-cell">路径：{pathLength.toFixed(2)} m</span>
        </div>

        <section className="dock-section connection-section" aria-labelledby="connection-heading">
          <div className="section-heading-row">
            <h2 id="connection-heading">ROSBridge 连接</h2>
            <span className={`bridge-badge bridge-badge-${bridgeStatus}`}>
              {bridgeStatusLabel(bridgeStatus)}
            </span>
          </div>
          <div className="connection-row">
            <input
              type="url"
              aria-label="地址"
              className="connection-url"
              value={bridgeUrl}
              disabled={bridgeStatus === "connected"}
              onChange={(event) => setBridgeUrl(event.target.value)}
            />
            <button
              type="button"
              className="ghost-icon-btn"
              title="检查/重连"
              aria-label="检查/重连"
              onClick={handleStartBridge}
              disabled={bridgeStatus === "connecting"}
            >
              ↻
            </button>
          </div>
          <div className="button-row two">
            <button
              type="button"
              className="command-button command-primary"
              onClick={handleConnectBridge}
              disabled={bridgeStatus === "connecting" || bridgeStatus === "connected"}
            >
              连接
            </button>
            <button
              type="button"
              className="command-button"
              onClick={handleDisconnectBridge}
              disabled={bridgeStatus === "idle" || bridgeStatus === "closed"}
            >
              断开
            </button>
          </div>
          <p
            className={`bridge-status${
              /emergency_stop/.test(bridgeMessage)
                ? " is-emergency"
                : /safety_replan_success|tracking_goal_reached/.test(bridgeMessage)
                  ? " is-success"
                  : ""
            }`}
          >
            {bridgeMessage}
          </p>
          {planStatusKind === "partial" ? (
            <p className="inline-warning">当前轨迹为 partial=true：bridge 已显示可达的 shortened fallback，但未到达请求目标。</p>
          ) : null}
          {planStatusKind === "failed" ? (
            <p className="inline-warning">
              当前目标被 planner 拒绝，未发布新的 /nav3d/trajectory；请检查 bridge 加载的 PCD 与页面地图/坐标是否一致。
            </p>
          ) : null}
        </section>

        <section className="dock-section map-section" aria-labelledby="map-heading">
          <div className="section-heading-row">
            <h2 id="map-heading">地图加载</h2>
            <span className="map-section-meta">PCD 路径</span>
          </div>
          <input
            type="text"
            className="map-path-input"
            aria-label="自定义 PCD 路径"
            value={pcdPath}
            placeholder="/absolute/path/to/map.pcd"
            onChange={(event) => setPcdPath(event.target.value)}
          />
          <div className="map-quick-chips" role="list" aria-label="预设地图">
            {mapPresets.map((preset) => (
              <button
                key={preset.id}
                type="button"
                role="listitem"
                className={`map-chip${selectedMapId === preset.id ? " active" : ""}`}
                title={`${preset.hint} · ${preset.path}`}
                onClick={() => handlePickMapPreset(preset.id)}
              >
                {preset.label}
              </button>
            ))}
          </div>
          <button
            type="button"
            className="full-button command-primary"
            onClick={handlePublishPcdPath}
            disabled={bridgeStatus !== "connected" || pcdPath.trim().length === 0}
          >
            加载 PCD 地图
          </button>
          {bridgeStatus !== "connected" && pcdPath.trim().length > 0 ? (
            <p className="inline-warning">连接 ROSBridge 后该路径会被发布到 /nav3d/load_pcd_path 触发地图重载。</p>
          ) : null}
        </section>

        <section className="dock-section" aria-labelledby="coordinate-heading">
          <h2 id="coordinate-heading">起点 / 目标</h2>
          <div className="segmented-control point-role-control" role="group" aria-label="点位类型">
            <button
              type="button"
              aria-pressed={pointRole === "start"}
              className={pointRole === "start" ? "active" : ""}
              onClick={() => setPointRole("start")}
            >
              起点
            </button>
            <button
              type="button"
              aria-pressed={pointRole === "goal"}
              className={pointRole === "goal" ? "active" : ""}
              onClick={() => setPointRole("goal")}
            >
              目标
            </button>
          </div>
          <form className="coordinate-form" onSubmit={handleSubmit}>
            <label>
              X
              <input
                type="number"
                step="0.1"
                value={form.x}
                onChange={(event) => setForm((current) => ({ ...current, x: event.target.value }))}
              />
            </label>
            <label>
              Y
              <input
                type="number"
                step="0.1"
                value={form.y}
                onChange={(event) => setForm((current) => ({ ...current, y: event.target.value }))}
              />
            </label>
            <label>
              Z
              <input
                type="number"
                step="0.1"
                value={mode === "2d" ? "0.00" : form.z}
                disabled={mode === "2d"}
                onChange={(event) => setForm((current) => ({ ...current, z: event.target.value }))}
              />
            </label>
            <button type="submit">
              {editingGoalIndex !== null ? "更新目标" : pointRole === "start" ? "设置起点" : "添加目标"}
            </button>
          </form>
          <label className="field-row">
            <span>速度</span>
            <input
              type="range"
              min="0.2"
              max="4"
              step="0.1"
              value={speed}
              onChange={handleSpeedChange}
            />
            <strong>{speed.toFixed(1)} m/s</strong>
          </label>
          <p className="field-hint">
            仅作为下游 controller 的最大线速度上限（/nav3d/max_speed），不会驱动浏览器侧的机器人位置；机器人位置由 /nav3d/current_pose 实时更新。
          </p>
          <div className="button-row two">
            <button
              type="button"
              className="command-button"
              onClick={handleUndoLastGoal}
              disabled={goals.length === 0}
              title="撤销刚刚添加的目标"
              aria-label="撤销最后一个目标"
            >
              撤销最后
            </button>
            <button
              type="button"
              className="command-button command-danger"
              onClick={handleClearGoals}
              disabled={goals.length === 0}
            >
              清空目标
            </button>
          </div>
          {!canSendGoal ? (
            <p className="inline-warning">请先连接 ROSBridge；未连接时不会发送起点/目标，也不会生成轨迹。</p>
          ) : null}
        </section>

        <GoalManager
          startPoint={startPoint}
          goals={goals}
          editingGoalIndex={editingGoalIndex}
          onEditGoal={handleEditGoal}
          onDeleteGoal={handleDeleteGoal}
          onCancelEdit={handleCancelEdit}
        />

        <section className="dock-section" aria-labelledby="layer-heading">
          <h2 id="layer-heading">地图层状态</h2>
          <div className="layer-list" aria-label="地图层状态">
            <div className="layer-row layer-global">
              <span className="layer-token" aria-hidden="true" />
              <strong>3D 体素层</strong>
              <span>{layerCounts.global}</span>
            </div>
            <div className="layer-row layer-local">
              <span className="layer-token" aria-hidden="true" />
              <strong>局部障碍层</strong>
              <span>{layerCounts.local}</span>
            </div>
            <div className="layer-row layer-grid">
              <span className="layer-token" aria-hidden="true" />
              <strong>2D 栅格层</strong>
              <span>{layerCounts.grid}</span>
            </div>
          </div>
        </section>

        <section className="dock-section" aria-labelledby="track-heading">
          <h2 id="track-heading">轨迹与定位</h2>
          <dl className="state-grid">
            <dt>运行</dt>
            <dd>{playbackLabel(playback)}</dd>
            <dt>进度</dt>
            <dd>
              {progressMeters.toFixed(2)} / {pathLength.toFixed(2)} m
            </dd>
            <dt>规划点</dt>
            <dd>{displayedRoutePoints.length}</dd>
            <dt>规划</dt>
            <dd>
              {planStatusKind === "partial"
                ? "partial=true"
                : planStatusKind === "failed"
                  ? "failed"
                  : planStatusKind === "full"
                    ? "partial=false"
                    : "待规划"}
            </dd>
            <dt>定位</dt>
            <dd>
              {formatCoordinate(displayedRobotPosition.x)}, {formatCoordinate(displayedRobotPosition.y)},{" "}
              {formatCoordinate(displayedRobotPosition.z)}
            </dd>
            <dt>下一目标</dt>
            <dd>
              {nextGoal
                ? `${formatCoordinate(nextGoal.x)}, ${formatCoordinate(nextGoal.y)}, ${formatCoordinate(nextGoal.z)}`
                : "无"}
            </dd>
            <dt>轨迹来源</dt>
            <dd>{trajectorySource}</dd>
            <dt>分辨率</dt>
            <dd>{mapResolution > 0 ? `${mapResolution.toFixed(2)} m` : "未知"}</dd>
            <dt>尺寸</dt>
            <dd>
              {formatCoordinate(mapSpan.x)} x {formatCoordinate(mapSpan.y)} x {formatCoordinate(mapSpan.z)} m
            </dd>
            <dt>地图边界</dt>
            <dd>
              X {formatCoordinate(mapBounds.min.x)}..{formatCoordinate(mapBounds.max.x)}
            </dd>
            <dt>Y 边界</dt>
            <dd>
              Y {formatCoordinate(mapBounds.min.y)}..{formatCoordinate(mapBounds.max.y)}
            </dd>
            <dt>Z 边界</dt>
            <dd>
              Z {formatCoordinate(mapBounds.min.z)}..{formatCoordinate(mapBounds.max.z)}
            </dd>
          </dl>
        </section>
      </aside>
      {isEmergencyStop ? (
        <div className="emergency-overlay" role="alert" aria-label="紧急停止">
          <div className="emergency-banner">
            <span className="emergency-pulse" aria-hidden="true" />
            <strong>紧急停止</strong>
            <span className="emergency-hint">{bridgeMessage}</span>
          </div>
        </div>
      ) : null}
    </main>
  );
}
