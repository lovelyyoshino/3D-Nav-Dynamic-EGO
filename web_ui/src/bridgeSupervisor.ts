import type { Plugin } from "vite";
import { spawn, type ChildProcessWithoutNullStreams } from "node:child_process";
import { createWriteStream, mkdirSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

type BridgeStatus = {
  available: boolean;
  running: boolean;
  pid: number | null;
  command: string[];
  logPath: string;
  message: string;
};

let bridgeProcess: ChildProcessWithoutNullStreams | null = null;
let lastMessage = "bridge supervisor idle";

const repoRoot = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
const logPath = resolve(repoRoot, ".codex", "nav3d-web-bridge.log");

function bridgeCommand(): string[] {
  const pcdPath =
    process.env.NAV3D_WEB_PCD_PATH ??
    resolve(repoRoot, "reference/OctoPlanner3D-ROS2/octomap/pcd_files/building2_9.pcd");
  const rosbridgePort = process.env.NAV3D_WEB_ROSBRIDGE_PORT ?? "9090";
  return [
    "bash",
    "-lc",
    [
      "set -e",
      "source /opt/ros/humble/setup.bash",
      "source install/setup.bash",
      [
        "exec ros2 launch nav3d_ros2_bridge nav3d_bridge.launch.py",
        `pcd_path:=${pcdPath}`,
        `rosbridge_port:=${rosbridgePort}`,
      ].join(" "),
    ].join(" && "),
  ];
}

function currentStatus(): BridgeStatus {
  const allowLaunch = process.env.NAV3D_WEB_ALLOW_LAUNCH === "1";
  return {
    available: allowLaunch,
    running: Boolean(bridgeProcess && !bridgeProcess.killed),
    pid: bridgeProcess?.pid ?? null,
    command: bridgeCommand(),
    logPath,
    message: allowLaunch
      ? lastMessage
      : "Set NAV3D_WEB_ALLOW_LAUNCH=1 before npm run dev to allow web-triggered nav3d_bridge launch.",
  };
}

export function getBridgeSupervisorStatusForTest(): BridgeStatus {
  return currentStatus();
}

function writeJson(response: import("node:http").ServerResponse, status: number, body: BridgeStatus): void {
  response.statusCode = status;
  response.setHeader("content-type", "application/json");
  response.end(JSON.stringify(body));
}

function startBridge(): BridgeStatus {
  if (process.env.NAV3D_WEB_ALLOW_LAUNCH !== "1") {
    return currentStatus();
  }
  if (bridgeProcess && !bridgeProcess.killed) {
    return currentStatus();
  }

  mkdirSync(dirname(logPath), { recursive: true });
  const [command, ...args] = bridgeCommand();
  const log = createWriteStream(logPath, { flags: "a" });
  bridgeProcess = spawn(command, args, {
    cwd: repoRoot,
    env: process.env,
  });
  lastMessage = `started nav3d_bridge launch pid=${bridgeProcess.pid ?? "unknown"}`;
  bridgeProcess.stdout.on("data", (chunk: Buffer) => {
    log.write(chunk);
    process.stdout.write(`[nav3d_bridge] ${chunk}`);
  });
  bridgeProcess.stderr.on("data", (chunk: Buffer) => {
    log.write(chunk);
    process.stderr.write(`[nav3d_bridge] ${chunk}`);
  });
  bridgeProcess.on("exit", (code: number | null, signal: NodeJS.Signals | null) => {
    lastMessage = `nav3d_bridge exited code=${code ?? "null"} signal=${signal ?? "null"}`;
    bridgeProcess = null;
    log.end();
  });

  return currentStatus();
}

export function nav3dBridgeSupervisorPlugin(): Plugin {
  return {
    name: "nav3d-bridge-supervisor",
    configureServer(server) {
      server.middlewares.use("/api/nav3d/bridge/status", (_request, response) => {
        writeJson(response, 200, currentStatus());
      });
      server.middlewares.use("/api/nav3d/bridge/start", (request, response) => {
        if (request.method !== "POST") {
          writeJson(response, 405, currentStatus());
          return;
        }
        writeJson(response, 200, startBridge());
      });
    },
  };
}
