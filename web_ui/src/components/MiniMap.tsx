import { useEffect, useRef } from "react";
import type { OccupancyCell } from "../rosbridge/client";
import type { Point3 } from "../utils/trajectory";
import type { Bounds } from "../utils/bounds";

type MiniMapProps = {
  cells: OccupancyCell[];
  startPoint: Point3 | null;
  goals: Point3[];
  routePoints: Point3[];
  robotPosition: Point3;
  bounds: Bounds;
};

export function MiniMap({ cells, startPoint, goals, routePoints, robotPosition, bounds }: MiniMapProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    const rect = canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio, 2);
    const width = Math.max(1, Math.floor(rect.width * dpr));
    const height = Math.max(1, Math.floor(rect.height * dpr));
    canvas.width = width;
    canvas.height = height;
    const context = canvas.getContext("2d");
    if (!context) {
      return;
    }

    context.fillStyle = "#03070d";
    context.fillRect(0, 0, width, height);
    context.strokeStyle = "rgba(31, 47, 74, 0.55)";
    context.lineWidth = 1;
    for (let x = 0; x <= width; x += width / 6) {
      context.beginPath();
      context.moveTo(x, 0);
      context.lineTo(x, height);
      context.stroke();
    }
    for (let y = 0; y <= height; y += height / 5) {
      context.beginPath();
      context.moveTo(0, y);
      context.lineTo(width, y);
      context.stroke();
    }

    const toCanvas = (point: Point3) => {
      const xSpan = Math.max(1e-6, bounds.max.x - bounds.min.x);
      const ySpan = Math.max(1e-6, bounds.max.y - bounds.min.y);
      return {
        x: ((point.x - bounds.min.x) / xSpan) * width,
        y: height - ((point.y - bounds.min.y) / ySpan) * height,
      };
    };

    // Compute the Z range once so the global voxel ramp matches the 3D scene.
    let zMin = Number.POSITIVE_INFINITY;
    let zMax = Number.NEGATIVE_INFINITY;
    cells.forEach((cell) => {
      if (cell.layer === "grid") {
        return;
      }
      if (cell.z < zMin) zMin = cell.z;
      if (cell.z > zMax) zMax = cell.z;
    });
    const zSpan = Number.isFinite(zMin) && Number.isFinite(zMax) && zMax > zMin ? zMax - zMin : 0;

    cells.forEach((cell) => {
      const point = toCanvas(cell);
      if (cell.layer === "grid" && cell.occupancy === "occupied") {
        // Bright cobalt to match the 3D occupied palette — visible on navy.
        context.fillStyle = "rgba(77, 196, 255, 0.95)";
      } else if (cell.layer === "grid" && cell.occupancy === "unknown") {
        context.fillStyle = "rgba(107, 128, 168, 0.75)";
      } else if (cell.layer === "grid") {
        context.fillStyle = "rgba(230, 237, 247, 0.85)";
      } else if (cell.layer === "local") {
        context.fillStyle = "rgba(255, 122, 210, 0.95)";
      } else if (zSpan > 0) {
        const t = Math.max(0, Math.min(1, (cell.z - zMin) / zSpan));
        const hue = 200 + t * 90;
        const lightness = 56 + t * 10;
        context.fillStyle = `hsla(${hue}, 78%, ${lightness}%, 0.85)`;
      } else {
        context.fillStyle = "rgba(77, 196, 255, 0.78)";
      }
      context.fillRect(point.x, point.y, 1.6 * dpr, 1.6 * dpr);
    });

    if (routePoints.length >= 2) {
      context.strokeStyle = "#4DC4FF";
      context.lineWidth = 1.5 * dpr;
      context.beginPath();
      routePoints.forEach((routePoint, index) => {
        const point = toCanvas(routePoint);
        if (index === 0) {
          context.moveTo(point.x, point.y);
        } else {
          context.lineTo(point.x, point.y);
        }
      });
      context.stroke();
    }

    if (startPoint) {
      const point = toCanvas(startPoint);
      context.strokeStyle = "#2BD9A8";
      context.lineWidth = 2 * dpr;
      context.strokeRect(point.x - 5 * dpr, point.y - 5 * dpr, 10 * dpr, 10 * dpr);
    }

    goals.forEach((goal) => {
      const point = toCanvas(goal);
      context.fillStyle = "#FF5D8F";
      context.fillRect(point.x - 3 * dpr, point.y - 3 * dpr, 6 * dpr, 6 * dpr);
    });

    const robot = toCanvas(robotPosition);
    context.fillStyle = "#B07CFF";
    context.beginPath();
    context.arc(robot.x, robot.y, 4.5 * dpr, 0, Math.PI * 2);
    context.fill();
  }, [bounds, cells, goals, robotPosition, routePoints, startPoint]);

  return <canvas ref={canvasRef} className="minimap-canvas" aria-label="OctoMap 投影" />;
}
