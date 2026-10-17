import type { LastCommand } from "./protocol";

export interface SourceInfo {
  label: string;
  /** Sent from this browser tab. */
  mine: boolean;
  /** Full Tailwind class strings, so the build keeps them. */
  dot: string;
  text: string;
  flash: string;
  ring: string;
}

const YOU = { dot: "bg-blue-500", text: "text-blue-600 dark:text-blue-400", flash: "bg-blue-500/10", ring: "border-blue-500" };
const BROWSER = { dot: "bg-violet-500", text: "text-violet-600 dark:text-violet-400", flash: "bg-violet-500/15", ring: "border-violet-500" };
const CLI = { dot: "bg-teal-500", text: "text-teal-700 dark:text-teal-300", flash: "bg-teal-500/15", ring: "border-teal-500" };
const ROS = { dot: "bg-slate-400", text: "text-slate-600 dark:text-slate-300", flash: "bg-slate-500/15", ring: "border-slate-400" };
const SAFETY = { dot: "bg-rose-500", text: "text-rose-600 dark:text-rose-400", flash: "bg-rose-500/15", ring: "border-rose-500" };

export function describeSource(last: LastCommand, clientId: string | null): SourceInfo {
  switch (last.source) {
    case "web":
      if (last.client === null) return { label: "Safety stop (all browsers left)", mine: false, ...SAFETY };
      if (last.client === clientId) return { label: "You", mine: true, ...YOU };
      return { label: "Another browser", mine: false, ...BROWSER };
    case "cli":
      return { label: "CLI", mine: false, ...CLI };
    case "robot":
      return { label: "Safety stop (command timeout)", mine: false, ...SAFETY };
    default:
      return { label: "ROS node", mine: false, ...ROS };
  }
}

export function formatAge(seconds: number): string {
  if (seconds < 5) return "just now";
  if (seconds < 60) return `${Math.floor(seconds)} s ago`;
  if (seconds < 3600) return `${Math.floor(seconds / 60)} min ago`;
  return `${Math.floor(seconds / 3600)} h ago`;
}

/** Native range thumbs stop half a thumb short of each end; markers must line up with them. */
const THUMB_PX = 16;
export function trackLeft(pct: number): string {
  const clamped = Math.min(100, Math.max(0, pct));
  return `calc(${clamped}% + ${(0.5 - clamped / 100) * THUMB_PX}px)`;
}
