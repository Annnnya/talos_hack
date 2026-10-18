import type { Direction } from "./joystick";
import type { Mode } from "./protocol";

const ALL: ReadonlySet<Direction> = new Set(["up", "down", "left", "right", "ccw", "cw"]);
const NO_TURN: ReadonlySet<Direction> = new Set(["up", "down", "left", "right"]);
const NONE: ReadonlySet<Direction> = new Set();

/**
 * Directions each mode allows, the same as `DRIVE_KEYS_BY_MODE` in
 * src/talos_servo/teleop.py. Climbing cannot turn on the walls; aligning and
 * clamping move the robot by themselves.
 */
const BY_MODE: Record<Mode, ReadonlySet<Direction>> = {
  free: ALL,
  climbing: NO_TURN,
  aligning: NONE,
  clamping: NONE,
};

/**
 * An unknown mode (no mode node) allows everything, like `allowed_keys` in
 * teleop.py. Fails open on purpose: without the mode node its stop lock goes
 * quiet, so twist_mux is locked and nothing drives anyway.
 */
export function allowedDirections(mode: Mode | null): ReadonlySet<Direction> {
  return (mode && BY_MODE[mode]) ?? ALL;
}

export const MODE_LABEL: Record<Mode, string> = {
  free: "Free (XY)",
  aligning: "Aligning",
  clamping: "Clamping",
  climbing: "Climbing (Z)",
};

export const MODE_STYLE: Record<Mode, string> = {
  free: "bg-emerald-500/15 text-emerald-700 dark:text-emerald-400",
  aligning: "bg-amber-500/15 text-amber-700 dark:text-amber-400",
  clamping: "bg-amber-500/15 text-amber-700 dark:text-amber-400",
  climbing: "bg-violet-500/15 text-violet-700 dark:text-violet-300",
};

/** `active_input` values from mux_status. */
export function describeInput(input: string | null): string {
  switch (input) {
    case "web":
      return "web";
    case "keyboard":
      return "keyboard";
    case "locked":
      return "nobody (STOP)";
    case "none":
      return "nobody";
    default:
      return "unknown";
  }
}
