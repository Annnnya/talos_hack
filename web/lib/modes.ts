import { DIRECTIONS, type Direction } from "./joystick";
import type { Mode } from "./protocol";

type Axis = "vx" | "vy" | "wz";
/** Same order as the fractions in `DIRECTIONS`. */
const AXES: readonly Axis[] = ["vx", "vy", "wz"];
const ALL_AXES: ReadonlySet<Axis> = new Set(AXES);

/**
 * Body velocity axes each mode allows, the same rule as `DRIVE_AXES_BY_MODE`
 * in src/talos_servo/modes.py, which the keyboard and the web server use.
 * Climbing cannot turn on the walls; aligning and clamping move the robot by
 * themselves. The server refuses the rest anyway, this only greys buttons out.
 */
const AXES_BY_MODE: Record<Mode, ReadonlySet<Axis>> = {
  free: ALL_AXES,
  climbing: new Set(["vx", "vy"]),
  aligning: new Set(),
  clamping: new Set(),
};

/** Directions that only use `axes`. */
function directionsWithin(axes: ReadonlySet<Axis>): ReadonlySet<Direction> {
  return new Set(
    (Object.keys(DIRECTIONS) as Direction[]).filter((dir) =>
      DIRECTIONS[dir].every((f, i) => f === 0 || axes.has(AXES[i])),
    ),
  );
}

const ALL = directionsWithin(ALL_AXES);
const BY_MODE = Object.fromEntries(
  Object.entries(AXES_BY_MODE).map(([mode, axes]) => [mode, directionsWithin(axes)]),
) as Record<Mode, ReadonlySet<Direction>>;

/**
 * An unknown mode (no mode node) allows everything, like `allowed_axes` in
 * modes.py. Fails open on purpose: without the mode node its stop lock goes
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
