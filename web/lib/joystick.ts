"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import type { ClientMessage } from "./protocol";
import { useDrive } from "./useDrive";

export type Direction = "up" | "down" | "left" | "right" | "ccw" | "cw";

/**
 * (vx, vy, wz) as fractions of the linear and angular speed. Same as `KEYS` in
 * src/talos_servo/teleop.py, so the page and the keyboard controller agree.
 */
export const DIRECTIONS: Record<Direction, readonly [number, number, number]> = {
  up: [1, 0, 0],
  down: [-1, 0, 0],
  left: [0, 1, 0],
  right: [0, -1, 0],
  ccw: [0, 0, 1],
  cw: [0, 0, -1],
};

/** Desktop keys, the same map as `talos-servo drive`. */
export const DIRECTION_KEYS: Record<string, Direction> = {
  w: "up",
  s: "down",
  a: "left",
  d: "right",
  q: "ccw",
  e: "cw",
};

export const KEY_FOR: Record<Direction, string> = Object.fromEntries(
  Object.entries(DIRECTION_KEYS).map(([key, dir]) => [dir, key.toUpperCase()]),
) as Record<Direction, string>;

export function velocity(dir: Direction, linear: number, angular: number) {
  const [fx, fy, fz] = DIRECTIONS[dir];
  return [fx * linear, fy * linear, fz * angular] as const;
}

/** Typing in a field must not drive the robot. */
function typing(e: KeyboardEvent) {
  const tag = (e.target as HTMLElement)?.tagName;
  return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT";
}

/**
 * Hold-to-drive in one direction at a time, from buttons and W/S/A/D/Q/E.
 * `start(dir)` drives until `end(dir)`; ending another direction is a no-op,
 * so letting go of one finger or key never stops a newer one. Becoming
 * disabled, and everything `useDrive` stops on, ends the hold too.
 */
export function useJoystick(
  send: (msg: ClientMessage) => void,
  {
    linear,
    angular,
    enabled,
    allowed,
  }: { linear: number; angular: number; enabled: boolean; allowed: ReadonlySet<Direction> },
) {
  const [active, setActive] = useState<Direction | null>(null);
  const activeRef = useRef<Direction | null>(null);
  const cleared = useCallback(() => {
    activeRef.current = null;
    setActive(null);
  }, []);
  const { hold, release } = useDrive(send, cleared);

  const start = useCallback(
    (dir: Direction) => {
      if (!enabled || !allowed.has(dir)) return;
      activeRef.current = dir;
      setActive(dir);
      hold(...velocity(dir, linear, angular));
    },
    [enabled, allowed, hold, linear, angular],
  );

  const end = useCallback(
    (dir: Direction) => {
      if (activeRef.current === dir) release();
    },
    [release],
  );

  // A mode change can take away the direction being held.
  useEffect(() => {
    const dir = activeRef.current;
    if (!enabled || (dir !== null && !allowed.has(dir))) release();
  }, [enabled, allowed, release]);

  useEffect(() => {
    const onDown = (e: KeyboardEvent) => {
      if (typing(e) || e.ctrlKey || e.metaKey || e.altKey) return;
      const dir = DIRECTION_KEYS[e.key.toLowerCase()];
      if (!dir) return;
      e.preventDefault();
      // Auto-repeat would restart the hold; useDrive already re-sends.
      if (e.repeat || activeRef.current === dir) return;
      start(dir);
    };
    const onUp = (e: KeyboardEvent) => {
      const dir = DIRECTION_KEYS[e.key.toLowerCase()];
      if (dir) end(dir);
    };
    window.addEventListener("keydown", onDown);
    window.addEventListener("keyup", onUp);
    return () => {
      window.removeEventListener("keydown", onDown);
      window.removeEventListener("keyup", onUp);
    };
  }, [start, end]);

  return { active, start, end, release };
}
