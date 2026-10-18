"use client";

import { useCallback, useEffect, useRef } from "react";
import type { ClientMessage } from "./protocol";

/**
 * How often a held drive is re-sent. Well under the 0.5 s timeout of the web
 * input in config/twist_mux.yaml, so the mux keeps it while a button is held
 * and drops it soon after the browser goes away.
 */
export const DRIVE_RESEND_MS = 100;

/**
 * Hold-to-drive for the base. `hold(vx, vy, wz)` sends at once and keeps
 * re-sending; `release()` stops re-sending and sends one stop. Unmounting
 * releases too.
 */
export function useDrive(send: (msg: ClientMessage) => void) {
  const timer = useRef<ReturnType<typeof setInterval> | null>(null);
  const sendRef = useRef(send);
  sendRef.current = send;

  const clear = () => {
    if (timer.current !== null) {
      clearInterval(timer.current);
      timer.current = null;
    }
  };

  const hold = useCallback((vx: number, vy: number, wz: number) => {
    clear();
    const msg: ClientMessage = { type: "drive", vx, vy, wz };
    sendRef.current(msg);
    timer.current = setInterval(() => sendRef.current(msg), DRIVE_RESEND_MS);
  }, []);

  const release = useCallback(() => {
    const wasHeld = timer.current !== null;
    clear();
    if (wasHeld) sendRef.current({ type: "drive", vx: 0, vy: 0, wz: 0 });
  }, []);

  useEffect(() => release, [release]);

  return { hold, release };
}
