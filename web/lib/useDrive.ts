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
 * re-sending; `release()` stops re-sending and sends one stop. Unmounting,
 * hiding the tab, losing window focus and leaving the page release too.
 * `onReleased` runs whenever a hold ends, whoever ended it.
 */
export function useDrive(send: (msg: ClientMessage) => void, onReleased?: () => void) {
  const timer = useRef<ReturnType<typeof setInterval> | null>(null);
  const sendRef = useRef(send);
  sendRef.current = send;
  const onReleasedRef = useRef(onReleased);
  onReleasedRef.current = onReleased;

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
    if (wasHeld) {
      sendRef.current({ type: "drive", vx: 0, vy: 0, wz: 0 });
      onReleasedRef.current?.();
    }
  }, []);

  // A held button may never see its release: the tab is hidden, the window
  // loses focus or the page goes away. Hidden tabs also slow timers past the
  // mux timeout. Stop on all of those, so no page has to remember to.
  useEffect(() => {
    const onVisibility = () => {
      if (document.hidden) release();
    };
    window.addEventListener("blur", release);
    window.addEventListener("pagehide", release);
    document.addEventListener("visibilitychange", onVisibility);
    return () => {
      window.removeEventListener("blur", release);
      window.removeEventListener("pagehide", release);
      document.removeEventListener("visibilitychange", onVisibility);
      release();
    };
  }, [release]);

  return { hold, release };
}
