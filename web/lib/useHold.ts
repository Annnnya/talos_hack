"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import type { ClientMessage } from "./protocol";
import { DRIVE_RESEND_MS } from "./useDrive";

export type Sign = 1 | -1;

/**
 * Hold-to-move in one of two directions (hinge open/close, climb up/down).
 * `message(sign)` is sent at once and re-sent while held, since the node
 * stops after its timeout; letting go, hiding the tab or losing focus sends
 * `message(0)` once. `keys` maps desktop keys to a direction.
 */
export function useHold(
  send: (msg: ClientMessage) => void,
  message: (sign: Sign | 0) => ClientMessage,
  enabled: boolean,
  keys: Record<string, Sign> = {},
) {
  const [held, setHeld] = useState<Sign | null>(null);
  const heldRef = useRef<Sign | null>(null);
  const timer = useRef<ReturnType<typeof setInterval> | null>(null);
  const sendRef = useRef(send);
  sendRef.current = send;
  const messageRef = useRef(message);
  messageRef.current = message;

  const release = useCallback(() => {
    if (timer.current !== null) clearInterval(timer.current);
    timer.current = null;
    if (heldRef.current !== null) sendRef.current(messageRef.current(0));
    heldRef.current = null;
    setHeld(null);
  }, []);

  const start = useCallback(
    (sign: Sign) => {
      if (!enabled || heldRef.current === sign) return;
      release();
      heldRef.current = sign;
      setHeld(sign);
      // Rebuilt each time so a speed change applies while held.
      const tick = () => sendRef.current(messageRef.current(sign));
      tick();
      timer.current = setInterval(tick, DRIVE_RESEND_MS);
    },
    [enabled, release],
  );

  const end = useCallback(
    (sign: Sign) => {
      if (heldRef.current === sign) release();
    },
    [release],
  );

  useEffect(() => {
    if (!enabled) release();
  }, [enabled, release]);

  useEffect(() => {
    const onDown = (e: KeyboardEvent) => {
      const tag = (e.target as HTMLElement)?.tagName;
      if (tag === "INPUT" || tag === "TEXTAREA" || e.ctrlKey || e.metaKey || e.altKey) return;
      const sign = keys[e.key.toLowerCase()];
      if (!sign) return;
      e.preventDefault();
      if (!e.repeat) start(sign);
    };
    const onUp = (e: KeyboardEvent) => {
      const sign = keys[e.key.toLowerCase()];
      if (sign) end(sign);
    };
    const onVisibility = () => {
      if (document.hidden) release();
    };
    window.addEventListener("keydown", onDown);
    window.addEventListener("keyup", onUp);
    window.addEventListener("blur", release);
    window.addEventListener("pagehide", release);
    document.addEventListener("visibilitychange", onVisibility);
    return () => {
      window.removeEventListener("keydown", onDown);
      window.removeEventListener("keyup", onUp);
      window.removeEventListener("blur", release);
      window.removeEventListener("pagehide", release);
      document.removeEventListener("visibilitychange", onVisibility);
      release();
    };
  }, [keys, start, end, release]);

  return { held, start, end };
}
