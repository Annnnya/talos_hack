"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import type { ClientMessage, HingeState } from "@/lib/protocol";
import { DRIVE_RESEND_MS } from "@/lib/useDrive";

type Sign = 1 | -1;

/** Desktop keys, the same as `talos-servo drive`: R opens, F closes. */
const HINGE_KEYS: Record<string, Sign> = { r: 1, f: -1 };

/**
 * Hold-to-move for the hinge through the expansion node. Sends the rate at
 * once and re-sends it while held (the node stops it after its timeout);
 * letting go, hiding the tab or losing focus sends one stop.
 */
function useHingeHold(send: (msg: ClientMessage) => void, rate: number, enabled: boolean) {
  const [held, setHeld] = useState<Sign | null>(null);
  const heldRef = useRef<Sign | null>(null);
  const timer = useRef<ReturnType<typeof setInterval> | null>(null);
  const sendRef = useRef(send);
  sendRef.current = send;

  const release = useCallback(() => {
    if (timer.current !== null) clearInterval(timer.current);
    timer.current = null;
    if (heldRef.current !== null) sendRef.current({ type: "hinge", rate: 0 });
    heldRef.current = null;
    setHeld(null);
  }, []);

  const start = useCallback(
    (sign: Sign) => {
      if (!enabled || heldRef.current === sign) return;
      release();
      const msg: ClientMessage = { type: "hinge", rate: sign * rate };
      heldRef.current = sign;
      setHeld(sign);
      sendRef.current(msg);
      timer.current = setInterval(() => sendRef.current(msg), DRIVE_RESEND_MS);
    },
    [enabled, rate, release],
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
      const sign = HINGE_KEYS[e.key.toLowerCase()];
      if (!sign) return;
      e.preventDefault();
      if (!e.repeat) start(sign);
    };
    const onUp = (e: KeyboardEvent) => {
      const sign = HINGE_KEYS[e.key.toLowerCase()];
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
  }, [start, end, release]);

  return { held, start, end };
}

function HoldButton({
  label,
  hint,
  sign,
  held,
  disabled,
  onStart,
  onEnd,
}: {
  label: string;
  hint: string;
  sign: Sign;
  held: Sign | null;
  disabled: boolean;
  onStart: (s: Sign) => void;
  onEnd: (s: Sign) => void;
}) {
  return (
    <button
      type="button"
      disabled={disabled}
      onPointerDown={(e) => {
        e.currentTarget.setPointerCapture(e.pointerId);
        onStart(sign);
      }}
      onPointerUp={() => onEnd(sign)}
      onPointerCancel={() => onEnd(sign)}
      onLostPointerCapture={() => onEnd(sign)}
      onContextMenu={(e) => e.preventDefault()}
      className={`touch-none select-none rounded-2xl px-4 py-4 font-semibold shadow-md active:scale-95 disabled:opacity-40 ${
        held === sign ? "bg-violet-700 text-white" : "bg-violet-600 text-white"
      }`}
    >
      {label} <span className="ml-1 text-xs font-normal opacity-80">{hint}</span>
    </button>
  );
}

const fmt = (v: number | null, digits = 0) => (v === null ? "?" : v.toFixed(digits));

/** Hinge (servo 1) hold buttons and the readout for calibrating its flat step. */
export default function HingePanel({
  hinge,
  send,
  enabled,
}: {
  hinge: HingeState;
  send: (msg: ClientMessage) => void;
  enabled: boolean;
}) {
  const { held, start, end } = useHingeHold(send, hinge.max_rate, enabled);
  const unknown = hinge.angle_deg === null;

  return (
    <section className="rounded-2xl border border-black/10 p-3 dark:border-white/15">
      <div className="mb-2 flex items-baseline justify-between gap-2">
        <h2 className="font-semibold">Hinge</h2>
        <span className="text-xs text-black/60 dark:text-white/60">
          {fmt(hinge.min_deg)}..{fmt(hinge.max_deg)}°, 180 flat
        </span>
      </div>
      <dl className="mb-3 grid grid-cols-3 gap-2 text-center">
        <div>
          <dt className="text-xs text-black/60 dark:text-white/60">Angle</dt>
          <dd className="text-xl font-bold tabular-nums">{fmt(hinge.angle_deg, 1)}°</dd>
        </div>
        <div>
          <dt className="text-xs text-black/60 dark:text-white/60">Raw step</dt>
          <dd className="text-xl font-bold tabular-nums">{fmt(hinge.raw_steps)}</dd>
        </div>
        <div>
          <dt className="text-xs text-black/60 dark:text-white/60">Flat (config)</dt>
          <dd className="text-xl font-bold tabular-nums">{fmt(hinge.flat_steps)}</dd>
        </div>
      </dl>
      <div className="grid grid-cols-2 gap-3">
        <HoldButton
          label="Close"
          hint="F"
          sign={-1}
          held={held}
          disabled={!enabled || unknown}
          onStart={start}
          onEnd={end}
        />
        <HoldButton
          label="Open"
          hint="R"
          sign={1}
          held={held}
          disabled={!enabled || unknown}
          onStart={start}
          onEnd={end}
        />
      </div>
      <p className="mt-2 text-xs text-black/60 dark:text-white/60">
        {unknown
          ? "Angle unknown: the servo is offline or outside the range, so the hinge will not move. Fix model.bend_flat_steps."
          : "To calibrate: lay the robot flat, read the raw step and put it in model.bend_flat_steps."}
      </p>
    </section>
  );
}
