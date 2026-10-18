"use client";

import { useCallback } from "react";
import type { ClientMessage, HingeState } from "@/lib/protocol";
import { type Sign, useHold } from "@/lib/useHold";

const HINGE_KEYS: Record<string, Sign> = { r: 1, f: -1 };

export function HoldButton({
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
  const message = useCallback(
    (sign: Sign | 0): ClientMessage => ({ type: "hinge", rate: sign * hinge.max_rate }),
    [hinge.max_rate],
  );
  const { held, start, end } = useHold(send, message, enabled, HINGE_KEYS);
  const unknown = hinge.angle_deg === null;

  return (
    <section className="rounded-2xl border border-black/10 p-3 dark:border-white/15">
      <div className="mb-2 flex items-baseline justify-between gap-2">
        <h2 className="font-semibold">Hinge</h2>
        <span className="text-xs text-black/60 dark:text-white/60">
          {hinge.current_a === null ? "" : `${Math.abs(hinge.current_a).toFixed(2)} A · `}
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
