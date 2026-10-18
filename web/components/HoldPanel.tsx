"use client";

import { useEffect, useRef, useState } from "react";
import type { ClientMessage, HoldState } from "@/lib/protocol";
import { useThrottle } from "@/lib/useBridge";

/** Seconds of history in the trace. */
const TRACE_S = 10;
const HOT_C = 60;
const W = 300;
const H = 70;

type Sample = { t: number; setpoint: number; measured: number };

/** Setpoint vs measured hinge current over the last TRACE_S seconds. */
function Trace({ samples, max }: { samples: Sample[]; max: number }) {
  if (samples.length < 2) return null;
  const now = samples[samples.length - 1].t;
  const x = (t: number) => W - ((now - t) / (TRACE_S * 1000)) * W;
  const y = (a: number) => H - Math.min(1, a / max) * H;
  const line = (key: "setpoint" | "measured") =>
    samples.map((s, i) => `${i ? "L" : "M"}${x(s.t).toFixed(1)},${y(s[key]).toFixed(1)}`).join("");
  return (
    <svg viewBox={`0 0 ${W} ${H}`} className="h-16 w-full" preserveAspectRatio="none" aria-label="Current trace">
      <path d={line("setpoint")} fill="none" stroke="currentColor" strokeOpacity={0.4} strokeDasharray="4 3" />
      <path d={line("measured")} fill="none" stroke="#10b981" strokeWidth={2} />
    </svg>
  );
}

/**
 * Hinge current hold after wall nearby (#142): the expansion node moves the
 * hinge goal until the hinge current matches the setpoint, and keeps holding
 * through STOP and climbing until turned off here or by Release.
 */
export default function HoldPanel({
  hold,
  send,
  enabled,
  temp,
}: {
  hold: HoldState;
  send: (msg: ClientMessage) => void;
  enabled: boolean;
  /** Hinge servo temperature, °C. */
  temp: number | null;
}) {
  const [setpoint, setSetpoint] = useState(Math.min(0.3, hold.max_current_a));
  const [samples, setSamples] = useState<Sample[]>([]);
  const activeRef = useRef(hold.active);
  activeRef.current = hold.active;

  useEffect(() => {
    const t = Date.now();
    setSamples((prev) => [
      ...prev.filter((s) => t - s.t < TRACE_S * 1000),
      { t, setpoint: hold.active ? hold.setpoint_a : 0, measured: hold.measured_a },
    ]);
  }, [hold]);

  // A new setpoint while holding goes out at once (throttled while dragging).
  const sendSetpoint = useThrottle((value: number) => {
    if (activeRef.current) send({ type: "hinge_hold", enabled: true, setpoint: value });
  }, 150);

  const toggle = () => send({ type: "hinge_hold", enabled: !hold.active, setpoint });

  return (
    <section className="rounded-2xl border border-black/10 p-3 dark:border-white/15">
      <div className="mb-2 flex items-baseline justify-between gap-2">
        <h2 className="font-semibold">Hold (hinge current)</h2>
        <span
          className={`rounded-full px-2 py-0.5 text-xs font-medium ${
            hold.active
              ? "bg-emerald-500/15 text-emerald-600 dark:text-emerald-400"
              : "bg-black/5 text-black/60 dark:bg-white/10 dark:text-white/60"
          }`}
        >
          {hold.active ? "Holding" : "Off"}
        </span>
      </div>
      <dl className="mb-2 grid grid-cols-3 gap-2 text-center">
        <div>
          <dt className="text-xs text-black/60 dark:text-white/60">Measured</dt>
          <dd className="text-xl font-bold tabular-nums">{hold.measured_a.toFixed(2)} A</dd>
        </div>
        <div>
          <dt className="text-xs text-black/60 dark:text-white/60">Setpoint</dt>
          <dd className="text-xl font-bold tabular-nums">{setpoint.toFixed(2)} A</dd>
        </div>
        <div>
          <dt className="text-xs text-black/60 dark:text-white/60">Goal lead</dt>
          <dd className="text-xl font-bold tabular-nums">{((hold.offset_rad * 180) / Math.PI).toFixed(1)}°</dd>
        </div>
      </dl>
      <Trace samples={samples} max={hold.max_current_a} />
      <input
        type="range"
        min={0}
        max={hold.max_current_a}
        step={0.05}
        value={setpoint}
        onChange={(e) => {
          const value = Number(e.target.value);
          setSetpoint(value);
          sendSetpoint(value);
        }}
        className="my-2 w-full accent-emerald-600"
        aria-label="Hinge current setpoint"
      />
      <button
        type="button"
        onClick={toggle}
        disabled={!enabled && !hold.active}
        className={`w-full rounded-2xl px-4 py-3 font-semibold text-white shadow-md active:scale-95 disabled:opacity-40 ${
          hold.active ? "bg-red-600" : "bg-emerald-600"
        }`}
      >
        {hold.active ? "Stop holding" : `Hold at ${setpoint.toFixed(2)} A`}
      </button>
      {hold.note && !hold.active && (
        <p className="mt-2 text-xs text-amber-700 dark:text-amber-400">Stopped: {hold.note}</p>
      )}
      {temp !== null && temp >= HOT_C && (
        <p className="mt-2 text-xs font-semibold text-red-600 dark:text-red-400">
          Hinge servo at {temp} °C: lower the setpoint.
        </p>
      )}
    </section>
  );
}
