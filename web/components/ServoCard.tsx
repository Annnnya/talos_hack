"use client";

import { useEffect, useRef, useState } from "react";
import type { ClientMessage, Limits, Mode, ServoState } from "@/lib/protocol";
import { stepsToDeg } from "@/lib/protocol";
import { useThrottle } from "@/lib/useBridge";

interface Props {
  servo: ServoState;
  limits: Limits;
  send: (msg: ClientMessage) => void;
}

export default function ServoCard({ servo, limits, send }: Props) {
  const [target, setTarget] = useState(servo.target ?? servo.position);
  const [moveSpeed, setMoveSpeed] = useState(1000);
  const [acc, setAcc] = useState(50);
  const [wheelSpeed, setWheelSpeed] = useState(servo.target_speed);
  const dragging = useRef(false);

  // Follow changes made from other laptops, unless this user is dragging.
  useEffect(() => {
    if (!dragging.current && servo.target !== null) setTarget(servo.target);
  }, [servo.target]);
  useEffect(() => {
    if (!dragging.current) setWheelSpeed(servo.target_speed);
  }, [servo.target_speed]);

  const sendMove = useThrottle((position: number) =>
    send({ type: "move", id: servo.id, position, speed: moveSpeed, acc }),
  );
  const sendSpin = useThrottle((speed: number) => send({ type: "spin", id: servo.id, speed, acc }));

  const isWheel = servo.mode === "wheel";
  const disabled = !servo.online;
  const setMode = (mode: Mode) => mode !== servo.mode && send({ type: "set_mode", id: servo.id, mode });
  const span = Math.max(1, servo.max_pos - servo.min_pos);
  const markerPct = Math.min(100, Math.max(0, ((servo.position - servo.min_pos) / span) * 100));

  const dragProps = {
    onPointerDown: () => (dragging.current = true),
    onPointerUp: () => (dragging.current = false),
    onBlur: () => (dragging.current = false),
  };

  return (
    <section
      className={`rounded-xl border p-4 flex flex-col gap-4 bg-white dark:bg-[#161a21] border-black/10 dark:border-white/10 ${
        disabled ? "opacity-60" : ""
      }`}
    >
      {/* Header */}
      <div className="flex items-center justify-between gap-2">
        <div className="flex items-center gap-2 min-w-0">
          <span
            className={`h-2.5 w-2.5 shrink-0 rounded-full ${servo.online ? "bg-emerald-500" : "bg-red-500"}`}
            title={servo.online ? "Online" : "Offline"}
          />
          <h2 className="font-semibold truncate">{servo.name}</h2>
          <span className="text-xs text-black/50 dark:text-white/50">ID {servo.id}</span>
        </div>
        <label className="flex items-center gap-2 text-sm cursor-pointer select-none">
          Torque
          <input
            type="checkbox"
            className="h-4 w-4"
            checked={servo.torque}
            disabled={disabled}
            onChange={(e) => send({ type: "torque", id: servo.id, enabled: e.target.checked })}
          />
        </label>
      </div>

      {/* Mode toggle */}
      <div className="grid grid-cols-2 rounded-lg bg-black/5 dark:bg-white/5 p-1 text-sm font-medium">
        {(["position", "wheel"] as Mode[]).map((m) => (
          <button
            key={m}
            disabled={disabled}
            onClick={() => setMode(m)}
            className={`rounded-md py-1.5 transition ${
              servo.mode === m ? "bg-blue-600 text-white shadow" : "hover:bg-black/5 dark:hover:bg-white/10"
            }`}
          >
            {m === "position" ? "Position" : "Speed"}
          </button>
        ))}
      </div>

      {/* Controls */}
      {!isWheel ? (
        <div className="flex flex-col gap-3">
          <Field label="Target position" value={`${target} (${stepsToDeg(target).toFixed(1)}°)`}>
            <div className="relative">
              <input
                type="range"
                className="w-full"
                min={servo.min_pos}
                max={servo.max_pos}
                value={target}
                disabled={disabled}
                {...dragProps}
                onChange={(e) => {
                  const v = Number(e.target.value);
                  setTarget(v);
                  sendMove(v);
                }}
              />
              {/* live position marker */}
              <div
                className="pointer-events-none absolute -bottom-1.5 h-1.5 w-1.5 -translate-x-1/2 rotate-45 bg-amber-500"
                style={{ left: `${markerPct}%` }}
                title="Current position"
              />
            </div>
          </Field>
          <div className="flex gap-2">
            <input
              type="number"
              className="w-24 rounded-md border border-black/15 dark:border-white/15 bg-transparent px-2 py-1 text-sm"
              min={servo.min_pos}
              max={servo.max_pos}
              value={target}
              disabled={disabled}
              onChange={(e) => setTarget(Number(e.target.value))}
              onKeyDown={(e) => e.key === "Enter" && sendMove(target)}
            />
            <button
              disabled={disabled}
              onClick={() => sendMove(target)}
              className="rounded-md bg-blue-600 px-3 py-1 text-sm font-medium text-white hover:bg-blue-500"
            >
              Go
            </button>
            <button
              disabled={disabled}
              onClick={() => {
                const center = Math.round((servo.min_pos + servo.max_pos) / 2);
                setTarget(center);
                sendMove(center);
              }}
              className="rounded-md border border-black/15 dark:border-white/15 px-3 py-1 text-sm hover:bg-black/5 dark:hover:bg-white/10"
            >
              Center
            </button>
          </div>
          <Slider label="Move speed" value={moveSpeed} min={0} max={limits.speed_max} onChange={setMoveSpeed} disabled={disabled} hint={moveSpeed === 0 ? "max" : "steps/s"} />
        </div>
      ) : (
        <div className="flex flex-col gap-3">
          <Field label="Speed" value={`${wheelSpeed} steps/s`}>
            <input
              type="range"
              className="w-full"
              min={-limits.speed_max}
              max={limits.speed_max}
              value={wheelSpeed}
              disabled={disabled}
              {...dragProps}
              onChange={(e) => {
                const v = Number(e.target.value);
                setWheelSpeed(v);
                sendSpin(v);
              }}
            />
            <div className="flex justify-between text-[10px] text-black/40 dark:text-white/40">
              <span>◀ reverse</span>
              <span>stop</span>
              <span>forward ▶</span>
            </div>
          </Field>
          <button
            disabled={disabled}
            onClick={() => {
              setWheelSpeed(0);
              send({ type: "stop", id: servo.id });
            }}
            className="rounded-md bg-red-600 py-1.5 text-sm font-semibold text-white hover:bg-red-500"
          >
            Stop
          </button>
        </div>
      )}

      <Slider label="Acceleration" value={acc} min={0} max={limits.acc_max} onChange={setAcc} disabled={disabled} hint={acc === 0 ? "max" : ""} />

      {/* Telemetry */}
      <dl className="grid grid-cols-3 gap-2 text-xs font-mono">
        <Stat k="Pos" v={`${servo.position}`} />
        <Stat k="Speed" v={`${servo.speed}`} />
        <Stat k="Load" v={`${servo.load}`} />
        <Stat k="Volt" v={`${servo.voltage.toFixed(1)} V`} />
        <Stat k="Temp" v={`${servo.temp} °C`} warn={servo.temp >= 60} />
        <Stat k="Angle" v={`${stepsToDeg(servo.position).toFixed(0)}°`} />
      </dl>

      {servo.error && <p className="text-xs text-red-500 break-words">{servo.error}</p>}
    </section>
  );
}

function Field({ label, value, children }: { label: string; value: string; children: React.ReactNode }) {
  return (
    <div className="flex flex-col gap-1">
      <div className="flex justify-between text-sm">
        <span className="text-black/60 dark:text-white/60">{label}</span>
        <span className="font-mono">{value}</span>
      </div>
      {children}
    </div>
  );
}

function Slider(props: {
  label: string;
  value: number;
  min: number;
  max: number;
  hint?: string;
  disabled?: boolean;
  onChange: (v: number) => void;
}) {
  return (
    <Field label={props.label} value={`${props.value}${props.hint ? ` ${props.hint}` : ""}`}>
      <input
        type="range"
        className="w-full"
        min={props.min}
        max={props.max}
        value={props.value}
        disabled={props.disabled}
        onChange={(e) => props.onChange(Number(e.target.value))}
      />
    </Field>
  );
}

function Stat({ k, v, warn }: { k: string; v: string; warn?: boolean }) {
  return (
    <div className="rounded-md bg-black/5 dark:bg-white/5 px-2 py-1">
      <dt className="text-black/50 dark:text-white/50">{k}</dt>
      <dd className={warn ? "text-red-500" : ""}>{v}</dd>
    </div>
  );
}
