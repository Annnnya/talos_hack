"use client";

import { CommandCaption, GhostMarker } from "@/components/LastCommand";
import { trackLeft } from "@/lib/lastCommand";
import type { ClientMessage, Limits, ServoSettings, ServoState } from "@/lib/protocol";
import { DEFAULT_SETTINGS, stepsToDeg } from "@/lib/protocol";
import { useThrottle } from "@/lib/useBridge";
import { useFollow } from "@/lib/useFollow";

interface Props {
  servo: ServoState;
  limits: Limits;
  /** Shared move speed and acceleration; undefined until the first state message. */
  settings: ServoSettings | undefined;
  clientId: string | null;
  /** Commands go through the ROS robot node, which has no acceleration input. */
  ros: boolean;
  send: (msg: ClientMessage) => void;
}

export default function ServoCard({ servo, limits, settings, clientId, ros, send }: Props) {
  const shared = settings ?? DEFAULT_SETTINGS;
  // Every control follows the last command or shared setting, wherever it came from,
  // except while this user is holding it.
  const target = useFollow(servo.target, servo.position);
  const wheel = useFollow(servo.target_speed, 0);
  const moveSpeed = useFollow(shared.move_speed, DEFAULT_SETTINGS.move_speed);
  const acc = useFollow(shared.acc, DEFAULT_SETTINGS.acc);

  const sendMove = useThrottle((position: number) =>
    send({ type: "move", joint: servo.name, position, speed: moveSpeed.value, acc: acc.value }),
  );
  const sendSpin = useThrottle((speed: number) => send({ type: "spin", joint: servo.name, speed, acc: acc.value }));
  // One throttle per setting, so a quick change to one never drops the other.
  const sendMoveSpeed = useThrottle((v: number) => send({ type: "settings", joint: servo.name, move_speed: v }));
  const sendAcc = useThrottle((v: number) => send({ type: "settings", joint: servo.name, acc: v }));

  const isWheel = servo.control === "speed";
  const disabled = !servo.online;
  const span = Math.max(1, servo.max_pos - servo.min_pos);
  const posPct = (steps: number) => ((steps - servo.min_pos) / span) * 100;
  const speedPct = (speed: number) => ((speed + limits.speed_max) / (2 * limits.speed_max)) * 100;

  const goTo = (position: number) => {
    target.setValue(position);
    sendMove(position);
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
          <span className="text-xs text-black/50 dark:text-white/50">
            ID {servo.id} · {isWheel ? "Speed" : "Position"}
          </span>
        </div>
        <label className="flex items-center gap-2 text-sm cursor-pointer select-none">
          Torque
          <input
            type="checkbox"
            className="h-4 w-4"
            checked={servo.torque}
            disabled={disabled}
            onChange={(e) => send({ type: "torque", joint: servo.name, enabled: e.target.checked })}
          />
        </label>
      </div>

      {/* Controls */}
      {!isWheel ? (
        <div className="flex flex-col gap-3">
          <Field label="Target position" value={`${target.value} (${stepsToDeg(target.value).toFixed(1)}°)`}>
            <div className="relative pt-1">
              <input
                type="range"
                className="w-full"
                min={servo.min_pos}
                max={servo.max_pos}
                value={target.value}
                disabled={disabled}
                aria-label="Target position"
                aria-valuetext={`${target.value} steps, ${stepsToDeg(target.value).toFixed(0)} degrees`}
                {...target.holdProps}
                onChange={(e) => goTo(Number(e.target.value))}
              />
              {servo.target !== null && servo.target !== target.value && (
                <GhostMarker
                  pct={posPct(servo.target)}
                  last={servo.last_command}
                  clientId={clientId}
                  valueText={`${servo.target} (${stepsToDeg(servo.target).toFixed(1)}°)`}
                />
              )}
              <LiveMarker pct={posPct(servo.position)} title={`Current position ${servo.position}`} />
            </div>
            <CommandCaption last={servo.last_command} clientId={clientId} />
          </Field>
          <div className="flex gap-2">
            <input
              type="number"
              className="w-24 rounded-md border border-black/15 dark:border-white/15 bg-transparent px-2 py-1 text-sm"
              min={servo.min_pos}
              max={servo.max_pos}
              value={target.value}
              disabled={disabled}
              aria-label="Target position (steps)"
              onFocus={target.hold}
              onBlur={target.release}
              onChange={(e) => target.setValue(Number(e.target.value))}
              onKeyDown={(e) => {
                if (e.key === "Enter") {
                  sendMove(target.value);
                  target.release();
                }
              }}
            />
            <button
              disabled={disabled}
              onClick={() => {
                sendMove(target.value);
                target.release();
              }}
              className="rounded-md bg-blue-600 px-3 py-1 text-sm font-medium text-white hover:bg-blue-500"
            >
              Go
            </button>
            <button
              disabled={disabled}
              onClick={() => goTo(Math.round((servo.min_pos + servo.max_pos) / 2))}
              className="rounded-md border border-black/15 dark:border-white/15 px-3 py-1 text-sm hover:bg-black/5 dark:hover:bg-white/10"
            >
              Center
            </button>
          </div>
          <Slider
            label="Move speed"
            value={moveSpeed.value}
            min={1}
            max={limits.speed_max}
            hint="steps/s"
            disabled={disabled}
            holdProps={moveSpeed.holdProps}
            onChange={(v) => {
              moveSpeed.setValue(v);
              sendMoveSpeed(v);
            }}
          />
        </div>
      ) : (
        <div className="flex flex-col gap-3">
          <Field label="Speed" value={`${wheel.value} steps/s`}>
            <div className="relative pt-1">
              <input
                type="range"
                className="w-full"
                min={-limits.speed_max}
                max={limits.speed_max}
                value={wheel.value}
                disabled={disabled}
                aria-label="Wheel speed"
                aria-valuetext={`${wheel.value} steps per second`}
                {...wheel.holdProps}
                onChange={(e) => {
                  const v = Number(e.target.value);
                  wheel.setValue(v);
                  sendSpin(v);
                }}
              />
              {servo.target_speed !== wheel.value && (
                <GhostMarker
                  pct={speedPct(servo.target_speed)}
                  last={servo.last_command}
                  clientId={clientId}
                  valueText={`${servo.target_speed} steps/s`}
                />
              )}
              <LiveMarker pct={speedPct(servo.speed)} title={`Current speed ${servo.speed} steps/s`} />
            </div>
            <div className="flex justify-between text-[10px] text-black/40 dark:text-white/40">
              <span>◀ reverse</span>
              <span>stop</span>
              <span>forward ▶</span>
            </div>
            <CommandCaption last={servo.last_command} clientId={clientId} />
          </Field>
          <button
            disabled={disabled}
            onClick={() => {
              wheel.setValue(0);
              send({ type: "stop", joint: servo.name });
            }}
            className="rounded-md bg-red-600 py-1.5 text-sm font-semibold text-white hover:bg-red-500"
          >
            Stop
          </button>
        </div>
      )}

      <Slider
        label="Acceleration"
        value={acc.value}
        min={0}
        max={limits.acc_max}
        disabled={disabled || ros}
        valueText={ros ? "not used over ROS" : undefined}
        holdProps={acc.holdProps}
        onChange={(v) => {
          acc.setValue(v);
          sendAcc(v);
        }}
      />

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

/** Amber diamond below the track: where the servo actually is right now. */
function LiveMarker({ pct, title }: { pct: number; title: string }) {
  return (
    <div
      className="pointer-events-none absolute -bottom-1.5 h-1.5 w-1.5 -translate-x-1/2 rotate-45 bg-amber-500"
      style={{ left: trackLeft(pct) }}
      title={title}
    />
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
  /** Replaces the number in the header, e.g. to say why the slider is disabled. */
  valueText?: string;
  disabled?: boolean;
  holdProps?: React.InputHTMLAttributes<HTMLInputElement>;
  onChange: (v: number) => void;
}) {
  const shown = props.valueText ?? `${props.value}${props.hint ? ` ${props.hint}` : ""}`;
  return (
    <Field label={props.label} value={shown}>
      <input
        type="range"
        className="w-full"
        min={props.min}
        max={props.max}
        value={props.value}
        disabled={props.disabled}
        aria-label={props.label}
        {...props.holdProps}
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
