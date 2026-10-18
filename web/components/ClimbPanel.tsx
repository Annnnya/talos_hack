"use client";

import { useCallback } from "react";
import { HoldButton } from "@/components/HingePanel";
import type { ClientMessage, LevelState } from "@/lib/protocol";
import { type Sign, useHold } from "@/lib/useHold";

/** Desktop keys next to the hinge keys: T up, G down. */
const CLIMB_KEYS: Record<string, Sign> = { t: 1, g: -1 };
/** Sideways like on the ground: A left, D right. */
const SIDE_KEYS: Record<string, Sign> = { a: 1, d: -1 };

/**
 * Up and Down after wall nearby: the front wheels turn one way and the rear
 * wheels the other, so a robot pressed between two walls climbs. Hold to move.
 */
export default function ClimbPanel({
  send,
  speed,
  enabled,
  wheelCurrents,
  level,
  levelEnabled,
}: {
  send: (msg: ClientMessage) => void;
  /** m/s while held. */
  speed: number;
  enabled: boolean;
  /** Wheel currents (A) by joint name. */
  wheelCurrents: Record<string, number> | null;
  /** IMU level loop (#82); null without the expansion node. */
  level: LevelState | null;
  /** The loop may be turned on (near the wall, ROS). */
  levelEnabled: boolean;
}) {
  const message = useCallback((sign: Sign | 0): ClientMessage => ({ type: "climb", speed: sign * speed }), [speed]);
  const { held, start, end } = useHold(send, message, enabled, CLIMB_KEYS);
  const sideMessage = useCallback(
    (sign: Sign | 0): ClientMessage => ({ type: "wall_side", speed: sign * speed }),
    [speed],
  );
  const side = useHold(send, sideMessage, enabled, SIDE_KEYS);

  return (
    <section className="rounded-2xl border border-black/10 p-3 dark:border-white/15">
      <div className="mb-2 flex items-baseline justify-between gap-2">
        <h2 className="font-semibold">On the wall</h2>
        <span className="text-xs text-black/60 dark:text-white/60">{(speed * 100).toFixed(1)} cm/s</span>
      </div>
      <div className="grid grid-cols-3 gap-2">
        <span />
        <HoldButton label="Up" hint="T" sign={1} held={held} disabled={!enabled} onStart={start} onEnd={end} />
        <span />
        <HoldButton
          label="Left"
          hint="A"
          sign={1}
          held={side.held}
          disabled={!enabled}
          onStart={side.start}
          onEnd={side.end}
        />
        <span />
        <HoldButton
          label="Right"
          hint="D"
          sign={-1}
          held={side.held}
          disabled={!enabled}
          onStart={side.start}
          onEnd={side.end}
        />
        <span />
        <HoldButton label="Down" hint="G" sign={-1} held={held} disabled={!enabled} onStart={start} onEnd={end} />
        <span />
      </div>
      {level && (
        <div className="mt-3 flex flex-wrap items-center justify-between gap-2 text-sm">
          <button
            type="button"
            onClick={() => send({ type: "level", enabled: !level.active })}
            disabled={!levelEnabled && !level.active}
            title="Keep the platform level with the IMU: pitch trims the front pair against the rear, roll the left against the right"
            className={`rounded-2xl px-4 py-2 font-semibold text-white shadow-md active:scale-95 disabled:opacity-40 ${
              level.active ? "bg-red-600" : "bg-emerald-600"
            }`}
          >
            {level.active ? "Stop levelling" : "Level with the IMU"}
          </button>
          <span className="tabular-nums text-black/60 dark:text-white/60">
            {level.imu_ok && level.pitch_deg !== null && level.roll_deg !== null
              ? `pitch ${level.pitch_deg.toFixed(1)}°, roll ${level.roll_deg.toFixed(1)}°`
              : "no IMU"}
            {level.active &&
              ` · trim ${(level.front_rear_m_s * 100).toFixed(1)} / ${(level.left_right_m_s * 100).toFixed(1)} cm/s`}
          </span>
          {level.fault && (
            <span className="w-full text-xs font-semibold text-red-600 dark:text-red-400">{level.fault}</span>
          )}
          {level.note && !level.active && !level.fault && (
            <span className="w-full text-xs text-amber-700 dark:text-amber-400">Level stopped: {level.note}</span>
          )}
        </div>
      )}
      {wheelCurrents && Object.keys(wheelCurrents).length > 0 && (
        <dl className="mt-3 grid grid-cols-5 gap-1 text-center text-xs">
          {Object.entries(wheelCurrents).map(([name, a]) => (
            <div key={name}>
              <dt className="text-black/60 dark:text-white/60">
                {name
                  .split("_")
                  .slice(0, 2)
                  .map((w) => w[0].toUpperCase())
                  .join("")}
              </dt>
              <dd className="font-bold tabular-nums">{Math.abs(a).toFixed(2)}</dd>
            </div>
          ))}
          <div>
            <dt className="text-black/60 dark:text-white/60">Sum A</dt>
            <dd className="font-bold tabular-nums">
              {Object.values(wheelCurrents)
                .reduce((sum, a) => sum + Math.abs(a), 0)
                .toFixed(2)}
            </dd>
          </div>
        </dl>
      )}
      <p className="mt-2 text-xs text-black/60 dark:text-white/60">
        Up/Down: front wheels one way, rear the other. Left/Right: like sideways on the ground. Hold both for diagonals.
        Clamp with the hinge first.
      </p>
    </section>
  );
}
