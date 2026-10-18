"use client";

import { useCallback } from "react";
import { HoldButton } from "@/components/HingePanel";
import type { ClientMessage } from "@/lib/protocol";
import { type Sign, useHold } from "@/lib/useHold";

/** Desktop keys next to the hinge keys: T up, G down. */
const CLIMB_KEYS: Record<string, Sign> = { t: 1, g: -1 };

/**
 * Up and Down after wall nearby: the front wheels turn one way and the rear
 * wheels the other, so a robot pressed between two walls climbs. Hold to move.
 */
export default function ClimbPanel({
  send,
  speed,
  enabled,
}: {
  send: (msg: ClientMessage) => void;
  /** m/s while held. */
  speed: number;
  enabled: boolean;
}) {
  const message = useCallback((sign: Sign | 0): ClientMessage => ({ type: "climb", speed: sign * speed }), [speed]);
  const { held, start, end } = useHold(send, message, enabled, CLIMB_KEYS);

  return (
    <section className="rounded-2xl border border-black/10 p-3 dark:border-white/15">
      <div className="mb-2 flex items-baseline justify-between gap-2">
        <h2 className="font-semibold">Climb</h2>
        <span className="text-xs text-black/60 dark:text-white/60">{(speed * 100).toFixed(1)} cm/s</span>
      </div>
      <div className="grid grid-cols-2 gap-3">
        <HoldButton label="Down" hint="G" sign={-1} held={held} disabled={!enabled} onStart={start} onEnd={end} />
        <HoldButton label="Up" hint="T" sign={1} held={held} disabled={!enabled} onStart={start} onEnd={end} />
      </div>
      <p className="mt-2 text-xs text-black/60 dark:text-white/60">
        Front wheels one way, rear wheels the other. Clamp with the hinge first.
      </p>
    </section>
  );
}
