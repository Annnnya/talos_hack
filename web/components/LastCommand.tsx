"use client";

import { useEffect, useRef, useState } from "react";
import { describeSource, formatAge, trackLeft } from "@/lib/lastCommand";
import type { LastCommand } from "@/lib/protocol";

// Commands closer together than this are one gesture (someone dragging), not a new event.
const NEW_COMMAND_GAP_S = 0.5;
const FLASH_MS = 1500;

/** "● CLI · 3 s ago" under a slider. Fixed height, so the card never jumps. */
export function CommandCaption({ last, clientId }: { last: LastCommand | null; clientId: string | null }) {
  const info = last ? describeSource(last, clientId) : null;
  const [flash, setFlash] = useState(false);
  const [announce, setAnnounce] = useState("");
  const prev = useRef<{ key: string; age: number } | null>(null);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const source = last?.source;
  const client = last?.client;
  const age = last?.age;
  const mine = info?.mine ?? false;
  const label = info?.label ?? "";

  // A new command is a different sender, or the age starting over. Works the same over ROS,
  // where the server re-estimates when a command happened on every status message.
  useEffect(() => {
    if (source === undefined || age === undefined) return;
    const key = `${source}:${client}`;
    const before = prev.current;
    prev.current = { key, age };
    const isNew = before !== null && (before.key !== key || age < before.age - NEW_COMMAND_GAP_S);
    if (!isNew || mine) return;
    setFlash(true);
    setAnnounce(`Moved by ${label}`);
    if (timer.current) clearTimeout(timer.current);
    timer.current = setTimeout(() => setFlash(false), FLASH_MS);
  }, [source, client, age, mine, label]);

  useEffect(() => () => {
    if (timer.current) clearTimeout(timer.current);
  }, []);

  return (
    <div className="flex h-5 items-center text-[11px] leading-none text-black/45 dark:text-white/45">
      {info && last ? (
        <span
          className={`-mx-1.5 inline-flex items-center gap-1.5 rounded-full px-1.5 py-1 transition-colors duration-700 ${
            flash ? info.flash : "bg-transparent"
          }`}
          title={`Last command from ${info.label}`}
        >
          <span className={`h-1.5 w-1.5 shrink-0 rounded-full ${info.dot}`} />
          <span className={`font-medium transition-colors duration-700 ${flash ? info.text : "text-black/65 dark:text-white/65"}`}>
            {info.label}
          </span>
          <span aria-hidden>·</span>
          <span className="tabular-nums">{formatAge(last.age)}</span>
        </span>
      ) : (
        <span>No commands yet</span>
      )}
      <span className="sr-only" aria-live="polite">
        {announce}
      </span>
    </div>
  );
}

/**
 * Hollow ring above the track at what someone else last commanded. Only shown while your slider
 * differs from it (you are dragging, or typed a number you have not sent), so it never doubles
 * the thumb.
 */
export function GhostMarker({
  pct,
  last,
  clientId,
  valueText,
}: {
  pct: number;
  last: LastCommand | null;
  clientId: string | null;
  valueText: string;
}) {
  if (!last) return null;
  const info = describeSource(last, clientId);
  if (info.mine) return null;
  return (
    <div
      className={`pointer-events-none absolute -top-2 h-2.5 w-2.5 -translate-x-1/2 rounded-full border-2 bg-white dark:bg-[#161a21] ${info.ring}`}
      style={{ left: trackLeft(pct) }}
      title={`Last command: ${info.label}, ${valueText}`}
      aria-hidden
    />
  );
}
