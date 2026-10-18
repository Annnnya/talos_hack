"use client";

import Link from "next/link";
import { useCallback } from "react";
import ServoCard from "@/components/ServoCard";
import StopAllButton from "@/components/StopAllButton";
import { useBridge } from "@/lib/useBridge";

const STATUS_STYLE = {
  connected: "bg-emerald-500/15 text-emerald-600 dark:text-emerald-400",
  connecting: "bg-amber-500/15 text-amber-600 dark:text-amber-400",
  disconnected: "bg-red-500/15 text-red-600 dark:text-red-400",
};

export default function DebugPage() {
  const { status, servos, limits, ros, settings, clientId, lastError, clearError, send } =
    useBridge();
  const stopAll = useCallback(() => send({ type: "stop_all" }), [send]);
  const online = servos.filter((s) => s.online).length;

  return (
    <main className="mx-auto max-w-7xl px-4 py-6 flex flex-col gap-6">
      <header className="flex flex-wrap items-center justify-between gap-4">
        <div>
          <h1 className="text-2xl font-bold">Robot Servo Control</h1>
          <div className="mt-1 flex flex-wrap items-center gap-2 text-sm">
            <span className={`rounded-full px-2.5 py-0.5 font-medium ${STATUS_STYLE[status]}`}>
              {status === "connected" ? "Connected" : status === "connecting" ? "Connecting…" : "Disconnected - retrying"}
            </span>
            <span className="text-black/60 dark:text-white/60">
              {online}/{servos.length} joints online
            </span>
            {ros && (
              <span
                className="rounded-full bg-teal-500/15 px-2.5 py-0.5 font-medium text-teal-700 dark:text-teal-300"
                title="Commands go through the ROS robot node, shared with the CLI and other ROS nodes"
              >
                Via ROS
              </span>
            )}
            <Link href="/" className="text-black/60 underline hover:text-black dark:text-white/60 dark:hover:text-white">
              Joystick
            </Link>
          </div>
        </div>
        <StopAllButton onStop={stopAll} disabled={status !== "connected"} />
      </header>

      {lastError && (
        <div className="flex items-start justify-between gap-4 rounded-lg border border-red-500/30 bg-red-500/10 px-4 py-2 text-sm text-red-600 dark:text-red-400">
          <span>{lastError}</span>
          <button onClick={clearError} className="font-bold" aria-label="Dismiss">
            ×
          </button>
        </div>
      )}

      {servos.length === 0 ? (
        <p className="text-black/60 dark:text-white/60">Waiting for the robot…</p>
      ) : (
        <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-5">
          {servos.map((s) => (
            <ServoCard
              key={s.name}
              servo={s}
              limits={limits}
              settings={settings[s.name]}
              clientId={clientId}
              ros={ros}
              send={send}
            />
          ))}
        </div>
      )}
    </main>
  );
}
