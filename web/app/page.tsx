"use client";

import Link from "next/link";
import { useCallback } from "react";
import ReleaseStopButton from "@/components/ReleaseStopButton";
import StopAllButton from "@/components/StopAllButton";
import { useBridge } from "@/lib/useBridge";

// Placeholder until the joystick page (#70) takes over `/`. It keeps its own
// connection so STOP ALL works from here, e.g. on a phone while the CLI
// drives.
export default function Home() {
  const { status, ros, stopped, send } = useBridge();
  const stopAll = useCallback(() => send({ type: "stop_all" }), [send]);
  const releaseStop = useCallback(() => send({ type: "release_stop" }), [send]);

  return (
    <main className="mx-auto max-w-7xl px-4 py-6 flex flex-col gap-6">
      <header className="flex flex-wrap items-center justify-between gap-4">
        <div>
          <h1 className="text-2xl font-bold">Talos</h1>
          <Link
            href="/debug"
            className="text-sm text-black/60 underline hover:text-black dark:text-white/60 dark:hover:text-white"
          >
            Per-motor debug
          </Link>
        </div>
        <div className="flex items-center gap-3">
          {ros && stopped && (
            <ReleaseStopButton onRelease={releaseStop} disabled={status !== "connected"} />
          )}
          <StopAllButton onStop={stopAll} disabled={status !== "connected"} />
        </div>
      </header>
      <p className="text-black/60 dark:text-white/60">
        The joystick is coming soon. Use the per-motor debug page for bench
        tests.
      </p>
    </main>
  );
}
