"use client";

import Link from "next/link";
import { useCallback, useEffect, useMemo } from "react";
import Joystick from "@/components/Joystick";
import ReleaseStopButton from "@/components/ReleaseStopButton";
import { useStopKey } from "@/components/StopAllButton";
import { useJoystick } from "@/lib/joystick";
import { allowedDirections, describeInput, MODE_LABEL, MODE_STYLE } from "@/lib/modes";
import { useBridge } from "@/lib/useBridge";

const STATUS_STYLE = {
  connected: "bg-emerald-500/15 text-emerald-600 dark:text-emerald-400",
  connecting: "bg-amber-500/15 text-amber-600 dark:text-amber-400",
  disconnected: "bg-red-500/15 text-red-600 dark:text-red-400",
};

const STATUS_TEXT = {
  connected: "Connected",
  connecting: "Connecting…",
  disconnected: "Disconnected - retrying",
};

/** One-shot keys (N, L). A null action ignores its key. */
function useActionKeys(actions: Record<string, (() => void) | null>) {
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const tag = (e.target as HTMLElement)?.tagName;
      if (e.repeat || e.ctrlKey || e.metaKey || e.altKey || tag === "INPUT" || tag === "TEXTAREA") return;
      const action = actions[e.key.toLowerCase()];
      if (action) {
        e.preventDefault();
        action();
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [actions]);
}

export default function Home() {
  const { status, limits, ros, stopped, mode, modeReason, activeInput, lastError, clearError, send } =
    useBridge();
  const connected = status === "connected";
  // Driving goes through twist_mux, so it needs the ROS server and a base.
  const canDrive = connected && ros && limits.linear_max !== null && limits.angular_max !== null;
  const { active, start, end, release } = useJoystick(send, {
    linear: limits.linear_max ?? 0,
    angular: limits.angular_max ?? 0,
    enabled: canDrive,
    allowed: allowedDirections(mode),
  });
  const stopAll = useCallback(() => {
    release();
    send({ type: "stop_all" });
  }, [release, send]);
  const releaseStop = useCallback(() => send({ type: "release_stop" }), [send]);
  useStopKey(stopAll);
  // Wall nearby starts aligning from free; release brings the robot back to
  // free from any other mode. The mode node checks both again.
  const canWallNearby = connected && ros && mode === "free";
  const canRelease = connected && ros && mode !== null && mode !== "free";
  const wallNearby = useCallback(() => send({ type: "wall_nearby" }), [send]);
  const releaseWalls = useCallback(() => send({ type: "release" }), [send]);
  const actionKeys = useMemo(
    () => ({ n: canWallNearby ? wallNearby : null, l: canRelease ? releaseWalls : null }),
    [canWallNearby, canRelease, wallNearby, releaseWalls],
  );
  useActionKeys(actionKeys);

  let notice: string | null = null;
  if (connected && !ros) notice = "Driving needs the ROS web server: talos-servo web --ros.";
  else if (connected && limits.linear_max === null) notice = "The config has no base, so there is nothing to drive.";

  return (
    <main className="mx-auto flex min-h-dvh max-w-md flex-col gap-4 px-4 py-4">
      <header className="flex items-center justify-between gap-3">
        <div>
          <h1 className="text-2xl font-bold">Talos</h1>
          <div className="mt-1 flex flex-wrap items-center gap-2 text-sm">
            <span className={`rounded-full px-2.5 py-0.5 font-medium ${STATUS_STYLE[status]}`}>
              {STATUS_TEXT[status]}
            </span>
            <Link
              href="/debug"
              className="text-black/60 underline hover:text-black dark:text-white/60 dark:hover:text-white"
            >
              Per-motor debug
            </Link>
          </div>
        </div>
        {ros && stopped && <ReleaseStopButton onRelease={releaseStop} disabled={!connected} />}
      </header>

      {lastError && (
        <div className="flex items-start justify-between gap-4 rounded-lg border border-red-500/30 bg-red-500/10 px-4 py-2 text-sm text-red-600 dark:text-red-400">
          <span>{lastError}</span>
          <button onClick={clearError} className="font-bold" aria-label="Dismiss">
            ×
          </button>
        </div>
      )}
      {ros && (
        <div className="flex flex-wrap items-center gap-2 text-sm">
          <span
            className={`rounded-full px-2.5 py-0.5 font-medium ${
              mode ? MODE_STYLE[mode] : "bg-black/5 text-black/60 dark:bg-white/10 dark:text-white/60"
            }`}
          >
            {mode ? MODE_LABEL[mode] : "Mode unknown"}
          </span>
          {modeReason && <span className="text-black/60 dark:text-white/60">{modeReason}</span>}
          <span className="ml-auto text-black/60 dark:text-white/60">Driving: {describeInput(activeInput)}</span>
        </div>
      )}
      {ros && mode === null && connected && (
        <p className="text-sm text-black/60 dark:text-white/60">
          No word from the mode node: start it with `pixi run -e ros mode-node`.
        </p>
      )}
      {notice && <p className="text-sm text-black/60 dark:text-white/60">{notice}</p>}
      {ros && stopped && (
        <p className="text-sm text-amber-700 dark:text-amber-400">
          STOP is on: the robot ignores every input until you release it.
        </p>
      )}

      <div className="flex flex-1 items-center justify-center">
        <Joystick
          active={active}
          allowed={allowedDirections(mode)}
          disabled={!canDrive}
          onStart={start}
          onEnd={end}
          onStop={stopAll}
          stopDisabled={!connected}
        />
      </div>

      <div className="grid grid-cols-2 gap-3">
        <button
          type="button"
          onClick={wallNearby}
          disabled={!canWallNearby}
          title="Start aligning with the walls (N)"
          className="rounded-2xl bg-blue-600 px-4 py-4 font-semibold text-white shadow-md active:scale-95 disabled:opacity-40"
        >
          Wall nearby <span className="ml-1 text-xs font-normal opacity-80">N</span>
        </button>
        <button
          type="button"
          onClick={releaseWalls}
          disabled={!canRelease}
          title="Release the walls and go back to free mode (L)"
          className="rounded-2xl border border-black/15 px-4 py-4 font-semibold active:scale-95 disabled:opacity-40 dark:border-white/20"
        >
          Release <span className="ml-1 text-xs font-normal opacity-60">L</span>
        </button>
      </div>

      <p className="hidden text-center text-xs text-black/50 sm:block dark:text-white/50">
        Hold W/S/A/D to move, Q/E to turn. Space stops, Enter releases STOP. N wall nearby, L release.
      </p>
    </main>
  );
}
