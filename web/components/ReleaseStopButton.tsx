"use client";

import { useEffect } from "react";

/**
 * Releases STOP (`/robot/set_stop` false) so twist_mux drives again. Shown
 * only over ROS while STOP is on. Also Enter, like the keyboard controller
 * (unless typing in a field or on a focused button, which Enter clicks).
 */
export default function ReleaseStopButton({ onRelease, disabled }: { onRelease: () => void; disabled?: boolean }) {
  useEffect(() => {
    if (disabled) return;
    const onKey = (e: KeyboardEvent) => {
      const tag = (e.target as HTMLElement)?.tagName;
      if (e.key === "Enter" && tag !== "INPUT" && tag !== "TEXTAREA" && tag !== "BUTTON") {
        e.preventDefault();
        onRelease();
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [onRelease, disabled]);

  return (
    <button
      onClick={onRelease}
      disabled={disabled}
      className="rounded-lg border border-emerald-600/40 px-4 py-2.5 font-semibold text-emerald-700 hover:bg-emerald-500/10 active:scale-95 disabled:opacity-50 dark:text-emerald-400"
      title="STOP is on: the base ignores the web and the keyboard until released (Enter)"
    >
      Release STOP <span className="ml-1 text-xs font-normal opacity-80">(Enter)</span>
    </button>
  );
}
