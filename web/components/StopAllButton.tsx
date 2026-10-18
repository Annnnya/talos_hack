"use client";

import { useEffect } from "react";

/** Space calls `onStop` (unless typing in a field). */
export function useStopKey(onStop: () => void) {
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const tag = (e.target as HTMLElement)?.tagName;
      if (e.code === "Space" && tag !== "INPUT" && tag !== "TEXTAREA") {
        e.preventDefault();
        onStop();
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [onStop]);
}

/** Big emergency stop. Also triggered by the Space key (unless typing in a field). */
export default function StopAllButton({ onStop, disabled }: { onStop: () => void; disabled?: boolean }) {
  useStopKey(onStop);

  return (
    <button
      onClick={onStop}
      disabled={disabled}
      className="rounded-lg bg-red-600 px-5 py-2.5 font-bold tracking-wide text-white shadow-lg shadow-red-600/30 hover:bg-red-500 active:scale-95 disabled:opacity-50"
      title="Stop all spinning servos (Space)"
    >
      STOP ALL <span className="ml-1 text-xs font-normal opacity-80">(Space)</span>
    </button>
  );
}
