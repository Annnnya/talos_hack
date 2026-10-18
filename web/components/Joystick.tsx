"use client";

import type { PointerEvent } from "react";
import { type Direction, KEY_FOR } from "@/lib/joystick";

const LABELS: Record<Direction, { icon: string; name: string }> = {
  up: { icon: "↑", name: "Forward" },
  down: { icon: "↓", name: "Backward" },
  left: { icon: "←", name: "Left" },
  right: { icon: "→", name: "Right" },
  ccw: { icon: "↺", name: "Rotate counterclockwise" },
  cw: { icon: "↻", name: "Rotate clockwise" },
};

// Grid cells of the cross: rotate on top, then the arrows around STOP.
const CELLS: Record<Direction, string> = {
  ccw: "col-start-1 row-start-1",
  cw: "col-start-3 row-start-1",
  up: "col-start-2 row-start-2",
  left: "col-start-1 row-start-3",
  right: "col-start-3 row-start-3",
  down: "col-start-2 row-start-4",
};

// Squircle-ish, big touch target, no long-press menu, text selection or
// double-tap zoom.
const BUTTON =
  "flex size-[min(26vw,7rem)] flex-col items-center justify-center rounded-[30%] select-none touch-none " +
  "[-webkit-touch-callout:none] [-webkit-tap-highlight-color:transparent] transition-transform " +
  "disabled:opacity-40";

export default function Joystick({
  active,
  disabled,
  onStart,
  onEnd,
  onStop,
  stopDisabled,
}: {
  active: Direction | null;
  disabled: boolean;
  onStart: (dir: Direction) => void;
  onEnd: (dir: Direction) => void;
  onStop: () => void;
  stopDisabled: boolean;
}) {
  const down = (dir: Direction) => (e: PointerEvent<HTMLButtonElement>) => {
    e.preventDefault();
    // Touch captures the pointer to the button by default, so sliding the
    // finger off would keep driving. Let go of it, so leaving stops.
    if (e.currentTarget.hasPointerCapture(e.pointerId)) {
      e.currentTarget.releasePointerCapture(e.pointerId);
    }
    onStart(dir);
  };
  const up = (dir: Direction) => () => onEnd(dir);

  return (
    <div className="grid grid-cols-3 grid-rows-4 gap-3">
      {(Object.keys(CELLS) as Direction[]).map((dir) => {
        const on = active === dir;
        return (
          <button
            key={dir}
            type="button"
            aria-label={LABELS[dir].name}
            aria-pressed={on}
            title={`${LABELS[dir].name} (${KEY_FOR[dir]})`}
            disabled={disabled}
            onPointerDown={down(dir)}
            onPointerUp={up(dir)}
            onPointerLeave={up(dir)}
            onPointerCancel={up(dir)}
            onContextMenu={(e) => e.preventDefault()}
            className={`${BUTTON} ${CELLS[dir]} ${
              on
                ? "scale-95 bg-blue-600 text-white shadow-inner"
                : "bg-black/5 text-black/80 shadow-md dark:bg-white/10 dark:text-white/90"
            }`}
          >
            <span className="text-4xl leading-none">{LABELS[dir].icon}</span>
            <span className="mt-1 text-xs opacity-60">{KEY_FOR[dir]}</span>
          </button>
        );
      })}
      <button
        type="button"
        aria-label="Stop"
        title="Stop every input (Space)"
        disabled={stopDisabled}
        onPointerDown={(e) => {
          e.preventDefault();
          onStop();
        }}
        onContextMenu={(e) => e.preventDefault()}
        className={`${BUTTON} col-start-2 row-start-3 bg-red-600 font-bold text-white shadow-lg shadow-red-600/30 active:scale-95`}
      >
        <span className="text-xl">STOP</span>
        <span className="mt-1 text-xs font-normal opacity-80">Space</span>
      </button>
    </div>
  );
}
