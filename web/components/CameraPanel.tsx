"use client";

import Image from "next/image";
import { cameraFrameUrl } from "@/lib/useBridge";

interface CameraPanelProps {
  connected: boolean;
  streaming: boolean | null;
  frameSequence: number | null;
  onToggle: () => void;
}

export default function CameraPanel({
  connected,
  streaming,
  frameSequence,
  onToggle,
}: CameraPanelProps) {
  const isStreaming = streaming === true;
  const buttonLabel = isStreaming ? "Stop video" : "Start video";

  return (
    <section className="w-full space-y-2">
      <button
        type="button"
        onClick={onToggle}
        disabled={!connected}
        aria-label={`${buttonLabel} streaming`}
        aria-pressed={isStreaming}
        title={`${buttonLabel} streaming`}
        className={`flex w-full items-center justify-center gap-2 rounded-2xl px-4 py-3 font-semibold shadow-md active:scale-95 disabled:opacity-40 ${
          isStreaming ? "bg-red-600 text-white" : "bg-emerald-600 text-white"
        }`}
      >
        <svg
          aria-hidden="true"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="1.8"
          className="h-5 w-5"
        >
          <rect x="3" y="6" width="13" height="12" rx="2" />
          <path d="m16 10 5-3v10l-5-3" />
        </svg>
        {buttonLabel}
      </button>
      {isStreaming && (
        <div className="flex aspect-video w-full items-center justify-center overflow-hidden rounded-xl bg-black">
          {frameSequence === null ? (
            <p className="text-sm text-white/70" aria-live="polite">
              Waiting for video frame…
            </p>
          ) : (
            <Image
              src={cameraFrameUrl(frameSequence)}
              alt="Live camera stream"
              width={640}
              height={360}
              sizes="(max-width: 448px) 100vw, 448px"
              unoptimized
              className="h-full w-full object-contain"
            />
          )}
        </div>
      )}
    </section>
  );
}
