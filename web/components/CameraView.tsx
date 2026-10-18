"use client";

import { useCallback, useEffect, useRef, useState } from "react";

// A glance for the driver, not a video: low on purpose to spare Wi-Fi and Pi.
const CAMERA_FPS = 2;
const FRAME_INTERVAL_MS = 1000 / CAMERA_FPS;
const SHOW_KEY = "talos.cameraShown";

function cameraUrl() {
  // Same host as the page; for `npm run dev` follow NEXT_PUBLIC_BRIDGE_URL
  // (ws://<pi>.local:8000/ws) like the WebSocket does.
  const bridge = process.env.NEXT_PUBLIC_BRIDGE_URL;
  if (bridge) return new URL("/api/camera.jpg", bridge.replace(/^ws/, "http")).toString();
  return "/api/camera.jpg";
}

/** Show or hide the camera, remembered per browser. */
function useShown() {
  const [shown, setShown] = useState(true);
  useEffect(() => {
    try {
      if (window.localStorage.getItem(SHOW_KEY) === "false") setShown(false);
    } catch {}
  }, []);
  const toggle = useCallback(() => {
    setShown((was) => {
      try {
        window.localStorage.setItem(SHOW_KEY, String(!was));
      } catch {}
      return !was;
    });
  }, []);
  return [shown, toggle] as const;
}

function CameraFrame() {
  // A new stamp makes the browser fetch the next frame. The next one is asked
  // for only after this one loaded or failed, so a slow phone never piles up
  // requests. The server answers 204 while the camera is off, which the image
  // reports as an error.
  const [stamp, setStamp] = useState(() => Date.now());
  const [live, setLive] = useState(false);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  useEffect(() => () => clearTimeout(timer.current ?? undefined), []);
  const next = useCallback((ok: boolean) => {
    setLive(ok);
    const wait = () => {
      timer.current = setTimeout(() => {
        if (document.hidden) wait();
        else setStamp(Date.now());
      }, FRAME_INTERVAL_MS);
    };
    wait();
  }, []);

  return (
    <div className="mt-2 flex aspect-video items-center justify-center overflow-hidden rounded-xl bg-black/5 dark:bg-white/5">
      {/* eslint-disable-next-line @next/next/no-img-element -- live frames from the robot */}
      <img
        src={`${cameraUrl()}?t=${stamp}`}
        alt="Ceiling camera"
        onLoad={() => next(true)}
        onError={() => next(false)}
        className={live ? "h-full w-full object-cover" : "hidden"}
      />
      {!live && (
        <span className="px-4 text-center text-sm text-black/60 dark:text-white/60">
          No camera: start the robot with camera:=true
        </span>
      )}
    </div>
  );
}

export default function CameraView() {
  const [shown, toggle] = useShown();
  return (
    <section className="rounded-2xl border border-black/10 p-3 dark:border-white/15">
      <button
        type="button"
        onClick={toggle}
        aria-expanded={shown}
        className="flex w-full items-center justify-between text-sm font-semibold"
      >
        Camera
        <span className="text-xs font-normal text-black/60 dark:text-white/60">
          {shown ? `Hide (${CAMERA_FPS} fps)` : "Show"}
        </span>
      </button>
      {shown && <CameraFrame />}
    </section>
  );
}
