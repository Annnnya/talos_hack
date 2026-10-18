"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import type {
  ClientMessage,
  HingeState,
  HoldState,
  Limits,
  Mode,
  ServerMessage,
  ServoSettings,
  ServoState,
  TiltState,
} from "./protocol";

export type ConnStatus = "connecting" | "connected" | "disconnected";

function bridgeUrl() {
  // When served by talos-servo web (normal use), connect back to the same host.
  // For `npm run dev`, set NEXT_PUBLIC_BRIDGE_URL=ws://<pi>.local:8000/ws
  if (process.env.NEXT_PUBLIC_BRIDGE_URL) return process.env.NEXT_PUBLIC_BRIDGE_URL;
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${window.location.host}/ws`;
}

export function useBridge() {
  const [status, setStatus] = useState<ConnStatus>("connecting");
  const [servos, setServos] = useState<ServoState[]>([]);
  const [limits, setLimits] = useState<Limits>({
    pos_max: 4095,
    speed_max: 3000,
    acc_max: 254,
    linear_max: null,
    angular_max: null,
    climb_max: 0,
  });
  const [ros, setRos] = useState(false);
  const [stopped, setStopped] = useState<boolean | null>(null);
  const [mode, setMode] = useState<Mode | null>(null);
  const [modeReason, setModeReason] = useState<string | null>(null);
  const [activeInput, setActiveInput] = useState<string | null>(null);
  const [hinge, setHinge] = useState<HingeState | null>(null);
  const [hold, setHold] = useState<HoldState | null>(null);
  const [tilt, setTilt] = useState<TiltState | null>(null);
  const [settings, setSettings] = useState<Record<string, ServoSettings>>({});
  const [clientId, setClientId] = useState<string | null>(null);
  const [lastError, setLastError] = useState<string | null>(null);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    let stopped = false;
    let retry: ReturnType<typeof setTimeout>;

    const connect = () => {
      setStatus("connecting");
      const ws = new WebSocket(bridgeUrl());
      wsRef.current = ws;
      ws.onopen = () => setStatus("connected");
      ws.onmessage = (ev) => {
        const msg = JSON.parse(ev.data) as ServerMessage;
        if (msg.type === "hello") {
          setClientId(msg.client_id);
        } else if (msg.type === "state") {
          setServos(msg.servos);
          setLimits(msg.limits);
          setRos(msg.ros);
          setStopped(msg.stopped);
          setMode(msg.mode);
          setModeReason(msg.mode_reason);
          setActiveInput(msg.active_input);
          setHinge(msg.hinge ?? null);
          setHold(msg.hinge_hold ?? null);
          setTilt(msg.tilt ?? null);
          setSettings(msg.settings);
        } else if (msg.type === "error") {
          setLastError(msg.message);
        }
      };
      ws.onclose = () => {
        wsRef.current = null;
        setStatus("disconnected");
        if (!stopped) retry = setTimeout(connect, 1000);
      };
      ws.onerror = () => ws.close();
    };

    connect();
    return () => {
      stopped = true;
      clearTimeout(retry);
      wsRef.current?.close();
    };
  }, []);

  const send = useCallback((msg: ClientMessage) => {
    const ws = wsRef.current;
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
  }, []);

  return {
    status,
    servos,
    limits,
    ros,
    stopped,
    mode,
    modeReason,
    activeInput,
    hinge,
    hold,
    tilt,
    settings,
    clientId,
    lastError,
    clearError: () => setLastError(null),
    send,
  };
}

/** Rate-limit a function (leading + trailing call), used while dragging sliders. */
export function useThrottle<A extends unknown[]>(fn: (...args: A) => void, ms = 50) {
  const last = useRef(0);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const pending = useRef<A | null>(null);
  const fnRef = useRef(fn);
  fnRef.current = fn;

  return useCallback(
    (...args: A) => {
      const now = Date.now();
      const wait = ms - (now - last.current);
      if (wait <= 0) {
        last.current = now;
        fnRef.current(...args);
      } else {
        pending.current = args;
        if (!timer.current) {
          timer.current = setTimeout(() => {
            timer.current = null;
            last.current = Date.now();
            if (pending.current) fnRef.current(...pending.current);
            pending.current = null;
          }, wait);
        }
      }
    },
    [ms],
  );
}
