"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import type { ClientMessage, Limits, ServerMessage, ServoState } from "./protocol";

export type ConnStatus = "connecting" | "connected" | "disconnected";

function bridgeUrl() {
  // When served by the bridge (normal use), connect back to the same host.
  // For `npm run dev`, set NEXT_PUBLIC_BRIDGE_URL=ws://<pi>.local:8000/ws
  if (process.env.NEXT_PUBLIC_BRIDGE_URL) return process.env.NEXT_PUBLIC_BRIDGE_URL;
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${window.location.host}/ws`;
}

export function useBridge() {
  const [status, setStatus] = useState<ConnStatus>("connecting");
  const [servos, setServos] = useState<ServoState[]>([]);
  const [limits, setLimits] = useState<Limits>({ pos_max: 4095, speed_max: 1000, acc_max: 254 });
  const [mock, setMock] = useState(false);
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
        if (msg.type === "state") {
          setServos(msg.servos);
          setLimits(msg.limits);
          setMock(msg.mock);
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

  return { status, servos, limits, mock, lastError, clearError: () => setLastError(null), send };
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
