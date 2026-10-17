"use client";

import { useEffect, useRef, useState } from "react";

/**
 * A control value that follows the server, except while the user holds the control.
 *
 * Spread `holdProps` on the input. While held (pointer down, or a text field focused) server
 * updates are ignored so nothing jumps under the user's hand; once released, the next server
 * change takes over again. A `null` server value (nothing commanded yet) is ignored.
 */
export function useFollow(server: number | null, initial: number) {
  const [value, setValue] = useState(server ?? initial);
  const held = useRef(false);

  useEffect(() => {
    if (!held.current && server !== null) setValue(server);
  }, [server]);

  const hold = () => {
    held.current = true;
  };
  const release = () => {
    held.current = false;
  };
  const holdProps = {
    onPointerDown: hold,
    onPointerUp: release,
    onPointerCancel: release,
    onBlur: release,
  };
  return { value, setValue, hold, release, holdProps };
}
