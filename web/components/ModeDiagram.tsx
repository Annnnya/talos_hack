"use client";

import { useEffect, useId, useState } from "react";
import { MODE_LABEL, MODE_ORDER, MODE_TRANSITIONS } from "@/lib/modes";
import type { Mode } from "@/lib/protocol";

/** Mermaid source for the mode state machine, `mode` highlighted. */
export function modeDiagram(mode: Mode | null): string {
  const lines = ["stateDiagram-v2"];
  for (const m of MODE_ORDER) lines.push(`  state "${MODE_LABEL[m]}" as ${m}`);
  for (const t of MODE_TRANSITIONS) lines.push(`  ${t.from} --> ${t.to}: ${t.label}`);
  if (mode) {
    lines.push("  classDef current fill:#10b981,stroke:#047857,color:#fff,font-weight:bold");
    lines.push(`  class ${mode} current`);
  }
  return lines.join("\n");
}

/** The robot mode state machine, drawn by Mermaid. */
export default function ModeDiagram({ mode }: { mode: Mode | null }) {
  const id = `mode-diagram-${useId().replace(/:/g, "")}`;
  const [svg, setSvg] = useState("");

  useEffect(() => {
    let cancelled = false;
    (async () => {
      const mermaid = (await import("mermaid")).default;
      const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
      mermaid.initialize({ startOnLoad: false, theme: dark ? "dark" : "default" });
      const { svg } = await mermaid.render(id, modeDiagram(mode));
      if (!cancelled) setSvg(svg);
    })().catch((err) => console.error("mode diagram failed", err));
    return () => {
      cancelled = true;
    };
  }, [id, mode]);

  return (
    <figure className="flex flex-col gap-1">
      <div
        role="img"
        aria-label={mode ? `Mode state machine, now ${MODE_LABEL[mode]}` : "Mode state machine"}
        className="flex justify-center [&_svg]:h-auto [&_svg]:max-w-full"
        dangerouslySetInnerHTML={{ __html: svg }}
      />
      <figcaption className="text-center text-xs text-black/50 dark:text-white/50">
        Release (L) goes back to Free from any mode.
      </figcaption>
    </figure>
  );
}
