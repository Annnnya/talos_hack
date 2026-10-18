"use client";

/** Speed as a percent of the config caps, for the joystick and climbing. */
export default function SpeedSlider({
  percent,
  onChange,
  linearMax,
  climbMax,
}: {
  percent: number;
  onChange: (percent: number) => void;
  linearMax: number | null;
  climbMax: number;
}) {
  const cm = (max: number) => ((max * percent) / 100) * 100;
  return (
    <label className="flex flex-col gap-1 text-sm">
      <span className="flex justify-between">
        <span className="font-semibold">Speed {percent}%</span>
        <span className="text-black/60 dark:text-white/60">
          drive {linearMax === null ? "-" : cm(linearMax).toFixed(1)} cm/s, climb {cm(climbMax).toFixed(1)} cm/s
        </span>
      </span>
      <input
        type="range"
        min={5}
        max={100}
        step={5}
        value={percent}
        onChange={(e) => onChange(Number(e.target.value))}
        className="w-full accent-violet-600"
      />
    </label>
  );
}
