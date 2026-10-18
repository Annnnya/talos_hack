import Link from "next/link";

// Placeholder until the joystick page (#70) takes over `/`.
export default function Home() {
  return (
    <main className="mx-auto max-w-7xl px-4 py-6 flex flex-col gap-6">
      <header className="flex flex-wrap items-center justify-between gap-4">
        <h1 className="text-2xl font-bold">Talos</h1>
        <Link
          href="/debug"
          className="text-sm text-black/60 underline hover:text-black dark:text-white/60 dark:hover:text-white"
        >
          Per-motor debug
        </Link>
      </header>
      <p className="text-black/60 dark:text-white/60">
        The joystick is coming soon. Use the per-motor debug page for bench
        tests.
      </p>
    </main>
  );
}
