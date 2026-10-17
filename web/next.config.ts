import type { NextConfig } from "next";

// Static export: `npm run build` writes web/out, which the Python bridge serves.
const nextConfig: NextConfig = {
  output: "export",
  images: { unoptimized: true },
};

export default nextConfig;
