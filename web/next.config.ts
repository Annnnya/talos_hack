import type { NextConfig } from "next";

// Static export: `npm run build` writes web/out, which the Python bridge serves.
const nextConfig: NextConfig = {
  output: "export",
  images: { unoptimized: true },
  // Write each page as `<route>/index.html`: the Python static server only
  // finds `/debug` as a folder, not as `debug.html`.
  trailingSlash: true,
};

export default nextConfig;
