import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Robot Servo Control",
  description: "Position and speed control for the robot's servos",
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="en">
      <body className="antialiased">{children}</body>
    </html>
  );
}
