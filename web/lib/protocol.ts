// Message types shared with src/talos_servo/web/server.py

/** Fixed per joint in config/robot.yaml. */
export type Control = "position" | "speed";

/** Who sent a joint its last move, spin or stop (SOURCE_* in robot.py). */
export type CommandSource = "web" | "cli" | "ros" | "robot";

export interface LastCommand {
  source: CommandSource;
  /** Browser connection id for "web"; null for the web server's own safety stop. */
  client: string | null;
  /** Seconds since the command, as of this message. */
  age: number;
}

export interface ServoState {
  id: number;
  name: string;
  min_pos: number;
  max_pos: number;
  online: boolean;
  control: Control;
  torque: boolean;
  position: number;
  speed: number;
  load: number;
  voltage: number;
  temp: number;
  current: number;
  moving: boolean;
  target: number | null;
  target_speed: number;
  error: string | null;
  last_command: LastCommand | null;
}

export interface Limits {
  pos_max: number;
  speed_max: number;
  acc_max: number;
  /** Base speed limits for drive (m/s, rad/s); null when the config has no base. */
  linear_max: number | null;
  angular_max: number | null;
}

/** Shared by every browser, kept by the server until it restarts. */
export interface ServoSettings {
  move_speed: number;
  acc: number;
}

export type ServerMessage =
  | { type: "hello"; client_id: string }
  | {
      type: "state";
      ros: boolean;
      limits: Limits;
      settings: Record<string, ServoSettings>;
      servos: ServoState[];
    }
  | { type: "error"; message: string };

export type ClientMessage =
  | { type: "move"; joint: string; position: number; speed: number; acc: number }
  | { type: "spin"; joint: string; speed: number; acc: number }
  | { type: "torque"; joint: string; enabled: boolean }
  | { type: "stop"; joint: string }
  | { type: "stop_all" }
  /** Body velocity (m/s, m/s, rad/s), ROS only. Re-send while held: see useDrive. */
  | { type: "drive"; vx: number; vy: number; wz: number }
  | { type: "settings"; joint: string; move_speed?: number; acc?: number };

export const STEPS_PER_TURN = 4096;
export const stepsToDeg = (steps: number) => (steps * 360) / STEPS_PER_TURN;

/** Same low defaults as the server, used until its first state message arrives. */
export const DEFAULT_SETTINGS: ServoSettings = { move_speed: 200, acc: 20 };
