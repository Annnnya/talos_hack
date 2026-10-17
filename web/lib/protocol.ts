// Message types shared with src/talos_servo/web/server.py

export type Mode = "position" | "wheel";

/** Who sent a servo its last move, spin or stop (SOURCE_* in controller.py). */
export type CommandSource = "web" | "cli" | "ros" | "bridge";

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
  mode: Mode;
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
      mock: boolean;
      ros: boolean;
      limits: Limits;
      settings: Record<string, ServoSettings>;
      servos: ServoState[];
    }
  | { type: "error"; message: string };

export type ClientMessage =
  | { type: "set_mode"; id: number; mode: Mode }
  | { type: "move"; id: number; position: number; speed: number; acc: number }
  | { type: "spin"; id: number; speed: number; acc: number }
  | { type: "torque"; id: number; enabled: boolean }
  | { type: "stop"; id: number }
  | { type: "stop_all" }
  | { type: "settings"; id: number; move_speed?: number; acc?: number };

export const STEPS_PER_TURN = 4096;
export const stepsToDeg = (steps: number) => (steps * 360) / STEPS_PER_TURN;

/** Same low defaults as the server, used until its first state message arrives. */
export const DEFAULT_SETTINGS: ServoSettings = { move_speed: 200, acc: 20 };
