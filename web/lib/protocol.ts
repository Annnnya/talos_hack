// Message types shared with bridge/main.py

export type Mode = "position" | "wheel";

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
  target: number | null;
  target_speed: number;
  error: string | null;
}

export interface Limits {
  pos_max: number;
  speed_max: number;
  acc_max: number;
}

export type ServerMessage =
  | { type: "state"; mock: boolean; limits: Limits; servos: ServoState[] }
  | { type: "error"; message: string };

export type ClientMessage =
  | { type: "set_mode"; id: number; mode: Mode }
  | { type: "move"; id: number; position: number; speed: number; acc: number }
  | { type: "spin"; id: number; speed: number; acc: number }
  | { type: "torque"; id: number; enabled: boolean }
  | { type: "stop"; id: number }
  | { type: "stop_all" };

export const STEPS_PER_TURN = 4096;
export const stepsToDeg = (steps: number) => (steps * 360) / STEPS_PER_TURN;
