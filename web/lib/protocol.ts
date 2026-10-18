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
  /** The servo's own protection flag (overload, overheat) while it answers. */
  fault: string | null;
  last_command: LastCommand | null;
}

/** Robot modes from the mode node (src/talos_servo/modes.py). */
export type Mode = "free" | "aligning" | "clamping" | "climbing";

export interface Limits {
  pos_max: number;
  speed_max: number;
  acc_max: number;
  /** Base speed limits for drive (m/s, rad/s); null when the config has no base. */
  linear_max: number | null;
  angular_max: number | null;
  /** m/s cap of the Up and Down buttons. */
  climb_max: number;
}

/** Hinge readout for calibrating model.bend_flat_steps (ROS only). */
export interface HingeState {
  joint: string;
  /** Opening angle, 180 flat; null while the bridge cannot tell. */
  angle_deg: number | null;
  /** Raw servo position as `talos-servo monitor` shows it; null while unknown. */
  raw_steps: number | null;
  /** model.bend_flat_steps in config/robot.yaml. */
  flat_steps: number | null;
  min_deg: number;
  max_deg: number;
  /** Opening speed of the hold buttons, rad/s. */
  max_rate: number;
  /** Hinge current (A, signed), fresh every control tick; null until heard. */
  current_a: number | null;
  /** Online wheel currents (A, signed) by joint name, about 5 Hz. */
  wheel_currents: Record<string, number> | null;
}

/** Hinge current hold in the expansion node (#142). */
export interface HoldState {
  active: boolean;
  setpoint_a: number;
  /** Filtered |hinge current| the loop runs on, A. */
  measured_a: number;
  /** How far the goal leads the measured angle, rad. */
  offset_rad: number;
  max_current_a: number;
  /** Why it stopped last ("no wall contact", "turned off", ...). */
  note: string | null;
  /** Hinge servo load in %, null when stale. */
  load_pct: number | null;
  /** True while the load limit backs the goal off. */
  load_limited: boolean;
}

/** Platform roll and pitch from the IMU (#91), degrees. */
export interface TiltState {
  roll_deg: number;
  pitch_deg: number;
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
      /** True while STOP locks twist_mux; null without ROS. */
      stopped: boolean | null;
      /** From the mode node; null until heard or without ROS. */
      mode: Mode | null;
      mode_reason: string | null;
      /** twist_mux input driving now: "web", "keyboard", "none" or "locked". */
      active_input: string | null;
      /** Null without ROS or without a hinge. */
      hinge: HingeState | null;
      /** Null without the expansion node. */
      hinge_hold: HoldState | null;
      /** Null without ROS or while the IMU chain is quiet. */
      tilt: TiltState | null;
      /** Null without the camera node. */
      camera_streaming: boolean | null;
      /** Null until the camera node publishes a frame. */
      camera_frame_sequence: number | null;
    }
  | { type: "error"; message: string };

export type ClientMessage =
  | { type: "move"; joint: string; position: number; speed: number; acc: number }
  | { type: "spin"; joint: string; speed: number; acc: number }
  | { type: "torque"; joint: string; enabled: boolean }
  | { type: "stop"; joint: string }
  | { type: "stop_all" }
  /** Release STOP so twist_mux drives again, ROS only. */
  | { type: "release_stop" }
  /** Ask the mode node to start aligning (free only), ROS only. */
  | { type: "wall_nearby" }
  /** Release the walls and go back to free, ROS only. */
  | { type: "release" }
  /** Body velocity (m/s, m/s, rad/s), ROS only. Re-send while held: see useDrive. */
  | { type: "drive"; vx: number; vy: number; wz: number }
  /** Hinge opening speed (rad/s, + opens, 0 stops), ROS only. Re-send while held. */
  | { type: "hinge"; rate: number }
  /** Climb speed (m/s, + up, 0 stops), ROS only. Re-send while held. */
  | { type: "climb"; speed: number }
  /** Sideways speed on the wall (m/s, + left, 0 stops), ROS only. Re-send while held. */
  | { type: "wall_side"; speed: number }
  /** Hinge current hold on/off with its setpoint (A), ROS only. */
  | { type: "hinge_hold"; enabled: boolean; setpoint: number }
  /** Enable or disable the camera stream, ROS only. */
  | { type: "camera_stream"; enabled: boolean }
  | { type: "settings"; joint: string; move_speed?: number; acc?: number };

export const STEPS_PER_TURN = 4096;
export const stepsToDeg = (steps: number) => (steps * 360) / STEPS_PER_TURN;

/** Same low defaults as the server, used until its first state message arrives. */
export const DEFAULT_SETTINGS: ServoSettings = { move_speed: 200, acc: 20 };
