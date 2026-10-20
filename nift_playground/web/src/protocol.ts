/*
 * File: protocol.ts
 *
 * Description:
 *   The messages between the page and the sim worker (the Pyodide worker in src/worker/, or the
 *   mock worker in src/mock/). This is the TypeScript mirror of ARCH.md section 8, which owns the
 *   contract. Python speaks the same JSON in playground/session.py.
 *
 *   Units are SI and the map frame (x east, y north, metres; yaw in radians, counter-clockwise
 *   from +x). Time t is sim time in seconds, starting at 0.
 */

export type XY = [number, number]

/** trajectory_controller's ControlMode (nift_control/modes/mode_base.py) */
export type ControlMode = 'IDLE' | 'AUTONOMOUS' | 'MANUAL' | 'ESTOP' | 'SAFE_STOP'

/** /nav_telemetry STATUS */
export type NavStatus = 'IDLE' | 'NAVIGATING' | 'MANUAL' | 'SAFE_STOP' | 'ESTOP'

/** localization_filter's status level (nift_sensor/localization_filter.py) */
export type LocStatus = 'OK' | 'DEGRADED' | 'LOST'

export type Gear = 'D' | 'R' | 'N'

export type LogLevel = 'debug' | 'info' | 'warn' | 'error' | 'fatal'

export type CmdName = 'start' | 'stop' | 'estop' | 'resume' | 'manual' | 'dbw_estop' | 'power_cycle'

export interface Pose2D {
  x: number
  y: number
  yaw: number
}

/** One diagnostic_msgs/DiagnosticStatus, flattened. level: 0 OK, 1 WARN, 2 ERROR, 3 STALE. */
export interface DiagItem {
  name: string
  hardwareId: string
  level: 0 | 1 | 2 | 3
  message: string
  values: Record<string, string>
}

export interface Obstacle {
  oid: string
  poly: XY[]
}

/** Launch options (ARCH.md 6.4). Kept open here; the runner owns the keys. */
export type LaunchOptions = Record<string, unknown>

/** A shareable run (ARCH.md 8.5) */
export interface Scenario {
  v: 1
  launch: LaunchOptions
  events: { tick: number; msg: PageToWorker }[]
  note?: string
}

/**
 * Any page message may carry an id; the worker answers it with a reply. Obstacles are named by
 * `oid` everywhere (ARCH.md 8.3, 2026-10-20), so the two never clash.
 */
interface WithId {
  id?: string
}

// ---------------------------------------------------------------- page -> worker (ARCH.md 8.1)

export interface InitMsg extends WithId {
  type: 'init'
  baseUrl: string
  mapUrl: string
  bundleUrl?: string // the Python bundle zip; default <baseUrl>py/nift_bundle.zip
  launch?: LaunchOptions
  scenario?: Scenario
  stateHz?: number // default 10
}

export interface GoalMsg extends WithId {
  type: 'goal'
  x: number
  y: number
  yaw?: number
}

export interface CmdMsg extends WithId {
  type: 'cmd'
  name: CmdName
}

export interface TeleopMsg extends WithId {
  type: 'teleop'
  v: number
  steer: number
}

export interface InitialPoseMsg extends WithId {
  type: 'initialpose'
  x: number
  y: number
  yaw: number
}

export interface PauseMsg extends WithId {
  type: 'pause'
}

export interface ResumeSimMsg extends WithId {
  type: 'resume_sim'
}

export interface StepMsg extends WithId {
  type: 'step'
  n?: number // default 1
}

export interface SpeedMsg extends WithId {
  type: 'speed'
  factor: number | 'max' // 0.1 to 100; 1 is real time
}

export interface SetParamMsg extends WithId {
  type: 'set_param'
  node: string
  name: string
  value: unknown
}

export interface ObstacleAddMsg extends WithId {
  type: 'obstacle_add'
  oid: string
  poly?: XY[]
  box?: { x: number; y: number; yaw: number; length: number; width: number }
}

export interface ObstacleMoveMsg extends WithId {
  type: 'obstacle_move'
  oid: string
  x: number
  y: number
  yaw?: number
}

export interface ObstacleRemoveMsg extends WithId {
  type: 'obstacle_remove'
  oid: string
}

export interface SetObstaclesMsg extends WithId {
  type: 'set_obstacles'
  obstacles: Obstacle[]
}

export interface GnssFaultMsg extends WithId {
  type: 'gnss_fault'
  preset: string
  params?: Record<string, unknown>
}

export interface ResetMsg extends WithId {
  type: 'reset'
  launch?: LaunchOptions
}

export interface ScenarioSaveMsg extends WithId {
  type: 'scenario_save'
}

export interface ScenarioLoadMsg extends WithId {
  type: 'scenario_load'
  scenario: Scenario
}

export type PageToWorker =
  | InitMsg
  | GoalMsg
  | CmdMsg
  | TeleopMsg
  | InitialPoseMsg
  | PauseMsg
  | ResumeSimMsg
  | StepMsg
  | SpeedMsg
  | SetParamMsg
  | ObstacleAddMsg
  | ObstacleMoveMsg
  | ObstacleRemoveMsg
  | SetObstaclesMsg
  | GnssFaultMsg
  | ResetMsg
  | ScenarioSaveMsg
  | ScenarioLoadMsg

// ---------------------------------------------------------------- worker -> page (ARCH.md 8.2)

export interface ProgressMsg {
  type: 'progress'
  stage: string // pyodide, packages, bundle, map, nodes, ready
  pct: number // 0 to 100
}

/**
 * A GNSS fault preset (ARCH.md 8.4). 'scheduled' is a scenario of gnss_faults.yaml, applied with a
 * reset (launch option gnss_faults). 'sim' and 'live' kinds apply at once with gnss_fault.
 */
export interface ScheduledFaultPreset {
  name: string
  kind: 'scheduled'
  route?: string | null
  faults: Record<string, unknown>[]
  target?: Record<string, unknown> | null
}

export interface LiveFaultPreset {
  name: string
  kind: 'sim' | 'link'
  params: Record<string, number | string>
  doc?: string
}

export type FaultPreset = ScheduledFaultPreset | LiveFaultPreset

/** A route of nift_carla/config/test_routes.yaml. spawn and goal are [x, y, yaw]. */
export interface RoutePreset {
  id: string
  name?: string | null
  description?: string | null
  spawn: [number, number, number]
  goal: [number, number, number]
  timeout_s?: number | null
}

export interface ReadyMsg {
  type: 'ready'
  pyodide: string | null
  python: string | null
  launch: Record<string, unknown>
  presets: {
    gnss_faults: FaultPreset[]
    gnss_dead_zones?: string[]
    routes: RoutePreset[]
  }
}

export interface StateMsg {
  type: 'state'
  t: number
  tick: number
  paused: boolean
  factor: number | 'max'
  rtf: number
  pose: Pose2D // ground truth
  poseEst: Pose2D | null // what the stack believes (TF map -> base_link)
  speed: number // m/s
  steer: number // front wheel angle, rad
  // null until the first /nav_telemetry: the controller starts in IDLE but publishes nothing until
  // it leaves IDLE once (control_modes.py IdleMode.on_enter). The RViz panel enables Start then too.
  status: NavStatus | null
  mode: ControlMode | null
  targetSpeed: number | null
  cte: number | null
  distEnd: number | null
  locStatus: LocStatus | null // null in direct mode
  locMessage: string | null
  gear: Gear
  ecu: string // /carla/dbw_state mode (no_command, active, timeout, estop); SimClient upper-cases it
  pedal: number // 0..1
  brake: number // 0..1
  gnss: { ok: boolean; stale: boolean; fixAge: number | null }
  goal: XY | null
}

export interface PathMsg {
  type: 'path'
  points: XY[] // empty clears the path
  t: number
}

export interface GoalAckMsg {
  type: 'goal_ack'
  x: number
  y: number
  ok: boolean
  reason?: string
}

export interface LogMsg {
  type: 'log'
  t: number
  level: LogLevel
  node: string
  text: string
}

export interface DiagMsg {
  type: 'diag'
  items: DiagItem[]
}

export interface ObstaclesMsg {
  type: 'obstacles'
  polygons: Obstacle[] // world, map frame
  perceived: XY[][] | null // the last /obstacles in map frame, null without perception
}

export interface EventMsg {
  type: 'event'
  kind: 'gnss_fault' | 'gps_health' | 'mode' | 'arrived'
  t: number
  data: unknown
}

export interface ReplyMsg {
  type: 'reply'
  id: string
  ok: boolean
  message: string
}

export interface ScenarioMsg {
  type: 'scenario'
  scenario: Scenario
}

export interface ErrorMsg {
  type: 'error'
  message: string
  traceback: string
  fatal: boolean
}

export type WorkerToPage =
  | ProgressMsg
  | ReadyMsg
  | StateMsg
  | PathMsg
  | GoalAckMsg
  | LogMsg
  | DiagMsg
  | ObstaclesMsg
  | EventMsg
  | ReplyMsg
  | ScenarioMsg
  | ErrorMsg

/** Narrowing helper for handlers: isMsg(m, 'state') */
export function isMsg<T extends WorkerToPage['type']>(
  m: WorkerToPage,
  type: T,
): m is Extract<WorkerToPage, { type: T }> {
  return m.type === type
}
