/*
 * File: icons.tsx
 *
 * Description:
 *   Small line icons for the map tool bar and the header, drawn in currentColor on a 16 px grid.
 */

import type { ReactNode } from 'react'

function Icon({ children, className = 'w-4 h-4' }: { children: ReactNode; className?: string }) {
  return (
    <svg viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth={1.5} strokeLinecap="round" strokeLinejoin="round" className={className} aria-hidden="true">
      {children}
    </svg>
  )
}

/** A flag on a pole: 2D Goal. */
export function GoalIcon() {
  return (
    <Icon>
      <path d="M4 14.5V2" />
      <path d="M4 2.5h8l-2 2.75L12 8H4" fill="currentColor" fillOpacity={0.25} />
    </Icon>
  )
}

/** A dot with a heading arrow: Set pose. */
export function PoseIcon() {
  return (
    <Icon>
      <circle cx="4.5" cy="11.5" r="2" fill="currentColor" fillOpacity={0.25} />
      <path d="M6 10 13 3" />
      <path d="M8.5 3H13v4.5" />
    </Icon>
  )
}

/** A box: Obstacle. */
export function ObstacleIcon() {
  return (
    <Icon>
      <path d="M8 1.75 14 5v6l-6 3.25L2 11V5z" fill="currentColor" fillOpacity={0.15} />
      <path d="M2 5l6 3.25L14 5M8 8.25v6" />
    </Icon>
  )
}

/** A square seen from above: the top-down camera. */
export function TopDownIcon() {
  return (
    <Icon>
      <rect x="2.5" y="2.5" width="11" height="11" rx="1" />
      <path d="M2.5 8h11M8 2.5v11" strokeOpacity={0.5} />
    </Icon>
  )
}

/** A tilted plane: the 3D camera. */
export function PerspIcon() {
  return (
    <Icon>
      <path d="M4.5 4h7l3 9h-13z" />
      <path d="M8 4v9M3 8.5h10" strokeOpacity={0.5} />
    </Icon>
  )
}

/** Four corners: fit the map. */
export function FitIcon() {
  return (
    <Icon>
      <path d="M2 6V2h4M10 2h4v4M14 10v4h-4M6 14H2v-4" />
    </Icon>
  )
}

/** A target: follow the shuttle. */
export function FollowIcon() {
  return (
    <Icon>
      <circle cx="8" cy="8" r="5" />
      <circle cx="8" cy="8" r="1.5" fill="currentColor" />
      <path d="M8 1v2M8 13v2M1 8h2M13 8h2" />
    </Icon>
  )
}

/** A cog with eight teeth: the settings window. */
export function SettingsIcon({ className = 'w-5 h-5' }: { className?: string }) {
  const teeth = Array.from({ length: 8 }, (_, i) => {
    const a = (i * Math.PI) / 4
    const p = (r: number, da: number) => `${(8 + r * Math.cos(a + da)).toFixed(2)} ${(8 + r * Math.sin(a + da)).toFixed(2)}`
    return `L${p(5, -0.3)} L${p(6.75, -0.2)} L${p(6.75, 0.2)} L${p(5, 0.3)}`
  })
  const d = `M${teeth.join(' ').slice(1)} Z`
  return (
    <Icon className={className}>
      <path d={d} />
      <circle cx="8" cy="8" r="2" />
    </Icon>
  )
}

/** A pointer arrow: the Select tool (no map tool). */
export function SelectIcon() {
  return (
    <Icon>
      <path d="M3.5 2.5 12 8.25l-3.75.75L10.5 13.5l-1.75.9-2.25-4.4L3.5 12.5z" fill="currentColor" fillOpacity={0.2} />
    </Icon>
  )
}

export function PlayIcon() {
  return (
    <Icon>
      <path d="M5 3v10l8-5z" fill="currentColor" />
    </Icon>
  )
}

export function PauseIcon() {
  return (
    <Icon>
      <path d="M5 3v10M11 3v10" strokeWidth={2.5} />
    </Icon>
  )
}

/** Play to a bar: step n ticks. */
export function StepIcon() {
  return (
    <Icon>
      <path d="M3.5 3.5v9l6.5-4.5z" fill="currentColor" />
      <path d="M12.5 3.5v9" strokeWidth={2} />
    </Icon>
  )
}

/** A circular arrow: reset. */
export function ResetIcon() {
  return (
    <Icon>
      <path d="M3 8a5 5 0 1 0 1.6-3.7" />
      <path d="M3 2.5v3h3" />
    </Icon>
  )
}

/** Three linked nodes: the stack (launcher) window. */
export function StackIcon({ className = 'w-5 h-5' }: { className?: string }) {
  return (
    <Icon className={className}>
      <circle cx="3.5" cy="12" r="1.75" />
      <circle cx="8" cy="4" r="1.75" />
      <circle cx="12.5" cy="12" r="1.75" />
      <path d="M4.4 10.5 7.1 5.5M8.9 5.5l2.7 5M5.25 12h5.5" />
    </Icon>
  )
}

/** A light bulb: the guided tour. */
export function TipsIcon({ className = 'w-5 h-5' }: { className?: string }) {
  return (
    <Icon className={className}>
      <path d="M6 11.5c0-1.5-2.5-2.75-2.5-5.25a4.5 4.5 0 0 1 9 0C12.5 8.75 10 10 10 11.5z" />
      <path d="M6.25 13.5h3.5M7 15h2" />
    </Icon>
  )
}

// GNSS fault kinds of gnss_faults.yaml, one icon each (ScenariosPane.tsx)

/** A satellite dish with a cross: no fixes. */
export function FaultDropoutIcon() {
  return (
    <Icon>
      <path d="M2.5 9.5a5 5 0 0 0 5 5" />
      <path d="M2.5 9.5 7.5 14.5" />
      <path d="M5 12l2.5-2.5" />
      <path d="M10 2.5l4 4M14 2.5l-4 4" />
    </Icon>
  )
}

/** Signal bars that break off: short bursts of loss. */
export function FaultBurstIcon() {
  return (
    <Icon>
      <path d="M2.5 13.5v-2M5.5 13.5v-4.5M8.5 13.5V7" />
      <path d="M11.5 13.5v-1M14 13.5v-1" strokeOpacity={0.4} />
      <path d="M11 4.5l3 3M14 4.5l-3 3" />
    </Icon>
  )
}

/** A clock: stamps or delivery late. */
export function FaultClockIcon() {
  return (
    <Icon>
      <circle cx="8" cy="8" r="5.75" />
      <path d="M8 4.75V8l2.25 1.5" />
    </Icon>
  )
}

/** A dot that jumps: the fix offset. */
export function FaultJumpIcon() {
  return (
    <Icon>
      <circle cx="3.75" cy="12" r="1.5" strokeOpacity={0.45} />
      <circle cx="12" cy="4" r="1.75" fill="currentColor" />
      <path d="M4.75 10.75C6 6 8.5 4.5 10.25 4.25" strokeDasharray="1.5 1.5" />
    </Icon>
  )
}

/** Scattered dots: noisy fixes. */
export function FaultNoiseIcon() {
  return (
    <Icon>
      {[
        [8, 8],
        [4.5, 5],
        [11.5, 4.5],
        [4, 11],
        [11, 11.5],
        [8, 3],
        [13, 8],
      ].map(([x, y], i) => (
        <circle key={i} cx={x} cy={y} r="1" fill="currentColor" stroke="none" />
      ))}
    </Icon>
  )
}

/** A pause sign in a box: the feed stuck on one epoch, or frozen. */
export function FaultStuckIcon() {
  return (
    <Icon>
      <rect x="2.5" y="2.5" width="11" height="11" rx="2" />
      <path d="M6.5 5.5v5M9.5 5.5v5" />
    </Icon>
  )
}

/** An hourglass: fixes delivered late. */
export function FaultDelayIcon() {
  return (
    <Icon>
      <path d="M4 2.5h8M4 13.5h8" />
      <path d="M5 2.5c0 3 6 3.5 6 5.5s-6 2.5-6 5.5M11 2.5c0 3-6 3.5-6 5.5s6 2.5 6 5.5" />
    </Icon>
  )
}

/** Two crossing arrows: front and rear antennas swapped. */
export function FaultSwapIcon() {
  return (
    <Icon>
      <path d="M2.5 5h9.5M9.5 2.5 12 5 9.5 7.5" />
      <path d="M13.5 11H4M6.5 8.5 4 11l2.5 2.5" />
    </Icon>
  )
}

/** A check in a circle: a control run without faults. */
export function FaultNoneIcon() {
  return (
    <Icon>
      <circle cx="8" cy="8" r="5.75" />
      <path d="M5.5 8.25 7.25 10l3.25-3.75" />
    </Icon>
  )
}
