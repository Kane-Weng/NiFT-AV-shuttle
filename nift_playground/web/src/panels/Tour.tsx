/*
 * File: Tour.tsx
 *
 * Description:
 *   The guided tour under the light bulb in the header, like the tips of a game. Each step dims the
 *   page, leaves a hole over the part it explains (which stays clickable, so the user can try it)
 *   and shows a card beside it with Back and Next. The order follows a first run: place the
 *   shuttle, give a goal, start, or drive by hand, then read the telemetry, run the clock, and find
 *   the stack and the settings.
 *
 *   Steps name their target by CSS selector (data-testid or data-tour). A step whose target is not
 *   on the page shows its card in the middle. Escape ends the tour, the arrow keys move through it.
 */

import { useEffect, useLayoutEffect, useState } from 'react'

interface Step {
  /** Selectors of the part to show; the hole covers all of them. None: a card in the middle. */
  target?: string[]
  title: string
  body: string
}

const STEPS: Step[] = [
  {
    title: 'Welcome to the NiFT playground',
    body: 'Our shuttle stack runs here in your browser: the same ROS 2 nodes as on the vehicle, in Python, on a simulated shuttle. This tour shows the main controls. It takes about a minute.',
  },
  {
    target: ['[data-testid="tool-pose"]'],
    title: '1. Set pose',
    body: 'Pick Set pose, then press on a road and drag in the driving direction. The shuttle jumps there and the stack is told where it is, like 2D Pose Estimate in RViz.',
  },
  {
    target: ['[data-testid="tool-goal"]'],
    title: '2. Give a goal',
    body: 'Pick 2D Goal and click where the shuttle should go. Drag to give the heading. The planner answers with a green path along the lanes.',
  },
  {
    target: ['[data-tour="nav-buttons"]'],
    title: '3. Start navigation',
    body: 'Start sends the shuttle along the path. Stop brings it to a Safe Stop, and Resume clears that so you can start again.',
  },
  {
    target: ['[data-testid="manual-section"]'],
    title: 'Or drive it yourself',
    body: 'Enable manual, click the pad, then drive with W A S D or the arrow keys. Space stops.',
  },
  {
    target: ['[data-testid="tool-obstacle"]'],
    title: 'Obstacles',
    body: 'Pick Obstacle and click the road to park a car. Click a car again to take it away. With perception on (Settings, Launch), the shuttle stops for it.',
  },
  {
    target: ['[data-testid="telemetry-panel"]'],
    title: 'Telemetry',
    body: 'Speed against the target speed, the wheel angle, gear, pedal and brake, and the plots of speed and how far the shuttle is from the path.',
  },
  {
    target: ['[data-testid="playground-panel"]'],
    title: 'The sim clock',
    body: 'Pause, step a few ticks at a time, or run up to the speed your computer allows. Reset rebuilds the whole stack.',
  },
  {
    target: ['[data-tour="camera"]', '[data-testid="view-follow"]'],
    title: 'The view',
    body: 'Top-down or 3D. Right drag rotates. Fit shows the whole map, and Follow keeps the shuttle in the middle.',
  },
  {
    target: ['[data-testid="stack-open"]'],
    title: 'The stack',
    body: 'Which nodes run and how healthy they are. Switch a node off to see what stops working without it.',
  },
  {
    target: ['[data-testid="settings-open"]'],
    title: 'Settings and scenarios',
    body: 'Ready-made test routes and GNSS fault scenarios, the 3D world, the ROS log, launch options, parameters, and a link to share your run.',
  },
  {
    target: ['[data-testid="tips-open"]'],
    title: 'That is all',
    body: 'The tour is here whenever you want it again. Have fun breaking things.',
  },
]

type Rect = { x: number; y: number; w: number; h: number }
const PAD = 6
const CARD_W = 300

function rectOf(sels: string[] | undefined): Rect | null {
  if (!sels) return null
  const rs = sels
    .map((s) => document.querySelector(s)?.getBoundingClientRect())
    .filter((r): r is DOMRect => !!r && r.width > 0 && r.height > 0)
  if (!rs.length) return null
  const x = Math.min(...rs.map((r) => r.left)) - PAD
  const y = Math.min(...rs.map((r) => r.top)) - PAD
  return { x, y, w: Math.max(...rs.map((r) => r.right)) + PAD - x, h: Math.max(...rs.map((r) => r.bottom)) + PAD - y }
}

const same = (a: Rect | null, b: Rect | null) => a === b || (!!a && !!b && a.x === b.x && a.y === b.y && a.w === b.w && a.h === b.h)

export default function Tour({ open, onClose }: { open: boolean; onClose: () => void }) {
  const [i, setI] = useState(0)
  const [rect, setRect] = useState<Rect | null>(null)
  const step = STEPS[i]

  useEffect(() => {
    if (open) setI(0)
  }, [open])

  // Bring the target into view, then follow it (layout changes, scrolling, resizing)
  useLayoutEffect(() => {
    if (!open) return
    const el = step.target && document.querySelector(step.target[0])
    if (el) el.scrollIntoView({ block: 'nearest', inline: 'nearest' })
    let raf = 0
    let last: Rect | null = null
    const tick = () => {
      const r = rectOf(step.target)
      if (!same(r, last)) {
        last = r
        setRect(r)
      }
      raf = requestAnimationFrame(tick)
    }
    tick()
    return () => cancelAnimationFrame(raf)
  }, [open, step])

  useEffect(() => {
    if (!open) return
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose()
      else if (e.key === 'ArrowRight' || e.key === 'Enter') i < STEPS.length - 1 ? setI(i + 1) : onClose()
      else if (e.key === 'ArrowLeft' && i > 0) setI(i - 1)
      else return
      e.preventDefault()
      e.stopPropagation()
    }
    window.addEventListener('keydown', onKey, true)
    return () => window.removeEventListener('keydown', onKey, true)
  }, [open, i, onClose])

  if (!open) return null
  const vw = window.innerWidth
  const vh = window.innerHeight

  // The card goes below the hole, else above, else beside it (left first), kept on screen
  let card: { left: number; top: number }
  if (!rect) card = { left: (vw - CARD_W) / 2, top: vh / 2 - 100 }
  else if (rect.y + rect.h + 190 < vh) card = { left: rect.x, top: rect.y + rect.h + 12 }
  else if (rect.y > 200) card = { left: rect.x, top: rect.y - 12 - 180 }
  else if (rect.x > CARD_W + 24) card = { left: rect.x - CARD_W - 12, top: rect.y }
  else card = { left: rect.x + rect.w + 12, top: rect.y }
  card = { left: Math.max(12, Math.min(vw - CARD_W - 12, card.left)), top: Math.max(12, Math.min(vh - 200, card.top)) }

  // Four blockers around the hole: the dim area takes no clicks, the hole does
  const r = rect ?? { x: vw / 2, y: vh / 2, w: 0, h: 0 }
  const block = 'fixed pointer-events-auto'
  return (
    <div className="fixed inset-0 z-[60] pointer-events-none" data-testid="tour">
      <div className={block} style={{ left: 0, top: 0, width: vw, height: Math.max(0, r.y) }} />
      <div className={block} style={{ left: 0, top: r.y + r.h, width: vw, height: Math.max(0, vh - r.y - r.h) }} />
      <div className={block} style={{ left: 0, top: r.y, width: Math.max(0, r.x), height: r.h }} />
      <div className={block} style={{ left: r.x + r.w, top: r.y, width: Math.max(0, vw - r.x - r.w), height: r.h }} />
      <div
        className="fixed rounded-lg transition-all duration-300 ease-out"
        style={{
          left: r.x,
          top: r.y,
          width: r.w,
          height: r.h,
          boxShadow: '0 0 0 9999px rgba(3, 7, 18, 0.62)',
          outline: rect ? '2px solid rgb(56 189 248)' : 'none',
          outlineOffset: 0,
        }}
      />
      <div
        role="dialog"
        aria-label={step.title}
        className="fixed pointer-events-auto rounded-lg border border-sky-500/60 bg-white dark:bg-gray-900 shadow-2xl p-3 transition-all duration-300 ease-out"
        style={{ left: card.left, top: card.top, width: CARD_W }}
      >
        <div className="flex items-start gap-2">
          <div className="text-sm font-semibold flex-1">{step.title}</div>
          <span className="text-[11px] font-mono text-gray-400 tabular-nums">
            {i + 1}/{STEPS.length}
          </span>
        </div>
        <p className="mt-1 text-xs leading-5 text-gray-600 dark:text-gray-300">{step.body}</p>
        <div className="mt-2 flex items-center gap-1.5">
          <button data-testid="tour-skip" onClick={onClose} className="text-[11px] text-gray-400 hover:text-gray-700 dark:hover:text-gray-200">
            {i < STEPS.length - 1 ? 'Skip tour' : ''}
          </button>
          <div className="ml-auto flex gap-1.5">
            {i > 0 && (
              <button onClick={() => setI(i - 1)} className="rounded px-2.5 py-1 text-xs border border-gray-300 dark:border-gray-700 hover:bg-gray-100 dark:hover:bg-gray-800">
                Back
              </button>
            )}
            <button
              data-testid="tour-next"
              onClick={() => (i < STEPS.length - 1 ? setI(i + 1) : onClose())}
              className="rounded px-3 py-1 text-xs font-medium bg-sky-600 hover:bg-sky-500 text-white"
            >
              {i === 0 ? 'Show me' : i < STEPS.length - 1 ? 'Next' : 'Done'}
            </button>
          </div>
        </div>
        <div className="mt-2 flex gap-1 justify-center">
          {STEPS.map((_, k) => (
            <span key={k} className={`h-1 rounded-full transition-all ${k === i ? 'w-4 bg-sky-500' : 'w-1.5 bg-gray-300 dark:bg-gray-700'}`} />
          ))}
        </div>
      </div>
    </div>
  )
}
