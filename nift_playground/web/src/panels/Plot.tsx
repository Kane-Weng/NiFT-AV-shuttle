/*
 * File: Plot.tsx
 *
 * Description:
 *   A small time plot in SVG for the telemetry panel: the last minute of one measure, with an
 *   optional dashed reference series (the target speed), a zero line, a recessive grid and a
 *   hover readout. Two measures with different units go in two plots, never on two axes.
 */

import { useRef, useState } from 'react'
import type { Sample } from './history'
import { PLOT_WINDOW_S } from './history'

export interface PlotSeries {
  label: string
  get: (s: Sample) => number | null
  /** Tailwind stroke classes for light and dark */
  stroke: string
  dashed?: boolean
}

interface Props {
  title: string
  unit: string
  samples: Sample[]
  series: PlotSeries[]
  /** The y range always covers at least this span around the data */
  minSpan: number
  symmetric?: boolean
  testid?: string
}

const W = 300
const H = 64
const PAD_L = 0
const PAD_R = 0

function pathOf(samples: Sample[], get: (s: Sample) => number | null, x: (t: number) => number, y: (v: number) => number): string {
  let d = ''
  let pen = false
  for (const s of samples) {
    const v = get(s)
    if (v === null || !Number.isFinite(v)) {
      pen = false
      continue
    }
    d += `${pen ? 'L' : 'M'}${x(s.t).toFixed(1)},${y(v).toFixed(1)}`
    pen = true
  }
  return d
}

export default function Plot({ title, unit, samples, series, minSpan, symmetric, testid }: Props) {
  const ref = useRef<SVGSVGElement>(null)
  const [hover, setHover] = useState<number | null>(null)
  const tEnd = samples.length ? samples[samples.length - 1].t : 0
  const t0 = Math.max(0, tEnd - PLOT_WINDOW_S)
  const t1 = Math.max(t0 + 10, tEnd)

  let lo = Infinity
  let hi = -Infinity
  for (const s of samples)
    for (const ser of series) {
      const v = ser.get(s)
      if (v !== null && Number.isFinite(v)) {
        lo = Math.min(lo, v)
        hi = Math.max(hi, v)
      }
    }
  if (!Number.isFinite(lo)) {
    lo = 0
    hi = minSpan
  }
  if (symmetric) {
    const m = Math.max(Math.abs(lo), Math.abs(hi), minSpan / 2)
    lo = -m
    hi = m
  } else {
    lo = Math.min(0, lo)
    hi = Math.max(hi, lo + minSpan)
  }
  const x = (t: number) => PAD_L + ((t - t0) / (t1 - t0)) * (W - PAD_L - PAD_R)
  const y = (v: number) => H - 2 - ((v - lo) / (hi - lo)) * (H - 4)

  const hovered = hover !== null && samples.length ? samples.reduce((a, b) => (Math.abs(b.t - hover) < Math.abs(a.t - hover) ? b : a)) : null
  const last = samples.length ? samples[samples.length - 1] : null
  const shown = hovered ?? last

  const onMove = (e: React.PointerEvent) => {
    const r = ref.current!.getBoundingClientRect()
    setHover(t0 + ((e.clientX - r.left) / r.width) * (t1 - t0))
  }

  return (
    <div data-testid={testid} className="mt-2">
      <div className="flex items-baseline justify-between text-[11px]">
        <span className="text-gray-500">
          {title} <span className="text-gray-400 dark:text-gray-500">({unit}, last {PLOT_WINDOW_S} s)</span>
        </span>
        <span className="font-mono tabular-nums text-gray-700 dark:text-gray-200">
          {shown
            ? series
                .map((s) => {
                  const v = s.get(shown)
                  return `${series.length > 1 ? s.label + ' ' : ''}${v === null ? '-' : v.toFixed(2)}`
                })
                .join('  ')
            : '-'}
          {hovered ? <span className="text-gray-400"> @ {hovered.t.toFixed(1)} s</span> : null}
        </span>
      </div>
      <svg
        ref={ref}
        viewBox={`0 0 ${W} ${H}`}
        preserveAspectRatio="none"
        className="w-full h-16 rounded bg-gray-50 dark:bg-gray-950 border border-gray-200 dark:border-gray-800 cursor-crosshair"
        onPointerMove={onMove}
        onPointerLeave={() => setHover(null)}
      >
        {/* recessive grid: quarter lines and the zero line */}
        {[0.25, 0.5, 0.75].map((f) => (
          <line key={f} x1={0} x2={W} y1={H * f} y2={H * f} className="stroke-gray-200 dark:stroke-gray-800" strokeWidth={1} vectorEffect="non-scaling-stroke" />
        ))}
        {lo < 0 && hi > 0 && <line x1={0} x2={W} y1={y(0)} y2={y(0)} className="stroke-gray-400 dark:stroke-gray-600" strokeWidth={1} vectorEffect="non-scaling-stroke" />}
        {series.map((s) => (
          <path
            key={s.label}
            d={pathOf(samples, s.get, x, y)}
            fill="none"
            className={s.stroke}
            strokeWidth={s.dashed ? 1.25 : 2}
            strokeDasharray={s.dashed ? '4 3' : undefined}
            strokeLinejoin="round"
            vectorEffect="non-scaling-stroke"
          />
        ))}
        {hovered && <line x1={x(hovered.t)} x2={x(hovered.t)} y1={0} y2={H} className="stroke-gray-500" strokeWidth={1} vectorEffect="non-scaling-stroke" />}
      </svg>
      <div className="flex justify-between text-[10px] font-mono text-gray-400 dark:text-gray-500">
        <span>
          {hi.toFixed(1)} / {lo.toFixed(1)}
        </span>
        {series.length > 1 && (
          <span className="flex gap-3">
            {series.map((s) => (
              <span key={s.label} className="flex items-center gap-1">
                <svg width="14" height="4">
                  <line x1="0" x2="14" y1="2" y2="2" className={s.stroke} strokeWidth={2} strokeDasharray={s.dashed ? '3 2' : undefined} />
                </svg>
                {s.label}
              </span>
            ))}
          </span>
        )}
      </div>
    </div>
  )
}
