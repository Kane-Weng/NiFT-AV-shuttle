/*
 * File: theme.ts
 *
 * Description:
 *   Scene colours. Dark is the RViz look (its default 48/48/48 background, grey grid, green path).
 *   Light keeps the same meaning of each colour on a pale background.
 */

export type ThemeName = 'dark' | 'light'

export interface SceneTheme {
  background: number
  gridMinor: number
  gridMajor: number
  edge: number
  divider: number
  center: number
  virtual: number
  centerline: number
  path: number
  goal: number
  obstacle: number
  body: number
  bodyEdge: number
  front: number
  trail: number
  pose: number // 2D Pose Estimate arrow
  perceived: number // what /obstacles held
  selected: number // the selected obstacle's outline
  poseEst: number // the stack's pose (TF map -> base_link)
}

export const THEMES: Record<ThemeName, SceneTheme> = {
  dark: {
    background: 0x303030,
    gridMinor: 0x3b3b3b,
    gridMajor: 0x555555,
    edge: 0xe6e6e6,
    divider: 0xb5b5b5,
    center: 0xf2c230,
    virtual: 0x7a7a7a,
    centerline: 0x3fa7e0,
    path: 0x19ff00,
    goal: 0xe040fb,
    obstacle: 0xff4d4f,
    body: 0x8c9bb0,
    bodyEdge: 0x1f2937,
    front: 0xff8c1a,
    trail: 0x7fd6ff,
    pose: 0x19ff00,
    perceived: 0xffd60a,
    selected: 0xffffff,
    poseEst: 0x00e5ff,
  },
  light: {
    background: 0xf3f4f6,
    gridMinor: 0xe2e4e8,
    gridMajor: 0xc4c8ce,
    edge: 0x2f3742,
    divider: 0x6b7280,
    center: 0xc98a00,
    virtual: 0xa3a8b0,
    centerline: 0x2563eb,
    path: 0x16a34a,
    goal: 0xc026d3,
    obstacle: 0xdc2626,
    body: 0x64748b,
    bodyEdge: 0x111827,
    front: 0xf97316,
    trail: 0x0284c7,
    pose: 0x16a34a,
    perceived: 0xb45309,
    selected: 0x111827,
    poseEst: 0x0891b2,
  },
}
