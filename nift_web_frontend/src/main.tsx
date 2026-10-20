/*
 * File: main.tsx
 * Author: Kane Weng
 * Date: April 21, 2026
 *
 * Description:
 *   The entry point of the React application. Its only job is to find the
 *   <div id="root"> element in index.html and mount the entire app inside it.
 *
 *   StrictMode is a React development helper that intentionally runs certain
 *   lifecycle functions twice to help catch accidental side effects early.
 *   It has no effect in the production build.
 *
 *   Also imports global CSS and the Leaflet map stylesheet here so they are
 *   loaded exactly once for the whole app.
 *
 * Usage:
 *   Not called directly — Vite uses this file as the JavaScript bundle
 *   entry point (configured in index.html).
 */

import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import 'leaflet/dist/leaflet.css'
import './index.css'
import App from './App'

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
)
