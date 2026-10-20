/*
 * File: main.tsx
 *
 * Description:
 *   Entry point of the web playground page.
 */

import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App'
import './index.css'
import { SimClient, simKindFromUrl } from './sim/SimClient'
import { mapUrl } from './map/loadMap'

// One sim worker per page, created outside React so StrictMode's double mount does not start two.
// The real stack (Pyodide) is the default; ?sim=mock is for work on the page.
const kind = simKindFromUrl(window.location.href)
const client = new SimClient(kind, mapUrl(), import.meta.env.BASE_URL)

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App client={client} />
  </StrictMode>,
)
