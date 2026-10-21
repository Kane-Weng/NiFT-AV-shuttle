/*
 * File: settings.mjs
 *
 * Description:
 *   Puppeteer helpers for the settings and stack windows in the header, shared by smoke.mjs and
 *   e2e_pyodide.mjs. The theme, the view options, the test routes and the launch, parameter and share panes live there.
 */

/** The settings tab that holds each pane, by the pane's testid. */
const TAB_OF = { 'view-section': 'view', 'scenarios-section': 'scenarios', 'launch-section': 'launch', 'params-section': 'params', 'scenario-section': 'share' }

const isOpen = (page) => page.$eval('[data-testid="settings-open"]', (b) => b.getAttribute('aria-expanded') === 'true')

export async function openSettings(page, tab) {
  if (!(await isOpen(page))) await page.click('[data-testid="settings-open"]')
  await page.click(`[data-testid="settings-tab-${tab}"]`)
}

export async function closeSettings(page) {
  if (await isOpen(page)) await page.click('[data-testid="settings-close"]')
}

/** Switches dark and light through the View tab, then closes the window so screenshots show the map. */
export async function toggleTheme(page) {
  await openSettings(page, 'view')
  const dark = await page.evaluate(() => document.documentElement.classList.contains('dark'))
  await page.click(`[data-testid="theme-${dark ? 'light' : 'dark'}"]`)
  await closeSettings(page)
}

/** Opens the stack window (nodes and diagnostics). */
export async function openStack(page) {
  const open = await page.$eval('[data-testid="stack-open"]', (b) => b.getAttribute('aria-expanded') === 'true')
  if (!open) await page.click('[data-testid="stack-open"]')
  await page.waitForSelector('[data-testid="stack-map"]', { visible: true, timeout: 5000 })
}

/** Shows the probe inside a pane: opens its settings tab or the stack window, or unfolds a sidebar section. */
export async function openSection(page, section, probe) {
  const tab = TAB_OF[section]
  if (tab) await openSettings(page, tab)
  else if (section === 'diag-section') await openStack(page)
  else if (!(await page.$(`[data-testid="${section}"] [data-testid="${probe}"]`))) await page.click(`[data-testid="${section}"] button`)
  await page.waitForSelector(`[data-testid="${section}"] [data-testid="${probe}"]`, { visible: true, timeout: 3000 })
}
