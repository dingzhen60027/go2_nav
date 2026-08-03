import { chromium } from 'playwright-core'

const baseUrl = process.env.MAP_MANAGER_URL || 'http://127.0.0.1:8765'
const browser = await chromium.launch({
  executablePath: '/usr/bin/google-chrome',
  headless: true,
  args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-gl=swiftshader'],
})

const results = { consoleErrors: [], desktop: {}, cloud: {}, mobile: {} }

try {
  const desktop = await browser.newPage({ viewport: { width: 1440, height: 900 }, deviceScaleFactor: 1 })
  desktop.on('console', (message) => {
    if (message.type() === 'error') results.consoleErrors.push(message.text())
  })
  desktop.on('pageerror', (error) => results.consoleErrors.push(error.message))
  await desktop.goto(baseUrl, { waitUntil: 'networkidle' })
  await desktop.locator('.app-shell').waitFor()
  await desktop.locator('.version-row').first().waitFor()
  await desktop.screenshot({ path: '/tmp/go2-map-workspace-desktop.png', fullPage: true })
  results.desktop = await desktop.evaluate(() => ({
    viewport: [innerWidth, innerHeight],
    scrollWidth: document.documentElement.scrollWidth,
    versions: document.querySelectorAll('.version-row').length,
    mapImage: Boolean(document.querySelector('.map-stage img')),
    inspectorVisible: getComputedStyle(document.querySelector('.inspector')).display !== 'none',
    bodyFontSize: getComputedStyle(document.body).fontSize,
    lifecycleCommands: document.querySelectorAll('.pipeline-stage button').length,
    primaryControlHeight: Math.round(document.querySelector('.pipeline-stage > button').getBoundingClientRect().height),
  }))

  await desktop.locator('.section-tabs button').filter({ hasText: '回收站' }).click()
  await desktop.locator('.trash-list').waitFor()
  results.desktop.trashVisible = true
  await desktop.locator('.section-tabs button').filter({ hasText: '生成地图' }).click()

  await desktop.getByRole('button', { name: '3D 点云' }).click()
  await desktop.locator('.cloud-host[data-cloud-state="ready"]').waitFor({ timeout: 15000 })
  await desktop.waitForTimeout(700)
  await desktop.screenshot({ path: '/tmp/go2-map-workspace-cloud.png', fullPage: true })
  results.cloud = await desktop.evaluate(() => {
    const canvas = document.querySelector('.cloud-host canvas')
    const host = document.querySelector('.cloud-host')
    const canvasRect = canvas.getBoundingClientRect()
    const hostRect = host.getBoundingClientRect()
    return {
      canvasPixels: [canvas.width, canvas.height],
      cssSize: [Math.round(canvasRect.width), Math.round(canvasRect.height)],
      hostSize: [Math.round(hostRect.width), Math.round(hostRect.height)],
      pointLabel: document.querySelector('.canvas-meta')?.textContent,
      nonBlankSize: canvas.width > 100 && canvas.height > 100,
    }
  })

  const mobile = await browser.newPage({ viewport: { width: 390, height: 844 }, deviceScaleFactor: 1 })
  mobile.on('pageerror', (error) => results.consoleErrors.push(error.message))
  await mobile.goto(baseUrl, { waitUntil: 'networkidle' })
  await mobile.locator('.app-shell').waitFor()
  await mobile.screenshot({ path: '/tmp/go2-map-workspace-mobile.png', fullPage: true })
  results.mobile = await mobile.evaluate(() => {
    const sidebar = document.querySelector('.sidebar').getBoundingClientRect()
    const preview = document.querySelector('.preview-area').getBoundingClientRect()
    return {
      viewport: [innerWidth, innerHeight],
      scrollWidth: document.documentElement.scrollWidth,
      noHorizontalOverflow: document.documentElement.scrollWidth <= innerWidth,
      sidebarBottom: Math.round(sidebar.bottom),
      previewTop: Math.round(preview.top),
      sectionsDoNotOverlap: preview.top >= sidebar.bottom - 1,
    }
  })

  const ok = results.consoleErrors.length === 0
    && results.desktop.scrollWidth === results.desktop.viewport[0]
    && results.desktop.mapImage
    && results.desktop.lifecycleCommands === 4
    && results.desktop.primaryControlHeight >= 40
    && results.desktop.trashVisible
    && results.cloud.nonBlankSize
    && results.mobile.noHorizontalOverflow
    && results.mobile.sectionsDoNotOverlap
  console.log(JSON.stringify({ ok, ...results }, null, 2))
  if (!ok) process.exitCode = 1
} finally {
  await browser.close()
}
