import { chromium } from 'playwright-core'

const baseUrl = process.env.MAP_MANAGER_URL || 'http://127.0.0.1:8765'
const browser = await chromium.launch({
  executablePath: '/usr/bin/google-chrome',
  headless: true,
  args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-gl=swiftshader'],
})

const results = { consoleErrors: [], desktop: {}, rawCloud: {}, cloud: {}, mobile: {} }

try {
  const desktop = await browser.newPage({ viewport: { width: 1440, height: 900 }, deviceScaleFactor: 1 })
  desktop.on('console', (message) => {
    if (message.type() === 'error') results.consoleErrors.push(message.text())
  })
  desktop.on('pageerror', (error) => results.consoleErrors.push(error.message))
  await desktop.goto(baseUrl, { waitUntil: 'networkidle' })
  await desktop.locator('.app-shell').waitFor()
  await desktop.locator('.session-row').first().waitFor()
  await desktop.locator('.cloud-host[data-cloud-state="ready"]').waitFor({ timeout: 120000 })
  await desktop.waitForTimeout(700)
  await desktop.screenshot({ path: '/tmp/go2-map-workspace-raw-cloud.png', fullPage: true })
  results.rawCloud = await desktop.evaluate(() => ({
    activeSection: document.querySelector('.section-tabs button.active')?.textContent,
    selectedSession: Boolean(document.querySelector('.session-row.selected')),
    pointLabel: document.querySelector('.canvas-meta')?.textContent,
    heightLegend: Boolean(document.querySelector('.height-legend')),
    inspectorTitle: document.querySelector('.inspector-head span')?.textContent,
    algorithmInSession: document.querySelector('.session-row span')?.textContent,
  }))

  await desktop.locator('.section-tabs button').filter({ hasText: '生成地图' }).click()
  await desktop.locator('.version-row').first().waitFor()
  const candidate = desktop.locator('.version-row').filter({ hasText: '候选' }).first()
  if (await candidate.count()) await candidate.click()
  await desktop.locator('.map-stage img').waitFor()
  await desktop.screenshot({ path: '/tmp/go2-map-workspace-desktop.png', fullPage: true })
  results.desktop = await desktop.evaluate(() => {
    const activationButton = [...document.querySelectorAll('.inspector-actions button')]
      .find((button) => button.textContent.includes('设为定位 / 导航地图'))
    const activationRect = activationButton?.getBoundingClientRect()
    return {
      viewport: [innerWidth, innerHeight],
      scrollWidth: document.documentElement.scrollWidth,
      versions: document.querySelectorAll('.version-row').length,
      mapImage: Boolean(document.querySelector('.map-stage img')),
      inspectorVisible: getComputedStyle(document.querySelector('.inspector')).display !== 'none',
      bodyFontSize: getComputedStyle(document.body).fontSize,
      lifecycleCommands: document.querySelectorAll('.pipeline-stage button').length,
      primaryControlHeight: Math.round(document.querySelector('.pipeline-stage > button').getBoundingClientRect().height),
      mappingAlgorithm: document.querySelector('[aria-label="建图算法"]')?.value,
      mappingAlgorithms: [...document.querySelectorAll('[aria-label="建图算法"] option')].map((option) => option.value),
      explicitActivation: Boolean(activationButton),
      activationInViewport: Boolean(activationRect && activationRect.top >= 0 && activationRect.bottom <= innerHeight),
    }
  })

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
    const probe = document.createElement('canvas')
    probe.width = canvas.width
    probe.height = canvas.height
    const context = probe.getContext('2d', { willReadFrequently: true })
    context.drawImage(canvas, 0, 0, probe.width, probe.height)
    const pixels = context.getImageData(0, 0, probe.width, probe.height).data
    let renderedPixels = 0
    let coolPixels = 0
    let warmPixels = 0
    for (let index = 0; index < pixels.length; index += 4) {
      const red = pixels[index]
      const green = pixels[index + 1]
      const blue = pixels[index + 2]
      if (Math.abs(red - 13) + Math.abs(green - 20) + Math.abs(blue - 31) > 24) renderedPixels += 1
      if (blue > red + 24 && blue > 70) coolPixels += 1
      if (red > blue + 24 && red > 90) warmPixels += 1
    }
    return {
      canvasPixels: [canvas.width, canvas.height],
      cssSize: [Math.round(canvasRect.width), Math.round(canvasRect.height)],
      hostSize: [Math.round(hostRect.width), Math.round(hostRect.height)],
      pointLabel: document.querySelector('.canvas-meta')?.textContent,
      heightLegend: Boolean(document.querySelector('.height-legend')),
      heightLabels: [...document.querySelectorAll('.height-labels span')].map((node) => node.textContent),
      nonBlankSize: canvas.width > 100 && canvas.height > 100,
      renderedPixels,
      coolPixels,
      warmPixels,
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
    && results.desktop.mappingAlgorithm === 'faster_lio'
    && results.desktop.mappingAlgorithms.join(',') === 'faster_lio,fastlio2'
    && results.desktop.explicitActivation
    && results.desktop.activationInViewport
    && results.desktop.trashVisible
    && results.rawCloud.activeSection.includes('原始 PCD')
    && results.rawCloud.selectedSession
    && results.rawCloud.pointLabel
    && results.rawCloud.heightLegend
    && results.rawCloud.inspectorTitle === '原始 PCD'
    && results.rawCloud.algorithmInSession
    && results.cloud.nonBlankSize
    && results.cloud.heightLegend
    && results.cloud.heightLabels.length === 3
    && results.cloud.renderedPixels > 100
    && results.cloud.coolPixels > 5
    && results.cloud.warmPixels > 5
    && results.mobile.noHorizontalOverflow
    && results.mobile.sectionsDoNotOverlap
  console.log(JSON.stringify({ ok, ...results }, null, 2))
  if (!ok) process.exitCode = 1
} finally {
  await browser.close()
}
