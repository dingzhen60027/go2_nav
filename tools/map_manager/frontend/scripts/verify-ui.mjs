import { chromium } from 'playwright-core'

const baseUrl = process.env.MAP_MANAGER_URL || 'http://127.0.0.1:8765'
const browser = await chromium.launch({
  executablePath: '/usr/bin/google-chrome',
  headless: true,
  args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-gl=swiftshader'],
})

const results = { consoleErrors: [], desktop: {}, mapEditor: {}, rawCloud: {}, cloud: {}, waypoint: {}, mobile: {} }

try {
  const desktop = await browser.newPage({ viewport: { width: 1440, height: 900 }, deviceScaleFactor: 1 })
  desktop.on('console', (message) => {
    if (message.type() === 'error') results.consoleErrors.push(message.text())
  })
  desktop.on('pageerror', (error) => results.consoleErrors.push(error.message))
  await desktop.goto(baseUrl, { waitUntil: 'networkidle' })
  await desktop.locator('.app-shell').waitFor()
  await desktop.locator('.section-tabs button').filter({ hasText: '原始 PCD' }).click()
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
      localizationModule: document.querySelector('[aria-label="定位模块"]')?.value,
      localizationModules: [...document.querySelectorAll('[aria-label="定位模块"] option')].map((option) => option.value),
      processCleanupButton: Boolean(document.querySelector('[aria-label="清理所有项目进程"]')),
      explicitActivation: Boolean(activationButton),
      activationInViewport: Boolean(activationRect && activationRect.top >= 0 && activationRect.bottom <= innerHeight),
    }
  })

  await desktop.locator('[aria-label="清理所有项目进程"]').click()
  results.desktop.processCleanupConfirmation = await desktop.getByRole('heading', { name: '强制结束 ROS 运行栈' }).isVisible()
  await desktop.getByRole('button', { name: '取消', exact: true }).click()

  await desktop.getByRole('button', { name: '修整 2D 地图', exact: true }).click()
  await desktop.locator('.map-editor canvas.ready').waitFor()
  await desktop.getByRole('button', { name: /直线补墙/ }).click()
  const editorCanvas = desktop.locator('.map-editor canvas.ready')
  const editorBounds = await editorCanvas.boundingBox()
  await desktop.mouse.move(editorBounds.x + editorBounds.width * 0.35, editorBounds.y + editorBounds.height * 0.45)
  await desktop.mouse.down()
  await desktop.mouse.move(editorBounds.x + editorBounds.width * 0.62, editorBounds.y + editorBounds.height * 0.45, { steps: 8 })
  await desktop.mouse.up()
  await desktop.screenshot({ path: '/tmp/go2-map-workspace-map-editor.png', fullPage: true })
  results.mapEditor = await desktop.evaluate(() => ({
    canvasReady: Boolean(document.querySelector('.map-editor canvas.ready')),
    sourcePreserved: document.querySelector('.map-editor-safety')?.textContent.includes('源版本保持不变'),
    operationCount: Number(document.querySelector('.map-editor-summary > div strong')?.textContent),
    saveEnabled: ![...document.querySelectorAll('.map-editor-save-actions button')]
      .find((button) => button.textContent.includes('保存为新地图版本'))?.disabled,
    noHorizontalOverflow: document.querySelector('.map-editor').scrollWidth <= document.querySelector('.map-editor').clientWidth,
  }))
  await desktop.getByRole('button', { name: '撤销', exact: true }).click()
  results.mapEditor.afterUndo = Number(await desktop.locator('.map-editor-summary > div strong').textContent())
  await desktop.getByRole('button', { name: '重做', exact: true }).click()
  results.mapEditor.afterRedo = Number(await desktop.locator('.map-editor-summary > div strong').textContent())
  await desktop.getByRole('button', { name: '恢复到原始2D地图', exact: true }).click()
  await desktop.getByTitle('关闭编辑器').click()

  let localizationStartPayload = null
  await desktop.route('**/api/runtime/start', async (route) => {
    localizationStartPayload = route.request().postDataJSON()
    await route.fulfill({
      status: 202,
      contentType: 'application/json',
      body: JSON.stringify({ status: 'running', mode: 'localization', algorithm: localizationStartPayload.algorithm }),
    })
  })
  await desktop.locator('[aria-label="定位模块"]').selectOption('pure_icp')
  await desktop.getByRole('button', { name: '纯 ICP', exact: true }).click()
  await desktop.waitForTimeout(200)
  results.desktop.localizationStartPayload = localizationStartPayload
  await desktop.unroute('**/api/runtime/start')

  let navigationStartPayload = null
  await desktop.route('**/api/runtime/start', async (route) => {
    navigationStartPayload = route.request().postDataJSON()
    await route.fulfill({
      status: 202,
      contentType: 'application/json',
      body: JSON.stringify({ status: 'running', mode: 'navigation', algorithm: navigationStartPayload.algorithm }),
    })
  })
  await desktop.locator('[aria-label="定位模块"]').selectOption('fused_ekf')
  await desktop.getByRole('button', { name: '导航', exact: true }).click()
  await desktop.waitForTimeout(200)
  results.desktop.navigationStartPayload = navigationStartPayload
  await desktop.unroute('**/api/runtime/start')

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

  let cancelRequested = false
  await desktop.route('**/api/overview', async (route) => {
    const response = await route.fetch()
    const body = await response.json()
    body.runtime = { ...body.runtime, status: 'running', mode: 'navigation', logs: ['Nav2 ready'] }
    body.waypoints = {
      map_id: body.active?.id,
      count: 3,
      items: [
        { id: 'wp-20260805-a', name: '装卸区', position: { x: 1.2, y: -0.5, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 }, yaw: 0 },
        { id: 'wp-20260805-b', name: '巡检通道', position: { x: 4.8, y: 2.1, z: 0 }, orientation: { x: 0, y: 0, z: 0.707, w: 0.707 }, yaw: 1.5708 },
        { id: 'wp-20260805-c', name: '充电区', position: { x: -1.3, y: 3.5, z: 0 }, orientation: { x: 0, y: 0, z: 1, w: 0 }, yaw: 3.1416 },
      ],
    }
    body.waypoint_mission = {
      status: 'running', active: true, can_cancel: true, mission_id: 'mission-ui-test',
      map_id: body.active?.id, total: 3, processed: 1, completed: 1, failed_count: 0,
      current_index: 1, current_waypoint_id: 'wp-20260805-b', current_waypoint_name: '巡检通道',
      progress_percent: 54.2, leg_progress_percent: 62.5, distance_remaining: 1.84,
      elapsed_sec: 73, message: '正在前往 巡检通道', results: [{ waypoint_id: 'wp-20260805-a', succeeded: true }],
    }
    await route.fulfill({ response, json: body })
  })
  await desktop.route('**/api/waypoint-mission/cancel', async (route) => {
    cancelRequested = true
    await route.fulfill({ status: 202, contentType: 'application/json', body: JSON.stringify({ status: 'cancelling' }) })
  })
  await desktop.reload({ waitUntil: 'networkidle' })
  await desktop.locator('.section-tabs button').filter({ hasText: '多点导航' }).click()
  await desktop.locator('.mission-workspace').waitFor()
  await desktop.screenshot({ path: '/tmp/go2-map-workspace-waypoints.png', fullPage: true })
  results.waypoint = await desktop.evaluate(() => ({
    waypointRows: document.querySelectorAll('.waypoint-row').length,
    routeStops: document.querySelectorAll('.route-timeline > button').length,
    progressValue: document.querySelector('.mission-workspace [role="progressbar"]')?.getAttribute('aria-valuenow'),
    currentGoal: document.querySelector('.route-timeline > button.current strong')?.textContent,
    persistentDrawer: Boolean(document.querySelector('.mission-drawer')),
    cancelButton: Boolean([...document.querySelectorAll('.mission-drawer button')].find((button) => button.textContent.includes('取消任务'))),
    noHorizontalOverflow: document.documentElement.scrollWidth <= innerWidth,
  }))
  await desktop.locator('.mission-drawer button').filter({ hasText: '取消任务' }).click()
  await desktop.waitForTimeout(100)
  results.waypoint.cancelRequested = cancelRequested

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
    && results.desktop.localizationModule === 'fused_ekf'
    && results.desktop.localizationModules.join(',') === 'fused_ekf,pure_icp'
    && results.desktop.processCleanupButton
    && results.desktop.processCleanupConfirmation
    && results.desktop.localizationStartPayload?.mode === 'localization'
    && results.desktop.localizationStartPayload?.algorithm === 'pure_icp'
    && results.desktop.navigationStartPayload?.mode === 'navigation'
    && results.desktop.navigationStartPayload?.algorithm === 'fused_ekf'
    && results.desktop.explicitActivation
    && results.desktop.activationInViewport
    && results.desktop.trashVisible
    && results.mapEditor.canvasReady
    && results.mapEditor.sourcePreserved
    && results.mapEditor.operationCount === 1
    && results.mapEditor.saveEnabled
    && results.mapEditor.afterUndo === 0
    && results.mapEditor.afterRedo === 1
    && results.mapEditor.noHorizontalOverflow
    && results.rawCloud.activeSection.includes('原始 PCD')
    && results.rawCloud.selectedSession
    && results.rawCloud.pointLabel
    && results.rawCloud.heightLegend
    && ['原始 PCD', '聚类清理副本'].includes(results.rawCloud.inspectorTitle)
    && results.rawCloud.algorithmInSession
    && results.cloud.nonBlankSize
    && results.cloud.heightLegend
    && results.cloud.heightLabels.length === 3
    && results.cloud.renderedPixels > 100
    && results.cloud.coolPixels > 5
    && results.cloud.warmPixels > 5
    && results.waypoint.waypointRows === 3
    && results.waypoint.routeStops === 3
    && results.waypoint.progressValue === '54.2'
    && results.waypoint.currentGoal === '巡检通道'
    && results.waypoint.persistentDrawer
    && results.waypoint.cancelButton
    && results.waypoint.cancelRequested
    && results.waypoint.noHorizontalOverflow
    && results.mobile.noHorizontalOverflow
    && results.mobile.sectionsDoNotOverlap
  console.log(JSON.stringify({ ok, ...results }, null, 2))
  if (!ok) process.exitCode = 1
} finally {
  await browser.close()
}
