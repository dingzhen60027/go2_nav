import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  Check,
  Eraser,
  FileWarning,
  LoaderCircle,
  Minus,
  Pencil,
  Plus,
  Redo2,
  RotateCcw,
  Save,
  Slash,
  Undo2,
  X,
} from 'lucide-react'

async function api(path, options) {
  const response = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  })
  const data = await response.json().catch(() => ({}))
  if (!response.ok) throw new Error(data.detail || `请求失败 (${response.status})`)
  return data
}

const COLORS = {
  occupied: '#000000',
  free: '#fefefe',
  unknown: '#cdcdcd',
}

const MODE_LABELS = {
  occupied: '画墙',
  free: '擦除杂点',
  unknown: '设为未知',
}

function clamp(value, low, high) {
  return Math.min(high, Math.max(low, value))
}

function drawOperation(context, operation) {
  if (!operation?.points?.length) return
  const points = operation.points
  context.save()
  context.strokeStyle = COLORS[operation.mode]
  context.fillStyle = COLORS[operation.mode]
  context.lineWidth = operation.size
  context.lineCap = 'round'
  context.lineJoin = 'round'
  if (points.length === 1) {
    context.beginPath()
    context.arc(points[0].x, points[0].y, Math.max(0.5, operation.size / 2), 0, Math.PI * 2)
    context.fill()
  } else {
    context.beginPath()
    context.moveTo(points[0].x, points[0].y)
    for (let index = 1; index < points.length; index += 1) {
      context.lineTo(points[index].x, points[index].y)
    }
    context.stroke()
  }
  context.restore()
}

export default function MapEditor({ version, onClose, onDone, notify }) {
  const canvasRef = useRef(null)
  const viewportRef = useRef(null)
  const sourceImageRef = useRef(null)
  const operationsRef = useRef([])
  const activeStrokeRef = useRef(null)
  const [ready, setReady] = useState(false)
  const [loadError, setLoadError] = useState('')
  const [operations, setOperations] = useState([])
  const [redoStack, setRedoStack] = useState([])
  const [mode, setMode] = useState('occupied')
  const [shape, setShape] = useState('brush')
  const [brushSize, setBrushSize] = useState(3)
  const [zoom, setZoom] = useState(1)
  const [imageSize, setImageSize] = useState({ width: 0, height: 0 })
  const [cursor, setCursor] = useState(null)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [name, setName] = useState(`${version.name}（2D修订）`)
  const [note, setNote] = useState('手工去除2D杂点并补齐墙体')
  const resolution = Number(version.parameters?.resolution) || 0.05

  const renderCanvas = useCallback((draft = null) => {
    const canvas = canvasRef.current
    const image = sourceImageRef.current
    if (!canvas || !image) return
    const context = canvas.getContext('2d', { alpha: false })
    context.imageSmoothingEnabled = false
    context.clearRect(0, 0, canvas.width, canvas.height)
    context.drawImage(image, 0, 0, canvas.width, canvas.height)
    operationsRef.current.forEach((operation) => drawOperation(context, operation))
    if (draft) drawOperation(context, draft)
  }, [])

  const fitToView = useCallback(() => {
    const viewport = viewportRef.current
    if (!viewport || !imageSize.width || !imageSize.height) return
    const horizontal = Math.max(120, viewport.clientWidth - 48) / imageSize.width
    const vertical = Math.max(120, viewport.clientHeight - 48) / imageSize.height
    setZoom(clamp(Math.min(horizontal, vertical), 0.1, 4))
  }, [imageSize])

  useEffect(() => {
    let cancelled = false
    setReady(false)
    setLoadError('')
    setOperations([])
    setRedoStack([])
    operationsRef.current = []
    const image = new Image()
    image.onload = () => {
      if (cancelled) return
      const canvas = canvasRef.current
      canvas.width = image.naturalWidth
      canvas.height = image.naturalHeight
      sourceImageRef.current = image
      setImageSize({ width: image.naturalWidth, height: image.naturalHeight })
      setReady(true)
      renderCanvas()
    }
    image.onerror = () => !cancelled && setLoadError('2D 地图加载失败，请确认 PGM 文件完整')
    image.src = `${version.map_preview_url}?editor=${Date.now()}`
    return () => { cancelled = true }
  }, [renderCanvas, version.id, version.map_preview_url])

  useEffect(() => {
    operationsRef.current = operations
    if (ready) renderCanvas()
  }, [operations, ready, renderCanvas])

  useEffect(() => {
    if (!ready) return undefined
    const frame = window.requestAnimationFrame(fitToView)
    return () => window.cancelAnimationFrame(frame)
  }, [fitToView, ready])

  const pointFromEvent = useCallback((event) => {
    const canvas = canvasRef.current
    const bounds = canvas.getBoundingClientRect()
    return {
      x: clamp((event.clientX - bounds.left) * canvas.width / bounds.width, 0, canvas.width - 1),
      y: clamp((event.clientY - bounds.top) * canvas.height / bounds.height, 0, canvas.height - 1),
    }
  }, [])

  const commitStroke = useCallback((operation) => {
    if (!operation?.points?.length) return
    setOperations((current) => [...current, operation])
    setRedoStack([])
  }, [])

  const handlePointerDown = (event) => {
    if (!ready || busy || event.button !== 0) return
    event.preventDefault()
    event.currentTarget.setPointerCapture(event.pointerId)
    const point = pointFromEvent(event)
    const operation = {
      mode,
      shape,
      size: brushSize,
      points: shape === 'line' ? [point, point] : [point],
    }
    activeStrokeRef.current = operation
    if (shape === 'line') renderCanvas(operation)
    else drawOperation(canvasRef.current.getContext('2d'), operation)
  }

  const handlePointerMove = (event) => {
    if (ready) setCursor(pointFromEvent(event))
    const operation = activeStrokeRef.current
    if (!operation) return
    event.preventDefault()
    const point = pointFromEvent(event)
    if (operation.shape === 'line') {
      operation.points[1] = point
      renderCanvas(operation)
      return
    }
    const previous = operation.points.at(-1)
    if (Math.hypot(point.x - previous.x, point.y - previous.y) < 0.5) return
    operation.points.push(point)
    drawOperation(canvasRef.current.getContext('2d'), {
      ...operation,
      points: [previous, point],
    })
  }

  const handlePointerUp = (event) => {
    const operation = activeStrokeRef.current
    if (!operation) return
    event.preventDefault()
    activeStrokeRef.current = null
    commitStroke({
      ...operation,
      points: operation.points.map((point) => ({ x: point.x, y: point.y })),
    })
  }

  const cancelActiveStroke = () => {
    if (!activeStrokeRef.current) return
    activeStrokeRef.current = null
    renderCanvas()
  }

  const undo = useCallback(() => {
    setOperations((current) => {
      if (!current.length) return current
      const removed = current.at(-1)
      setRedoStack((redo) => [...redo, removed])
      return current.slice(0, -1)
    })
  }, [])

  const redo = useCallback(() => {
    setRedoStack((current) => {
      if (!current.length) return current
      const restored = current.at(-1)
      setOperations((items) => [...items, restored])
      return current.slice(0, -1)
    })
  }, [])

  const reset = () => {
    setOperations([])
    setRedoStack([])
    activeStrokeRef.current = null
  }

  useEffect(() => {
    const handleKeyDown = (event) => {
      if (!(event.ctrlKey || event.metaKey)) return
      if (event.key.toLowerCase() === 'z') {
        event.preventDefault()
        if (event.shiftKey) redo()
        else undo()
      } else if (event.key.toLowerCase() === 'y') {
        event.preventDefault()
        redo()
      }
    }
    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [redo, undo])

  const closeEditor = () => {
    if (busy) return
    if (operations.length && !window.confirm('尚未保存的编辑会丢失，确定关闭吗？')) return
    onClose()
  }

  const save = async (event) => {
    event.preventDefault()
    if (!operations.length || !name.trim()) return
    setBusy(true)
    setError('')
    try {
      const result = await api(`/api/versions/${version.id}/edit-2d`, {
        method: 'POST',
        body: JSON.stringify({ name: name.trim(), note, operations }),
      })
      notify(`2D 修订版已保存：修改 ${result.edit_summary.changed_pixels} 个像素`)
      await onDone(result.version_id)
    } catch (requestError) {
      setError(requestError.message)
      notify(requestError.message, 'error')
    } finally {
      setBusy(false)
    }
  }

  const counts = useMemo(() => operations.reduce((summary, operation) => ({
    ...summary,
    [operation.mode]: summary[operation.mode] + 1,
  }), { occupied: 0, free: 0, unknown: 0 }), [operations])

  return (
    <div className="map-editor-backdrop">
      <section className="map-editor" role="dialog" aria-modal="true" aria-label="修整2D地图">
        <header className="map-editor-head">
          <div><span>非破坏式编辑</span><h2>修整 2D 导航地图</h2><p>{version.name} · 保存后生成新候选版本，ICP PCD 保持不变</p></div>
          <button className="icon-button" title="关闭编辑器" onClick={closeEditor} disabled={busy}><X size={19} /></button>
        </header>

        <div className="map-editor-body">
          <aside className="map-editor-tools">
            <section>
              <strong>修改内容</strong>
              <div className="map-editor-tool-grid">
                <button className={mode === 'occupied' ? 'active wall' : ''} onClick={() => setMode('occupied')}><Pencil size={17} /><span>画墙</span><small>标记为占据</small></button>
                <button className={mode === 'free' ? 'active free' : ''} onClick={() => setMode('free')}><Eraser size={17} /><span>擦除杂点</span><small>标记为可通行</small></button>
                <button className={mode === 'unknown' ? 'active unknown' : ''} onClick={() => setMode('unknown')}><FileWarning size={17} /><span>未知区域</span><small>不确定区域</small></button>
              </div>
            </section>

            <section>
              <strong>落笔方式</strong>
              <div className="map-editor-shapes">
                <button className={shape === 'brush' ? 'active' : ''} onClick={() => setShape('brush')}><Pencil size={15} />自由笔刷</button>
                <button className={shape === 'line' ? 'active' : ''} onClick={() => setShape('line')}><Slash size={15} />直线补墙</button>
              </div>
            </section>

            <section className="map-editor-brush">
              <div><strong>笔刷宽度</strong><b>{brushSize} px · {(brushSize * resolution).toFixed(2)} m</b></div>
              <input type="range" min="1" max="40" step="1" value={brushSize} onChange={(event) => setBrushSize(Number(event.target.value))} />
              <small>墙体建议 2–4 像素；大面积去噪可增大橡皮擦。</small>
            </section>

            <section className="map-editor-history">
              <strong>编辑历史</strong>
              <div><button onClick={undo} disabled={!operations.length || busy}><Undo2 size={15} />撤销</button><button onClick={redo} disabled={!redoStack.length || busy}><Redo2 size={15} />重做</button></div>
              <button className="reset" onClick={reset} disabled={!operations.length || busy}><RotateCcw size={15} />恢复到原始2D地图</button>
              <small>快捷键：Ctrl+Z 撤销，Ctrl+Shift+Z / Ctrl+Y 重做。</small>
            </section>

            <section className="map-editor-legend">
              <strong>地图颜色</strong>
              <span><i className="occupied" />黑色：墙体 / 障碍</span>
              <span><i className="free" />白色：自由空间</span>
              <span><i className="unknown" />灰色：未知空间</span>
            </section>
          </aside>

          <main className="map-editor-stage">
            <div className="map-editor-stage-bar">
              <div><b>{MODE_LABELS[mode]}</b><span>{shape === 'line' ? '拖动起点到终点' : '按住鼠标拖动'}</span></div>
              <div className="map-editor-zoom"><button onClick={() => setZoom((value) => clamp(value / 1.25, 0.1, 6))} title="缩小"><Minus size={15} /></button><b>{Math.round(zoom * 100)}%</b><button onClick={() => setZoom((value) => clamp(value * 1.25, 0.1, 6))} title="放大"><Plus size={15} /></button><button onClick={fitToView}>适应窗口</button></div>
            </div>
            <div className="map-editor-viewport" ref={viewportRef}>
              {!ready && !loadError && <div className="map-editor-loading"><LoaderCircle className="spin" size={21} />正在加载原始 PGM</div>}
              {loadError && <div className="map-editor-loading error"><FileWarning size={21} />{loadError}</div>}
              <canvas
                ref={canvasRef}
                className={ready ? 'ready' : ''}
                style={{ width: imageSize.width * zoom, height: imageSize.height * zoom }}
                onPointerDown={handlePointerDown}
                onPointerMove={handlePointerMove}
                onPointerUp={handlePointerUp}
                onPointerCancel={cancelActiveStroke}
                onPointerLeave={() => setCursor(null)}
              />
              {ready && <div className="map-editor-canvas-info"><span>{imageSize.width} × {imageSize.height} px</span><span>{resolution.toFixed(3)} m/格</span>{cursor && <span>x {Math.round(cursor.x)} · y {Math.round(cursor.y)}</span>}</div>}
            </div>
          </main>

          <form className="map-editor-save" onSubmit={save}>
            <div className="map-editor-safety"><Check size={17} /><span><strong>源版本保持不变</strong><small>新版本继续绑定同一套 ICP 地图</small></span></div>
            <label><span>新版本名称</span><input maxLength="80" value={name} onChange={(event) => setName(event.target.value)} /></label>
            <label><span>修订说明</span><textarea rows="4" maxLength="500" value={note} onChange={(event) => setNote(event.target.value)} /></label>
            <section className="map-editor-summary">
              <div><span>操作总数</span><strong>{operations.length}</strong></div>
              <p><span><i className="occupied" />补墙 {counts.occupied}</span><span><i className="free" />擦除 {counts.free}</span><span><i className="unknown" />未知 {counts.unknown}</span></p>
            </section>
            {error && <div className="map-editor-error"><FileWarning size={16} />{error}</div>}
            <div className="map-editor-save-actions"><button type="button" className="button secondary" onClick={closeEditor} disabled={busy}>取消</button><button className="button primary" disabled={busy || !ready || !operations.length || !name.trim()}>{busy ? <LoaderCircle className="spin" size={16} /> : <Save size={16} />}{busy ? '正在创建版本…' : '保存为新地图版本'}</button></div>
          </form>
        </div>
      </section>
    </div>
  )
}
