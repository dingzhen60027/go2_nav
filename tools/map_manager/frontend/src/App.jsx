import { lazy, Suspense, useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  Activity,
  Archive,
  Box,
  Check,
  ChevronRight,
  Clock3,
  Copy,
  Database,
  FileWarning,
  FolderInput,
  HardDrive,
  History,
  Image as ImageIcon,
  Layers3,
  LoaderCircle,
  Map,
  MapPin,
  Navigation,
  RefreshCw,
  RotateCcw,
  Search,
  SlidersHorizontal,
  Square,
  Trash2,
  Undo2,
  X,
} from 'lucide-react'

const PointCloudView = lazy(() => import('./PointCloudView.jsx'))

const MAPPING_ALGORITHMS = {
  faster_lio: 'FASTer-LIO',
  fastlio2: 'FAST-LIO2',
  unknown: '算法未记录',
}

function algorithmLabel(value) {
  return MAPPING_ALGORITHMS[value] || MAPPING_ALGORITHMS.unknown
}


async function api(path, options) {
  const response = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  })
  const data = await response.json().catch(() => ({}))
  if (!response.ok) throw new Error(data.detail || `请求失败 (${response.status})`)
  return data
}

function formatTime(value) {
  if (!value) return '时间未知'
  return new Intl.DateTimeFormat('zh-CN', {
    month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit',
  }).format(new Date(value))
}

function StatusBadge({ status, complete = true }) {
  if (!complete) return <span className="badge danger"><FileWarning size={13} />不完整</span>
  const map = {
    active: ['active', '使用中'],
    candidate: ['candidate', '候选'],
    archived: ['muted', '已归档'],
  }
  const [className, label] = map[status] || ['muted', status]
  return <span className={`badge ${className}`}>{status === 'active' && <Check size={13} />}{label}</span>
}

function IconButton({ title, children, onClick, disabled = false }) {
  return (
    <button className="icon-button" onClick={onClick} disabled={disabled} title={title} aria-label={title}>
      {children}
    </button>
  )
}

function Modal({ title, subtitle, onClose, children }) {
  return (
    <div className="modal-backdrop" onMouseDown={(event) => event.target === event.currentTarget && onClose()}>
      <div className="modal" role="dialog" aria-modal="true" aria-label={title}>
        <div className="modal-head">
          <div><h2>{title}</h2><p>{subtitle}</p></div>
          <IconButton title="关闭" onClick={onClose}><X size={18} /></IconButton>
        </div>
        {children}
      </div>
    </div>
  )
}

function Field({ label, children, hint }) {
  return (
    <label className="field">
      <span>{label}</span>
      {children}
      {hint && <small>{hint}</small>}
    </label>
  )
}

function PathRow({ label, value, notify }) {
  const copy = async () => {
    await navigator.clipboard.writeText(value)
    notify('路径已复制')
  }
  return (
    <div className="path-row">
      <span>{label}</span>
      <code title={value}>{value}</code>
      <IconButton title={`复制${label}路径`} onClick={copy}><Copy size={15} /></IconButton>
    </div>
  )
}

function ArchiveModal({ capture, stopFirst = false, onClose, onDone, onDiscard, notify }) {
  const [form, setForm] = useState({ name: `建图 ${new Date().toLocaleString('zh-CN')}`, site: 'default', note: '' })
  const [busy, setBusy] = useState(false)
  const submit = async (event) => {
    event.preventDefault()
    setBusy(true)
    try {
      if (stopFirst) {
        const stopped = await api('/api/runtime/stop', { method: 'POST' })
        if (!stopped.current_capture?.available || stopped.current_capture.archived || !stopped.current_capture.new_for_run) {
          throw new Error('本次建图没有生成新的 PCD，请检查雷达连接')
        }
      }
      await api('/api/sessions/archive', { method: 'POST', body: JSON.stringify(form) })
      notify('原始 PCD 已保存')
      onDone()
    } catch (error) {
      notify(error.message, 'error')
    } finally {
      setBusy(false)
    }
  }
  return (
    <Modal title={stopFirst ? '结束建图并保存 PCD' : '保存原始 PCD'} subtitle={capture?.file?.name || 'scans.pcd'} onClose={onClose}>
      <form onSubmit={submit} className="modal-form">
        <Field label="PCD 名称"><input autoFocus value={form.name} onChange={(event) => setForm({ ...form, name: event.target.value })} /></Field>
        <Field label="场地"><input value={form.site} onChange={(event) => setForm({ ...form, site: event.target.value })} /></Field>
        <Field label="备注"><textarea rows="3" value={form.note} onChange={(event) => setForm({ ...form, note: event.target.value })} /></Field>
        <div className="source-summary"><HardDrive size={16} /><span>{algorithmLabel(capture?.algorithm)}{capture?.file?.size_human ? ` · ${capture.file.size_human}` : ''}</span><code>{stopFirst && !capture?.new_for_run ? '停止后自动读取本次建图 PCD' : capture?.source_path}</code></div>
        <div className="modal-actions">{stopFirst && <button type="button" className="button ghost-danger" onClick={onDiscard} disabled={busy}>停止但不保存</button>}<button type="button" className="button secondary" onClick={onClose}>取消</button><button className="button primary" disabled={busy}>{busy && <LoaderCircle className="spin" size={16} />}{stopFirst ? '结束并保存' : '保存 PCD'}</button></div>
      </form>
    </Modal>
  )
}

function BuildModal({ sessions, initialSession, onClose, onDone, notify }) {
  const [form, setForm] = useState({
    session_id: initialSession || sessions[0]?.id || '',
    name: sessions.find((item) => item.id === initialSession)?.name || sessions[0]?.name || '新地图版本',
    note: '', voxel_leaf: 0.25, z_min: 0.4, z_max: 1.5, resolution: 0.05,
  })
  const [busy, setBusy] = useState(false)
  const submit = async (event) => {
    event.preventDefault()
    setBusy(true)
    try {
      await api('/api/versions/build', { method: 'POST', body: JSON.stringify(form) })
      notify('地图构建任务已开始')
      onDone()
    } catch (error) {
      notify(error.message, 'error')
    } finally {
      setBusy(false)
    }
  }
  return (
    <Modal title="从 PCD 生成地图" subtitle="输出 2D 导航地图和 ICP 匹配 PCD" onClose={onClose}>
      <form onSubmit={submit} className="modal-form">
        <Field label="原始 PCD">
          <select value={form.session_id} onChange={(event) => {
            const session = sessions.find((item) => item.id === event.target.value)
            setForm({ ...form, session_id: event.target.value, name: session?.name || form.name })
          }}>
            {sessions.map((session) => <option key={session.id} value={session.id}>{session.name} · {formatTime(session.created_at)}</option>)}
          </select>
        </Field>
        <Field label="地图名称"><input value={form.name} onChange={(event) => setForm({ ...form, name: event.target.value })} /></Field>
        <Field label="备注"><textarea rows="2" value={form.note} onChange={(event) => setForm({ ...form, note: event.target.value })} /></Field>
        <details className="advanced-options"><summary>高级参数</summary><div className="field-grid">
          <Field label="预览体素"><input type="number" step="0.05" value={form.voxel_leaf} onChange={(event) => setForm({ ...form, voxel_leaf: Number(event.target.value) })} /></Field>
          <Field label="2D 分辨率"><input type="number" step="0.01" value={form.resolution} onChange={(event) => setForm({ ...form, resolution: Number(event.target.value) })} /></Field>
          <Field label="高度切片下限"><input type="number" step="0.1" value={form.z_min} onChange={(event) => setForm({ ...form, z_min: Number(event.target.value) })} /></Field>
          <Field label="高度切片上限"><input type="number" step="0.1" value={form.z_max} onChange={(event) => setForm({ ...form, z_max: Number(event.target.value) })} /></Field>
        </div></details>
        <div className="modal-actions"><button type="button" className="button secondary" onClick={onClose}>取消</button><button className="button primary" disabled={busy || !form.session_id}>{busy && <LoaderCircle className="spin" size={16} />}生成 2D + ICP 地图</button></div>
      </form>
    </Modal>
  )
}

function ConfirmPurge({ item, onClose, onConfirm, busy }) {
  return (
    <Modal title="彻底删除" subtitle="此操作不能撤销" onClose={onClose}>
      <div className="confirm-body danger-confirm">
        <Trash2 size={30} />
        <h3>{item.name}</h3>
        <p>该项目将从项目回收站中永久移除。</p>
      </div>
      <div className="modal-actions"><button className="button secondary" onClick={onClose}>取消</button><button className="button danger-button" onClick={onConfirm} disabled={busy}>{busy && <LoaderCircle className="spin" size={16} />}彻底删除</button></div>
    </Modal>
  )
}

export default function App() {
  const [data, setData] = useState(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')
  const [section, setSection] = useState('sessions')
  const [selectedId, setSelectedId] = useState(null)
  const [selectedSessionId, setSelectedSessionId] = useState(null)
  const [legacyId, setLegacyId] = useState(null)
  const [view, setView] = useState('2d')
  const [search, setSearch] = useState('')
  const [modal, setModal] = useState(null)
  const [toast, setToast] = useState(null)
  const [actionBusy, setActionBusy] = useState(false)
  const [mappingAlgorithm, setMappingAlgorithm] = useState(() => window.localStorage.getItem('go2.mappingAlgorithm') || 'faster_lio')
  const handledJobRef = useRef(null)

  const notify = useCallback((message, type = 'success') => {
    setToast({ message, type })
    window.setTimeout(() => setToast(null), 2800)
  }, [])

  const refresh = useCallback(async (quiet = false) => {
    if (!quiet) setLoading(true)
    try {
      const next = await api('/api/overview')
      setData(next)
      setError('')
      setSelectedId((current) => next.versions.some((version) => version.id === current) ? current : next.active?.id || next.versions[0]?.id || null)
      setSelectedSessionId((current) => next.sessions.some((session) => session.id === current) ? current : next.sessions[0]?.id || null)
    } catch (requestError) {
      setError(requestError.message)
    } finally {
      if (!quiet) setLoading(false)
    }
  }, [])

  useEffect(() => { refresh() }, [refresh])
  useEffect(() => { window.localStorage.setItem('go2.mappingAlgorithm', mappingAlgorithm) }, [mappingAlgorithm])
  useEffect(() => {
    const busy = data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)
    const interval = window.setInterval(() => refresh(true), busy ? 1200 : 7000)
    return () => window.clearInterval(interval)
  }, [data?.job?.running, data?.runtime?.status, refresh])
  useEffect(() => {
    const versionId = data?.job?.stage === 'complete' ? data.job.result?.version_id : null
    if (!versionId || handledJobRef.current === versionId || !data?.versions?.some((version) => version.id === versionId)) return
    handledJobRef.current = versionId
    setSelectedId(versionId)
    setSection('versions')
    setView('2d')
    notify('2D 地图和 ICP PCD 已生成')
  }, [data?.job?.result?.version_id, data?.job?.stage, notify])

  const selected = useMemo(
    () => data?.versions.find((item) => item.id === selectedId) || null,
    [data, selectedId],
  )
  const selectedSession = useMemo(
    () => data?.sessions.find((item) => item.id === selectedSessionId) || null,
    [data, selectedSessionId],
  )
  const selectedLegacy = useMemo(
    () => data?.legacy_maps.find((item) => item.id === legacyId) || null,
    [data, legacyId],
  )
  const filteredVersions = useMemo(() => {
    const query = search.trim().toLowerCase()
    if (!query) return data?.versions || []
    return data.versions.filter((item) => `${item.name} ${item.id} ${item.site}`.toLowerCase().includes(query))
  }, [data, search])

  const rollback = async () => {
    setActionBusy(true)
    try {
      await api('/api/active/rollback', { method: 'POST' })
      notify('已回退到上一版地图')
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const activateVersion = async (version) => {
    setActionBusy(true)
    try {
      await api('/api/active', { method: 'POST', body: JSON.stringify({ version_id: version.id }) })
      notify(`${version.name} 已设为定位 / 导航地图`)
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const startRuntime = async (mode) => {
    setActionBusy(true)
    try {
      const payload = mode === 'mapping' ? { mode, algorithm: mappingAlgorithm } : { mode }
      await api('/api/runtime/start', { method: 'POST', body: JSON.stringify(payload) })
      notify(mode === 'mapping' ? `${algorithmLabel(mappingAlgorithm)} 建图已启动` : `${mode === 'localization' ? 'ICP 定位' : '导航'}已使用当前地图启动`)
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const stopRuntime = async () => {
    setActionBusy(true)
    try {
      const result = await api('/api/runtime/stop', { method: 'POST' })
      notify(result.mode === 'mapping' && result.current_capture?.available ? '建图已结束，输出可归档' : '流程已停止并完成清理')
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const discardMapping = async () => {
    setActionBusy(true)
    try {
      await api('/api/runtime/discard-mapping', { method: 'POST' })
      setModal(null)
      notify('建图已停止，本次原始 PCD 已删除')
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const cancelJob = async () => {
    setActionBusy(true)
    try {
      const result = await api('/api/jobs/cancel', { method: 'POST' })
      notify(result.running ? '正在取消地图任务' : '地图任务已取消并完成清理')
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const moveToTrash = async (kind, itemId) => {
    setActionBusy(true)
    try {
      await api('/api/trash', { method: 'POST', body: JSON.stringify({ kind, item_id: itemId }) })
      if (kind === 'version' && selectedId === itemId) setSelectedId(null)
      if (kind === 'session' && selectedSessionId === itemId) setSelectedSessionId(null)
      notify('已移到项目回收站')
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const restoreTrash = async (trashId) => {
    setActionBusy(true)
    try {
      await api(`/api/trash/${trashId}/restore`, { method: 'POST' })
      notify('项目已恢复')
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const purgeTrash = async (item) => {
    setActionBusy(true)
    try {
      await api(`/api/trash/${item.id}`, { method: 'DELETE' })
      notify('已彻底删除')
      setModal(null)
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const adoptLegacy = async () => {
    setActionBusy(true)
    try {
      const result = await api('/api/versions/adopt-current', {
        method: 'POST', body: JSON.stringify({ name: '现有导航地图', note: '从 legacy latest 文件纳管，需实机验证' }),
      })
      setSelectedId(result.version_id)
      setSection('versions')
      notify('现有地图已纳入版本工作区')
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  if (loading && !data) {
    return <div className="boot-screen"><div className="boot-mark"><Map size={28} /><LoaderCircle className="spin" size={18} /></div><span>正在读取地图工作区</span></div>
  }

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand"><div className="brand-mark"><Map size={22} /></div><div><strong>地图工作台</strong><span>GO2 · MID360</span></div></div>
        <div className="active-strip">
          <span>当前使用地图</span>
          {data?.active ? <><i /><strong>{data.active.name}</strong><code>{data.active.id}</code></> : <><b /><strong>尚未激活</strong></>}
        </div>
        <div className="top-actions">
          <IconButton title="刷新工作区" onClick={() => refresh()}><RefreshCw size={17} /></IconButton>
        </div>
      </header>

      {error && <div className="error-banner"><FileWarning size={17} />{error}<button onClick={() => refresh()}>重试</button></div>}

      <section className="runtime-bar">
        <div className={`runtime-state ${data?.runtime?.status || 'idle'}`}>
          <span className="runtime-pulse" />
          <div><strong>{data?.runtime?.status === 'running' ? `${data.runtime.mode === 'mapping' ? `${algorithmLabel(data.runtime.algorithm)} 正在建图` : { localization: '定位运行中', navigation: '导航运行中' }[data.runtime.mode]}` : data?.runtime?.status === 'stopping' ? '正在停止' : data?.runtime?.status === 'failed' ? '流程异常退出' : '系统待命'}</strong><span>{data?.runtime?.status === 'running' || data?.runtime?.status === 'failed' ? data?.runtime?.logs?.at(-1) || '正在启动' : '当前没有运行 ROS 流程'}</span></div>
        </div>
        <div className="core-pipeline" aria-label="地图处理流程">
          <section className={`pipeline-stage ${data?.runtime?.mode === 'mapping' && data?.runtime?.status === 'running' ? 'running' : ''}`}>
            <div className="pipeline-title"><span>1</span><div><strong>建图</strong><select className="mapping-algorithm-select" aria-label="建图算法" value={data?.runtime?.mode === 'mapping' && ['running', 'stopping'].includes(data?.runtime?.status) ? data.runtime.algorithm || mappingAlgorithm : mappingAlgorithm} onChange={(event) => setMappingAlgorithm(event.target.value)} disabled={actionBusy || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><option value="faster_lio">FASTer-LIO</option><option value="fastlio2">FAST-LIO2</option></select></div></div>
            <button onClick={() => data?.runtime?.mode === 'mapping' && data?.runtime?.status === 'running' ? setModal('finish-mapping') : startRuntime('mapping')} disabled={actionBusy || data?.job?.running || data?.runtime?.status === 'stopping' || (data?.runtime?.status === 'running' && data?.runtime?.mode !== 'mapping')}>
              {data?.runtime?.mode === 'mapping' && data?.runtime?.status === 'running' ? <><Square size={16} />结束并保存 PCD</> : <><Map size={16} />开始建图</>}
            </button>
          </section>
          <ChevronRight className="pipeline-arrow" size={18} />
          <section className={`pipeline-stage ${data?.job?.running ? 'running' : ''}`}>
            <div className="pipeline-title"><span>2</span><div><strong>处理 PCD</strong><small>输出：2D 地图 + ICP PCD</small></div></div>
            <button onClick={() => setModal('build')} disabled={actionBusy || !data?.sessions?.length || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><Layers3 size={16} />{data?.job?.running ? '正在生成' : '选择 PCD 并生成'}</button>
          </section>
          <ChevronRight className="pipeline-arrow" size={18} />
          <section className={`pipeline-stage run-stage ${['localization', 'navigation'].includes(data?.runtime?.mode) && data?.runtime?.status === 'running' ? 'running' : ''}`}>
            <div className="pipeline-title"><span>3</span><div><strong>使用地图</strong><small>{data?.active?.name || '先设定定位 / 导航地图'}</small></div></div>
            {['localization', 'navigation'].includes(data?.runtime?.mode) && ['running', 'stopping'].includes(data?.runtime?.status)
              ? <button className="stop-map" onClick={stopRuntime} disabled={actionBusy || data.runtime.status === 'stopping'}><Square size={16} />停止并清理</button>
              : <div className="map-run-actions"><button onClick={() => startRuntime('localization')} disabled={actionBusy || !data?.active?.complete || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><MapPin size={16} />ICP 定位</button><button onClick={() => startRuntime('navigation')} disabled={actionBusy || !data?.active?.complete || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><Navigation size={16} />导航</button></div>}
          </section>
        </div>
      </section>

      <div className="workspace">
        <aside className="sidebar">
          <div className="section-tabs">
            <button className={section === 'sessions' ? 'active' : ''} onClick={() => setSection('sessions')}><HardDrive size={16} />原始 PCD <span>{data?.sessions.length || 0}</span></button>
            <button className={section === 'versions' ? 'active' : ''} onClick={() => setSection('versions')}><Database size={16} />生成地图 <span>{data?.versions.length || 0}</span></button>
            <button className={section === 'trash' ? 'active' : ''} onClick={() => setSection('trash')}><Trash2 size={16} />回收站 <span>{data?.trash.length || 0}</span></button>
            <button className={section === 'legacy' ? 'active' : ''} onClick={() => setSection('legacy')}><History size={16} />历史文件</button>
          </div>

          {section === 'versions' && (
            <>
              <div className="search"><Search size={15} /><input value={search} onChange={(event) => setSearch(event.target.value)} placeholder="搜索名称、场地、版本" /></div>
              <div className="item-list">
                {filteredVersions.map((version) => (
                  <button key={version.id} className={`version-row ${selectedId === version.id ? 'selected' : ''}`} onClick={() => { setSelectedId(version.id); setView('2d') }}>
                    <div className="version-thumb">{version.map_preview_url ? <img src={version.map_preview_url} alt="" /> : <Map size={20} />}</div>
                    <div className="version-copy"><div><strong>{version.name}</strong><StatusBadge status={version.status} complete={version.complete} /></div><span>{version.site} · {formatTime(version.created_at)}</span><code>{version.id}</code></div>
                    <ChevronRight size={16} />
                  </button>
                ))}
                {!filteredVersions.length && <div className="list-empty"><Database size={22} /><span>还没有受管地图版本</span></div>}
              </div>
            </>
          )}

          {section === 'sessions' && (
            <div className="item-list session-list">
              {data?.current_capture.available && <div className="current-capture"><div><Activity size={16} /><strong>当前原始 PCD</strong><span>{data.current_capture.archived ? '已保存' : data.current_capture.new_for_run ? '本次未保存' : '历史残留'}</span></div><p>{algorithmLabel(data.current_capture.algorithm)} · {data.current_capture.file.size_human} · {formatTime(data.current_capture.file.modified_at)}</p><button className="button secondary wide" onClick={() => setModal('archive')} disabled={data.current_capture.archived || !data.current_capture.new_for_run}><Archive size={15} />{data.current_capture.archived ? 'PCD 已在列表中' : data.current_capture.new_for_run ? '保存这个 PCD' : '不能保存历史残留'}</button></div>}
              {data?.sessions.map((session) => (
                <div
                  className={`session-row ${selectedSessionId === session.id ? 'selected' : ''}`}
                  key={session.id}
                  role="button"
                  tabIndex={0}
                  onClick={() => setSelectedSessionId(session.id)}
                  onKeyDown={(event) => { if (event.key === 'Enter' || event.key === ' ') setSelectedSessionId(session.id) }}
                >
                  <div className="session-icon"><HardDrive size={18} /></div>
                  <div><strong>{session.name}</strong><span>{algorithmLabel(session.algorithm)} · {session.site} · {formatTime(session.created_at)}</span><code>{session.raw?.size_human || '文件缺失'}</code></div>
                  <div className="row-actions" onClick={(event) => event.stopPropagation()}><IconButton title="从该 PCD 生成地图" onClick={() => setModal({ type: 'build', sessionId: session.id })}><Layers3 size={16} /></IconButton><IconButton title="移到回收站" onClick={() => moveToTrash('session', session.id)} disabled={actionBusy}><Trash2 size={16} /></IconButton></div>
                </div>
              ))}
              {!data?.sessions.length && <div className="list-empty"><HardDrive size={24} /><strong>还没有原始 PCD</strong><span>完成一次建图并保存后会显示在这里</span></div>}
            </div>
          )}

          {section === 'trash' && (
            <div className="item-list trash-list">
              {data?.trash.map((item) => (
                <div className="trash-row" key={item.id}><div className="trash-icon"><Trash2 size={19} /></div><div><strong>{item.name}</strong><span>{item.kind === 'version' ? '生成地图' : '原始 PCD'} · {item.size_human}</span><code>{formatTime(item.deleted_at)}</code></div><div className="row-actions"><IconButton title="恢复" onClick={() => restoreTrash(item.id)} disabled={actionBusy}><Undo2 size={17} /></IconButton><IconButton title="彻底删除" onClick={() => setModal({ type: 'purge', item })} disabled={actionBusy}><X size={17} /></IconButton></div></div>
              ))}
              {!data?.trash.length && <div className="list-empty"><Trash2 size={24} /><strong>回收站为空</strong><span>删除的版本和会话会保留在这里</span></div>}
            </div>
          )}

          {section === 'legacy' && (
            <div className="item-list legacy-list">
              <div className="legacy-notice"><FileWarning size={17} /><p>旧地图没有成对保存 ICP PCD，只能预览，不能直接激活。</p></div>
              {data?.legacy_maps.map((mapItem) => (
                <button key={mapItem.id} className={`legacy-row ${legacyId === mapItem.id ? 'selected' : ''}`} onClick={() => setLegacyId(mapItem.id)}><div className="version-thumb">{mapItem.map_preview_url && <img src={mapItem.map_preview_url} alt="" />}</div><div><strong>{mapItem.name}</strong><span>{mapItem.pgm?.size_human || 'PGM 缺失'}</span></div><ChevronRight size={16} /></button>
              ))}
              {data?.legacy_icp && <button className="button secondary wide adopt" onClick={adoptLegacy} disabled={actionBusy}>{actionBusy ? <LoaderCircle className="spin" size={16} /> : <FolderInput size={16} />}纳管当前 latest 组合</button>}
            </div>
          )}
        </aside>

        <main className="preview-area">
          {(section === 'sessions' && selectedSession) ? (
            <>
              <div className="preview-toolbar raw-toolbar"><div><strong>{selectedSession.name}</strong><span>{algorithmLabel(selectedSession.algorithm)} / 原始 PCD / {selectedSession.site} / {selectedSession.raw?.size_human}</span></div><span className="badge raw-badge">原始数据</span></div>
              <div className="map-stage"><Suspense fallback={<div className="canvas-overlay"><LoaderCircle className="spin" size={20} />正在载入 3D 模块</div>}><PointCloudView url={selectedSession.cloud_preview_url} /></Suspense></div>
              <div className="stage-footer"><span><MapPin size={14} />{selectedSession.site}</span><span><Clock3 size={14} />{formatTime(selectedSession.created_at)}</span><span><HardDrive size={14} />{algorithmLabel(selectedSession.algorithm)} · {selectedSession.raw?.size_human}</span></div>
            </>
          ) : (section === 'legacy' && selectedLegacy) ? (
            <><div className="preview-toolbar"><div><strong>{selectedLegacy.name}</strong><span>只读旧地图</span></div><span className="badge muted">仅 2D</span></div><div className="map-stage legacy-stage"><img src={selectedLegacy.map_preview_url} alt={`${selectedLegacy.name} 栅格地图`} /></div></>
          ) : (section === 'versions' && selected) ? (
            <>
              <div className="preview-toolbar">
                <div><strong>{selected.name}</strong><span>{selected.site} / {selected.size_human}</span></div>
                <div className="view-switch"><button className={view === '2d' ? 'active' : ''} onClick={() => setView('2d')}><ImageIcon size={16} />2D 栅格</button><button className={view === '3d' ? 'active' : ''} onClick={() => setView('3d')}><Box size={16} />3D 点云</button></div>
                <div className="toolbar-status"><StatusBadge status={selected.status} complete={selected.complete} /></div>
              </div>
              <div className="map-stage">
                {view === '2d' ? <div className="map-image-wrap"><img src={selected.map_preview_url} alt={`${selected.name} 栅格地图`} /><div className="map-grid" /></div> : <Suspense fallback={<div className="canvas-overlay"><LoaderCircle className="spin" size={20} />正在载入 3D 模块</div>}><PointCloudView url={selected.cloud_preview_url} /></Suspense>}
              </div>
              <div className="stage-footer"><span><MapPin size={14} />{selected.site}</span><span><Clock3 size={14} />{formatTime(selected.created_at)}</span><span><HardDrive size={14} />{algorithmLabel(selected.mapping_algorithm)} · {selected.size_human}</span></div>
            </>
          ) : section === 'sessions' ? <div className="empty-stage"><div className="empty-map-symbol"><HardDrive size={34} /></div><h2>还没有原始 PCD</h2><div><button className="button primary" onClick={() => startRuntime('mapping')}><Map size={16} />开始建图</button></div></div>
            : section === 'versions' ? <div className="empty-stage"><div className="empty-map-symbol"><Map size={34} /></div><h2>还没有生成地图</h2><div><button className="button secondary" onClick={() => setSection('legacy')}><History size={16} />查看旧地图</button><button className="button primary" onClick={() => setModal('build')} disabled={!data?.sessions?.length}><Layers3 size={16} />从 PCD 生成</button></div></div>
              : <div className="empty-stage"><div className="empty-map-symbol"><SlidersHorizontal size={34} /></div></div>}
        </main>

        <aside className="inspector">
          {section === 'sessions' && selectedSession ? (
            <>
              <div className="inspector-head"><div><span>原始 PCD</span><strong>{selectedSession.name}</strong></div></div>
              {!selectedSession.complete && <div className="issue-panel"><FileWarning size={17} /><div><strong>PCD 文件缺失</strong></div></div>}
              <section className="detail-section"><h3>建图记录</h3><dl><div><dt>记录 ID</dt><dd><code>{selectedSession.id}</code></dd></div><div><dt>建图算法</dt><dd>{algorithmLabel(selectedSession.algorithm)}</dd></div><div><dt>场地</dt><dd>{selectedSession.site}</dd></div><div><dt>时间</dt><dd>{formatTime(selectedSession.created_at)}</dd></div><div><dt>备注</dt><dd>{selectedSession.note || '无'}</dd></div></dl></section>
              {selectedSession.raw && <section className="detail-section"><h3>原始文件</h3><PathRow label="PCD" value={selectedSession.raw.path} notify={notify} /></section>}
              <div className="inspector-actions"><button className="button primary wide" onClick={() => setModal({ type: 'build', sessionId: selectedSession.id })} disabled={!selectedSession.complete || actionBusy || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><Layers3 size={16} />生成 2D + ICP 地图</button><button className="button ghost-danger wide" onClick={() => moveToTrash('session', selectedSession.id)} disabled={actionBusy}><Trash2 size={16} />移到回收站</button></div>
            </>
          ) : section === 'versions' && selected ? (
            <>
              <div className="inspector-head"><div><span>版本详情</span><strong>{selected.name}</strong></div></div>
              {!selected.complete && <div className="issue-panel"><FileWarning size={17} /><div><strong>地图包不完整</strong>{selected.issues.map((issue) => <span key={issue}>{issue}</span>)}</div></div>}
              <section className="detail-section"><h3>身份</h3><dl><div><dt>版本</dt><dd><code>{selected.id}</code></dd></div><div><dt>建图算法</dt><dd>{algorithmLabel(selected.mapping_algorithm)}</dd></div><div><dt>来源</dt><dd>{selected.origin}</dd></div><div><dt>建图记录</dt><dd>{selected.source_session || '历史导入'}</dd></div><div><dt>备注</dt><dd>{selected.note || '无'}</dd></div></dl></section>
              <section className="detail-section"><h3>生成文件</h3><PathRow label="2D地图" value={selected.paths.map_yaml} notify={notify} /><PathRow label="ICP PCD" value={selected.paths.localization_pcd} notify={notify} /></section>
              <div className="inspector-actions">
                {selected.status === 'active' ? <div className="active-confirm"><Check size={17} /><span><strong>当前定位 / 导航地图</strong><small>2D 地图与 ICP PCD 已绑定使用</small></span></div> : <><div className="active-confirm selection-confirm"><MapPin size={17} /><span><strong>候选地图</strong><small>当前仅查看，尚未用于机器人</small></span></div><button className="button primary wide" onClick={() => activateVersion(selected)} disabled={!selected.complete || actionBusy || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><Navigation size={16} />设为定位 / 导航地图</button></>}
                <button className="button secondary wide" onClick={rollback} disabled={!data?.previous_active_id || actionBusy}><RotateCcw size={16} />回退上一版</button>
                <button className="button ghost-danger wide" onClick={() => moveToTrash('version', selected.id)} disabled={selected.status === 'active' || actionBusy}><Trash2 size={16} />移到回收站</button>
              </div>
              <section className="detail-section"><h3>构建参数</h3>{Object.keys(selected.parameters || {}).length ? <dl>{Object.entries(selected.parameters).map(([key, value]) => <div key={key}><dt>{key}</dt><dd>{String(value)}</dd></div>)}</dl> : <p className="muted-copy">旧地图未记录构建参数</p>}</section>
            </>
          ) : <div className="inspector-empty"><SlidersHorizontal size={22} /><span>{section === 'sessions' ? '选择一个原始 PCD 查看详情' : section === 'versions' ? '选择一套生成地图查看详情' : '从左侧选择项目'}</span></div>}
        </aside>
      </div>

      {data?.job?.running && <div className="job-drawer"><div className="job-progress" style={{ width: `${data.job.progress}%` }} /><div><LoaderCircle className="spin" size={17} /><strong>地图任务进行中</strong><span>{data.job.stage}</span></div><code>{data.job.logs.at(-1) || '等待任务输出'}</code><b>{data.job.progress}%</b><IconButton title="取消地图任务" onClick={cancelJob} disabled={actionBusy}><X size={17} /></IconButton></div>}
      {toast && <div className={`toast ${toast.type}`}>{toast.type === 'success' ? <Check size={16} /> : <FileWarning size={16} />}{toast.message}</div>}
      {modal === 'archive' && <ArchiveModal capture={data?.current_capture} onClose={() => setModal(null)} onDone={() => { setModal(null); setSection('sessions'); refresh(true) }} notify={notify} />}
      {modal === 'finish-mapping' && <ArchiveModal capture={data?.current_capture} stopFirst onClose={() => setModal(null)} onDiscard={discardMapping} onDone={() => { setModal(null); setSection('sessions'); refresh(true) }} notify={notify} />}
      {(modal === 'build' || modal?.type === 'build') && <BuildModal sessions={data?.sessions || []} initialSession={modal?.sessionId} onClose={() => setModal(null)} onDone={() => { setModal(null); refresh(true) }} notify={notify} />}
      {modal?.type === 'purge' && <ConfirmPurge item={modal.item} onClose={() => setModal(null)} onConfirm={() => purgeTrash(modal.item)} busy={actionBusy} />}
    </div>
  )
}
