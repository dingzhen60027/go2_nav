import { lazy, Suspense, useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  Activity,
  ArrowDown,
  ArrowUp,
  Archive,
  Box,
  Check,
  ChevronRight,
  CircleStop,
  Clock3,
  Copy,
  Crosshair,
  Database,
  Eraser,
  FileWarning,
  Flag,
  FolderInput,
  HardDrive,
  History,
  Image as ImageIcon,
  Layers3,
  ListChecks,
  LoaderCircle,
  Map,
  MapPin,
  Navigation,
  Pencil,
  Play,
  RefreshCw,
  RotateCcw,
  Route,
  Save,
  Search,
  SlidersHorizontal,
  Square,
  Trash2,
  Undo2,
  X,
} from 'lucide-react'

const PointCloudView = lazy(() => import('./PointCloudView.jsx'))
const ClusterEditor = lazy(() => import('./ClusterEditor.jsx'))
const MapEditor = lazy(() => import('./MapEditor.jsx'))

const MAPPING_ALGORITHMS = {
  faster_lio: 'FASTer-LIO',
  fastlio2: 'FAST-LIO2',
  unknown: '算法未记录',
}

const LOCALIZATION_MODULES = {
  fused_ekf: '融合定位',
  pure_icp: '纯 ICP',
}

const DEFAULT_BUILD_PARAMETERS = {
  statistical_mean_k: 20,
  statistical_std_dev_mul: 0.5,
  radius: 0.3,
  radius_min_points: 4,
  voxel_leaf: 0.15,
  z_min: 0.4,
  z_max: 1.5,
  resolution: 0.05,
}

const BUILD_PARAMETER_KEYS = Object.keys(DEFAULT_BUILD_PARAMETERS)

function buildParameterPayload(source) {
  return Object.fromEntries(BUILD_PARAMETER_KEYS.map((key) => [key, source[key]]))
}

function algorithmLabel(value) {
  return MAPPING_ALGORITHMS[value] || MAPPING_ALGORITHMS.unknown
}

function localizationModuleLabel(value) {
  return LOCALIZATION_MODULES[value] || '定位模块'
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

function formatDuration(value) {
  const seconds = Math.max(0, Math.round(Number(value) || 0))
  const minutes = Math.floor(seconds / 60)
  const remainder = seconds % 60
  return minutes ? `${minutes}分 ${remainder}秒` : `${remainder}秒`
}

function missionStatusLabel(status) {
  return {
    idle: '未开始',
    queued: '正在启动',
    waiting_server: '连接 Nav2',
    running: '导航中',
    cancelling: '正在取消',
    succeeded: '全部完成',
    partial: '部分完成',
    failed: '任务失败',
    cancelled: '已取消',
  }[status] || status
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

function BuildModal({ sessions, profiles, initialSession, onClose, onDone, onProfilesChanged, notify }) {
  const [form, setForm] = useState({
    session_id: initialSession || sessions[0]?.id || '',
    name: sessions.find((item) => item.id === initialSession)?.name || sessions[0]?.name || '新地图版本',
    note: '', ...DEFAULT_BUILD_PARAMETERS,
  })
  const [busy, setBusy] = useState(false)
  const [profileBusy, setProfileBusy] = useState(false)
  const [selectedProfileId, setSelectedProfileId] = useState('')
  const [profileName, setProfileName] = useState('')

  const setNumber = (key, value) => setForm((current) => ({ ...current, [key]: Number(value) }))
  const loadProfile = (profileId) => {
    setSelectedProfileId(profileId)
    const profile = profiles.find((item) => item.id === profileId)
    if (!profile) return
    setProfileName(profile.name)
    setForm((current) => ({ ...current, ...profile.parameters }))
    notify(`已加载参数方案：${profile.name}`)
  }
  const saveProfile = async () => {
    const name = profileName.trim()
    if (!name) {
      notify('请输入参数方案名称', 'error')
      return
    }
    setProfileBusy(true)
    try {
      const result = await api('/api/build-profiles', {
        method: 'POST',
        body: JSON.stringify({ name, ...buildParameterPayload(form) }),
      })
      setSelectedProfileId(result.profile.id)
      notify(result.created ? '参数方案已保存' : '同名参数方案已更新')
      await onProfilesChanged()
    } catch (error) {
      notify(error.message, 'error')
    } finally {
      setProfileBusy(false)
    }
  }
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
        <section className="parameter-profile">
          <div className="parameter-profile-head"><SlidersHorizontal size={16} /><div><strong>参数方案</strong><span>保存在地图工作区，可重复加载</span></div></div>
          <Field label="加载方案"><select value={selectedProfileId} onChange={(event) => loadProfile(event.target.value)}><option value="">当前参数（未加载方案）</option>{profiles.map((profile) => <option key={profile.id} value={profile.id}>{profile.name}</option>)}</select></Field>
          <div className="parameter-profile-save"><input maxLength="80" placeholder="方案名称，例如：仓库高密度" value={profileName} onChange={(event) => setProfileName(event.target.value)} /><button type="button" className="button secondary" onClick={saveProfile} disabled={profileBusy || !profileName.trim()}>{profileBusy ? <LoaderCircle className="spin" size={15} /> : <Save size={15} />}保存方案</button></div>
          <small>使用同名方案保存时会更新原方案。</small>
        </section>
        <details className="advanced-options" open><summary>点云滤波与地图参数</summary><div className="field-grid">
          <div className="parameter-group-title">统计离群点过滤</div>
          <Field label="邻域点数量 mean_k"><input type="number" min="2" max="500" step="1" value={form.statistical_mean_k} onChange={(event) => setNumber('statistical_mean_k', event.target.value)} /></Field>
          <Field label="标准差倍数 std_dev"><input type="number" min="0.05" max="10" step="0.05" value={form.statistical_std_dev_mul} onChange={(event) => setNumber('statistical_std_dev_mul', event.target.value)} /></Field>
          <div className="parameter-group-title">半径离群点过滤</div>
          <Field label="搜索半径 radius (m)"><input type="number" min="0.01" max="5" step="0.01" value={form.radius} onChange={(event) => setNumber('radius', event.target.value)} /></Field>
          <Field label="最少邻点 min_pts"><input type="number" min="1" max="500" step="1" value={form.radius_min_points} onChange={(event) => setNumber('radius_min_points', event.target.value)} /></Field>
          <div className="parameter-group-title">输出与预览</div>
          <Field label="3D 预览体素 (m)" hint="用于 Web 中显示生成后的 ICP 点云；实际 ICP 匹配体素由两个定位包各自固定为 0.15m"><input type="number" min="0.05" max="1" step="0.05" value={form.voxel_leaf} onChange={(event) => setNumber('voxel_leaf', event.target.value)} /></Field>
          <Field label="2D 分辨率 (m/格)"><input type="number" min="0.01" max="0.5" step="0.01" value={form.resolution} onChange={(event) => setNumber('resolution', event.target.value)} /></Field>
          <Field label="2D 高度下限 z_min (m)"><input type="number" min="-10" max="10" step="0.1" value={form.z_min} onChange={(event) => setNumber('z_min', event.target.value)} /></Field>
          <Field label="2D 高度上限 z_max (m)"><input type="number" min="-10" max="10" step="0.1" value={form.z_max} onChange={(event) => setNumber('z_max', event.target.value)} /></Field>
        </div></details>
        <div className="modal-actions"><button type="button" className="button secondary" onClick={onClose}>取消</button><button className="button primary" disabled={busy || !form.session_id}>{busy && <LoaderCircle className="spin" size={16} />}生成 2D + ICP 地图</button></div>
      </form>
    </Modal>
  )
}

function RenameModal({ kind, item, onClose, onDone, notify }) {
  const [name, setName] = useState(item.name)
  const [busy, setBusy] = useState(false)
  const label = kind === 'session' ? '原始 PCD' : '2D + ICP 地图'
  const submit = async (event) => {
    event.preventDefault()
    setBusy(true)
    try {
      const path = kind === 'session' ? `/api/sessions/${item.id}` : `/api/versions/${item.id}`
      await api(path, { method: 'PATCH', body: JSON.stringify({ name: name.trim() }) })
      notify(`${label}已重命名`)
      await onDone()
    } catch (error) {
      notify(error.message, 'error')
    } finally {
      setBusy(false)
    }
  }
  return (
    <Modal title={`重命名${label}`} subtitle={item.id} onClose={onClose}>
      <form onSubmit={submit} className="modal-form">
        <Field label="名称" hint="仅修改 Web 中的显示名称，不改变内部 ID 和文件路径">
          <input autoFocus maxLength="80" value={name} onChange={(event) => setName(event.target.value)} />
        </Field>
        <div className="modal-actions"><button type="button" className="button secondary" onClick={onClose}>取消</button><button className="button primary" disabled={busy || !name.trim()}>{busy && <LoaderCircle className="spin" size={16} />}保存名称</button></div>
      </form>
    </Modal>
  )
}

function ConfirmPurge({ items, onClose, onConfirm, busy }) {
  const count = items.length
  return (
    <Modal title="彻底删除" subtitle="此操作不能撤销" onClose={onClose}>
      <div className="confirm-body danger-confirm">
        <Trash2 size={30} />
        <h3>{count === 1 ? items[0].name : `${count} 个项目`}</h3>
        <p>{count === 1 ? '该项目将从项目回收站中永久移除。' : `所选 ${count} 个项目将从回收站中永久移除，且不能恢复。`}</p>
      </div>
      <div className="modal-actions"><button className="button secondary" onClick={onClose}>取消</button><button className="button danger-button" onClick={onConfirm} disabled={busy}>{busy && <LoaderCircle className="spin" size={16} />}彻底删除</button></div>
    </Modal>
  )
}

function ConfirmProcessCleanup({ onClose, onConfirm, busy }) {
  return (
    <Modal title="清理所有项目进程" subtitle="Web 服务会继续运行" onClose={onClose}>
      <div className="confirm-body danger-confirm">
        <CircleStop size={30} />
        <h3>强制结束 ROS 运行栈</h3>
        <p>将取消当前导航目标，并关闭本项目启动的建图、定位、Nav2、RViz、雷达驱动和遗留静态 TF。正在写入的建图文件可能不完整，其他普通程序不会被处理。</p>
      </div>
      <div className="modal-actions"><button className="button secondary" onClick={onClose} disabled={busy}>取消</button><button className="button danger-button" onClick={onConfirm} disabled={busy}>{busy && <LoaderCircle className="spin" size={16} />}确认清理</button></div>
    </Modal>
  )
}

function WaypointRenameModal({ waypoint, onClose, onDone, notify }) {
  const [name, setName] = useState(waypoint.name)
  const [busy, setBusy] = useState(false)
  const submit = async (event) => {
    event.preventDefault()
    setBusy(true)
    try {
      await api(`/api/waypoints/${waypoint.id}`, {
        method: 'PATCH', body: JSON.stringify({ name: name.trim() }),
      })
      notify('目标点名称已保存')
      await onDone()
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setBusy(false)
    }
  }
  return (
    <Modal title="重命名目标点" subtitle="导航顺序和记录坐标不会改变" onClose={onClose}>
      <form onSubmit={submit} className="modal-form">
        <Field label="目标点名称"><input autoFocus maxLength="80" value={name} onChange={(event) => setName(event.target.value)} /></Field>
        <div className="modal-actions"><button type="button" className="button secondary" onClick={onClose}>取消</button><button className="button primary" disabled={busy || !name.trim()}>{busy && <LoaderCircle className="spin" size={16} />}保存名称</button></div>
      </form>
    </Modal>
  )
}

function ConfirmWaypointDelete({ waypoint, onClose, onConfirm, busy }) {
  return (
    <Modal title="删除目标点" subtitle="只删除这条导航记录，不会删除地图" onClose={onClose}>
      <div className="confirm-body danger-confirm"><Trash2 size={30} /><h3>{waypoint.name}</h3><p>删除后将从后续多目标导航任务中移除。</p></div>
      <div className="modal-actions"><button className="button secondary" onClick={onClose}>取消</button><button className="button danger-button" onClick={onConfirm} disabled={busy}>{busy && <LoaderCircle className="spin" size={16} />}删除目标点</button></div>
    </Modal>
  )
}

function MissionProgress({ mission, compact = false }) {
  const progress = Math.max(0, Math.min(100, Number(mission?.progress_percent) || 0))
  const active = ['queued', 'waiting_server', 'running', 'cancelling'].includes(mission?.status)
  return (
    <section className={`mission-progress-card status-${mission?.status || 'idle'} ${compact ? 'compact' : ''}`}>
      <div className="mission-progress-head">
        <div className="mission-status-icon">{active ? <LoaderCircle className="spin" size={18} /> : mission?.status === 'succeeded' ? <Check size={18} /> : mission?.status === 'failed' ? <FileWarning size={18} /> : <Route size={18} />}</div>
        <div><span>{missionStatusLabel(mission?.status || 'idle')}</span><strong>{mission?.message || '尚未开始导航任务'}</strong></div>
        <b>{progress.toFixed(1)}%</b>
      </div>
      <div className="mission-progress-track" role="progressbar" aria-label="多目标导航任务进度" aria-valuemin="0" aria-valuemax="100" aria-valuenow={progress}>
        <div style={{ width: `${progress}%` }} />
      </div>
      <div className="mission-progress-meta">
        <span>已处理 <strong>{mission?.processed || 0}/{mission?.total || 0}</strong></span>
        {mission?.current_waypoint_name && <span>当前 <strong>{mission.current_waypoint_name}</strong></span>}
        {mission?.distance_remaining != null && <span>剩余 <strong>{Number(mission.distance_remaining).toFixed(2)} m</strong></span>}
        <span>用时 <strong>{formatDuration(mission?.elapsed_sec)}</strong></span>
      </div>
      {!compact && mission?.error && <div className="mission-error"><FileWarning size={15} />{mission.error}</div>}
    </section>
  )
}

export default function App() {
  const [data, setData] = useState(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')
  const [section, setSection] = useState('sessions')
  const [selectedId, setSelectedId] = useState(null)
  const [selectedSessionId, setSelectedSessionId] = useState(null)
  const [selectedWaypointId, setSelectedWaypointId] = useState(null)
  const [legacyId, setLegacyId] = useState(null)
  const [view, setView] = useState('2d')
  const [search, setSearch] = useState('')
  const [modal, setModal] = useState(null)
  const [toast, setToast] = useState(null)
  const [actionBusy, setActionBusy] = useState(false)
  const [selectedTrashIds, setSelectedTrashIds] = useState([])
  const [mappingAlgorithm, setMappingAlgorithm] = useState(() => window.localStorage.getItem('go2.mappingAlgorithm') || 'faster_lio')
  const [localizationModule, setLocalizationModule] = useState(() => window.localStorage.getItem('go2.localizationModule') || 'fused_ekf')
  const [missionConfig, setMissionConfig] = useState(() => {
    try {
      return { stop_on_failure: true, waypoint_timeout_sec: 300, pause_between_sec: 0, ...JSON.parse(window.localStorage.getItem('go2.waypointMissionConfig') || '{}') }
    } catch {
      return { stop_on_failure: true, waypoint_timeout_sec: 300, pause_between_sec: 0 }
    }
  })
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
      setSelectedWaypointId((current) => next.waypoints?.items?.some((waypoint) => waypoint.id === current) ? current : next.waypoints?.items?.[0]?.id || null)
    } catch (requestError) {
      setError(requestError.message)
    } finally {
      if (!quiet) setLoading(false)
    }
  }, [])

  useEffect(() => { refresh() }, [refresh])
  useEffect(() => { window.localStorage.setItem('go2.mappingAlgorithm', mappingAlgorithm) }, [mappingAlgorithm])
  useEffect(() => { window.localStorage.setItem('go2.localizationModule', localizationModule) }, [localizationModule])
  useEffect(() => { window.localStorage.setItem('go2.waypointMissionConfig', JSON.stringify(missionConfig)) }, [missionConfig])
  useEffect(() => {
    const available = new Set((data?.trash || []).map((item) => item.id))
    setSelectedTrashIds((current) => current.filter((id) => available.has(id)))
  }, [data?.trash])
  useEffect(() => {
    const busy = data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status) || data?.waypoint_mission?.active
    const interval = window.setInterval(() => refresh(true), busy ? 900 : 7000)
    return () => window.clearInterval(interval)
  }, [data?.job?.running, data?.runtime?.status, data?.waypoint_mission?.active, refresh])
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
  const waypoints = data?.waypoints?.items || []
  const mission = data?.waypoint_mission || { status: 'idle', progress_percent: 0, active: false }
  const selectedWaypoint = waypoints.find((item) => item.id === selectedWaypointId) || null
  const navigationRunning = data?.runtime?.status === 'running' && data?.runtime?.mode === 'navigation'
  const selectedLegacy = useMemo(
    () => data?.legacy_maps.find((item) => item.id === legacyId) || null,
    [data, legacyId],
  )
  const selectedTrashItems = useMemo(() => {
    const selectedIds = new Set(selectedTrashIds)
    return (data?.trash || []).filter((item) => selectedIds.has(item.id))
  }, [data?.trash, selectedTrashIds])
  const allTrashSelected = Boolean(data?.trash?.length) && selectedTrashItems.length === data.trash.length
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
      const payload = mode === 'mapping'
        ? { mode, algorithm: mappingAlgorithm }
        : ['localization', 'navigation'].includes(mode)
          ? { mode, algorithm: localizationModule }
          : { mode }
      const result = await api('/api/runtime/start', { method: 'POST', body: JSON.stringify(payload) })
      const archivedNote = result.auto_archived_session_id ? '；上一份待归档 PCD 已自动保存' : ''
      notify((mode === 'mapping'
        ? `${algorithmLabel(mappingAlgorithm)} 建图已启动`
        : `${localizationModuleLabel(localizationModule)}${mode === 'navigation' ? '导航' : ''}已使用当前地图启动`) + archivedNote)
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
      if (result.save_error) {
        notify(result.save_error, 'error')
      } else {
        notify(result.mode === 'mapping' && result.current_capture?.available ? '建图已结束，PCD 已保存，可以归档' : '流程已停止并完成清理')
      }
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const cleanupProcesses = async () => {
    setActionBusy(true)
    try {
      const result = await api('/api/runtime/cleanup', { method: 'POST' })
      setModal(null)
      if (result.auto_archived_session_id) {
        notify('进程已清理，刚才的建图 PCD 已自动归档')
      } else if (result.remaining_count) {
        notify(`已结束 ${result.stopped_count} 个进程，仍有 ${result.remaining_count} 个未退出`, 'error')
      } else if (result.errors?.length) {
        notify(`清理完成，但有警告：${result.errors.join('；')}`, 'error')
      } else {
        notify(result.stopped_count ? `已清理 ${result.stopped_count} 个项目进程` : '没有发现需要清理的项目进程')
      }
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

  const captureWaypoint = async () => {
    setActionBusy(true)
    try {
      const result = await api('/api/waypoints/capture', {
        method: 'POST', body: JSON.stringify({ name: null }),
      })
      setSelectedWaypointId(result.waypoint.id)
      notify(`已记录 ${result.waypoint.name}`)
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const reorderWaypoint = async (waypointId, direction) => {
    const index = waypoints.findIndex((item) => item.id === waypointId)
    const nextIndex = index + direction
    if (index < 0 || nextIndex < 0 || nextIndex >= waypoints.length) return
    const reordered = [...waypoints]
    ;[reordered[index], reordered[nextIndex]] = [reordered[nextIndex], reordered[index]]
    setActionBusy(true)
    try {
      await api('/api/waypoints/reorder', {
        method: 'PUT', body: JSON.stringify({ waypoint_ids: reordered.map((item) => item.id) }),
      })
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const deleteWaypoint = async (waypoint) => {
    setActionBusy(true)
    try {
      await api(`/api/waypoints/${waypoint.id}`, { method: 'DELETE' })
      setModal(null)
      setSelectedWaypointId(null)
      notify(`${waypoint.name} 已删除`)
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const startWaypointMission = async () => {
    setActionBusy(true)
    try {
      await api('/api/waypoint-mission/start', {
        method: 'POST', body: JSON.stringify(missionConfig),
      })
      notify('多目标导航任务已开始')
      await refresh(true)
    } catch (requestError) {
      notify(requestError.message, 'error')
    } finally {
      setActionBusy(false)
    }
  }

  const cancelWaypointMission = async () => {
    setActionBusy(true)
    try {
      await api('/api/waypoint-mission/cancel', { method: 'POST' })
      notify('取消请求已发送，正在安全停止机器人')
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

  const toggleTrashSelection = (trashId) => {
    setSelectedTrashIds((current) => current.includes(trashId)
      ? current.filter((id) => id !== trashId)
      : [...current, trashId])
  }

  const toggleAllTrash = () => {
    setSelectedTrashIds(allTrashSelected ? [] : (data?.trash || []).map((item) => item.id))
  }

  const purgeTrash = async (items) => {
    setActionBusy(true)
    try {
      const trashIds = items.map((item) => item.id)
      await api('/api/trash/purge', { method: 'POST', body: JSON.stringify({ trash_ids: trashIds }) })
      setSelectedTrashIds((current) => current.filter((id) => !trashIds.includes(id)))
      notify(items.length === 1 ? '已彻底删除' : `已彻底删除 ${items.length} 个项目`)
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
          <button type="button" className="button emergency-cleanup" aria-label="清理所有项目进程" title="清理本项目残留的 ROS 进程" onClick={() => setModal('cleanup-processes')} disabled={actionBusy}><CircleStop size={17} /><span>清理所有进程</span></button>
          <IconButton title="刷新工作区" onClick={() => refresh()}><RefreshCw size={17} /></IconButton>
        </div>
      </header>

      {error && <div className="error-banner"><FileWarning size={17} />{error}<button onClick={() => refresh()}>重试</button></div>}

      <section className="runtime-bar">
        <div className={`runtime-state ${data?.runtime?.status || 'idle'}`}>
          <span className="runtime-pulse" />
          <div><strong>{data?.runtime?.status === 'running' ? `${data.runtime.mode === 'mapping' ? `${algorithmLabel(data.runtime.algorithm)} 正在建图` : data.runtime.mode === 'localization' ? `${localizationModuleLabel(data.runtime.algorithm)}运行中` : `${localizationModuleLabel(data.runtime.algorithm)}导航运行中`}` : data?.runtime?.status === 'stopping' ? '正在停止' : data?.runtime?.status === 'failed' ? '流程异常退出' : '系统待命'}</strong><span>{data?.runtime?.status === 'running' || data?.runtime?.status === 'failed' ? data?.runtime?.logs?.at(-1) || '正在启动' : '当前没有运行 ROS 流程'}</span></div>
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
            <div className="pipeline-title"><span>3</span><div><strong>定位 / 导航</strong><select className="mapping-algorithm-select localization-module-select" aria-label="定位模块" value={['localization', 'navigation'].includes(data?.runtime?.mode) && ['running', 'stopping'].includes(data?.runtime?.status) ? data.runtime.algorithm || localizationModule : localizationModule} onChange={(event) => setLocalizationModule(event.target.value)} disabled={actionBusy || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}>{(data?.runtime_modules?.localization || [{ id: 'fused_ekf', name: '融合定位' }, { id: 'pure_icp', name: '纯 ICP' }]).map((module) => <option key={module.id} value={module.id}>{module.name}{module.id === 'fused_ekf' ? '（推荐）' : '（原有）'}</option>)}</select></div></div>
            {['localization', 'navigation'].includes(data?.runtime?.mode) && ['running', 'stopping'].includes(data?.runtime?.status)
              ? <button className="stop-map" onClick={stopRuntime} disabled={actionBusy || data.runtime.status === 'stopping'}><Square size={16} />停止并清理</button>
              : <div className="map-run-actions"><button onClick={() => startRuntime('localization')} disabled={actionBusy || !data?.active?.complete || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><MapPin size={16} />{localizationModuleLabel(localizationModule)}</button><button onClick={() => startRuntime('navigation')} disabled={actionBusy || !data?.active?.complete || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><Navigation size={16} />导航</button></div>}
          </section>
        </div>
      </section>

      <div className="workspace">
        <aside className="sidebar">
          <div className="section-tabs">
            <button className={section === 'sessions' ? 'active' : ''} onClick={() => setSection('sessions')}><HardDrive size={16} />原始 PCD <span>{data?.sessions.length || 0}</span></button>
            <button className={section === 'versions' ? 'active' : ''} onClick={() => setSection('versions')}><Database size={16} />生成地图 <span>{data?.versions.length || 0}</span></button>
            <button className={section === 'waypoints' ? 'active' : ''} onClick={() => setSection('waypoints')}><Route size={16} />多点导航 <span>{data?.waypoints?.count || 0}</span></button>
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
              {data?.current_capture.available && <div className="current-capture"><div><Activity size={16} /><strong>当前原始 PCD</strong><span>{data.current_capture.archived ? '已归档' : data.current_capture.new_for_run ? '已落盘 · 待归档' : '历史残留'}</span></div><p>{algorithmLabel(data.current_capture.algorithm)} · {data.current_capture.file.size_human} · {formatTime(data.current_capture.file.modified_at)}</p><button className="button secondary wide" onClick={() => setModal('archive')} disabled={data.current_capture.archived || !data.current_capture.new_for_run}><Archive size={15} />{data.current_capture.archived ? 'PCD 已在列表中' : data.current_capture.new_for_run ? '归档到 PCD 列表' : '不能保存历史残留'}</button></div>}
              {data?.sessions.map((session) => (
                <div
                  className={`session-row ${selectedSessionId === session.id ? 'selected' : ''}`}
                  key={session.id}
                  role="button"
                  tabIndex={0}
                  onClick={() => setSelectedSessionId(session.id)}
                  onKeyDown={(event) => { if (event.key === 'Enter' || event.key === ' ') setSelectedSessionId(session.id) }}
                >
                  <div className={`session-icon ${session.origin === 'cluster-cleaned' ? 'cleaned' : ''}`}>{session.origin === 'cluster-cleaned' ? <Eraser size={18} /> : <HardDrive size={18} />}</div>
                  <div><strong>{session.name}</strong><span>{session.origin === 'cluster-cleaned' ? '清理副本' : '原始数据'} · {algorithmLabel(session.algorithm)} · {formatTime(session.created_at)}</span><code>{session.raw?.size_human || '文件缺失'}</code></div>
                  <div className="row-actions" onClick={(event) => event.stopPropagation()}><IconButton title="聚类清理人影" onClick={() => setModal({ type: 'cluster', sessionId: session.id })} disabled={!session.complete || actionBusy}><Eraser size={16} /></IconButton><IconButton title="从该 PCD 生成地图" onClick={() => setModal({ type: 'build', sessionId: session.id })}><Layers3 size={16} /></IconButton><IconButton title="移到回收站" onClick={() => moveToTrash('session', session.id)} disabled={actionBusy}><Trash2 size={16} /></IconButton></div>
                </div>
              ))}
              {!data?.sessions.length && <div className="list-empty"><HardDrive size={24} /><strong>还没有原始 PCD</strong><span>完成一次建图并保存后会显示在这里</span></div>}
            </div>
          )}

          {section === 'waypoints' && (
            <div className="waypoint-sidebar">
              <div className="waypoint-capture-card">
                <div><Crosshair size={17} /><span><strong>记录机器狗当前位置</strong><small>读取 map → base_link 实时位姿</small></span></div>
                <button className="button primary wide" onClick={captureWaypoint} disabled={actionBusy || !navigationRunning || mission.active}>{actionBusy ? <LoaderCircle className="spin" size={16} /> : <MapPin size={16} />}记录为新目标点</button>
                {!navigationRunning && <p><FileWarning size={14} />需要先启动导航并完成初始定位</p>}
              </div>
              <div className="waypoint-list-head"><strong>导航顺序</strong><span>{waypoints.length} 个目标点</span></div>
              <div className="item-list waypoint-list">
                {waypoints.map((waypoint, index) => {
                  const current = mission.active && mission.current_waypoint_id === waypoint.id
                  const done = mission.mission_id && index < (mission.processed || 0)
                  return (
                    <div key={waypoint.id} className={`waypoint-row ${selectedWaypointId === waypoint.id ? 'selected' : ''} ${current ? 'current' : ''} ${done ? 'done' : ''}`} role="button" tabIndex={0} onClick={() => setSelectedWaypointId(waypoint.id)} onKeyDown={(event) => { if (event.key === 'Enter' || event.key === ' ') setSelectedWaypointId(waypoint.id) }}>
                      <span className="waypoint-index">{done ? <Check size={14} /> : index + 1}</span>
                      <div className="waypoint-copy"><strong>{waypoint.name}</strong><span>x {waypoint.position.x.toFixed(2)} · y {waypoint.position.y.toFixed(2)}</span><small>朝向 {(waypoint.yaw * 180 / Math.PI).toFixed(1)}°</small></div>
                      <div className="waypoint-row-actions" onClick={(event) => event.stopPropagation()}>
                        <IconButton title="前移" onClick={() => reorderWaypoint(waypoint.id, -1)} disabled={actionBusy || mission.active || index === 0}><ArrowUp size={14} /></IconButton>
                        <IconButton title="后移" onClick={() => reorderWaypoint(waypoint.id, 1)} disabled={actionBusy || mission.active || index === waypoints.length - 1}><ArrowDown size={14} /></IconButton>
                        <IconButton title="删除" onClick={() => setModal({ type: 'delete-waypoint', waypoint })} disabled={actionBusy || mission.active}><Trash2 size={14} /></IconButton>
                      </div>
                    </div>
                  )
                })}
                {!waypoints.length && <div className="list-empty waypoint-empty"><Route size={25} /><strong>还没有目标点</strong><span>把机器狗移动到目标位置，然后点击上方“记录”</span></div>}
              </div>
            </div>
          )}

          {section === 'trash' && (
            <div className="item-list trash-list">
              {Boolean(data?.trash.length) && <div className="trash-bulkbar"><label><input type="checkbox" checked={allTrashSelected} onChange={toggleAllTrash} /><span>全选</span></label><strong>已选 {selectedTrashItems.length}</strong><button className="button ghost-danger" onClick={() => setModal({ type: 'purge-bulk', items: selectedTrashItems })} disabled={!selectedTrashItems.length || actionBusy}><Trash2 size={15} />彻底删除所选</button></div>}
              {data?.trash.map((item) => (
                <div className={`trash-row ${selectedTrashIds.includes(item.id) ? 'selected' : ''}`} key={item.id}><label className="trash-select" onClick={(event) => event.stopPropagation()}><input type="checkbox" checked={selectedTrashIds.includes(item.id)} onChange={() => toggleTrashSelection(item.id)} aria-label={`选择 ${item.name}`} /></label><div className="trash-icon"><Trash2 size={19} /></div><div className="trash-copy"><strong>{item.name}</strong><span>{item.kind === 'version' ? '生成地图' : '原始 PCD'} · {item.size_human}</span><code>{formatTime(item.deleted_at)}</code></div><div className="row-actions"><IconButton title="恢复" onClick={() => restoreTrash(item.id)} disabled={actionBusy}><Undo2 size={17} /></IconButton><IconButton title="彻底删除" onClick={() => setModal({ type: 'purge', item })} disabled={actionBusy}><X size={17} /></IconButton></div></div>
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
          {section === 'waypoints' ? (
            <div className="mission-workspace">
              <div className="mission-workspace-head">
                <div><span>多目标自主导航</span><h2>{data?.active?.name || '尚未激活地图'}</h2><p>任务启动时会锁定当前目标点顺序，依次发送给 Nav2。</p></div>
                <div className={`mission-status-pill status-${mission.status}`}>{mission.active && <span />} {missionStatusLabel(mission.status)}</div>
              </div>
              <MissionProgress mission={mission} />
              <section className="route-board">
                <div className="route-board-head"><div><ListChecks size={18} /><span><strong>任务路线</strong><small>从 1 开始依次执行</small></span></div><b>{waypoints.length} 站</b></div>
                {waypoints.length ? <div className="route-timeline">
                  {waypoints.map((waypoint, index) => {
                    const current = mission.active && mission.current_waypoint_id === waypoint.id
                    const processed = index < (mission.processed || 0)
                    const result = (mission.results || []).find((item) => item.waypoint_id === waypoint.id)
                    const failed = result && !result.succeeded
                    return (
                      <button key={waypoint.id} className={`${selectedWaypointId === waypoint.id ? 'selected' : ''} ${current ? 'current' : ''} ${processed ? 'processed' : ''} ${failed ? 'failed' : ''}`} onClick={() => setSelectedWaypointId(waypoint.id)}>
                        <div className="route-node">{failed ? <X size={16} /> : processed ? <Check size={16} /> : current ? <Navigation size={16} /> : <Flag size={16} />}</div>
                        <div><span>第 {index + 1} 站</span><strong>{waypoint.name}</strong><small>x {waypoint.position.x.toFixed(2)} m · y {waypoint.position.y.toFixed(2)} m · {(waypoint.yaw * 180 / Math.PI).toFixed(1)}°</small></div>
                        {current && <em>正在前往</em>}
                        {result && <em>{result.succeeded ? '已到达' : '失败'}</em>}
                      </button>
                    )
                  })}
                </div> : <div className="route-board-empty"><div><Route size={34} /></div><h3>建立第一条导航路线</h3><p>先启动导航并完成初始定位，再到不同位置记录目标点。每个目标点会保存位置和朝向。</p><button className="button primary" onClick={captureWaypoint} disabled={!navigationRunning || actionBusy}><Crosshair size={16} />记录当前位置</button></div>}
              </section>
              <div className="mission-safety-note"><FileWarning size={16} /><div><strong>实机运行提示</strong><span>开始前确认定位稳定、路线无障碍并保持急停可用。取消任务会立即向 Nav2 撤销当前目标。</span></div></div>
            </div>
          ) : (section === 'sessions' && selectedSession) ? (
            <>
              <div className="preview-toolbar raw-toolbar"><div><strong>{selectedSession.name}</strong><span>{algorithmLabel(selectedSession.algorithm)} / {selectedSession.origin === 'cluster-cleaned' ? '聚类清理副本' : '原始 PCD'} / {selectedSession.site} / {selectedSession.raw?.size_human}</span></div><span className={`badge raw-badge ${selectedSession.origin === 'cluster-cleaned' ? 'cleaned' : ''}`}>{selectedSession.origin === 'cluster-cleaned' ? '清理副本' : '原始数据'}</span></div>
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
          {section === 'waypoints' ? (
            <>
              <div className="inspector-head"><div><span>任务控制</span><strong>多目标导航</strong></div></div>
              <section className="mission-preflight">
                <h3>运行前检查</h3>
                <div className={data?.active?.complete ? 'ok' : ''}>{data?.active?.complete ? <Check size={15} /> : <X size={15} />}<span>已激活完整地图</span></div>
                <div className={navigationRunning ? 'ok' : ''}>{navigationRunning ? <Check size={15} /> : <X size={15} />}<span>导航流程运行中</span></div>
                <div className={waypoints.length ? 'ok' : ''}>{waypoints.length ? <Check size={15} /> : <X size={15} />}<span>已记录目标点</span><b>{waypoints.length}</b></div>
              </section>
              <section className="detail-section mission-settings">
                <h3>任务策略</h3>
                <label className="toggle-row"><span><strong>失败时停止</strong><small>某一站失败后不再前往后续目标</small></span><input type="checkbox" checked={missionConfig.stop_on_failure} onChange={(event) => setMissionConfig({ ...missionConfig, stop_on_failure: event.target.checked })} disabled={mission.active} /></label>
                <Field label="单个目标最长时间" hint="30–3600 秒，超时后取消当前目标"><div className="unit-input"><input type="number" min="30" max="3600" step="10" value={missionConfig.waypoint_timeout_sec} onChange={(event) => setMissionConfig({ ...missionConfig, waypoint_timeout_sec: Number(event.target.value) })} disabled={mission.active} /><span>秒</span></div></Field>
                <Field label="目标间停留时间" hint="到达一站后再前往下一站"><div className="unit-input"><input type="number" min="0" max="60" step="1" value={missionConfig.pause_between_sec} onChange={(event) => setMissionConfig({ ...missionConfig, pause_between_sec: Number(event.target.value) })} disabled={mission.active} /><span>秒</span></div></Field>
              </section>
              <div className="inspector-actions mission-controls">
                {mission.active ? <button className="button danger-button wide mission-cancel" onClick={cancelWaypointMission} disabled={actionBusy || mission.status === 'cancelling'}>{mission.status === 'cancelling' ? <LoaderCircle className="spin" size={17} /> : <CircleStop size={17} />}{mission.status === 'cancelling' ? '正在安全取消…' : '立即取消任务'}</button> : <button className="button primary wide mission-start" onClick={startWaypointMission} disabled={actionBusy || !navigationRunning || !data?.active?.complete || !waypoints.length}><Play size={17} />开始多点导航</button>}
                {!navigationRunning && <button className="button secondary wide" onClick={() => startRuntime('navigation')} disabled={actionBusy || !data?.active?.complete || ['running', 'stopping'].includes(data?.runtime?.status)}><Navigation size={16} />先启动导航</button>}
              </div>
              {selectedWaypoint && <section className="selected-waypoint-detail">
                <div><span>已选目标点</span><strong>{selectedWaypoint.name}</strong></div>
                <dl><div><dt>X</dt><dd>{selectedWaypoint.position.x.toFixed(3)} m</dd></div><div><dt>Y</dt><dd>{selectedWaypoint.position.y.toFixed(3)} m</dd></div><div><dt>朝向</dt><dd>{(selectedWaypoint.yaw * 180 / Math.PI).toFixed(1)}°</dd></div></dl>
                <div><button className="button secondary" onClick={() => setModal({ type: 'rename-waypoint', waypoint: selectedWaypoint })} disabled={mission.active || actionBusy}><Pencil size={15} />重命名</button><button className="button ghost-danger" onClick={() => setModal({ type: 'delete-waypoint', waypoint: selectedWaypoint })} disabled={mission.active || actionBusy}><Trash2 size={15} />删除</button></div>
              </section>}
            </>
          ) : section === 'sessions' && selectedSession ? (
            <>
              <div className="inspector-head"><div><span>{selectedSession.origin === 'cluster-cleaned' ? '聚类清理副本' : '原始 PCD'}</span><strong>{selectedSession.name}</strong></div></div>
              {!selectedSession.complete && <div className="issue-panel"><FileWarning size={17} /><div><strong>PCD 文件缺失</strong></div></div>}
              <section className="detail-section"><h3>建图记录</h3><dl><div><dt>记录 ID</dt><dd><code>{selectedSession.id}</code></dd></div><div><dt>数据类型</dt><dd>{selectedSession.origin === 'cluster-cleaned' ? '清理副本（源 PCD 保留）' : '原始数据'}</dd></div>{selectedSession.source_session && <div><dt>来源 PCD</dt><dd><code>{selectedSession.source_session}</code></dd></div>}<div><dt>建图算法</dt><dd>{algorithmLabel(selectedSession.algorithm)}</dd></div><div><dt>场地</dt><dd>{selectedSession.site}</dd></div><div><dt>时间</dt><dd>{formatTime(selectedSession.created_at)}</dd></div><div><dt>备注</dt><dd>{selectedSession.note || '无'}</dd></div></dl></section>
              {selectedSession.raw && <section className="detail-section"><h3>原始文件</h3><PathRow label="PCD" value={selectedSession.raw.path} notify={notify} /></section>}
              <div className="inspector-actions"><button className="button secondary wide" onClick={() => setModal({ type: 'rename', kind: 'session', item: selectedSession })} disabled={actionBusy}><Pencil size={16} />重命名 PCD</button><button className="button secondary wide" onClick={() => setModal({ type: 'cluster', sessionId: selectedSession.id })} disabled={!selectedSession.complete || actionBusy || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><Eraser size={16} />聚类清理人影</button><button className="button primary wide" onClick={() => setModal({ type: 'build', sessionId: selectedSession.id })} disabled={!selectedSession.complete || actionBusy || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><Layers3 size={16} />生成 2D + ICP 地图</button><button className="button ghost-danger wide" onClick={() => moveToTrash('session', selectedSession.id)} disabled={actionBusy}><Trash2 size={16} />移到回收站</button></div>
            </>
          ) : section === 'versions' && selected ? (
            <>
              <div className="inspector-head"><div><span>版本详情</span><strong>{selected.name}</strong></div></div>
              {!selected.complete && <div className="issue-panel"><FileWarning size={17} /><div><strong>地图包不完整</strong>{selected.issues.map((issue) => <span key={issue}>{issue}</span>)}</div></div>}
              <section className="detail-section"><h3>身份</h3><dl><div><dt>版本</dt><dd><code>{selected.id}</code></dd></div><div><dt>建图算法</dt><dd>{algorithmLabel(selected.mapping_algorithm)}</dd></div><div><dt>来源</dt><dd>{selected.origin}</dd></div><div><dt>建图记录</dt><dd>{selected.source_session || '历史导入'}</dd></div><div><dt>备注</dt><dd>{selected.note || '无'}</dd></div></dl></section>
              <section className="detail-section"><h3>生成文件</h3><PathRow label="2D地图" value={selected.paths.map_yaml} notify={notify} /><PathRow label="ICP PCD" value={selected.paths.localization_pcd} notify={notify} /></section>
              <div className="inspector-actions">
                <button className="button secondary wide" onClick={() => setModal({ type: 'edit-map', version: selected })} disabled={!selected.complete || actionBusy || data?.job?.running || ['running', 'stopping'].includes(data?.runtime?.status)}><Pencil size={16} />修整 2D 地图</button>
                <button className="button secondary wide" onClick={() => setModal({ type: 'rename', kind: 'version', item: selected })} disabled={actionBusy}><Pencil size={16} />重命名地图</button>
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
      {mission.active && <div className="mission-drawer"><MissionProgress mission={mission} compact /><button className="button danger-button" onClick={cancelWaypointMission} disabled={actionBusy || mission.status === 'cancelling'}>{mission.status === 'cancelling' ? <LoaderCircle className="spin" size={16} /> : <CircleStop size={16} />}{mission.status === 'cancelling' ? '取消中' : '取消任务'}</button></div>}
      {toast && <div className={`toast ${toast.type}`}>{toast.type === 'success' ? <Check size={16} /> : <FileWarning size={16} />}{toast.message}</div>}
      {modal === 'archive' && <ArchiveModal capture={data?.current_capture} onClose={() => setModal(null)} onDone={() => { setModal(null); setSection('sessions'); refresh(true) }} notify={notify} />}
      {modal === 'finish-mapping' && <ArchiveModal capture={data?.current_capture} stopFirst onClose={() => setModal(null)} onDiscard={discardMapping} onDone={() => { setModal(null); setSection('sessions'); refresh(true) }} notify={notify} />}
      {(modal === 'build' || modal?.type === 'build') && <BuildModal sessions={data?.sessions || []} profiles={data?.build_profiles || []} initialSession={modal?.sessionId} onClose={() => setModal(null)} onDone={() => { setModal(null); refresh(true) }} onProfilesChanged={() => refresh(true)} notify={notify} />}
      {modal?.type === 'rename' && <RenameModal kind={modal.kind} item={modal.item} onClose={() => setModal(null)} onDone={async () => { setModal(null); await refresh(true) }} notify={notify} />}
      {modal?.type === 'rename-waypoint' && <WaypointRenameModal waypoint={modal.waypoint} onClose={() => setModal(null)} onDone={async () => { setModal(null); await refresh(true) }} notify={notify} />}
      {modal?.type === 'delete-waypoint' && <ConfirmWaypointDelete waypoint={modal.waypoint} onClose={() => setModal(null)} onConfirm={() => deleteWaypoint(modal.waypoint)} busy={actionBusy} />}
      {modal?.type === 'cluster' && data?.sessions.find((session) => session.id === modal.sessionId) && <Suspense fallback={<div className="modal-backdrop"><div className="canvas-overlay"><LoaderCircle className="spin" size={20} />正在载入点云清理工具</div></div>}><ClusterEditor session={data.sessions.find((session) => session.id === modal.sessionId)} onClose={() => setModal(null)} onDone={async (sessionId) => { setModal(null); setSection('sessions'); setSelectedSessionId(sessionId); await refresh(true) }} notify={notify} /></Suspense>}
      {modal?.type === 'edit-map' && <Suspense fallback={<div className="modal-backdrop"><div className="canvas-overlay"><LoaderCircle className="spin" size={20} />正在载入 2D 地图编辑器</div></div>}><MapEditor version={modal.version} onClose={() => setModal(null)} onDone={async (versionId) => { setModal(null); setSection('versions'); setSelectedId(versionId); setView('2d'); await refresh(true) }} notify={notify} /></Suspense>}
      {modal?.type === 'purge' && <ConfirmPurge items={[modal.item]} onClose={() => setModal(null)} onConfirm={() => purgeTrash([modal.item])} busy={actionBusy} />}
      {modal?.type === 'purge-bulk' && <ConfirmPurge items={modal.items} onClose={() => setModal(null)} onConfirm={() => purgeTrash(modal.items)} busy={actionBusy} />}
      {modal === 'cleanup-processes' && <ConfirmProcessCleanup onClose={() => setModal(null)} onConfirm={cleanupProcesses} busy={actionBusy} />}
    </div>
  )
}
