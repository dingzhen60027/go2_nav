import { lazy, Suspense, useMemo, useState } from 'react'
import {
  Check,
  Eraser,
  FileWarning,
  LoaderCircle,
  MousePointer2,
  RotateCcw,
  ShieldCheck,
  SlidersHorizontal,
  X,
} from 'lucide-react'

const PointCloudView = lazy(() => import('./PointCloudView.jsx'))

async function request(path, options) {
  const response = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  })
  const data = await response.json().catch(() => ({}))
  if (!response.ok) throw new Error(data.detail || `请求失败 (${response.status})`)
  return data
}

function NumberField({ label, value, step, min, max, onChange, hint }) {
  return (
    <label className="cluster-number-field">
      <span>{label}</span>
      <input
        type="number"
        value={value}
        step={step}
        min={min}
        max={max}
        onChange={(event) => onChange(Number(event.target.value))}
      />
      {hint && <small>{hint}</small>}
    </label>
  )
}

function formatMeters(value) {
  return Number.isFinite(value) ? `${value.toFixed(2)} m` : '--'
}

function clusterSize(cluster) {
  return cluster.bounds.max.map((value, index) => Math.max(0, value - cluster.bounds.min[index]))
}

export default function ClusterEditor({ session, onClose, onDone, notify }) {
  const [parameters, setParameters] = useState({
    voxel_leaf: 0.15,
    tolerance: 0.30,
    min_voxels: 5,
    z_min: 0.20,
    z_max: 2.20,
  })
  const [result, setResult] = useState(null)
  const [selected, setSelected] = useState(() => new Set())
  const [name, setName] = useState(`${session.name}（去除人影）`)
  const [note, setNote] = useState('手动聚类清理；原始 PCD 已保留')
  const [busy, setBusy] = useState('')
  const [error, setError] = useState('')

  const orderedClusters = useMemo(
    () => [...(result?.clusters || [])].sort((left, right) => right.point_count - left.point_count),
    [result],
  )
  const selectedIds = useMemo(() => [...selected].sort((left, right) => left - right), [selected])
  const selectedPoints = useMemo(
    () => (result?.clusters || []).reduce(
      (sum, cluster) => sum + (selected.has(cluster.id) ? cluster.point_count : 0),
      0,
    ),
    [result, selected],
  )

  const changeParameter = (key, value) => setParameters((current) => ({ ...current, [key]: value }))
  const toggleCluster = (clusterId) => {
    setSelected((current) => {
      const next = new Set(current)
      if (next.has(clusterId)) next.delete(clusterId)
      else next.add(clusterId)
      return next
    })
  }

  const runClustering = async () => {
    setBusy('cluster')
    setError('')
    try {
      const next = await request(`/api/sessions/${session.id}/clusters`, {
        method: 'POST',
        body: JSON.stringify(parameters),
      })
      setResult(next)
      setSelected(new Set())
      if (!next.cluster_count) setError('当前参数没有找到可选择的聚类，请调整高度或聚类距离后重试。')
    } catch (requestError) {
      setError(requestError.message)
    } finally {
      setBusy('')
    }
  }

  const createCopy = async () => {
    if (!selected.size) return
    setBusy('apply')
    setError('')
    try {
      const created = await request(`/api/sessions/${session.id}/clusters/apply`, {
        method: 'POST',
        body: JSON.stringify({
          cluster_run_id: result.id,
          cluster_ids: selectedIds,
          name,
          note,
        }),
      })
      notify(`已生成清理副本，移除约 ${selectedPoints.toLocaleString()} 个点；原始 PCD 未修改`)
      onDone(created.session_id)
    } catch (requestError) {
      setError(requestError.message)
    } finally {
      setBusy('')
    }
  }

  return (
    <div className="modal-backdrop cluster-backdrop">
      <div className="cluster-editor" role="dialog" aria-modal="true" aria-label="点云聚类清理">
        <header className="cluster-editor-head">
          <div>
            <span className="cluster-kicker"><Eraser size={15} />点云聚类清理</span>
            <h2>{session.name}</h2>
            <p>在原始 PCD 上计算聚类，手动选择要剔除的人影。</p>
          </div>
          <button className="icon-button" onClick={onClose} disabled={Boolean(busy)} title="关闭" aria-label="关闭"><X size={19} /></button>
        </header>

        <div className="cluster-safety-banner">
          <ShieldCheck size={19} />
          <div><strong>原始 PCD 只读，不会被覆盖</strong><span>确认后将创建一个新的“清理副本”，原始文件和原始会话都会保留。</span></div>
        </div>

        {!result ? (
          <div className="cluster-setup">
            <section className="cluster-setup-copy">
              <SlidersHorizontal size={28} />
              <h3>设置聚类范围</h3>
              <p>只对指定高度内的体素做欧式聚类。通常人影高度可先使用 0.20–2.20 m，完成后再从彩色预览中手动选择。</p>
              <div className="cluster-source-card"><span>读取源文件</span><strong>{session.raw?.size_human || 'PCD'}</strong><code>{session.raw?.path}</code></div>
            </section>
            <section className="cluster-parameter-panel">
              <div className="cluster-parameter-grid">
                <NumberField label="预览体素" value={parameters.voxel_leaf} step="0.01" min="0.05" max="0.50" hint="越小越细，计算量越大" onChange={(value) => changeParameter('voxel_leaf', value)} />
                <NumberField label="聚类距离" value={parameters.tolerance} step="0.05" min="0.05" max="2.0" hint="相邻点归为一类的距离" onChange={(value) => changeParameter('tolerance', value)} />
                <NumberField label="最少体素数" value={parameters.min_voxels} step="1" min="2" max="10000" hint="过滤过小噪点" onChange={(value) => changeParameter('min_voxels', value)} />
                <NumberField label="高度下限" value={parameters.z_min} step="0.10" min="-10" max="10" onChange={(value) => changeParameter('z_min', value)} />
                <NumberField label="高度上限" value={parameters.z_max} step="0.10" min="-10" max="10" onChange={(value) => changeParameter('z_max', value)} />
              </div>
              {error && <div className="cluster-error"><FileWarning size={17} />{error}</div>}
              <button className="button primary wide" onClick={runClustering} disabled={Boolean(busy)}>
                {busy === 'cluster' ? <LoaderCircle className="spin" size={17} /> : <MousePointer2 size={17} />}
                {busy === 'cluster' ? '正在聚类原始 PCD…' : '开始聚类并生成彩色预览'}
              </button>
            </section>
          </div>
        ) : (
          <div className="cluster-workspace">
            <section className="cluster-viewer">
              <Suspense fallback={<div className="canvas-overlay"><LoaderCircle className="spin" size={20} />正在载入聚类点云</div>}>
                <PointCloudView
                  url={result.preview_url}
                  colorMode="cluster"
                  selectedClusterIds={selectedIds}
                  onClusterClick={toggleCluster}
                />
              </Suspense>
            </section>

            <aside className="cluster-selection-panel">
              <div className="cluster-selection-head">
                <div><strong>手动选择要删除的聚类</strong><span>点预览或下方列表；亮红色表示待删除</span></div>
                <button className="button secondary compact" onClick={() => { setResult(null); setSelected(new Set()); setError('') }} disabled={Boolean(busy)}><RotateCcw size={15} />重新聚类</button>
              </div>

              <div className="cluster-list" aria-label="聚类列表">
                {orderedClusters.map((cluster) => {
                  const size = clusterSize(cluster)
                  const active = selected.has(cluster.id)
                  return (
                    <button key={cluster.id} className={`cluster-item ${active ? 'selected' : ''}`} onClick={() => toggleCluster(cluster.id)}>
                      <span className="cluster-swatch" style={{ backgroundColor: active ? '#f04438' : cluster.color_hex }} />
                      <span className="cluster-item-copy">
                        <span><strong>聚类 #{cluster.id}</strong><b>{cluster.point_count.toLocaleString()} 点</b></span>
                        <small>{size.map(formatMeters).join(' × ')} · 中心 Z {formatMeters(cluster.centroid[2])}</small>
                      </span>
                      <span className="cluster-check">{active && <Check size={15} />}</span>
                    </button>
                  )
                })}
              </div>

              <div className="cluster-output-form">
                <div className="cluster-selection-summary"><strong>{selected.size}</strong><span>个聚类待删除</span><b>约 {selectedPoints.toLocaleString()} 点</b></div>
                <label><span>清理副本名称</span><input value={name} maxLength={80} onChange={(event) => setName(event.target.value)} /></label>
                <label><span>备注</span><textarea rows="2" value={note} maxLength={500} onChange={(event) => setNote(event.target.value)} /></label>
                {error && <div className="cluster-error"><FileWarning size={17} />{error}</div>}
                <button className="button primary wide" onClick={createCopy} disabled={Boolean(busy) || !selected.size || !name.trim()}>
                  {busy === 'apply' ? <LoaderCircle className="spin" size={17} /> : <ShieldCheck size={17} />}
                  {busy === 'apply' ? '正在生成清理副本…' : '生成清理副本（保留原始 PCD）'}
                </button>
              </div>
            </aside>
          </div>
        )}
      </div>
    </div>
  )
}
