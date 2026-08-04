import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js'
import { Box, LoaderCircle, RotateCcw } from 'lucide-react'

const HEIGHT_COLORS = [
  '#243b93',
  '#1769c2',
  '#18a9df',
  '#35cfad',
  '#8bd646',
  '#e7d43b',
  '#f49a2f',
  '#e94b35',
  '#a91532',
].map((value) => new THREE.Color(value))

function percentile(sortedValues, ratio) {
  const index = (sortedValues.length - 1) * ratio
  const lower = Math.floor(index)
  const upper = Math.ceil(index)
  if (lower === upper) return sortedValues[lower]
  return sortedValues[lower] + (sortedValues[upper] - sortedValues[lower]) * (index - lower)
}

function colorizeByHeight(geometry) {
  const position = geometry.getAttribute('position')
  if (!position?.count) return null

  const heights = new Float32Array(position.count)
  for (let index = 0; index < position.count; index += 1) {
    heights[index] = position.getZ(index)
  }
  heights.sort()

  const actualMin = heights[0]
  const actualMax = heights[heights.length - 1]
  const low = percentile(heights, 0.02)
  const high = percentile(heights, 0.98)
  const range = high - low
  const colors = new Float32Array(position.count * 3)
  const sampledColor = new THREE.Color()

  for (let index = 0; index < position.count; index += 1) {
    const ratio = range > 1e-6
      ? THREE.MathUtils.clamp((position.getZ(index) - low) / range, 0, 1)
      : 0.5
    const scaled = ratio * (HEIGHT_COLORS.length - 1)
    const lower = Math.min(Math.floor(scaled), HEIGHT_COLORS.length - 2)
    sampledColor.lerpColors(HEIGHT_COLORS[lower], HEIGHT_COLORS[lower + 1], scaled - lower)
    colors[index * 3] = sampledColor.r
    colors[index * 3 + 1] = sampledColor.g
    colors[index * 3 + 2] = sampledColor.b
  }
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3))
  return { low, high, actualMin, actualMax }
}

function createPointTexture() {
  const canvas = document.createElement('canvas')
  canvas.width = 32
  canvas.height = 32
  const context = canvas.getContext('2d')
  context.fillStyle = '#ffffff'
  context.beginPath()
  context.arc(16, 16, 14, 0, Math.PI * 2)
  context.fill()
  const texture = new THREE.CanvasTexture(canvas)
  texture.colorSpace = THREE.SRGBColorSpace
  return texture
}

function formatHeight(value) {
  return Number.isFinite(value) ? value.toFixed(2) : '--'
}

export default function PointCloudView({ url }) {
  const hostRef = useRef(null)
  const fitRef = useRef(() => {})
  const [state, setState] = useState('loading')
  const [pointCount, setPointCount] = useState(0)
  const [heightRange, setHeightRange] = useState(null)

  useEffect(() => {
    const host = hostRef.current
    if (!host || !url) return undefined

    setState('loading')
    setPointCount(0)
    setHeightRange(null)
    const scene = new THREE.Scene()
    scene.background = new THREE.Color('#0d141f')

    const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 5000)
    camera.up.set(0, 0, 1)

    const renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: 'high-performance',
      preserveDrawingBuffer: true,
    })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.outputColorSpace = THREE.SRGBColorSpace
    host.appendChild(renderer.domElement)

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.08
    controls.screenSpacePanning = true

    let cloud = null
    let grid = null
    let animationFrame = 0
    let disposed = false
    const pointTexture = createPointTexture()

    const fit = () => {
      if (!cloud) return
      const box = new THREE.Box3().setFromObject(cloud)
      const sphere = box.getBoundingSphere(new THREE.Sphere())
      const radius = Math.max(sphere.radius, 1)
      camera.position.copy(sphere.center).add(new THREE.Vector3(radius * 0.85, -radius * 1.2, radius * 1.05))
      camera.near = Math.max(radius / 1000, 0.01)
      camera.far = radius * 30
      camera.updateProjectionMatrix()
      controls.target.copy(sphere.center)
      controls.update()
    }
    fitRef.current = fit

    const loader = new PLYLoader()
    loader.load(
      url,
      (geometry) => {
        if (disposed) {
          geometry.dispose()
          return
        }
        geometry.computeBoundingBox()
        geometry.computeBoundingSphere()
        const range = colorizeByHeight(geometry)
        const radius = Math.max(geometry.boundingSphere?.radius || 10, 1)
        const bounds = geometry.boundingBox
        const boundsSize = bounds.getSize(new THREE.Vector3())
        const boundsCenter = bounds.getCenter(new THREE.Vector3())
        const gridSize = Math.max(Math.ceil(Math.max(boundsSize.x, boundsSize.y) / 10) * 10, 10)
        const gridDivisions = THREE.MathUtils.clamp(Math.round(gridSize / 2), 10, 80)
        grid = new THREE.GridHelper(gridSize, gridDivisions, '#54647c', '#273246')
        grid.rotation.x = Math.PI / 2
        grid.position.set(
          boundsCenter.x,
          boundsCenter.y,
          bounds.min.z - Math.max(boundsSize.z * 0.01, 0.02),
        )
        grid.material.transparent = true
        grid.material.opacity = 0.56
        scene.add(grid)
        scene.fog = new THREE.Fog('#0d141f', radius * 2.2, radius * 8)

        const material = new THREE.PointsMaterial({
          size: THREE.MathUtils.clamp(radius / 280, 0.035, 0.28),
          sizeAttenuation: true,
          vertexColors: true,
          map: pointTexture,
          alphaTest: 0.35,
          transparent: true,
          opacity: 0.96,
          depthWrite: false,
        })
        cloud = new THREE.Points(geometry, material)
        scene.add(cloud)
        setPointCount(geometry.getAttribute('position')?.count || 0)
        setHeightRange(range)
        setState('ready')
        fit()
      },
      undefined,
      () => {
        if (!disposed) setState('error')
      },
    )

    const resize = () => {
      const width = Math.max(host.clientWidth, 1)
      const height = Math.max(host.clientHeight, 1)
      renderer.setSize(width, height, false)
      camera.aspect = width / height
      camera.updateProjectionMatrix()
    }
    const observer = new ResizeObserver(resize)
    observer.observe(host)
    resize()

    const animate = () => {
      controls.update()
      renderer.render(scene, camera)
      animationFrame = requestAnimationFrame(animate)
    }
    animate()

    return () => {
      disposed = true
      cancelAnimationFrame(animationFrame)
      observer.disconnect()
      controls.dispose()
      if (cloud) {
        cloud.geometry.dispose()
        cloud.material.dispose()
      }
      if (grid) {
        grid.geometry.dispose()
        grid.material.dispose()
      }
      pointTexture.dispose()
      renderer.dispose()
      renderer.domElement.remove()
    }
  }, [url])

  if (!url) {
    return (
      <div className="empty-state">
        <Box size={28} />
        <strong>没有 3D 预览</strong>
        <span>该地图尚未纳入版本工作区</span>
      </div>
    )
  }

  return (
    <div className="cloud-host" ref={hostRef} data-cloud-state={state}>
      {state === 'loading' && (
        <div className="canvas-overlay"><LoaderCircle className="spin" size={20} /> 正在载入点云</div>
      )}
      {state === 'error' && (
        <div className="canvas-overlay error">点云预览加载失败</div>
      )}
      {state === 'ready' && (
        <>
          <div className="canvas-meta"><strong>{pointCount.toLocaleString()}</strong><span>points</span></div>
          {heightRange && (
            <div className="height-legend" aria-label="点云 Z 高度色带">
              <div className="height-legend-head"><strong>Z 高度</strong><span>m</span></div>
              <div className="height-legend-body">
                <div className="height-gradient" />
                <div className="height-labels">
                  <span>{formatHeight(heightRange.high)}</span>
                  <span>{formatHeight((heightRange.high + heightRange.low) / 2)}</span>
                  <span>{formatHeight(heightRange.low)}</span>
                </div>
              </div>
            </div>
          )}
        </>
      )}
      <button className="canvas-action" onClick={() => fitRef.current()} title="复位 3D 视角" aria-label="复位 3D 视角">
        <RotateCcw size={17} />
      </button>
    </div>
  )
}
