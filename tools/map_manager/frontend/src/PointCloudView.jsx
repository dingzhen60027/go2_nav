import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js'
import { Box, LoaderCircle, RotateCcw } from 'lucide-react'

function colorizeByHeight(geometry) {
  const position = geometry.getAttribute('position')
  if (!position?.count) return

  let min = Infinity
  let max = -Infinity
  for (let index = 0; index < position.count; index += 1) {
    const z = position.getZ(index)
    min = Math.min(min, z)
    max = Math.max(max, z)
  }

  const range = Math.max(max - min, 0.001)
  const low = new THREE.Color('#45c7d8')
  const middle = new THREE.Color('#dce8ff')
  const high = new THREE.Color('#f6b84b')
  const colors = new Float32Array(position.count * 3)

  for (let index = 0; index < position.count; index += 1) {
    const ratio = (position.getZ(index) - min) / range
    const color = ratio < 0.55
      ? low.clone().lerp(middle, ratio / 0.55)
      : middle.clone().lerp(high, (ratio - 0.55) / 0.45)
    colors[index * 3] = color.r
    colors[index * 3 + 1] = color.g
    colors[index * 3 + 2] = color.b
  }
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3))
}

export default function PointCloudView({ url }) {
  const hostRef = useRef(null)
  const fitRef = useRef(() => {})
  const [state, setState] = useState('loading')
  const [pointCount, setPointCount] = useState(0)

  useEffect(() => {
    const host = hostRef.current
    if (!host || !url) return undefined

    setState('loading')
    const scene = new THREE.Scene()
    scene.background = new THREE.Color('#101622')
    scene.fog = new THREE.FogExp2('#101622', 0.012)

    const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 5000)
    camera.up.set(0, 0, 1)

    const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' })
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.outputColorSpace = THREE.SRGBColorSpace
    host.appendChild(renderer.domElement)

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.08
    controls.screenSpacePanning = true

    const grid = new THREE.GridHelper(120, 120, '#3b4a66', '#222c3e')
    grid.rotation.x = Math.PI / 2
    grid.position.z = -0.02
    scene.add(grid)

    let cloud = null
    let animationFrame = 0

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
        geometry.computeBoundingBox()
        geometry.computeBoundingSphere()
        colorizeByHeight(geometry)
        const radius = Math.max(geometry.boundingSphere?.radius || 10, 1)
        const material = new THREE.PointsMaterial({
          size: Math.max(radius / 700, 0.018),
          sizeAttenuation: true,
          vertexColors: true,
          transparent: true,
          opacity: 0.92,
        })
        cloud = new THREE.Points(geometry, material)
        scene.add(cloud)
        setPointCount(geometry.getAttribute('position')?.count || 0)
        setState('ready')
        fit()
      },
      undefined,
      () => setState('error'),
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
      cancelAnimationFrame(animationFrame)
      observer.disconnect()
      controls.dispose()
      if (cloud) {
        cloud.geometry.dispose()
        cloud.material.dispose()
      }
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
        <div className="canvas-meta">{pointCount.toLocaleString()} points</div>
      )}
      <button className="canvas-action" onClick={() => fitRef.current()} title="复位 3D 视角" aria-label="复位 3D 视角">
        <RotateCcw size={17} />
      </button>
    </div>
  )
}
