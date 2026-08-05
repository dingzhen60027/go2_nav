# Go2 Map Workspace

本地地图全生命周期工作台，统一管理建图、定位、导航、归档、地图生成、预览、激活、回退和回收站。

```bash
./start_map_manager.sh
```

浏览器访问 `http://127.0.0.1:8765`。

## 数据约定

```text
maps/
├── active -> workspace/versions/<version-id>
└── workspace/
    ├── sessions/<session-id>/
    │   ├── raw.pcd
    │   └── session.yaml
    ├── versions/<version-id>/
    │   ├── manifest.yaml
    │   ├── localization.pcd
    │   ├── map.pgm
    │   ├── map.yaml
    │   └── preview.ply
    ├── trash/<trash-id>/
    │   └── .trash.json
    ├── waypoint_navigation/
    │   ├── status.json
    │   └── missions/<mission-id>.json
    ├── waypoints.yaml
    └── state.json
```

- 导出只创建候选版本，不自动改变导航地图。
- 激活操作原子切换 `maps/active`。
- Web 的定位模块选择同时作用于“定位”和“导航”：纯 ICP 分别使用原有
  `start_localization.sh` / `start_navigation.sh`，融合定位分别使用
  `start_fused_localization.sh` / `start_fused_navigation.sh`。
- 纯 ICP 导航继续使用原有 `nav2_config/nav2_params.yaml`；融合导航使用独立的
  `nav2_config/nav2_fused_params.yaml`，消费 `/localization/odometry/local`，并遵循
  `map -> odom -> base_footprint -> base_link` 双 EKF 动态 TF。两套配置互不覆盖、也不互相启动对方节点。
- Web 启动的建图、定位和导航流程由后端统一监督；停止时按进程组清理，Web 服务退出时也会收尾。
- 页头“清理所有进程”用于应急收尾：取消导航任务后，分级结束本项目的 ROS2、建图、定位、Nav2、RViz、Livox 和遗留静态 TF 进程，但保留 Web 服务与无关程序。正在写入的建图结果可能不完整，因此操作前需要二次确认。
- 删除版本或会话只会移动到项目内的回收站，恢复后回到原 ID；彻底删除需要二次确认。
- 旧 `maps/map_*.{pgm,yaml}` 文件保持只读，不自动迁移或删除。

## 2D 地图修整

在“生成地图”中选择一套完整版本，点击右侧“修整 2D 地图”。编辑器支持：

- 画墙：自由笔刷或直线补齐没有闭合的墙体；
- 擦除杂点：把误障碍区域恢复为可通行栅格；
- 未知区域：把不确定位置标记为未知；
- 笔刷宽度、缩放、适应窗口、像素坐标、撤销、重做和恢复原图。

保存采用矢量笔画而不是浏览器截图。后端在原始 PGM 尺寸上重新执行笔画，继承原地图的
`resolution`、`origin` 和占据阈值，再创建新的候选版本。源版本不会被覆盖，新版本的
`localization.pcd` 与源版本内容保持一致；因此手工补画的 2D 墙只影响 Nav2 栅格地图，
不会伪造 ICP 三维点。检查无误后，仍需手动点击“设为定位 / 导航地图”才会投入使用。

## 多目标导航

“多点导航”是 Web 工具内的独立 Nav2 客户端，不修改导航、ICP 或融合定位源码。

1. 激活完整地图，在 Web 中启动“导航”。
2. 用初始位姿完成定位，并确认机器人位姿稳定。
3. 把机器人移动到目标位置，点击“记录为新目标点”；记录内容为当时的
   `map -> base_link` 位置和朝向。
4. 调整目标点名称与顺序，设置单点超时、站间停留和失败策略。
5. 开始任务。页面会显示总进度、当前目标、剩余距离、耗时和恢复次数；底部控制条在切换页面后仍然可见。
6. “取消任务”会先向 Nav2 撤销当前 action；若 ROS 通信异常，后台看门狗会再安全终止任务进程。

目标点按地图版本 ID 隔离。任务启动时会保存一份目标点快照，因此任务运行期间禁止修改目标点或切换地图。Web 异常重启时会检查并终止遗留任务，避免继续执行失去监管的导航。

## 前端开发

```bash
cd tools/map_manager/frontend
npm install
npm run dev
```

生产构建由 FastAPI 直接托管：

```bash
npm run build
```
