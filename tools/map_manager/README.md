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
    └── state.json
```

- 导出只创建候选版本，不自动改变导航地图。
- 激活操作原子切换 `maps/active`。
- 定位模块可选择原有 `start_localization.sh`（纯 ICP）或
  `start_fused_localization.sh`（运动预测 + ICP 修正）；两者只读取完整的 active 地图包。
- `start_navigation.sh` 独立管理现有导航流程。
- Web 启动的建图、定位和导航流程由后端统一监督；停止时按进程组清理，Web 服务退出时也会收尾。
- 删除版本或会话只会移动到项目内的回收站，恢复后回到原 ID；彻底删除需要二次确认。
- 旧 `maps/map_*.{pgm,yaml}` 文件保持只读，不自动迁移或删除。

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
