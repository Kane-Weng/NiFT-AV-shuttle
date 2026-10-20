/// <reference types="vite/client" />

interface ImportMetaEnv {
  // rosbridge WebSocket URL, e.g. ws://<host>:9090 (default ws://localhost:9090)
  readonly VITE_ROS_URL?: string
}

interface ImportMeta {
  readonly env: ImportMetaEnv
}
