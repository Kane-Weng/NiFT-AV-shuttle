/*
 * File: App.tsx
 * Author: Kane Weng
 * Date: April 2026
 *
 * Description:
 *   Root component of the NIFT web dashboard. Responsible for two things:
 *
 *   1. ROS2 Connection — opens a WebSocket to rosbridge_websocket (default
 *      ws://localhost:9090) using the roslib library. Listens for connection,
 *      error, and close events and auto-reconnects every 3 seconds if the
 *      bridge goes down. The live ROSLIB.Ros instance is shared app-wide via
 *      RosContext so any child component can subscribe or publish without
 *      needing to open its own connection.
 *
 *   2. Layout — defines the full-screen shell:
 *        - Header bar: NIFT logo + live connection indicator (yellow/green/red)
 *        - Left area (flex-1): MapView — the main 2D map
 *        - Right sidebar (fixed 288 px): TelemetryPanel, NodeHealthChecker,
 *          ControlPanel
 *
 *   goalMode is lifted here because both MapView (shows the crosshair cursor
 *   and handles the click) and ControlPanel (the toggle button) need to read
 *   and change it.
 *
 * Usage:
 *   Rendered once by main.tsx. To reach rosbridge on another machine, set
 *   VITE_ROS_URL=ws://<host>:9090 when starting Vite (e.g. in .env.local).
 */

import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { RosContext } from "./hooks/useRos";
import MapView from "./components/MapView";
import TelemetryPanel from "./components/TelemetryPanel";
import ObstacleRadar from "./components/ObstacleRadar";
import NodeHealthChecker from "./components/NodeHealthChecker";
import ControlPanel from "./components/ControlPanel";

// rosbridge URL: VITE_ROS_URL (e.g. ws://<host>:9090) or this machine by default
const ROS_URL = import.meta.env.VITE_ROS_URL || "ws://localhost:9090";

type ConnState = "connecting" | "connected" | "disconnected";

export default function App() {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [connState, setConnState] = useState<ConnState>("connecting");
  const [goalMode, setGoalMode] = useState(false);
  const rosRef = useRef<ROSLIB.Ros | null>(null);

  useEffect(() => {
    function connect() {
      const instance = new ROSLIB.Ros({ url: ROS_URL });
      rosRef.current = instance;

      instance.on("connection", () => {
        setRos(instance);
        setConnState("connected");
      });

      instance.on("error", () => {
        setConnState("disconnected");
      });

      instance.on("close", () => {
        setRos(null);
        setConnState("disconnected");
        // Auto-reconnect after 3 s
        setTimeout(connect, 3000);
      });
    }

    connect();

    return () => {
      rosRef.current?.close();
    };
  }, []);

  return (
    <RosContext.Provider value={ros}>
      <div className="flex flex-col h-screen bg-gray-950 text-white">
        {/* ── Header ── */}
        <header className="flex items-center justify-between px-4 py-2 bg-gray-900 border-b border-gray-800 shrink-0">
          <div className="flex items-center gap-3">
            <span className="font-bold tracking-widest text-sm">NIFT</span>
            <span className="text-gray-500 text-xs">Web Dashboard</span>
          </div>
          <ConnIndicator state={connState} />
        </header>

        {/* ── Body ── */}
        <div className="flex flex-1 overflow-hidden">
          {/* Map — takes remaining width */}
          <div className="flex-1 relative">
            <MapView goalMode={goalMode} onGoalModeChange={setGoalMode} />
          </div>

          {/* Right sidebar */}
          <aside className="w-72 flex flex-col gap-3 p-3 bg-gray-900 border-l border-gray-800 overflow-y-auto shrink-0 [&>*]:shrink-0">
            <NodeHealthChecker />
            <div className="border-t border-gray-800" />
            <TelemetryPanel />
            <div className="border-t border-gray-800" />
            <ObstacleRadar />
            <div className="border-t border-gray-800" />
            <ControlPanel goalMode={goalMode} onGoalModeChange={setGoalMode} />
          </aside>
        </div>
      </div>
    </RosContext.Provider>
  );
}

function ConnIndicator({ state }: { state: ConnState }) {
  const colors: Record<ConnState, string> = {
    connecting: "text-yellow-400",
    connected: "text-green-400",
    disconnected: "text-red-400",
  };
  const dots: Record<ConnState, string> = {
    connecting: "bg-yellow-400 animate-pulse",
    connected: "bg-green-400",
    disconnected: "bg-red-400",
  };
  const labels: Record<ConnState, string> = {
    connecting: "Connecting…",
    connected: "Connected",
    disconnected: "Disconnected",
  };
  return (
    <div className={`flex items-center gap-2 text-xs ${colors[state]}`}>
      <div className={`w-2 h-2 rounded-full ${dots[state]}`} />
      {labels[state]}
    </div>
  );
}
