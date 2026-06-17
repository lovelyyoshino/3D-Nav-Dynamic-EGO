import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { nav3dBridgeSupervisorPlugin } from "./src/bridgeSupervisor";

export default defineConfig({
  plugins: [react(), nav3dBridgeSupervisorPlugin()],
});
