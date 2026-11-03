# Autonomous Vehicle Digital Twin — Unstructured *Kutcha* Roads

An interactive 3D Autonomous Vehicle Digital Twin built with **Three.js** for simulating perception, virtual lane synthesis, pothole avoidance, and lateral/longitudinal control on unmarked Indian *kutcha* (earthen/gravel) roads without lane markings or structured curbs.

---

## Key Capabilities

1. **Procedural Unstructured *Kutcha* Road & Terrain Engine**
   - Multi-octave noise-eroded left and right boundaries (`wLeft(s)`, `wRight(s)`) with unmarked shoulders and single-track pinch bottlenecks.
   - Real-time 3D vertex-carved pothole craters with configurable diameter, depth, and spatial density.
2. **Real-Time 3D Perception & Virtual Lane Synthesis**
   - **Road Width Scanlines**: Live 3D width measurement bars at `+6m`, `+12m`, `+18m`, `+24m`, and `+30m` ahead of the ego vehicle.
   - **Virtual Lane Synthesizer**: Computes a continuous target lane in unmarked terrain, supporting both **Keep-Left (Indian LHD convention)** and **Center Spine** modes.
   - **3D Pothole Detection Boxes**: Highlights potholes inside the `32m` forward perception horizon with 3D wireframe bounding boxes and Bird's-Eye View (BEV) radar projection.
3. **Frenet Local Path Planner & Stanley Lateral Controller**
   - Dynamically deflects the nominal virtual lane around potholes when lateral road width allows safe clearance, or commands a crawl speed reduction when the road narrows.
   - Closed-loop **Stanley Steering Law** minimizing cross-track error $e(t)$ and heading error $\psi(t)$:
     $$\delta(t) = \psi(t) + \arctan\left(\frac{k \cdot e(t)}{v(t)}\right)$$

---

## Project Structure

```text
├── index.html       # Main application entry point & HUD layout
├── style.css        # Telemetry HUD & responsive panel styling
├── app.js           # 3D terrain engine, perception stack, Frenet planner & Stanley controller
├── package.json     # Optional local dev server configuration
├── .gitignore       # Standard Git ignore rules
├── LICENSE          # MIT License
└── README.md        # Project documentation
```

---

## Quick Start & GitHub Pages Deployment

### 1. Run Locally
Because the simulation uses zero external build steps, you can open `index.html` directly in any modern browser, or serve it locally:

```bash
# Using Python
python -m http.server 8080

# Or using Node.js
npx serve .
```

Then open `http://localhost:8080` in your browser.

### 2. Push to GitHub & Enable GitHub Pages
```bash
git init
git add .
git commit -m "Initial commit: 3D Autonomous Vehicle Digital Twin for Kutcha Roads"
git branch -M main
git remote add origin https://github.com/<your-username>/<your-repo-name>.git
git push -u origin main
```
To host live on **GitHub Pages**, go to your repository **Settings → Pages**, select **Deploy from a branch (`main` / `root`)**, and save.
