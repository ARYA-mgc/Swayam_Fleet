"""
Swayam Dashboard — Flask web UI for fleet monitoring
=====================================================
Run: python swayam_dashboard.py
Open: http://localhost:5050
"""

import sys
import os
import json
import time
import sqlite3
import threading
from datetime import datetime
from flask import Flask, jsonify, render_template_string, request

# Allow importing from src/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))
from swayam_core import SwayamFleet, FlightDatabase

app = Flask(__name__)

# Global fleet (simulation mode by default)
fleet = SwayamFleet(db_path="swayam_flights.db")
fleet.add_obstacle(10, 10, radius=2)
fleet.add_obstacle(20, 15, radius=3)
fleet.add_obstacle(30, 25, radius=2)
fleet.add_drone("ALPHA", system_id=1, simulation=True)
fleet.add_drone("BETA",  system_id=2, simulation=True)
fleet.add_drone("GAMMA", system_id=3, simulation=True)
fleet.connect_all()


# ── HTML Template ──────────────────────────────

DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Swayam Fleet Dashboard</title>
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap" rel="stylesheet">
<style>
  :root {
    --bg: #090e1a;
    --surface: #0d1526;
    --border: #1a2a45;
    --accent: #00d4ff;
    --accent2: #00ff99;
    --warn: #ffb800;
    --danger: #ff3b5c;
    --text: #cce0ff;
    --muted: #4a6080;
    --font-mono: 'Share Tech Mono', monospace;
    --font-ui: 'Rajdhani', sans-serif;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--font-ui);
    min-height: 100vh;
    font-size: 15px;
  }

  /* Noise overlay */
  body::before {
    content: '';
    position: fixed; inset: 0;
    background-image: url("data:image/svg+xml,%3Csvg viewBox='0 0 200 200' xmlns='http://www.w3.org/2000/svg'%3E%3Cfilter id='n'%3E%3CfeTurbulence type='fractalNoise' baseFrequency='0.75' numOctaves='4' stitchTiles='stitch'/%3E%3C/filter%3E%3Crect width='100%25' height='100%25' filter='url(%23n)' opacity='0.03'/%3E%3C/svg%3E");
    pointer-events: none; z-index: 0; opacity: 0.4;
  }

  header {
    background: linear-gradient(90deg, #0a1428 0%, #0d1e3a 100%);
    border-bottom: 1px solid var(--border);
    padding: 0 2rem;
    height: 60px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    position: sticky; top: 0; z-index: 100;
  }
  .logo {
    font-family: var(--font-mono);
    font-size: 1.3rem;
    color: var(--accent);
    letter-spacing: 0.15em;
  }
  .logo span { color: var(--accent2); }
  .header-right {
    display: flex; align-items: center; gap: 1.5rem;
    font-family: var(--font-mono); font-size: 0.8rem; color: var(--muted);
  }
  .live-dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: var(--accent2);
    box-shadow: 0 0 8px var(--accent2);
    animation: pulse 1.5s ease-in-out infinite;
  }
  @keyframes pulse { 0%,100% { opacity: 1; } 50% { opacity: 0.3; } }

  main { padding: 1.5rem 2rem; position: relative; z-index: 1; }

  /* Drone cards */
  .cards { display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 1rem; margin-bottom: 1.5rem; }
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 1.2rem;
    position: relative;
    overflow: hidden;
    transition: border-color 0.3s;
  }
  .card::before {
    content: '';
    position: absolute; top: 0; left: 0; right: 0; height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    opacity: 0.6;
  }
  .card.armed::before { background: linear-gradient(90deg, transparent, var(--accent2), transparent); }
  .card:hover { border-color: var(--accent); }

  .card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 1rem; }
  .drone-id { font-family: var(--font-mono); font-size: 1.1rem; color: var(--accent); font-weight: 700; }
  .badge {
    padding: 2px 10px; border-radius: 20px; font-size: 0.75rem;
    font-family: var(--font-mono); letter-spacing: 0.05em;
  }
  .badge.armed { background: rgba(0,255,153,0.15); color: var(--accent2); border: 1px solid var(--accent2); }
  .badge.disarmed { background: rgba(255,59,92,0.1); color: var(--danger); border: 1px solid var(--danger); }
  .badge.active { background: rgba(0,212,255,0.1); color: var(--accent); border: 1px solid var(--accent); }

  .stat-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 0.5rem; }
  .stat { background: rgba(0,0,0,0.3); border-radius: 4px; padding: 0.5rem 0.75rem; }
  .stat-label { font-size: 0.7rem; color: var(--muted); text-transform: uppercase; letter-spacing: 0.08em; }
  .stat-value { font-family: var(--font-mono); font-size: 1rem; color: var(--text); margin-top: 2px; }
  .stat-value.accent { color: var(--accent); }
  .stat-value.green  { color: var(--accent2); }
  .stat-value.warn   { color: var(--warn); }

  .battery-bar {
    margin-top: 1rem;
    height: 6px; background: rgba(255,255,255,0.05); border-radius: 3px; overflow: hidden;
  }
  .battery-fill {
    height: 100%; border-radius: 3px;
    background: linear-gradient(90deg, var(--accent2), var(--accent));
    transition: width 0.5s ease;
  }

  /* Map canvas */
  .map-section { margin-bottom: 1.5rem; }
  .section-title {
    font-family: var(--font-mono); font-size: 0.85rem; color: var(--muted);
    text-transform: uppercase; letter-spacing: 0.12em; margin-bottom: 0.75rem;
    display: flex; align-items: center; gap: 0.5rem;
  }
  .section-title::after { content: ''; flex: 1; height: 1px; background: var(--border); }

  .map-wrap {
    background: var(--surface); border: 1px solid var(--border); border-radius: 8px;
    padding: 1rem; display: flex; justify-content: center;
  }
  canvas { display: block; border-radius: 4px; }

  /* Log table */
  .log-wrap { background: var(--surface); border: 1px solid var(--border); border-radius: 8px; overflow: hidden; }
  .log-toolbar {
    padding: 0.75rem 1rem; border-bottom: 1px solid var(--border);
    display: flex; gap: 0.5rem; flex-wrap: wrap; align-items: center;
  }
  .filter-btn {
    background: transparent; border: 1px solid var(--border); color: var(--muted);
    padding: 3px 12px; border-radius: 4px; font-family: var(--font-mono); font-size: 0.75rem;
    cursor: pointer; transition: all 0.2s;
  }
  .filter-btn:hover, .filter-btn.active {
    border-color: var(--accent); color: var(--accent); background: rgba(0,212,255,0.05);
  }
  table { width: 100%; border-collapse: collapse; font-family: var(--font-mono); font-size: 0.8rem; }
  th { padding: 0.5rem 1rem; text-align: left; color: var(--muted); font-size: 0.7rem; text-transform: uppercase; letter-spacing: 0.1em; border-bottom: 1px solid var(--border); }
  td { padding: 0.5rem 1rem; border-bottom: 1px solid rgba(26,42,69,0.5); color: var(--text); }
  tr:hover td { background: rgba(0,212,255,0.03); }
  .level-INFO  { color: var(--accent); }
  .level-WARN  { color: var(--warn); }
  .level-ERROR { color: var(--danger); }
  .ts { color: var(--muted); font-size: 0.75rem; }

  /* Actions */
  .actions { display: flex; gap: 0.75rem; margin-bottom: 1.5rem; flex-wrap: wrap; }
  .btn {
    padding: 0.5rem 1.2rem; border-radius: 4px; font-family: var(--font-mono); font-size: 0.8rem;
    cursor: pointer; border: 1px solid; transition: all 0.2s; letter-spacing: 0.05em;
  }
  .btn-primary { background: rgba(0,212,255,0.1); border-color: var(--accent); color: var(--accent); }
  .btn-danger  { background: rgba(255,59,92,0.1);  border-color: var(--danger);  color: var(--danger); }
  .btn:hover { filter: brightness(1.3); transform: translateY(-1px); }
</style>
</head>
<body>

<header>
  <div class="logo">SWAYAM <span>//</span> FLEET OPS</div>
  <div class="header-right">
    <span id="clock">--:--:--</span>
    <div class="live-dot"></div>
    <span>LIVE</span>
  </div>
</header>

<main>
  <div class="actions">
    <button class="btn btn-primary" onclick="launchMission()">▶ BROADCAST MISSION</button>
    <button class="btn btn-danger"  onclick="emergencyLand()">⚠ EMERGENCY LAND ALL</button>
  </div>

  <div class="section-title">FLEET STATUS</div>
  <div class="cards" id="cards"></div>

  <div class="map-section">
    <div class="section-title">INS POSITION MAP — 50×50m GRID</div>
    <div class="map-wrap">
      <canvas id="mapCanvas" width="500" height="500"></canvas>
    </div>
  </div>

  <div class="section-title">FLIGHT LOGS</div>
  <div class="log-wrap">
    <div class="log-toolbar">
      <span style="color:var(--muted);font-family:var(--font-mono);font-size:0.75rem;">FILTER:</span>
      <button class="filter-btn active" onclick="setFilter('ALL',this)">ALL</button>
      <button class="filter-btn" onclick="setFilter('ALPHA',this)">ALPHA</button>
      <button class="filter-btn" onclick="setFilter('BETA',this)">BETA</button>
      <button class="filter-btn" onclick="setFilter('GAMMA',this)">GAMMA</button>
    </div>
    <table>
      <thead><tr>
        <th>TIME</th><th>DRONE</th><th>LEVEL</th><th>EVENT</th><th>DETAILS</th>
      </tr></thead>
      <tbody id="logTable"></tbody>
    </table>
  </div>
</main>

<script>
let logFilter = 'ALL';
const DRONE_COLORS = { ALPHA: '#00d4ff', BETA: '#00ff99', GAMMA: '#ffb800' };

function setFilter(f, btn) {
  logFilter = f;
  document.querySelectorAll('.filter-btn').forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
}

function fmtTime(ts) {
  return new Date(ts * 1000).toLocaleTimeString();
}

function renderCards(drones) {
  const el = document.getElementById('cards');
  el.innerHTML = drones.map(d => {
    const ins = d.ins;
    const bat = d.battery_pct;
    const batColor = bat > 50 ? 'green' : bat > 20 ? 'warn' : 'level-ERROR';
    return `
    <div class="card ${d.armed ? 'armed' : ''}">
      <div class="card-header">
        <span class="drone-id">${d.drone_id}</span>
        <div style="display:flex;gap:0.4rem">
          <span class="badge ${d.armed ? 'armed' : 'disarmed'}">${d.armed ? 'ARMED' : 'DISARMED'}</span>
          ${d.mission_active ? '<span class="badge active">ACTIVE</span>' : ''}
        </div>
      </div>
      <div class="stat-grid">
        <div class="stat">
          <div class="stat-label">Mode</div>
          <div class="stat-value accent">${d.mode}</div>
        </div>
        <div class="stat">
          <div class="stat-label">Battery</div>
          <div class="stat-value ${batColor}">${bat.toFixed(1)}%</div>
        </div>
        <div class="stat">
          <div class="stat-label">Position N/E</div>
          <div class="stat-value">${ins.position_N.toFixed(2)} / ${ins.position_E.toFixed(2)}</div>
        </div>
        <div class="stat">
          <div class="stat-label">Altitude</div>
          <div class="stat-value accent">${(-ins.position_D).toFixed(2)}m</div>
        </div>
        <div class="stat">
          <div class="stat-label">Vel N/E</div>
          <div class="stat-value">${ins.velocity_N.toFixed(3)} / ${ins.velocity_E.toFixed(3)}</div>
        </div>
        <div class="stat">
          <div class="stat-label">Yaw</div>
          <div class="stat-value">${(ins.yaw_rad * 180 / Math.PI).toFixed(1)}°</div>
        </div>
      </div>
      <div class="battery-bar">
        <div class="battery-fill" style="width:${bat}%"></div>
      </div>
    </div>`;
  }).join('');
}

function renderMap(drones) {
  const canvas = document.getElementById('mapCanvas');
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const SCALE = W / 50;

  ctx.clearRect(0, 0, W, H);

  // Background
  ctx.fillStyle = '#080d1a';
  ctx.fillRect(0, 0, W, H);

  // Grid
  ctx.strokeStyle = 'rgba(26,42,69,0.6)';
  ctx.lineWidth = 0.5;
  for (let i = 0; i <= 50; i++) {
    ctx.beginPath(); ctx.moveTo(i * SCALE, 0); ctx.lineTo(i * SCALE, H); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, i * SCALE); ctx.lineTo(W, i * SCALE); ctx.stroke();
  }

  // Origin cross
  const ox = 25 * SCALE, oy = 25 * SCALE;
  ctx.strokeStyle = 'rgba(0,212,255,0.2)';
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(ox, 0); ctx.lineTo(ox, H); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, oy); ctx.lineTo(W, oy); ctx.stroke();
  ctx.setLineDash([]);

  // Drones
  drones.forEach(d => {
    const x = (d.ins.position_E + 25) * SCALE;
    const y = (25 - d.ins.position_N) * SCALE;
    const color = DRONE_COLORS[d.drone_id] || '#ffffff';

    // Glow
    const grd = ctx.createRadialGradient(x, y, 0, x, y, 20);
    grd.addColorStop(0, color + '55');
    grd.addColorStop(1, 'transparent');
    ctx.fillStyle = grd;
    ctx.beginPath(); ctx.arc(x, y, 20, 0, Math.PI * 2); ctx.fill();

    // Dot
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.fill();

    // Label
    ctx.fillStyle = color;
    ctx.font = 'bold 11px "Share Tech Mono"';
    ctx.fillText(d.drone_id, x + 8, y - 8);
  });
}

function renderLogs(logs) {
  const tbody = document.getElementById('logTable');
  const filtered = logFilter === 'ALL' ? logs : logs.filter(l => l.drone_id === logFilter);
  tbody.innerHTML = filtered.slice(0, 50).map(l => `
    <tr>
      <td class="ts">${fmtTime(l.timestamp)}</td>
      <td style="color:${DRONE_COLORS[l.drone_id]||'#ccc'}">${l.drone_id}</td>
      <td class="level-${l.level}">${l.level}</td>
      <td>${l.event}</td>
      <td style="color:var(--muted)">${l.details || ''}</td>
    </tr>`).join('');
}

async function refresh() {
  try {
    const [statusRes, logRes] = await Promise.all([
      fetch('/api/status'), fetch('/api/logs')
    ]);
    const drones = await statusRes.json();
    const logs   = await logRes.json();
    renderCards(drones);
    renderMap(drones);
    renderLogs(logs);
  } catch (e) { console.error('Refresh error:', e); }
}

async function emergencyLand() {
  await fetch('/api/emergency_land', { method: 'POST' });
  refresh();
}

async function launchMission() {
  const gn = prompt('Goal North (meters):', '15');
  const ge = prompt('Goal East (meters):', '12');
  if (gn === null || ge === null) return;
  fetch('/api/mission', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ goal_n: parseFloat(gn), goal_e: parseFloat(ge) })
  });
  setTimeout(refresh, 500);
}

// Clock
setInterval(() => {
  document.getElementById('clock').textContent = new Date().toLocaleTimeString();
}, 1000);

// Auto-refresh
refresh();
setInterval(refresh, 4000);
</script>
</body>
</html>"""


# ── API Routes ─────────────────────────────────

@app.route("/")
def index():
    return render_template_string(DASHBOARD_HTML)

@app.route("/api/status")
def api_status():
    return jsonify(fleet.fleet_status())

@app.route("/api/logs")
def api_logs():
    drone_id = request.args.get("drone_id")
    limit = int(request.args.get("limit", 100))
    return jsonify(fleet.db.get_recent_logs(drone_id, limit))

@app.route("/api/ins/<drone_id>")
def api_ins(drone_id):
    return jsonify(fleet.db.get_ins_history(drone_id))

@app.route("/api/emergency_land", methods=["POST"])
def api_emergency_land():
    fleet.emergency_land_all()
    return jsonify({"status": "emergency_land_sent"})

@app.route("/api/mission", methods=["POST"])
def api_mission():
    data = request.json
    gn = float(data.get("goal_n", 10))
    ge = float(data.get("goal_e", 10))
    alt = float(data.get("altitude", 10))
    t = threading.Thread(target=fleet.broadcast_destination, args=(gn, ge, alt), daemon=True)
    t.start()
    return jsonify({"status": "mission_dispatched", "goal_n": gn, "goal_e": ge})

@app.route("/api/export")
def api_export():
    path = "logs/swayam_export.json"
    fleet.db.export_json(path)
    return jsonify({"status": "exported", "path": path})


if __name__ == "__main__":
    print("=" * 55)
    print("  Swayam Fleet Dashboard")
    print("  http://localhost:5050")
    print("=" * 55)
    app.run(host="0.0.0.0", port=5050, debug=False)
