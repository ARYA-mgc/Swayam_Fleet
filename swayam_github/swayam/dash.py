\
\
\
\
\
   
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
from core import SwayamFleet, FlightDatabase

app = Flask(__name__)

# UI looks like it's from 1995 but it works
# websockets kept dropping so we are just long-polling /api/status now. deal with it.
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

DASHBOARD_HTML =\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
          
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
