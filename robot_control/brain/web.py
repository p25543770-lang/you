"""HTTP-интерфейс симуляции ROS + ИИ: страница /brain и JSON-состояние.

Все маршруты под авторизацией пульта (guard = login_required).
Симуляция стартует лениво — при первом запросе, а не при импорте приложения.
"""

from __future__ import annotations

from pathlib import Path

from flask import Blueprint, abort, jsonify, send_from_directory

from .runtime import BrainSim

STATIC_DIR = Path(__file__).resolve().parent.parent / "static"


def create_brain_blueprint(sim_holder: dict, guard=None) -> Blueprint:
    bp = Blueprint("brain", __name__)
    guard = guard or (lambda view: view)

    def get_sim() -> BrainSim:
        sim = sim_holder.get("sim")
        if sim is None:
            try:
                sim = BrainSim()
            except (OSError, ValueError, KeyError) as exc:
                abort(503, description=f"веса ИИ не загружены: {exc}")
            sim_holder["sim"] = sim
        sim.ensure_running()
        return sim

    @bp.get("/brain")
    @guard
    def page():
        get_sim()
        response = send_from_directory(STATIC_DIR, "brain.html")
        response.headers["Cache-Control"] = "no-store"
        return response

    @bp.get("/api/brain/state")
    @guard
    def state():
        return jsonify({"ok": True, "data": get_sim().snapshot()})

    @bp.post("/api/brain/reset")
    @guard
    def reset():
        get_sim().reset()
        return jsonify({"ok": True})

    return bp
