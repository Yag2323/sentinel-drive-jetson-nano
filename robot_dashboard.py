#!/usr/bin/env python3

"""Read-only monitoring and CSI camera dashboard for the Jetson Nano robot car.

This process never imports the motor controller, never initializes the
PCA9685, and exposes no HTTP command endpoint. It reads INA219 bus voltage,
Linux system telemetry, and an optional read-only CSI camera stream.

Compatible with Python 3.6.
"""

from __future__ import print_function

import argparse
import copy
import datetime
import glob
import ipaddress
import json
import math
import os
import shutil
import signal
import socket
import socketserver
import sys
import threading
import time

try:
    from http.server import BaseHTTPRequestHandler, HTTPServer
except ImportError:  # pragma: no cover - retained for very old Python only
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

try:
    import smbus
except ImportError:
    smbus = None


APP_NAME = "Sentinel Drive"
SCHEMA_VERSION = 2

DEFAULT_I2C_BUS = 1
DEFAULT_INA219_ADDRESS = 0x41
INA219_BUS_VOLTAGE_REGISTER = 0x02

AUTONOMOUS_VOLTAGE = 7.00
CRITICAL_VOLTAGE = 6.60
DISCONNECTED_VOLTAGE = 1.00
DISPLAY_MAX_VOLTAGE = 8.40

DEFAULT_SAMPLE_INTERVAL = 0.50
HISTORY_LIMIT = 600
STALE_AFTER_SECONDS = 2.00

DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 360
DEFAULT_CAMERA_SOURCE_FPS = 30
DEFAULT_CAMERA_STREAM_FPS = 12
DEFAULT_CAMERA_JPEG_QUALITY = 82
CAMERA_STALE_AFTER_SECONDS = 2.50
CAMERA_STREAM_BOUNDARY = "sentinel-frame"

CONTROLLER_WORDS = (
    "xbox",
    "x-box",
    "gamepad",
    "wireless controller",
)


DASHBOARD_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="theme-color" content="#07111f">
  <title>Sentinel Drive | Robot Dashboard</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #07111f;
      --panel: rgba(15, 28, 47, 0.86);
      --panel-strong: #10223a;
      --line: rgba(148, 180, 214, 0.16);
      --text: #eef7ff;
      --muted: #8fa9c2;
      --blue: #38bdf8;
      --green: #34d399;
      --amber: #fbbf24;
      --red: #fb7185;
      --slate: #64748b;
      --shadow: 0 22px 70px rgba(0, 0, 0, 0.30);
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      min-height: 100vh;
      color: var(--text);
      background:
        radial-gradient(circle at 10% 0%, rgba(56, 189, 248, 0.12), transparent 34rem),
        radial-gradient(circle at 100% 30%, rgba(52, 211, 153, 0.07), transparent 32rem),
        var(--bg);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont,
        "Segoe UI", sans-serif;
      letter-spacing: 0.01em;
    }

    body::before {
      content: "";
      position: fixed;
      inset: 0;
      pointer-events: none;
      opacity: 0.32;
      background-image:
        linear-gradient(rgba(255,255,255,0.018) 1px, transparent 1px),
        linear-gradient(90deg, rgba(255,255,255,0.018) 1px, transparent 1px);
      background-size: 32px 32px;
      mask-image: linear-gradient(to bottom, black, transparent 85%);
    }

    .shell { width: min(1440px, calc(100% - 32px)); margin: 0 auto; padding: 26px 0 40px; }

    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 20px;
      margin-bottom: 22px;
    }

    .brand { display: flex; align-items: center; gap: 13px; }
    .brand-mark {
      display: grid;
      place-items: center;
      width: 45px;
      height: 45px;
      border: 1px solid rgba(56, 189, 248, 0.34);
      border-radius: 14px;
      color: var(--blue);
      background: linear-gradient(145deg, rgba(56,189,248,.16), rgba(56,189,248,.03));
      box-shadow: inset 0 0 20px rgba(56,189,248,.07);
      font: 800 17px/1 ui-monospace, SFMono-Regular, Menlo, monospace;
    }
    .eyebrow {
      color: var(--blue);
      font-size: 11px;
      font-weight: 800;
      letter-spacing: .18em;
      text-transform: uppercase;
    }
    h1 { margin: 3px 0 0; font-size: clamp(19px, 2vw, 27px); letter-spacing: -.02em; }

    .top-status { display: flex; align-items: center; gap: 14px; }
    .clock { color: var(--muted); font: 600 12px/1.4 ui-monospace, SFMono-Regular, Menlo, monospace; text-align: right; }
    .live-pill, .chip {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 8px 11px;
      background: rgba(11, 23, 39, 0.78);
      font-size: 11px;
      font-weight: 800;
      letter-spacing: .08em;
      text-transform: uppercase;
    }
    .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--green); box-shadow: 0 0 14px var(--green); }
    .offline .dot { background: var(--red); box-shadow: 0 0 14px var(--red); }

    .grid { display: grid; grid-template-columns: repeat(12, 1fr); gap: 16px; }
    .panel {
      position: relative;
      overflow: hidden;
      border: 1px solid var(--line);
      border-radius: 20px;
      background: var(--panel);
      box-shadow: var(--shadow);
      backdrop-filter: blur(14px);
    }
    .panel::after {
      content: "";
      position: absolute;
      inset: 0;
      pointer-events: none;
      border-radius: inherit;
      box-shadow: inset 0 1px rgba(255,255,255,.025);
    }

    .hero { grid-column: span 8; min-height: 308px; padding: 28px; display: flex; flex-direction: column; justify-content: space-between; }
    .battery { grid-column: span 4; min-height: 308px; padding: 25px; }
    .hero-glow {
      position: absolute;
      width: 390px;
      height: 390px;
      right: -180px;
      top: -220px;
      border-radius: 50%;
      background: rgba(251, 191, 36, .10);
      filter: blur(2px);
    }
    .section-label { color: var(--muted); font-size: 11px; font-weight: 800; letter-spacing: .15em; text-transform: uppercase; }
    .state-title { margin: 13px 0 8px; font-size: clamp(31px, 4.6vw, 58px); line-height: .98; letter-spacing: -.055em; max-width: 800px; }
    .state-copy { max-width: 730px; margin: 0; color: #b7c9da; font-size: 14px; line-height: 1.65; }
    .chips { display: flex; flex-wrap: wrap; gap: 9px; margin-top: 24px; }
    .chip { color: var(--muted); padding: 7px 10px; }
    .chip.locked { color: var(--amber); border-color: rgba(251,191,36,.24); background: rgba(251,191,36,.07); }
    .chip.bad { color: var(--red); border-color: rgba(251,113,133,.24); background: rgba(251,113,133,.07); }
    .chip.good { color: var(--green); border-color: rgba(52,211,153,.24); background: rgba(52,211,153,.07); }

    .battery-top { display: flex; justify-content: space-between; align-items: flex-start; gap: 12px; }
    .power-zone { color: var(--muted); font-size: 11px; font-weight: 800; letter-spacing: .09em; text-transform: uppercase; text-align: right; }
    .gauge-wrap { display: grid; place-items: center; margin: 14px 0 8px; }
    .gauge {
      --level: 0deg;
      --gauge-color: var(--slate);
      width: 164px;
      height: 164px;
      border-radius: 50%;
      display: grid;
      place-items: center;
      background: conic-gradient(var(--gauge-color) var(--level), rgba(148,180,214,.10) 0);
      transition: background .4s ease;
      box-shadow: 0 0 38px color-mix(in srgb, var(--gauge-color) 15%, transparent);
    }
    .gauge::before {
      content: "";
      width: 132px;
      height: 132px;
      border-radius: 50%;
      background: #0d1c30;
      border: 1px solid var(--line);
      position: absolute;
    }
    .gauge-value { position: relative; text-align: center; }
    .voltage { font-size: 38px; font-weight: 780; letter-spacing: -.05em; }
    .unit { margin-left: 2px; color: var(--muted); font-size: 14px; font-weight: 800; }
    .window-label { color: var(--muted); font-size: 10px; font-weight: 800; letter-spacing: .08em; text-transform: uppercase; }
    .battery-foot { display: flex; justify-content: space-between; align-items: end; gap: 12px; margin-top: 14px; }
    .small-value { margin-top: 5px; font-size: 13px; font-weight: 750; }

    .metric { grid-column: span 3; min-height: 126px; padding: 19px 20px; }
    .metric-head { display: flex; justify-content: space-between; align-items: center; }
    .metric-icon { color: var(--blue); font: 800 11px/1 ui-monospace, monospace; }
    .metric-value { margin-top: 16px; font-size: 27px; font-weight: 780; letter-spacing: -.035em; }
    .metric-sub { margin-top: 4px; color: var(--muted); font-size: 11px; }
    .mini-bar { height: 4px; margin-top: 13px; overflow: hidden; border-radius: 99px; background: rgba(148,180,214,.10); }
    .mini-bar > span { display: block; width: 0%; height: 100%; border-radius: inherit; background: linear-gradient(90deg, var(--blue), var(--green)); transition: width .4s ease; }

    .readiness { grid-column: span 5; min-height: 356px; padding: 24px; }
    .controller { grid-column: span 3; min-height: 356px; padding: 24px; }
    .events { grid-column: span 4; min-height: 356px; padding: 24px; }
    .trend { grid-column: span 12; min-height: 310px; padding: 24px; }
    .panel-title-row { display: flex; justify-content: space-between; align-items: center; gap: 12px; margin-bottom: 17px; }
    .panel-title { margin: 0; font-size: 16px; letter-spacing: -.01em; }
    .count { color: var(--muted); font: 700 11px/1 ui-monospace, monospace; }

    .readiness-list { display: grid; gap: 9px; }
    .ready-row {
      display: grid;
      grid-template-columns: 10px 1fr auto;
      gap: 11px;
      align-items: center;
      padding: 12px 13px;
      border: 1px solid rgba(148,180,214,.11);
      border-radius: 12px;
      background: rgba(5, 15, 27, .34);
    }
    .status-light { width: 8px; height: 8px; border-radius: 50%; background: var(--slate); }
    .status-light.good { background: var(--green); box-shadow: 0 0 12px rgba(52,211,153,.45); }
    .status-light.warn { background: var(--amber); box-shadow: 0 0 12px rgba(251,191,36,.38); }
    .status-light.bad { background: var(--red); box-shadow: 0 0 12px rgba(251,113,133,.38); }
    .ready-name { font-size: 12px; font-weight: 700; }
    .ready-status { color: var(--muted); font-size: 10px; font-weight: 800; letter-spacing: .07em; text-transform: uppercase; text-align: right; }

    .controller-orb {
      width: 112px;
      height: 112px;
      margin: 28px auto 20px;
      display: grid;
      place-items: center;
      border-radius: 50%;
      border: 1px solid rgba(251,113,133,.28);
      color: var(--red);
      background: radial-gradient(circle, rgba(251,113,133,.14), rgba(251,113,133,.02) 65%);
      font: 800 28px/1 ui-monospace, monospace;
    }
    .controller-orb.connected { color: var(--green); border-color: rgba(52,211,153,.28); background: radial-gradient(circle, rgba(52,211,153,.14), rgba(52,211,153,.02) 65%); }
    .center { text-align: center; }
    .controller-name { font-size: 14px; font-weight: 780; }
    .controller-path { min-height: 16px; margin-top: 6px; color: var(--muted); font: 600 10px/1.5 ui-monospace, monospace; word-break: break-all; }
    .notice { margin-top: 22px; padding: 11px 12px; border-radius: 11px; color: #a9bdd0; background: rgba(56,189,248,.055); border: 1px solid rgba(56,189,248,.14); font-size: 10px; line-height: 1.5; }

    .event-list { display: grid; gap: 0; max-height: 276px; overflow: auto; padding-right: 4px; }
    .event { display: grid; grid-template-columns: 7px 1fr; gap: 10px; padding: 10px 0; border-bottom: 1px solid rgba(148,180,214,.09); }
    .event:last-child { border-bottom: 0; }
    .event-mark { width: 6px; height: 6px; margin-top: 5px; border-radius: 50%; background: var(--blue); }
    .event.warn .event-mark { background: var(--amber); }
    .event.bad .event-mark { background: var(--red); }
    .event.good .event-mark { background: var(--green); }
    .event-message { font-size: 11px; line-height: 1.45; }
    .event-time { margin-top: 4px; color: var(--muted); font: 600 9px/1 ui-monospace, monospace; }

    .chart-wrap { position: relative; height: 225px; }
    canvas { display: block; width: 100%; height: 100%; }
    .legend { display: flex; flex-wrap: wrap; gap: 14px; color: var(--muted); font-size: 10px; }
    .legend span { display: inline-flex; align-items: center; gap: 6px; }
    .legend i { width: 18px; height: 2px; background: var(--blue); }
    .legend .amber { background: var(--amber); }
    .legend .red { background: var(--red); }

    footer { display: flex; justify-content: space-between; gap: 15px; margin-top: 16px; color: #6f8aa4; font-size: 10px; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }

    @media (max-width: 1020px) {
      .hero, .battery { grid-column: span 12; }
      .metric { grid-column: span 6; }
      .readiness { grid-column: span 7; }
      .controller { grid-column: span 5; }
      .events { grid-column: span 12; }
    }
    @media (max-width: 680px) {
      .shell { width: min(100% - 20px, 1440px); padding-top: 16px; }
      header { align-items: flex-start; }
      .clock { display: none; }
      .hero, .battery, .metric, .readiness, .controller, .events, .trend { grid-column: span 12; }
      .hero { min-height: 330px; padding: 22px; }
      .battery, .readiness, .controller, .events, .trend { padding: 20px; }
      .state-title { font-size: 38px; }
      footer { flex-direction: column; }
    }

    /* Sentinel command-console visual refresh. */
    :root {
      --bg: #050a11;
      --panel: rgba(11, 21, 33, .96);
      --panel-strong: #0e1b2a;
      --line: rgba(137, 174, 207, .14);
      --line-strong: rgba(137, 174, 207, .22);
      --text: #f4f8fc;
      --muted: #8193a7;
      --blue: #38c7f4;
      --green: #36d6a3;
      --amber: #ffb547;
      --red: #ff647c;
      --slate: #64748b;
      --hero-accent: #ffb547;
      --hero-glow: rgba(255, 181, 71, .12);
      --power-accent: #64748b;
      --power-glow: rgba(100, 116, 139, .10);
      --shadow: 0 18px 50px rgba(0, 0, 0, .22);
    }

    :root[data-zone="AUTONOMOUS_CAPABLE"] { --power-accent: #36d6a3; --power-glow: rgba(54, 214, 163, .12); }
    :root[data-zone="MANUAL_RECOVERY"] { --power-accent: #ffb547; --power-glow: rgba(255, 181, 71, .12); }
    :root[data-zone="CRITICAL_STOP"],
    :root[data-zone="BATTERY_ABSENT_OR_CRITICAL"],
    :root[data-zone="SENSOR_FAULT"] { --power-accent: #ff647c; --power-glow: rgba(255, 100, 124, .13); }
    :root[data-safety="CRITICAL_STOP_REQUIRED"],
    :root[data-safety="TELEMETRY_FAULT_HOLD"],
    :root[data-zone="BATTERY_ABSENT_OR_CRITICAL"] { --hero-accent: #ff647c; --hero-glow: rgba(255, 100, 124, .13); }

    html { background: var(--bg); }
    body {
      overflow-x: hidden;
      background:
        radial-gradient(circle at 14% -8%, rgba(56, 199, 244, .115), transparent 31rem),
        radial-gradient(circle at 94% 22%, rgba(54, 214, 163, .065), transparent 32rem),
        linear-gradient(145deg, #050a11 0%, #07111b 48%, #050a11 100%);
      font-variant-numeric: tabular-nums;
    }
    body::before {
      opacity: .24;
      background-size: 36px 36px;
      -webkit-mask-image: linear-gradient(to bottom, black, transparent 88%);
      mask-image: linear-gradient(to bottom, black, transparent 88%);
    }
    body::after {
      content: "";
      position: fixed;
      width: 520px;
      height: 520px;
      right: -320px;
      bottom: -310px;
      pointer-events: none;
      border: 1px solid rgba(56, 199, 244, .08);
      border-radius: 50%;
      box-shadow: 0 0 0 70px rgba(56, 199, 244, .018), 0 0 0 140px rgba(56, 199, 244, .012);
    }

    .shell {
      width: calc(100% - 40px);
      width: min(1510px, calc(100% - 40px));
      padding: 25px 0 38px;
    }

    header {
      min-height: 62px;
      margin-bottom: 20px;
      padding: 0 2px 18px;
      border-bottom: 1px solid rgba(137, 174, 207, .09);
    }
    .brand { gap: 14px; }
    .brand-mark {
      position: relative;
      width: 48px;
      height: 48px;
      border-color: rgba(56, 199, 244, .32);
      border-radius: 13px;
      color: var(--blue);
      background: linear-gradient(145deg, rgba(56,199,244,.14), rgba(56,199,244,.025));
      box-shadow: inset 0 0 24px rgba(56,199,244,.055), 0 12px 30px rgba(0,0,0,.18);
      font-size: 15px;
      letter-spacing: .05em;
    }
    .brand-mark::after {
      content: "";
      position: absolute;
      right: 6px;
      top: 6px;
      width: 4px;
      height: 4px;
      border-radius: 50%;
      background: var(--green);
      box-shadow: 0 0 9px var(--green);
    }
    .eyebrow { color: var(--blue); font-size: 10px; letter-spacing: .22em; }
    h1 { margin-top: 2px; font-size: 25px; font-weight: 750; letter-spacing: -.035em; }
    .brand-subline {
      display: flex;
      align-items: center;
      margin-top: 5px;
      color: #61758b;
      font: 700 8px/1 ui-monospace, SFMono-Regular, Menlo, monospace;
      letter-spacing: .13em;
    }
    .brand-subline span { margin-right: 7px; }
    .brand-subline i { width: 3px; height: 3px; margin-right: 7px; border-radius: 50%; background: #3e5469; }

    .top-status { gap: 12px; }
    .mode-pill {
      display: inline-flex;
      align-items: center;
      padding: 6px 9px;
      border: 1px solid rgba(56, 199, 244, .13);
      border-radius: 9px;
      color: #8da1b6;
      background: rgba(6, 16, 26, .68);
      font: 700 9px/1 ui-monospace, SFMono-Regular, Menlo, monospace;
      letter-spacing: .08em;
      text-transform: uppercase;
    }
    .mode-pill span { margin-right: 7px; color: var(--blue); }
    .clock { color: #72869b; font-size: 10px; line-height: 1.5; }
    .live-pill {
      min-width: 102px;
      justify-content: center;
      padding: 8px 11px;
      border-color: rgba(54, 214, 163, .2);
      background: rgba(9, 23, 31, .82);
      font-size: 10px;
    }
    .dot { width: 7px; height: 7px; }

    .grid { gap: 17px; }
    .panel {
      isolation: isolate;
      border-color: var(--line);
      border-radius: 16px;
      background:
        linear-gradient(145deg, rgba(255,255,255,.018), transparent 42%),
        var(--panel);
      box-shadow: var(--shadow);
      backdrop-filter: none;
    }
    .panel::after {
      z-index: -1;
      background: linear-gradient(90deg, transparent, rgba(166, 210, 245, .06), transparent) top / 72% 1px no-repeat;
      box-shadow: inset 0 1px rgba(255,255,255,.018);
    }

    .hero {
      min-height: 314px;
      padding: 30px 31px 27px 34px;
      border-left: 3px solid var(--hero-accent);
      background:
        linear-gradient(110deg, var(--hero-glow), transparent 30%),
        linear-gradient(145deg, rgba(255,255,255,.018), transparent 42%),
        var(--panel);
    }
    .hero-copy, .hero-bottom { position: relative; z-index: 2; }
    .hero-glow {
      width: 380px;
      height: 380px;
      right: -145px;
      top: -160px;
      border: 1px solid var(--hero-glow);
      background: repeating-radial-gradient(circle, transparent 0 43px, var(--hero-glow) 44px 45px);
      box-shadow: 0 0 90px var(--hero-glow);
      opacity: .78;
      filter: none;
    }
    .hero-glow::before,
    .hero-glow::after {
      content: "";
      position: absolute;
      left: 50%;
      top: 50%;
      background: var(--hero-glow);
      transform: translate(-50%, -50%);
    }
    .hero-glow::before { width: 100%; height: 1px; }
    .hero-glow::after { width: 1px; height: 100%; }
    .section-label {
      color: #7890a7;
      font-size: 9px;
      font-weight: 800;
      letter-spacing: .18em;
    }
    .section-index { margin-right: 8px; color: var(--blue); font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    .state-title {
      max-width: 790px;
      margin: 15px 0 11px;
      font-size: 52px;
      font-size: clamp(36px, 4vw, 52px);
      font-weight: 760;
      line-height: .98;
      letter-spacing: -.055em;
      text-shadow: 0 10px 35px rgba(0,0,0,.28);
    }
    .state-copy { max-width: 750px; color: #aebfd0; font-size: 13px; line-height: 1.65; }
    .hero-bottom { margin-top: 25px; }
    .chips { margin-top: 0; gap: 8px; }
    .chip {
      padding: 7px 10px;
      border-radius: 8px;
      color: #9baec0;
      background: rgba(5, 14, 23, .58);
      font-size: 9px;
      letter-spacing: .1em;
    }
    .chip i { width: 5px; height: 5px; border-radius: 50%; background: currentColor; box-shadow: 0 0 8px currentColor; }
    .chip.locked { color: var(--amber); border-color: rgba(255,181,71,.2); background: rgba(255,181,71,.055); }
    .blockers {
      display: flex;
      flex-wrap: wrap;
      margin-top: 10px;
      color: #8ea1b4;
      font-size: 10px;
    }
    .blocker {
      display: inline-flex;
      align-items: center;
      margin: 0 18px 5px 0;
      line-height: 1.4;
    }
    .blocker b {
      display: inline-grid;
      place-items: center;
      width: 21px;
      height: 21px;
      margin-right: 7px;
      border: 1px solid rgba(137,174,207,.16);
      border-radius: 6px;
      color: #698096;
      font: 800 8px/1 ui-monospace, SFMono-Regular, Menlo, monospace;
    }
    .blocker.bad { color: #e79aa6; }
    .blocker.bad b { color: var(--red); border-color: rgba(255,100,124,.2); background: rgba(255,100,124,.055); }

    .battery {
      min-height: 314px;
      padding: 27px 27px 23px;
      background:
        radial-gradient(circle at 50% 40%, var(--power-glow), transparent 38%),
        linear-gradient(145deg, rgba(255,255,255,.018), transparent 42%),
        var(--panel);
    }
    .battery-top { position: relative; z-index: 3; }
    .power-zone {
      padding: 6px 8px;
      border: 1px solid var(--line);
      border-radius: 7px;
      color: var(--power-accent);
      background: rgba(4, 12, 21, .5);
      font-size: 9px;
      letter-spacing: .1em;
    }
    .gauge-wrap { margin: 5px 0 0; }
    .gauge {
      --level: 0deg;
      --gauge-color: var(--slate);
      position: relative;
      width: 158px;
      height: 158px;
      isolation: isolate;
      background: rgba(137,174,207,.085);
      background: conic-gradient(var(--gauge-color) 0 var(--level), rgba(137,174,207,.085) var(--level) 360deg);
      box-shadow: 0 20px 50px rgba(0,0,0,.24), 0 0 36px var(--power-glow);
      transition: background .28s ease, box-shadow .28s ease;
    }
    .gauge::before {
      left: 50%;
      top: 50%;
      width: 128px;
      height: 128px;
      border-color: rgba(137,174,207,.12);
      background:
        radial-gradient(circle at 50% 35%, rgba(255,255,255,.045), transparent 42%),
        #0a1725;
      box-shadow: inset 0 0 34px rgba(0,0,0,.22);
      transform: translate(-50%, -50%);
    }
    .gauge::after {
      content: "";
      position: absolute;
      left: 50%;
      top: 50%;
      width: 176px;
      height: 176px;
      border: 1px solid rgba(137,174,207,.09);
      border-radius: 50%;
      box-shadow: inset 0 0 0 5px rgba(137,174,207,.015);
      pointer-events: none;
      transform: translate(-50%, -50%);
    }
    .gauge-value { z-index: 2; }
    .voltage { font-size: 36px; font-weight: 760; }
    .unit { font-size: 12px; }
    .window-label { margin-top: 3px; color: #70859a; font-size: 8px; letter-spacing: .09em; }
    .battery-delta {
      min-height: 17px;
      margin-top: -1px;
      color: #91a6ba;
      font: 650 9px/1.5 ui-monospace, SFMono-Regular, Menlo, monospace;
      text-align: center;
    }
    .threshold-track {
      position: relative;
      height: 5px;
      margin-top: 11px;
      border-radius: 99px;
      background: linear-gradient(90deg,
        rgba(255,100,124,.27) 0 18.18%,
        rgba(255,181,71,.27) 18.18% 36.36%,
        rgba(54,214,163,.25) 36.36% 100%);
      box-shadow: inset 0 1px 2px rgba(0,0,0,.45);
    }
    .threshold-track span {
      display: block;
      width: 0;
      height: 100%;
      border-radius: inherit;
      background: rgba(255,255,255,.23);
      transition: width .28s ease;
    }
    .threshold-track i,
    .threshold-track b {
      position: absolute;
      top: 50%;
      transform: translate(-50%, -50%);
    }
    .threshold-track i { width: 1px; height: 11px; background: rgba(244,248,252,.55); }
    .threshold-critical { left: 18.18%; }
    .threshold-autonomy { left: 36.36%; }
    .threshold-track b {
      left: 0;
      width: 10px;
      height: 10px;
      border: 2px solid #0a1725;
      border-radius: 50%;
      background: var(--slate);
      box-shadow: 0 0 10px currentColor;
      transition: left .28s ease;
    }
    .battery-foot { margin-top: 8px; }
    .threshold-name { color: #6f8498; font-size: 8px; font-weight: 800; letter-spacing: .13em; text-transform: uppercase; }
    .small-value { margin-top: 4px; font-size: 11px; }

    .health { grid-column: span 12; padding: 20px 22px 0; }
    .health-heading { display: flex; align-items: end; justify-content: space-between; padding: 0 2px 15px; }
    .health-copy { margin-top: 4px; color: #677c91; font-size: 10px; }
    .health-grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); border-top: 1px solid rgba(137,174,207,.09); }
    .health .metric {
      --metric-accent: #38c7f4;
      --metric-soft: rgba(56,199,244,.07);
      position: relative;
      grid-column: auto;
      min-height: 132px;
      padding: 18px 20px 20px;
      border-left: 1px solid rgba(137,174,207,.09);
      background: radial-gradient(circle at 92% 0%, var(--metric-soft), transparent 45%);
    }
    .health .metric:first-child { border-left: 0; }
    .health .metric::before {
      content: "";
      position: absolute;
      left: 20px;
      right: 20px;
      top: -1px;
      height: 1px;
      background: linear-gradient(90deg, transparent, var(--metric-accent), transparent);
      opacity: .45;
    }
    .metric-cpu { --metric-accent: #38c7f4 !important; --metric-soft: rgba(56,199,244,.07) !important; }
    .metric-temp { --metric-accent: #9f8cff !important; --metric-soft: rgba(159,140,255,.065) !important; }
    .metric-memory { --metric-accent: #36d6a3 !important; --metric-soft: rgba(54,214,163,.06) !important; }
    .metric-uptime { --metric-accent: #64a8ff !important; --metric-soft: rgba(100,168,255,.06) !important; }
    .metric-head { align-items: center; }
    .metric-label { color: #8296aa; font-size: 9px; font-weight: 800; letter-spacing: .15em; text-transform: uppercase; }
    .metric-icon {
      min-width: 34px;
      padding: 5px 7px;
      border: 1px solid rgba(137,174,207,.12);
      border-radius: 7px;
      color: var(--metric-accent);
      background: var(--metric-soft);
      font-size: 9px;
      text-align: center;
    }
    .metric-value { margin-top: 13px; font-size: 29px; font-weight: 750; }
    .metric-sub { min-height: 16px; color: #70859a; font-size: 10px; }
    .metric-meta { display: flex; justify-content: space-between; min-height: 16px; margin-top: 4px; color: #70859a; font-size: 9px; }
    .mini-bar { height: 3px; margin-top: 12px; background: rgba(137,174,207,.08); }
    .mini-bar > span { background: linear-gradient(90deg, rgba(255,255,255,.55), var(--metric-accent)); box-shadow: 0 0 12px var(--metric-soft); }
    .metric.warn { --metric-accent: #ffb547 !important; --metric-soft: rgba(255,181,71,.07) !important; }
    .metric.bad { --metric-accent: #ff647c !important; --metric-soft: rgba(255,100,124,.075) !important; }

    .readiness, .controller, .events { min-height: 388px; padding: 23px 24px; }
    .panel-title-row { align-items: end; margin-bottom: 15px; }
    .panel-title { margin-top: 5px; font-size: 15px; font-weight: 720; }
    .count { color: #70869c; font-size: 9px; letter-spacing: .08em; }

    .readiness-list { gap: 0; border-top: 1px solid rgba(137,174,207,.08); }
    .ready-row {
      grid-template-columns: 8px 1fr auto;
      gap: 11px;
      min-height: 43px;
      padding: 10px 7px;
      border: 0;
      border-bottom: 1px solid rgba(137,174,207,.075);
      border-radius: 0;
      background: transparent;
    }
    .ready-row.bad { margin: 0 -7px; padding-right: 14px; padding-left: 14px; background: linear-gradient(90deg, rgba(255,100,124,.045), transparent 68%); }
    .ready-row.warn { background: linear-gradient(90deg, rgba(255,181,71,.025), transparent 62%); }
    .status-light { width: 6px; height: 6px; }
    .ready-name { font-size: 11px; font-weight: 680; }
    .ready-status { color: #8298ac; font: 750 9px/1.3 ui-monospace, SFMono-Regular, Menlo, monospace; }

    .controller { display: flex; flex-direction: column; }
    .controller-orb {
      position: relative;
      width: 92px;
      height: 92px;
      margin: 23px auto 17px;
      border-color: rgba(255,100,124,.2);
      color: #af7080;
      background:
        linear-gradient(rgba(255,100,124,.13), rgba(255,100,124,.13)) center / 1px 36px no-repeat,
        linear-gradient(90deg, rgba(255,100,124,.13), rgba(255,100,124,.13)) center / 36px 1px no-repeat,
        radial-gradient(circle, rgba(255,100,124,.065), transparent 66%);
      box-shadow: inset 0 0 30px rgba(255,100,124,.035), 0 0 34px rgba(255,100,124,.025);
      font-size: 14px;
    }
    .controller-orb::before,
    .controller-orb::after {
      content: "";
      position: absolute;
      left: 50%;
      top: 50%;
      border-radius: 50%;
      pointer-events: none;
      transform: translate(-50%, -50%);
    }
    .controller-orb::before { width: 68px; height: 68px; border: 1px dashed rgba(255,100,124,.15); }
    .controller-orb::after { width: 7px; height: 7px; border: 2px solid currentColor; background: #0b1521; }
    .controller-orb span { position: relative; z-index: 2; color: currentColor; font-size: 9px; letter-spacing: .1em; }
    .controller-orb.connected {
      color: var(--green);
      border-color: rgba(54,214,163,.25);
      background:
        linear-gradient(rgba(54,214,163,.15), rgba(54,214,163,.15)) center / 1px 36px no-repeat,
        linear-gradient(90deg, rgba(54,214,163,.15), rgba(54,214,163,.15)) center / 36px 1px no-repeat,
        radial-gradient(circle, rgba(54,214,163,.08), transparent 66%);
    }
    .controller-name { font-size: 12px; }
    .controller-path { color: #6f8498; font-size: 9px; }
    .notice { margin-top: auto; padding: 11px 12px; border-color: rgba(56,199,244,.11); border-radius: 9px; color: #8da1b4; background: rgba(56,199,244,.035); font-size: 10px; }

    .event-list {
      position: relative;
      max-height: 291px;
      padding-right: 7px;
      scrollbar-width: thin;
      scrollbar-color: rgba(129,147,167,.35) transparent;
    }
    .event-list::-webkit-scrollbar { width: 5px; }
    .event-list::-webkit-scrollbar-track { background: transparent; }
    .event-list::-webkit-scrollbar-thumb { border-radius: 99px; background: rgba(129,147,167,.3); }
    .event {
      position: relative;
      grid-template-columns: 15px 1fr;
      gap: 9px;
      padding: 10px 8px;
      border: 0;
      border-radius: 9px;
    }
    .event + .event { margin-top: 2px; }
    .event::before {
      content: "";
      position: absolute;
      left: 12px;
      top: -4px;
      bottom: -4px;
      width: 1px;
      background: rgba(137,174,207,.1);
    }
    .event:first-child { border: 1px solid rgba(56,199,244,.075); background: rgba(56,199,244,.035); }
    .event-mark { position: relative; z-index: 1; width: 7px; height: 7px; margin-top: 4px; border: 2px solid #0b1521; box-shadow: 0 0 0 3px rgba(137,174,207,.045); }
    .event-message { color: #b7c5d2; font-size: 10.5px; line-height: 1.5; }
    .event-time { margin-top: 5px; color: #60758a; font-size: 9px; letter-spacing: .03em; }

    .camera-panel {
      grid-column: span 12;
      display: grid;
      grid-template-columns: minmax(0, 1.8fr) minmax(275px, .72fr);
      min-height: 420px;
      padding: 0;
    }
    .camera-viewport {
      position: relative;
      min-width: 0;
      min-height: 420px;
      overflow: hidden;
      border-right: 1px solid rgba(137,174,207,.11);
      background:
        linear-gradient(rgba(56,199,244,.025) 1px, transparent 1px),
        linear-gradient(90deg, rgba(56,199,244,.025) 1px, transparent 1px),
        radial-gradient(circle at 50% 50%, rgba(56,199,244,.07), transparent 58%),
        #030810;
      background-size: 36px 36px, 36px 36px, auto, auto;
    }
    .camera-image {
      position: absolute;
      inset: 0;
      z-index: 1;
      display: none;
      width: 100%;
      height: 100%;
      object-fit: contain;
      background: #02060b;
    }
    .camera-panel.feed-active .camera-image { display: block; }
    .camera-placeholder {
      position: absolute;
      inset: 0;
      z-index: 0;
      display: grid;
      place-items: center;
      padding: 28px;
      color: #73889c;
      text-align: center;
    }
    .camera-placeholder-mark {
      display: grid;
      place-items: center;
      width: 76px;
      height: 76px;
      margin: 0 auto 17px;
      border: 1px solid rgba(56,199,244,.18);
      color: #38c7f4;
      background: rgba(56,199,244,.045);
      font: 800 11px/1 ui-monospace, SFMono-Regular, Menlo, monospace;
      letter-spacing: .15em;
      clip-path: polygon(14% 0, 86% 0, 100% 14%, 100% 86%, 86% 100%, 14% 100%, 0 86%, 0 14%);
    }
    .camera-placeholder strong { display: block; color: #a9bccd; font-size: 13px; }
    .camera-placeholder span { display: block; margin-top: 8px; font-size: 10px; line-height: 1.55; }
    .camera-hud {
      position: absolute;
      inset: 0;
      z-index: 2;
      pointer-events: none;
      border: 1px solid rgba(56,199,244,.08);
      box-shadow: inset 0 0 55px rgba(1,7,14,.52);
    }
    .camera-hud::before,
    .camera-hud::after {
      content: "";
      position: absolute;
      left: 50%;
      top: 50%;
      background: rgba(76,211,255,.22);
      transform: translate(-50%, -50%);
    }
    .camera-hud::before { width: 54px; height: 1px; }
    .camera-hud::after { width: 1px; height: 54px; }
    .camera-corners {
      position: absolute;
      inset: 17px;
      z-index: 3;
      pointer-events: none;
      border: 1px solid transparent;
      background:
        linear-gradient(#38c7f4, #38c7f4) left top / 25px 1px no-repeat,
        linear-gradient(#38c7f4, #38c7f4) left top / 1px 25px no-repeat,
        linear-gradient(#38c7f4, #38c7f4) right top / 25px 1px no-repeat,
        linear-gradient(#38c7f4, #38c7f4) right top / 1px 25px no-repeat,
        linear-gradient(#38c7f4, #38c7f4) left bottom / 25px 1px no-repeat,
        linear-gradient(#38c7f4, #38c7f4) left bottom / 1px 25px no-repeat,
        linear-gradient(#38c7f4, #38c7f4) right bottom / 25px 1px no-repeat,
        linear-gradient(#38c7f4, #38c7f4) right bottom / 1px 25px no-repeat;
      opacity: .42;
    }
    .camera-overlay {
      position: absolute;
      z-index: 4;
      display: flex;
      align-items: center;
      gap: 7px;
      padding: 7px 9px;
      border: 1px solid rgba(137,174,207,.16);
      border-radius: 7px;
      color: #c1d1df;
      background: rgba(3,9,16,.78);
      backdrop-filter: blur(8px);
      font: 750 9px/1 ui-monospace, SFMono-Regular, Menlo, monospace;
      letter-spacing: .08em;
      text-transform: uppercase;
    }
    .camera-overlay.live { left: 27px; top: 27px; }
    .camera-overlay.orientation { right: 27px; bottom: 27px; }
    .camera-overlay .status-light { flex: 0 0 auto; }
    .camera-sidebar {
      position: relative;
      z-index: 3;
      display: flex;
      flex-direction: column;
      padding: 25px 24px 23px;
      background:
        linear-gradient(135deg, rgba(56,199,244,.035), transparent 45%),
        rgba(9,20,33,.86);
    }
    .camera-title { margin: 7px 0 4px; font-size: 20px; letter-spacing: -.025em; }
    .camera-copy { margin: 0; color: #71869a; font-size: 10px; line-height: 1.6; }
    .camera-state {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin: 22px 0 17px;
      padding: 13px 0;
      border-top: 1px solid rgba(137,174,207,.09);
      border-bottom: 1px solid rgba(137,174,207,.09);
    }
    .camera-state-label { color: #71869a; font-size: 8px; font-weight: 800; letter-spacing: .14em; text-transform: uppercase; }
    .camera-state-value { color: #ffb547; font: 800 10px/1.3 ui-monospace, SFMono-Regular, Menlo, monospace; text-align: right; }
    .camera-state-value.good { color: #36d6a3; }
    .camera-state-value.bad { color: #ff647c; }
    .camera-facts { display: grid; grid-template-columns: 1fr 1fr; gap: 1px; background: rgba(137,174,207,.08); }
    .camera-fact { min-height: 66px; padding: 12px; background: #0a1726; }
    .camera-fact span { display: block; color: #61778c; font-size: 8px; font-weight: 800; letter-spacing: .12em; text-transform: uppercase; }
    .camera-fact b { display: block; margin-top: 7px; color: #bdcbd8; font: 750 10px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace; }
    .camera-contract {
      margin-top: auto;
      padding: 12px;
      border-left: 2px solid rgba(56,199,244,.45);
      color: #8397aa;
      background: rgba(56,199,244,.035);
      font-size: 9px;
      line-height: 1.55;
    }

    .trend { min-height: 334px; padding: 23px 25px 22px; }
    .chart-head-right { display: flex; align-items: center; gap: 21px; }
    .chart-stats { display: flex; gap: 14px; }
    .chart-stats span { color: #677d91; font: 750 8px/1.4 ui-monospace, SFMono-Regular, Menlo, monospace; letter-spacing: .1em; }
    .chart-stats b { display: block; margin-top: 3px; color: #c4d2df; font-size: 10px; letter-spacing: 0; }
    .legend { gap: 12px; font-size: 9px; }
    .legend i { width: 16px; }
    .chart-wrap {
      height: 246px;
      margin-top: 3px;
      border-top: 1px solid rgba(137,174,207,.055);
      background: linear-gradient(180deg, rgba(56,199,244,.012), transparent 54%);
    }

    footer { margin-top: 17px; padding: 0 3px; color: #536a7f; font-size: 9px; }
    footer strong { color: #7c94a9; }

    @media (max-width: 1120px) {
      .hero, .battery { grid-column: span 12; }
      .health-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .health .metric:nth-child(3) { border-left: 0; }
      .health .metric:nth-child(n+3) { border-top: 1px solid rgba(137,174,207,.09); }
      .readiness { grid-column: span 7; }
      .controller { grid-column: span 5; }
      .events { grid-column: span 12; }
      .camera-panel { grid-template-columns: minmax(0, 1.55fr) minmax(265px, .75fr); }
      .event-list { max-height: 320px; }
      .chart-head-right { align-items: flex-end; flex-direction: column; gap: 9px; }
    }

    @media (max-width: 700px) {
      .shell { width: calc(100% - 20px); padding-top: 14px; }
      header { align-items: flex-start; min-height: 55px; padding-bottom: 14px; }
      .brand-mark { width: 40px; height: 40px; border-radius: 11px; font-size: 12px; }
      h1 { font-size: 17px; }
      .brand-subline, .mode-pill, .clock { display: none; }
      .live-pill { min-width: 88px; padding: 7px 8px; font-size: 8px; }
      .grid { gap: 11px; }
      .hero, .battery, .health, .readiness, .controller, .events, .camera-panel, .trend { grid-column: span 12; }
      .hero { min-height: 350px; padding: 23px 20px 21px; }
      .hero-glow { right: -235px; top: -185px; }
      .state-title { margin-top: 13px; font-size: 34px; line-height: 1.01; }
      .state-copy { font-size: 12px; }
      .blockers { display: grid; grid-template-columns: 1fr; }
      .blocker { margin-right: 0; }
      .battery, .readiness, .controller, .events, .trend { padding: 20px; }
      .health { padding: 18px 17px 0; }
      .health-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .health .metric { min-height: 126px; padding: 17px 12px 18px; }
      .health .metric:nth-child(odd) { border-left: 0; }
      .health .metric:nth-child(n+3) { border-top: 1px solid rgba(137,174,207,.09); }
      .metric-value { font-size: 25px; }
      .metric-meta { display: block; }
      .metric-meta span { display: block; }
      .panel-title-row { align-items: flex-start; flex-wrap: wrap; }
      .ready-row { grid-template-columns: 7px 1fr; }
      .ready-status { grid-column: 2; text-align: left; }
      .event-list { max-height: none; overflow: visible; }
      .camera-panel { grid-template-columns: 1fr; min-height: 0; }
      .camera-viewport { min-height: 225px; border-right: 0; border-bottom: 1px solid rgba(137,174,207,.11); }
      .camera-sidebar { min-height: 350px; padding: 20px; }
      .camera-overlay.live { left: 18px; top: 18px; }
      .camera-overlay.orientation { right: 18px; bottom: 18px; }
      .chart-head-right { width: 100%; margin-top: 12px; align-items: flex-start; }
      .chart-stats { width: 100%; justify-content: space-between; }
      .legend { display: none; }
      .chart-wrap { height: 215px; }
      footer { flex-direction: column; line-height: 1.5; }
    }

    @media (max-width: 430px) {
      .health-grid { grid-template-columns: 1fr; }
      .health .metric { border-left: 0; border-top: 1px solid rgba(137,174,207,.09); }
      .health .metric:first-child { border-top: 0; }
      .battery-top { align-items: flex-start; }
      .power-zone { max-width: 135px; }
    }

    @media (prefers-reduced-motion: no-preference) {
      .live-pill:not(.offline) .dot { animation: livePulse 2.4s ease-out infinite; }
      @keyframes livePulse {
        0%, 55%, 100% { box-shadow: 0 0 9px var(--green); }
        75% { box-shadow: 0 0 9px var(--green), 0 0 0 7px rgba(54,214,163,0); }
      }
    }

    @media (prefers-reduced-motion: reduce) {
      *, *::before, *::after { scroll-behavior: auto !important; animation: none !important; transition-duration: .01ms !important; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <header>
      <div class="brand">
        <div class="brand-mark">SD</div>
        <div>
          <div class="eyebrow">Sentinel Drive</div>
          <h1>Jetson Nano Robot Car</h1>
          <div class="brand-subline"><span>EDGE NODE 01</span><i></i><span>LIVE OBSERVABILITY</span></div>
        </div>
      </div>
      <div class="top-status">
        <div class="mode-pill"><span>R/O</span> Safety monitor</div>
        <div class="clock"><div id="utcClock">--:--:-- UTC</div><div id="lastUpdate">Waiting for telemetry</div></div>
        <div class="live-pill offline" id="livePill" role="status" aria-live="polite"><span class="dot"></span><span id="liveText">Connecting</span></div>
      </div>
    </header>

    <main class="grid">
      <section class="panel hero">
        <div class="hero-glow"></div>
        <div class="hero-copy">
          <div class="section-label"><span class="section-index">01</span> Overall safety state</div>
          <h2 class="state-title" id="stateTitle" role="status" aria-live="polite">INITIALIZING</h2>
          <p class="state-copy" id="stateCopy">Waiting for the first read-only telemetry sample from the Jetson.</p>
        </div>
        <div class="hero-bottom">
          <div class="chips">
            <span class="chip locked"><i></i>Read-only monitoring</span>
            <span class="chip locked"><i></i>Motion not authorized</span>
          </div>
          <div class="blockers" aria-label="Active blockers">
            <span class="blocker"><b>01</b> Physical outputs unverified</span>
            <span class="blocker"><b>02</b> Integration validation pending</span>
            <span class="blocker" id="controllerChip"><b>03</b><span id="controllerChipText">Controller unavailable</span></span>
          </div>
        </div>
      </section>

      <section class="panel battery">
        <div class="battery-top">
          <div><div class="section-label"><span class="section-index">02</span> Motor battery bus</div><div class="small-value mono">INA219 · 0x41</div></div>
          <div class="power-zone" id="powerZone">Sensor pending</div>
        </div>
        <div class="gauge-wrap">
          <div class="gauge" id="batteryGauge" role="meter" aria-label="Battery bus voltage" aria-valuemin="0" aria-valuemax="16">
            <div class="gauge-value"><span class="voltage" id="voltage">--</span><span class="unit">V</span><div class="window-label" id="windowLabel">operating window</div></div>
          </div>
        </div>
        <div class="battery-delta" id="batteryDelta">Awaiting voltage sample</div>
        <div class="threshold-track" aria-hidden="true"><span id="thresholdFill"></span><i class="threshold-critical"></i><i class="threshold-autonomy"></i><b id="voltageMarker"></b></div>
        <div class="battery-foot">
          <div><div class="threshold-name">Critical</div><div class="small-value mono">6.600 V</div></div>
          <div style="text-align:right"><div class="threshold-name">Autonomy</div><div class="small-value mono">7.000 V</div></div>
        </div>
      </section>

      <section class="panel health">
        <div class="health-heading"><div><div class="section-label"><span class="section-index">03</span> Jetson health</div><div class="health-copy">Edge-compute vital signs</div></div><span class="count">JETSON NANO</span></div>
        <div class="health-grid">
          <article class="metric metric-cpu" id="cpuMetric">
            <div class="metric-head"><div class="metric-label">CPU load</div><div class="metric-icon">CPU</div></div>
            <div class="metric-value" id="cpuValue">--</div><div class="metric-sub">Processor utilization</div>
            <div class="mini-bar"><span id="cpuBar"></span></div>
          </article>
          <article class="metric metric-temp" id="tempMetric">
            <div class="metric-head"><div class="metric-label">Temperature</div><div class="metric-icon">°C</div></div>
            <div class="metric-value" id="tempValue">--</div><div class="metric-sub" id="tempLabel">Highest thermal zone</div>
            <div class="mini-bar"><span id="tempBar"></span></div>
          </article>
          <article class="metric metric-memory" id="memoryMetric">
            <div class="metric-head"><div class="metric-label">Memory</div><div class="metric-icon">RAM</div></div>
            <div class="metric-value" id="memoryValue">--</div><div class="metric-sub">System memory used</div>
            <div class="mini-bar"><span id="memoryBar"></span></div>
          </article>
          <article class="metric metric-uptime" id="diskMetric">
            <div class="metric-head"><div class="metric-label">Uptime</div><div class="metric-icon">UP</div></div>
            <div class="metric-value" id="uptimeValue">--</div>
            <div class="metric-meta"><span id="loadValue">Load unavailable</span><span id="diskValue">Disk unavailable</span></div>
            <div class="mini-bar"><span id="diskBar"></span></div>
          </article>
        </div>
      </section>

      <section class="panel camera-panel" id="cameraPanel" aria-label="Read-only CSI camera monitor">
        <div class="camera-viewport">
          <img class="camera-image" id="cameraFeed" alt="Live forward-facing CSI camera feed configured with 180-degree rotation">
          <div class="camera-placeholder" id="cameraPlaceholder">
            <div><div class="camera-placeholder-mark">CSI</div><strong id="cameraPlaceholderTitle">Camera monitoring disabled</strong><span id="cameraPlaceholderCopy">Launch with --camera csi to enable the read-only forward view.</span></div>
          </div>
          <div class="camera-hud" aria-hidden="true"></div>
          <div class="camera-corners" aria-hidden="true"></div>
          <div class="camera-overlay live"><span class="status-light" id="cameraOverlayLight"></span><span id="cameraOverlayText">CSI offline</span></div>
          <div class="camera-overlay orientation" id="cameraOrientationOverlay">ROTATION PENDING</div>
        </div>
        <div class="camera-sidebar">
          <div class="section-label">04 · Perception input</div>
          <h3 class="camera-title">Forward CSI view</h3>
          <p class="camera-copy">Single-owner, latest-frame monitoring stream from the Jetson camera pipeline.</p>
          <div class="camera-state"><span class="camera-state-label">Stream state</span><strong class="camera-state-value" id="cameraState">DISABLED</strong></div>
          <div class="camera-facts">
            <div class="camera-fact"><span>Source</span><b id="cameraSource">CSI sensor 0</b></div>
            <div class="camera-fact"><span>Output</span><b id="cameraResolution">640 × 360</b></div>
            <div class="camera-fact"><span>Frame age</span><b id="cameraFrameAge">--</b></div>
            <div class="camera-fact"><span>Orientation</span><b id="cameraOrientation">--</b></div>
          </div>
          <div class="camera-contract">Read-only MJPEG telemetry. No recording, drive controls, motor routes, or command endpoints are exposed.</div>
        </div>
      </section>

      <section class="panel readiness">
        <div class="panel-title-row"><div><div class="section-label">05 · Safety matrix</div><h3 class="panel-title">Subsystem readiness</h3></div><span class="count">READ ONLY</span></div>
        <div class="readiness-list" id="readinessList"></div>
      </section>

      <section class="panel controller">
        <div class="panel-title-row"><div><div class="section-label">06 · Input link</div><h3 class="panel-title">Manual controller</h3></div><span class="count" id="controllerState">OFFLINE</span></div>
        <div class="controller-orb" id="controllerOrb"><span>--</span></div>
        <div class="center"><div class="controller-name" id="controllerName">Xbox controller not detected</div><div class="controller-path" id="controllerPath">No accessible input event</div></div>
        <div class="notice">Bluetooth is not required for this monitoring stage. Motion remains unauthorized even if a controller appears.</div>
      </section>

      <section class="panel events">
        <div class="panel-title-row"><div><div class="section-label">07 · Event stream</div><h3 class="panel-title">Safety events</h3></div><span class="count" id="eventCount">0 LOGGED</span></div>
        <div class="event-list" id="eventList"><div class="event"><span class="event-mark"></span><div><div class="event-message">Waiting for telemetry.</div></div></div></div>
      </section>

      <section class="panel trend">
        <div class="panel-title-row">
          <div><div class="section-label">08 · Power telemetry</div><h3 class="panel-title">Battery voltage history</h3></div>
          <div class="chart-head-right">
            <div class="chart-stats"><span>NOW <b id="chartNow">--</b></span><span>MIN <b id="chartMin">--</b></span><span>MAX <b id="chartMax">--</b></span></div>
            <div class="legend"><span><i></i>Bus voltage</span><span><i class="amber"></i>7.00 V</span><span><i class="red"></i>6.60 V</span></div>
          </div>
        </div>
        <div class="chart-wrap"><canvas id="voltageChart" role="img" aria-label="Recent motor battery bus voltage trend"></canvas></div>
      </section>
    </main>

    <footer><span>Motion authorization: <strong>FALSE</strong> · No motor command endpoints exist</span><span class="mono" id="hostInfo">Jetson telemetry service</span></footer>
  </div>

  <script>
    "use strict";
    const $ = (id) => document.getElementById(id);
    let latestHistory = [];
    let requestInFlight = false;
    let lastReadinessSignature = "";
    let lastEventSignature = "";
    let resizeTimer = null;
    let cameraFeedRequested = false;
    let cameraRetryAfter = 0;

    function clamp(value, low, high) { return Math.max(low, Math.min(high, value)); }
    function setTextIfChanged(id, value) {
      const element = $(id);
      if (element.textContent !== value) element.textContent = value;
    }
    function formatNumber(value, digits, suffix) {
      return Number.isFinite(value) ? value.toFixed(digits) + suffix : "--";
    }
    function formatUptime(seconds) {
      if (!Number.isFinite(seconds)) return "--";
      const days = Math.floor(seconds / 86400);
      const hours = Math.floor((seconds % 86400) / 3600);
      const minutes = Math.floor((seconds % 3600) / 60);
      if (days > 0) return days + "d " + hours + "h";
      if (hours > 0) return hours + "h " + minutes + "m";
      return minutes + "m";
    }
    function colorForZone(zone) {
      if (zone === "AUTONOMOUS_CAPABLE") return "var(--green)";
      if (zone === "MANUAL_RECOVERY") return "var(--amber)";
      if (zone === "CRITICAL_STOP" || zone === "BATTERY_ABSENT_OR_CRITICAL") return "var(--red)";
      return "var(--slate)";
    }
    function updateClock() {
      const now = new Date();
      $("utcClock").textContent = now.toISOString().slice(11, 19) + " UTC";
    }

    function setMetricTone(id, value, warningAt, criticalAt) {
      const card = $(id);
      const valid = Number.isFinite(value);
      card.classList.toggle("warn", valid && value >= warningAt && value < criticalAt);
      card.classList.toggle("bad", valid && value >= criticalAt);
    }

    function formatEventTime(value) {
      if (!value) return "";
      const parsed = new Date(value);
      if (Number.isNaN(parsed.getTime())) return value;
      const months = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
      const iso = parsed.toISOString();
      return iso.slice(8, 10) + " " + months[parsed.getUTCMonth()] + " · " + iso.slice(11, 19) + " UTC";
    }

    function formatChartTime(value) {
      if (!value) return "--:--";
      const parsed = new Date(value);
      return Number.isNaN(parsed.getTime()) ? "--:--" : parsed.toISOString().slice(11, 16);
    }

    function renderReadiness(items) {
      const signature = JSON.stringify(items || []);
      if (signature === lastReadinessSignature) return;
      lastReadinessSignature = signature;
      const list = $("readinessList");
      list.textContent = "";
      items.forEach((item) => {
        const row = document.createElement("div");
        row.className = "ready-row " + (item.level || "");
        const light = document.createElement("span");
        light.className = "status-light " + (item.level || "");
        const name = document.createElement("span");
        name.className = "ready-name";
        name.textContent = item.name;
        const status = document.createElement("span");
        status.className = "ready-status";
        status.textContent = item.status;
        row.appendChild(light); row.appendChild(name); row.appendChild(status);
        list.appendChild(row);
      });
    }

    function renderEvents(events) {
      const all = events || [];
      const visible = all.slice().reverse().slice(0, 12);
      $("eventCount").textContent = all.length + " LOGGED";
      const signature = JSON.stringify(visible);
      if (signature === lastEventSignature) return;
      lastEventSignature = signature;
      const list = $("eventList");
      list.textContent = "";
      if (!visible.length) {
        const empty = document.createElement("div");
        empty.className = "notice";
        empty.textContent = "No safety events recorded in this session.";
        list.appendChild(empty);
        return;
      }
      visible.forEach((item) => {
        const row = document.createElement("div");
        row.className = "event " + (item.level || "");
        const mark = document.createElement("span");
        mark.className = "event-mark";
        const body = document.createElement("div");
        const message = document.createElement("div");
        message.className = "event-message";
        message.textContent = item.message;
        const stamp = document.createElement("div");
        stamp.className = "event-time";
        stamp.textContent = formatEventTime(item.timestamp_utc);
        stamp.title = item.timestamp_utc || "";
        body.appendChild(message); body.appendChild(stamp);
        row.appendChild(mark); row.appendChild(body); list.appendChild(row);
      });
    }

    function requestCameraFeed() {
      if (cameraFeedRequested || Date.now() < cameraRetryAfter) return;
      cameraFeedRequested = true;
      $("cameraFeed").src = "/camera.mjpg?session=" + Date.now();
    }

    function updateCamera(camera) {
      camera = camera || { enabled: false, live: false };
      const enabled = camera.enabled === true;
      const live = enabled && camera.live === true;
      const panel = $("cameraPanel");
      const state = $("cameraState");

      if (!enabled && cameraFeedRequested) {
        $("cameraFeed").removeAttribute("src");
        cameraFeedRequested = false;
      }
      if (enabled && live) requestCameraFeed();

      panel.classList.toggle("feed-active", live && cameraFeedRequested);
      state.textContent = live ? "STREAM LIVE" : enabled ? "CAMERA FAULT" : "DISABLED";
      state.className = "camera-state-value " + (live ? "good" : enabled ? "bad" : "");
      $("cameraOverlayLight").className = "status-light " + (live ? "good" : enabled ? "bad" : "");
      $("cameraOverlayText").textContent = live ? "CSI live" : enabled ? "CSI fault" : "CSI offline";
      $("cameraSource").textContent = enabled ? "CSI sensor " + camera.sensor_id : "CSI not started";
      $("cameraResolution").textContent = enabled ? camera.width + " × " + camera.height : "--";
      $("cameraFrameAge").textContent = Number.isFinite(camera.frame_age_ms) ? camera.frame_age_ms + " ms" : "--";
      const flipLabel = Number.isInteger(camera.flip_method) ? "FLIP " + camera.flip_method : "--";
      $("cameraOrientation").textContent = enabled ? flipLabel + " / VERIFY" : "--";
      $("cameraOrientationOverlay").textContent = enabled ? "ROTATION 180° · " + flipLabel : "ROTATION PENDING";
      $("cameraPlaceholderTitle").textContent = enabled ? (camera.error ? "Camera stream unavailable" : "Starting camera stream") : "Camera monitoring disabled";
      $("cameraPlaceholderCopy").textContent = camera.error || (enabled ? "Waiting for the first upright CSI frame." : "Launch with --camera csi to enable the read-only forward view.");
    }

    function renderChart() {
      const canvas = $("voltageChart");
      const box = canvas.getBoundingClientRect();
      const ratio = Math.min(window.devicePixelRatio || 1, 2);
      const width = Math.max(280, Math.floor(box.width));
      const height = Math.max(180, Math.floor(box.height));
      canvas.width = width * ratio;
      canvas.height = height * ratio;
      const ctx = canvas.getContext("2d");
      ctx.scale(ratio, ratio);
      ctx.clearRect(0, 0, width, height);

      const pad = { left: 46, right: 18, top: 12, bottom: 28 };
      const plotW = width - pad.left - pad.right;
      const plotH = height - pad.top - pad.bottom;
      const minV = 6.2, maxV = 8.5;
      const y = (v) => pad.top + (maxV - v) / (maxV - minV) * plotH;

      ctx.fillStyle = "rgba(255,100,124,.030)";
      ctx.fillRect(pad.left, y(6.6), plotW, y(minV) - y(6.6));
      ctx.fillStyle = "rgba(255,181,71,.022)";
      ctx.fillRect(pad.left, y(7.0), plotW, y(6.6) - y(7.0));
      ctx.fillStyle = "rgba(54,214,163,.014)";
      ctx.fillRect(pad.left, y(maxV), plotW, y(7.0) - y(maxV));

      ctx.font = "9px ui-monospace, monospace";
      ctx.lineWidth = 1;
      [6.2, 6.6, 7.0, 7.5, 8.0, 8.5].forEach((v) => {
        ctx.setLineDash(v === 6.6 || v === 7.0 ? [5, 5] : []);
        ctx.beginPath(); ctx.moveTo(pad.left, y(v)); ctx.lineTo(width - pad.right, y(v));
        ctx.strokeStyle = v === 6.6 ? "rgba(255,100,124,.42)" : v === 7.0 ? "rgba(255,181,71,.42)" : "rgba(137,174,207,.085)";
        ctx.stroke(); ctx.fillStyle = "#61788e"; ctx.fillText(v.toFixed(1) + "V", 5, y(v) + 3);
      });
      ctx.setLineDash([]);

      const points = latestHistory.filter((p) => Number.isFinite(p.voltage_v));
      const values = points.map((point) => point.voltage_v);
      $("chartNow").textContent = values.length ? values[values.length - 1].toFixed(3) + " V" : "--";
      $("chartMin").textContent = values.length ? Math.min.apply(null, values).toFixed(3) + " V" : "--";
      $("chartMax").textContent = values.length ? Math.max.apply(null, values).toFixed(3) + " V" : "--";

      const coords = points.map((point, index) => ({
        x: pad.left + (index / Math.max(1, points.length - 1)) * plotW,
        y: y(clamp(point.voltage_v, minV, maxV)),
      }));

      if (coords.length > 1) {
        const fill = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
        fill.addColorStop(0, "rgba(56,199,244,.20)");
        fill.addColorStop(1, "rgba(56,199,244,0)");
        ctx.beginPath();
        coords.forEach((point, index) => index === 0 ? ctx.moveTo(point.x, point.y) : ctx.lineTo(point.x, point.y));
        ctx.lineTo(coords[coords.length - 1].x, y(minV));
        ctx.lineTo(coords[0].x, y(minV));
        ctx.closePath();
        ctx.fillStyle = fill;
        ctx.fill();

        ctx.beginPath();
        coords.forEach((point, index) => index === 0 ? ctx.moveTo(point.x, point.y) : ctx.lineTo(point.x, point.y));
        ctx.lineJoin = "round"; ctx.lineCap = "round"; ctx.lineWidth = 2.25;
        ctx.strokeStyle = "#38c7f4";
        ctx.shadowColor = "rgba(56,199,244,.45)"; ctx.shadowBlur = 9; ctx.stroke(); ctx.shadowBlur = 0;
      }

      if (coords.length) {
        const latest = coords[coords.length - 1];
        ctx.beginPath(); ctx.arc(latest.x, latest.y, 3.6, 0, Math.PI * 2);
        ctx.fillStyle = "#d9f6ff"; ctx.shadowColor = "#38c7f4"; ctx.shadowBlur = 13; ctx.fill(); ctx.shadowBlur = 0;
      } else {
        ctx.fillStyle = "#61788e"; ctx.textAlign = "center";
        ctx.fillText("Waiting for valid battery samples", pad.left + plotW / 2, pad.top + plotH / 2);
        ctx.textAlign = "left";
      }

      if (values.some((value) => value < minV || value > maxV)) {
        ctx.fillStyle = "#ff647c"; ctx.textAlign = "right";
        ctx.fillText("OUTSIDE DISPLAY RANGE", width - pad.right, pad.top + 10);
      }

      ctx.fillStyle = "#61788e";
      if (points.length) {
        const middle = points[Math.floor((points.length - 1) / 2)];
        ctx.textAlign = "left"; ctx.fillText(formatChartTime(points[0].timestamp_utc), pad.left, height - 7);
        ctx.textAlign = "center"; ctx.fillText(formatChartTime(middle.timestamp_utc), pad.left + plotW / 2, height - 7);
        ctx.textAlign = "right"; ctx.fillText(formatChartTime(points[points.length - 1].timestamp_utc), width - pad.right, height - 7);
      } else {
        ctx.textAlign = "left"; ctx.fillText("oldest", pad.left, height - 7);
        ctx.textAlign = "right"; ctx.fillText("now", width - pad.right, height - 7);
      }
      ctx.textAlign = "left";
    }

    function applyStatus(data) {
      $("livePill").classList.toggle("offline", !data.telemetry_live);
      setTextIfChanged("liveText", data.telemetry_live ? (data.demo_mode ? "Demo live" : "Monitor live") : "Sampler stale");
      $("lastUpdate").textContent = "Updated " + (data.sample_age_ms || 0) + " ms ago";
      setTextIfChanged("stateTitle", data.safety.overall_state.replace(/_/g, " "));
      setTextIfChanged("stateCopy", data.safety.message);

      const power = data.power;
      document.documentElement.setAttribute("data-safety", data.safety.overall_state || "INITIALIZING");
      document.documentElement.setAttribute("data-zone", power.zone || "SENSOR_FAULT");
      const voltage = Number.isFinite(power.voltage_v) ? power.voltage_v : null;
      $("voltage").textContent = voltage === null ? "--" : voltage.toFixed(3);
      $("powerZone").textContent = power.zone.replace(/_/g, " ");
      const gaugeMin = 6.2, gaugeMax = 8.4;
      const pct = voltage === null ? 0 : clamp((voltage - gaugeMin) / (gaugeMax - gaugeMin) * 100, 0, 100);
      const gauge = $("batteryGauge");
      gauge.style.setProperty("--level", (pct * 3.6).toFixed(1) + "deg");
      gauge.style.setProperty("--gauge-color", colorForZone(power.zone));
      $("windowLabel").textContent = power.sensor_ok ? "6.2-8.4 V display range" : "telemetry unavailable";
      if (voltage === null) {
        $("batteryDelta").textContent = "No valid voltage sample";
        gauge.removeAttribute("aria-valuenow");
        gauge.setAttribute("aria-valuetext", "Battery voltage unavailable");
      } else {
        gauge.setAttribute("aria-valuenow", voltage.toFixed(3));
        gauge.setAttribute("aria-valuetext", voltage.toFixed(3) + " volts, " + power.zone.replace(/_/g, " "));
        if (voltage >= 7.0) {
          $("batteryDelta").textContent = "+" + (voltage - 7.0).toFixed(3) + " V above autonomy threshold";
        } else if (voltage >= 6.6) {
          $("batteryDelta").textContent = (7.0 - voltage).toFixed(3) + " V below autonomy threshold";
        } else {
          $("batteryDelta").textContent = (6.6 - voltage).toFixed(3) + " V below critical threshold";
        }
      }
      $("thresholdFill").style.width = pct + "%";
      $("voltageMarker").style.left = pct + "%";
      $("voltageMarker").style.background = colorForZone(power.zone);

      const system = data.system || {};
      $("cpuValue").textContent = formatNumber(system.cpu_used_pct, 1, "%");
      $("cpuBar").style.width = clamp(system.cpu_used_pct || 0, 0, 100) + "%";
      $("memoryValue").textContent = formatNumber(system.memory_used_pct, 1, "%");
      $("memoryBar").style.width = clamp(system.memory_used_pct || 0, 0, 100) + "%";
      $("uptimeValue").textContent = formatUptime(system.uptime_s);
      $("loadValue").textContent = Number.isFinite(system.load_1m) ? "1-minute load " + system.load_1m.toFixed(2) : "Load average unavailable";
      $("diskValue").textContent = Number.isFinite(system.disk_used_pct) ? "Disk " + system.disk_used_pct.toFixed(1) + "% used" : "Disk unavailable";
      $("diskBar").style.width = clamp(system.disk_used_pct || 0, 0, 100) + "%";
      $("tempValue").textContent = formatNumber(system.max_temperature_c, 1, "°C");
      $("tempLabel").textContent = system.max_temperature_zone || "Highest thermal zone";
      $("tempBar").style.width = clamp((system.max_temperature_c || 0) / 90 * 100, 0, 100) + "%";
      setMetricTone("cpuMetric", system.cpu_used_pct, 75, 90);
      setMetricTone("tempMetric", system.max_temperature_c, 70, 82);
      setMetricTone("memoryMetric", system.memory_used_pct, 80, 92);
      setMetricTone("diskMetric", system.disk_used_pct, 80, 92);

      const controller = data.controller || {};
      const inputPresent = !!controller.input_device_present;
      $("controllerState").textContent = inputPresent ? (controller.accessible ? "INPUT PRESENT" : "NO ACCESS") : "NOT PRESENT";
      $("controllerOrb").classList.toggle("connected", inputPresent && controller.accessible);
      $("controllerOrb").textContent = inputPresent && controller.accessible ? "INPUT" : "--";
      $("controllerName").textContent = inputPresent ? controller.name : "Xbox input device not present";
      $("controllerPath").textContent = inputPresent ? (controller.path || "Event path unavailable") : "No matching input event";
      $("controllerChipText").textContent = inputPresent ? "Controller input present · unverified" : "Controller unavailable";
      $("controllerChip").className = "blocker";

      renderReadiness(data.readiness || []);
      renderEvents(data.events || []);
      updateCamera(data.camera);
      latestHistory = data.history || [];
      renderChart();
      $("hostInfo").textContent = (system.hostname || "Jetson") + " · " + (system.ip_address || "local") + " · schema v" + data.schema_version;
    }

    function markOffline(message) {
      $("livePill").classList.add("offline");
      setTextIfChanged("liveText", "Telemetry offline");
      $("lastUpdate").textContent = message || "Connection failed";
    }

    async function poll() {
      if (requestInFlight) return;
      requestInFlight = true;
      let timer = null;
      try {
        const controller = typeof AbortController === "undefined" ? null : new AbortController();
        const options = { cache: "no-store" };
        if (controller) {
          options.signal = controller.signal;
          timer = setTimeout(() => controller.abort(), 2500);
        }
        const response = await fetch("/api/status", options);
        if (!response.ok) throw new Error("HTTP " + response.status);
        applyStatus(await response.json());
      } catch (error) {
        markOffline(error && error.name === "AbortError" ? "Telemetry request timed out" : "Cannot reach dashboard service");
      } finally {
        if (timer !== null) clearTimeout(timer);
        requestInFlight = false;
        setTimeout(poll, 1000);
      }
    }

    updateClock();
    setInterval(updateClock, 1000);
    $("cameraFeed").addEventListener("error", () => {
      cameraFeedRequested = false;
      cameraRetryAfter = Date.now() + 2500;
      $("cameraPanel").classList.remove("feed-active");
    });
    window.addEventListener("resize", () => {
      if (resizeTimer !== null) clearTimeout(resizeTimer);
      resizeTimer = setTimeout(renderChart, 120);
    });
    poll();
  </script>
</body>
</html>
"""


def utc_timestamp():
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def safe_float(value, digits=3):
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return round(number, digits)


def swap_word(value):
    return ((value & 0x00FF) << 8) | ((value & 0xFF00) >> 8)


def classify_voltage(voltage):
    if voltage is None:
        return "SENSOR_FAULT"
    if voltage < DISCONNECTED_VOLTAGE:
        return "BATTERY_ABSENT_OR_CRITICAL"
    if voltage < CRITICAL_VOLTAGE:
        return "CRITICAL_STOP"
    if voltage < AUTONOMOUS_VOLTAGE:
        return "MANUAL_RECOVERY"
    return "AUTONOMOUS_CAPABLE"


class Ina219VoltageReader(object):
    def __init__(self, bus_number, address, demo=False):
        self.bus_number = int(bus_number)
        self.address = int(address)
        self.demo = bool(demo)
        self.bus = None
        self.started = time.monotonic()

    def _ensure_bus(self):
        if self.demo:
            return
        if smbus is None:
            raise RuntimeError("Python smbus module is not installed")
        if self.bus is None:
            self.bus = smbus.SMBus(self.bus_number)

    def read_voltage(self):
        if self.demo:
            elapsed = time.monotonic() - self.started
            # Crosses the 7.00 V line slowly without entering the critical zone.
            return 7.18 + 0.30 * math.sin(elapsed / 11.0)

        self._ensure_bus()
        raw_value = self.bus.read_word_data(
            self.address,
            INA219_BUS_VOLTAGE_REGISTER,
        )
        voltage_register = swap_word(raw_value)
        return (voltage_register >> 3) * 0.004

    def reset_bus(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None

    def close(self):
        self.reset_bus()


class SystemMetrics(object):
    def __init__(self, demo=False):
        self.demo = bool(demo)
        self.started = time.monotonic()
        self.previous_cpu = None
        self.hostname = socket.gethostname()
        self.ip_address = self._ip_address()

    def _read_cpu_percent(self):
        try:
            with open("/proc/stat", "r") as stat_file:
                values = stat_file.readline().split()[1:]
            values = [int(value) for value in values]
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            total = sum(values)
            current = (idle, total)
            if self.previous_cpu is None:
                self.previous_cpu = current
                return None
            idle_delta = idle - self.previous_cpu[0]
            total_delta = total - self.previous_cpu[1]
            self.previous_cpu = current
            if total_delta <= 0:
                return None
            return max(0.0, min(100.0, 100.0 * (1.0 - float(idle_delta) / total_delta)))
        except (IOError, OSError, ValueError, IndexError):
            return None

    @staticmethod
    def _read_memory_percent():
        try:
            values = {}
            with open("/proc/meminfo", "r") as memory_file:
                for line in memory_file:
                    key, value = line.split(":", 1)
                    values[key] = int(value.strip().split()[0])
            total = float(values["MemTotal"])
            available = float(values.get("MemAvailable", values.get("MemFree", 0)))
            if total <= 0:
                return None
            return max(0.0, min(100.0, 100.0 * (total - available) / total))
        except (IOError, OSError, ValueError, KeyError):
            return None

    @staticmethod
    def _read_uptime():
        try:
            with open("/proc/uptime", "r") as uptime_file:
                return float(uptime_file.read().split()[0])
        except (IOError, OSError, ValueError, IndexError):
            return None

    @staticmethod
    def _read_load():
        try:
            with open("/proc/loadavg", "r") as load_file:
                return float(load_file.read().split()[0])
        except (IOError, OSError, ValueError, IndexError):
            try:
                return float(os.getloadavg()[0])
            except (AttributeError, OSError):
                return None

    @staticmethod
    def _read_thermal_zones():
        zones = {}
        for temp_path in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
            zone_directory = os.path.dirname(temp_path)
            try:
                with open(temp_path, "r") as temp_file:
                    raw_temperature = float(temp_file.read().strip())
                temperature = raw_temperature / 1000.0 if raw_temperature > 200.0 else raw_temperature
                if temperature < -20.0 or temperature > 150.0:
                    continue
                type_path = os.path.join(zone_directory, "type")
                try:
                    with open(type_path, "r") as type_file:
                        name = type_file.read().strip()
                except (IOError, OSError):
                    name = os.path.basename(zone_directory)
                zones[name] = round(temperature, 1)
            except (IOError, OSError, ValueError):
                continue
        return zones

    @staticmethod
    def _disk_percent():
        try:
            usage = shutil.disk_usage("/")
            if usage.total <= 0:
                return None
            return 100.0 * float(usage.used) / float(usage.total)
        except (IOError, OSError):
            return None

    @staticmethod
    def _ip_address():
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            udp_socket.connect(("8.8.8.8", 80))
            return udp_socket.getsockname()[0]
        except OSError:
            return "127.0.0.1"
        finally:
            udp_socket.close()

    def read(self):
        cpu = self._read_cpu_percent()
        memory = self._read_memory_percent()
        uptime = self._read_uptime()
        load = self._read_load()
        zones = self._read_thermal_zones()

        if self.demo:
            elapsed = time.monotonic() - self.started
            if cpu is None:
                cpu = 28.0 + 7.0 * math.sin(elapsed / 5.0)
            if memory is None:
                memory = 41.0
            if uptime is None:
                uptime = elapsed + 14320.0
            if load is None:
                load = 0.42
            if not zones:
                zones = {"CPU-therm": 47.5 + 1.5 * math.sin(elapsed / 7.0)}

        max_zone = None
        max_temperature = None
        if zones:
            max_zone = max(zones, key=lambda key: zones[key])
            max_temperature = zones[max_zone]

        return {
            "hostname": self.hostname,
            "ip_address": self.ip_address,
            "cpu_used_pct": safe_float(cpu, 1),
            "memory_used_pct": safe_float(memory, 1),
            "disk_used_pct": safe_float(self._disk_percent(), 1),
            "uptime_s": safe_float(uptime, 0),
            "load_1m": safe_float(load, 2),
            "temperatures_c": zones,
            "max_temperature_c": safe_float(max_temperature, 1),
            "max_temperature_zone": max_zone,
        }


def find_controller():
    empty_result = {
        "input_device_present": False,
        "accessible": False,
        "configured": False,
        "name": None,
        "path": None,
    }
    try:
        with open("/proc/bus/input/devices", "r") as devices_file:
            blocks = devices_file.read().split("\n\n")
    except (IOError, OSError):
        return empty_result

    for block in blocks:
        lower_block = block.lower()
        if not any(word in lower_block for word in CONTROLLER_WORDS):
            continue

        name = "Game controller"
        event_path = None
        event_capabilities = None
        for line in block.splitlines():
            if line.startswith("N: Name="):
                name = line.split("=", 1)[1].strip().strip('"')
            if line.startswith("H: Handlers="):
                for handler in line.split("=", 1)[1].split():
                    if handler.startswith("event"):
                        event_path = "/dev/input/" + handler
                        break
            if line.startswith("B: EV="):
                try:
                    event_capabilities = int(line.split("=", 1)[1].strip(), 16)
                except ValueError:
                    event_capabilities = None

        has_keys_and_axes = (
            event_capabilities is not None
            and bool(event_capabilities & (1 << 1))
            and bool(event_capabilities & (1 << 3))
        )

        if event_path and os.path.exists(event_path) and has_keys_and_axes:
            accessible = False
            descriptor = None
            try:
                descriptor = os.open(event_path, os.O_RDONLY | os.O_NONBLOCK)
                accessible = True
            except OSError:
                accessible = False
            finally:
                if descriptor is not None:
                    os.close(descriptor)

            return {
                "input_device_present": True,
                "accessible": accessible,
                "configured": False,
                "name": name,
                "path": event_path,
            }

    return empty_result


class DashboardMonitor(threading.Thread):
    def __init__(self, reader, interval, demo=False):
        threading.Thread.__init__(self, name="dashboard-monitor")
        self.daemon = True
        self.reader = reader
        self.interval = max(0.20, float(interval))
        self.demo = bool(demo)
        self.system_metrics = SystemMetrics(demo=demo)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.history = []
        self.events = []
        self.previous_zone = None
        self.previous_controller = None
        self.previous_sensor_ok = None
        self.last_monitor_error = None
        self.created_monotonic = time.monotonic()
        self.latest = self._initial_snapshot()

    def _initial_snapshot(self):
        return {
            "schema_version": SCHEMA_VERSION,
            "timestamp_utc": utc_timestamp(),
            "sample_age_ms": None,
            "sample_monotonic": self.created_monotonic,
            "telemetry_live": False,
            "monitor_mode": True,
            "demo_mode": self.demo,
            "power": {
                "address": "0x{:02X}".format(self.reader.address),
                "voltage_v": None,
                "last_good_voltage_v": None,
                "zone": "INITIALIZING",
                "sensor_ok": False,
                "stale": True,
                "error": None,
            },
            "safety": {
                "overall_state": "INITIALIZING",
                "message": "Waiting for the first read-only telemetry sample.",
                "motion_authorized": False,
                "dashboard_motor_commands_enabled": False,
                "physical_motor_output_state": "UNVERIFIED",
                "enforcement": "NONE_MONITOR_ONLY",
                "interlocks": [
                    "MONITOR_ONLY",
                    "INTEGRATION_VALIDATION_REQUIRED",
                    "CONTROLLER_UNAVAILABLE",
                ],
            },
            "controller": {
                "input_device_present": False,
                "accessible": False,
                "configured": False,
                "name": None,
                "path": None,
            },
            "motors": {
                "left": "PRIOR_CALIBRATION_PASS_ADAPTER_UNVERIFIED",
                "right": "PRIOR_CALIBRATION_PASS_ADAPTER_UNVERIFIED",
                "outputs": "NOT_CONTROLLED_BY_DASHBOARD",
                "physical_output_state": "UNVERIFIED",
            },
            "system": {},
            "readiness": [],
            "events": [],
            "history": [],
        }

    def _add_event(self, level, message):
        event = {
            "timestamp_utc": utc_timestamp(),
            "level": level,
            "message": message,
        }
        self.events.append(event)
        if len(self.events) > 40:
            self.events = self.events[-40:]

    @staticmethod
    def _safety_for_zone(zone):
        if zone == "CRITICAL_STOP":
            return (
                "CRITICAL_STOP_REQUIRED",
                "Battery voltage is below 6.60 V. A motor-control supervisor must perform and latch the stop; this monitor cannot enforce it.",
            )
        if zone == "BATTERY_ABSENT_OR_CRITICAL":
            return (
                "MAINTENANCE_HOLD",
                "The reading is below 1 V. The battery may be absent, deeply discharged, or the measurement path may be faulty; treat it as unsafe.",
            )
        if zone == "SENSOR_FAULT":
            return (
                "TELEMETRY_FAULT_HOLD",
                "INA219 battery telemetry is unavailable. An unknown voltage is never treated as safe.",
            )
        if zone == "MANUAL_RECOVERY":
            return (
                "MAINTENANCE_HOLD",
                "Battery voltage is in the manual-recovery range; motion remains unauthorized because controller input and physical outputs are unverified.",
            )
        return (
            "MAINTENANCE_HOLD",
            "Battery voltage supports autonomous operation, but motion is not authorized until the physical integration gates are completed.",
        )

    @staticmethod
    def _readiness(power, controller, monitor_live=True):
        if power["sensor_ok"]:
            power_level = "good" if power["zone"] == "AUTONOMOUS_CAPABLE" else "warn"
            power_status = power["zone"].replace("_", " ")
        else:
            power_level = "bad"
            power_status = "TELEMETRY FAULT"

        if controller["input_device_present"] and controller["accessible"]:
            controller_status = "INPUT PRESENT / UNVERIFIED"
            controller_level = "warn"
        elif controller["input_device_present"]:
            controller_status = "PRESENT / NO READ ACCESS"
            controller_level = "bad"
        else:
            controller_status = "NOT PRESENT"
            controller_level = "bad"

        return [
            {
                "name": "Dashboard sampler",
                "status": "ONLINE" if monitor_live else "STALE / FAILED",
                "level": "good" if monitor_live else "bad",
            },
            {"name": "INA219 battery sensor", "status": power_status, "level": power_level},
            {"name": "Physical motor outputs", "status": "UNVERIFIED / NO COMMANDS", "level": "warn"},
            {"name": "Left drive motor", "status": "PRIOR CALIBRATION / ADAPTER UNVERIFIED", "level": "warn"},
            {"name": "Right drive motor", "status": "PRIOR CALIBRATION / ADAPTER UNVERIFIED", "level": "warn"},
            {
                "name": "Xbox controller",
                "status": controller_status,
                "level": controller_level,
            },
            {"name": "Autonomous motion", "status": "NOT AUTHORIZED", "level": "bad"},
        ]

    def _sample(self):
        sample_monotonic = time.monotonic()
        voltage = None
        sensor_error = None

        try:
            voltage = float(self.reader.read_voltage())
            if not math.isfinite(voltage) or voltage < 0.0 or voltage > 16.0:
                raise RuntimeError("INA219 returned an implausible voltage")
        except Exception as error:
            sensor_error = str(error)
            self.reader.reset_bus()
            voltage = None

        sensor_ok = voltage is not None
        zone = classify_voltage(voltage)
        controller = find_controller()
        system = self.system_metrics.read()
        overall_state, safety_message = self._safety_for_zone(zone)

        if self.previous_sensor_ok is None:
            if sensor_ok:
                self._add_event("good", "INA219 battery telemetry started.")
            else:
                self._add_event("bad", "INA219 telemetry unavailable: {}".format(sensor_error))
        elif self.previous_sensor_ok != sensor_ok:
            self._add_event(
                "good" if sensor_ok else "bad",
                "INA219 telemetry restored." if sensor_ok else "INA219 telemetry lost: {}".format(sensor_error),
            )

        if zone != self.previous_zone:
            zone_level = {
                "AUTONOMOUS_CAPABLE": "good",
                "MANUAL_RECOVERY": "warn",
                "CRITICAL_STOP": "bad",
                "BATTERY_ABSENT_OR_CRITICAL": "bad",
                "SENSOR_FAULT": "bad",
            }.get(zone, "")
            self._add_event(zone_level, "Battery observation changed to {}.".format(zone.replace("_", " ")))

        controller_present = controller["input_device_present"]
        if self.previous_controller is None or self.previous_controller != controller_present:
            if controller_present:
                self._add_event("warn", "Controller-like input device appeared; connection and mapping remain unverified.")
            elif self.previous_controller is not None:
                self._add_event("warn", "Controller-like input device is no longer present.")

        if sensor_ok:
            self.history.append({
                "timestamp_utc": utc_timestamp(),
                "voltage_v": round(voltage, 3),
            })
            if len(self.history) > HISTORY_LIMIT:
                self.history = self.history[-HISTORY_LIMIT:]

        previous_last_good = self.latest.get("power", {}).get("last_good_voltage_v")
        last_good = round(voltage, 3) if sensor_ok else previous_last_good
        interlocks = ["MONITOR_ONLY", "INTEGRATION_VALIDATION_REQUIRED"]
        if not controller_present:
            interlocks.append("CONTROLLER_UNAVAILABLE")
        elif not controller["accessible"]:
            interlocks.append("CONTROLLER_INPUT_NOT_ACCESSIBLE")
        else:
            interlocks.append("CONTROLLER_INPUT_UNVERIFIED")
        if not sensor_ok:
            interlocks.append("BATTERY_TELEMETRY_UNAVAILABLE")
        elif zone in ("CRITICAL_STOP", "BATTERY_ABSENT_OR_CRITICAL"):
            interlocks.append("BATTERY_CRITICAL_STOP_REQUIRED")

        snapshot = {
            "schema_version": SCHEMA_VERSION,
            "timestamp_utc": utc_timestamp(),
            "sample_monotonic": sample_monotonic,
            "sample_age_ms": 0,
            "telemetry_live": True,
            "monitor_mode": True,
            "demo_mode": self.demo,
            "power": {
                "address": "0x{:02X}".format(self.reader.address),
                "voltage_v": round(voltage, 3) if sensor_ok else None,
                "last_good_voltage_v": last_good,
                "zone": zone,
                "classification_type": "INSTANTANEOUS_OBSERVATION",
                "sensor_ok": sensor_ok,
                "stale": not sensor_ok,
                "error": sensor_error,
                "critical_voltage_v": CRITICAL_VOLTAGE,
                "autonomous_voltage_v": AUTONOMOUS_VOLTAGE,
            },
            "safety": {
                "overall_state": overall_state,
                "message": safety_message,
                "motion_authorized": False,
                "dashboard_motor_commands_enabled": False,
                "physical_motor_output_state": "UNVERIFIED",
                "enforcement": "NONE_MONITOR_ONLY",
                "interlocks": interlocks,
            },
            "controller": controller,
            "motors": {
                "left": "PRIOR_CALIBRATION_PASS_ADAPTER_UNVERIFIED",
                "right": "PRIOR_CALIBRATION_PASS_ADAPTER_UNVERIFIED",
                "outputs": "NOT_CONTROLLED_BY_DASHBOARD",
                "physical_output_state": "UNVERIFIED",
            },
            "system": system,
            "readiness": self._readiness(
                {
                    "sensor_ok": sensor_ok,
                    "zone": zone,
                },
                controller,
                monitor_live=True,
            ),
            "events": list(self.events),
            "history": list(self.history),
        }

        self.previous_sensor_ok = sensor_ok
        self.previous_zone = zone
        self.previous_controller = controller_present

        with self.lock:
            self.latest = snapshot

        if self.last_monitor_error is not None:
            self._add_event("good", "Dashboard sampler recovered from an internal error.")
            self.last_monitor_error = None

    def _publish_monitor_failure(self, error):
        error_text = "{}: {}".format(type(error).__name__, error)
        if error_text != self.last_monitor_error:
            self._add_event("bad", "Dashboard sampler error: {}".format(error_text))
            self.last_monitor_error = error_text

        with self.lock:
            snapshot = copy.deepcopy(self.latest)

        snapshot["timestamp_utc"] = utc_timestamp()
        snapshot["sample_monotonic"] = time.monotonic()
        snapshot["sample_age_ms"] = 0
        snapshot["telemetry_live"] = True
        snapshot["power"]["voltage_v"] = None
        snapshot["power"]["zone"] = "SENSOR_FAULT"
        snapshot["power"]["sensor_ok"] = False
        snapshot["power"]["stale"] = True
        snapshot["power"]["error"] = error_text
        snapshot["safety"]["overall_state"] = "TELEMETRY_FAULT_HOLD"
        snapshot["safety"]["message"] = "The dashboard sampler encountered an internal error. No voltage is being treated as safe."
        snapshot["safety"]["motion_authorized"] = False
        if "DASHBOARD_SAMPLER_ERROR" not in snapshot["safety"]["interlocks"]:
            snapshot["safety"]["interlocks"].append("DASHBOARD_SAMPLER_ERROR")
        snapshot["readiness"] = self._readiness(
            {"sensor_ok": False, "zone": "SENSOR_FAULT"},
            snapshot["controller"],
            monitor_live=True,
        )
        snapshot["events"] = list(self.events)

        with self.lock:
            self.latest = snapshot

    def run(self):
        while not self.stop_event.is_set():
            started = time.monotonic()
            try:
                self._sample()
            except Exception as error:
                self._publish_monitor_failure(error)
            delay = max(0.0, self.interval - (time.monotonic() - started))
            self.stop_event.wait(delay)

    def snapshot(self):
        with self.lock:
            result = copy.deepcopy(self.latest)
        sample_time = result.pop("sample_monotonic", None)
        age = float("inf")
        if sample_time is not None:
            age = max(0.0, time.monotonic() - sample_time)
            result["sample_age_ms"] = int(round(age * 1000.0))
        else:
            result["sample_age_ms"] = None

        thread_live = self.is_alive() and not self.stop_event.is_set()
        telemetry_live = thread_live and age <= STALE_AFTER_SECONDS
        result["telemetry_live"] = telemetry_live
        result["monitor"] = {
            "thread_alive": thread_live,
            "sample_stale": age > STALE_AFTER_SECONDS,
        }

        if not telemetry_live:
            result["power"]["stale"] = True
            result["power"]["sensor_ok"] = False
            result["power"]["voltage_v"] = None
            result["power"]["zone"] = "SENSOR_FAULT"
            result["safety"]["overall_state"] = "TELEMETRY_FAULT_HOLD"
            result["safety"]["message"] = "The dashboard sampler is stale or stopped. An old voltage reading is never treated as safe."
            result["safety"]["motion_authorized"] = False
            if "DASHBOARD_SAMPLER_STALE" not in result["safety"]["interlocks"]:
                result["safety"]["interlocks"].append("DASHBOARD_SAMPLER_STALE")
            result["readiness"] = self._readiness(
                {"sensor_ok": False, "zone": "SENSOR_FAULT"},
                result["controller"],
                monitor_live=False,
            )
        return result

    def stop(self):
        self.stop_event.set()


class CameraMonitor(threading.Thread):
    """Single-producer, latest-frame CSI monitor for read-only MJPEG clients."""

    def __init__(self, sensor_id=0):
        threading.Thread.__init__(self, name="sentinel-csi-camera")
        self.daemon = True
        self.sensor_id = int(sensor_id)
        self.width = DEFAULT_CAMERA_WIDTH
        self.height = DEFAULT_CAMERA_HEIGHT
        self.source_fps = DEFAULT_CAMERA_SOURCE_FPS
        self.stream_fps = DEFAULT_CAMERA_STREAM_FPS
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.condition = threading.Condition()
        self.capture = None
        self.latest_jpeg = None
        self.latest_sequence = 0
        self.last_frame_monotonic = None
        self.frames_encoded = 0
        self.error = None
        self.flip_method = None

    def _record_error(self, error):
        with self.condition:
            self.error = str(error)
            self.condition.notify_all()

    def run(self):
        capture = None
        try:
            try:
                import cv2
            except ImportError:
                raise RuntimeError(
                    "OpenCV is required for dashboard JPEG encoding."
                )

            try:
                from gst_camera_bridge import (
                    DEFAULT_CSI_FLIP_METHOD,
                    GstCamera,
                )
            except Exception as error:
                raise RuntimeError(
                    "CSI bridge could not be loaded: {}".format(error)
                )

            if int(DEFAULT_CSI_FLIP_METHOD) != 2:
                raise RuntimeError(
                    "Shared CSI orientation is not flip-method=2."
                )
            self.flip_method = int(DEFAULT_CSI_FLIP_METHOD)
            capture = GstCamera(
                sensor_id=self.sensor_id,
                capture_width=1280,
                capture_height=720,
                output_width=self.width,
                output_height=self.height,
                framerate=self.source_fps,
                flip_method=DEFAULT_CSI_FLIP_METHOD,
            )
            self.capture = capture

            minimum_interval = 1.0 / float(self.stream_fps)
            next_encode = 0.0
            jpeg_options = [
                int(getattr(cv2, "IMWRITE_JPEG_QUALITY", 1)),
                int(DEFAULT_CAMERA_JPEG_QUALITY),
            ]

            while not self.stop_event.is_set():
                success, frame, metadata = capture.read_with_metadata(
                    timeout_seconds=1.0
                )
                if not success or frame is None:
                    continue
                if metadata and metadata.get("fresh") is False:
                    continue

                now = time.monotonic()
                if now < next_encode:
                    continue
                encoded_ok, encoded = cv2.imencode(
                    ".jpg", frame, jpeg_options
                )
                if not encoded_ok:
                    raise RuntimeError("OpenCV could not encode a camera frame.")

                capture_time = None
                if metadata:
                    capture_time = metadata.get("capture_monotonic")
                if capture_time is None:
                    capture_time = now

                with self.condition:
                    self.latest_jpeg = encoded.tobytes()
                    self.latest_sequence += 1
                    self.frames_encoded += 1
                    self.last_frame_monotonic = float(capture_time)
                    self.ready_event.set()
                    self.condition.notify_all()
                next_encode = now + minimum_interval

        except Exception as error:
            self._record_error(error)
        finally:
            if capture is not None:
                try:
                    capture.close()
                except Exception as error:
                    if self.error is None:
                        self._record_error(
                            "Camera shutdown failed: {}".format(error)
                        )
            self.capture = None
            self.ready_event.set()
            with self.condition:
                self.condition.notify_all()

    def start_and_wait(self, timeout_seconds=12.0):
        self.start()
        if not self.ready_event.wait(float(timeout_seconds)):
            self.stop()
            raise RuntimeError(
                "Timed out waiting for the first CSI camera frame."
            )
        status = self.status()
        if status["error"]:
            raise RuntimeError(status["error"])
        if status["frame_sequence"] <= 0:
            raise RuntimeError("CSI camera stopped before producing a frame.")

    def wait_for_frame(self, after_sequence, timeout_seconds=2.0):
        deadline = time.monotonic() + float(timeout_seconds)
        with self.condition:
            while (
                self.latest_sequence <= int(after_sequence)
                and not self.stop_event.is_set()
                and self.error is None
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.condition.wait(remaining)
            if self.latest_sequence <= int(after_sequence):
                return None, None
            return self.latest_sequence, self.latest_jpeg

    def status(self):
        with self.condition:
            sequence = int(self.latest_sequence)
            frame_time = self.last_frame_monotonic
            error = self.error
            frames_encoded = int(self.frames_encoded)
        frame_age_ms = None
        if frame_time is not None:
            frame_age_ms = max(
                0, int(round((time.monotonic() - frame_time) * 1000.0))
            )
        live = bool(
            self.is_alive()
            and not self.stop_event.is_set()
            and error is None
            and frame_age_ms is not None
            and frame_age_ms <= int(CAMERA_STALE_AFTER_SECONDS * 1000.0)
        )
        return {
            "enabled": True,
            "mode": "CSI",
            "live": live,
            "sensor_id": self.sensor_id,
            "width": self.width,
            "height": self.height,
            "source_fps": self.source_fps,
            "stream_fps_limit": self.stream_fps,
            "jpeg_quality": DEFAULT_CAMERA_JPEG_QUALITY,
            "flip_method": self.flip_method,
            "frame_sequence": sequence,
            "frames_encoded": frames_encoded,
            "frame_age_ms": frame_age_ms,
            "error": error,
            "read_only": True,
            "recording": False,
        }

    def stop(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()


class DashboardRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def setup(self):
        BaseHTTPRequestHandler.setup(self)
        self.connection.settimeout(5.0)

    def log_message(self, format_string, *arguments):
        sys.stdout.write("{} - {}\n".format(self.address_string(), format_string % arguments))
        sys.stdout.flush()

    def _send_bytes(self, status, content_type, payload, head_only=False):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Connection", "close")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; style-src 'self' 'unsafe-inline'; "
            "script-src 'self' 'unsafe-inline'; connect-src 'self'; "
            "img-src 'self' data:; frame-ancestors 'none'",
        )
        self.end_headers()
        self.close_connection = True
        if not head_only:
            self.wfile.write(payload)

    def _send_json(self, status, value, head_only=False):
        payload = json.dumps(value, separators=(",", ":"), sort_keys=True).encode("utf-8")
        self._send_bytes(status, "application/json; charset=utf-8", payload, head_only)

    def _camera_status(self):
        camera = getattr(self.server, "camera_monitor", None)
        if camera is None:
            return {
                "enabled": False,
                "mode": "OFF",
                "live": False,
                "sensor_id": None,
                "width": None,
                "height": None,
                "frame_age_ms": None,
                "error": None,
                "read_only": True,
                "recording": False,
            }
        return camera.status()

    def _status_snapshot(self):
        status_snapshot = self.server.monitor.snapshot()
        status_snapshot["camera"] = self._camera_status()
        return status_snapshot

    def _send_camera_stream(self, head_only=False):
        camera = getattr(self.server, "camera_monitor", None)
        if camera is None:
            self._send_json(
                404,
                {"error": "camera_disabled", "read_only": True},
                head_only,
            )
            return
        camera_status = camera.status()
        if not camera_status["live"] and camera_status["frame_sequence"] <= 0:
            self._send_json(
                503,
                {
                    "error": "camera_unavailable",
                    "detail": camera_status.get("error"),
                    "read_only": True,
                },
                head_only,
            )
            return

        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary={}".format(
                CAMERA_STREAM_BOUNDARY
            ),
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        if head_only:
            self.close_connection = True
            return

        sequence = -1
        try:
            while not camera.stop_event.is_set():
                next_sequence, jpeg = camera.wait_for_frame(
                    sequence,
                    timeout_seconds=2.0,
                )
                if jpeg is None:
                    status = camera.status()
                    if status["error"] or not camera.is_alive():
                        break
                    continue
                sequence = next_sequence
                part_header = (
                    "--{boundary}\r\n"
                    "Content-Type: image/jpeg\r\n"
                    "Content-Length: {length}\r\n"
                    "X-Frame-Sequence: {sequence}\r\n\r\n"
                ).format(
                    boundary=CAMERA_STREAM_BOUNDARY,
                    length=len(jpeg),
                    sequence=sequence,
                ).encode("ascii")
                self.wfile.write(part_header)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (IOError, OSError, socket.timeout):
            pass
        finally:
            self.close_connection = True

    def _route(self, head_only=False):
        path = self.path.split("?", 1)[0]

        if path == "/":
            self._send_bytes(
                200,
                "text/html; charset=utf-8",
                DASHBOARD_HTML.encode("utf-8"),
                head_only,
            )
            return

        if path == "/api/status":
            self._send_json(200, self._status_snapshot(), head_only)
            return

        if path == "/api/camera/status":
            self._send_json(200, self._camera_status(), head_only)
            return

        if path == "/camera.mjpg":
            self._send_camera_stream(head_only)
            return

        if path in ("/api/health", "/healthz"):
            status_snapshot = self._status_snapshot()
            sampler_live = bool(status_snapshot.get("telemetry_live"))
            sensor_ok = bool(status_snapshot["power"]["sensor_ok"])
            telemetry_ready = sampler_live and sensor_ok
            self._send_json(
                200 if telemetry_ready else 503,
                {
                    "ok": telemetry_ready,
                    "status": "ready" if telemetry_ready else ("degraded" if sampler_live else "stale"),
                    "mode": "MONITOR_ONLY",
                    "sampler_live": sampler_live,
                    "sensor_ok": sensor_ok,
                    "telemetry_ready": telemetry_ready,
                    "sample_age_ms": status_snapshot.get("sample_age_ms"),
                    "motion_authorized": False,
                    "camera": status_snapshot["camera"],
                    "timestamp_utc": utc_timestamp(),
                },
                head_only,
            )
            return

        if path == "/favicon.ico":
            self._send_bytes(204, "image/x-icon", b"", head_only)
            return

        self._send_json(404, {"error": "not_found"}, head_only)

    def do_GET(self):
        self._route(False)

    def do_HEAD(self):
        self._route(True)

    def _method_not_allowed(self):
        self.send_response(405)
        self.send_header("Allow", "GET, HEAD")
        payload = b'{"error":"read_only_dashboard"}'
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Connection", "close")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.close_connection = True
        self.wfile.write(payload)

    do_POST = _method_not_allowed
    do_PUT = _method_not_allowed
    do_PATCH = _method_not_allowed
    do_DELETE = _method_not_allowed


class ThreadedDashboardServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def positive_port(value):
    port = int(value)
    if port < 1 or port > 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return port


def sample_interval(value):
    interval = float(value)
    if not math.isfinite(interval) or interval < 0.20 or interval > 1.0:
        raise argparse.ArgumentTypeError("interval must be between 0.20 and 1.00 seconds")
    return interval


def parse_address(value):
    try:
        address = int(value, 0)
    except ValueError:
        raise argparse.ArgumentTypeError("I2C address must be decimal or hexadecimal")
    if address < 0x03 or address > 0x77:
        raise argparse.ArgumentTypeError("I2C address must be in the 7-bit device range")
    return address


def nonnegative_integer(value):
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("value must be zero or greater")
    return number


def loopback_bind_host(value):
    text = str(value).strip().lower()
    if text == "localhost":
        return True
    try:
        return bool(ipaddress.ip_address(text).is_loopback)
    except ValueError:
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Read-only Jetson robot monitoring dashboard",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Listen address. Use 0.0.0.0 only on a trusted LAN.",
    )
    parser.add_argument("--port", type=positive_port, default=8080)
    parser.add_argument("--i2c-bus", type=int, default=DEFAULT_I2C_BUS)
    parser.add_argument("--ina-address", type=parse_address, default=DEFAULT_INA219_ADDRESS)
    parser.add_argument("--interval", type=sample_interval, default=DEFAULT_SAMPLE_INTERVAL)
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Use synthetic battery telemetry without I2C hardware.",
    )
    parser.add_argument(
        "--camera",
        choices=("off", "csi"),
        default="off",
        help="Optional read-only camera monitor (default: off).",
    )
    parser.add_argument(
        "--camera-source",
        type=nonnegative_integer,
        default=0,
        help="CSI sensor-id used when --camera csi is selected (default: 0).",
    )
    parser.add_argument(
        "--allow-lan-camera",
        action="store_true",
        help=(
            "Acknowledge that the unauthenticated read-only camera feed will "
            "be visible on the trusted LAN."
        ),
    )
    arguments = parser.parse_args()

    if (
        arguments.camera == "csi"
        and not loopback_bind_host(arguments.host)
        and not arguments.allow_lan_camera
    ):
        parser.error(
            "--allow-lan-camera is required when CSI video is bound to a "
            "non-loopback address"
        )

    def stop_signal(signum, frame):
        del signum, frame
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGINT, stop_signal)

    reader = Ina219VoltageReader(
        arguments.i2c_bus,
        arguments.ina_address,
        demo=arguments.demo,
    )
    monitor = DashboardMonitor(reader, arguments.interval, demo=arguments.demo)
    camera_monitor = None

    try:
        server = ThreadedDashboardServer(
            (arguments.host, arguments.port),
            DashboardRequestHandler,
        )
    except OSError as error:
        print("Dashboard could not start: {}".format(error))
        return 1

    server.monitor = monitor
    server.camera_monitor = None

    if arguments.camera == "csi":
        camera_monitor = CameraMonitor(arguments.camera_source)
        try:
            camera_monitor.start_and_wait(timeout_seconds=12.0)
        except KeyboardInterrupt:
            camera_monitor.stop()
            camera_monitor.join(timeout=4.0)
            server.server_close()
            reader.close()
            print("\nDashboard camera startup interrupted.")
            return 130
        except Exception as error:
            camera_monitor.stop()
            camera_monitor.join(timeout=4.0)
            server.server_close()
            reader.close()
            print("Dashboard camera could not start: {}".format(error))
            print(
                "Check OpenCV/GI availability, stop every other CSI process, "
                "and inspect the ribbon cable."
            )
            return 1
        server.camera_monitor = camera_monitor

    monitor.start()

    display_host = arguments.host
    if display_host == "0.0.0.0":
        display_host = SystemMetrics._ip_address()

    print("=" * 58)
    print("{} ROBOT DASHBOARD".format(APP_NAME.upper()))
    print("=" * 58)
    print("URL:            http://{}:{}".format(display_host, arguments.port))
    print("Mode:           {}".format("DEMO" if arguments.demo else "MONITOR ONLY"))
    print("INA219:         bus {} address 0x{:02X}".format(arguments.i2c_bus, arguments.ina_address))
    if camera_monitor is None:
        print("Camera:         OFF")
    else:
        print(
            "Camera:         CSI {} at {}x{} / flip-method={} / read-only".format(
                arguments.camera_source,
                DEFAULT_CAMERA_WIDTH,
                DEFAULT_CAMERA_HEIGHT,
                camera_monitor.status()["flip_method"],
            )
        )
    print("Motor commands: NONE (read-only dashboard)")
    print("Physical output state: UNVERIFIED - keep motor power disconnected")
    print("Motion control: NOT IMPLEMENTED")
    if arguments.host == "0.0.0.0":
        print("Network note:   exposed to the trusted local network")
    print("Ctrl+C stops the dashboard.")
    print("=" * 58)

    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nStopping dashboard...")
    finally:
        server.server_close()
        if camera_monitor is not None:
            camera_monitor.stop()
            camera_monitor.join(timeout=4.0)
            if camera_monitor.is_alive():
                print("Warning: camera worker did not stop within 4 seconds.")
        monitor.stop()
        monitor.join(timeout=max(3.0, arguments.interval * 4.0))
        if monitor.is_alive():
            print("Warning: sampler did not stop; leaving I2C handle untouched.")
        else:
            reader.close()

    print("Dashboard stopped. It sent no motor commands.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
