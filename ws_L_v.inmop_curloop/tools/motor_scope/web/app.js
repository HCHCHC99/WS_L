"use strict";

/* ================= 常量 ================= */
const POLL_MS = 50;
const POLE_PAIRS = 10;          // 与 motor_config.h FOC_POLE_PAIRS 一致
const DEG = Math.PI / 180;
const TAU = Math.PI * 2;
const RAD2DEG = 180 / Math.PI;

const PHASE_LIST = [
  { id: 0, name: "idle",     color: "#6b7280" },
  { id: 1, name: "hold",     color: "#f59e0b" },
  { id: 2, name: "ramp/sync", color: "#60a5fa" },
  { id: 3, name: "run",      color: "#22c55e" },
  { id: 4, name: "align",    color: "#a855f7" },
];
const MODE_NAMES = { 0: "停止", 1: "开环", 2: "电流环", 3: "对齐" };

/* 电机剖面几何（画布 560x560，中心 280,280） */
const MW = 560, MH = 560;
const R_SY = 215, R_ST = 168, R_R = 158, R_M = 118, R_SH = 24;

const hub = {
  latest: null, lastSeq: 0,
  status: "connecting", detail: "--", fps: 0,
  rpmMax: 500, curMax: 1000,
  curHist: [],          // {t, iq, id, rotorDeg, thetaDeg, diffRad}
  logs: [],
};

const motorCv = document.getElementById("motor");
const mctx = motorCv.getContext("2d");
const gaugeCv = document.getElementById("rpmGauge");
const gctx = gaugeCv.getContext("2d");
const scope1Cv = document.getElementById("scope1");
const scope2Cv = document.getElementById("scope2");
const scope3Cv = document.getElementById("scope3");

/* ================= 数据轮询 ================= */
async function poll() {
  try {
    const r = await fetch(`/data?since=${hub.lastSeq}&_=${Date.now()}`);
    const d = await r.json();
    hub.status = d.status; hub.detail = d.detail || "--"; hub.fps = d.fps || 0;
    if (d.latest) {
      hub.latest = {
        mode: d.latest[0], phase: d.latest[1], rotor: d.latest[2],
        theta: d.latest[3], iq: d.latest[4], id: d.latest[5],
        vq: d.latest[6], vd: d.latest[7], spd: d.latest[8],
        sync: d.latest[9], diff: d.latest[10], freq: d.latest[11],
        ms: d.latest[12],
      };
      hub.rpmMax = Math.max(hub.rpmMax, Math.abs(hub.latest.spd) * 1.25);
      hub.curMax = Math.max(hub.curMax, Math.abs(hub.latest.iq), Math.abs(hub.latest.id));
    }
    for (const h of d.history) {
      const f = h[2];
      hub.curHist.push({
        t: h[1],
        iq: f[4], id: f[5],
        rotorDeg: (f[2] / 1000) * RAD2DEG,        // 电角度 °
        thetaDeg: (f[3] / 1000) * RAD2DEG,
        diffRad: f[10] / 1000,
      });
      if (hub.lastSeq < h[0]) {                  // 用最新一帧锚定显示角度
        anchorFromFrame(f);
      }
    }
    if (hub.curHist.length) {
      const tEnd = hub.curHist[hub.curHist.length - 1].t;
      while (hub.curHist.length && hub.curHist[0].t < tEnd - 2.0) hub.curHist.shift();
    }
    if (d.logs && d.logs.length) { hub.logs = d.logs; renderLog(); }
    if (d.latest && d.seq > hub.lastSeq) hub.lastSeq = d.seq;
    updateStatus();
  } catch (e) { hub.status = "error"; updateStatus(); }
}
setInterval(poll, POLL_MS);

/* ================= 连接健康检查 / 重连 ================= */
async function health() {
  try {
    const r = await fetch("/health?_=" + Date.now());
    const d = await r.json();
    const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
    let stText = d.status;
    if (d.status === "running") stText += " ✅";
    else if (d.status === "error") stText += " ❌";
    else stText += " ⏳";
    set("cStatus", stText);
    set("cDetail", d.detail || "--");
    set("cChannel", d.channel !== null && d.channel !== undefined ? String(d.channel) : "--");
    set("cFrames", d.frames);
    set("cAge", d.last_age_ms < 0 ? "--" : d.last_age_ms.toFixed(0) + " ms");
    set("cFps", d.fps.toFixed(0) + " fps");
    const dot = document.getElementById("connDot");
    dot.className = "dot " + (d.status === "running" ? "good" : d.status === "error" ? "bad" : "");
    document.getElementById("connText").textContent =
      d.status === "running" ? "实时连接" : d.status === "error" ? "连接异常" : "连接中…";
    const hint = document.getElementById("cHint");
    if (d.status === "running" && d.last_age_ms >= 0) {
      hint.textContent = "收到 RTT 帧，动画应已更新。若画面不动，检查目标是否在 mode 22 运行。";
      hint.style.color = "#4ade80";
    } else if (d.status === "running") {
      hint.textContent = "已连接但尚未收到帧：确认目标在运行、固件含 MOTF 发送（通道 " +
        (d.channel ?? "?") + "）。";
      hint.style.color = "#f59e0b";
    } else if (d.status === "error") {
      hint.textContent = "连接异常：" + (d.detail || "");
      hint.style.color = "#ef4444";
    } else {
      hint.textContent = "正在连接 J-Link / 搜索 RTT 控制块…";
      hint.style.color = "#f59e0b";
    }
  } catch (e) { /* 页面刚打开或服务器重启，忽略 */ }
}
setInterval(health, 1000);

async function doReconnect(ch) {
  const url = "/reconnect" + (ch !== undefined && ch !== null ? "?channel=" + ch : "");
  try { await fetch(url); } catch (e) {}
  hub.lastSeq = 0;        // 重连后 seq 可能重置，重新对齐
  hub.curHist = [];
  hub.latest = null;
  health();
}

document.getElementById("chApply").addEventListener("click", () => {
  const ch = parseInt(document.getElementById("chInput").value, 10);
  if (isNaN(ch)) { return; }
  doReconnect(ch);
});
document.getElementById("reconnBtn").addEventListener("click", () => {
  const raw = document.getElementById("chInput").value;
  const ch = parseInt(raw, 10);
  doReconnect(isNaN(ch) ? undefined : ch);
});

function anchorFromFrame(f) {
  visRotorMech = ((f[2] / 1000) * RAD2DEG) / POLE_PAIRS;   // 转子机械角
  visCtrlElec = (f[3] / 1000) * RAD2DEG;                   // 控制电角度
  visRotorElec = (f[2] / 1000) * RAD2DEG;
}

/* ================= 状态栏 ================= */
function updateStatus() {
  const dot = document.getElementById("connDot");
  const txt = document.getElementById("connText");
  if (hub.status === "running") { dot.className = "dot good"; txt.textContent = "实时连接"; }
  else if (hub.status === "error") { dot.className = "dot bad"; txt.textContent = "连接异常"; }
  else { dot.className = "dot"; txt.textContent = "连接中…"; }
  const f = hub.latest;
  document.getElementById("modePill").textContent = f ? ("mode " + f.mode + " " + (MODE_NAMES[f.mode] || "")) : "--";
  document.getElementById("phasePill").textContent = f ? ("phase " + f.phase + " " + (f.phase <= 4 ? PHASE_LIST[f.phase].name : "")) : "--";
  const sp = document.getElementById("syncPill");
  if (f) { sp.textContent = f.sync ? "已同步" : "未同步"; sp.style.color = f.sync ? "#4ade80" : "#f59e0b"; }
  document.getElementById("srcPill").textContent = hub.detail;
  document.getElementById("fpsPill").textContent = hub.fps.toFixed(0) + " fps";
}

/* ================= 显示角度（帧间按转速/频率积分，动画平滑） ================= */
let visRotorMech = 0, visRotorElec = 0, visCtrlElec = 0, lastNow = performance.now();
function advance(now) {
  const dt = (now - lastNow) / 1000; lastNow = now;
  const f = hub.latest;
  if (!f) return;
  // 转子按机械转速积分（mech rev/s = rpm/60）
  visRotorMech = (visRotorMech + (f.spd / 60) * 360 * dt + 360) % 360;
  // 转子电角度 = 机械角 x 极对数（锚定来自最新帧，帧间按转速积分平滑）
  visRotorElec = visRotorMech * POLE_PAIRS;
  // 控制角按电频率积分
  visCtrlElec = (visCtrlElec + (f.freq / 100) * 360 * dt + 360) % 360;
}

/* ================= 电机剖视图 ================= */
function drawArrow(ctx, x, y, ang, size) {
  ctx.save(); ctx.translate(x, y); ctx.rotate(ang);
  ctx.beginPath(); ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.55, size * 0.45); ctx.lineTo(-size * 0.55, -size * 0.45);
  ctx.closePath(); ctx.fill(); ctx.restore();
}

function drawVector(ctx, color, len, angDeg, width, dash) {
  if (len < 2) return;
  ctx.save();
  ctx.strokeStyle = color; ctx.fillStyle = color;
  ctx.lineWidth = width; ctx.lineCap = "round";
  if (dash) ctx.setLineDash(dash);
  ctx.beginPath(); ctx.moveTo(0, 0);
  ctx.lineTo(Math.cos(angDeg * DEG) * len, Math.sin(angDeg * DEG) * len);
  ctx.stroke();
  if (dash) ctx.setLineDash([]);
  drawArrow(ctx, Math.cos(angDeg * DEG) * len, Math.sin(angDeg * DEG) * len, angDeg * DEG, 9);
  ctx.restore();
}

function drawMotor() {
  const ctx = mctx;
  ctx.clearRect(0, 0, MW, MH);
  const cx = MW / 2, cy = MH / 2;
  const f = hub.latest;

  // 背景
  const bg = ctx.createRadialGradient(cx, cy, 60, cx, cy, MW * 0.72);
  bg.addColorStop(0, "#1c232b"); bg.addColorStop(1, "#0c1014");
  ctx.fillStyle = bg; ctx.fillRect(0, 0, MW, MH);

  ctx.save(); ctx.translate(cx, cy);

  // 定子（示意：12 槽）
  ctx.fillStyle = "#333b44";
  ctx.beginPath(); ctx.arc(0, 0, R_SY, 0, TAU); ctx.fill();
  ctx.strokeStyle = "#454e59"; ctx.lineWidth = 2; ctx.stroke();
  ctx.fillStyle = "#0c1014";
  ctx.beginPath(); ctx.arc(0, 0, R_ST - 2, 0, TAU); ctx.fill();
  for (let k = 0; k < 12; k++) {
    ctx.save(); ctx.rotate(k * 30 * DEG);
    ctx.fillStyle = "#4c5560"; ctx.strokeStyle = "#5c6672"; ctx.lineWidth = 1.2;
    ctx.fillRect(-10, R_ST, 20, R_SY - R_ST);
    ctx.strokeRect(-10, R_ST, 20, R_SY - R_ST);
    ctx.restore();
  }
  // 机械角刻度
  ctx.font = "11px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  for (let k = 0; k < 12; k++) {
    const a = k * 30 * DEG;
    ctx.strokeStyle = "rgba(255,255,255,0.22)"; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(Math.cos(a) * (R_SY + 5), Math.sin(a) * (R_SY + 5));
    ctx.lineTo(Math.cos(a) * (R_SY + 14), Math.sin(a) * (R_SY + 14));
    ctx.stroke();
    ctx.fillStyle = "rgba(255,255,255,0.5)";
    ctx.fillText(String(k * 30), Math.cos(a) * (R_SY + 28), Math.sin(a) * (R_SY + 28));
  }

  // 转子体
  ctx.fillStyle = "#22272e";
  ctx.beginPath(); ctx.arc(0, 0, R_R, 0, TAU); ctx.fill();
  ctx.strokeStyle = "#39404a"; ctx.lineWidth = 2; ctx.stroke();

  // 20 块磁钢（10 对极），d 轴 = 转子 N 极中心
  const poleDeg = 180 / POLE_PAIRS;
  for (let k = 0; k < 2 * POLE_PAIRS; k++) {
    const a0 = (visRotorMech + k * poleDeg - poleDeg / 2) * DEG;
    const a1 = (visRotorMech + k * poleDeg + poleDeg / 2) * DEG;
    ctx.beginPath();
    ctx.arc(0, 0, R_R, a0, a1);
    ctx.arc(0, 0, R_M, a1, a0, true);
    ctx.closePath();
    ctx.fillStyle = (k % 2 === 0) ? "#e5484d" : "#3b82f6";
    ctx.fill();
    ctx.strokeStyle = "rgba(0,0,0,0.35)"; ctx.lineWidth = 1; ctx.stroke();
  }
  // N 标记
  ctx.fillStyle = "#fff"; ctx.font = "bold 13px sans-serif";
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  ctx.fillText("N", Math.cos(visRotorMech * DEG) * (R_R + R_M) / 2,
               Math.sin(visRotorMech * DEG) * (R_R + R_M) / 2);

  // d/q 轴
  const qAng = visRotorMech + 90 / POLE_PAIRS;
  ctx.strokeStyle = "rgba(255,255,255,0.30)"; ctx.setLineDash([3, 5]); ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(Math.cos(visRotorMech * DEG) * (R_R - 4), Math.sin(visRotorMech * DEG) * (R_R - 4)); ctx.stroke();
  ctx.strokeStyle = "rgba(255,255,255,0.16)";
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(Math.cos(qAng * DEG) * (R_R - 4), Math.sin(qAng * DEG) * (R_R - 4)); ctx.stroke();
  ctx.setLineDash([]);

  // 控制角 θ（I-F 合成角 / RUN 控制角）——白色虚线指针
  const ctrlMech = visCtrlElec / POLE_PAIRS;
  ctx.strokeStyle = "#ffffff"; ctx.setLineDash([6, 4]); ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(Math.cos(ctrlMech * DEG) * (R_R - 10), Math.sin(ctrlMech * DEG) * (R_R - 10)); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#ffffff"; ctx.font = "11px Consolas, monospace";
  ctx.fillText("θ", Math.cos(ctrlMech * DEG) * (R_R - 24), Math.sin(ctrlMech * DEG) * (R_R - 24));

  // 电流矢量：id（d 轴）/ iq（q 轴）/ is（合成）
  if (f) {
    const maxA = Math.max(500, hub.curMax * 1.1);
    const Lmax = R_R - 34;
    const idL = f.id / maxA * Lmax;
    const iqL = f.iq / maxA * Lmax;
    const idX = idL * Math.cos(visRotorMech * DEG), idY = idL * Math.sin(visRotorMech * DEG);
    const iqX = iqL * Math.cos(qAng * DEG),       iqY = iqL * Math.sin(qAng * DEG);
    const isL = Math.hypot(idL, iqL);
    const isAng = Math.atan2(idY + iqY, idX + iqX) / DEG;
    drawVector(ctx, "#22d3ee", idL, visRotorMech, 4);          // id
    drawVector(ctx, "#fb923c", iqL, qAng, 4);                  // iq
    drawVector(ctx, "#facc15", isL, isAng, 5);                 // is
    // 电压矢量（vd/vq）
    const vLmax = 100;
    const vdL = f.vd / 1000 * vLmax, vqL = f.vq / 1000 * vLmax;
    const vx = vdL * Math.cos(visRotorMech * DEG) + vqL * Math.cos(qAng * DEG);
    const vy = vdL * Math.sin(visRotorMech * DEG) + vqL * Math.sin(qAng * DEG);
    const vL = Math.hypot(vx, vy);
    if (vL > 3) drawVector(ctx, "#e879f9", vL, Math.atan2(vy, vx) / DEG, 3, [4, 4]);
    // 标注
    ctx.font = "bold 13px Consolas, monospace";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillStyle = "#facc15";
    ctx.fillText("is", Math.cos(isAng * DEG) * (isL + 18), Math.sin(isAng * DEG) * (isL + 18));
  }

  // 轴
  ctx.fillStyle = "#0b0d10";
  ctx.beginPath(); ctx.arc(0, 0, R_SH, 0, TAU); ctx.fill();
  ctx.strokeStyle = "#2c333c"; ctx.lineWidth = 2; ctx.stroke();

  // 底部读数
  if (f) {
    ctx.font = "13px Consolas, monospace"; ctx.fillStyle = "#9aa5b1";
    ctx.textAlign = "center"; ctx.textBaseline = "alphabetic";
    ctx.fillText(
      `θe转子=${(visRotorElec % 360).toFixed(0)}°  θe控制=${(visCtrlElec % 360).toFixed(0)}°  ` +
      `iq=${f.iq.toFixed(0)}mA  id=${f.id.toFixed(0)}mA  n=${f.spd.toFixed(0)}rpm`,
      0, MH - 14);
  }
  ctx.restore();
}

/* ================= 转速表 ================= */
function drawGauge() {
  const ctx = gctx;
  const W = gaugeCv.width, H = gaugeCv.height;
  ctx.clearRect(0, 0, W, H);
  const f = hub.latest;
  const rpm = f ? Math.abs(f.spd) : 0;
  const max = Math.max(100, hub.rpmMax);
  const cx = W / 2, cy = H - 14, R = Math.min(W / 2, H) - 22;
  const a0 = Math.PI, a1 = 2 * Math.PI;
  const frac = Math.min(1, rpm / max);
  ctx.fillStyle = "#0d1217";
  ctx.beginPath(); ctx.arc(cx, cy, R + 18, 0, TAU); ctx.fill();
  ctx.strokeStyle = "#1e2833"; ctx.stroke();
  ctx.strokeStyle = "#2a313a"; ctx.lineWidth = 13; ctx.lineCap = "round";
  ctx.beginPath(); ctx.arc(cx, cy, R, a0, a1); ctx.stroke();
  ctx.font = "9px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  ctx.fillStyle = "#5b6672";
  for (let v = 0; v <= 10; v++) {
    const a = a0 + (a1 - a0) * (v / 10);
    ctx.strokeStyle = "#3a4350"; ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(a) * (R - 8), cy + Math.sin(a) * (R - 8));
    ctx.lineTo(cx + Math.cos(a) * (R + 8), cy + Math.sin(a) * (R + 8));
    ctx.stroke();
    ctx.fillText(String(Math.round(max * v / 10)), cx + Math.cos(a) * (R + 20), cy + Math.sin(a) * (R + 20));
  }
  const grad = ctx.createLinearGradient(0, 0, W, 0);
  grad.addColorStop(0, "#22c55e"); grad.addColorStop(0.6, "#facc15"); grad.addColorStop(1, "#ef4444");
  ctx.strokeStyle = grad; ctx.lineWidth = 13;
  ctx.beginPath(); ctx.arc(cx, cy, R, a0, a0 + (a1 - a0) * frac); ctx.stroke();
  const na = a0 + (a1 - a0) * frac;
  ctx.strokeStyle = "#fff"; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(cx + Math.cos(na) * 10, cy + Math.sin(na) * 10);
  ctx.lineTo(cx + Math.cos(na) * (R - 22), cy + Math.sin(na) * (R - 22)); ctx.stroke();
  ctx.fillStyle = "#fff"; ctx.font = "bold 30px Consolas, monospace"; ctx.textAlign = "center";
  ctx.fillText(rpm.toFixed(0), cx, cy - 36);
  ctx.fillStyle = "#8fa0b0"; ctx.font = "12px sans-serif"; ctx.fillText("rpm", cx, cy - 16);
  ctx.fillStyle = "#3f4a56"; ctx.font = "9px sans-serif";
  ctx.fillText("量程 " + max.toFixed(0), cx, cy + R + 16);
}

/* ================= 数值面板 ================= */
function buildNumGrid() {
  const rows = [
    ["转速", "rpmVal", "0 rpm"], ["转子电角 θe", "rotorVal", "0°"],
    ["控制角 θ", "ctrlVal", "0°"], ["iq", "iqVal", "0 mA"],
    ["id", "idVal", "0 mA"], ["vq", "vqVal", "0 mV"],
    ["vd", "vdVal", "0 mV"], ["电频率", "freqVal", "0 Hz"],
    ["角度偏差 diff", "diffVal", "0 rad"],
  ];
  const grid = document.getElementById("numGrid");
  grid.innerHTML = "";
  for (const [k, id, def] of rows) {
    const kd = document.createElement("div"); kd.className = "k"; kd.textContent = k;
    const vd = document.createElement("div"); vd.className = "v"; vd.id = id; vd.textContent = def;
    grid.appendChild(kd); grid.appendChild(vd);
  }
}
function updateNum() {
  const f = hub.latest;
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
  if (!f) return;
  set("rpmVal", f.spd.toFixed(0) + " rpm");
  set("rotorVal", ((f.rotor / 1000) * RAD2DEG % 360).toFixed(1) + "°");
  set("ctrlVal", ((f.theta / 1000) * RAD2DEG % 360).toFixed(1) + "°");
  set("iqVal", f.iq.toFixed(0) + " mA");
  set("idVal", f.id.toFixed(0) + " mA");
  set("vqVal", f.vq.toFixed(0) + " mV");
  set("vdVal", f.vd.toFixed(0) + " mV");
  set("freqVal", (f.freq / 100).toFixed(2) + " Hz");
  set("diffVal", (f.diff / 1000).toFixed(3) + " rad");
}

/* ================= I-F 状态机 ================= */
function buildPhases() {
  const row = document.getElementById("phaseRow");
  row.innerHTML = "";
  for (const p of PHASE_LIST) {
    const chip = document.createElement("div");
    chip.className = "phase-chip"; chip.id = "chip" + p.id;
    chip.textContent = p.name;
    row.appendChild(chip);
  }
}
function updatePhases() {
  const f = hub.latest;
  for (const p of PHASE_LIST) {
    const chip = document.getElementById("chip" + p.id);
    if (!chip) continue;
    if (f && f.phase === p.id) {
      chip.className = "phase-chip on";
      chip.style.background = p.color;
    } else {
      chip.className = "phase-chip";
      chip.style.background = "";
    }
  }
  const led = document.getElementById("syncLed");
  const txt = document.getElementById("syncText");
  if (f && f.sync) { led.className = "led on"; txt.textContent = "已同步（handover 完成，切入编码器角）"; txt.style.color = "#4ade80"; }
  else { led.className = "led"; txt.textContent = "未同步（I-F 启动中）"; txt.style.color = "#f59e0b"; }
}

/* ================= 波形 ================= */
function drawScope(cv, kind) {
  const ctx = cv.getContext("2d");
  const W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#10151a"; ctx.fillRect(0, 0, W, H);
  ctx.strokeStyle = "#1d242c"; ctx.lineWidth = 1;
  for (let gx = 1; gx < 8; gx++) {
    ctx.beginPath(); ctx.moveTo(gx * W / 8, 0); ctx.lineTo(gx * W / 8, H); ctx.stroke();
  }
  for (let gy = 1; gy < 4; gy++) {
    ctx.beginPath(); ctx.moveTo(0, gy * H / 4); ctx.lineTo(W, gy * H / 4); ctx.stroke();
  }
  const hist = hub.curHist;
  if (hist.length < 2) {
    ctx.fillStyle = "#5b6672"; ctx.font = "12px sans-serif";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText("等待数据…", W / 2, H / 2);
    return;
  }
  const tEnd = hist[hist.length - 1].t, tWin = 0.8, t0 = tEnd - tWin;
  const x = (t) => (t - t0) / tWin * W;
  const yMid = H / 2;
  if (kind === "cur") {
    ctx.strokeStyle = "#2a333d";
    ctx.beginPath(); ctx.moveTo(0, yMid); ctx.lineTo(W, yMid); ctx.stroke();
    let maxA = 1;
    for (const p of hist) maxA = Math.max(maxA, Math.abs(p.iq), Math.abs(p.id));
    maxA *= 1.2;
    const traces = [["iq", "#fb923c"], ["id", "#22d3ee"]];
    for (const [key, color] of traces) {
      ctx.strokeStyle = color; ctx.lineWidth = 1.6; ctx.beginPath();
      let started = false;
      for (const p of hist) {
        if (p.t < t0) continue;
        const px = x(p.t), py = yMid - (p[key] / maxA) * (H / 2 - 14);
        if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
      }
      ctx.stroke();
    }
    ctx.font = "11px Consolas, monospace"; ctx.textBaseline = "top";
    ctx.textAlign = "left"; ctx.fillStyle = "#fb923c"; ctx.fillText("iq", 8, 8);
    ctx.fillStyle = "#22d3ee"; ctx.fillText("id", 8, 22);
    ctx.fillStyle = "#7c8794"; ctx.textAlign = "right"; ctx.fillText("±" + maxA.toFixed(0) + " mA", W - 8, 8);
  } else if (kind === "angle") {
    const traces = [["rotorDeg", "#e5484d", false], ["thetaDeg", "#ffffff", true]];
    for (const [key, color, dash] of traces) {
      ctx.strokeStyle = color; ctx.lineWidth = 1.6;
      if (dash) ctx.setLineDash([5, 4]);
      ctx.beginPath(); let started = false;
      for (const p of hist) {
        if (p.t < t0) continue;
        const px = x(p.t), py = H - 8 - ((p[key] % 360) / 360) * (H - 16);
        if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.font = "11px Consolas, monospace"; ctx.textBaseline = "top";
    ctx.textAlign = "left"; ctx.fillStyle = "#e5484d"; ctx.fillText("转子角", 8, 8);
    ctx.fillStyle = "#ffffff"; ctx.fillText("控制角", 8, 22);
    ctx.fillStyle = "#7c8794"; ctx.textAlign = "right"; ctx.fillText("0–360°", W - 8, 8);
  } else { // diff
    ctx.strokeStyle = "#2a333d";
    ctx.beginPath(); ctx.moveTo(0, yMid); ctx.lineTo(W, yMid); ctx.stroke();
    ctx.strokeStyle = "#c084fc"; ctx.lineWidth = 1.6; ctx.beginPath();
    let started = false;
    for (const p of hist) {
      if (p.t < t0) continue;
      const px = x(p.t), py = yMid - (p.diffRad / Math.PI) * (H / 2 - 12);
      if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
    }
    ctx.stroke();
    ctx.font = "11px Consolas, monospace"; ctx.textBaseline = "top";
    ctx.textAlign = "left"; ctx.fillStyle = "#c084fc"; ctx.fillText("diff", 8, 8);
    ctx.fillStyle = "#7c8794"; ctx.textAlign = "right"; ctx.fillText("±π rad", W - 8, 8);
  }
}

/* ================= 日志 ================= */
function renderLog() {
  const box = document.getElementById("log");
  if (!hub.logs.length) { box.textContent = "等待数据…"; return; }
  box.innerHTML = "";
  for (const line of hub.logs.slice(-20)) {
    const div = document.createElement("div");
    if (line.startsWith("MOTF")) div.className = "mot";
    div.textContent = line;
    box.appendChild(div);
  }
  box.scrollTop = box.scrollHeight;
}

/* ================= 主循环 ================= */
buildNumGrid();
buildPhases();

function frame(now) {
  advance(now);
  drawMotor();
  drawGauge();
  updateNum();
  updatePhases();
  drawScope(scope1Cv, "cur");
  drawScope(scope2Cv, "angle");
  drawScope(scope3Cv, "diff");
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

