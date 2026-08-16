"use strict";

/* ================= 常量 ================= */
const POLL_MS = 15;             // 轮询周期：越短则数据越鲜、外推间隙越小（快速扭动不过冲）
const STALE_MS = 400;            // 超过该时长没有新帧 => 数据中断，转子冻结（不得用旧转速假装转动）
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
  lastAgeMs: -1,          // 最新帧年龄（ms），-1 = 尚未收到帧
  lastFrameWall: 0,       // 最近一次收到新帧的墙钟时间（用于外推窗口）
  logs: [],
  lastLogSeq: 0,
  logFilter: "",
};

const motorCv = document.getElementById("motor");
const mctx = motorCv.getContext("2d");
const gaugeCv = document.getElementById("rpmGauge");
const gctx = gaugeCv.getContext("2d");
const scope1Cv = document.getElementById("scope1");
const scope2Cv = document.getElementById("scope2");
const scope3Cv = document.getElementById("scope3");
const scope4Cv = document.getElementById("scope4");

/* ================= 数据轮询 ================= */
async function poll() {
  try {
    const r = await fetch(`/data?since=${hub.lastSeq}&_=${Date.now()}`);
    const d = await r.json();
    hub.status = d.status; hub.detail = d.detail || "--"; hub.fps = d.fps || 0;
    if (d.last_age_ms !== undefined) hub.lastAgeMs = d.last_age_ms;
    if (d.seq < hub.lastSeq) hub.lastSeq = 0;   // 服务器重启（seq 倒退）：重新对齐
    const paused = !scopeNav.followLive;        // 暂停：缓冲写入/裁剪/最新帧全部冻结
    if (d.latest && !paused) {
      hub.latest = {
        mode: d.latest[0], phase: d.latest[1], rotor: d.latest[2],
        theta: d.latest[3], iq: d.latest[4], id: d.latest[5],
        vq: d.latest[6], vd: d.latest[7], spd: d.latest[8],
        sync: d.latest[9], diff: d.latest[10], freq: d.latest[11],
        ms: d.latest[12], mech: d.latest[13],
        isMa: d.latest[14], isAng: d.latest[15],
        vMv: d.latest[16], vAng: d.latest[17],
        thetaMech: d.latest[18],   // 控制角机械角 mrad（θ 指针走 thetaMechDeg 插值，此字段保留协议完整性）
      };
      hub.rpmMax = Math.max(hub.rpmMax, Math.abs(hub.latest.spd) * 1.25);
      hub.curMax = Math.max(hub.curMax, Math.abs(hub.latest.iq), Math.abs(hub.latest.id), Math.abs(hub.latest.isMa));
    }
    if (!paused) {
      for (const h of d.history) {
        const f = h[2];
        scopeAppend({
          t: h[1],
          iq: f[4], id: f[5],
          rotorDeg: (f[2] / 1000) * RAD2DEG,        // 电角度 °
          thetaDeg: (f[3] / 1000) * RAD2DEG,
          mechDeg: (f[13] / 1000) * RAD2DEG,   // 固件直传连续机械角 deg
          thetaMechDeg: (f[18] / 1000) * RAD2DEG,   // 固件直传控制角机械角 deg
          diffRad: f[10] / 1000,
          mode: f[0],
        });
      }
      scopeTrim();
      if (d.history && d.history.length) hub.lastFrameWall = Date.now();
    }
    if (d.logs && d.logs.length) {
      for (const e of d.logs) {
        if (e[0] > hub.lastLogSeq) { hub.logs.push(e[1]); hub.lastLogSeq = e[0]; }
      }
      while (hub.logs.length > 2000) hub.logs.shift();
      renderLog();
    }
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
    const stale = d.status === "running" && d.last_age_ms >= 0 && d.last_age_ms > STALE_MS;
    const hint = document.getElementById("cHint");
    if (stale) {
      hint.textContent = "数据中断：" + d.last_age_ms.toFixed(0) + " ms 无新帧（目标可能已停止、复位或连接断开）。";
      hint.style.color = "#f59e0b";
    } else if (d.status === "running" && d.last_age_ms >= 0) {
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
  // 注意：lastSeq（帧消费游标）不重置——已消费的帧绝不重发、不重新处理
  hub.lastLogSeq = 0;
  hub.logs = [];
  scopeClear();
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

/* ================= 状态栏 ================= */
function updateStatus() {
  const dot = document.getElementById("connDot");
  const txt = document.getElementById("connText");
  const stale = hub.status === "running" && hub.lastAgeMs >= 0 && hub.lastAgeMs > STALE_MS;
  if (hub.status === "running" && !stale) { dot.className = "dot good"; txt.textContent = "实时连接"; }
  else if (hub.status === "running" && stale) { dot.className = "dot"; txt.textContent = "数据中断（无新帧）"; }
  else if (hub.status === "error") { dot.className = "dot bad"; txt.textContent = "连接异常"; }
  else { dot.className = "dot"; txt.textContent = "连接中…"; }
  const f = hub.latest;
  const simWarn = document.getElementById("simWarn");
  if (simWarn) simWarn.style.display = (hub.detail || "").includes("仿真") ? "block" : "none";
  document.getElementById("modePill").textContent = f ? ("mode " + f.mode + " " + (MODE_NAMES[f.mode] || "")) : "--";
  document.getElementById("phasePill").textContent = f ? ("phase " + f.phase + " " + (f.phase <= 4 ? PHASE_LIST[f.phase].name : "")) : "--";
  const sp = document.getElementById("syncPill");
  if (f) { sp.textContent = f.sync ? "已同步" : "未同步"; sp.style.color = f.sync ? "#4ade80" : "#f59e0b"; }
  document.getElementById("srcPill").textContent = hub.detail;
  document.getElementById("fpsPill").textContent = hub.fps.toFixed(0) + " fps";
}

/* ================= 显示角度（帧间按转速/频率积分，动画平滑） ================= */
let visRotorMech = 0, visCtrlMech = 0, visRotorElec = 0, visCtrlElec = 0;
let lastNow = performance.now();

function advance(now) {
  const s = hub.scope;
  if (!s.n || !hub.latest) return;
  // 数据新鲜度：超过 STALE_MS 没有新帧 => 冻结在最新位置
  const stale = !scopeNav.followLive || hub.lastAgeMs < 0 || hub.lastAgeMs > STALE_MS;

  // 转子/控制角：用最新两个数据点做"线性插值/外推"，完全跟随真实帧数据。
  // 不再用转速积分、不做收敛校正——彻底避免"转一点又弹回原位"。
  const iLast = (s.head + s.n - 1) % s.cap;
  const iPrev = (s.head + s.n - 2) % s.cap;
  const tLast = s.t[iLast], tPrev = s.t[iPrev];
  const span = tLast - tPrev;

  // 外推窗口：距最新数据点收到的时间（≤轮询周期，上限 20ms 避免快速瞬态过冲）；数据中断则冻结
  const ext = stale ? 0 : Math.min((Date.now() - (hub.lastFrameWall || Date.now())) / 1000, 0.02);

  let rot = s.rotorDeg[iLast];
  let th = s.thetaDeg[iLast];
  if (span > 1e-6) {
    let dRot = s.rotorDeg[iLast] - s.rotorDeg[iPrev];
    dRot = ((dRot % 360) + 540) % 360 - 180;      // 连续旋转的最小角差
    let dTh = s.thetaDeg[iLast] - s.thetaDeg[iPrev];
    dTh = ((dTh % 360) + 540) % 360 - 180;
    rot = s.rotorDeg[iLast] + dRot / span * ext;
    th  = s.thetaDeg[iLast]  + dTh  / span * ext;
  }
  // 机械角：固件直传连续值（编码器 g_enc_count*FOC_ENC_DIR 换算），直接插值/外推，
  // 不再由电角度解卷÷极对数反推（避免窗口滑动时锚点漂移导致的 0°/360° 回跳）。
  let mech = s.mechDeg[iLast];
  if (span > 1e-6) {
    const dMech = s.mechDeg[iLast] - s.mechDeg[iPrev];
    mech = s.mechDeg[iLast] + dMech / span * ext;
  }
  visRotorElec = rot;
  visRotorMech = mech;
  // 控制角机械角：固件直传（theta/极对数，折回值），插值/外推。
  // 回绕周期按相位：RUN=360°；I-F（hold/ramp/sync）=360°/极对数（控制角每电周期折叠）。
  let ctrlMech = s.thetaMechDeg[iLast];
  if (span > 1e-6) {
    const ctrlWrap = (hub.latest.phase === 3) ? 360 : 360 / POLE_PAIRS;
    let dCtrlMech = s.thetaMechDeg[iLast] - s.thetaMechDeg[iPrev];
    dCtrlMech = ((dCtrlMech % ctrlWrap) + ctrlWrap * 1.5) % ctrlWrap - ctrlWrap / 2;
    ctrlMech = s.thetaMechDeg[iLast] + dCtrlMech / span * ext;
  }
  visCtrlMech = ctrlMech;
  visCtrlElec = th;
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
  // 转子标记：第一个 N 极上放亮黄色 ★（快速转动时能看出旋转趋势）
  {
    const mx = Math.cos(visRotorMech * DEG) * (R_R + R_M) / 2;
    const my = Math.sin(visRotorMech * DEG) * (R_R + R_M) / 2;
    ctx.save();
    ctx.shadowColor = "#facc15"; ctx.shadowBlur = 16;
    ctx.fillStyle = "#fde047";
    ctx.strokeStyle = "#241505"; ctx.lineWidth = 2.5;
    ctx.font = "bold 26px sans-serif";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.strokeText("★", mx, my + 1);
    ctx.fillText("★", mx, my + 1);
    ctx.restore();
  }

  // d/q 轴
  const qAng = visRotorMech + 90 / POLE_PAIRS;
  ctx.strokeStyle = "rgba(255,255,255,0.30)"; ctx.setLineDash([3, 5]); ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(Math.cos(visRotorMech * DEG) * (R_R - 4), Math.sin(visRotorMech * DEG) * (R_R - 4)); ctx.stroke();
  ctx.strokeStyle = "rgba(255,255,255,0.16)";
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(Math.cos(qAng * DEG) * (R_R - 4), Math.sin(qAng * DEG) * (R_R - 4)); ctx.stroke();
  ctx.setLineDash([]);

  // 控制角 θ（I-F 合成角 / RUN 控制角）——白色虚线指针
  const ctrlMech = visCtrlMech;   // 固件直传控制角机械角
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
    const isL = (f.isMa / maxA) * Lmax;                                   // 固件直传 is 幅值
    const isAng = visRotorMech + (f.isAng / 1000) * RAD2DEG / POLE_PAIRS; // dq 电角度→机械画布角
    drawVector(ctx, "#22d3ee", idL, visRotorMech, 4);          // id
    drawVector(ctx, "#fb923c", iqL, qAng, 4);                  // iq
    drawVector(ctx, "#facc15", isL, isAng, 5);                 // is
    // 电压矢量（vd/vq）
    const vLmax = 100;
    const vL = f.vMv / 1000 * vLmax;                                      // 固件直传 v 幅值
    const vAng = visRotorMech + (f.vAng / 1000) * RAD2DEG / POLE_PAIRS;   // dq 电角度→机械画布角
    if (vL > 3) drawVector(ctx, "#e879f9", vL, vAng, 3, [4, 4]);
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
      `θe转子=${(visRotorElec % 360).toFixed(0)}° θm机械=${(((visRotorMech % 360) + 360) % 360).toFixed(0)}° ` +
      `θe控制=${(visCtrlElec % 360).toFixed(0)}°  iq=${f.iq.toFixed(0)}mA id=${f.id.toFixed(0)}mA n=${f.spd.toFixed(0)}rpm`,
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
    ["控制角 θ", "ctrlVal", "0°"], ["机械角 θm", "mechVal", "0°"],
    ["iq", "iqVal", "0 mA"],
    ["id", "idVal", "0 mA"], ["is", "isVal", "0 mA"],
    ["vq", "vqVal", "0 mV"],
    ["vd", "vdVal", "0 mV"], ["|v|", "vMagVal", "0 mV"],
    ["电频率", "freqVal", "0 Hz"],
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
  set("mechVal", ((((hub.latest.mech / 1000) * RAD2DEG % 360) + 360) % 360).toFixed(1) + "°");
  set("iqVal", f.iq.toFixed(0) + " mA");
  set("idVal", f.id.toFixed(0) + " mA");
  set("vqVal", f.vq.toFixed(0) + " mV");
  set("vdVal", f.vd.toFixed(0) + " mV");
  set("isVal", f.isMa.toFixed(0) + " mA");
  set("vMagVal", f.vMv.toFixed(0) + " mV");
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

/* ================= 示波器（环形缓冲 + 坐标轴 + 时间回看 + 悬停读数） ================= */
const SCOPE_CAP = 30000;              // 环形缓冲最大点数（1kHz 下 30s，内存约 720KB，恒定有界）
const SCOPE_KEEP_SEC = 20.0;          // 按时间裁剪
hub.scope = {
  cap: SCOPE_CAP, n: 0, head: 0,
  t: new Float64Array(SCOPE_CAP),  // 时间戳=Unix秒(~1.79e9)，float32 仅 24bit 整数精度会全部崩塌成同一值，必须 float64
  // （其余量 iq/id/角度/diff/mode 数值小，float32 足够）
  iq: new Float32Array(SCOPE_CAP),
  id: new Float32Array(SCOPE_CAP),
  rotorDeg: new Float32Array(SCOPE_CAP),
  thetaDeg: new Float32Array(SCOPE_CAP),
  mechDeg: new Float32Array(SCOPE_CAP),
  thetaMechDeg: new Float32Array(SCOPE_CAP),
  diffRad: new Float32Array(SCOPE_CAP),
  mode: new Float32Array(SCOPE_CAP),
};

function scopeClear() { hub.scope.n = 0; hub.scope.head = 0; }
function scopeCount() { return hub.scope.n; }
function scopeFirstT() { const s = hub.scope; return s.n ? s.t[s.head] : 0; }
function scopeLastT() { const s = hub.scope; return s.n ? s.t[(s.head + s.n - 1) % s.cap] : 0; }
function scopeAppend(p) {
  const s = hub.scope;
  const idx = (s.head + s.n) % s.cap;
  s.t[idx] = p.t; s.iq[idx] = p.iq; s.id[idx] = p.id;
  s.rotorDeg[idx] = p.rotorDeg; s.thetaDeg[idx] = p.thetaDeg; s.mechDeg[idx] = p.mechDeg;
  s.thetaMechDeg[idx] = p.thetaMechDeg;
  s.diffRad[idx] = p.diffRad;
  s.mode[idx] = p.mode;
  if (s.n < s.cap) s.n++; else s.head = (s.head + 1) % s.cap;
}
function scopeTrim() {
  const s = hub.scope;
  if (!s.n) return;
  const minT = scopeLastT() - SCOPE_KEEP_SEC;
  while (s.n > 0 && s.t[s.head] < minT) {
    s.head = (s.head + 1) % s.cap;
    s.n--;
  }
}
function scopeLowerBound(t) {
  const s = hub.scope; let lo = 0, hi = s.n;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (s.t[(s.head + mid) % s.cap] < t) lo = mid + 1; else hi = mid;
  }
  return lo;
}

const scopeNav = { windowSec: 1.0, followLive: true, viewEnd: 0, hover: null };
const scopeCfg = {
  cur:   { iq: true,  id: true },
  angle: { rotor: true, theta: true, mech: true },
  diff:  { diff: true },
};

function fmtVal(v) {
  const a = Math.abs(v);
  if (a >= 1e6) return (v / 1e6).toFixed(2) + "M";
  if (a >= 1e3) return (v / 1e3).toFixed(1) + "k";
  if (a >= 100) return v.toFixed(0);
  if (a >= 1) return v.toFixed(1);
  return v.toFixed(2);
}

function scopeWindow() {
  if (!scopeCount()) return null;
  const maxT = scopeLastT(), minT = scopeFirstT();
  let t1 = scopeNav.followLive ? maxT : scopeNav.viewEnd;
  if (t1 > maxT) t1 = maxT;
  let t0 = t1 - scopeNav.windowSec;
  if (t0 < minT) { t0 = minT; t1 = t0 + scopeNav.windowSec; if (t1 > maxT) t1 = maxT; }
  return { t0, t1, minT, maxT };
}

function interpVal(t, key) {
  const s = hub.scope;
  if (!s.n) return null;
  if (t <= scopeFirstT()) return s[key][s.head];
  if (t >= scopeLastT()) return s[key][(s.head + s.n - 1) % s.cap];
  let lo = 0, hi = s.n - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (s.t[(s.head + mid) % s.cap] < t) lo = mid + 1; else hi = mid;
  }
  if (lo === 0) lo = 1;
  const a = lo - 1, b = lo;
  const ta = s.t[(s.head + a) % s.cap], tb = s.t[(s.head + b) % s.cap];
  const f = (t - ta) / ((tb - ta) || 1);
  const va = s[key][(s.head + a) % s.cap], vb = s[key][(s.head + b) % s.cap];
  return va + (vb - va) * f;
}

function drawScope(cv, kind) {
  const ctx = cv.getContext("2d");
  const W = cv.width, H = cv.height;
  const mL = 58, mR = 14, mT = 16, mB = 26;
  const pw = W - mL - mR, ph = H - mT - mB;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#10151a";
  ctx.fillRect(0, 0, W, H);

  const s = hub.scope;
  const win = scopeWindow();
  if (!win || s.n < 2) {
    ctx.fillStyle = "#5b6672"; ctx.font = "12px sans-serif";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText("等待数据…", W / 2, H / 2);
    return;
  }
  const t0 = win.t0, t1 = win.t1, n = s.n;
  const x = (t) => mL + (t - t0) / (t1 - t0) * pw;
  const startIdx = scopeLowerBound(t0);

  /* Y 轴配置 */
  let ticks, fmt, yMap, unit, traces;
  if (kind === "cur") {
    let maxA = 1;
    for (let i = startIdx; i < n; i++) {
      const idx = (s.head + i) % s.cap;
      if (s.t[idx] > t1) break;
      maxA = Math.max(maxA, Math.abs(s.iq[idx]), Math.abs(s.id[idx]));
    }
    maxA *= 1.15;
    const yMid = mT + ph / 2;
    ticks = [-maxA, -maxA / 2, 0, maxA / 2, maxA];
    fmt = fmtVal; unit = "mA";
    yMap = (v) => yMid - (v / maxA) * (ph / 2 - 16);
    traces = [
      { key: "iq", color: "#fb923c", dash: false, on: () => scopeCfg.cur.iq },
      { key: "id", color: "#22d3ee", dash: false, on: () => scopeCfg.cur.id },
    ];
  } else if (kind === "angle") {
    ticks = [0, 90, 180, 270, 360];
    fmt = (v) => v.toFixed(0); unit = "°";
    yMap = (v) => mT + 10 + (360 - (((v % 360) + 360) % 360)) / 360 * (ph - 20);
    traces = [
      { key: "rotorDeg", color: "#e5484d", dash: false, on: () => scopeCfg.angle.rotor },
      { key: "thetaDeg", color: "#ffffff", dash: true,  on: () => scopeCfg.angle.theta },
      { mech: true, key: "mechDeg", color: "#4ade80", dash: false, on: () => scopeCfg.angle.mech },
    ];
  } else if (kind === "mode") {
    ticks = [0, 1, 2, 3, 4];
    fmt = (v) => v.toFixed(0); unit = "";
    yMap = (v) => mT + 10 + (4 - v) / 4 * (ph - 20);
    traces = [{ key: "mode", color: "#f472b6", dash: false, on: () => true }];
  } else {
    const yMid = mT + ph / 2;
    ticks = [-Math.PI, -Math.PI / 2, 0, Math.PI / 2, Math.PI];
    fmt = (v) => (v === 0 ? "0" : (v / Math.PI).toFixed(1) + "π");
    unit = "rad";
    yMap = (v) => yMid - (v / Math.PI) * (ph / 2 - 14);
    traces = [{ key: "diffRad", color: "#c084fc", dash: false, on: () => scopeCfg.diff.diff }];
  }

  /* 网格 + Y 轴刻度 */
  ctx.font = "10px Consolas, monospace";
  ctx.textAlign = "right"; ctx.textBaseline = "middle";
  for (const v of ticks) {
    const y = yMap(v);
    ctx.strokeStyle = "rgba(255,255,255,0.06)";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(mL, y); ctx.lineTo(W - mR, y); ctx.stroke();
    ctx.fillStyle = "#7c8794";
    ctx.fillText(fmt(v), mL - 6, y);
  }
  ctx.fillStyle = "#5b6672"; ctx.textAlign = "left"; ctx.textBaseline = "top";
  ctx.fillText(unit, mL + 6, mT);

  /* X 轴刻度 + 网格 */
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  for (let k2 = 0; k2 <= 4; k2++) {
    const tt = t0 + (t1 - t0) * k2 / 4;
    ctx.strokeStyle = "rgba(255,255,255,0.06)";
    ctx.beginPath(); ctx.moveTo(x(tt), mT); ctx.lineTo(x(tt), mT + ph); ctx.stroke();
    ctx.fillStyle = "#7c8794";
    ctx.fillText((tt - t1) > -0.0005 ? "0" : (tt - t1).toFixed(2) + "s", x(tt), H - mB + 5);
  }
  ctx.fillStyle = "#5b6672"; ctx.textAlign = "right";
  ctx.fillText("t/s", W - mR, H - mB + 5);

  /* 波形 */
  for (const tr of traces) {
    if (!tr.on()) continue;
    ctx.strokeStyle = tr.color;
    ctx.lineWidth = 1.6;
    if (tr.dash) ctx.setLineDash([5, 4]);
    ctx.beginPath();
    let started = false;
    for (let i = startIdx; i < n; i++) {
      const idx = (s.head + i) % s.cap;
      const tt = s.t[idx];
      if (tt > t1) break;
      let v;
      if (tr.mech) {
        v = s.mechDeg[idx];   // 固件直传连续机械角，yMap 的 %360 负责 0/360 回绕
      } else {
        v = s[tr.key][idx];
      }
      const px = x(tt), py = yMap(v);
      if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  /* 悬停十字光标 + 读数 */
  const hv = scopeNav.hover;
  if (hv && hv.kind === kind && hv.x >= mL && hv.x <= W - mR) {
    const th = t0 + (hv.x - mL) / pw * (t1 - t0);
    ctx.strokeStyle = "rgba(255,255,255,0.5)";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x(th), mT); ctx.lineTo(x(th), mT + ph); ctx.stroke();
    const rows = [];
    for (const tr of traces) {
      if (!tr.on() || tr.mech) continue;   // 机械角在数值面板实时显示，悬停跳过
      const v = interpVal(th, tr.key);
      if (v === null) continue;
      const label = tr.key === "rotorDeg" ? "转子" : tr.key === "thetaDeg" ? "控制" : tr.key;
      rows.push({ color: tr.color, text: label + " " + fmtVal(v) });
    }
    const bw = 150, bh = 18 * (rows.length + 1) + 6;
    const bx = Math.min(W - mR - bw - 6, x(th) + 10), by = mT + 4;
    ctx.fillStyle = "rgba(8,12,16,0.88)";
    ctx.strokeStyle = "#2a3643";
    ctx.fillRect(bx, by, bw, bh);
    ctx.strokeRect(bx, by, bw, bh);
    ctx.font = "11px Consolas, monospace";
    ctx.textAlign = "left"; ctx.textBaseline = "middle";
    ctx.fillStyle = "#8fa0b0";
    ctx.fillText("t " + (th - t1).toFixed(3) + " s", bx + 6, by + 10);
    rows.forEach((r, i3) => {
      ctx.fillStyle = r.color;
      ctx.fillText(r.text, bx + 6, by + 10 + 18 * (i3 + 1));
    });
  }
}

function updateSlider() {
  const win = scopeWindow();
  const s = document.getElementById("timeSlider");
  const lbl = document.getElementById("timeLabel");
  const live = document.getElementById("liveBtn");
  const pause = document.getElementById("pauseBtn");
  if (!win) { if (s) s.value = 1000; if (lbl) lbl.textContent = "--"; return; }
  const { minT, maxT } = win;
  const lo = minT + scopeNav.windowSec, hi = maxT;
  if (scopeNav.followLive || hi <= lo + 1e-6) {
    scopeNav.viewEnd = maxT;
    if (s) s.value = 1000;
    if (lbl) lbl.textContent = "实时";
    if (live) live.classList.add("btn-primary");
    if (pause) pause.classList.remove("btn-primary");
  } else {
    const frac = Math.min(1, Math.max(0, (scopeNav.viewEnd - lo) / (hi - lo)));
    if (s) s.value = Math.round(frac * 1000);
    if (lbl) lbl.textContent = "回看 " + Math.max(0, maxT - scopeNav.viewEnd).toFixed(2) + " s";
    if (live) live.classList.remove("btn-primary");
    if (pause) pause.classList.add("btn-primary");
  }
}

document.getElementById("winSel").addEventListener("change", (e) => {
  scopeNav.windowSec = parseFloat(e.target.value);
  scopeNav.followLive = true;
});
document.getElementById("pauseBtn").addEventListener("click", () => {
  scopeNav.followLive = false;
  const win = scopeWindow();
  if (win) scopeNav.viewEnd = win.maxT;
});
document.getElementById("liveBtn").addEventListener("click", () => { scopeNav.followLive = true; });
document.getElementById("timeSlider").addEventListener("input", (e) => {
  const win = scopeWindow();
  if (!win) return;
  scopeNav.followLive = false;
  const lo = win.minT + scopeNav.windowSec, hi = win.maxT;
  const frac = parseFloat(e.target.value) / 1000;
  scopeNav.viewEnd = lo + frac * (hi - lo);
});
document.querySelectorAll(".chip").forEach((ch) => {
  ch.addEventListener("click", () => {
    const g = ch.dataset.group, k = ch.dataset.key;
    if (!g || !k || !scopeCfg[g]) return;
    scopeCfg[g][k] = !scopeCfg[g][k];
    ch.classList.toggle("on", scopeCfg[g][k]);
  });
});
[["scope1", "cur"], ["scope2", "angle"], ["scope3", "diff"], ["scope4", "mode"]].forEach(([id, kind]) => {
  const cv = document.getElementById(id);
  cv.addEventListener("mousemove", (e) => {
    const r = cv.getBoundingClientRect();
    scopeNav.hover = { kind, x: (e.clientX - r.left) / r.width * cv.width };
  });
  cv.addEventListener("mouseleave", () => { scopeNav.hover = null; });
});


/* ================= 日志 ================= */
function escapeHtml(s) {
  return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

function filteredLogs() {
  const f = (hub.logFilter || "").toLowerCase();
  return f ? hub.logs.filter(l => l.toLowerCase().includes(f)) : hub.logs.slice();
}

function renderLog() {
  const box = document.getElementById("log");
  const cnt = document.getElementById("logCount");
  const f = hub.logFilter || "";
  if (!hub.logs.length) {
    box.textContent = "等待数据…";
    if (cnt) cnt.textContent = "0 / 0 条";
    return;
  }
  const lines = filteredLogs();
  if (cnt) cnt.textContent = lines.length + " / " + hub.logs.length + " 条";
  box.innerHTML = "";
  for (const line of lines) {
    const div = document.createElement("div");
    if (line.startsWith("MOTF")) div.className = "mot";
    let html = escapeHtml(line);
    if (f) {
      const lc = html.toLowerCase();
      const idx = lc.indexOf(f.toLowerCase());
      if (idx >= 0) {
        html = html.slice(0, idx) + "<mark>" + html.slice(idx, idx + f.length) +
               "</mark>" + html.slice(idx + f.length);
      }
    }
    div.innerHTML = html;
    box.appendChild(div);
  }
  box.scrollTop = box.scrollHeight;
}

function flashBtn(id, msg) {
  const b = document.getElementById(id);
  if (!b) return;
  const old = b.textContent;
  b.textContent = msg;
  setTimeout(() => { b.textContent = old; }, 1200);
}

function copyText(txt, btnId) {
  const done = () => flashBtn(btnId, "已复制");
  const fb = () => {
    const ta = document.createElement("textarea");
    ta.value = txt; document.body.appendChild(ta); ta.select();
    try { document.execCommand("copy"); done(); } catch (e) {}
    document.body.removeChild(ta);
  };
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(txt).then(done).catch(fb);
  } else { fb(); }
}

document.getElementById("logSearch").addEventListener("input", (e) => {
  hub.logFilter = e.target.value;
  renderLog();
});
document.getElementById("logCopy").addEventListener("click", () => {
  copyText(filteredLogs().join("\n"), "logCopy");
});
document.getElementById("logClear").addEventListener("click", async () => {
  try { await fetch("/clearlogs?_=" + Date.now()); } catch (e) {}
  hub.logs = [];
  hub.lastLogSeq = 0;
  renderLog();
});

/* 页面退出：自动清空 history.txt（sendBeacon 在 unload 时也能发出） */
window.addEventListener("pagehide", () => {
  try { navigator.sendBeacon("/clear_history"); } catch (e) {
    fetch("/clear_history").catch(() => {});
  }
});

/* ================= 主循环 ================= */
buildNumGrid();
buildPhases();

function frame(now) {
  advance(now);
  drawMotor();
  drawGauge();
  updateNum();
  updatePhases();
  updateSlider();
  drawScope(scope1Cv, "cur");
  drawScope(scope2Cv, "angle");
  drawScope(scope3Cv, "diff");
  drawScope(scope4Cv, "mode");
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

