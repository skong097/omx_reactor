/* app.js — omx_reactor 고유 dashboard (omx 토픽만 처리).
 *
 * vendored engaging-analytics.js 가 자체적으로 /ws/v1/engaging 직접 구독 —
 * 본 app.js 와 독립. dashboard_node 의 /ws/v1/engaging 가 doby opserver 와
 * 동일 payload shape 발행하므로 engaging-analytics.js 0-modification 작동.
 *
 * 본 app.js 는 /ws/stream (omx 고유 — reactor 모션 + 이벤트) 만 처리해서
 * 상단 status bar (현 모션 / 큐 / track) + 우측 events 영역 갱신.
 *
 * Step B 의 이벤트 타임라인 시각화도 본 채널의 데이터로 추후 추가.
 */
import { sunglassesOverlay } from '/static/sunglasses-overlay.js';

(function () {
  const $ = (id) => document.getElementById(id);
  const motionEventsEl = $('motion-events');
  let _lastMotionId = null;

  function setStatus(p) {
    if (!p) return;
    if (p.current_motion !== undefined) {
      const m = p.current_motion || '-';
      const el = $('current_motion');
      if (el) el.textContent = m;
      const tag = $('ea-current-motion');
      if (tag) tag.textContent = m;
    }
    if (p.queued_motion !== undefined) {
      const el = $('queued_motion');
      if (el) el.textContent = p.queued_motion || '-';
    }
    if (p.track_id !== undefined) {
      const el = $('track');
      if (el) el.textContent = (p.track_id >= 0 ? p.track_id : '-');
    }
  }

  function pushMotionEvent(p) {
    if (!motionEventsEl) return;
    // 첫 placeholder 제거
    if (motionEventsEl.children.length === 1 && motionEventsEl.children[0].classList.contains('muted')) {
      motionEventsEl.innerHTML = '';
    }
    const li = document.createElement('li');
    li.className = 'mev';
    const ts = new Date().toLocaleTimeString('ko-KR');
    const quad = p.quadrant || (p.in_deadband ? 'deadband' : '-');
    const v = (p.v !== null && p.v !== undefined) ? p.v.toFixed(2) : '-';
    const a = (p.a !== null && p.a !== undefined) ? p.a.toFixed(2) : '-';
    const session = p.session_event ? ` ${p.session_event}` : '';
    li.innerHTML =
      `<span class="mev-ts">${ts}</span>` +
      `<span class="mev-id">▶ ${p.current_motion}</span>` +
      `<span class="mev-state">${quad}  V=${v} A=${a}${session}</span>`;
    motionEventsEl.prepend(li);
    while (motionEventsEl.children.length > 15) motionEventsEl.lastChild.remove();
  }

  function handle(msg) {
    if (msg.type === 'snapshot') {
      const pay = msg.payload || {};
      if (pay.reactor) {
        setStatus(pay.reactor);
        _lastMotionId = pay.reactor.current_motion || null;
        sunglassesOverlay.setActive(_lastMotionId);
      }
    } else if (msg.type === 'reactor') {
      const p = msg.payload;
      setStatus(p);
      // current_motion 이 바뀐 순간에만 push (매 100ms publish 라 spam 방지)
      const cur = p.current_motion || null;
      if (cur && cur !== _lastMotionId) {
        pushMotionEvent(p);
      }
      _lastMotionId = cur;
      sunglassesOverlay.setActive(cur);
    }
    // 'emotion' / 'rapport' 는 engaging-analytics 가 직접 /ws/v1/engaging 으로 처리
  }

  // Gazebo view 동적 zoom (CSS transform — 브라우저 클라이언트 측만)
  (function initZoom() {
    const slider = $('gazebo-zoom');
    const label = $('gazebo-zoom-label');
    const img = $('gazebo-view-img');
    const overlay = $('sunglasses-overlay');
    if (!slider || !img) return;
    function apply(val) {
      img.style.transform = `scale(${val})`;
      if (overlay) overlay.style.transform = `scale(${val})`;
      if (label) label.textContent = `${parseFloat(val).toFixed(1)}x`;
    }
    slider.addEventListener('input', (e) => apply(e.target.value));
    apply(slider.value);
  })();

  // sunglasses overlay 초기화 (FaceLandmarker async load — 실패해도 다른 UI 정상)
  sunglassesOverlay.init(
    document.getElementById('gazebo-view-img'),
    document.getElementById('sunglasses-overlay'),
  );

  function connect() {
    const ws = new WebSocket(`ws://${location.host}/ws/stream`);
    ws.onmessage = (ev) => {
      try { handle(JSON.parse(ev.data)); } catch (e) { console.error('[omx-ws]', e); }
    };
    ws.onclose = () => setTimeout(connect, 5_000);
    // keep-alive (dashboard_node 의 await ws.receive_text())
    setInterval(() => { if (ws.readyState === 1) ws.send('ping'); }, 25_000);
  }
  connect();
})();
