/**
 * sunglasses-overlay — active OMX 모션 동안 사용자 얼굴 눈에 썬글라스 overlay.
 *
 * 사용:
 *   import { sunglassesOverlay } from '/static/sunglasses-overlay.js';
 *   await sunglassesOverlay.init(imgEl, canvasEl);
 *   sunglassesOverlay.setActive('NOD');   // 활성
 *   sunglassesOverlay.setActive('IDLE');  // 비활성
 *   sunglassesOverlay.setActive(null);    // 비활성
 *
 * 상태: detection + 그리기 동작 (FaceLandmarker IMAGE mode, 10Hz). app.js 연결은 Task 6.
 */

import {
  FaceLandmarker,
  FilesetResolver,
} from 'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.14/vision_bundle.mjs';

let _img = null;
let _canvas = null;
let _ctx = null;
let _faceLandmarker = null;   // Task 4 에서 채워짐
let _active = false;
let _rafId = null;
let _ro = null;               // ResizeObserver instance

const DETECT_PERIOD_MS = 100;          // 10Hz throttle
const LENS_WIDTH_SCALE = 1.4;          // lens width = eye width * this
const LENS_ASPECT = 0.55;              // lens height / lens width
const BRIDGE_THICKNESS_RATIO = 0.15;   // bridge height / max lens height
let _lastDetectAt = 0;

async function init(imgEl, canvasEl) {
  if (!imgEl || !canvasEl) {
    throw new Error('[sunglasses-overlay] init requires imgEl and canvasEl');
  }
  // Re-init guard: cancel any orphan RAF + disconnect orphan ResizeObserver
  if (_rafId !== null) {
    cancelAnimationFrame(_rafId);
    _rafId = null;
  }
  if (_ro !== null) {
    _ro.disconnect();
    _ro = null;
  }
  _img = imgEl;
  _canvas = canvasEl;
  _ctx = _canvas.getContext('2d');

  try {
    const filesetResolver = await FilesetResolver.forVisionTasks(
      'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.14/wasm'
    );
    _faceLandmarker = await FaceLandmarker.createFromOptions(filesetResolver, {
      baseOptions: {
        modelAssetPath: '/static/face_landmarker.task',
        delegate: 'CPU',
      },
      runningMode: 'IMAGE',
      numFaces: 1,
    });
    console.info('[sunglasses-overlay] FaceLandmarker ready');
  } catch (e) {
    console.warn('[sunglasses-overlay] FaceLandmarker init 실패 — overlay 비활성', e);
    _faceLandmarker = null;
  }

  // img 크기 변화 추적 — 1-column responsive wrap / window resize 시 canvas 정렬 유지.
  // _detectLoop 의 width/height sync 는 100ms throttle 이라 그 사이 resize 시 어긋남 → 즉시 보정.
  if (typeof ResizeObserver !== 'undefined') {
    _ro = new ResizeObserver(() => {
      if (_img && _canvas) {
        _canvas.width = _img.clientWidth;
        _canvas.height = _img.clientHeight;
      }
    });
    _ro.observe(_img);
  }
}

/**
 * 478 face landmark 에서 안경 직사각형 좌표 산출.
 * landmarks: [{x, y, z}, ...]  (정규화 0~1)
 * canvasW/H: 픽셀 크기
 * 반환: { leftLens, rightLens, bridge } 각 {x, y, w, h} 픽셀.
 */
function _sunglassesRectsFromLandmarks(landmarks, canvasW, canvasH) {
  // 좌안 (얼굴 기준 왼쪽 = 화면상 오른쪽)
  const lOuter = landmarks[33];   // outer corner
  const lInner = landmarks[133];  // inner corner
  // 우안 (얼굴 기준 오른쪽 = 화면상 왼쪽)
  const rInner = landmarks[362];  // inner corner
  const rOuter = landmarks[263];  // outer corner

  // 정규화 → 픽셀
  const lOx = lOuter.x * canvasW, lOy = lOuter.y * canvasH;
  const lIx = lInner.x * canvasW, lIy = lInner.y * canvasH;
  const rIx = rInner.x * canvasW, rIy = rInner.y * canvasH;
  const rOx = rOuter.x * canvasW, rOy = rOuter.y * canvasH;

  // 각 눈의 폭 → 렌즈 크기
  const leftEyeW = Math.abs(lOx - lIx);
  const rightEyeW = Math.abs(rOx - rIx);

  // 렌즈 폭 = 눈 폭의 LENS_WIDTH_SCALE 배, 높이 = 폭의 LENS_ASPECT
  const lLensW = leftEyeW * LENS_WIDTH_SCALE;
  const lLensH = lLensW * LENS_ASPECT;
  const rLensW = rightEyeW * LENS_WIDTH_SCALE;
  const rLensH = rLensW * LENS_ASPECT;

  // 렌즈 중심 = 눈 중심
  const lCx = (lOx + lIx) / 2, lCy = (lOy + lIy) / 2;
  const rCx = (rOx + rIx) / 2, rCy = (rOy + rIy) / 2;

  const leftLens = {
    x: lCx - lLensW / 2, y: lCy - lLensH / 2, w: lLensW, h: lLensH,
  };
  const rightLens = {
    x: rCx - rLensW / 2, y: rCy - rLensH / 2, w: rLensW, h: rLensH,
  };

  // 다리: 두 inner corner 를 잇는 짧은 직사각형
  const bridgeStartX = Math.min(lIx, rIx);
  const bridgeEndX = Math.max(lIx, rIx);
  const bridgeY = (lCy + rCy) / 2;
  const bridgeH = Math.max(lLensH, rLensH) * BRIDGE_THICKNESS_RATIO;
  const bridge = {
    x: bridgeStartX,
    y: bridgeY - bridgeH / 2,
    w: bridgeEndX - bridgeStartX,
    h: bridgeH,
  };

  return { leftLens, rightLens, bridge };
}

function _drawSunglasses(rects) {
  _ctx.fillStyle = '#000';
  _ctx.fillRect(rects.leftLens.x, rects.leftLens.y, rects.leftLens.w, rects.leftLens.h);
  _ctx.fillRect(rects.rightLens.x, rects.rightLens.y, rects.rightLens.w, rects.rightLens.h);
  _ctx.fillRect(rects.bridge.x, rects.bridge.y, rects.bridge.w, rects.bridge.h);
}

function _detectLoop(t) {
  if (!_active) { _rafId = null; return; }
  _rafId = requestAnimationFrame(_detectLoop);

  if (t - _lastDetectAt < DETECT_PERIOD_MS) return;
  _lastDetectAt = t;

  if (!_faceLandmarker) return;
  if (!_img || !_img.naturalWidth) return;

  // Sync canvas pixel size to img display size (CSS px).
  // ResizeObserver in Task 7 will handle resize between detect ticks.
  if (_canvas.width !== _img.clientWidth || _canvas.height !== _img.clientHeight) {
    _canvas.width = _img.clientWidth;
    _canvas.height = _img.clientHeight;
  }

  const result = _faceLandmarker.detect(_img);

  if (!result.faceLandmarks || result.faceLandmarks.length === 0) {
    _ctx.clearRect(0, 0, _canvas.width, _canvas.height);
    return;
  }

  const rects = _sunglassesRectsFromLandmarks(
    result.faceLandmarks[0], _canvas.width, _canvas.height);
  _ctx.clearRect(0, 0, _canvas.width, _canvas.height);
  _drawSunglasses(rects);
}

function setActive(motion) {
  const shouldBeActive = !!(motion && motion !== 'IDLE');
  if (shouldBeActive === _active) return;
  _active = shouldBeActive;
  console.info(`[sunglasses-overlay] active=${_active} (motion=${motion})`);

  if (_active && _rafId === null) {
    _rafId = requestAnimationFrame(_detectLoop);
  }
  if (!_active && _rafId !== null) {
    cancelAnimationFrame(_rafId);
    _rafId = null;
  }
  if (!_active && _ctx && _canvas) {
    _ctx.clearRect(0, 0, _canvas.width, _canvas.height);
  }
}

export const sunglassesOverlay = { init, setActive };
