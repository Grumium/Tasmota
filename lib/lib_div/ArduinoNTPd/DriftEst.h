/*
  DriftEst.h - crystal drift estimator for GPS PPS and upstream NTP

  SPDX-FileCopyrightText: 2026 Jan-David Förster and Claude

  SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef DRIFT_EST_H
#define DRIFT_EST_H

#include <stdint.h>

// Parameters per time source
struct DriftEstParams {
  uint32_t maxGapSecs;     // Longest step between samples, s
  uint32_t tolUs;          // Allowed step deviation from whole seconds, us
  uint32_t tolUsPerSec;    // plus this per second of step, us
  uint32_t noiseUs;        // Sample noise, us
  uint32_t windowSecs;     // Rolling window to detect a drift change, s
  uint32_t minSpanSecs;    // Shortest span for a drift figure, s
  uint32_t halflifeSecs;   // Required span halves per this outage time, s
  uint32_t movedPpb;       // Window figure this far off replaces the current one, ppb
  uint32_t claimPpb;       // Resolution needed for a first figure, ppb
};

// A zeroed instance is a valid empty estimator
struct DriftEst {
  uint64_t accumUs;        // Local us since the start of the span
  uint64_t winAccumUs;
  uint32_t secs;           // Reference seconds since the start of the span
  uint32_t winSecs;
  uint32_t spanBest;       // Longest span published, s
  uint32_t lostUptime;     // Uptime when the source was lost, 0 = present
  uint8_t  anchored;
};

enum DriftEvent : uint8_t { DRIFT_OK, DRIFT_RESTART, DRIFT_MOVED, DRIFT_SHORT, DRIFT_HELD, DRIFT_PUBLISH };

struct DriftResult {
  int32_t  ppb;            // MOVED: window drift, HELD/PUBLISH: span drift
  int32_t  fracUs;         // RESTART: step deviation from whole seconds, us
  int32_t  errUs;          // PUBLISH: error accumulated over the span, us
  uint32_t secs;           // RESTART: span lost, MOVED: window, HELD/PUBLISH: span, s
  uint32_t bar;            // HELD: required span, s
  uint32_t resPpb;         // PUBLISH: resolution, ppb
};

inline int32_t DriftSat32(int64_t v) {
  return (v > INT32_MAX) ? INT32_MAX : ((v < INT32_MIN) ? INT32_MIN : (int32_t)v);
}

inline int32_t DriftPpb(uint64_t us, uint32_t secs) {
  return DriftSat32((((int64_t)us - (int64_t)secs * 1000000) * 1000) / (int64_t)secs);
}

// Resolution of a span in ppb: sqrt(2) x noise / span
inline uint32_t DriftResPpb(const DriftEstParams* p, uint32_t secs) { return (p->noiseUs * 1414 / secs) + 1; }

inline void DriftAnchor(DriftEst* e) {
  e->anchored = 1;
  e->accumUs = 0;
  e->secs = 0;
  e->winAccumUs = 0;
  e->winSecs = 0;
}

// Add one step between two samples. standingPpb is the drift in use, 0 = none
inline DriftEvent DriftStep(DriftEst* e, const DriftEstParams* p, uint64_t stepUs, uint32_t stepSecs,
                            int32_t standingPpb, DriftResult* r) {
  int64_t frac = (int64_t)stepUs - (int64_t)stepSecs * 1000000;
  uint64_t tol = (uint64_t)p->tolUs + (uint64_t)stepSecs * p->tolUsPerSec;
  if ((stepSecs < 1) || (stepSecs > p->maxGapSecs) || ((uint64_t)(frac < 0 ? -frac : frac) > tol)) {
    r->fracUs = DriftSat32(frac);
    r->secs = e->secs;
    DriftAnchor(e);
    return DRIFT_RESTART;
  }
  e->accumUs += stepUs;
  e->secs += stepSecs;
  e->winAccumUs += stepUs;
  e->winSecs += stepSecs;
  if (e->winSecs < p->windowSecs) { return DRIFT_OK; }

  DriftEvent ev = DRIFT_OK;
  int32_t win_ppb = DriftPpb(e->winAccumUs, e->winSecs);
  uint32_t res = DriftResPpb(p, e->winSecs) + DriftResPpb(p, e->secs);
  uint32_t moved = (10 * res > p->movedPpb) ? (10 * res) : p->movedPpb;
  int64_t off = (int64_t)win_ppb - standingPpb;
  if (standingPpb && ((uint64_t)(off < 0 ? -off : off) > moved)) {
    r->ppb = win_ppb;
    r->secs = e->winSecs;
    e->accumUs = e->winAccumUs;                  // Restart the span with the window
    e->secs = e->winSecs;
    e->spanBest = e->winSecs;
    e->lostUptime = 0;
    ev = DRIFT_MOVED;
  }
  e->winAccumUs = 0;                             // Next window
  e->winSecs = 0;
  return ev;
}

// Span a new figure needs: the best span, halved per halflife of outage
inline uint32_t DriftSpanBar(const DriftEst* e, const DriftEstParams* p, uint32_t uptime) {
  uint32_t best = e->spanBest;
  if (!best) { return 0; }
  if (!e->lostUptime) { return best; }
  uint32_t halvings = (uptime - e->lostUptime) / p->halflifeSecs;
  if (halvings >= 32) { return p->minSpanSecs; }
  uint32_t bar = (2 * p->halflifeSecs) >> halvings;
  if (bar > best) { bar = best; }
  return (bar < p->minSpanSecs) ? p->minSpanSecs : bar;
}

// Drift figure of the span, once it is long and precise enough
inline DriftEvent DriftFigure(DriftEst* e, const DriftEstParams* p, uint32_t uptime, DriftResult* r) {
  if ((e->secs < p->minSpanSecs) || (DriftResPpb(p, e->secs) > p->claimPpb)) { return DRIFT_SHORT; }
  r->ppb = DriftPpb(e->accumUs, e->secs);
  r->secs = e->secs;
  r->bar = DriftSpanBar(e, p, uptime);
  if (e->secs < r->bar) { return DRIFT_HELD; }
  r->errUs = DriftSat32((int64_t)e->accumUs - (int64_t)e->secs * 1000000);
  r->resPpb = DriftResPpb(p, e->secs);
  e->spanBest = e->secs;
  e->lostUptime = 0;
  return DRIFT_PUBLISH;
}

#endif // DRIFT_EST_H
