"use strict";

const fs = require("fs");
const path = require("path");
const dashboard = require("../dashboard/decision_dashboard.js");

if (process.argv.length !== 3) {
  throw new Error("usage: node test_dashboard_evidence.js EVIDENCE_JSON");
}

const evidencePath = path.resolve(process.argv[2]);
const data = JSON.parse(fs.readFileSync(evidencePath, "utf8"));
const model = dashboard.buildDecisionViewModel(data);

if (model.samples < 50) throw new Error("too few replay samples");
if (model.avoidCount <= 0) throw new Error("traffic avoidance was not exercised");
if (model.rejoinCount <= 0) throw new Error("mission rejoin was not exercised");
if (model.fullRtaSamples <= 0) throw new Error("no fully assured selected candidate");
if (model.candidates.length !== 3) throw new Error("expected LEFT/RIGHT/CLIMB");
if (!model.ml.enabled) throw new Error("ML advisory is not enabled");
if (!model.ml.inferenceValid) throw new Error("ML inference evidence is invalid");
if (model.ml.modelSha256.length !== 64) throw new Error("ML model SHA-256 missing");
if (model.ml.usedForSelection && !model.ml.proposalAccepted) {
  throw new Error("ML selection used without accepted proposal");
}
if (model.ml.features.length !== 3 ||
    model.ml.features.some((features) => features.length !== 8)) {
  throw new Error("ML feature vectors missing from audit evidence");
}

const selected = model.candidates.filter((candidate) => candidate.selected);
if (selected.length !== 1) throw new Error("dashboard must show exactly one selected candidate");
if (selected[0].fullRta !== "PASS") throw new Error("selected candidate is not Full-RTA safe");
if (selected[0].traffic !== "PASS") throw new Error("selected candidate traffic gate failed");
if (selected[0].terrain !== "PASS") throw new Error("selected candidate terrain gate failed");
if (selected[0].geofence !== "PASS") throw new Error("selected candidate geofence gate failed");
if (selected[0].envelope !== "PASS") throw new Error("selected candidate envelope gate failed");

console.log(
  "[PASS] dashboard evidence: " +
  model.decision + " / " + selected[0].name +
  " with all Full RTA gates PASS and auditable ML provenance"
);
