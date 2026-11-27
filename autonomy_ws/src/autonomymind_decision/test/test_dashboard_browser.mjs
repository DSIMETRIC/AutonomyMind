import crypto from "node:crypto";
import fs from "node:fs";
import http from "node:http";
import path from "node:path";
import process from "node:process";
import { chromium } from "playwright";

const EXPECTED_SCREENSHOT_SHA256 = "";

function contentType(filePath) {
  if (filePath.endsWith(".html")) return "text/html; charset=utf-8";
  if (filePath.endsWith(".js")) return "text/javascript; charset=utf-8";
  if (filePath.endsWith(".json")) return "application/json; charset=utf-8";
  return "application/octet-stream";
}

function assert(condition, message) {
  if (!condition) {
    throw new Error(message);
  }
}

async function main() {
  if (process.argv.length !== 5) {
    throw new Error(
      "usage: node test_dashboard_browser.mjs DASHBOARD_DIR EVIDENCE_JSON SCREENSHOT_PNG"
    );
  }

  const dashboardDir = path.resolve(process.argv[2]);
  const evidencePath = path.resolve(process.argv[3]);
  const screenshotPath = path.resolve(process.argv[4]);

  assert(fs.existsSync(path.join(dashboardDir, "index.html")), "dashboard index missing");
  assert(fs.existsSync(path.join(dashboardDir, "decision_dashboard.js")), "dashboard JS missing");
  assert(fs.existsSync(evidencePath), "dashboard evidence missing");

  // Serve the dashboard over HTTP so fetch("evidence.json") exercises the same
  // browser security model used by a real dashboard deployment.
  const server = http.createServer((request, response) => {
    const urlPath = new URL(request.url, "http://127.0.0.1").pathname;
    let filePath;

    if (urlPath === "/" || urlPath === "/index.html") {
      filePath = path.join(dashboardDir, "index.html");
    } else if (urlPath === "/decision_dashboard.js") {
      filePath = path.join(dashboardDir, "decision_dashboard.js");
    } else if (urlPath === "/evidence.json") {
      filePath = evidencePath;
    } else {
      response.writeHead(404);
      response.end("not found");
      return;
    }

    response.writeHead(200, { "Content-Type": contentType(filePath) });
    response.end(fs.readFileSync(filePath));
  });

  await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
  const address = server.address();
  const dashboardUrl = `http://127.0.0.1:${address.port}/index.html`;

  const browser = await chromium.launch({ headless: true });
  const page = await browser.newPage({
    viewport: { width: 1440, height: 1000 },
    deviceScaleFactor: 1,
  });

  const pageErrors = [];
  const consoleErrors = [];
  page.on("pageerror", (error) => pageErrors.push(String(error)));
  page.on("console", (message) => {
    if (message.type() === "error") {
      consoleErrors.push(message.text());
    }
  });

  try {
    await page.goto(dashboardUrl, { waitUntil: "networkidle" });

    // DOM assertions prove that the browser-rendered HMI agrees with the
    // evidence produced by the Decision runtime, rather than only testing the
    // view-model function in Node.
    const summary = await page.locator("#summary").innerText();
    assert(summary.includes("Decision: AVOID_RIGHT"), "dashboard decision is not AVOID_RIGHT");
    assert(summary.includes("Samples: 101"), "dashboard sample count is not 101");

    const mlText = await page.locator("#ml").innerText();
    assert(mlText.includes("ML: ENABLED"), "dashboard ML advisory is not enabled");
    assert(mlText.includes("sha256:"), "dashboard ML model provenance is missing");

    const cards = page.locator(".candidate");
    assert(await cards.count() === 3, "dashboard must render LEFT/RIGHT/CLIMB");

    const selected = page.locator(".candidate.selected");
    assert(await selected.count() === 1, "dashboard must render exactly one selected candidate");
    assert((await selected.getAttribute("data-candidate")) === "RIGHT", "RIGHT must be selected");

    const selectedText = await selected.innerText();
    for (const gate of ["Dynamic", "Traffic", "Terrain", "Geofence", "Envelope", "Full RTA"]) {
      const expression = new RegExp(`${gate}\\s+PASS`);
      assert(expression.test(selectedText), `${gate} gate is not PASS in rendered dashboard`);
    }

    assert(pageErrors.length === 0, `page errors: ${pageErrors.join(" | ")}`);
    assert(consoleErrors.length === 0, `console errors: ${consoleErrors.join(" | ")}`);

    // Capture a deterministic full-page image. The first run records its hash;
    // once the baseline is committed, subsequent runs become a pixel-level
    // visual regression gate on the pinned browser/runner combination.
    await page.screenshot({
      path: screenshotPath,
      fullPage: true,
      animations: "disabled",
    });

    const screenshotBytes = fs.readFileSync(screenshotPath);
    const screenshotSha256 = crypto
      .createHash("sha256")
      .update(screenshotBytes)
      .digest("hex");

    console.log(`[INFO] dashboard screenshot sha256: ${screenshotSha256}`);

    if (EXPECTED_SCREENSHOT_SHA256) {
      assert(
        screenshotSha256 === EXPECTED_SCREENSHOT_SHA256,
        `dashboard screenshot regression: expected ${EXPECTED_SCREENSHOT_SHA256}, got ${screenshotSha256}`
      );
    }

    console.log(
      "[PASS] Chromium dashboard DOM: Decision + Full RTA + auditable ML provenance rendered"
    );
  } finally {
    await browser.close();
    await new Promise((resolve) => server.close(resolve));
  }
}

main().catch((error) => {
  console.error(error.stack || String(error));
  process.exit(1);
});
