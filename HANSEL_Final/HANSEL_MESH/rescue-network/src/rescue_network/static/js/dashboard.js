"use strict";

// Rescue-team dashboard. Prefers a live Server-Sent Events stream (push); falls
// back to periodic polling if EventSource is unavailable or the stream errors.
// All values are inserted via textContent (never innerHTML) so field-node text
// can never be injected as markup.

(function () {
  const POLL_MS = 5000;
  const rowsEl = document.getElementById("rows");
  const countEl = document.getElementById("count");
  const updatedEl = document.getElementById("updated");
  const errorEl = document.getElementById("error");

  const INJURY_LABEL = { yes: "부상 있음", no: "부상 없음", unknown: "모름" };
  // request_id -> item, so stream upserts never duplicate a row.
  const byId = new Map();

  function fmtTime(iso) {
    if (!iso) return "-";
    const d = new Date(iso);
    return Number.isNaN(d.getTime()) ? iso : d.toLocaleString();
  }

  function shortId(id) {
    return typeof id === "string" && id.length > 8 ? id.slice(0, 8) + "…" : id || "-";
  }

  function cell(text, opts) {
    const td = document.createElement("td");
    td.textContent = text === null || text === undefined || text === "" ? "-" : String(text);
    if (opts && opts.title) td.title = String(opts.title);
    if (opts && opts.className) td.className = opts.className;
    return td;
  }

  function render() {
    // Newest first by received_at.
    const items = Array.from(byId.values()).sort((a, b) =>
      String(b.received_at).localeCompare(String(a.received_at))
    );
    rowsEl.replaceChildren();
    if (!items.length) {
      const tr = document.createElement("tr");
      const td = cell("수신된 구조 요청이 없습니다.");
      td.colSpan = 9;
      td.className = "empty";
      tr.appendChild(td);
      rowsEl.appendChild(tr);
    } else {
      for (const it of items) {
        const tr = document.createElement("tr");
        if (it.injury_status === "yes") tr.className = "row-injured";
        tr.appendChild(cell(fmtTime(it.received_at)));
        tr.appendChild(cell(shortId(it.request_id), { title: it.request_id }));
        tr.appendChild(cell(it.source_node_id));
        tr.appendChild(cell(it.people_count));
        tr.appendChild(cell(INJURY_LABEL[it.injury_status] || it.injury_status));
        tr.appendChild(cell(it.condition));
        tr.appendChild(cell(it.message));
        tr.appendChild(cell(it.location_text));
        tr.appendChild(cell(fmtTime(it.original_created_at)));
        rowsEl.appendChild(tr);
      }
    }
    countEl.textContent = String(byId.size);
    updatedEl.textContent = "갱신: " + new Date().toLocaleTimeString();
  }

  function upsert(item) {
    if (item && item.request_id) byId.set(item.request_id, item);
  }

  // --- live stream (preferred) ---
  function startStream() {
    if (typeof EventSource === "undefined") return startPolling();
    let es;
    try {
      es = new EventSource("/api/received/stream");
    } catch (e) {
      return startPolling();
    }
    es.onmessage = function (ev) {
      try {
        upsert(JSON.parse(ev.data));
        render();
        errorEl.hidden = true;
      } catch (e) {
        /* ignore a malformed frame */
      }
    };
    es.onerror = function () {
      // Stream dropped — degrade to polling.
      es.close();
      errorEl.hidden = false;
      errorEl.textContent = "실시간 연결이 끊겨 주기적 갱신으로 전환했습니다.";
      startPolling();
    };
  }

  // --- polling fallback ---
  let pollTimer = null;
  async function pollOnce() {
    try {
      const res = await fetch("/api/received", { headers: { Accept: "application/json" } });
      if (!res.ok) throw new Error("HTTP " + res.status);
      const items = await res.json();
      for (const it of items) upsert(it);
      render();
      errorEl.hidden = true;
    } catch (e) {
      errorEl.hidden = false;
      errorEl.textContent = "데이터를 불러오지 못했습니다: " + e.message;
    }
  }
  function startPolling() {
    if (pollTimer) return;
    pollOnce();
    pollTimer = setInterval(pollOnce, POLL_MS);
  }

  render();
  startStream();
})();
