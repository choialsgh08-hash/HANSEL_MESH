"use strict";

// Progressive-enhancement client for the rescue form. Submits JSON to the
// intake API and reports the accepted/failed state accessibly (aria-live).

(function () {
  const form = document.getElementById("rescue-form");
  const btn = document.getElementById("submit-btn");
  const statusBox = document.getElementById("status");

  function showStatus(kind, text) {
    statusBox.hidden = false;
    statusBox.className = "status " + kind;
    statusBox.textContent = text;
  }

  function readForm() {
    const injuryEl = form.querySelector('input[name="injury_status"]:checked');
    const locationText = form.location_text.value.trim();
    return {
      people_count: Number.parseInt(form.people_count.value, 10),
      injury_status: injuryEl ? injuryEl.value : "unknown",
      condition: form.condition.value.trim(),
      message: form.message.value.trim(),
      location_text: locationText === "" ? null : locationText,
    };
  }

  // Turn FastAPI's 422 validation payload into a readable message.
  function formatValidationError(detail) {
    if (!Array.isArray(detail)) return "입력값을 확인해주세요.";
    return detail
      .map((d) => {
        const field = Array.isArray(d.loc) ? d.loc[d.loc.length - 1] : "입력";
        return `• ${field}: ${d.msg}`;
      })
      .join("\n");
  }

  form.addEventListener("submit", async function (event) {
    event.preventDefault();
    if (!form.reportValidity()) return;

    btn.disabled = true;
    showStatus("ok", "전송 중…");

    try {
      const res = await fetch("/api/rescue", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(readForm()),
      });

      if (res.status === 201) {
        const data = await res.json();
        showStatus(
          "ok",
          `${data.message}\n요청 번호: ${data.request_id}\n상태: ${data.delivery_status}`
        );
        form.reset();
      } else if (res.status === 422) {
        const err = await res.json();
        showStatus("err", "요청을 저장하지 못했습니다.\n" + formatValidationError(err.detail));
        btn.disabled = false;
      } else {
        showStatus("err", `요청을 저장하지 못했습니다. (오류 ${res.status})`);
        btn.disabled = false;
      }
    } catch (e) {
      showStatus("err", "네트워크 오류로 전송하지 못했습니다. 다시 시도해주세요.");
      btn.disabled = false;
    }
  });
})();
