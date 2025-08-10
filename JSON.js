


function sendUpdate() {
  const version = document.getElementById("version").value;
  const url = document.getElementById("url").value;
  const token = document.getElementById("token").value;

  const data = {
    version: version,
    url: url,
    token: token
  };

  fetch("http://10.145.4.97/update", {
    method: "POST",
    headers: {
      "Content-Type": "application/json"
    },
    body: JSON.stringify(data)
  })
  .then(response => {
    if (response.ok) {
      alert("✅ Update sent successfully to ESP32!");
    } else {
      alert("⚠️ Failed to send update. ESP32 may be offline.");
    }
  })
  .catch(error => {
    alert("❌ Error: " + error.message);
  });
}