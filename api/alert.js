export default function handler(req, res) {
  if (req.method !== "POST") {
    return res.status(405).json({ error: "Method not allowed" });
  }

  const secret = req.headers["x-webhook-secret"];
  if (secret !== process.env.WEBHOOK_SECRET) {
    return res.status(401).json({ error: "Invalid secret" });
  }

  const { event, msg, ts } = req.body;

  console.log("📩 Webhook received:", { event, msg, ts });

  // You can later store it in a DB, send notifications, etc.
  return res.status(200).json({ ok: true });
}
