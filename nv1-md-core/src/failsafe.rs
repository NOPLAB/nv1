/// Tracks freshness of incoming Hub commands so the control loop can
/// fall back to a safe state when communication stalls.
///
/// The caller is responsible for supplying a monotonic millisecond
/// timestamp (e.g. from `embassy_time::Instant::now().as_millis()`).
#[derive(Clone, Copy, Debug)]
pub struct CommHealth {
    last_seen_ms: Option<u64>,
    timeout_ms: u64,
}

impl CommHealth {
    pub const fn new(timeout_ms: u64) -> Self {
        Self {
            last_seen_ms: None,
            timeout_ms,
        }
    }

    /// Record that a valid command was received at `now_ms`.
    pub fn mark_received(&mut self, now_ms: u64) {
        self.last_seen_ms = Some(now_ms);
    }

    /// Force the comm into a "lost" state without waiting for timeout.
    /// Call this on framing errors, parse failures, or external triggers.
    pub fn invalidate(&mut self) {
        self.last_seen_ms = None;
    }

    /// `true` if a recent valid command was seen within `timeout_ms`.
    pub fn is_alive(&self, now_ms: u64) -> bool {
        match self.last_seen_ms {
            None => false,
            Some(t) => now_ms.saturating_sub(t) < self.timeout_ms,
        }
    }

    pub fn timeout_ms(&self) -> u64 {
        self.timeout_ms
    }

    pub fn last_seen_ms(&self) -> Option<u64> {
        self.last_seen_ms
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_dead() {
        let h = CommHealth::new(100);
        assert!(!h.is_alive(0));
        assert!(!h.is_alive(50));
    }

    #[test]
    fn alive_after_mark_received() {
        let mut h = CommHealth::new(100);
        h.mark_received(10);
        assert!(h.is_alive(10));
        assert!(h.is_alive(50));
        assert!(h.is_alive(109));
    }

    #[test]
    fn dies_at_or_after_timeout() {
        let mut h = CommHealth::new(100);
        h.mark_received(0);
        assert!(h.is_alive(99));
        assert!(!h.is_alive(100));
        assert!(!h.is_alive(101));
    }

    #[test]
    fn invalidate_kills_immediately() {
        let mut h = CommHealth::new(100);
        h.mark_received(10);
        assert!(h.is_alive(10));
        h.invalidate();
        assert!(!h.is_alive(10));
    }

    #[test]
    fn fresh_message_revives() {
        let mut h = CommHealth::new(100);
        h.mark_received(0);
        assert!(!h.is_alive(200));
        h.mark_received(200);
        assert!(h.is_alive(250));
    }

    #[test]
    fn clock_skew_does_not_underflow() {
        let mut h = CommHealth::new(100);
        h.mark_received(1_000);
        // Earlier timestamp than last_seen — saturating_sub keeps us alive.
        assert!(h.is_alive(999));
        // saturating_sub returns 0, which is < timeout, so still alive.
    }
}
