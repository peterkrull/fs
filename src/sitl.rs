//! Lockstep with ArduPilot SITL through its JSON interface.
//!
//! ArduPilot sends its servo outputs as a binary packet over UDP, and waits for the simulator
//! to step by its frame time and reply with the sensor data as a line of JSON. The simulation
//! thereby advances exactly as fast as the flight controller runs, faster or slower than real
//! time. See `libraries/SITL/examples/JSON/readme.md` in the ArduPilot repository.
//!
//! Start ArduPlane with `sim_vehicle.py -v ArduPlane --model JSON:127.0.0.1`.

use std::{
    io,
    net::{SocketAddr, UdpSocket},
    time::Duration,
};

use serde_json::json;

use crate::sim::{STEP, Sensors};

/// The port ArduPilot sends its servo outputs to.
pub const PORT: u16 = 9002;

const MAGIC_16: u16 = 18458;
const MAGIC_32: u16 = 29569;

#[derive(Debug, Clone, PartialEq)]
pub struct ServoPacket {
    /// Rate at which ArduPilot steps the simulation, in Hz.
    pub frame_rate: u16,
    pub frame_count: u32,
    /// Pulse widths in microseconds, from servo output 1.
    pub pwm: Vec<u16>,
}

impl ServoPacket {
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let u16_at = |i: usize| Some(u16::from_le_bytes(bytes.get(i..i + 2)?.try_into().ok()?));
        let channels = match u16_at(0)? {
            MAGIC_16 => 16,
            MAGIC_32 => 32,
            _ => return None,
        };
        if bytes.len() < 8 + channels * 2 {
            return None;
        }
        Some(Self {
            frame_rate: u16_at(2)?,
            frame_count: u32::from_le_bytes(bytes[4..8].try_into().ok()?),
            pwm: (0..channels)
                .map(|i| u16_at(8 + i * 2))
                .collect::<Option<_>>()?,
        })
    }

    /// The encoding ArduPilot uses, for testing.
    pub fn encode(&self) -> Vec<u8> {
        let magic = if self.pwm.len() > 16 {
            MAGIC_32
        } else {
            MAGIC_16
        };
        let channels = if self.pwm.len() > 16 { 32 } else { 16 };
        let mut bytes = Vec::with_capacity(8 + channels * 2);
        bytes.extend(magic.to_le_bytes());
        bytes.extend(self.frame_rate.to_le_bytes());
        bytes.extend(self.frame_count.to_le_bytes());
        for i in 0..channels {
            bytes.extend(self.pwm.get(i).copied().unwrap_or(0).to_le_bytes());
        }
        bytes
    }

    /// The number of simulation steps, and their length, to advance by one frame.
    pub fn steps(&self) -> (usize, f64) {
        let frame = 1.0 / self.frame_rate.max(1) as f64;
        let steps = (frame / STEP).ceil().max(1.0) as usize;
        (steps, frame / steps as f64)
    }
}

/// How a received frame relates to the previous one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Frame {
    /// The next frame, to step the simulation for.
    Next,
    /// The same frame again, as ArduPilot resends its outputs when it gets no reply.
    Repeat,
    /// ArduPilot restarted, and counts its frames from the start again.
    Restart,
}

#[derive(Debug, Default)]
pub struct FrameTracker {
    last: Option<u32>,
}

impl FrameTracker {
    pub fn track(&mut self, frame_count: u32) -> Frame {
        let frame = match self.last {
            Some(last) if frame_count == last => Frame::Repeat,
            Some(last) if frame_count < last => Frame::Restart,
            _ => Frame::Next,
        };
        self.last = Some(frame_count);
        frame
    }
}

/// The sensor data in the format ArduPilot expects. Its parser looks for keys by searching the
/// text, so the line must not contain spaces after the colons, and `velocity` must come before
/// `velocity_wind`, which the sorted keys of `serde_json` ensure.
pub fn sensor_json(sensors: &Sensors) -> String {
    let v = |v: bevy::math::DVec3| [finite(v.x), finite(v.y), finite(v.z)];
    let q = sensors.attitude_ned;
    let value = json!({
        "timestamp": sensors.time,
        "imu": {
            "gyro": v(sensors.gyro),
            "accel_body": v(sensors.accel),
        },
        "latitude": sensors.latitude,
        "longitude": sensors.longitude,
        "altitude": sensors.altitude,
        "velocity": v(sensors.velocity_ned),
        "quaternion": [finite(q.w), finite(q.x), finite(q.y), finite(q.z)],
        "airspeed": finite(sensors.airspeed),
        "velocity_wind": v(sensors.wind_ned),
        "battery": {
            "voltage": finite(sensors.battery_voltage),
            "current": finite(sensors.battery_current),
        },
    });
    format!("\n{value}\n")
}

fn finite(value: f64) -> f64 {
    if value.is_finite() { value } else { 0.0 }
}

/// The UDP socket ArduPilot talks to.
pub struct Link {
    socket: UdpSocket,
    buffer: Vec<u8>,
}

impl Link {
    pub fn bind(port: u16) -> io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", port))?;
        socket.set_read_timeout(Some(Duration::from_millis(50)))?;
        Ok(Self {
            socket,
            buffer: vec![0; 1024],
        })
    }

    /// Wait for the next servo packet, giving up after a short while so that the caller can
    /// handle other things.
    pub fn receive(&mut self) -> Option<(ServoPacket, SocketAddr)> {
        loop {
            match self.socket.recv_from(&mut self.buffer) {
                Ok((len, from)) => match ServoPacket::parse(&self.buffer[..len]) {
                    Some(packet) => return Some((packet, from)),
                    None => log::warn!("Ignoring a packet of {len} bytes from {from}"),
                },
                Err(err)
                    if matches!(
                        err.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    return None;
                }
                Err(err) => {
                    log::warn!("Unable to receive from ArduPilot: {err}");
                    return None;
                }
            }
        }
    }

    pub fn send(&self, to: SocketAddr, text: &str) {
        if let Err(err) = self.socket.send_to(text.as_bytes(), to) {
            log::warn!("Unable to send to ArduPilot at {to}: {err}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_roundtrip() {
        let packet = ServoPacket {
            frame_rate: 1200,
            frame_count: 42,
            pwm: (0..16).map(|i| 1000 + i * 10).collect(),
        };
        let bytes = packet.encode();
        assert_eq!(bytes.len(), 40);
        assert_eq!(ServoPacket::parse(&bytes), Some(packet));
        assert_eq!(ServoPacket::parse(&bytes[..30]), None);
    }

    #[test]
    fn frame_tracking() {
        let mut tracker = FrameTracker::default();
        assert_eq!(tracker.track(1), Frame::Next);
        assert_eq!(tracker.track(2), Frame::Next);
        assert_eq!(tracker.track(2), Frame::Repeat);
        assert_eq!(tracker.track(5), Frame::Next);
        assert_eq!(tracker.track(0), Frame::Restart);
    }

    #[test]
    fn steps_per_frame() {
        let packet = |frame_rate| ServoPacket {
            frame_rate,
            frame_count: 0,
            pwm: vec![],
        };
        assert_eq!(packet(1000).steps(), (1, 0.001));
        let (steps, dt) = packet(400).steps();
        assert_eq!(steps, 3);
        assert!((dt * 3.0 - 0.0025).abs() < 1e-12);
    }

    #[test]
    fn json_parses_like_ardupilot() {
        let text = sensor_json(&Sensors {
            time: 1.5,
            ..Default::default()
        });
        assert!(text.starts_with('\n') && text.ends_with('\n'));
        assert!(!text.contains(": "));
        let velocity = text.find("\"velocity\"").unwrap();
        let wind = text.find("\"velocity_wind\"").unwrap();
        assert!(velocity < wind);
        let value: serde_json::Value = serde_json::from_str(text.trim()).unwrap();
        assert_eq!(value["timestamp"], 1.5);
        assert_eq!(value["quaternion"][0], 1.0);
    }
}
