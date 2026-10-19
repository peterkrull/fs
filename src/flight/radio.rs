//! RC radios as game controllers, such as EdgeTX and OpenTX radios in their USB joystick mode.
//!
//! In its classic joystick mode, EdgeTX reports channels 1 to 8 as the axes X, Y, Z, Rx, Ry,
//! Rz, slider and dial, ranging from 0 to 2047, and channels 9 to 32 as buttons. On Linux,
//! gilrs has no mapping for these radios, and hands the first six axes to Bevy as stick axes,
//! with the Y axes inverted. The slider and dial are dropped, so channels 7 and 8 are not
//! available.
//!
//! Unlike the sticks of a gamepad, those of a radio do not spring back, so they set the
//! controls directly, including the throttle.
//!
//! The position of an axis is only known once it changes: gilrs does not read the axes of a
//! controller when it connects, and Bevy starts them centered. A radio filters its outputs, so a
//! stick that is not touched may not report at all. Until the throttle channel reports, the
//! throttle is therefore held closed, rather than taken to be half open.

use bevy::{
    input::gamepad::{AxisSettings, Gamepad, GamepadAxis, GamepadSettings},
    prelude::*,
};

/// The USB vendor and product ID of EdgeTX and OpenTX radios in joystick mode.
const EDGETX: (u16, u16) = (0x1209, 0x4F54);

/// The axes channels 1 to 6 arrive on, and the sign gilrs gave them.
const CHANNEL_AXES: [(GamepadAxis, f32); 6] = [
    (GamepadAxis::LeftStickX, 1.0),
    (GamepadAxis::LeftStickY, -1.0),
    (GamepadAxis::LeftZ, 1.0),
    (GamepadAxis::RightStickX, 1.0),
    (GamepadAxis::RightStickY, -1.0),
    (GamepadAxis::RightZ, 1.0),
];

pub const CHANNELS: usize = CHANNEL_AXES.len();

/// Which channels of the radio carry which controls.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
pub struct RadioMap {
    roll: Channel,
    pitch: Channel,
    yaw: Channel,
    throttle: Channel,
    order: [char; 4],
    /// Treat every game controller as a radio, not only those recognized as one.
    pub all_gamepads: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Channel {
    index: usize,
    sign: f64,
}

impl Default for RadioMap {
    /// The default channel order of EdgeTX.
    fn default() -> Self {
        let mut map = Self::parse("AETR").expect("a valid channel order");
        map.all_gamepads = false;
        map
    }
}

impl RadioMap {
    /// Parse a channel order such as `AETR` or `TAER`, of aileron, elevator, throttle and
    /// rudder on channels 1 to 4. A lowercase letter reverses that channel.
    pub fn parse(order: &str) -> Result<Self, String> {
        let letters: Vec<char> = order.chars().collect();
        let find = |letter: char| {
            let found: Vec<usize> = (0..letters.len())
                .filter(|i| letters[*i].eq_ignore_ascii_case(&letter))
                .collect();
            match found[..] {
                [index] if index < CHANNELS => Ok(Channel {
                    index,
                    sign: if letters[index].is_lowercase() {
                        -1.0
                    } else {
                        1.0
                    },
                }),
                _ => Err(format!(
                    "the radio channel order {order} needs each of A, E, T and R once, \
                     on the first {CHANNELS} channels"
                )),
            }
        };
        Ok(Self {
            roll: find('A')?,
            pitch: find('E')?,
            yaw: find('R')?,
            throttle: find('T')?,
            order: ['A', 'E', 'T', 'R'].map(|c| {
                letters
                    .iter()
                    .position(|l| l.eq_ignore_ascii_case(&c))
                    .map_or(c, |i| letters[i])
            }),
            all_gamepads: true,
        })
    }

    pub fn applies_to(&self, gamepad: &Gamepad) -> bool {
        self.all_gamepads
            || (gamepad.vendor_id(), gamepad.product_id()) == (Some(EDGETX.0), Some(EDGETX.1))
    }

    /// Roll, pitch, yaw and throttle from the channels, which range from `-1` to `1`, or are
    /// unknown until they report.
    ///
    /// Like the sticks themselves, the channels of EdgeTX are positive with the aileron stick
    /// right, the elevator stick forward, the throttle up and the rudder right. Pushing the
    /// elevator forward pitches down.
    pub fn controls(&self, channels: &[Option<f64>; CHANNELS]) -> [f64; 4] {
        let value = |c: Channel| channels[c.index].map(|v| v * c.sign);
        [
            value(self.roll).unwrap_or(0.0),
            -value(self.pitch).unwrap_or(0.0),
            value(self.yaw).unwrap_or(0.0),
            value(self.throttle).map_or(0.0, |v| (v + 1.0) / 2.0),
        ]
    }

    /// Whether all channels used for control have reported their position.
    pub fn synced(&self, channels: &[Option<f64>; CHANNELS]) -> bool {
        [self.roll, self.pitch, self.yaw, self.throttle]
            .iter()
            .all(|c| channels[c.index].is_some())
    }

    pub fn describe(&self) -> String {
        let mut order = vec![' '; CHANNELS];
        for (channel, letter) in [self.roll, self.pitch, self.throttle, self.yaw]
            .iter()
            .zip(self.order)
        {
            order[channel.index] = letter;
        }
        order.into_iter().collect::<String>().trim_end().to_string()
    }
}

/// Which channels of a radio have reported their position.
#[derive(Component, Default)]
pub struct Reported([bool; CHANNELS]);

/// Channels 1 to 6 of a radio, from `-1` to `1`, or `None` until they first report.
///
/// An axis that never reported reads exactly zero, and an axis that did almost never does, so
/// any other value marks a channel as reported.
pub fn channels(gamepad: &Gamepad, reported: &mut Reported) -> [Option<f64>; CHANNELS] {
    let mut channels = [None; CHANNELS];
    for (i, (axis, sign)) in CHANNEL_AXES.into_iter().enumerate() {
        let value = gamepad.get(axis).unwrap_or(0.0) * sign;
        reported.0[i] |= value != 0.0;
        if reported.0[i] {
            channels[i] = Some(value as f64);
        }
    }
    channels
}

/// Radios need every bit of their resolution, and center precisely, unlike gamepads.
pub(super) fn configure(
    mut commands: Commands,
    map: Res<RadioMap>,
    mut gamepads: Query<(Entity, &Gamepad, &mut GamepadSettings), Added<GamepadSettings>>,
) {
    for (entity, gamepad, mut settings) in &mut gamepads {
        if map.applies_to(gamepad) {
            settings.default_axis_settings =
                AxisSettings::new(-1.0, 0.0, 0.0, 1.0, 0.0).expect("valid axis settings");
            commands.entity(entity).insert(Reported::default());
            info!(
                "Using the game controller as an RC radio, with channels {}",
                map.describe()
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_order() {
        let map = RadioMap::default();
        assert!(!map.all_gamepads);
        // Aileron right, elevator pulled back, half throttle, rudder left
        let channels = [0.5, -1.0, 0.0, -0.25, 0.0, 0.0].map(Some);
        let [roll, pitch, yaw, throttle] = map.controls(&channels);
        assert_eq!((roll, pitch, yaw, throttle), (0.5, 1.0, -0.25, 0.5));
        assert_eq!(map.describe(), "AETR");
    }

    #[test]
    fn reordered_and_reversed() {
        let map = RadioMap::parse("TaER").unwrap();
        assert!(map.all_gamepads);
        let [roll, _, _, throttle] = map.controls(&[1.0, 0.3, 0.0, 0.0, 0.0, 0.0].map(Some));
        assert_eq!((roll, throttle), (-0.3, 1.0));
        assert_eq!(map.describe(), "TaER");
    }

    /// A throttle that has not reported is closed, not half open.
    #[test]
    fn unreported_channels() {
        let map = RadioMap::default();
        let channels = [Some(0.2), None, None, None, None, None];
        assert_eq!(map.controls(&channels), [0.2, 0.0, 0.0, 0.0]);
        assert!(!map.synced(&channels));
        assert!(map.synced(&[Some(0.0); CHANNELS]));
    }

    #[test]
    fn rejects_invalid_orders() {
        assert!(RadioMap::parse("AETX").is_err());
        assert!(RadioMap::parse("AAER").is_err());
        assert!(RadioMap::parse("AET").is_err());
        assert!(RadioMap::parse("xxxxxxAETR").is_err());
    }
}
