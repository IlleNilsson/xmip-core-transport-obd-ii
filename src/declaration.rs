//! What a Location of this technology is configured with: its declaration,
//! which `Configured` reads, built with the crate's `declaration` feature
//! alone and the code behind the default (ADR-0064, amendments 2026-10-22
//! and 2026-11-26), for the library the surfaces load to carry (open
//! problem 48). The parameter and the identifiers a Location takes unless
//! told otherwise are declared here, and the code takes them from here;
//! the wait is ISO-TP's, taken by that crate's own `declaration` feature.

use iso_tp::declaration::DEFAULT_TIMEOUT;
use xcore::settings::{Applies, Fixed, Kind, Presence, Setting, Settings};

/// Mode 09: vehicle information.
pub const VEHICLE_INFORMATION: u8 = 0x09;
/// The functional address a tester asks every emissions ECU at.
pub const FUNCTIONAL: u32 = 0x7df;
/// Where the first ECU answers from.
pub const ECU: u32 = 0x7e8;

/// The parameter a Location polls or answers unless told otherwise, of
/// vehicle information: the ECU name, information type `0a`, the one
/// vehicle information item a manufacturer fills freely.
pub(crate) const ECU_NAME: u8 = 0x0a;

/// A byte of a parameter: its mode or its number.
const BYTE: Kind = Kind::Integer {
    minimum: 0,
    maximum: 255,
};

/// A CAN identifier, extended above `0x7ff`.
const CAN_ID: Kind = Kind::Integer {
    minimum: 0,
    maximum: 0x1fff_ffff,
};

/// Every setting a Location takes beyond its address, the CAN interface,
/// `can0`, the ISO-TP link rides. A Receive Location is the tester and
/// transmits at the functional address; a Send Location is the ECU and
/// answers from its own.
pub const SETTINGS: &Settings = &Settings {
    technology: env!("CARGO_PKG_NAME"),
    settings: &[
        Setting {
            name: "mode",
            kind: BYTE,
            presence: Presence::Default(Fixed::Integer(VEHICLE_INFORMATION as i64)),
            meaning: "The mode of the parameter polled or answered: 1 current data, 9 \
                      vehicle information.",
            applies: Applies::Both,
        },
        Setting {
            name: "pid",
            kind: BYTE,
            presence: Presence::Default(Fixed::Integer(ECU_NAME as i64)),
            meaning: "The number of the parameter polled or answered within its mode.",
            applies: Applies::Both,
        },
        Setting {
            name: "request_id",
            kind: CAN_ID,
            presence: Presence::Default(Fixed::Integer(FUNCTIONAL as i64)),
            meaning: "The CAN identifier a tester asks under.",
            applies: Applies::Receive,
        },
        Setting {
            name: "response_id",
            kind: CAN_ID,
            presence: Presence::Default(Fixed::Integer(ECU as i64)),
            meaning: "The CAN identifier an ECU answers from.",
            applies: Applies::Send,
        },
        transport::configured::timeout(
            Applies::Both,
            Some(DEFAULT_TIMEOUT),
            "How long a peer that stops mid-message is waited on; the ISO-TP \
             link's own when left out.",
        ),
    ],
};

/// The schemes a Location's address is entered with to choose this technology.
pub const SCHEMES: &[&str] = &["obd"];
