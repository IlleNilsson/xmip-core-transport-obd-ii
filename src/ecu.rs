//! One emissions ECU's worth of SAE J1979: the parameters it answers, and
//! the bitmaps that say which.
//!
//! An ECU answers a parameter it holds with the data it holds, parameter
//! `00` of any mode with the bitmap of the next thirty-two, and anything
//! else negatively: an unsupported mode, a parameter it does not have, a
//! request that is not a request. What it holds is set from outside — a
//! Send Location answering as the ECU sets the parameter it answers.

use std::collections::HashMap;
use std::sync::{Mutex, PoisonError};

use transport::error::Result;

use crate::pid::{CURRENT_DATA, Pid, SUPPORTED, VEHICLE_INFORMATION, code};

/// An ECU: what it answers, by parameter.
#[derive(Default)]
pub struct Ecu {
    answers: Mutex<HashMap<Pid, Vec<u8>>>,
}

impl Ecu {
    /// An ECU answering nothing but the supported-parameter bitmaps.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Answer `pid` with `data`.
    #[must_use]
    pub fn answering(self, pid: Pid, data: &[u8]) -> Self {
        self.set(pid, data);
        self
    }

    /// Answer `pid` with `data` from now on.
    pub fn set(&self, pid: Pid, data: &[u8]) {
        self.answers
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(pid, data.to_vec());
    }

    /// What `pid` is answered with.
    #[must_use]
    pub fn held(&self, pid: Pid) -> Option<Vec<u8>> {
        self.answers
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(&pid)
            .cloned()
    }

    /// The one response to `request`.
    ///
    /// # Errors
    /// Data held that no response can carry.
    pub fn answer(&self, request: &[u8]) -> Result<Vec<u8>> {
        let Ok(pid) = Pid::parse_request(request) else {
            let mode = request.first().copied().unwrap_or(0);
            return Ok(Pid { mode, pid: 0 }
                .negative(code::INCORRECT_MESSAGE_LENGTH)
                .to_vec());
        };
        if pid.mode != CURRENT_DATA && pid.mode != VEHICLE_INFORMATION {
            return Ok(pid.negative(code::SERVICE_NOT_SUPPORTED).to_vec());
        }
        if pid.pid == SUPPORTED {
            return pid.response(&self.supported(pid.mode));
        }
        match self.held(pid) {
            Some(data) => pid.response(&data),
            None => Ok(pid.negative(code::REQUEST_OUT_OF_RANGE).to_vec()),
        }
    }

    /// The bitmap of parameters `01` to `20` of `mode` this ECU answers:
    /// the highest bit of the first byte is parameter `01`.
    fn supported(&self, mode: u8) -> [u8; 4] {
        let mut bits: u32 = 0;
        for pid in self
            .answers
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .keys()
            .filter(|pid| pid.mode == mode && (1..=32).contains(&pid.pid))
        {
            bits |= 1 << (32 - u32::from(pid.pid));
        }
        bits.to_be_bytes()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_ecu_answers_what_it_holds_and_bitmaps_it() {
        let ecu = Ecu::new()
            .answering(Pid::current(0x0c), &[0x1a, 0xf8])
            .answering(Pid::current(0x05), &[0x5a])
            .answering(Pid::information(0x02), b"VIN");
        assert_eq!(
            ecu.answer(&[0x01, 0x0c]).expect("rpm"),
            [0x41, 0x0c, 0x1a, 0xf8]
        );
        assert_eq!(
            ecu.answer(&[0x01, 0x00]).expect("bitmap"),
            [0x41, 0x00, 0x08, 0x10, 0x00, 0x00],
            "parameters 05 and 0c are bits 5 and 12 from the top"
        );
        assert_eq!(
            ecu.answer(&[0x09, 0x00]).expect("bitmap"),
            [0x49, 0x00, 0x01, 0x40, 0x00, 0x00, 0x00]
        );
        assert_eq!(ecu.answer(&[0x09, 0x02]).expect("vin"), b"\x49\x02\x01VIN");
        ecu.set(Pid::information(0x02), b"NEW");
        assert_eq!(ecu.held(Pid::information(0x02)).expect("held"), b"NEW");
    }

    #[test]
    fn an_ecu_refuses_what_it_does_not_have_with_the_code_that_says_so() {
        let ecu = Ecu::default();
        assert_eq!(
            ecu.answer(&[0x01, 0x0c]).expect("unknown"),
            [0x7f, 0x01, 0x31]
        );
        assert_eq!(
            ecu.answer(&[0x22, 0xf1]).expect("no mode 22"),
            [0x7f, 0x22, 0x11]
        );
        assert_eq!(ecu.answer(&[0x09]).expect("short"), [0x7f, 0x09, 0x13]);
        assert_eq!(ecu.answer(&[]).expect("empty"), [0x7f, 0x00, 0x13]);
        let over = Ecu::new().answering(Pid::information(0x0a), &[0; 5000]);
        assert!(
            over.answer(&[0x09, 0x0a]).is_err(),
            "no response carries it"
        );
    }
}
