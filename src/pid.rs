//! A parameter identifier of SAE J1979: the mode it belongs to, the request
//! that asks for it, and the response that answers.
//!
//! An OBD-II request is two bytes, a mode and a parameter: mode `01` for
//! current data — engine speed, coolant temperature, the supported-parameter
//! bitmaps — and mode `09` for vehicle information, the VIN, calibration
//! identifiers and the ECU name. A response echoes the mode with `0x40`
//! added and the parameter, and for mode `09` counts the data items before
//! them; a negative response is `0x7f`, the mode and a code, the ISO 15031-5
//! shape. One ISO-TP message carries it all, so a parameter's data is at
//! most the message's ceiling less that header.

use transport::ceiling;
use transport::error::{Result, protocol_error};

/// Mode 01: current data.
pub const CURRENT_DATA: u8 = 0x01;
/// Mode 09: vehicle information.
pub const VEHICLE_INFORMATION: u8 = 0x09;
/// What a positive response adds to the mode.
pub const POSITIVE: u8 = 0x40;
/// The first byte of every negative response.
pub const NEGATIVE: u8 = 0x7f;
/// The functional address a tester asks every emissions ECU at.
pub const FUNCTIONAL: u32 = 0x7df;
/// Where the first ECU answers from.
pub const ECU: u32 = 0x7e8;
/// The parameter in every mode that bitmaps the next thirty-two.
pub const SUPPORTED: u8 = 0x00;

/// The negative response codes an ECU answers with.
pub mod code {
    /// The mode is not one the ECU offers.
    pub const SERVICE_NOT_SUPPORTED: u8 = 0x11;
    /// The request is the wrong length.
    pub const INCORRECT_MESSAGE_LENGTH: u8 = 0x13;
    /// The parameter is not one the ECU has.
    pub const REQUEST_OUT_OF_RANGE: u8 = 0x31;
}

/// One parameter of one mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Pid {
    pub mode: u8,
    pub pid: u8,
}

impl Pid {
    /// Mode 01, parameter `pid`.
    #[must_use]
    pub const fn current(pid: u8) -> Self {
        Self {
            mode: CURRENT_DATA,
            pid,
        }
    }

    /// Mode 09, information type `pid`.
    #[must_use]
    pub const fn information(pid: u8) -> Self {
        Self {
            mode: VEHICLE_INFORMATION,
            pid,
        }
    }

    /// The two bytes that ask for this parameter.
    #[must_use]
    pub const fn request(self) -> [u8; 2] {
        [self.mode, self.pid]
    }

    /// The parameter a request asks for.
    ///
    /// # Errors
    /// A request that is not two bytes.
    pub fn parse_request(bytes: &[u8]) -> Result<Self> {
        match bytes {
            [mode, pid] => Ok(Self {
                mode: *mode,
                pid: *pid,
            }),
            _ => Err(protocol_error(
                "an OBD-II request is a mode and a parameter",
            )),
        }
    }

    /// How many bytes of a response are header: the mode, the parameter,
    /// and for vehicle information the count of items.
    #[must_use]
    pub const fn header(self) -> usize {
        if self.mode == VEHICLE_INFORMATION {
            3
        } else {
            2
        }
    }

    /// The most data this parameter's response carries in one ISO-TP
    /// message: the message's ceiling less the header.
    #[must_use]
    pub const fn ceiling(self) -> usize {
        iso_tp::frame::CLASSIC_CEILING - self.header()
    }

    /// The positive response carrying `data`.
    ///
    /// # Errors
    /// Data over [`Self::ceiling`].
    pub fn response(self, data: &[u8]) -> Result<Vec<u8>> {
        ceiling::within(data.len(), self.ceiling(), "one OBD-II response carries")?;
        let mut out = Vec::with_capacity(self.header() + data.len());
        out.push(self.mode | POSITIVE);
        out.push(self.pid);
        if self.mode == VEHICLE_INFORMATION {
            out.push(1);
        }
        out.extend_from_slice(data);
        Ok(out)
    }

    /// The negative response refusing this parameter with `code`.
    #[must_use]
    pub const fn negative(self, code: u8) -> [u8; 3] {
        [NEGATIVE, self.mode, code]
    }

    /// The data a response to this parameter carries.
    ///
    /// # Errors
    /// A negative response, one to another parameter, or one too short for
    /// its header.
    pub fn parse_response(self, bytes: &[u8]) -> Result<Vec<u8>> {
        if let [NEGATIVE, mode, code] = bytes {
            return Err(protocol_error(format!(
                "mode {mode:#04x} refused with code {code:#04x}"
            )));
        }
        let (head, data) = bytes
            .split_at_checked(self.header())
            .ok_or_else(|| protocol_error("a response too short for its header"))?;
        if head[0] != self.mode | POSITIVE || head[1] != self.pid {
            return Err(protocol_error(format!(
                "an answer to {:#04x}/{:#04x}, not {:#04x}/{:#04x}",
                head[0] & !POSITIVE,
                head[1],
                self.mode,
                self.pid
            )));
        }
        if self.mode == VEHICLE_INFORMATION && head[2] != 1 {
            return Err(protocol_error(
                "a vehicle information item counted other than once",
            ));
        }
        Ok(data.to_vec())
    }

    /// `obd://<bus>/0x<mode>/0x<pid>`.
    #[must_use]
    pub fn origin(self, bus: &str) -> String {
        format!("obd://{bus}/{:#04x}/{:#04x}", self.mode, self.pid)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_current_data_response_is_mode_and_parameter_then_data() {
        let rpm = Pid::current(0x0c);
        assert_eq!(rpm.request(), [0x01, 0x0c]);
        assert_eq!(Pid::parse_request(&[0x01, 0x0c]).expect("request"), rpm);
        let response = rpm.response(&[0x1a, 0xf8]).expect("response");
        assert_eq!(response, [0x41, 0x0c, 0x1a, 0xf8]);
        assert_eq!(rpm.parse_response(&response).expect("data"), [0x1a, 0xf8]);
        assert_eq!(rpm.ceiling(), 4093);
        assert_eq!(rpm.origin("loopback"), "obd://loopback/0x01/0x0c");
    }

    #[test]
    fn a_vehicle_information_response_counts_its_one_item() {
        let vin = Pid::information(0x02);
        let response = vin.response(b"WVWZZZ1JZXW000001").expect("response");
        assert_eq!(&response[..3], &[0x49, 0x02, 0x01]);
        assert_eq!(
            vin.parse_response(&response).expect("data"),
            b"WVWZZZ1JZXW000001"
        );
        assert_eq!(vin.response(&[]).expect("empty"), [0x49, 0x02, 0x01]);
        assert!(
            vin.parse_response(&[0x49, 0x02, 0x01])
                .expect("empty")
                .is_empty()
        );
        assert_eq!(vin.ceiling(), 4092);
        assert!(vin.response(&[0; 4093]).is_err(), "over the ceiling");
        assert!(
            vin.parse_response(&[0x49, 0x02, 0x02]).is_err(),
            "two items"
        );
    }

    #[test]
    fn a_response_that_does_not_answer_the_question_is_refused() {
        let vin = Pid::information(0x02);
        let error = vin
            .parse_response(&vin.negative(code::REQUEST_OUT_OF_RANGE))
            .expect_err("negative");
        assert!(error.message.contains("0x31"), "{error}");
        assert!(
            vin.parse_response(&[0x49, 0x04, 0x01]).is_err(),
            "another parameter"
        );
        assert!(
            vin.parse_response(&[0x41, 0x02, 0x01]).is_err(),
            "another mode"
        );
        assert!(vin.parse_response(&[0x49]).is_err(), "short");
        assert!(Pid::parse_request(&[0x09]).is_err(), "no parameter");
        assert!(Pid::parse_request(&[0x09, 0x02, 0x00]).is_err(), "too long");
    }
}
