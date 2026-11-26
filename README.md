# xmip-core-transport-obd-ii

OBD-II transport: SAE J1979 over ISO-TP — mode 01 parameters and mode 09 vehicle information asked at the functional address 0x7DF and answered from 0x7E8; a Receive Location polls one, a Send Location answers one as the ECU. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

A send target is read by `net::Target` in [xmip-core-library-net](https://github.com/IlleNilsson/xmip-core-library-net), the one reading of a URI every technology calls: scheme, authority, path and decoded query. Until 2026-09-28 it was read through the transport capability's `socket::target`, which split it on its first slash and left the query in the path.

A `0x` number in a target is read by `codec::hex::prefixed_number` in [xmip-core-library-codec](https://github.com/IlleNilsson/xmip-core-library-codec), which refuses a sign; until 2026-09-28 it was read with `from_str_radix`, which took `0x+7e8`.

## Acknowledgement

A receive is a poll of one parameter, which consumes nothing at the ECU. Its
verdict therefore has nothing to tell the ECU, whichever it is: a receive cycle
that did not complete loses nothing, and the next poll asks again. The
parameter's data arrives whole.

## The declaration

What a Location of this technology is configured with is two `const`s in its `declaration` module, `src/declaration.rs`, beside the code, which `Configured` reads (ADR-0064, amendment 2026-11-26): `declaration::SETTINGS`, `mode`, `pid`, `request_id`, `response_id` and `timeout`, and `SCHEMES`, `obd`; the mode, the parameter and the identifiers those default to — vehicle information, the ECU name, the functional address and the first ECU's, `declaration::VEHICLE_INFORMATION`, `ECU_NAME`, `FUNCTIONAL` and `ECU` — are declared there too, where the public three were the parameter's (`pid.rs`) until then and the crate root names them as before, and the wait is the iso-tp technology's, which this crate's declaration takes by that crate's own `declaration` feature. Built with its `declaration` feature alone (`--no-default-features --features declaration`), the crate is those and nothing more: its code and every crate the code is built on — iso-tp's code and the can-bus technology beneath it among them — are behind the default feature. So the library the surfaces load carries it without the code, through the gathering crate xmip-core-declaration (open problem 48), and the language server and the desktop suggest and check an OBD-II Location's settings though no node in their process loaded the technology. `test/Declaration.Test.ps1` in the estate holds that the declaration builds nothing the transport capability does not.

## Toolchain

`rust-toolchain.toml` pins the toolchain for the whole estate. Do not change it
here.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.
