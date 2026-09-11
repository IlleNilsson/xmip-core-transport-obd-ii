# xmip-core-transport-obd-ii

OBD-II transport: SAE J1979 over ISO-TP — mode 01 parameters and mode 09 vehicle information asked at the functional address 0x7DF and answered from 0x7E8; a Receive Location polls one, a Send Location answers one as the ECU. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

## Toolchain

`rust-toolchain.toml` pins the toolchain for the whole estate. Do not change it
here.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.
