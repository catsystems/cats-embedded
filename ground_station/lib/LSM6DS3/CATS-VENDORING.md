# CATS LSM6DS3 delta

This library is rebased on Arduino_LSM6DS3 1.0.3, upstream commit
`70306716de488acf3135965e4fb5889a57daebcf`.

The local compatibility delta is limited to:

- retaining the existing default constructor and `begin(TwoWire&, uint8_t)` API;
- accepting the Ground Station sensor identities `0x69` and `0x6B`;
- never starting or ending the shared `TwoWire` bus from this library.
