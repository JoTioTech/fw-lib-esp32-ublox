# ESP32 GPS library for ublox library

Following codebase is in early development stage and is not yet ready for production use.
Basic functionality like getting position, velocity or changing basic parameters work.
However even to access the data user must know form of internal structures requiring some deeper knowledge of ublox protocol and this codebase.
Everything is designed primarily from Ublox M9V module, however given that UBX protocol is quite universal it should work with other Ublox devices as well.

Basic parser structure has been forked from [UBLOX_read](https://github.com/superjax/UBLOX_read).

## Roadmap
* [X] functions that extract basic information easily
* [X] dead reckoning
* [ ] geofencing
* [X] gyro access


# Hot start
* Save-on-shutdown: page 58 of integration manual
