Nordic nRF52 BLE - Continuous data transmission
===============================================

This is an example code to demonstrate how to use Nordic nRF52 BLE chip for high data rate applications. It creates a custom service (AIS) which is intialised with a single data characteristic. Within the main section of the code, a sequence of numbers is contniously updated and transmitted. The packets are continuously transmitted after a connection is made for as long as there is no error and the previous packet gets succesfully transmitted.


##Notes##

* SDK version: nRF_SDK_11.0.0-2.alpha
* SoftDevice: SD132-SD-v2 (2.0.0)
* The code is only for PCA10026 board and must be modified for others
* The project is created in Keil uVision v5.17.0.0 (MDK-Lite v5.17)
* The main project file is in `ble_app_ais/pca10036\s132/arm5_no_packs`
* Refer to application note [nAN-36](https://www.nordicsemi.com/eng/layout/set/print/nordic/download_resource/24020/5/14433281) for getting started with creating a custom service
* On the receiver side, the code has been tested with an iOS app and shows 6 packets being receieved per connection interval (30ms)

## License

&copy; Syed Anas Imtiaz | 2016 | MIT License – [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT)

