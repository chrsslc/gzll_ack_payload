Goal: 
Create a logging system that collects data, wirelessly transmits it, and displays/stores data in the cloud.

Steps:
  1. collect Data using an NRF52840DK (Temperature, button click, NFC activity)
  2. send that data over 2.4 Ghz radio (NRF's Gazelle in this case) to another nrf52840dk
  3. send data packets from the second NRF to a RPI using SPI
  4. Retrieve SPI data, and trasmit to an AWS server (Elastic Beanstalk) via Flask API
  5. Display data in the cloud using Flask 


TODO:
  Cleanup/Refactor Host Code
  complete AWS EB Flask API and Integrate with RPI via HTTP
