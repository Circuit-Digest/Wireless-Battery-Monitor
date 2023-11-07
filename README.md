# Wireless-Battery-Monitor
<img src="https://github.com/Circuit-Digest/Wireless-Battery-Monitor/blob/045d9b2cafc2a165b86609bafc7572758d66e6c8/Schematcs%20and%20PCB%20files/AD7280.png" width="" alt="alt_text" title="image_tooltip">
<br>

Batteries are everywhere and can be found in many of the day-to-day life equipment. Most of them are powered by Li ion or LIfepo4 batteries nowadays. Even though they need much less maintenance when compared to traditional Lead acid batteries, it is essential to regularly monitor them to avoid damage or even accidents. That’s where BMS comes into play. They not only provide safety but also help extend battery life. But the problem with most common BMSs is that, if we want to manually check the battery health, we will have to physically access it. There are BMS’s that offer wireless monitoring functionality, but the issue is that they are pretty expensive. That’s why we have decided to design and build a DIY wireless battery monitoring system, which can be used with the existing BMS.
For this project we have chosen the AD7280 Lithium-Ion Battery Monitoring chip from Analog Devices, and for wireless connectivity we have chosen our favourite ESP32. The AD7280 is capable of monitoring up to 6S battery configuration. Here we have configured it to monitor 4S battery configuration since that’s the most commonly used battery system. The AD7280 offers a 12bit ADC with a conversion time of 1 us per channel. It also features cell balancing function, which can be controlled manually. The ESP32 is used for communication and to host a Async webserver which can be used to monitor the cell voltages and charge levels.


<br>
[Note: As this projects are very simple we are only providing the code, schemaitic, and a few essential images if you want to get the images or code explanations do check out the Circuit Digest website.
<br>
<br>
