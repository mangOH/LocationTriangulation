LocationTriangulation
=====================

This app provides a way to gather location data from the WiFi and Cellular interfaces
of a mangOH Red and publish that data to AirVantage using LWM2M.  This app
attempts to conserve bandwidth by publishing only when the location readings
exceed preprogrammed thresholds or if no readings have been published for quite
a while.
