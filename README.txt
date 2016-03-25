LATERO CONTROL CLASS

Latero.cpp contains a class which defines a latero device object. A latero is an eight by eight bed of nails array composed of piezos which can be deflected laterally. The coordinated activity of these deflections produce shear forces in any contacting object, usually skin, which are designed to simulate natural percepts. See the following paper,

Wang, Q. and Hayward V. Compact, Portable, Modular, High-performance, Distributed Tactile Transducer Device Based on Lateral Skin Deformation. Proc. 14th Symposium on Haptic Interfaces For Virtual Environment And Teleoperator Systems IEEE VR 2006, pp. 67-72, 2006

...and Tactile Labs website (http://tactilelabs.com/products/haptics/latero-tactile-display/) for more information on the latero device.

The Latero class contains methods used to connect to the latero device, run textures, and set the pins for different textures based on the current active row/col. Only three percepts are defined thus far: slip, roll, and pressure. I will post more as I create them. The class also contains other methods specific to the larger controller framework which it's deployed (isActionFinished, go, setupAction). This class is meant to be a part of a larger software system however, it is presented here as an example for other latero developers.

Directory "latero_winAPI" contains the latero API provided by Tactile Labs which I adapted for use in a windows environment. The original linux compatible API can be found here: https://github.com/Motsai/latero

Erik Schluter
March 25, 2016

