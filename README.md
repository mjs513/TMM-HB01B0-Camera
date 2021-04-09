# TMM-HB01B0-Camera
Supports Teensy MicroMod Machine Learning Carrier Board with the HB01B0 Camera.

The driver is primarily ported from the OpenMV HB01B0 driver. "The OpenMV project aims at making machine vision more accessible to beginners by developing a user-friendly, open-source, low-cost machine vision platform."  The developers were kind enough to make their hard work open-source under MIT License:

>The MIT License (MIT)
>
>Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
>Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
>
>Permission is hereby granted, free of charge, to any person obtaining a copy
>of this software and associated documentation files (the "Software"), to deal
>in the Software without restriction, including without limitation the rights
>to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
>copies of the Software, and to permit persons to whom the Software is
>furnished to do so, subject to the following conditions:
>
>The above copyright notice and this permission notice shall be included in
>all copies or substantial portions of the Software.
>....
>

Calibration of AutoExposure came from the existing Sparkfun library for the HB01B0.

For more information on functions see the example provided with the library as well as: https://docs.openmv.io/library/omv.sensor.html.

Teensy DMA support also has been add to the library by PJRC user @KurtE based on previous camera work.  

See the PJRC Forum Thread MicroMod Beta Testing (https://forum.pjrc.com/threads/66771-MicroMod-Beta-Testing) for TMM.
