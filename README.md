# Underwater-Buoy-Detector: Geometric Shape Detector
** Created for ARVP buoy detection task, University of Alberta, Robosub 2017.
** ELSD algorithm codes modified based on https://github.com/viorik/ELSDc
# Overview

The buoy detector is actually a combination of shape detector and shape filter.

The geometric shape detector can detect elementary geometric shapes in the frame without manually tuning parameters.

The shape filter can select shapes which is more likely to be a buoy. Finally, the geometric center of the buoy can be obtained. Task accomplished.

![](https://lh4.googleusercontent.com/vrSjzf80lBfPkskNC5EsfzOsdmAQoOODPoC5KxxP5QrqrzvTeDgm6UruHLNn5aZ5Mny6asZUxYb1o-2GrcfBwZ8VCd2neuM6chMz1zSplJelp2Nraa9IWeN0p1Luw5JSMY7PmNaF)


# ELSD geometric shape detector
The ELSD (Ellipse and Line Segment detector) [Pătrăucean, et al. 2012] is a parameter-free combined line segment and elliptical arc detector. The detector obeys a 3-step scheme: candidate selection, candidate validation, and model selection. The latter two are formally sound, being grounded on statistical foundations. Only the candidate selection is heuristic, for efficiency reasons. ELSD algorithm is shown in the table below:

![](https://lh3.googleusercontent.com/WF0bkDpU8gNd9dTmjfE_PC-zHkBbjpLW3TZ1X4zZSEPjB4yDOOVWW9G0H2Ed-ShdzDBoYm72MsHYhtqALpk7tKvyMk2cVNAMB3H0o8fvAHdthWVrKcnhrxvAHTE4z2ItmmYP5ovY)



# Shape Filter

The output of an ELSD detector is a set of line segments and elliptical arcs. Considering the simple vision environment under water, it’s possible to design a shape filter based on some simple criteria:

+ The first criteria is that a buoy candidate must be an elliptical arc.
+ The second criteria is based on the relationship of start angle and end angle, which account for a descriptive segment of the buoy edge.
+ The third criteria is the size of elliptical arcs, which should not be too small.

Here below is an example result of the shape filter, where the reflection of buoy, as well as the small arcs are filtered.
![Fig. 1 Detection result of using the shape filter.](https://lh3.googleusercontent.com/XfhLmWglnLbhtz50v4tiaupTLsniHa4bO2LtREu2FIU6fx3Kt7xh2-UYHZAqJ7rRH53Za_Xd72cRZdER9a1teTPeawZrpgsgCgMpV0g9Y3NSQFgHFo48v9Wub26aYmCVb9Ss8svL)



# Detecting far-away buoy: the Color Space Locker

In the Robosub competition, the robot is required to recognize buoy at a certain distance. If the detector can finish detection task in very far away, our robot can set directions in advance, and thus increases the success possibility. Determining far away buoy is challenging due to the low visibility. However, such task can be accomplished by changing color space.

Initially, the buoy detector will use a RGB color space and set a locker to RGB color space. However, if the feedback from the shape filter indicates negative results, the Color Space Locker will change to HLS color space and set a locker on the color space. By using the Color Space Locker and the feedback, our detector can detect far away buoy.



# Results

**Processing time was evaluated using my pretty slow laptop Lenovo thinkpad T440. Hopefully you can get better results if you use more powerful machines, e.g., Jetson TX1.
![](https://lh3.googleusercontent.com/NNGMaK95QYuYbigz2xT6OK0PYSKr7oygjpJPxU_EWWFEpzudA1cCPh90iiOUaNKYlWZ8OLWPTZJQIpaLEnf8wveBsrf4cPMXQowplJgnc-mvcYN9_-K-VWm21sbscW6ucJuARL-A)

Detection result of far-away buoy.

![](https://lh6.googleusercontent.com/lefHeP4oEXpkHObrN1gB1ZlgGyL3uFJJhcjstr-9eYy1MrFsA599kVG_H4KkeJ_z0gs_GkZe0iT1JRDtSPb1emCmlCB-VrlJjg3j4Wh1i582qL9YtspsrPAhoWFu72vr2UaO8M5_)

Detection result of very far-away buoy.

![](https://lh4.googleusercontent.com/1DKT0plBaidN_vfDCH2ZdyqQA2j2eMb2Pcp1XCr9rPaUYqJnIaDpVNpQFdPGxdsyAAXej-6V79b-C40iZ0HBoGUGty2gV8VWI-TdJZu6hB-oIxLKUIi_hdCUgakHNmkSrnk-Gfpb)

## More results using competition samples.

![](https://lh4.googleusercontent.com/7PEkOMu2xdxU6jDB2kuKqs7hfkLE6hXHWBCdibKIRzuUjtt5tF2cHiqyUaNXM8PZNo95bIV2SkqMmWqtTHGVvawWV7L8xaEMVe2iyso5x1Kss0_1SFO5FdCPrN-47iWSxK0k7wRJ)
![](https://lh5.googleusercontent.com/xULPVT7sO-mXEYAgNPB64D9vQxQ2TBnG1aNpabfhiEFQirZ2I_yM1A8fLfIsFp9VBZONYT5P1DQ-Y84CpA9bYei2PPSUWFfyzU2MjTCcgwGUXcnDz04wIyloq-42lREdfAnFeGrI)

# Reference
Pătrăucean, Viorica, Pierre Gurdjos, and Rafael Grompone Von Gioi. "A parameterless line segment and elliptical arc detector with enhanced ellipse fitting." Computer Vision–ECCV 2012. Springer Berlin Heidelberg, 2012. 572-585.
