Brief demos summary:

rover: 
------
1. Flight side: cFS + a Python script  that gets data from cFS via UDP
2. Ground side: Edoras bridge + RQT GUI to send commands + Rviz to see telemetry back (using topics from Edoras bridge)
3. In summary: Bridge used only on the ground side.

gateway_single_arm:
-------------------
1. Flight side: cFS + Edoras bridge + a script that gets data from cFS via UDP
2. Ground side: Edoras bridge + a Python script that publishes gimbals to send a command to the flight side through the bridge.
3. In summary: Bridge used only on the ground side.

gateway_dual:
-------------
1. Flight side: cFS + Edoras bridge + a Python script that commands the 2 arms subscribing/publishing to the topics from Edoras
2. Ground side: Edoras bridge + a Python script that publishes gimbals to send a command to the flight side through the bridge.
3. In summary: Bridge used on both ground and flight side.
