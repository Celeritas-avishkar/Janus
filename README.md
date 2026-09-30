<div align="center">
<pre>
+------------------------------------------------------------------------+
|                                                                        |
|          █████   █████████   ██████   █████ █████  █████  █████████    |
|         ░░███   ███░░░░░███ ░░██████ ░░███ ░░███  ░░███  ███░░░░░███   |
|          ░███  ░███    ░███  ░███░███ ░███  ░███   ░███ ░███    ░░░    |
|          ░███  ░███████████  ░███░░███░███  ░███   ░███ ░░█████████    |
|          ░███  ░███░░░░░███  ░███ ░░██████  ░███   ░███  ░░░░░░░░███   |
|    ███   ░███  ░███    ░███  ░███  ░░█████  ░███   ░███  ███    ░███   |
|   ░░████████   █████   █████ █████  ░░█████ ░░████████  ░░█████████    |
|    ░░░░░░░░   ░░░░░   ░░░░░ ░░░░░    ░░░░░   ░░░░░░░░    ░░░░░░░░░     |
|                                                                        |
+------------------------------------------------------------------------+
|                    An Adaptive Flight-Path planner                     |
+------------------------------------------------------------------------+
</pre>
</div>

A **<ins>Pygame-based educational simulation</ins>** that demonstrates **<ins>live hazard detection</ins>** and **<ins>on-the-fly trajectory replanning</ins>** for a drone. The aircraft begins on a simple **nominal route**. A **<ins>forward-looking sensor</ins>** continuously scans ahead; when **volcanic ash**, **moderate/high turbulence**, or **solid obstacles** enter the sensor footprint, the system invokes **<ins>A\*</ins>** and locks a new **safe trajectory**.
