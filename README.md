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

A **<ins>Pygame-based Software</ins>** that  **<ins>live hazard detection</ins>** and **<ins>on-the-fly trajectory replanning</ins>** for a drone.
he aircraft begins on a simple **nominal route**. A **<ins>forward-looking sensor</ins>** continuously scans ahead; when **volcanic ash**, **moderate/high turbulence**, or **solid obstacles** enter the sensor footprint, the system invokes **<ins>Path-finding algorithms</ins>** and locks a new **safe trajectory**.

---

# 💠 **Key Capabilities :**


<table> <tr> <td valign="top" width="50%">

🢥 <ins>Path-finding algorithms</ins>

**Dijkstra :** Understand the traversable environment and calculate cost-to-go.
<br />
**A*** **:** Find an efficient global path from the drone's current position to its destination.
<br />
**BendyRuler :** Adjust the path locally when obstacles make the planned trajectory unsuitable.
<br />
</td> <td valign="top" width="50%">

🢥 <ins>Live sensing & adaptive replanning</ins>

Onboard sensors that can perform independently from ground station
</td> </tr> <tr> <td valign="top" width="50%">

・**Adaptable to multiple aircraft:** The ability to include multiple types of UAS ranging from hobbyists to defense grade equipment under our compatibility.
<br />

<br />
・**Ability to resolve multiple weather types:** The model has been trained to handle multiple types of contingency and disaster scenarios.
⠀
</td> <td valign="top" width="50%">

🢥 <ins>Avoidance rules</ins>

The system prioritizes threats based on the <b>severity of degradation</b> the system would face inside them. For example:

・Volcanic ash and high turbulence are <ins>hard exclusion volumes</ins>.
<br />・Low turbulence is a <ins>soft pass-through region</ins>.
<br />・TFRs are treated as <ins>solid barriers</ins> based on the altitude restrictions placed by regulators.
<br />
</td> </tr> </table>

> [!CAUTION]
> <ins>**Demo only:**</ins>
> <br />
> Path costs, energy figures and wind penalties are **illustrative demonstrator values**, **NOT flight-control limits**. Ash plumes that reach the top altitude layer are treated as **full vertical exclusion volumes** (the planner cannot “climb over” them). Low turbulence is intentionally pass-through; only moderate/high turbulence and ash force a re plan.

---

# ✅ **<ins>Requirements :</ins>**

<br />⠀⠀⠀・ Python 3.8
<br />⠀⠀⠀・ Pygame 2.6.1
<br />⠀⠀⠀・ Requests 2.32.0
<br />⠀⠀⠀・ Pillow 10.0.0

> [!Note]
> To use Map Mode install pillow using bash and have an internet connection while using it since it pulls OpenStreetMap API

> [!Important]
> For Installation refer [here](Installation.md)

---
