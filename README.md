# Digital DC/DC control

This model is written in Octave.

Some preliminary documentation of the approach can be found [here](http://learn.libre.solar/b/dc-control/development/digital_control.html). 

Modeling and design of the DC/DC power stage in e.g. Libre Solar charge controllers for improved control in nanogrid and/or MPPT applications.

The script models the plant and divider circuitry using the state space method. From analysis of the zeros and poles, a second order approximation of the system is used (since the dominant response is second order). The second and third order responses are compared to show this is a valid approximation. 

Ziegler Nichols tuning techniques are applied using the rules here:  http://faculty.mercer.edu/jenkins_he/documents/TuningforPIDControllers.pdf

However, the output of this does not provide an improved step response. 

**Next steps** 

Calculate the PID parameters based on: The ultimate gain method OR the pole placement method.  

NOTE: There is a legacy issue (from the Matlab implementation) surrounding the use of the stepinfo function. A GNU Octave alternative has been proposed [here](https://lists.gnu.org/archive/html/help-octave/2015-02/msg00023.html), however this is not tested and verified. So the use of this for calculating the bandwidth of the system is omitted. 