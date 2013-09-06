synchronization
===============

The most significant technical challenge is synchronizing the clock of
your computer with the luminance samples made by the USB device.

Sub-millisecond synchronization between your computer and USB device
--------------------------------------------------------------------

To allow your computer to determine exactly when samples were
acquired, a simple protocol involving timestamp exchange is
implemented between the host computer and the USB device. The host
sends a request to the arduino, keeping note of the time the request
was made (`T_host_request`). The lagtestino firmware responds to this
request by sending the current value of its clock (`C_arduino`). The
host records the time of arrival of this new data
(`T_host_receive`). By assuming that delays are symmetric and only
accepting responses that occur within a short latency (thus bounding
maximal error), the host calculates that the device's timetamp
occurred at `T_arduino_estimate`:

    request_response_roundtrip_duration = T_host_receive - T_host_request
    if request_response_roundtrip_duration < maximal_acceptable_error:
        T_arduino_estimate = (T_host_request + T_host_receive) / 2

So `T_arduino_estimate` is the best guess as to when `C_arduino` was
sampled, in the host computer's timebase. After many such samples, we
have two corresponding vectors:

    T = [ T_arduino_estimate[0], T_arduino_estimate[1], ... ]
    C = [ C_arduino[0],          C_arduino[1],          ... ]

We can now model the relation between the two clocks with:

    T = gain*C + offset

The values of `gain` and `offset` can be found by a least-squares fit
from the acquired vectors `T` and `C`. Doing so allows conversion from
any arduino clock value (`C_arduino`) to be directly converted to a
timestamp of the host computer.

This is concept is a simplification of the ideas contained in the IEEE
1588 specification but is sufficient for acheiving sub-millisecond
timing precision between an arduino and unpriviledged software.
